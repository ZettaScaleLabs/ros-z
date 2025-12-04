use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use zenoh::{Result, Session, Wait};

use crate::entity::{EndpointEntity, EntityKind};
use crate::msg::{ZDeserializer, ZSerializer};
use crate::{Builder};

use super::ZAction;
use super::messages::*;
use super::{GoalId, GoalInfo, GoalStatus};

pub struct ZActionServerBuilder<A: ZAction> {
    pub action_name: String,
    pub node: crate::entity::NodeEntity,
    pub session: Arc<Session>,
    pub result_timeout: Duration,
    pub _phantom: std::marker::PhantomData<A>,
}

impl<A: ZAction> ZActionServerBuilder<A> {
    pub fn new(action_name: &str, node: crate::entity::NodeEntity, session: Arc<Session>) -> Self {
        Self {
            action_name: action_name.to_string(),
            node,
            session,
            result_timeout: Duration::from_secs(10),
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn result_timeout(mut self, timeout: Duration) -> Self {
        self.result_timeout = timeout;
        self
    }
}

impl<A: ZAction> Builder for ZActionServerBuilder<A> {
    type Output = Arc<ZActionServer<A>>;

    fn build(self) -> Result<Self::Output> {
        // ROS 2 action naming conventions
        let goal_service_name = format!("{}/_action/send_goal", self.action_name);
        let result_service_name = format!("{}/_action/get_result", self.action_name);
        let cancel_service_name = format!("{}/_action/cancel_goal", self.action_name);
        let feedback_topic_name = format!("{}/_action/feedback", self.action_name);
        let status_topic_name = format!("{}/_action/status", self.action_name);

        // Create goal server
        let goal_entity = EndpointEntity {
            id: 0,
            node: self.node.clone(),
            kind: EntityKind::Service,
            topic: goal_service_name,
            type_info: None,
            qos: Default::default(),
        };
        let goal_server = crate::service::ZServerBuilder::<GoalService<A>> {
            entity: goal_entity,
            session: self.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create result server
        let result_entity = EndpointEntity {
            id: 0,
            node: self.node.clone(),
            kind: EntityKind::Service,
            topic: result_service_name,
            type_info: None,
            qos: Default::default(),
        };
        let result_server = crate::service::ZServerBuilder::<ResultService<A>> {
            entity: result_entity,
            session: self.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create cancel server
        let cancel_entity = EndpointEntity {
            id: 0,
            node: self.node.clone(),
            kind: EntityKind::Service,
            topic: cancel_service_name,
            type_info: None,
            qos: Default::default(),
        };
        let cancel_server = crate::service::ZServerBuilder::<CancelService> {
            entity: cancel_entity,
            session: self.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create feedback publisher
        let feedback_entity = EndpointEntity {
            id: 0,
            node: self.node.clone(),
            kind: EntityKind::Publisher,
            topic: feedback_topic_name,
            type_info: None,
            qos: Default::default(),
        };
        let feedback_pub = crate::pubsub::ZPubBuilder::<FeedbackMessage<A>> {
            entity: feedback_entity,
            session: self.session.clone(),
            with_attachment: false,
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create status publisher
        let status_entity = EndpointEntity {
            id: 0,
            node: self.node.clone(),
            kind: EntityKind::Publisher,
            topic: status_topic_name,
            type_info: None,
            qos: Default::default(),
        };
        let status_pub = crate::pubsub::ZPubBuilder::<StatusMessage> {
            entity: status_entity,
            session: self.session.clone(),
            with_attachment: false,
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        let goal_manager = Arc::new(Mutex::new(GoalManager {
            goals: HashMap::new(),
            result_timeout: self.result_timeout,
        }));

        Ok(Arc::new(ZActionServer {
            goal_server: Arc::new(goal_server),
            result_server: Arc::new(result_server),
            cancel_server: Arc::new(cancel_server),
            feedback_pub: Arc::new(feedback_pub),
            status_pub: Arc::new(status_pub),
            goal_manager,
        }))
    }
}

pub struct ZActionServer<A: ZAction> {
    goal_server: Arc<crate::service::ZServer<GoalService<A>>>,
    result_server: Arc<crate::service::ZServer<ResultService<A>>>,
    cancel_server: Arc<crate::service::ZServer<CancelService>>,
    feedback_pub: Arc<crate::pubsub::ZPub<FeedbackMessage<A>, crate::msg::CdrSerdes<FeedbackMessage<A>>>>,
    status_pub: Arc<crate::pubsub::ZPub<StatusMessage, crate::msg::CdrSerdes<StatusMessage>>>,
    goal_manager: Arc<Mutex<GoalManager<A>>>,
}

impl<A: ZAction> Clone for ZActionServer<A> {
    fn clone(&self) -> Self {
        Self {
            goal_server: self.goal_server.clone(),
            result_server: self.result_server.clone(),
            cancel_server: self.cancel_server.clone(),
            feedback_pub: self.feedback_pub.clone(),
            status_pub: self.status_pub.clone(),
            goal_manager: self.goal_manager.clone(),
        }
    }
}

impl<A: ZAction> ZActionServer<A> {
    fn publish_status(&self) {
        let manager = self.goal_manager.lock().unwrap();
        let status_list: Vec<GoalStatusInfo> = manager.goals.iter().map(|(goal_id, state)| {
            let status = match state {
                ServerGoalState::Accepted { .. } => GoalStatus::Accepted,
                ServerGoalState::Executing { .. } => GoalStatus::Executing,
                ServerGoalState::Canceling { .. } => GoalStatus::Canceling,
                ServerGoalState::Terminated { status, .. } => *status,
            };
            GoalStatusInfo {
                goal_info: GoalInfo::new(*goal_id),
                status,
            }
        }).collect();

        let msg = StatusMessage { status_list };
        let _ = self.status_pub.publish(&msg);
    }

    pub async fn recv_goal(&self) -> Result<RequestedGoal<A>> {
        let query = self.goal_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request: GoalRequest<A> = crate::msg::CdrSerdes::<GoalRequest<A>>::deserialize(&payload);

        Ok(RequestedGoal {
            goal: request.goal,
            info: GoalInfo::new(request.goal_id),
            server: Arc::new(self.clone()),  // Need Clone derive for ZActionServer!
            query,
        })
    }

    pub async fn recv_cancel(&self) -> Result<(CancelGoalRequest, zenoh::query::Query)> {
        let query = self.cancel_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request: CancelGoalRequest = crate::msg::CdrSerdes::<CancelGoalRequest>::deserialize(&payload);
        Ok((request, query))
    }

    pub async fn recv_result_request(&self) -> Result<(GoalId, zenoh::query::Query)> {
        let query = self.result_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request: ResultRequest = crate::msg::CdrSerdes::<ResultRequest>::deserialize(&payload);
        Ok((request.goal_id, query))
    }

    pub fn with_handler<F, Fut>(self, handler: F) -> Arc<Self>
    where
        F: Fn(ExecutingGoal<A>) -> Fut + Send + Sync + 'static,
        Fut: std::future::Future<Output = ()> + Send + 'static,
    {
        let server = Arc::new(self);
        let server_clone = server.clone();
        tokio::spawn(async move {
            loop {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    handler(executing).await;
                }
            }
        });
        server
    }
}

#[allow(dead_code)]
struct GoalManager<A: ZAction> {
    goals: HashMap<GoalId, ServerGoalState<A>>,
    result_timeout: Duration,
}

#[allow(dead_code)]
enum ServerGoalState<A: ZAction> {
    Accepted { goal: A::Goal, timestamp: Instant },
    Executing { goal: A::Goal, cancel_requested: bool },
    Canceling { goal: A::Goal },
    Terminated { result: A::Result, status: GoalStatus, timestamp: Instant },
}

// Type-state pattern for goal lifecycle
pub struct RequestedGoal<A: ZAction> {
    pub goal: A::Goal,
    pub info: GoalInfo,
    server: Arc<ZActionServer<A>>,
    query: zenoh::query::Query,
}

impl<A: ZAction> RequestedGoal<A> {
    pub fn accept(self) -> AcceptedGoal<A> {
        // Send acceptance response
        let response = GoalResponse { accepted: true, stamp: self.info.stamp };
        let response_bytes = crate::msg::CdrSerdes::<GoalResponse>::serialize(&response);
        let _ = self.query.reply(self.query.key_expr().clone(), response_bytes).wait();

        // Update server state to ACCEPTED
        {
            let mut manager = self.server.goal_manager.lock().unwrap();
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Accepted {
                    goal: self.goal.clone(),
                    timestamp: Instant::now(),
                },
            );
        }

        // Publish status update
        self.server.publish_status();

        AcceptedGoal {
            goal: self.goal,
            info: self.info,
            server: self.server,
        }
    }

    pub fn reject(self) -> Result<()> {
        // Send rejection response
        let response = GoalResponse { accepted: false, stamp: 0 };
        let response_bytes = crate::msg::CdrSerdes::<GoalResponse>::serialize(&response);
        let _ = self.query.reply(self.query.key_expr().clone(), response_bytes).wait();
        Ok(())
    }
}

pub struct AcceptedGoal<A: ZAction> {
    pub goal: A::Goal,
    pub info: GoalInfo,
    server: Arc<ZActionServer<A>>,
}

impl<A: ZAction> AcceptedGoal<A> {
    pub fn execute(self) -> ExecutingGoal<A> {
        // Transition to EXECUTING
        {
            let mut manager = self.server.goal_manager.lock().unwrap();
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Executing {
                    goal: self.goal.clone(),
                    cancel_requested: false,
                },
            );
        }

        self.server.publish_status();

        ExecutingGoal {
            goal: self.goal,
            info: self.info,
            server: self.server,
        }
    }
}

pub struct ExecutingGoal<A: ZAction> {
    pub goal: A::Goal,
    pub info: GoalInfo,
    server: Arc<ZActionServer<A>>,
}

impl<A: ZAction> ExecutingGoal<A> {
    pub fn publish_feedback(&self, feedback: A::Feedback) -> Result<()> {
        let msg = FeedbackMessage {
            goal_id: self.info.goal_id,
            feedback,
        };
        self.server.feedback_pub.publish(&msg)
    }

    pub fn is_cancel_requested(&self) -> bool {
        let manager = self.server.goal_manager.lock().unwrap();
        if let Some(ServerGoalState::Executing { cancel_requested, .. }) =
            manager.goals.get(&self.info.goal_id)
        {
            *cancel_requested
        } else {
            false
        }
    }

    pub fn succeed(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Succeeded)
    }

    pub fn abort(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Aborted)
    }

    pub fn canceled(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Canceled)
    }

    fn terminate(self, result: A::Result, status: GoalStatus) -> Result<()> {
        let mut manager = self.server.goal_manager.lock().unwrap();
        manager.goals.insert(
            self.info.goal_id,
            ServerGoalState::Terminated {
                result,
                status,
                timestamp: Instant::now(),
            },
        );
        self.server.publish_status();
        Ok(())
    }
}