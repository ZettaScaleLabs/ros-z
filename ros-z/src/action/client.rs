use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::{mpsc, oneshot, watch};

use zenoh::{Result, Session, sample::Sample};

use crate::entity::{EndpointEntity, EntityKind};
use crate::msg::{CdrSerdes, ZDeserializer};
use crate::{Builder};

use super::ZAction;

use super::messages::*;
use super::{GoalId, GoalInfo, GoalStatus};

pub struct ZActionClientBuilder<'a, A: ZAction> {
    pub action_name: String,
    pub node: &'a crate::node::ZNode,
    pub goal_service_qos: Option<crate::qos::QosProfile>,
    pub result_service_qos: Option<crate::qos::QosProfile>,
    pub cancel_service_qos: Option<crate::qos::QosProfile>,
    pub feedback_topic_qos: Option<crate::qos::QosProfile>,
    pub status_topic_qos: Option<crate::qos::QosProfile>,
    pub _phantom: std::marker::PhantomData<A>,
}

impl<'a, A: ZAction> ZActionClientBuilder<'a, A> {
    pub fn new(action_name: &str, node: &'a crate::node::ZNode) -> Self {
        Self {
            action_name: action_name.to_string(),
            node,
            goal_service_qos: None,
            result_service_qos: None,
            cancel_service_qos: None,
            feedback_topic_qos: None,
            status_topic_qos: None,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn with_goal_service_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.goal_service_qos = Some(qos);
        self
    }

    pub fn with_result_service_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.result_service_qos = Some(qos);
        self
    }

    pub fn with_cancel_service_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.cancel_service_qos = Some(qos);
        self
    }

    pub fn with_feedback_topic_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.feedback_topic_qos = Some(qos);
        self
    }

    pub fn with_status_topic_qos(mut self, qos: crate::qos::QosProfile) -> Self {
        self.status_topic_qos = Some(qos);
        self
    }
}

impl<'a, A: ZAction> Builder for ZActionClientBuilder<'a, A> {
    type Output = ZActionClient<A>;

    fn build(self) -> Result<Self::Output> {
        // Apply remapping to action name
        let action_name = self.node.remap_rules.apply(&self.action_name);

        // ROS 2 action naming conventions
        let goal_service_name = format!("{}/_action/send_goal", action_name);
        let result_service_name = format!("{}/_action/get_result", action_name);
        let cancel_service_name = format!("{}/_action/cancel_goal", action_name);
        let feedback_topic_name = format!("{}/_action/feedback", action_name);
        let status_topic_name = format!("{}/_action/status", action_name);

        // Create goal client
        let goal_entity = EndpointEntity {
            id: 0,
            node: self.node.entity.clone(),
            kind: EntityKind::Client,
            topic: goal_service_name,
            type_info: None,
            qos: self.goal_service_qos.unwrap_or_default(),
        };
        let goal_client = crate::service::ZClientBuilder::<GoalService<A>> {
            entity: goal_entity,
            session: self.node.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create result client
        let result_entity = EndpointEntity {
            id: 0,
            node: self.node.entity.clone(),
            kind: EntityKind::Client,
            topic: result_service_name,
            type_info: None,
            qos: self.result_service_qos.unwrap_or_default(),
        };
        let result_client = crate::service::ZClientBuilder::<ResultService<A>> {
            entity: result_entity,
            session: self.node.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create cancel client
        let cancel_entity = EndpointEntity {
            id: 0,
            node: self.node.entity.clone(),
            kind: EntityKind::Client,
            topic: cancel_service_name,
            type_info: None,
            qos: self.cancel_service_qos.unwrap_or_default(),
        };
        let cancel_client = crate::service::ZClientBuilder::<CancelService> {
            entity: cancel_entity,
            session: self.node.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create feedback subscriber
        let feedback_entity = EndpointEntity {
            id: 0,
            node: self.node.entity.clone(),
            kind: EntityKind::Subscription,
            topic: feedback_topic_name,
            type_info: None,
            qos: self.feedback_topic_qos.unwrap_or_default(),
        };
        let feedback_sub = crate::pubsub::ZSubBuilder::<FeedbackMessage<A>> {
            entity: feedback_entity,
            session: self.node.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        // Create status subscriber
        let status_entity = EndpointEntity {
            id: 0,
            node: self.node.entity.clone(),
            kind: EntityKind::Subscription,
            topic: status_topic_name,
            type_info: None,
            qos: self.status_topic_qos.unwrap_or_default(),
        };
        let status_sub = crate::pubsub::ZSubBuilder::<StatusMessage> {
            entity: status_entity,
            session: self.node.session.clone(),
            _phantom_data: std::marker::PhantomData,
        }.build()?;

        let goal_board = Arc::new(Mutex::new(GoalBoard {
            active_goals: HashMap::new(),
            pending_goals: HashMap::new(),
        }));

        // Spawn background task for feedback routing
        let goal_board_clone = goal_board.clone();
        let feedback_sub_arc = Arc::new(feedback_sub);
        let feedback_sub_task = feedback_sub_arc.clone();
        tokio::spawn(async move {
            while let Ok(sample) = feedback_sub_task.queue.recv_async().await {
                let payload = sample.payload().to_bytes();
                let msg = crate::msg::CdrSerdes::<FeedbackMessage<A>>::deserialize(&payload);
                // msg is FeedbackMessage<A>, use it directly
                if let Some(channels) = goal_board_clone.lock().unwrap().active_goals.get(&msg.goal_id) {
                    let _ = channels.feedback_tx.send(msg.feedback);
                }
            }
        });

        // Spawn background task for status routing
        let goal_board_clone2 = goal_board.clone();
        let status_sub_arc = Arc::new(status_sub);
        let status_sub_task = status_sub_arc.clone();
        tokio::spawn(async move {
            while let Ok(sample) = status_sub_task.queue.recv_async().await {
                let payload = sample.payload().to_bytes();
                let msg = crate::msg::CdrSerdes::<StatusMessage>::deserialize(&payload);
                // msg is StatusMessage, use it directly
                for status_info in msg.status_list {
                    if let Some(channels) = goal_board_clone2.lock().unwrap().active_goals.get(&status_info.goal_info.goal_id) {
                        let _ = channels.status_tx.send(status_info.status);
                    }
                }
            }
        });

        Ok(ZActionClient {
            goal_client: Arc::new(goal_client),
            result_client: Arc::new(result_client),
            cancel_client: Arc::new(cancel_client),
            feedback_sub: feedback_sub_arc,
            status_sub: status_sub_arc,
            goal_board,
        })
    }
}

pub struct ZActionClient<A: ZAction> {
    goal_client: Arc<crate::service::ZClient<GoalService<A>>>,
    result_client: Arc<crate::service::ZClient<ResultService<A>>>,
    cancel_client: Arc<crate::service::ZClient<CancelService>>,
    feedback_sub: Arc<crate::pubsub::ZSub<FeedbackMessage<A>, Sample, CdrSerdes<FeedbackMessage<A>>>>,
    status_sub: Arc<crate::pubsub::ZSub<StatusMessage, Sample, CdrSerdes<StatusMessage>>>,
    goal_board: Arc<Mutex<GoalBoard<A>>>,
}

impl<A: ZAction> Clone for ZActionClient<A> {
    fn clone(&self) -> Self {
        Self {
            goal_client: self.goal_client.clone(),
            result_client: self.result_client.clone(),
            cancel_client: self.cancel_client.clone(),
            feedback_sub: self.feedback_sub.clone(),
            status_sub: self.status_sub.clone(),
            goal_board: self.goal_board.clone(),
        }
    }
}

impl<A: ZAction> ZActionClient<A> {
    pub async fn send_goal(&self, goal: A::Goal) -> Result<GoalHandle<A>> {
        let goal_id = GoalId::new();

        // 1. Create channels for this goal
        let (feedback_tx, feedback_rx) = mpsc::unbounded_channel();
        let (status_tx, status_rx) = watch::channel(GoalStatus::Unknown);
        let (result_tx, result_rx) = oneshot::channel();

        // 2. Store channels in goal board
        {
            let mut board = self.goal_board.lock().unwrap();
            board.active_goals.insert(goal_id, GoalChannels {
                feedback_tx,
                status_tx,
                result_tx: Some(result_tx),
            });
        }

        // 3. Send goal request via service client
        let request = GoalRequest { goal_id, goal };
        self.goal_client.send_request(&request).await?;

        // 4. Wait for response
        let sample = self.goal_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        let response = crate::msg::CdrSerdes::<GoalResponse>::deserialize(&payload);

        // 5. Check if accepted
        if !response.accepted {
            // Clean up channels
            self.goal_board.lock().unwrap().active_goals.remove(&goal_id);
            return Err(zenoh::Error::from("Goal rejected".to_string()));
        }

        // 6. Return handle
        Ok(GoalHandle {
            id: goal_id,
            client: Arc::new(self.clone()),
            feedback_rx: Some(feedback_rx),
            status_rx: Some(status_rx),
            result_rx: Some(result_rx),
        })
    }

    pub async fn cancel_goal(&self, goal_id: GoalId) -> Result<CancelGoalResponse> {
        let goal_info = GoalInfo::new(goal_id);
        let request = CancelGoalRequest { goal_info };

        self.cancel_client.send_request(&request).await?;
        let sample = self.cancel_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        let response = crate::msg::CdrSerdes::<CancelGoalResponse>::deserialize(&payload);
        Ok(response)
    }

    pub async fn cancel_all_goals(&self) -> Result<CancelGoalResponse> {
        // ROS 2 convention: zero UUID + zero timestamp means "cancel all"
        let zero_goal_id = GoalId([0u8; 16]);
        let goal_info = GoalInfo { goal_id: zero_goal_id, stamp: 0 };
        let request = CancelGoalRequest { goal_info };

        self.cancel_client.send_request(&request).await?;
        let sample = self.cancel_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        let response = crate::msg::CdrSerdes::<CancelGoalResponse>::deserialize(&payload);
        Ok(response)
    }

    pub fn feedback_stream(&self, goal_id: GoalId) -> Option<mpsc::UnboundedReceiver<A::Feedback>> {
        let mut board = self.goal_board.lock().unwrap();
        board.active_goals.get_mut(&goal_id).map(|channels| {
            // Create new receiver (old one already taken via GoalHandle)
            let (tx, rx) = mpsc::unbounded_channel();
            channels.feedback_tx = tx;
            rx
        })
    }

    pub fn status_watch(&self, goal_id: GoalId) -> Option<watch::Receiver<GoalStatus>> {
        let board = self.goal_board.lock().unwrap();
        board.active_goals.get(&goal_id).map(|channels| {
            channels.status_tx.subscribe()
        })
    }

    pub async fn get_result(&self, goal_id: GoalId) -> Result<A::Result> {
        let request = ResultRequest { goal_id };

        self.result_client.send_request(&request).await?;
        let sample = self.result_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        let response = crate::msg::CdrSerdes::<ResultResponse<A>>::deserialize(&payload);

        Ok(response.result)
    }

    // Low-level methods for testing
    pub async fn send_goal_request_low(&self, request: &super::messages::GoalRequest<A>) -> Result<()> {
        self.goal_client.send_request(request).await
    }

    pub async fn recv_goal_response_low(&self) -> Result<super::messages::GoalResponse> {
        let sample = self.goal_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        Ok(crate::msg::CdrSerdes::<super::messages::GoalResponse>::deserialize(&payload))
    }

    pub async fn send_cancel_request_low(&self, request: &super::messages::CancelGoalRequest) -> Result<()> {
        self.cancel_client.send_request(request).await
    }

    pub async fn recv_cancel_response_low(&self) -> Result<super::messages::CancelGoalResponse> {
        let sample = self.cancel_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        Ok(crate::msg::CdrSerdes::<super::messages::CancelGoalResponse>::deserialize(&payload))
    }

    pub async fn send_result_request_low(&self, request: &super::messages::ResultRequest) -> Result<()> {
        self.result_client.send_request(request).await
    }

    pub async fn recv_result_response_low(&self) -> Result<super::messages::ResultResponse<A>> {
        let sample = self.result_client.rx.recv_async().await?;
        let payload = sample.payload().to_bytes();
        Ok(crate::msg::CdrSerdes::<super::messages::ResultResponse<A>>::deserialize(&payload))
    }
}

#[allow(dead_code)]
struct GoalBoard<A: ZAction> {
    active_goals: HashMap<GoalId, GoalChannels<A>>,
    pending_goals: HashMap<i64, oneshot::Sender<GoalResponse>>,
}

#[allow(dead_code)]
struct GoalChannels<A: ZAction> {
    feedback_tx: mpsc::UnboundedSender<A::Feedback>,
    status_tx: watch::Sender<GoalStatus>,
    result_tx: Option<oneshot::Sender<A::Result>>,
}

pub struct GoalHandle<A: ZAction> {
    id: GoalId,
    client: Arc<ZActionClient<A>>,
    feedback_rx: Option<mpsc::UnboundedReceiver<A::Feedback>>,
    status_rx: Option<watch::Receiver<GoalStatus>>,
    result_rx: Option<oneshot::Receiver<A::Result>>,
}

impl<A: ZAction> GoalHandle<A> {
    pub fn id(&self) -> GoalId {
        self.id
    }

    pub fn feedback_stream(&mut self) -> Option<mpsc::UnboundedReceiver<A::Feedback>> {
        self.feedback_rx.take()
    }

    pub fn status_watch(&mut self) -> Option<watch::Receiver<GoalStatus>> {
        self.status_rx.take()
    }

    pub async fn result(&mut self) -> Result<A::Result> {
        // Request the result from the server
        self.client.get_result(self.id).await
    }

    pub async fn cancel(&self) -> Result<CancelGoalResponse> {
        self.client.cancel_goal(self.id).await
    }
}