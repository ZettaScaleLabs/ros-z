//! Action server implementation for ROS 2 actions.
//!
//! This module provides the server-side functionality for ROS 2 actions,
//! allowing nodes to accept goals from action clients, execute them,
//! provide feedback, and return results.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio_util::sync::CancellationToken;
use zenoh::{Result, Wait};

use crate::msg::ZMessage;
use crate::{Builder};

use super::ZAction;
use super::messages::*;
use super::{GoalId, GoalInfo, GoalStatus};

/// Builder for creating an action server.
///
/// The `ZActionServerBuilder` allows you to configure timeouts and QoS settings
/// for different action communication channels before building the server.
///
/// # Examples
///
/// ```no_run
/// # use ros_z::action::*;
/// # use std::time::Duration;
/// # let node = todo!();
/// let server = node.create_action_server::<MyAction>("my_action")
///     .with_result_timeout(Duration::from_secs(30))
///     .build()?;
/// # Ok::<(), zenoh::Error>(())
/// ```
pub struct ZActionServerBuilder<'a, A: ZAction> {
    /// The name of the action.
    pub action_name: String,
    /// Reference to the node that will own this server.
    pub node: &'a crate::node::ZNode,
    /// Timeout for result requests.
    pub result_timeout: Duration,
    /// Optional timeout for goal execution.
    pub goal_timeout: Option<Duration>,
    /// QoS profile for the goal service.
    pub goal_service_qos: Option<crate::qos::QosProfile>,
    /// QoS profile for the result service.
    pub result_service_qos: Option<crate::qos::QosProfile>,
    /// QoS profile for the cancel service.
    pub cancel_service_qos: Option<crate::qos::QosProfile>,
    /// QoS profile for the feedback topic.
    pub feedback_topic_qos: Option<crate::qos::QosProfile>,
    /// QoS profile for the status topic.
    pub status_topic_qos: Option<crate::qos::QosProfile>,
    pub _phantom: std::marker::PhantomData<A>,
}

impl<'a, A: ZAction> ZActionServerBuilder<'a, A> {
    pub fn new(action_name: &str, node: &'a crate::node::ZNode) -> Self {
        Self {
            action_name: action_name.to_string(),
            node,
            result_timeout: Duration::from_secs(10),
            goal_timeout: None,
            goal_service_qos: None,
            result_service_qos: None,
            cancel_service_qos: None,
            feedback_topic_qos: None,
            status_topic_qos: None,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn result_timeout(mut self, timeout: Duration) -> Self {
        self.result_timeout = timeout;
        self
    }

    pub fn goal_timeout(mut self, timeout: Duration) -> Self {
        self.goal_timeout = Some(timeout);
        self
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

// Helper function to handle result requests
async fn handle_result_requests<A: ZAction>(
    result_server: Arc<crate::service::ZServer<ResultService<A>>>,
    goal_manager: Arc<Mutex<GoalManager<A>>>,
) {
    loop {
        if let Ok(query) = result_server.rx.recv_async().await {
            let payload = query.payload().unwrap().to_bytes();
            let request = <ResultRequest as ZMessage>::deserialize(&payload);

            // Look up goal result
            let manager = goal_manager.lock().unwrap();
            if let Some(ServerGoalState::Terminated { result, status, .. }) = manager.goals.get(&request.goal_id) {
                let result_clone = result.clone();
                let status_clone = *status;
                drop(manager);

                // Send result response
                let response = ResultResponse::<A> {
                    status: status_clone,
                    result: result_clone,
                };
                let response_bytes = <ResultResponse<A> as ZMessage>::serialize(&response);
                let _ = query.reply(query.key_expr().clone(), response_bytes).wait();
            }
        }
    }
}

impl<'a, A: ZAction> Builder for ZActionServerBuilder<'a, A> {
    type Output = Arc<ZActionServer<A>>;

    fn build(self) -> Result<Self::Output> {
        // Apply remapping to action name
        let action_name = self.node.remap_rules.apply(&self.action_name);

        // ROS 2 action naming conventions
        let goal_service_name = format!("{}/_action/send_goal", action_name);
        let result_service_name = format!("{}/_action/get_result", action_name);
        let cancel_service_name = format!("{}/_action/cancel_goal", action_name);
        let feedback_topic_name = format!("{}/_action/feedback", action_name);
        let status_topic_name = format!("{}/_action/status", action_name);

        // Create goal server using node API for proper graph registration
        let mut goal_server_builder = self.node.create_service_impl::<GoalService<A>>(&goal_service_name, None);
        if let Some(qos) = self.goal_service_qos {
            goal_server_builder.entity.qos = qos;
        }
        let goal_server = goal_server_builder.build()?;

        // Create result server using node API for proper graph registration
        let mut result_server_builder = self.node.create_service_impl::<ResultService<A>>(&result_service_name, None);
        if let Some(qos) = self.result_service_qos {
            result_server_builder.entity.qos = qos;
        }
        let result_server = result_server_builder.build()?;

        // Create cancel server using node API for proper graph registration
        let mut cancel_server_builder = self.node.create_service_impl::<CancelService>(&cancel_service_name, None);
        if let Some(qos) = self.cancel_service_qos {
            cancel_server_builder.entity.qos = qos;
        }
        let cancel_server = cancel_server_builder.build()?;

        // Create feedback publisher using node API for proper graph registration
        let mut feedback_pub_builder = self.node.create_pub_impl::<FeedbackMessage<A>>(&feedback_topic_name, None);
        if let Some(qos) = self.feedback_topic_qos {
            feedback_pub_builder.entity.qos = qos;
        }
        feedback_pub_builder.with_attachment = false;
        let feedback_pub = feedback_pub_builder.build()?;

        // Create status publisher using node API for proper graph registration
        let mut status_pub_builder = self.node.create_pub_impl::<StatusMessage>(&status_topic_name, None);
        if let Some(qos) = self.status_topic_qos {
            status_pub_builder.entity.qos = qos;
        }
        status_pub_builder.with_attachment = false;
        let status_pub = status_pub_builder.build()?;

        let goal_manager = Arc::new(Mutex::new(GoalManager {
            goals: HashMap::new(),
            result_timeout: self.result_timeout,
            goal_timeout: self.goal_timeout,
        }));

        let cancellation_token = CancellationToken::new();

        // Spawn background task to handle result requests
        let result_server_arc = Arc::new(result_server);
        let result_server_clone = result_server_arc.clone();
        let goal_manager_clone = goal_manager.clone();
        let token_clone = cancellation_token.clone();
        tokio::spawn(async move {
            tokio::select! {
                _ = token_clone.cancelled() => {},
                _ = handle_result_requests::<A>(result_server_clone, goal_manager_clone) => {},
            }
        });

        // TODO: Add background task for goal expiration checking

        Ok(Arc::new(ZActionServer {
            goal_server: Arc::new(goal_server),
            result_server: result_server_arc,
            cancel_server: Arc::new(cancel_server),
            feedback_pub: Arc::new(feedback_pub),
            status_pub: Arc::new(status_pub),
            goal_manager,
            _cancellation_token: cancellation_token,
        }))
    }
}

pub struct ZActionServer<A: ZAction> {
    goal_server: Arc<crate::service::ZServer<GoalService<A>>>,
    result_server: Arc<crate::service::ZServer<ResultService<A>>>,
    cancel_server: Arc<crate::service::ZServer<CancelService>>,
    feedback_pub: Arc<crate::pubsub::ZPub<FeedbackMessage<A>, <FeedbackMessage<A> as ZMessage>::Serdes>>,
    status_pub: Arc<crate::pubsub::ZPub<StatusMessage, <StatusMessage as ZMessage>::Serdes>>,
    goal_manager: Arc<Mutex<GoalManager<A>>>,
    _cancellation_token: CancellationToken,
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
            _cancellation_token: self._cancellation_token.clone(),
        }
    }
}

impl<A: ZAction> Drop for ZActionServer<A> {
    fn drop(&mut self) {
        self._cancellation_token.cancel();
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
        let request = <GoalRequest<A> as ZMessage>::deserialize(&payload);

        Ok(RequestedGoal {
            goal: request.goal,
            info: GoalInfo::new(request.goal_id),
            server: Arc::new(self.clone()),
            query,
        })
    }

    pub async fn recv_cancel(&self) -> Result<(CancelGoalRequest, zenoh::query::Query)> {
        let query = self.cancel_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request = <CancelGoalRequest as ZMessage>::deserialize(&payload);
        Ok((request, query))
    }

    pub fn is_cancel_request_ready(&self) -> bool {
        !self.cancel_server.rx.is_empty()
    }

    pub async fn recv_result_request(&self) -> Result<(GoalId, zenoh::query::Query)> {
        let query = self.result_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request = <ResultRequest as ZMessage>::deserialize(&payload);
        Ok((request.goal_id, query))
    }

    // Low-level methods for testing
    pub async fn recv_goal_request_low(&self) -> Result<(super::messages::GoalRequest<A>, zenoh::query::Query)> {
        let query = self.goal_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request = <super::messages::GoalRequest<A> as ZMessage>::deserialize(&payload);
        Ok((request, query))
    }

    pub fn send_goal_response_low(&self, query: &zenoh::query::Query, response: &super::messages::GoalResponse) -> Result<()> {
        let response_bytes = <super::messages::GoalResponse as ZMessage>::serialize(response);
        let _ = query.reply(query.key_expr().clone(), response_bytes).wait();
        Ok(())
    }

    pub async fn recv_cancel_request_low(&self) -> Result<(super::messages::CancelGoalRequest, zenoh::query::Query)> {
        let query = self.cancel_server.rx.recv_async().await?;
        let payload = query.payload().unwrap().to_bytes();
        let request = <super::messages::CancelGoalRequest as ZMessage>::deserialize(&payload);
        Ok((request, query))
    }

    pub fn send_cancel_response_low(&self, query: &zenoh::query::Query, response: &super::messages::CancelGoalResponse) -> Result<()> {
        let response_bytes = <super::messages::CancelGoalResponse as ZMessage>::serialize(response);
        let _ = query.reply(query.key_expr().clone(), response_bytes).wait();
        Ok(())
    }

    pub fn send_result_response_low(&self, query: &zenoh::query::Query, response: &super::messages::ResultResponse<A>) -> Result<()> {
        let response_bytes = <super::messages::ResultResponse<A> as ZMessage>::serialize(response);
        let _ = query.reply(query.key_expr().clone(), response_bytes).wait();
        Ok(())
    }

    pub fn with_handler<F, Fut>(self: Arc<Self>, handler: F) -> Arc<Self>
    where
        F: Fn(ExecutingGoal<A>) -> Fut + Send + Sync + 'static,
        Fut: std::future::Future<Output = ()> + Send + 'static,
    {
        let server_clone = self.clone();
        tokio::spawn(async move {
            loop {
                if let Ok(requested) = server_clone.recv_goal().await {
                    let accepted = requested.accept();
                    let executing = accepted.execute();
                    handler(executing).await;
                }
            }
        });
        self
    }

    /// Expires terminated goals that are older than the specified maximum age.
    ///
    /// This method removes terminated goals (succeeded, aborted, or canceled)
    /// from the goal manager if they have been in a terminal state longer
    /// than `max_age`. This prevents memory leaks in long-running servers.
    ///
    /// # Arguments
    ///
    /// * `max_age` - The maximum age for terminated goals before expiration
    ///
    /// # Returns
    ///
    /// Returns a vector of `GoalId`s that were expired.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use ros_z::action::*;
    /// # use std::time::Duration;
    /// # let server: ros_z::action::server::ZActionServer<MyAction> = todo!();
    /// // Expire goals older than 10 seconds
    /// let expired = server.expire_goals(Duration::from_secs(10));
    /// println!("Expired {} goals", expired.len());
    /// ```
    pub fn expire_goals(&self, max_age: Duration) -> Vec<GoalId> {
        let mut manager = self.goal_manager.lock().unwrap();
        let now = Instant::now();
        let mut expired = Vec::new();

        // Find terminated goals older than max_age
        manager.goals.retain(|goal_id, state| {
            if let ServerGoalState::Terminated { timestamp, .. } = state
                && now.duration_since(*timestamp) > max_age {
                    expired.push(*goal_id);
                    return false; // Remove this goal
                }
            true // Keep this goal
        });

        // Publish updated status if any goals were expired
        if !expired.is_empty() {
            drop(manager); // Release lock before publishing
            self.publish_status();
        }

        expired
    }

    /// Sets the result timeout for this server.
    ///
    /// This configures how long the server will keep terminated goals
    /// before they can be expired. Note: This does not automatically
    /// expire goals - you must call `expire_goals()` periodically.
    ///
    /// # Arguments
    ///
    /// * `timeout` - The result timeout duration
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use ros_z::action::*;
    /// # use std::time::Duration;
    /// # let mut server: ros_z::action::server::ZActionServer<MyAction> = todo!();
    /// server.set_result_timeout(Duration::from_secs(30));
    /// ```
    pub fn set_result_timeout(&self, timeout: Duration) {
        let mut manager = self.goal_manager.lock().unwrap();
        manager.result_timeout = timeout;
    }

    /// Gets the current result timeout for this server.
    ///
    /// # Returns
    ///
    /// The result timeout duration
    pub fn result_timeout(&self) -> Duration {
        let manager = self.goal_manager.lock().unwrap();
        manager.result_timeout
    }
}

struct GoalManager<A: ZAction> {
    goals: HashMap<GoalId, ServerGoalState<A>>,
    result_timeout: Duration,
    goal_timeout: Option<Duration>,
}

// TODO: check this dead_code
#[allow(dead_code)]
enum ServerGoalState<A: ZAction> {
    Accepted { goal: A::Goal, timestamp: Instant, expires_at: Option<Instant> },
    Executing { goal: A::Goal, cancel_requested: bool, expires_at: Option<Instant> },
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
        let response_bytes = <GoalResponse as ZMessage>::serialize(&response);
        let _ = self.query.reply(self.query.key_expr().clone(), response_bytes).wait();

        // Update server state to ACCEPTED
        {
            let mut manager = self.server.goal_manager.lock().unwrap();
            let expires_at = manager.goal_timeout.map(|timeout| Instant::now() + timeout);
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Accepted {
                    goal: self.goal.clone(),
                    timestamp: Instant::now(),
                    expires_at,
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
        let response_bytes = <GoalResponse as ZMessage>::serialize(&response);
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
            let expires_at = manager.goal_timeout.map(|timeout| Instant::now() + timeout);
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Executing {
                    goal: self.goal.clone(),
                    cancel_requested: false,
                    expires_at,
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
        {
            let mut manager = self.server.goal_manager.lock().unwrap();
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Terminated {
                    result,
                    status,
                    timestamp: Instant::now(),
                },
            );
        } // Drop the lock before publishing status
        self.server.publish_status();
        Ok(())
    }
}
