//! Action server implementation for ROS 2 actions.
//!
//! This module provides the server-side functionality for ROS 2 actions,
//! allowing nodes to accept goals from action clients, execute them,
//! provide feedback, and return results.

use std::{
    marker::PhantomData,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use tokio_util::sync::CancellationToken;
use zenoh::{Result, Wait};

use super::{
    GoalId, GoalInfo, GoalStatus, ZAction,
    messages::*,
    state::{SafeGoalManager, ServerGoalState},
};
use crate::{Builder, attachment::Attachment, msg::ZMessage, topic_name::qualify_topic_name};

/// Private implementation holding the actual server state.
/// This is wrapped by the public `ZActionServer` handle.
pub(crate) struct InnerServer<A: ZAction> {
    pub(crate) goal_server: Arc<crate::service::ZServer<GoalService<A>>>,
    pub(crate) result_server: Arc<crate::service::ZServer<ResultService<A>>>,
    pub(crate) cancel_server: Arc<crate::service::ZServer<CancelService<A>>>,
    pub(crate) feedback_pub:
        Arc<crate::pubsub::ZPub<FeedbackMessage<A>, <FeedbackMessage<A> as ZMessage>::Serdes>>,
    pub(crate) status_pub:
        Arc<crate::pubsub::ZPub<StatusMessage, <StatusMessage as ZMessage>::Serdes>>,
    pub(crate) goal_manager: Arc<SafeGoalManager<A>>,
    /// Token to cancel the default result handler when switching to full driver mode
    pub(crate) result_handler_token: CancellationToken,
}

/// Drop guard that triggers shutdown when the last server handle is dropped.
pub(crate) struct ShutdownGuard {
    pub(crate) token: CancellationToken,
}

impl Drop for ShutdownGuard {
    fn drop(&mut self) {
        tracing::debug!("ZActionServer handle dropped, triggering shutdown");
        self.token.cancel();
    }
}

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
    pub fn with_result_timeout(mut self, timeout: Duration) -> Self {
        self.result_timeout = timeout;
        self
    }

    pub fn with_goal_timeout(mut self, timeout: Duration) -> Self {
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
}

// Legacy result handler to preserve original behavior (using InnerServer)
async fn handle_result_requests_legacy_inner<A: ZAction>(
    inner: &InnerServer<A>,
    query: zenoh::query::Query,
) {
    tracing::debug!("Received result request");
    let payload = query.payload().unwrap().to_bytes();
    let request = match <GetResultRequest as ZMessage>::deserialize(&payload) {
        Ok(r) => r,
        Err(e) => {
            tracing::error!("Failed to deserialize result request: {}", e);
            return;
        }
    };

    // Look up goal result - extract data while holding lock, then release
    let result_data = inner.goal_manager.read(|manager| {
        if let Some(ServerGoalState::Terminated { result, status, .. }) =
            manager.goals.get(&request.goal_id)
        {
            Some((result.clone(), *status))
        } else {
            None
        }
    }); // Lock released here

    if let Some((result, status)) = result_data {
        tracing::debug!(
            "Goal {:?} is terminated with status {:?}",
            request.goal_id,
            status
        );

        // Send result response without holding lock
        let response = GetResultResponse::<A> {
            status: status as i8,
            result,
        };
        let response_bytes = <GetResultResponse<A> as ZMessage>::serialize(&response);
        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        //FIXME: address the result
        let _ = query
            .reply(query.key_expr().clone(), response_bytes)
            .attachment(attachment)
            .wait();
        tracing::debug!("Sent result response");
    } else {
        tracing::warn!("Goal {:?} not found or not terminated yet", request.goal_id);
        // Server doesn't reply if goal is not ready yet
    }
}

impl<'a, A: ZAction> Builder for ZActionServerBuilder<'a, A>
where
    
{
    type Output = ZActionServer<A>;

    fn build(self) -> Result<Self::Output> {
        // Apply remapping to action name
        let action_name = self.node.remap_rules.apply(&self.action_name);

        // Validate action name
        if action_name.is_empty() {
            return Err(zenoh::Error::from("Action name cannot be empty"));
        }

        // Qualify action name like a topic name
        let qualified_action_name = qualify_topic_name(
            &action_name,
            &self.node.entity.namespace,
            &self.node.entity.name,
        )?;

        tracing::debug!(
            "Action name: '{}', namespace: '{}', qualified: '{}'",
            action_name,
            self.node.entity.namespace,
            qualified_action_name
        );

        // ROS 2 action naming conventions
        let goal_service_name = format!("{}/_action/send_goal", qualified_action_name);
        let result_service_name = format!("{}/_action/get_result", qualified_action_name);
        let cancel_service_name = format!("{}/_action/cancel_goal", qualified_action_name);
        let feedback_topic_name = format!("{}/_action/feedback", qualified_action_name);
        let status_topic_name = format!("{}/_action/status", qualified_action_name);

        // Create goal server using node API for proper graph registration
        // Use the action's send_goal_type_info for proper ROS 2 interop
        let goal_type_info = Some(A::send_goal_type_info());
        let mut goal_server_builder = self
            .node
            .create_service_impl::<GoalService<A>>(&goal_service_name, goal_type_info);
        if let Some(qos) = self.goal_service_qos {
            goal_server_builder.entity.qos = qos.to_protocol_qos();
        }
        let goal_server = goal_server_builder.build()?;

        // Create result server using node API for proper graph registration
        // Use the action's get_result_type_info for proper ROS 2 interop
        let result_type_info = Some(A::get_result_type_info());
        let mut result_server_builder = self
            .node
            .create_service_impl::<ResultService<A>>(&result_service_name, result_type_info);
        if let Some(qos) = self.result_service_qos {
            result_server_builder.entity.qos = qos.to_protocol_qos();
        }
        let result_server = result_server_builder.build()?;
        tracing::debug!("Created result server for: {}", result_service_name);

        // Create cancel server using node API for proper graph registration
        // Use the action's cancel_goal_type_info for proper ROS 2 interop
        let cancel_type_info = Some(A::cancel_goal_type_info());
        let mut cancel_server_builder = self
            .node
            .create_service_impl::<CancelService<A>>(&cancel_service_name, cancel_type_info);
        if let Some(qos) = self.cancel_service_qos {
            cancel_server_builder.entity.qos = qos.to_protocol_qos();
        }
        let cancel_server = cancel_server_builder.build()?;

        // Create feedback publisher using node API for proper graph registration
        // Use the action's feedback_type_info for proper ROS 2 interop
        let feedback_type_info = Some(A::feedback_type_info());
        let mut feedback_pub_builder = self
            .node
            .create_pub_impl::<FeedbackMessage<A>>(&feedback_topic_name, feedback_type_info);
        if let Some(qos) = self.feedback_topic_qos {
            feedback_pub_builder.entity.qos = qos.to_protocol_qos();
        }
        // Keep attachments enabled for RMW-Zenoh compatibility
        let feedback_pub = feedback_pub_builder.build()?;

        // Create status publisher using node API for proper graph registration
        // Use the action's status_type_info for proper ROS 2 interop
        let status_type_info = Some(A::status_type_info());
        let mut status_pub_builder = self
            .node
            .create_pub_impl::<StatusMessage>(&status_topic_name, status_type_info);
        if let Some(qos) = self.status_topic_qos {
            status_pub_builder.entity.qos = qos.to_protocol_qos();
        }
        // Keep attachments enabled for RMW-Zenoh compatibility
        let status_pub = status_pub_builder.build()?;

        let goal_manager = Arc::new(SafeGoalManager::new(self.result_timeout, self.goal_timeout));

        let cancellation_token = CancellationToken::new();
        let result_handler_token = CancellationToken::new();

        // Create the inner server
        let inner = Arc::new(InnerServer {
            goal_server: Arc::new(goal_server),
            result_server: Arc::new(result_server),
            cancel_server: Arc::new(cancel_server),
            feedback_pub: Arc::new(feedback_pub),
            status_pub: Arc::new(status_pub),
            goal_manager,
            result_handler_token: result_handler_token.clone(),
        });

        // Spawn background task to handle result requests (default mode for manual goal handling)
        // This task will be cancelled if with_handler() is called
        let weak_inner = Arc::downgrade(&inner);
        let global_shutdown = cancellation_token.clone();
        let handler_token = result_handler_token.clone();

        tokio::spawn(async move {
            // Run until EITHER global shutdown OR handler-specific cancellation
            tokio::select! {
                _ = global_shutdown.cancelled() => {
                    tracing::debug!("Result handler stopping due to global shutdown");
                },
                _ = handler_token.cancelled() => {
                    tracing::debug!("Result handler stopping - switching to full driver mode");
                },
                _ = async {
                    while let Some(inner) = weak_inner.upgrade() {
                        let query = inner.result_server.queue().recv_async().await;
                        handle_result_requests_legacy_inner(&inner, query).await;
                    }
                } => {},
            }
        });

        Ok(ZActionServer {
            inner,
            _shutdown: Arc::new(ShutdownGuard {
                token: cancellation_token,
            }),
        })
    }
}

/// Action server handle using the Handle Pattern.
///
/// This is a lightweight, cloneable handle that wraps the actual server implementation.
/// When all handles are dropped, the server automatically shuts down.
pub struct ZActionServer<A: ZAction> {
    inner: Arc<InnerServer<A>>,
    /// Drop guard that triggers shutdown when the last handle is dropped
    _shutdown: Arc<ShutdownGuard>,
}

impl<A: ZAction> Clone for ZActionServer<A> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            _shutdown: self._shutdown.clone(),
        }
    }
}

// Internal helper for driver to create server handles and access inner fields
impl<A: ZAction> ZActionServer<A> {
    pub(crate) fn from_inner(inner: Arc<InnerServer<A>>) -> Self {
        // Create a dummy shutdown guard that doesn't do anything
        // The driver doesn't control the server lifetime
        let dummy_token = CancellationToken::new();
        Self {
            inner,
            _shutdown: Arc::new(ShutdownGuard { token: dummy_token }),
        }
    }
}

// Provide convenient access to inner fields via getter methods
impl<A: ZAction> ZActionServer<A> {
    fn goal_server(&self) -> &Arc<crate::service::ZServer<GoalService<A>>> {
        &self.inner.goal_server
    }

    fn result_server(&self) -> &Arc<crate::service::ZServer<ResultService<A>>> {
        &self.inner.result_server
    }

    fn cancel_server(&self) -> &Arc<crate::service::ZServer<CancelService<A>>> {
        &self.inner.cancel_server
    }

    fn feedback_pub(
        &self,
    ) -> &Arc<crate::pubsub::ZPub<FeedbackMessage<A>, <FeedbackMessage<A> as ZMessage>::Serdes>>
    {
        &self.inner.feedback_pub
    }

    fn status_pub(
        &self,
    ) -> &Arc<crate::pubsub::ZPub<StatusMessage, <StatusMessage as ZMessage>::Serdes>> {
        &self.inner.status_pub
    }

    /// Access the goal manager for advanced use cases and testing.
    ///
    /// # Warning
    ///
    /// This is a low-level API that gives direct access to the goal state.
    /// Use with caution as it bypasses the normal goal handle abstractions.
    pub fn goal_manager(&self) -> &Arc<SafeGoalManager<A>> {
        &self.inner.goal_manager
    }

    fn result_handler_token(&self) -> &CancellationToken {
        &self.inner.result_handler_token
    }
}

impl<A: ZAction> ZActionServer<A> {
    fn publish_status(&self) {
        // Build status list while holding lock, then release before publishing
        let status_list: Vec<GoalStatusInfo> = self.goal_manager().read(|manager| {
            manager
                .goals
                .iter()
                .map(|(goal_id, state)| {
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
                })
                .collect()
        }); // Lock released here

        // Publish without holding lock
        let msg = StatusMessage { status_list };
        // FIXME: address the result
        let _ = self.status_pub().publish(&msg);
    }

    pub async fn recv_goal(&self) -> Result<GoalHandle<A, Requested>> {
        let query = self.goal_server().queue().recv_async().await;
        let payload = query.payload().unwrap().to_bytes();
        let request = <SendGoalRequest<A> as ZMessage>::deserialize(&payload)
            .map_err(|e| zenoh::Error::from(e.to_string()))?;

        Ok(GoalHandle {
            goal: request.goal,
            info: GoalInfo::new(request.goal_id),
            server: self.clone(),
            query: Some(query),
            cancel_flag: None,
            _state: PhantomData,
        })
    }

    pub async fn recv_cancel(&self) -> Result<(CancelGoalServiceRequest, zenoh::query::Query)> {
        let query = self.cancel_server().queue().recv_async().await;
        let payload = query.payload().unwrap().to_bytes();
        let request = <CancelGoalServiceRequest as ZMessage>::deserialize(&payload)
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok((request, query))
    }

    pub fn is_cancel_request_ready(&self) -> bool {
        !self.cancel_server().queue().is_empty()
    }

    /// Marks a goal as canceling by setting its atomic cancel flag.
    /// This is a lock-free operation that can be called from any thread.
    pub fn request_cancel(&self, goal_id: GoalId) -> bool {
        self.goal_manager().read(|manager| {
            if let Some(ServerGoalState::Executing { cancel_flag, .. }) =
                manager.goals.get(&goal_id)
            {
                cancel_flag.store(true, Ordering::Relaxed);
                true
            } else {
                false
            }
        })
    }

    pub async fn recv_result_request(&self) -> Result<(GoalId, zenoh::query::Query)> {
        let query = self.result_server().queue().recv_async().await;
        let payload = query.payload().unwrap().to_bytes();
        let request = <ResultRequest as ZMessage>::deserialize(&payload)
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok((request.goal_id, query))
    }

    // FIXME: check the necessity
    pub fn send_goal_response_low(
        &self,
        query: &zenoh::query::Query,
        response: &GoalResponse,
    ) -> Result<()> {
        let response_bytes = <GoalResponse as ZMessage>::serialize(response);
        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let _ = query
            .reply(query.key_expr().clone(), response_bytes)
            .attachment(attachment)
            .wait();
        Ok(())
    }

    // FIXME: check the necessity
    pub async fn recv_cancel_request_low(
        &self,
    ) -> Result<(CancelGoalServiceRequest, zenoh::query::Query)> {
        let query = self.cancel_server().queue().recv_async().await;
        let payload = query.payload().unwrap().to_bytes();
        let request = <CancelGoalServiceRequest as ZMessage>::deserialize(&payload)
            .map_err(|e| zenoh::Error::from(e.to_string()))?;
        Ok((request, query))
    }

    pub fn send_cancel_response_low(
        &self,
        query: &zenoh::query::Query,
        response: &CancelGoalServiceResponse,
    ) -> Result<()> {
        let response_bytes = <CancelGoalServiceResponse as ZMessage>::serialize(response);
        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let _ = query
            .reply(query.key_expr().clone(), response_bytes)
            .attachment(attachment)
            .wait();
        Ok(())
    }

    // FIXME: check the necessity
    pub fn send_result_response_low(
        &self,
        query: &zenoh::query::Query,
        response: &GetResultResponse<A>,
    ) -> Result<()> {
        let response_bytes = <GetResultResponse<A> as ZMessage>::serialize(response);
        let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
        let _ = query
            .reply(query.key_expr().clone(), response_bytes)
            .attachment(attachment)
            .wait();
        Ok(())
    }

    /// Attaches an automatic goal handler to the server.
    ///
    /// This method transitions the server from "manual mode" (where you call `recv_goal()`)
    /// to "automatic mode" (where goals are handled by the provided callback).
    ///
    /// **Important**: This method cancels the default result-only handler and starts a full
    /// driver loop that handles all protocol events (goals, cancels, results) automatically.
    ///
    /// # Arguments
    ///
    /// * `handler` - Callback function that will be invoked for each accepted goal
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use ros_z::action::*;
    /// # let server = todo!();
    /// let server = server.with_handler(|executing| async move {
    ///     // Process the goal
    ///     executing.succeed(result).unwrap();
    /// });
    /// ```
    pub fn with_handler<F, Fut>(self, handler: F) -> Self
    where
        F: Fn(GoalHandle<A, Executing>) -> Fut + Send + Sync + 'static,
        Fut: std::future::Future<Output = ()> + Send + 'static,
    {
        // 1. Stop the default result-only handler to avoid competing for result_server.rx
        tracing::debug!("Cancelling default result handler to switch to full driver mode");
        self.result_handler_token().cancel();

        // 2. Start the full driver loop that handles all protocol events
        let weak_inner = Arc::downgrade(&self.inner);
        let shutdown_token = self._shutdown.token.clone();
        tokio::spawn(async move {
            crate::action::driver::run_driver_loop(weak_inner, shutdown_token, handler).await;
        });

        self
    }

    /// Expires goals that have passed their expiration time.
    ///
    /// This method checks all goals with `expires_at` timestamps and removes:
    /// - Accepted/Executing goals that have timed out (goal timeout)
    /// - Terminated goals whose results have expired (result timeout)
    ///
    /// Goals without expiration times (when timeouts are not configured) are never expired.
    ///
    /// # Returns
    ///
    /// Returns a vector of `GoalId`s that were expired.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use ros_z::action::*;
    /// # let server: ros_z::action::server::ZActionServer<MyAction> = todo!();
    /// // Check and expire any goals that have passed their expiration time
    /// let expired = server.expire_goals();
    /// println!("Expired {} goals", expired.len());
    /// ```
    pub fn expire_goals(&self) -> Vec<GoalId> {
        let expired = self.goal_manager().modify(|manager| {
            let now = Instant::now();
            let mut expired = Vec::new();

            // Find goals that have passed their expiration time
            manager.goals.retain(|goal_id, state| {
                let should_expire = match state {
                    ServerGoalState::Accepted { expires_at, .. }
                    | ServerGoalState::Executing { expires_at, .. }
                    | ServerGoalState::Terminated { expires_at, .. } => {
                        expires_at.is_some_and(|exp| now >= exp)
                    }
                    ServerGoalState::Canceling { .. } => false,
                };

                if should_expire {
                    expired.push(*goal_id);
                    false // Remove this goal
                } else {
                    true // Keep this goal
                }
            });

            expired
        }); // Lock released here

        // Publish updated status if any goals were expired
        if !expired.is_empty() {
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
        self.goal_manager().modify(|manager| {
            manager.result_timeout = timeout;
        });
    }

    /// Gets the current result timeout for this server.
    ///
    /// # Returns
    ///
    /// The result timeout duration
    pub fn result_timeout(&self) -> Duration {
        self.goal_manager().read(|manager| manager.result_timeout)
    }
}

// --- State Markers for Type-State Pattern ---
/// Marker type representing a goal that has been requested but not yet accepted or rejected.
pub struct Requested;

/// Marker type representing a goal that has been accepted but not yet executing.
pub struct Accepted;

/// Marker type representing a goal that is currently executing.
pub struct Executing;

// Type aliases for convenience
/// A goal handle in the "Requested" state.
pub type RequestedGoal<A> = GoalHandle<A, Requested>;

/// A goal handle in the "Accepted" state.
pub type AcceptedGoal<A> = GoalHandle<A, Accepted>;

/// A goal handle in the "Executing" state.
pub type ExecutingGoal<A> = GoalHandle<A, Executing>;

// Type-state pattern for goal lifecycle with PhantomData markers
/// A type-safe goal handle that uses compile-time state tracking.
///
/// The `GoalHandle` is generic over the action type `A` and the state `State`.
/// Different methods are available depending on the current state, enforced at compile time.
///
/// # Type States
///
/// - `GoalHandle<A, Requested>`: Can be accepted or rejected
/// - `GoalHandle<A, Accepted>`: Can be executed
/// - `GoalHandle<A, Executing>`: Can publish feedback and be terminated
///
/// # Examples
///
/// ```no_run
/// # use ros_z::action::*;
/// # let server: std::sync::Arc<server::ZActionServer<MyAction>> = todo!();
/// # async {
/// let requested = server.recv_goal().await?;
/// let accepted = requested.accept();
/// let executing = accepted.execute();
/// executing.succeed(result)?;
/// # Ok::<(), zenoh::Error>(())
/// # };
/// ```
pub struct GoalHandle<A: ZAction, State> {
    /// The goal data.
    pub goal: A::Goal,
    /// The goal metadata.
    pub info: GoalInfo,
    pub(crate) server: ZActionServer<A>,
    pub(crate) query: Option<zenoh::query::Query>,
    pub(crate) cancel_flag: Option<Arc<AtomicBool>>,
    pub(crate) _state: PhantomData<State>,
}

// --- State-specific implementations ---

/// Methods available only for goals in the "Requested" state.
impl<A: ZAction> GoalHandle<A, Requested> {
    /// Access the goal data.
    pub fn goal(&self) -> &A::Goal {
        &self.goal
    }

    /// Access the goal info.
    pub fn info(&self) -> &GoalInfo {
        &self.info
    }

    /// Accept this goal and transition to the "Accepted" state.
    ///
    /// This sends an acceptance response to the client and updates the server state.
    pub fn accept(mut self) -> GoalHandle<A, Accepted> {
        // Send acceptance response
        // Use timestamp from GoalInfo which is already in sec/nanosec format
        let response = SendGoalResponse {
            accepted: true,
            stamp_sec: self.info.stamp.sec,
            stamp_nanosec: self.info.stamp.nanosec,
        };
        let response_bytes = <SendGoalResponse as ZMessage>::serialize(&response);

        if let Some(query) = self.query.take() {
            let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
            // FIXME: address the result
            let _ = query
                .reply(query.key_expr().clone(), response_bytes)
                .attachment(attachment)
                .wait();
        }

        // Update server state to ACCEPTED
        self.server.goal_manager().modify(|manager| {
            let expires_at = manager.goal_timeout.map(|timeout| Instant::now() + timeout);
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Accepted {
                    goal: self.goal.clone(),
                    timestamp: Instant::now(),
                    expires_at,
                },
            );
        });

        // Publish status update
        self.server.publish_status();

        GoalHandle {
            goal: self.goal,
            info: self.info,
            server: self.server,
            query: None,
            cancel_flag: None,
            _state: PhantomData,
        }
    }

    /// Reject this goal.
    ///
    /// This sends a rejection response to the client. The goal will not be executed.
    pub fn reject(mut self) -> Result<()> {
        // Send rejection response
        let response = GoalResponse {
            accepted: false,
            stamp_sec: 0,
            stamp_nanosec: 0,
        };
        let response_bytes = <GoalResponse as ZMessage>::serialize(&response);

        if let Some(query) = self.query.take() {
            // FIXME: Address the unwrap usage
            let attachment: Attachment = query.attachment().unwrap().try_into().unwrap();
            let _ = query
                .reply(query.key_expr().clone(), response_bytes)
                .attachment(attachment)
                .wait();
        }
        Ok(())
    }
}

/// Methods available only for goals in the "Accepted" state.
impl<A: ZAction> GoalHandle<A, Accepted> {
    /// Access the goal data.
    pub fn goal(&self) -> &A::Goal {
        &self.goal
    }

    /// Access the goal info.
    pub fn info(&self) -> &GoalInfo {
        &self.info
    }

    /// Begin executing this goal and transition to the "Executing" state.
    ///
    /// This updates the server state to executing and publishes a status update.
    pub fn execute(self) -> GoalHandle<A, Executing> {
        // Create cancel flag
        let cancel_flag = Arc::new(AtomicBool::new(false));

        // Transition to EXECUTING
        self.server.goal_manager().modify(|manager| {
            let expires_at = manager.goal_timeout.map(|timeout| Instant::now() + timeout);
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Executing {
                    goal: self.goal.clone(),
                    cancel_flag: cancel_flag.clone(),
                    expires_at,
                },
            );
        });

        self.server.publish_status();

        GoalHandle {
            goal: self.goal,
            info: self.info,
            server: self.server,
            query: None,
            cancel_flag: Some(cancel_flag),
            _state: PhantomData,
        }
    }
}

/// Methods available only for goals in the "Executing" state.
impl<A: ZAction> GoalHandle<A, Executing> {
    /// Access the goal data.
    pub fn goal(&self) -> &A::Goal {
        &self.goal
    }

    /// Access the goal info.
    pub fn info(&self) -> &GoalInfo {
        &self.info
    }

    /// Publish feedback for this goal.
    ///
    /// Feedback can be published multiple times during goal execution to inform
    /// the client of progress.
    pub fn publish_feedback(&self, feedback: A::Feedback) -> Result<()> {
        let msg = FeedbackMessage {
            goal_id: self.info.goal_id,
            feedback,
        };
        self.server.feedback_pub().publish(&msg)
    }

    /// Check if cancellation has been requested for this goal.
    ///
    /// This is a lock-free operation that can be called frequently from the
    /// goal execution loop.
    ///
    /// # Returns
    ///
    /// `true` if a cancel request has been received, `false` otherwise.
    pub fn is_cancel_requested(&self) -> bool {
        self.cancel_flag
            .as_ref()
            .map(|flag| flag.load(Ordering::Relaxed))
            .unwrap_or(false)
    }

    /// Mark this goal as succeeded with the given result.
    ///
    /// This transitions the goal to a terminal state and consumes the handle.
    pub fn succeed(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Succeeded)
    }

    /// Mark this goal as aborted with the given result.
    ///
    /// This transitions the goal to a terminal state and consumes the handle.
    pub fn abort(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Aborted)
    }

    /// Mark this goal as canceled with the given result.
    ///
    /// This transitions the goal to a terminal state and consumes the handle.
    pub fn canceled(self, result: A::Result) -> Result<()> {
        self.terminate(result, GoalStatus::Canceled)
    }

    fn terminate(self, result: A::Result, status: GoalStatus) -> Result<()> {
        // Notify any waiting result futures
        let futures_to_notify = self.server.goal_manager().modify(|manager| {
            let now = Instant::now();
            let expires_at = Some(now + manager.result_timeout);
            manager.goals.insert(
                self.info.goal_id,
                ServerGoalState::Terminated {
                    result: result.clone(),
                    status,
                    timestamp: now,
                    expires_at,
                },
            );

            // Take all waiting result futures for this goal
            manager
                .result_futures
                .remove(&self.info.goal_id)
                .unwrap_or_default()
        }); // Drop the lock before notifying futures and publishing status

        // Notify all waiting result futures
        for tx in futures_to_notify {
            let _ = tx.send((result.clone(), status));
        }

        self.server.publish_status();
        Ok(())
    }
}
