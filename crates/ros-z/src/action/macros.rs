/// Macro to define a simple action type without ROS2 interop.
///
/// This macro is a simplified version of `impl_action!` for custom actions that don't need
/// ROS2 type hashes and interop. It only implements the basic `ZAction` trait methods.
///
/// # Syntax
///
/// ```ignore
/// define_action! {
///     ActionStruct,
///     action_name: "action_name",
///     Goal: GoalType,
///     Result: ResultType,
///     Feedback: FeedbackType,
/// }
/// ```
///
/// # Example
///
/// ```ignore
/// use ros_z::define_action;
/// use serde::{Deserialize, Serialize};
///
/// #[derive(Debug, Clone, Serialize, Deserialize)]
/// pub struct NavigateToPoseGoal {
///     pub target_x: f64,
///     pub target_y: f64,
/// }
///
/// #[derive(Debug, Clone, Serialize, Deserialize)]
/// pub struct NavigateToPoseResult {
///     pub success: bool,
/// }
///
/// #[derive(Debug, Clone, Serialize, Deserialize)]
/// pub struct NavigateToPoseFeedback {
///     pub current_x: f64,
///     pub current_y: f64,
/// }
///
/// pub struct NavigateToPose;
///
/// define_action! {
///     NavigateToPose,
///     action_name: "navigate_to_pose",
///     Goal: NavigateToPoseGoal,
///     Result: NavigateToPoseResult,
///     Feedback: NavigateToPoseFeedback,
/// }
/// ```
#[macro_export]
macro_rules! define_action {
    (
        $action_struct:ident,
        action_name: $action_name:expr,
        Goal: $goal_type:ty,
        Result: $result_type:ty,
        Feedback: $feedback_type:ty $(,)?
    ) => {
        impl $crate::action::ZAction for $action_struct {
            type Goal = $goal_type;
            type Result = $result_type;
            type Feedback = $feedback_type;

            fn name() -> &'static str {
                $action_name
            }
        }
    };
}
