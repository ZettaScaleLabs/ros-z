#[cfg(test)]
mod tests {
    use crate::action::{GoalStatus, GoalEvent, transition_goal_state};

    #[test]
    fn test_valid_transitions() {
        // From ACCEPTED
        assert_eq!(transition_goal_state(GoalStatus::Accepted, GoalEvent::Execute), GoalStatus::Executing);
        assert_eq!(transition_goal_state(GoalStatus::Accepted, GoalEvent::CancelGoal), GoalStatus::Canceling);

        // From EXECUTING
        assert_eq!(transition_goal_state(GoalStatus::Executing, GoalEvent::CancelGoal), GoalStatus::Canceling);
        assert_eq!(transition_goal_state(GoalStatus::Executing, GoalEvent::Succeed), GoalStatus::Succeeded);
        assert_eq!(transition_goal_state(GoalStatus::Executing, GoalEvent::Abort), GoalStatus::Aborted);

        // From CANCELING
        assert_eq!(transition_goal_state(GoalStatus::Canceling, GoalEvent::Canceled), GoalStatus::Canceled);
        assert_eq!(transition_goal_state(GoalStatus::Canceling, GoalEvent::Succeed), GoalStatus::Succeeded);
        assert_eq!(transition_goal_state(GoalStatus::Canceling, GoalEvent::Abort), GoalStatus::Aborted);
    }

    #[test]
    fn test_invalid_transitions() {
        // Invalid from ACCEPTED
        assert_eq!(transition_goal_state(GoalStatus::Accepted, GoalEvent::Succeed), GoalStatus::Unknown);
        assert_eq!(transition_goal_state(GoalStatus::Accepted, GoalEvent::Abort), GoalStatus::Unknown);
        assert_eq!(transition_goal_state(GoalStatus::Accepted, GoalEvent::Canceled), GoalStatus::Unknown);

        // Invalid from EXECUTING
        assert_eq!(transition_goal_state(GoalStatus::Executing, GoalEvent::Execute), GoalStatus::Unknown);
        assert_eq!(transition_goal_state(GoalStatus::Executing, GoalEvent::Canceled), GoalStatus::Unknown);

        // Invalid from terminal states
        assert_eq!(transition_goal_state(GoalStatus::Succeeded, GoalEvent::Execute), GoalStatus::Unknown);
        assert_eq!(transition_goal_state(GoalStatus::Canceled, GoalEvent::CancelGoal), GoalStatus::Unknown);
        assert_eq!(transition_goal_state(GoalStatus::Aborted, GoalEvent::Succeed), GoalStatus::Unknown);
    }

    #[test]
    fn test_status_queries() {
        // is_active
        assert!(GoalStatus::Accepted.is_active());
        assert!(GoalStatus::Executing.is_active());
        assert!(GoalStatus::Canceling.is_active());
        assert!(!GoalStatus::Succeeded.is_active());
        assert!(!GoalStatus::Canceled.is_active());
        assert!(!GoalStatus::Aborted.is_active());

        // is_terminal
        assert!(!GoalStatus::Accepted.is_terminal());
        assert!(!GoalStatus::Executing.is_terminal());
        assert!(!GoalStatus::Canceling.is_terminal());
        assert!(GoalStatus::Succeeded.is_terminal());
        assert!(GoalStatus::Canceled.is_terminal());
        assert!(GoalStatus::Aborted.is_terminal());
    }
}