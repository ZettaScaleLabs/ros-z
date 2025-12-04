#[cfg(test)]
mod tests {
    use crate::action::{GoalId, GoalInfo, GoalStatus, messages};

    #[test]
    fn test_goal_info_initialization() {
        let goal_id = GoalId::new();
        let goal_info = GoalInfo::new(goal_id.clone());
        assert_eq!(goal_info.goal_id, goal_id);
        assert!(goal_info.stamp > 0); // stamp should be set to current time
    }

    #[test]
    fn test_goal_status_array_initialization() {
        // Test that GoalStatus values are correct
        assert_eq!(GoalStatus::Unknown as i8, 0);
        assert_eq!(GoalStatus::Accepted as i8, 1);
        assert_eq!(GoalStatus::Executing as i8, 2);
        assert_eq!(GoalStatus::Canceling as i8, 3);
        assert_eq!(GoalStatus::Succeeded as i8, 4);
        assert_eq!(GoalStatus::Canceled as i8, 5);
        assert_eq!(GoalStatus::Aborted as i8, 6);
    }

    #[test]
    fn test_cancel_request_response_initialization() {
        let goal_id = GoalId::new();
        let goal_info = GoalInfo::new(goal_id);
        let cancel_request = messages::CancelGoalRequest {
            goal_info: goal_info.clone(),
        };
        assert_eq!(cancel_request.goal_info.goal_id, goal_info.goal_id);

        let cancel_response = messages::CancelGoalResponse {
            return_code: 0,
            goals_canceling: vec![goal_info],
        };
        assert_eq!(cancel_response.return_code, 0);
        assert_eq!(cancel_response.goals_canceling.len(), 1);
    }
}