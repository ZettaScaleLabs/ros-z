use std::time::{Duration, Instant};

use ros_z::action::{GoalEvent, GoalId, GoalInfo, GoalStatus, transition_goal_state};

// Simple performance measurement functions
fn measure_goal_id_generation(iterations: usize) -> Duration {
    let start = Instant::now();
    for _ in 0..iterations {
        let _id = GoalId::new();
    }
    start.elapsed()
}

fn measure_goal_id_validation(iterations: usize) -> Duration {
    let ids: Vec<GoalId> = (0..iterations).map(|_| GoalId::new()).collect();
    let start = Instant::now();
    for id in &ids {
        let _valid = id.is_valid();
    }
    start.elapsed()
}

fn measure_goal_info_creation(iterations: usize) -> Duration {
    let start = Instant::now();
    for _ in 0..iterations {
        let id = GoalId::new();
        let _info = GoalInfo::new(id);
    }
    start.elapsed()
}

fn measure_state_machine_transitions(iterations: usize) -> Duration {
    let transitions = vec![
        (GoalStatus::Accepted, GoalEvent::Execute),
        (GoalStatus::Executing, GoalEvent::Succeed),
        (GoalStatus::Executing, GoalEvent::CancelGoal),
        (GoalStatus::Canceling, GoalEvent::Canceled),
    ];

    let start = Instant::now();
    for _ in 0..iterations / transitions.len() {
        for (status, event) in &transitions {
            let _new_status = transition_goal_state(*status, *event);
        }
    }
    start.elapsed()
}

fn measure_message_serialization(iterations: usize) -> Duration {
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    struct TestMessage {
        pub order: i32,
        pub data: Vec<u8>,
    }

    let message = TestMessage {
        order: 42,
        data: vec![1, 2, 3, 4, 5],
    };

    let start = Instant::now();
    for _ in 0..iterations {
        let _json = serde_json::to_string(&message).unwrap();
    }
    start.elapsed()
}

fn measure_message_deserialization(iterations: usize) -> Duration {
    let json = r#"{"order":42,"data":[1,2,3,4,5]}"#;

    let start = Instant::now();
    for _ in 0..iterations {
        let _message: serde_json::Value = serde_json::from_str(json).unwrap();
    }
    start.elapsed()
}

fn main() {
    let iterations = 100_000;

    // Run performance measurements silently for CI compatibility
    let _ = measure_goal_id_generation(iterations);
    let _ = measure_goal_id_validation(iterations);
    let _ = measure_goal_info_creation(iterations);
    let _ = measure_state_machine_transitions(iterations);
    let _ = measure_message_serialization(iterations);
    let _ = measure_message_deserialization(iterations);

    // Benchmarks complete - output suppressed for automated testing
}
