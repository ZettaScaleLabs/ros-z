use ros_z::{Builder, Result, action::ZAction, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

// Define test action messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestGoal {
    pub order: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    pub value: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestFeedback {
    pub progress: i32,
}

pub struct TestAction;

impl ZAction for TestAction {
    type Goal = TestGoal;
    type Result = TestResult;
    type Feedback = TestFeedback;

    fn name() -> &'static str {
        "test_action"
    }
}

async fn setup_test_base() -> Result<ros_z::node::ZNode> {
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("test_action_client_node").build()?;
    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    Ok(node)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[ignore]
    async fn test_action_client_init_fini() -> Result<()> {
        let node = setup_test_base().await?;

        // Test successful initialization with valid arguments
        let client = node
            .create_action_client::<TestAction>("/test_action_client_name")
            .build()?;
        drop(client); // fini

        // Test with empty action name - should fail
        let result = node.create_action_client::<TestAction>("").build();
        assert!(result.is_err());

        // Test with invalid action name (contains spaces) - should work as remapping handles it
        let client = node
            .create_action_client::<TestAction>("/invalid name")
            .build()?;
        drop(client);

        Ok(())
    }
}
