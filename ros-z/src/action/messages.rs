use super::{GoalId, GoalInfo, GoalStatus, ZAction};
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;

// Standard ROS 2 cancel messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelGoalRequest {
    pub goal_info: GoalInfo,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelGoalResponse {
    pub return_code: i8,
    pub goals_canceling: Vec<GoalInfo>,
}

// Internal service/topic message types
#[derive(Debug, Clone)]
pub struct GoalRequest<A: ZAction> {
    pub goal_id: GoalId,
    pub goal: A::Goal,
}

impl<A: ZAction> serde::Serialize for GoalRequest<A>
where
    A: 'static,
    A::Goal: serde::Serialize + 'static,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("GoalRequest", 2)?;
        state.serialize_field("goal_id", &self.goal_id)?;
        state.serialize_field("goal", &self.goal)?;
        state.end()
    }
}

impl<'de, A: ZAction> serde::Deserialize<'de> for GoalRequest<A>
where
    A: 'static,
    A::Goal: serde::Deserialize<'de> + 'static,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(serde::Deserialize)]
        struct GoalRequestHelper<B> {
            goal_id: GoalId,
            goal: B,
        }
        let helper = GoalRequestHelper::<A::Goal>::deserialize(deserializer)?;
        Ok(GoalRequest {
            goal_id: helper.goal_id,
            goal: helper.goal,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoalResponse {
    pub accepted: bool,
    pub stamp: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResultRequest {
    pub goal_id: GoalId,
}

#[derive(Debug, Clone)]
pub struct ResultResponse<A: ZAction> {
    pub status: GoalStatus,
    pub result: A::Result,
}

impl<A: ZAction> serde::Serialize for ResultResponse<A>
where
    A: 'static,
    A::Result: serde::Serialize + 'static,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("ResultResponse", 2)?;
        state.serialize_field("status", &self.status)?;
        state.serialize_field("result", &self.result)?;
        state.end()
    }
}

impl<'de, A: ZAction> serde::Deserialize<'de> for ResultResponse<A>
where
    A: 'static,
    A::Result: serde::Deserialize<'de> + 'static,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(serde::Deserialize)]
        struct ResultResponseHelper<B> {
            status: GoalStatus,
            result: B,
        }
        let helper = ResultResponseHelper::<A::Result>::deserialize(deserializer)?;
        Ok(ResultResponse {
            status: helper.status,
            result: helper.result,
        })
    }
}

#[derive(Debug, Clone)]
pub struct FeedbackMessage<A: ZAction> {
    pub goal_id: GoalId,
    pub feedback: A::Feedback,
}

impl<A: ZAction> serde::Serialize for FeedbackMessage<A>
where
    A: 'static,
    A::Feedback: serde::Serialize + 'static,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("FeedbackMessage", 2)?;
        state.serialize_field("goal_id", &self.goal_id)?;
        state.serialize_field("feedback", &self.feedback)?;
        state.end()
    }
}

impl<'de, A: ZAction> serde::Deserialize<'de> for FeedbackMessage<A>
where
    A: 'static,
    A::Feedback: serde::Deserialize<'de> + 'static,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(serde::Deserialize)]
        struct FeedbackMessageHelper<B> {
            goal_id: GoalId,
            feedback: B,
        }
        let helper = FeedbackMessageHelper::<A::Feedback>::deserialize(deserializer)?;
        Ok(FeedbackMessage {
            goal_id: helper.goal_id,
            feedback: helper.feedback,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusMessage {
    pub status_list: Vec<GoalStatusInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoalStatusInfo {
    pub goal_info: GoalInfo,
    pub status: GoalStatus,
}

// Internal service type wrappers
pub struct GoalService<A: ZAction>(PhantomData<A>);
impl<A: ZAction> crate::msg::ZService for GoalService<A>
where
    A: 'static,
    A::Goal: serde::Serialize + for<'de> serde::Deserialize<'de> + 'static,
{
    type Request = GoalRequest<A>;
    type Response = GoalResponse;
}

pub struct ResultService<A: ZAction>(PhantomData<A>);
impl<A: ZAction> crate::msg::ZService for ResultService<A>
where
    A: 'static,
    A::Result: serde::Serialize + for<'de> serde::Deserialize<'de> + 'static,
{
    type Request = ResultRequest;
    type Response = ResultResponse<A>;
}

pub struct CancelService;
impl crate::msg::ZService for CancelService {
    type Request = CancelGoalRequest;
    type Response = CancelGoalResponse;
}