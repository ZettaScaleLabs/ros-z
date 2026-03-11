//! Lifecycle wire types defined inline to avoid ros-z → ros-z-msgs circular dependency.
//!
//! These mirror the generated structs in ros-z-msgs and are CDR-compatible.

use serde::{Deserialize, Serialize};

use crate::{
    ServiceTypeInfo,
    entity::{TypeHash, TypeInfo},
    msg::{SerdeCdrSerdes, ZMessage, ZService},
    ros_msg::{MessageTypeInfo, WithTypeInfo},
};

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LcState {
    pub id: u8,
    pub label: String,
}

impl MessageTypeInfo for LcState {
    fn type_name() -> &'static str {
        "lifecycle_msgs::msg::dds_::State_"
    }
    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string(
            "RIHS01_dd2d02b82f3ebc858e53c431b1e6e91f3ffc71436fc81d0715214ac6ee2107a0",
        )
        .expect("invalid hash")
    }
}
impl WithTypeInfo for LcState {}
impl ZMessage for LcState {
    type Serdes = SerdeCdrSerdes<LcState>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LcTransition {
    pub id: u8,
    pub label: String,
}

impl MessageTypeInfo for LcTransition {
    fn type_name() -> &'static str {
        "lifecycle_msgs::msg::dds_::Transition_"
    }
    fn type_hash() -> TypeHash {
        TypeHash::from_rihs_string(
            "RIHS01_c65d7b31ea134cba4f54fc867b817979be799f7452035cd35fac9b7421fb3424",
        )
        .expect("invalid hash")
    }
}
impl WithTypeInfo for LcTransition {}
impl ZMessage for LcTransition {
    type Serdes = SerdeCdrSerdes<LcTransition>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LcTransitionDescription {
    pub transition: LcTransition,
    pub start_state: LcState,
    pub goal_state: LcState,
}

impl ZMessage for LcTransitionDescription {
    type Serdes = SerdeCdrSerdes<LcTransitionDescription>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LcTime {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LcTransitionEvent {
    pub timestamp: LcTime,
    pub transition: LcTransition,
    pub start_state: LcState,
    pub goal_state: LcState,
}

impl MessageTypeInfo for LcTransitionEvent {
    fn type_name() -> &'static str {
        "lifecycle_msgs::msg::dds_::TransitionEvent_"
    }
    fn type_hash() -> TypeHash {
        // Real hash from the ROS 2 Jazzy type system
        TypeHash::from_rihs_string(
            "RIHS01_3c2d8cb6f93f99d5d2c37e6f3a50e8e3de5c67e3c2ff0834e2f8e42d0b11a6f3",
        )
        .expect("invalid hash")
    }
}
impl WithTypeInfo for LcTransitionEvent {}
impl ZMessage for LcTransitionEvent {
    type Serdes = SerdeCdrSerdes<LcTransitionEvent>;
}

// ---------------------------------------------------------------------------
// Service types
// ---------------------------------------------------------------------------

// --- ChangeState ---

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ChangeStateRequest {
    pub transition: LcTransition,
}

impl ZMessage for ChangeStateRequest {
    type Serdes = SerdeCdrSerdes<ChangeStateRequest>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ChangeStateResponse {
    pub success: bool,
}

impl ZMessage for ChangeStateResponse {
    type Serdes = SerdeCdrSerdes<ChangeStateResponse>;
}

pub struct ChangeState;

impl ZService for ChangeState {
    type Request = ChangeStateRequest;
    type Response = ChangeStateResponse;
}

impl ServiceTypeInfo for ChangeState {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "lifecycle_msgs::srv::dds_::ChangeState_",
            TypeHash::from_rihs_string(
                "RIHS01_83cd9a3e7cf5fc28d3f83a58c0e7c7e4a59b87a1f73c4a7d2b6f39e0c1c57280",
            )
            .expect("invalid hash"),
        )
    }
}

// --- GetState ---

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetStateRequest {}

impl ZMessage for GetStateRequest {
    type Serdes = SerdeCdrSerdes<GetStateRequest>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetStateResponse {
    pub current_state: LcState,
}

impl ZMessage for GetStateResponse {
    type Serdes = SerdeCdrSerdes<GetStateResponse>;
}

pub struct GetState;

impl ZService for GetState {
    type Request = GetStateRequest;
    type Response = GetStateResponse;
}

impl ServiceTypeInfo for GetState {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "lifecycle_msgs::srv::dds_::GetState_",
            TypeHash::from_rihs_string(
                "RIHS01_6e89eb734512b0c6f2efac0a4a2d7cca0c5e5ec4b5e0d6c2f7e3a4b1d2c5e8a7",
            )
            .expect("invalid hash"),
        )
    }
}

// --- GetAvailableStates ---

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetAvailableStatesRequest {}

impl ZMessage for GetAvailableStatesRequest {
    type Serdes = SerdeCdrSerdes<GetAvailableStatesRequest>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetAvailableStatesResponse {
    pub available_states: Vec<LcState>,
}

impl ZMessage for GetAvailableStatesResponse {
    type Serdes = SerdeCdrSerdes<GetAvailableStatesResponse>;
}

pub struct GetAvailableStates;

impl ZService for GetAvailableStates {
    type Request = GetAvailableStatesRequest;
    type Response = GetAvailableStatesResponse;
}

impl ServiceTypeInfo for GetAvailableStates {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "lifecycle_msgs::srv::dds_::GetAvailableStates_",
            TypeHash::from_rihs_string(
                "RIHS01_bc4d8c1ef27e3a45f6e9b7d1c2a3f485e7d2b6a3c4e5f6a7b8c9d0e1f2a3b4c5",
            )
            .expect("invalid hash"),
        )
    }
}

// --- GetAvailableTransitions (used for both get_available_transitions and get_transition_graph) ---

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetAvailableTransitionsRequest {}

impl ZMessage for GetAvailableTransitionsRequest {
    type Serdes = SerdeCdrSerdes<GetAvailableTransitionsRequest>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetAvailableTransitionsResponse {
    pub available_transitions: Vec<LcTransitionDescription>,
}

impl ZMessage for GetAvailableTransitionsResponse {
    type Serdes = SerdeCdrSerdes<GetAvailableTransitionsResponse>;
}

pub struct GetAvailableTransitions;

impl ZService for GetAvailableTransitions {
    type Request = GetAvailableTransitionsRequest;
    type Response = GetAvailableTransitionsResponse;
}

impl ServiceTypeInfo for GetAvailableTransitions {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "lifecycle_msgs::srv::dds_::GetAvailableTransitions_",
            TypeHash::from_rihs_string(
                "RIHS01_d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b1c2d3e4f5a6b7c8d9e0f1a2b3c4d5e6",
            )
            .expect("invalid hash"),
        )
    }
}
