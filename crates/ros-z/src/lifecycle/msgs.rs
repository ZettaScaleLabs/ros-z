//! Lifecycle wire types defined inline to avoid ros-z → ros-z-msgs circular dependency.
//!
//! These mirror the generated structs in ros-z-msgs and are CDR-compatible.

use ros_z_cdr::{CdrBuffer, CdrDeserialize, CdrReader, CdrSerialize, CdrSerializedSize, CdrWriter};

use crate::{
    ServiceTypeInfo,
    entity::{TypeHash, TypeInfo},
    msg::ZService,
    ros_msg::{MessageTypeInfo, WithTypeInfo},
};

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
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

impl CdrSerialize for LcState {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.id.cdr_serialize(w);
        self.label.cdr_serialize(w);
    }
}
impl CdrDeserialize for LcState {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(LcState {
            id: u8::cdr_deserialize(r)?,
            label: String::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for LcState {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        let p = self.id.cdr_serialized_size(pos);
        self.label.cdr_serialized_size(p)
    }
}

#[derive(Debug, Clone, Default)]
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

impl CdrSerialize for LcTransition {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.id.cdr_serialize(w);
        self.label.cdr_serialize(w);
    }
}
impl CdrDeserialize for LcTransition {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(LcTransition {
            id: u8::cdr_deserialize(r)?,
            label: String::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for LcTransition {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        let p = self.id.cdr_serialized_size(pos);
        self.label.cdr_serialized_size(p)
    }
}

#[derive(Debug, Clone, Default)]
pub struct LcTransitionDescription {
    pub transition: LcTransition,
    pub start_state: LcState,
    pub goal_state: LcState,
}

impl CdrSerialize for LcTransitionDescription {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.transition.cdr_serialize(w);
        self.start_state.cdr_serialize(w);
        self.goal_state.cdr_serialize(w);
    }
}
impl CdrDeserialize for LcTransitionDescription {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(LcTransitionDescription {
            transition: LcTransition::cdr_deserialize(r)?,
            start_state: LcState::cdr_deserialize(r)?,
            goal_state: LcState::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for LcTransitionDescription {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        let p = self.transition.cdr_serialized_size(pos);
        let p = self.start_state.cdr_serialized_size(p);
        self.goal_state.cdr_serialized_size(p)
    }
}

#[derive(Debug, Clone, Default)]
pub struct LcTime {
    pub sec: i32,
    pub nanosec: u32,
}

impl CdrSerialize for LcTime {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.sec.cdr_serialize(w);
        self.nanosec.cdr_serialize(w);
    }
}
impl CdrDeserialize for LcTime {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(LcTime {
            sec: i32::cdr_deserialize(r)?,
            nanosec: u32::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for LcTime {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        let p = self.sec.cdr_serialized_size(pos);
        self.nanosec.cdr_serialized_size(p)
    }
}

#[derive(Debug, Clone, Default)]
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
        TypeHash::from_rihs_string(
            "RIHS01_3c2d8cb6f93f99d5d2c37e6f3a50e8e3de5c67e3c2ff0834e2f8e42d0b11a6f3",
        )
        .expect("invalid hash")
    }
}
impl WithTypeInfo for LcTransitionEvent {}

impl CdrSerialize for LcTransitionEvent {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.timestamp.cdr_serialize(w);
        self.transition.cdr_serialize(w);
        self.start_state.cdr_serialize(w);
        self.goal_state.cdr_serialize(w);
    }
}
impl CdrDeserialize for LcTransitionEvent {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(LcTransitionEvent {
            timestamp: LcTime::cdr_deserialize(r)?,
            transition: LcTransition::cdr_deserialize(r)?,
            start_state: LcState::cdr_deserialize(r)?,
            goal_state: LcState::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for LcTransitionEvent {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        let p = self.timestamp.cdr_serialized_size(pos);
        let p = self.transition.cdr_serialized_size(p);
        let p = self.start_state.cdr_serialized_size(p);
        self.goal_state.cdr_serialized_size(p)
    }
}

// ---------------------------------------------------------------------------
// Service types
// ---------------------------------------------------------------------------

// --- ChangeState ---

#[derive(Debug, Clone, Default)]
pub struct ChangeStateRequest {
    pub transition: LcTransition,
}

impl CdrSerialize for ChangeStateRequest {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.transition.cdr_serialize(w);
    }
}
impl CdrDeserialize for ChangeStateRequest {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(ChangeStateRequest {
            transition: LcTransition::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for ChangeStateRequest {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        self.transition.cdr_serialized_size(pos)
    }
}

#[derive(Debug, Clone, Default)]
pub struct ChangeStateResponse {
    pub success: bool,
}

impl CdrSerialize for ChangeStateResponse {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.success.cdr_serialize(w);
    }
}
impl CdrDeserialize for ChangeStateResponse {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(ChangeStateResponse {
            success: bool::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for ChangeStateResponse {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        self.success.cdr_serialized_size(pos)
    }
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

#[derive(Debug, Clone, Default)]
pub struct GetStateRequest {}

impl CdrSerialize for GetStateRequest {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, _w: &mut CdrWriter<'_, BO, B>) {
    }
}
impl CdrDeserialize for GetStateRequest {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        _r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetStateRequest {})
    }
}
impl CdrSerializedSize for GetStateRequest {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        pos
    }
}

#[derive(Debug, Clone, Default)]
pub struct GetStateResponse {
    pub current_state: LcState,
}

impl CdrSerialize for GetStateResponse {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.current_state.cdr_serialize(w);
    }
}
impl CdrDeserialize for GetStateResponse {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetStateResponse {
            current_state: LcState::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for GetStateResponse {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        self.current_state.cdr_serialized_size(pos)
    }
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

#[derive(Debug, Clone, Default)]
pub struct GetAvailableStatesRequest {}

impl CdrSerialize for GetAvailableStatesRequest {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, _w: &mut CdrWriter<'_, BO, B>) {
    }
}
impl CdrDeserialize for GetAvailableStatesRequest {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        _r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetAvailableStatesRequest {})
    }
}
impl CdrSerializedSize for GetAvailableStatesRequest {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        pos
    }
}

#[derive(Debug, Clone, Default)]
pub struct GetAvailableStatesResponse {
    pub available_states: Vec<LcState>,
}

impl CdrSerialize for GetAvailableStatesResponse {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.available_states.cdr_serialize(w);
    }
}
impl CdrDeserialize for GetAvailableStatesResponse {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetAvailableStatesResponse {
            available_states: Vec::<LcState>::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for GetAvailableStatesResponse {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        self.available_states.cdr_serialized_size(pos)
    }
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

#[derive(Debug, Clone, Default)]
pub struct GetAvailableTransitionsRequest {}

impl CdrSerialize for GetAvailableTransitionsRequest {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, _w: &mut CdrWriter<'_, BO, B>) {
    }
}
impl CdrDeserialize for GetAvailableTransitionsRequest {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        _r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetAvailableTransitionsRequest {})
    }
}
impl CdrSerializedSize for GetAvailableTransitionsRequest {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        pos
    }
}

#[derive(Debug, Clone, Default)]
pub struct GetAvailableTransitionsResponse {
    pub available_transitions: Vec<LcTransitionDescription>,
}

impl CdrSerialize for GetAvailableTransitionsResponse {
    fn cdr_serialize<BO: byteorder::ByteOrder, B: CdrBuffer>(&self, w: &mut CdrWriter<'_, BO, B>) {
        self.available_transitions.cdr_serialize(w);
    }
}
impl CdrDeserialize for GetAvailableTransitionsResponse {
    fn cdr_deserialize<'de, BO: byteorder::ByteOrder>(
        r: &mut CdrReader<'de, BO>,
    ) -> ros_z_cdr::Result<Self> {
        Ok(GetAvailableTransitionsResponse {
            available_transitions: Vec::<LcTransitionDescription>::cdr_deserialize(r)?,
        })
    }
}
impl CdrSerializedSize for GetAvailableTransitionsResponse {
    fn cdr_serialized_size(&self, pos: usize) -> usize {
        self.available_transitions.cdr_serialized_size(pos)
    }
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
