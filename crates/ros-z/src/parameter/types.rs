//! User-facing parameter types.
//!
//! These types provide an ergonomic Rust API for declaring, reading, and
//! updating parameters. They convert to/from the wire format types for
//! CDR serialization.

use super::wire_types::{self, WireParameterValue, parameter_type};
use crate::ZBuf;

/// The type of a parameter value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParameterType {
    NotSet,
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

impl ParameterType {
    pub fn to_u8(self) -> u8 {
        match self {
            Self::NotSet => parameter_type::NOT_SET,
            Self::Bool => parameter_type::BOOL,
            Self::Integer => parameter_type::INTEGER,
            Self::Double => parameter_type::DOUBLE,
            Self::String => parameter_type::STRING,
            Self::ByteArray => parameter_type::BYTE_ARRAY,
            Self::BoolArray => parameter_type::BOOL_ARRAY,
            Self::IntegerArray => parameter_type::INTEGER_ARRAY,
            Self::DoubleArray => parameter_type::DOUBLE_ARRAY,
            Self::StringArray => parameter_type::STRING_ARRAY,
        }
    }

    pub fn from_u8(v: u8) -> Self {
        match v {
            parameter_type::BOOL => Self::Bool,
            parameter_type::INTEGER => Self::Integer,
            parameter_type::DOUBLE => Self::Double,
            parameter_type::STRING => Self::String,
            parameter_type::BYTE_ARRAY => Self::ByteArray,
            parameter_type::BOOL_ARRAY => Self::BoolArray,
            parameter_type::INTEGER_ARRAY => Self::IntegerArray,
            parameter_type::DOUBLE_ARRAY => Self::DoubleArray,
            parameter_type::STRING_ARRAY => Self::StringArray,
            _ => Self::NotSet,
        }
    }
}

/// A typed parameter value.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum ParameterValue {
    #[default]
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(std::string::String),
    ByteArray(Vec<u8>),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<std::string::String>),
}

impl ParameterValue {
    /// Returns the parameter type of this value.
    pub fn parameter_type(&self) -> ParameterType {
        match self {
            Self::NotSet => ParameterType::NotSet,
            Self::Bool(_) => ParameterType::Bool,
            Self::Integer(_) => ParameterType::Integer,
            Self::Double(_) => ParameterType::Double,
            Self::String(_) => ParameterType::String,
            Self::ByteArray(_) => ParameterType::ByteArray,
            Self::BoolArray(_) => ParameterType::BoolArray,
            Self::IntegerArray(_) => ParameterType::IntegerArray,
            Self::DoubleArray(_) => ParameterType::DoubleArray,
            Self::StringArray(_) => ParameterType::StringArray,
        }
    }

    /// Convert to wire format.
    pub(crate) fn to_wire(&self) -> WireParameterValue {
        let mut wire = WireParameterValue {
            r#type: self.parameter_type().to_u8(),
            ..Default::default()
        };
        match self {
            Self::NotSet => {}
            Self::Bool(v) => wire.bool_value = *v,
            Self::Integer(v) => wire.integer_value = *v,
            Self::Double(v) => wire.double_value = *v,
            Self::String(v) => wire.string_value = v.clone(),
            Self::ByteArray(v) => wire.byte_array_value = ZBuf::from(v.clone()),
            Self::BoolArray(v) => wire.bool_array_value = v.clone(),
            Self::IntegerArray(v) => wire.integer_array_value = v.clone(),
            Self::DoubleArray(v) => wire.double_array_value = v.clone(),
            Self::StringArray(v) => wire.string_array_value = v.clone(),
        }
        wire
    }

    /// Convert from wire format.
    pub(crate) fn from_wire(wire: &WireParameterValue) -> Self {
        match wire.r#type {
            parameter_type::BOOL => Self::Bool(wire.bool_value),
            parameter_type::INTEGER => Self::Integer(wire.integer_value),
            parameter_type::DOUBLE => Self::Double(wire.double_value),
            parameter_type::STRING => Self::String(wire.string_value.clone()),
            parameter_type::BYTE_ARRAY => {
                use zenoh_buffers::buffer::Buffer;
                let zbuf = wire.byte_array_value.clone().into_inner();
                let mut bytes = Vec::with_capacity(zbuf.len());
                for slice in zbuf.zslices() {
                    bytes.extend_from_slice(slice.as_slice());
                }
                Self::ByteArray(bytes)
            }
            parameter_type::BOOL_ARRAY => Self::BoolArray(wire.bool_array_value.clone()),
            parameter_type::INTEGER_ARRAY => Self::IntegerArray(wire.integer_array_value.clone()),
            parameter_type::DOUBLE_ARRAY => Self::DoubleArray(wire.double_array_value.clone()),
            parameter_type::STRING_ARRAY => Self::StringArray(wire.string_array_value.clone()),
            _ => Self::NotSet,
        }
    }
}

/// A parameter with its name and value.
#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: std::string::String,
    pub value: ParameterValue,
}

impl Parameter {
    pub fn new(name: impl Into<std::string::String>, value: ParameterValue) -> Self {
        Self {
            name: name.into(),
            value,
        }
    }

    pub(crate) fn to_wire(&self) -> wire_types::WireParameter {
        wire_types::WireParameter {
            name: self.name.clone(),
            value: self.value.to_wire(),
        }
    }

    pub(crate) fn from_wire(wire: &wire_types::WireParameter) -> Self {
        Self {
            name: wire.name.clone(),
            value: ParameterValue::from_wire(&wire.value),
        }
    }
}

/// Range constraint for floating point parameters.
#[derive(Debug, Clone)]
pub struct FloatingPointRange {
    pub from_value: f64,
    pub to_value: f64,
    pub step: f64,
}

impl FloatingPointRange {
    pub(crate) fn to_wire(&self) -> wire_types::WireFloatingPointRange {
        wire_types::WireFloatingPointRange {
            from_value: self.from_value,
            to_value: self.to_value,
            step: self.step,
        }
    }

    pub fn from_wire(wire: &wire_types::WireFloatingPointRange) -> Self {
        Self {
            from_value: wire.from_value,
            to_value: wire.to_value,
            step: wire.step,
        }
    }
}

/// Range constraint for integer parameters.
#[derive(Debug, Clone)]
pub struct IntegerRange {
    pub from_value: i64,
    pub to_value: i64,
    pub step: u64,
}

impl IntegerRange {
    pub(crate) fn to_wire(&self) -> wire_types::WireIntegerRange {
        wire_types::WireIntegerRange {
            from_value: self.from_value,
            to_value: self.to_value,
            step: self.step,
        }
    }

    pub fn from_wire(wire: &wire_types::WireIntegerRange) -> Self {
        Self {
            from_value: wire.from_value,
            to_value: wire.to_value,
            step: wire.step,
        }
    }
}

/// Descriptor for a parameter, including constraints.
#[derive(Debug, Clone)]
pub struct ParameterDescriptor {
    pub name: std::string::String,
    pub type_: ParameterType,
    pub description: std::string::String,
    pub additional_constraints: std::string::String,
    pub read_only: bool,
    pub dynamic_typing: bool,
    pub floating_point_range: Option<FloatingPointRange>,
    pub integer_range: Option<IntegerRange>,
}

impl Default for ParameterDescriptor {
    fn default() -> Self {
        Self {
            name: std::string::String::new(),
            type_: ParameterType::NotSet,
            description: std::string::String::new(),
            additional_constraints: std::string::String::new(),
            read_only: false,
            dynamic_typing: false,
            floating_point_range: None,
            integer_range: None,
        }
    }
}

impl ParameterDescriptor {
    /// Create a new descriptor with the given name and type.
    pub fn new(name: impl Into<std::string::String>, type_: ParameterType) -> Self {
        Self {
            name: name.into(),
            type_,
            ..Default::default()
        }
    }

    pub(crate) fn to_wire(&self) -> wire_types::WireParameterDescriptor {
        wire_types::WireParameterDescriptor {
            name: self.name.clone(),
            r#type: self.type_.to_u8(),
            description: self.description.clone(),
            additional_constraints: self.additional_constraints.clone(),
            read_only: self.read_only,
            dynamic_typing: self.dynamic_typing,
            floating_point_range: self
                .floating_point_range
                .as_ref()
                .map(|r| vec![r.to_wire()])
                .unwrap_or_default(),
            integer_range: self
                .integer_range
                .as_ref()
                .map(|r| vec![r.to_wire()])
                .unwrap_or_default(),
        }
    }

    pub fn from_wire(wire: &wire_types::WireParameterDescriptor) -> Self {
        Self {
            name: wire.name.clone(),
            type_: ParameterType::from_u8(wire.r#type),
            description: wire.description.clone(),
            additional_constraints: wire.additional_constraints.clone(),
            read_only: wire.read_only,
            dynamic_typing: wire.dynamic_typing,
            floating_point_range: wire
                .floating_point_range
                .first()
                .map(FloatingPointRange::from_wire),
            integer_range: wire.integer_range.first().map(IntegerRange::from_wire),
        }
    }
}

/// Result of a set_parameters operation.
#[derive(Debug, Clone)]
pub struct SetParametersResult {
    pub successful: bool,
    pub reason: std::string::String,
}

impl SetParametersResult {
    pub fn success() -> Self {
        Self {
            successful: true,
            reason: std::string::String::new(),
        }
    }

    pub fn failure(reason: impl Into<std::string::String>) -> Self {
        Self {
            successful: false,
            reason: reason.into(),
        }
    }

    pub(crate) fn to_wire(&self) -> wire_types::WireSetParametersResult {
        wire_types::WireSetParametersResult {
            successful: self.successful,
            reason: self.reason.clone(),
        }
    }
}
