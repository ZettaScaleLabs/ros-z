use color_eyre::eyre::Result;
use ros_z::parameter::ParameterValue;
use serde::Serialize;
use serde_json::{Value, json};

#[derive(Debug, Clone, Serialize)]
pub struct ParameterListView {
    pub node: String,
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

impl ParameterListView {
    pub fn new(node: String, names: Vec<String>, prefixes: Vec<String>) -> Self {
        Self {
            node,
            names,
            prefixes,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ParameterValueView {
    #[serde(rename = "type")]
    pub value_type: String,
    pub value: Value,
    #[serde(skip_serializing)]
    pub display: String,
}

impl ParameterValueView {
    pub fn from_parameter_value(value: &ParameterValue) -> Result<Self> {
        Ok(Self {
            value_type: parameter_value_type_name(value).to_string(),
            value: parameter_value_to_json(value),
            display: format_parameter_value(value)?,
        })
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ParameterGetView {
    pub node: String,
    pub name: String,
    #[serde(flatten)]
    pub parameter: ParameterValueView,
}

impl ParameterGetView {
    pub fn new(node: String, name: String, parameter: ParameterValueView) -> Self {
        Self {
            node,
            name,
            parameter,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ParameterSetView {
    pub node: String,
    pub name: String,
    #[serde(flatten)]
    pub parameter: ParameterValueView,
    pub atomic: bool,
    pub successful: bool,
}

impl ParameterSetView {
    pub fn new(
        node: String,
        name: String,
        parameter: ParameterValueView,
        atomic: bool,
        successful: bool,
    ) -> Self {
        Self {
            node,
            name,
            parameter,
            atomic,
            successful,
        }
    }
}

fn parameter_value_to_json(value: &ParameterValue) -> Value {
    match value {
        ParameterValue::NotSet => Value::Null,
        ParameterValue::Bool(value) => json!(value),
        ParameterValue::Integer(value) => json!(value),
        ParameterValue::Double(value) => json!(value),
        ParameterValue::String(value) => json!(value),
        ParameterValue::ByteArray(value) => json!(value),
        ParameterValue::BoolArray(value) => json!(value),
        ParameterValue::IntegerArray(value) => json!(value),
        ParameterValue::DoubleArray(value) => json!(value),
        ParameterValue::StringArray(value) => json!(value),
    }
}

fn parameter_value_type_name(value: &ParameterValue) -> &'static str {
    match value {
        ParameterValue::NotSet => "not_set",
        ParameterValue::Bool(_) => "bool",
        ParameterValue::Integer(_) => "integer",
        ParameterValue::Double(_) => "double",
        ParameterValue::String(_) => "string",
        ParameterValue::ByteArray(_) => "byte_array",
        ParameterValue::BoolArray(_) => "bool_array",
        ParameterValue::IntegerArray(_) => "integer_array",
        ParameterValue::DoubleArray(_) => "double_array",
        ParameterValue::StringArray(_) => "string_array",
    }
}

fn format_parameter_value(value: &ParameterValue) -> Result<String> {
    Ok(match value {
        ParameterValue::String(value) => value.clone(),
        _ => serde_json::to_string(&parameter_value_to_json(value))?,
    })
}
