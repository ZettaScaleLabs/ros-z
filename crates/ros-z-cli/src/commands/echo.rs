use std::time::Duration;

use color_eyre::eyre::{Result, WrapErr, bail, eyre};
use ros_z::{
    Builder,
    dynamic::{
        DynSub, DynamicMessage, DynamicNamedValue, DynamicValue, EnumPayloadValue, EnumValue,
        MessageSchemaTypeDescription,
    },
};
use serde_json::{Map, Value};

use crate::{
    app::AppContext,
    model::echo::{EchoHeader, EchoMessageView},
    render::{OutputMode, json, text},
};

const TYPE_DISCOVERY_TIMEOUT: Duration = Duration::from_secs(5);

pub async fn run(
    app: &AppContext,
    output_mode: OutputMode,
    topic: &str,
    count: Option<usize>,
    timeout: Option<f64>,
) -> Result<()> {
    let subscriber = app
        .create_dynamic_subscriber_builder(topic, TYPE_DISCOVERY_TIMEOUT)
        .await
        .and_then(|builder| builder.build().map_err(|error| eyre!(error)))
        .wrap_err_with(|| format!("failed to subscribe to {topic}"))?;
    let schema = subscriber
        .schema()
        .ok_or_else(|| eyre!("dynamic subscriber missing schema for {topic}"))?;
    let header = EchoHeader::new(
        topic.to_string(),
        schema.type_name.to_string(),
        schema
            .compute_type_hash()
            .map(|hash| hash.to_rihs_string())
            .unwrap_or_else(|_| "unknown".to_string()),
    );
    let deadline = timeout
        .map(Duration::from_secs_f64)
        .map(|timeout| tokio::time::Instant::now() + timeout);
    let mut seen = 0usize;

    if output_mode.is_text() {
        text::print_echo_header(&header);
    }

    loop {
        if count.is_some_and(|limit| seen >= limit) {
            return Ok(());
        }

        let message = receive_message(&subscriber, deadline, topic).await?;
        seen += 1;

        match output_mode {
            OutputMode::Json => {
                let view = EchoMessageView::new(
                    header.topic.clone(),
                    message.schema().type_name.to_string(),
                    header.type_hash.clone(),
                    dynamic_message_to_json(&message),
                );
                json::print_line(&view)?;
            }
            OutputMode::Text => {
                text::print_echo_message(&format_message_pretty(&message), count, seen);
            }
        }
    }
}

async fn receive_message(
    subscriber: &DynSub,
    deadline: Option<tokio::time::Instant>,
    topic: &str,
) -> Result<ros_z::dynamic::DynamicMessage> {
    let receive = subscriber.async_recv();

    match deadline {
        Some(deadline) => match tokio::time::timeout_at(deadline, receive).await {
            Ok(result) => {
                result.map_err(|error| eyre!("subscriber receive failed for {topic}: {error}"))
            }
            Err(_) => bail!("timed out waiting for messages on {topic}"),
        },
        None => receive
            .await
            .map_err(|error| eyre!("subscriber receive failed for {topic}: {error}")),
    }
}

fn dynamic_message_to_json(message: &DynamicMessage) -> Value {
    let mut fields = Map::new();
    for (name, value) in message.iter() {
        fields.insert(name.to_string(), dynamic_value_to_json(value));
    }
    Value::Object(fields)
}

fn dynamic_value_to_json(value: &DynamicValue) -> Value {
    match value {
        DynamicValue::Bool(value) => Value::Bool(*value),
        DynamicValue::Int8(value) => Value::Number((*value).into()),
        DynamicValue::Int16(value) => Value::Number((*value).into()),
        DynamicValue::Int32(value) => Value::Number((*value).into()),
        DynamicValue::Int64(value) => Value::Number((*value).into()),
        DynamicValue::Uint8(value) => Value::Number((*value).into()),
        DynamicValue::Uint16(value) => Value::Number((*value).into()),
        DynamicValue::Uint32(value) => Value::Number((*value).into()),
        DynamicValue::Uint64(value) => Value::Number((*value).into()),
        DynamicValue::Float32(value) => serde_json::Number::from_f64(*value as f64)
            .map(Value::Number)
            .unwrap_or(Value::Null),
        DynamicValue::Float64(value) => serde_json::Number::from_f64(*value)
            .map(Value::Number)
            .unwrap_or(Value::Null),
        DynamicValue::String(value) => Value::String(value.clone()),
        DynamicValue::Bytes(value) => Value::Array(
            value
                .iter()
                .map(|byte| Value::Number((*byte).into()))
                .collect(),
        ),
        DynamicValue::Message(value) => dynamic_message_to_json(value),
        DynamicValue::Optional(None) => Value::Null,
        DynamicValue::Optional(Some(value)) => dynamic_value_to_json(value),
        DynamicValue::Enum(value) => enum_value_to_json(value),
        DynamicValue::Array(values) => {
            Value::Array(values.iter().map(dynamic_value_to_json).collect())
        }
    }
}

fn enum_value_to_json(value: &EnumValue) -> Value {
    let mut fields = Map::new();
    fields.insert(
        "variant_index".to_string(),
        Value::Number(value.variant_index.into()),
    );
    fields.insert(
        "variant_name".to_string(),
        Value::String(value.variant_name.clone()),
    );
    fields.insert("payload".to_string(), enum_payload_to_json(&value.payload));
    Value::Object(fields)
}

fn enum_payload_to_json(payload: &EnumPayloadValue) -> Value {
    match payload {
        EnumPayloadValue::Unit => Value::Null,
        EnumPayloadValue::Newtype(value) => dynamic_value_to_json(value),
        EnumPayloadValue::Tuple(values) => {
            Value::Array(values.iter().map(dynamic_value_to_json).collect())
        }
        EnumPayloadValue::Struct(fields) => Value::Object(
            fields
                .iter()
                .map(|field| (field.name.clone(), dynamic_value_to_json(&field.value)))
                .collect(),
        ),
    }
}

fn format_message_pretty(message: &DynamicMessage) -> String {
    let mut output = String::new();
    for (name, value) in message.iter() {
        format_value_pretty(&mut output, name, value, 0);
    }
    output
}

fn format_value_pretty(output: &mut String, name: &str, value: &DynamicValue, indent: usize) {
    let prefix = "  ".repeat(indent);

    match value {
        DynamicValue::Bool(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Int8(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Int16(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Int32(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Int64(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Uint8(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Uint16(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Uint32(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Uint64(value) => output.push_str(&format!("{}{}: {}\n", prefix, name, value)),
        DynamicValue::Float32(value) => {
            output.push_str(&format!("{}{}: {}\n", prefix, name, value))
        }
        DynamicValue::Float64(value) => {
            output.push_str(&format!("{}{}: {}\n", prefix, name, value))
        }
        DynamicValue::String(value) => {
            output.push_str(&format!("{}{}: \"{}\"\n", prefix, name, value))
        }
        DynamicValue::Bytes(value) => output.push_str(&format!(
            "{}{}: [bytes: {} bytes]\n",
            prefix,
            name,
            value.len()
        )),
        DynamicValue::Message(value) => {
            output.push_str(&format!("{}{}:\n", prefix, name));
            for (nested_name, nested_value) in value.iter() {
                format_value_pretty(output, nested_name, nested_value, indent + 1);
            }
        }
        DynamicValue::Optional(None) => output.push_str(&format!("{}{}: null\n", prefix, name)),
        DynamicValue::Optional(Some(value)) => format_value_pretty(output, name, value, indent),
        DynamicValue::Enum(value) => format_enum_value_pretty(output, name, value, indent),
        DynamicValue::Array(values) => {
            if values.is_empty() {
                output.push_str(&format!("{}{}[]: []\n", prefix, name));
            } else {
                output.push_str(&format!("{}{}[{}]:\n", prefix, name, values.len()));
                for (index, value) in values.iter().enumerate() {
                    format_value_pretty(output, &format!("[{}]", index), value, indent + 1);
                }
            }
        }
    }
}

fn format_enum_value_pretty(output: &mut String, name: &str, value: &EnumValue, indent: usize) {
    let prefix = "  ".repeat(indent);
    output.push_str(&format!("{}{}:\n", prefix, name));
    output.push_str(&format!("{}  variant: {}\n", prefix, value.variant_name));
    format_enum_payload_pretty(output, &value.payload, indent + 1);
}

fn format_enum_payload_pretty(output: &mut String, payload: &EnumPayloadValue, indent: usize) {
    let prefix = "  ".repeat(indent);

    match payload {
        EnumPayloadValue::Unit => output.push_str(&format!("{}payload: null\n", prefix)),
        EnumPayloadValue::Newtype(value) => format_value_pretty(output, "payload", value, indent),
        EnumPayloadValue::Tuple(values) => {
            if values.is_empty() {
                output.push_str(&format!("{}payload[]: []\n", prefix));
            } else {
                output.push_str(&format!("{}payload[{}]:\n", prefix, values.len()));
                for (index, value) in values.iter().enumerate() {
                    format_value_pretty(output, &format!("[{}]", index), value, indent + 1);
                }
            }
        }
        EnumPayloadValue::Struct(fields) => {
            if fields.is_empty() {
                output.push_str(&format!("{}payload: {{}}\n", prefix));
            } else {
                output.push_str(&format!("{}payload:\n", prefix));
                format_named_fields_pretty(output, fields, indent + 1);
            }
        }
    }
}

fn format_named_fields_pretty(output: &mut String, fields: &[DynamicNamedValue], indent: usize) {
    for field in fields {
        format_value_pretty(output, &field.name, &field.value, indent);
    }
}
