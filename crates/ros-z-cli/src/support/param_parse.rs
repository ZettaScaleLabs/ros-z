use color_eyre::eyre::{Result, bail};
use ros_z::parameter::ParameterValue;
use serde_json::Value;

use crate::cli::ParameterValueTypeArg;

pub fn parse_parameter_value(
    input: &str,
    value_type: Option<ParameterValueTypeArg>,
) -> Result<ParameterValue> {
    match value_type {
        Some(value_type) => parse_typed_parameter_value(input, value_type),
        None => infer_parameter_value(input),
    }
}

fn parse_typed_parameter_value(
    input: &str,
    value_type: ParameterValueTypeArg,
) -> Result<ParameterValue> {
    match value_type {
        ParameterValueTypeArg::Bool => Ok(ParameterValue::Bool(parse_bool(input)?)),
        ParameterValueTypeArg::Integer => Ok(ParameterValue::Integer(input.parse()?)),
        ParameterValueTypeArg::Double => Ok(ParameterValue::Double(input.parse()?)),
        ParameterValueTypeArg::String => Ok(ParameterValue::String(input.to_string())),
        ParameterValueTypeArg::ByteArray => Ok(ParameterValue::ByteArray(parse_byte_array(input)?)),
        ParameterValueTypeArg::BoolArray => Ok(ParameterValue::BoolArray(parse_bool_array(input)?)),
        ParameterValueTypeArg::IntegerArray => {
            Ok(ParameterValue::IntegerArray(parse_integer_array(input)?))
        }
        ParameterValueTypeArg::DoubleArray => {
            Ok(ParameterValue::DoubleArray(parse_double_array(input)?))
        }
        ParameterValueTypeArg::StringArray => {
            Ok(ParameterValue::StringArray(parse_string_array(input)?))
        }
        ParameterValueTypeArg::NotSet => Ok(ParameterValue::NotSet),
    }
}

fn infer_parameter_value(input: &str) -> Result<ParameterValue> {
    if let Ok(values) = parse_json_array(input) {
        return infer_array_parameter_value(values);
    }

    if let Ok(value) = parse_bool(input) {
        return Ok(ParameterValue::Bool(value));
    }

    if let Ok(value) = input.parse::<i64>() {
        return Ok(ParameterValue::Integer(value));
    }

    if let Ok(value) = input.parse::<f64>() {
        return Ok(ParameterValue::Double(value));
    }

    Ok(ParameterValue::String(input.to_string()))
}

fn infer_array_parameter_value(values: Vec<Value>) -> Result<ParameterValue> {
    if values.is_empty() {
        return Ok(ParameterValue::StringArray(Vec::new()));
    }

    if values.iter().all(Value::is_boolean) {
        return Ok(ParameterValue::BoolArray(
            values
                .into_iter()
                .map(|value| value.as_bool().expect("checked above"))
                .collect(),
        ));
    }

    if values.iter().all(|value| value.as_i64().is_some()) {
        return Ok(ParameterValue::IntegerArray(
            values
                .into_iter()
                .map(|value| value.as_i64().expect("checked above"))
                .collect(),
        ));
    }

    if values.iter().all(|value| value.as_f64().is_some()) {
        return Ok(ParameterValue::DoubleArray(
            values
                .into_iter()
                .map(|value| value.as_f64().expect("checked above"))
                .collect(),
        ));
    }

    if values.iter().all(Value::is_string) {
        return Ok(ParameterValue::StringArray(
            values
                .into_iter()
                .map(|value| value.as_str().expect("checked above").to_string())
                .collect(),
        ));
    }

    bail!("array values must be homogeneous booleans, numbers, or strings")
}

fn parse_json_array(input: &str) -> Result<Vec<Value>> {
    let value: Value = serde_json::from_str(input)?;
    match value {
        Value::Array(values) => Ok(values),
        _ => bail!("expected JSON array"),
    }
}

fn parse_bool(input: &str) -> Result<bool> {
    match input {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => bail!("expected 'true' or 'false'"),
    }
}

fn parse_bool_array(input: &str) -> Result<Vec<bool>> {
    parse_json_array(input)?
        .into_iter()
        .map(|value| match value {
            Value::Bool(value) => Ok(value),
            _ => bail!("bool arrays must use JSON booleans, e.g. [true,false]"),
        })
        .collect()
}

fn parse_integer_array(input: &str) -> Result<Vec<i64>> {
    parse_json_array(input)?
        .into_iter()
        .map(|value| match value.as_i64() {
            Some(value) => Ok(value),
            None => bail!("integer arrays must use JSON integers, e.g. [1,2,3]"),
        })
        .collect()
}

fn parse_double_array(input: &str) -> Result<Vec<f64>> {
    parse_json_array(input)?
        .into_iter()
        .map(|value| match value.as_f64() {
            Some(value) => Ok(value),
            None => bail!("double arrays must use JSON numbers, e.g. [1.0,2.5]"),
        })
        .collect()
}

fn parse_string_array(input: &str) -> Result<Vec<String>> {
    parse_json_array(input)?
        .into_iter()
        .map(|value| match value {
            Value::String(value) => Ok(value),
            _ => bail!("string arrays must use JSON strings, e.g. [\"a\",\"b\"]"),
        })
        .collect()
}

fn parse_byte_array(input: &str) -> Result<Vec<u8>> {
    parse_json_array(input)?
        .into_iter()
        .map(|value| match value.as_u64() {
            Some(number) if u8::try_from(number).is_ok() => Ok(number as u8),
            _ => bail!("byte arrays must use JSON integers in the range 0..=255"),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use ros_z::parameter::ParameterValue;

    use super::parse_parameter_value;
    use crate::cli::ParameterValueTypeArg;

    #[test]
    fn infers_scalar_values() {
        assert_eq!(
            parse_parameter_value("true", None).unwrap(),
            ParameterValue::Bool(true)
        );
        assert_eq!(
            parse_parameter_value("42", None).unwrap(),
            ParameterValue::Integer(42)
        );
        assert_eq!(
            parse_parameter_value("3.5", None).unwrap(),
            ParameterValue::Double(3.5)
        );
        assert_eq!(
            parse_parameter_value("hello", None).unwrap(),
            ParameterValue::String("hello".to_string())
        );
    }

    #[test]
    fn infers_array_values() {
        assert_eq!(
            parse_parameter_value("[1,2,3]", None).unwrap(),
            ParameterValue::IntegerArray(vec![1, 2, 3])
        );
        assert_eq!(
            parse_parameter_value("[1.0,2.5]", None).unwrap(),
            ParameterValue::DoubleArray(vec![1.0, 2.5])
        );
        assert_eq!(
            parse_parameter_value(r#"["a","b"]"#, None).unwrap(),
            ParameterValue::StringArray(vec!["a".to_string(), "b".to_string()])
        );
    }

    #[test]
    fn honors_explicit_type_overrides() {
        assert_eq!(
            parse_parameter_value("001", Some(ParameterValueTypeArg::String)).unwrap(),
            ParameterValue::String("001".to_string())
        );
        assert_eq!(
            parse_parameter_value("[0,255]", Some(ParameterValueTypeArg::ByteArray)).unwrap(),
            ParameterValue::ByteArray(vec![0, 255])
        );
        assert_eq!(
            parse_parameter_value("ignored", Some(ParameterValueTypeArg::NotSet)).unwrap(),
            ParameterValue::NotSet
        );
    }
}
