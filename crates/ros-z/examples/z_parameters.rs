use std::collections::HashMap;

use clap::{Parser, ValueEnum};
use ros_z::{
    Builder, Result,
    context::ZContextBuilder,
    parameter::{
        FloatingPointRange, Parameter, ParameterDescriptor, ParameterType, ParameterValue,
        SetParametersResult,
    },
};

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Mode {
    /// Declare, get, and set parameters
    Declare,
    /// Register a validation callback that rejects out-of-range values
    Callback,
    /// Load parameters from a YAML file
    Yaml,
    /// Run all demos in sequence (default)
    All,
}

#[derive(Debug, Parser)]
#[command(about = "ros-z parameter demo")]
struct Args {
    /// Which demo to run
    #[arg(short, long, value_enum, default_value = "all")]
    mode: Mode,

    /// Zenoh router endpoint
    #[arg(short, long, default_value = "tcp/localhost:7447")]
    endpoint: String,
}

// ANCHOR: full_example
#[tokio::main]
async fn main() -> Result<()> {
    zenoh::init_log_from_env_or("error");
    let args = Args::parse();

    let ctx = ZContextBuilder::default()
        .with_connect_endpoints([&args.endpoint])
        .build()?;

    match args.mode {
        Mode::Declare => demo_declare(&ctx)?,
        Mode::Callback => demo_callback(&ctx)?,
        Mode::Yaml => demo_yaml(&ctx)?,
        Mode::All => {
            demo_declare(&ctx)?;
            demo_callback(&ctx)?;
            demo_yaml(&ctx)?;
        }
    }
    Ok(())
}
// ANCHOR_END: full_example

// ANCHOR: declare_snippet
/// Declare typed parameters, get/set them, and print the results.
fn demo_declare(ctx: &ros_z::context::ZContext) -> Result<()> {
    println!("\n=== Parameter Declaration Demo ===\n");

    let node = ctx.create_node("declare_demo").build()?;

    // Declare an integer parameter with a range constraint
    let mut desc = ParameterDescriptor::new("max_speed", ParameterType::Integer);
    desc.integer_range = Some(ros_z::parameter::IntegerRange {
        from_value: 0,
        to_value: 100,
        step: 1,
    });
    desc.description = "Maximum speed in m/s".to_string();

    let initial = node
        .declare_parameter("max_speed", ParameterValue::Integer(50), desc)
        .expect("declare max_speed");
    println!("Declared max_speed = {:?}", initial);

    // Declare a string parameter
    let desc = ParameterDescriptor::new("robot_name", ParameterType::String);
    node.declare_parameter("robot_name", ParameterValue::String("rosbot".into()), desc)
        .expect("declare robot_name");

    // Get parameter values
    println!("max_speed = {:?}", node.get_parameter("max_speed"));
    println!("robot_name = {:?}", node.get_parameter("robot_name"));

    // Set a new value
    node.set_parameter(Parameter::new("max_speed", ParameterValue::Integer(75)))
        .expect("set max_speed");
    println!(
        "max_speed after set = {:?}",
        node.get_parameter("max_speed")
    );

    // Attempting to set the wrong type fails
    let err = node
        .set_parameter(Parameter::new(
            "max_speed",
            ParameterValue::String("fast".into()),
        ))
        .unwrap_err();
    println!("Type mismatch rejected: {}", err);

    // Undeclare
    node.undeclare_parameter("robot_name")
        .expect("undeclare robot_name");
    println!(
        "robot_name after undeclare: {:?}",
        node.get_parameter("robot_name")
    );

    Ok(())
}
// ANCHOR_END: declare_snippet

// ANCHOR: callback_snippet
/// Register a validation callback that rejects values outside a range.
fn demo_callback(ctx: &ros_z::context::ZContext) -> Result<()> {
    println!("\n=== Validation Callback Demo ===\n");

    let node = ctx.create_node("callback_demo").build()?;

    let mut desc = ParameterDescriptor::new("temperature", ParameterType::Double);
    desc.floating_point_range = Some(FloatingPointRange {
        from_value: -40.0,
        to_value: 85.0,
        step: 0.0,
    });

    node.declare_parameter("temperature", ParameterValue::Double(20.0), desc)
        .expect("declare temperature");

    // Reject temperatures above 50.0 with a custom callback
    node.on_set_parameters(|params| {
        for p in params {
            if let ParameterValue::Double(v) = &p.value
                && *v > 50.0
            {
                return SetParametersResult::failure(format!(
                    "{} = {} exceeds safety limit 50.0",
                    p.name, v
                ));
            }
        }
        SetParametersResult::success()
    });

    // Valid change
    node.set_parameter(Parameter::new("temperature", ParameterValue::Double(25.0)))
        .expect("set to 25.0");
    println!("temperature = {:?}", node.get_parameter("temperature"));

    // Rejected by callback
    let err = node
        .set_parameter(Parameter::new("temperature", ParameterValue::Double(60.0)))
        .unwrap_err();
    println!("Callback rejected: {}", err);

    // Value unchanged
    println!(
        "temperature unchanged = {:?}",
        node.get_parameter("temperature")
    );

    Ok(())
}
// ANCHOR_END: callback_snippet

/// Load parameters from a YAML string via a temp file.
fn demo_yaml(ctx: &ros_z::context::ZContext) -> Result<()> {
    println!("\n=== YAML Parameter Loading Demo ===\n");

    let yaml = r#"
/**:
  ros__parameters:
    global_timeout: 5.0
    debug: true

/yaml_demo:
  ros__parameters:
    sensor_rate: 100
    device_name: "lidar_front"
"#;

    // Write YAML to a temp file
    let path = std::env::temp_dir().join("ros_z_param_demo.yaml");
    std::fs::write(&path, yaml).expect("write yaml");
    println!("YAML written to {}", path.display());

    // Build node with parameter file
    let node = ctx
        .create_node("yaml_demo")
        .with_parameter_file(&path)
        .expect("parse yaml")
        .build()?;

    // Declare parameters — overrides from file are applied automatically
    for (name, ty, default) in [
        (
            "global_timeout",
            ParameterType::Double,
            ParameterValue::Double(1.0),
        ),
        ("debug", ParameterType::Bool, ParameterValue::Bool(false)),
        (
            "sensor_rate",
            ParameterType::Integer,
            ParameterValue::Integer(10),
        ),
        (
            "device_name",
            ParameterType::String,
            ParameterValue::String("unknown".into()),
        ),
    ] {
        let desc = ParameterDescriptor::new(name, ty);
        let value = node
            .declare_parameter(name, default, desc)
            .expect("declare");
        println!("{} = {:?}", name, value);
    }

    // Also demonstrate programmatic overrides
    println!("\n--- Programmatic overrides ---");

    let mut overrides = HashMap::new();
    overrides.insert("count".to_string(), ParameterValue::Integer(99));

    let node2 = ctx
        .create_node("override_demo")
        .with_parameter_overrides(overrides)
        .build()?;

    let desc = ParameterDescriptor::new("count", ParameterType::Integer);
    let value = node2
        .declare_parameter("count", ParameterValue::Integer(0), desc)
        .expect("declare");
    println!("count = {:?} (default was 0, override is 99)", value);

    Ok(())
}
