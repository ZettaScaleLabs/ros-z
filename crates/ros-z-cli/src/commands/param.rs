use color_eyre::eyre::{Result, bail, eyre};
use ros_z::parameter::{Parameter, ParameterTarget};

use crate::{
    app::AppContext,
    cli::ParamCommand,
    model::param::{ParameterGetView, ParameterListView, ParameterSetView, ParameterValueView},
    render::{OutputMode, json, text},
    support::{
        display_error,
        nodes::{can_resolve_node_target, resolve_node_target},
        param_parse::parse_parameter_value,
    },
};

pub async fn run(app: &AppContext, output_mode: OutputMode, command: ParamCommand) -> Result<()> {
    match command {
        ParamCommand::List {
            node,
            prefix,
            depth,
        } => render_param_list(app, output_mode, &node, prefix, depth).await,
        ParamCommand::Get { name, node } => render_param_get(app, output_mode, &node, &name).await,
        ParamCommand::Set {
            name,
            value,
            node,
            value_type,
            atomic,
        } => render_param_set(app, output_mode, &node, &name, &value, value_type, atomic).await,
    }
}

async fn render_param_list(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    prefix: Vec<String>,
    depth: Option<u64>,
) -> Result<()> {
    let target = resolve_parameter_target(app, selector).await?;
    let client = app.parameter_client(target.clone())?;
    let response = client.list(&prefix, depth).await.map_err(display_error)?;
    let view = ParameterListView::new(
        target.fully_qualified_name(),
        response.names,
        response.prefixes,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_parameter_list(&view);
            Ok(())
        }
    }
}

async fn render_param_get(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    name: &str,
) -> Result<()> {
    let target = resolve_parameter_target(app, selector).await?;
    let client = app.parameter_client(target.clone())?;
    let values = client.get(&[name]).await.map_err(display_error)?;
    let value = values
        .into_iter()
        .next()
        .ok_or_else(|| eyre!("parameter not returned: {name}"))?;
    let view = ParameterGetView::new(
        target.fully_qualified_name(),
        name.to_string(),
        ParameterValueView::from_parameter_value(&value)?,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_parameter_get(&view);
            Ok(())
        }
    }
}

async fn render_param_set(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    name: &str,
    value: &str,
    value_type: Option<crate::cli::ParameterValueTypeArg>,
    atomic: bool,
) -> Result<()> {
    let target = resolve_parameter_target(app, selector).await?;
    let client = app.parameter_client(target.clone())?;
    let parameter_value = parse_parameter_value(value, value_type)?;
    let parameter = Parameter::new(name.to_string(), parameter_value.clone());

    if atomic {
        let result = client
            .set_atomically(&[parameter])
            .await
            .map_err(display_error)?;
        if !result.successful {
            bail!("parameter update rejected: {}", result.reason);
        }
    } else {
        let results = client.set(&[parameter]).await.map_err(display_error)?;
        let result = results
            .into_iter()
            .next()
            .ok_or_else(|| eyre!("missing set result for parameter {name}"))?;
        if !result.successful {
            bail!("parameter update rejected: {}", result.reason);
        }
    }

    let view = ParameterSetView::new(
        target.fully_qualified_name(),
        name.to_string(),
        ParameterValueView::from_parameter_value(&parameter_value)?,
        atomic,
        true,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_parameter_set(&view);
            Ok(())
        }
    }
}

async fn resolve_parameter_target(app: &AppContext, selector: &str) -> Result<ParameterTarget> {
    app.wait_for_graph_condition(|graph| can_resolve_node_target(graph, selector))
        .await;
    resolve_node_target(app.graph(), selector)
}
