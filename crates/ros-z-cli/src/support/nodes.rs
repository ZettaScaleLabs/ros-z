use std::collections::BTreeSet;

use color_eyre::eyre::{Result, bail, eyre};
use ros_z::parameter::ParameterTarget;

pub fn resolve_node_target(graph: &ros_z::graph::Graph, selector: &str) -> Result<ParameterTarget> {
    if selector.starts_with('/') {
        let target = ParameterTarget::from_fqn(selector)
            .ok_or_else(|| eyre!("invalid fully-qualified node name: {selector}"))?;
        if node_candidates(graph)
            .iter()
            .any(|candidate| candidate == &target)
        {
            return Ok(target);
        }
        bail!("node not found: {selector}");
    }

    let matches: Vec<_> = node_candidates(graph)
        .into_iter()
        .filter(|candidate| candidate.name == selector)
        .collect();

    match matches.as_slice() {
        [] => bail!("node not found: {selector}"),
        [target] => Ok(target.clone()),
        _ => bail!(
            "node name '{selector}' is ambiguous: {}",
            matches
                .iter()
                .map(ParameterTarget::fully_qualified_name)
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

pub fn can_resolve_node_target(graph: &ros_z::graph::Graph, selector: &str) -> bool {
    resolve_node_target(graph, selector).is_ok()
}

pub fn graph_node_key(target: &ParameterTarget) -> (String, String) {
    (
        normalize_node_namespace(&target.namespace),
        target.name.clone(),
    )
}

pub fn fully_qualified_node_name(namespace: &str, name: &str) -> String {
    if namespace == "/" {
        format!("/{name}")
    } else {
        format!("{namespace}/{name}")
    }
}

fn node_candidates(graph: &ros_z::graph::Graph) -> Vec<ParameterTarget> {
    let mut nodes = BTreeSet::new();
    for (name, namespace) in graph.get_node_names() {
        nodes.insert(ParameterTarget::new(namespace, name));
    }
    nodes.into_iter().collect()
}

fn normalize_node_namespace(namespace: &str) -> String {
    if namespace.is_empty() || namespace == "/" {
        String::new()
    } else {
        namespace.to_string()
    }
}
