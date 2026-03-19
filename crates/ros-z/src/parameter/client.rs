//! Remote ROS 2 parameter client helpers.

use zenoh::Result;

use super::{
    Parameter, ParameterDescriptor, ParameterType, ParameterValue, SetParametersResult,
    wire_types::{
        DEPTH_RECURSIVE, DescribeParametersRequest, DescribeParametersSrv,
        GetParameterTypesRequest, GetParameterTypesSrv, GetParametersRequest, GetParametersSrv,
        ListParametersRequest, ListParametersSrv, SetParametersAtomicallyRequest,
        SetParametersAtomicallySrv, SetParametersRequest, SetParametersSrv,
    },
};
use crate::{Builder, node::ZNode, service::ZClient};

/// Target node for remote parameter operations.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ParameterTarget {
    pub namespace: String,
    pub name: String,
}

impl ParameterTarget {
    pub fn new(namespace: impl Into<String>, name: impl Into<String>) -> Self {
        let namespace = normalize_namespace(namespace.into());
        Self {
            namespace,
            name: name.into(),
        }
    }

    pub fn from_fqn(fqn: &str) -> Option<Self> {
        let fqn = fqn.trim();
        if !fqn.starts_with('/') {
            return None;
        }

        let mut parts = fqn.rsplitn(2, '/');
        let name = parts.next()?.to_string();
        let namespace = parts.next().unwrap_or("").to_string();
        Some(Self::new(namespace, name))
    }

    pub fn fully_qualified_name(&self) -> String {
        if self.namespace == "/" {
            format!("/{}", self.name)
        } else {
            format!("{}/{}", self.namespace, self.name)
        }
    }

    fn service_name(&self, suffix: &str) -> String {
        format!("{}/{}", self.fully_qualified_name(), suffix)
    }

    fn validate(&self) -> Result<()> {
        crate::topic_name::qualify_topic_name("parameter_target", &self.namespace, &self.name)
            .map(|_| ())
            .map_err(|err| {
                zenoh::Error::from(format!(
                    "Invalid parameter target '{}': {}",
                    self.fully_qualified_name(),
                    err
                ))
            })
    }
}

fn normalize_namespace(namespace: String) -> String {
    let namespace = namespace.trim();
    if namespace.is_empty() || namespace == "/" {
        "/".to_string()
    } else if namespace.starts_with('/') {
        namespace.to_string()
    } else {
        format!("/{namespace}")
    }
}

/// Result of `list_parameters`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ParameterList {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

/// Builder for creating a high-level remote parameter client.
#[derive(Debug)]
pub struct ParameterClientBuilder<'a> {
    pub(crate) node: &'a ZNode,
    pub(crate) target: ParameterTarget,
}

impl<'a> ParameterClientBuilder<'a> {
    pub fn new(node: &'a ZNode, target: ParameterTarget) -> Self {
        Self { node, target }
    }
}

impl Builder for ParameterClientBuilder<'_> {
    type Output = ParameterClient;

    fn build(self) -> Result<Self::Output> {
        self.target.validate()?;

        let describe_parameters = self
            .node
            .create_client::<DescribeParametersSrv>(
                &self.target.service_name("describe_parameters"),
            )
            .build()?;
        let get_parameters = self
            .node
            .create_client::<GetParametersSrv>(&self.target.service_name("get_parameters"))
            .build()?;
        let get_parameter_types = self
            .node
            .create_client::<GetParameterTypesSrv>(&self.target.service_name("get_parameter_types"))
            .build()?;
        let list_parameters = self
            .node
            .create_client::<ListParametersSrv>(&self.target.service_name("list_parameters"))
            .build()?;
        let set_parameters = self
            .node
            .create_client::<SetParametersSrv>(&self.target.service_name("set_parameters"))
            .build()?;
        let set_parameters_atomically = self
            .node
            .create_client::<SetParametersAtomicallySrv>(
                &self.target.service_name("set_parameters_atomically"),
            )
            .build()?;

        Ok(ParameterClient {
            target: self.target,
            describe_parameters,
            get_parameters,
            get_parameter_types,
            list_parameters,
            set_parameters,
            set_parameters_atomically,
        })
    }
}

/// High-level remote parameter client.
#[derive(Debug)]
pub struct ParameterClient {
    target: ParameterTarget,
    describe_parameters: ZClient<DescribeParametersSrv>,
    get_parameters: ZClient<GetParametersSrv>,
    get_parameter_types: ZClient<GetParameterTypesSrv>,
    list_parameters: ZClient<ListParametersSrv>,
    set_parameters: ZClient<SetParametersSrv>,
    set_parameters_atomically: ZClient<SetParametersAtomicallySrv>,
}

impl ParameterClient {
    /// Return the target node this client addresses.
    pub fn target(&self) -> &ParameterTarget {
        &self.target
    }

    /// Describe remote parameters by name.
    pub async fn describe(&self, names: &[impl AsRef<str>]) -> Result<Vec<ParameterDescriptor>> {
        self.describe_parameters
            .send_request(&DescribeParametersRequest {
                names: names.iter().map(|name| name.as_ref().to_string()).collect(),
            })
            .await?;
        let response = self.describe_parameters.take_response_async().await?;
        Ok(response
            .descriptors
            .iter()
            .map(ParameterDescriptor::from_wire)
            .collect())
    }

    /// Fetch remote parameter values by name.
    pub async fn get(&self, names: &[impl AsRef<str>]) -> Result<Vec<ParameterValue>> {
        self.get_parameters
            .send_request(&GetParametersRequest {
                names: names.iter().map(|name| name.as_ref().to_string()).collect(),
            })
            .await?;
        let response = self.get_parameters.take_response_async().await?;
        Ok(response
            .values
            .iter()
            .map(ParameterValue::from_wire)
            .collect())
    }

    /// Fetch remote parameter types by name.
    pub async fn get_types(&self, names: &[impl AsRef<str>]) -> Result<Vec<ParameterType>> {
        self.get_parameter_types
            .send_request(&GetParameterTypesRequest {
                names: names.iter().map(|name| name.as_ref().to_string()).collect(),
            })
            .await?;
        let response = self.get_parameter_types.take_response_async().await?;
        Ok(response
            .types
            .into_iter()
            .map(|type_id| ParameterType::try_from(type_id).unwrap_or(ParameterType::NotSet))
            .collect())
    }

    /// List remote parameters, optionally constrained by prefixes and depth.
    pub async fn list(
        &self,
        prefixes: &[impl AsRef<str>],
        depth: Option<u64>,
    ) -> Result<ParameterList> {
        self.list_parameters
            .send_request(&ListParametersRequest {
                prefixes: prefixes
                    .iter()
                    .map(|prefix| prefix.as_ref().to_string())
                    .collect(),
                depth: depth.unwrap_or(DEPTH_RECURSIVE),
            })
            .await?;
        let response = self.list_parameters.take_response_async().await?;
        Ok(ParameterList {
            names: response.result.names,
            prefixes: response.result.prefixes,
        })
    }

    /// Set one or more remote parameters non-atomically.
    pub async fn set(&self, parameters: &[Parameter]) -> Result<Vec<SetParametersResult>> {
        self.set_parameters
            .send_request(&SetParametersRequest {
                parameters: parameters.iter().map(Parameter::to_wire).collect(),
            })
            .await?;
        let response = self.set_parameters.take_response_async().await?;
        Ok(response
            .results
            .into_iter()
            .map(|result| SetParametersResult {
                successful: result.successful,
                reason: result.reason,
            })
            .collect())
    }

    /// Set remote parameters atomically.
    pub async fn set_atomically(&self, parameters: &[Parameter]) -> Result<SetParametersResult> {
        self.set_parameters_atomically
            .send_request(&SetParametersAtomicallyRequest {
                parameters: parameters.iter().map(Parameter::to_wire).collect(),
            })
            .await?;
        let response = self.set_parameters_atomically.take_response_async().await?;
        Ok(SetParametersResult {
            successful: response.result.successful,
            reason: response.result.reason,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::ParameterTarget;

    #[test]
    fn target_normalizes_namespace() {
        let target = ParameterTarget::new("robot", "nav");
        assert_eq!(target.namespace, "/robot");
        assert_eq!(target.fully_qualified_name(), "/robot/nav");
        assert_eq!(
            target.service_name("get_parameters"),
            "/robot/nav/get_parameters"
        );
    }

    #[test]
    fn target_from_fqn() {
        let target = ParameterTarget::from_fqn("/demo/controller").unwrap();
        assert_eq!(target.namespace, "/demo");
        assert_eq!(target.name, "controller");
    }
}
