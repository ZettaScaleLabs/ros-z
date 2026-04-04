use clap::{Parser, Subcommand, ValueEnum};

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq, Default)]
pub enum Backend {
    #[default]
    RmwZenoh,
    #[value(name = "ros2dds")]
    Ros2Dds,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum ListTarget {
    Topics,
    Nodes,
    Services,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum InfoTarget {
    Topic,
    Service,
    Node,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum ParameterValueTypeArg {
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
    NotSet,
}

#[derive(Debug, Parser)]
#[command(name = "rosz")]
#[command(about = "Scriptable command-line companion to ros-z")]
pub struct Cli {
    /// Zenoh router address
    #[arg(long, default_value = "tcp/127.0.0.1:7447", global = true)]
    pub router: String,

    /// ROS domain ID
    #[arg(long, default_value_t = 0, global = true)]
    pub domain: usize,

    /// Backend selection (rmw-zenoh or ros2dds)
    #[arg(long, value_enum, default_value = "rmw-zenoh", global = true)]
    pub backend: Backend,

    /// Emit JSON output when supported
    #[arg(long, global = true)]
    pub json: bool,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// List graph entities
    List {
        #[arg(value_enum)]
        target: ListTarget,
    },
    /// Watch graph changes continuously
    Watch,
    /// Show the full graph snapshot
    Graph,
    /// Dynamically inspect a topic's messages
    Echo {
        topic: String,
        #[arg(long)]
        count: Option<usize>,
        #[arg(long)]
        timeout: Option<f64>,
    },
    /// Show metadata for a topic, service, or node
    Info {
        #[arg(value_enum)]
        target: InfoTarget,
        name: String,
    },
    /// Remote parameter operations
    Param {
        #[command(subcommand)]
        command: ParamCommand,
    },
}

#[derive(Debug, Subcommand)]
pub enum ParamCommand {
    /// List parameters on a node
    List {
        #[arg(long)]
        node: String,
        #[arg(long)]
        prefix: Vec<String>,
        #[arg(long)]
        depth: Option<u64>,
    },
    /// Get a parameter from a node
    Get {
        name: String,
        #[arg(long)]
        node: String,
    },
    /// Set a parameter on a node
    Set {
        name: String,
        value: String,
        #[arg(long)]
        node: String,
        #[arg(long = "type", value_enum)]
        value_type: Option<ParameterValueTypeArg>,
        #[arg(long)]
        atomic: bool,
    },
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::{Backend, Cli, Command, ListTarget, ParamCommand, ParameterValueTypeArg};

    #[test]
    fn parses_echo_command_with_defaults() {
        let cli = Cli::parse_from(["rosz", "echo", "/chatter", "--count", "1"]);

        assert_eq!(cli.router, "tcp/127.0.0.1:7447");
        assert_eq!(cli.domain, 0);
        assert_eq!(cli.backend, Backend::RmwZenoh);
        assert!(!cli.json);

        match cli.command {
            Command::Echo {
                topic,
                count,
                timeout,
            } => {
                assert_eq!(topic, "/chatter");
                assert_eq!(count, Some(1));
                assert_eq!(timeout, None);
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_global_flags_after_subcommand() {
        let cli = Cli::parse_from([
            "rosz",
            "list",
            "topics",
            "--router",
            "tcp/192.168.1.10:7447",
            "--domain",
            "7",
            "--backend",
            "ros2dds",
            "--json",
        ]);

        assert_eq!(cli.router, "tcp/192.168.1.10:7447");
        assert_eq!(cli.domain, 7);
        assert_eq!(cli.backend, Backend::Ros2Dds);
        assert!(cli.json);

        match cli.command {
            Command::List { target } => assert_eq!(target, ListTarget::Topics),
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_param_set_with_type_override() {
        let cli = Cli::parse_from([
            "rosz",
            "param",
            "set",
            "max_speed",
            "42",
            "--node",
            "talker",
            "--type",
            "integer",
            "--atomic",
        ]);

        match cli.command {
            Command::Param { command } => match command {
                ParamCommand::Set {
                    name,
                    value,
                    node,
                    value_type,
                    atomic,
                } => {
                    assert_eq!(name, "max_speed");
                    assert_eq!(value, "42");
                    assert_eq!(node, "talker");
                    assert_eq!(value_type, Some(ParameterValueTypeArg::Integer));
                    assert!(atomic);
                }
                other => panic!("unexpected param command: {other:?}"),
            },
            other => panic!("unexpected command: {other:?}"),
        }
    }
}
