pub mod endpoints;
pub mod graph;
pub mod nodes;
pub mod param_parse;

use std::fmt::Display;

use color_eyre::eyre::{Report, eyre};

pub fn display_error<E: Display>(error: E) -> Report {
    eyre!("{error}")
}
