pub mod json;
pub mod text;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    Text,
    Json,
}

impl OutputMode {
    pub const fn from_json_flag(json: bool) -> Self {
        if json { Self::Json } else { Self::Text }
    }

    pub const fn is_text(self) -> bool {
        matches!(self, Self::Text)
    }
}
