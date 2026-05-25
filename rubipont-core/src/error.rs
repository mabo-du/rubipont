use thiserror::Error;

#[derive(Debug, Error)]
pub enum RubipontError {
    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("Parse error in {format} at offset {offset}: {detail}")]
    ParseError {
        format: String,
        offset: u64,
        detail: String,
    },

    #[error("Corrupt chunk in {format} (chunk {chunk}): {detail}")]
    CorruptChunk {
        format: String,
        chunk: u64,
        detail: String,
    },

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Precision loss: {0}")]
    PrecisionLoss(String),
}

pub type Result<T> = std::result::Result<T, RubipontError>;
