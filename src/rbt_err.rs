use thiserror::Error;

#[derive(Error, Debug)]
pub enum RbtError {
    #[error("No camera available")]
    NoCamera,

    #[error("No serial available")]
    NoSerial,

    #[error("No Usb available")]
    NoUsb,

    #[error("Ort error: {0}")]
    OrtError(#[from] ort::error::Error),

    #[error("Rerun dbg error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),

    #[error("Toml parse error: {0}")]
    TomlParseError(#[from] toml::de::Error),

    #[error("Tokio error: {0}")]
    TokioIoError(#[from] tokio::io::Error),

    #[error("Tracing subscriber env filter parse error: {0}")]
    TracingSubscriberEnvFilterParseError(#[from] tracing_subscriber::filter::ParseError),

    #[error("Failed to lock mutex: {0}")]
    LockMutexError(String),

    #[error("Frame id not found in start_tims")]
    FrameIdNotFound(u64),

    #[error("Failed to get camera frame: {0}")]
    InvalidArmorClassIndex(usize),

    #[error("Unsupported execution provider: {0}")]
    UnsupportedExecutionProvider(String),
}

pub type RbtResult<T> = Result<T, RbtError>;
