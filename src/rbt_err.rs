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
    #[error("Rerun Dbg error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),
}

pub type RbtResult<T> = Result<T, RbtError>;
