use thiserror::Error;

#[derive(Error, Debug)]
pub enum RbtError {
    #[error("No camera available")]
    NoCamera,
    #[error("No serial available")]
    NoSerial,
    #[error("No Usb available")]
    NoUsb,
}

pub type RbtResult<T> = Result<T, RbtError>;
