use thiserror::Error;

#[derive(Error, Debug)]
pub enum RbtError {
    // 设备相关错误
    #[error("No camera available")]
    NoCamera,

    #[error("No serial available")]
    NoSerial,

    #[error("No Usb available")]
    NoUsb,

    // AI/模型相关错误
    #[error("Ort error: {0}")]
    OrtError(#[from] ort::error::Error),

    // 配置相关错误
    #[error("Toml parse error: {0}")]
    TomlParseError(#[from] toml::de::Error),

    #[error("Invalid config: {0}")]
    InvalidConfig(String),

    // IO相关错误
    /// 注意 tokio::io::Error 本质是 std::io::Error
    /// 报错之后需要确认到底是同步接口的错，还是异步接口的错
    #[error("Tokio error: {0}")]
    TokioIoError(#[from] std::io::Error),

    // 日志相关错误
    #[error("Tracing subscriber env filter parse error: {0}")]
    TracingSubscriberEnvFilterParseError(#[from] tracing_subscriber::filter::ParseError),

    // 同步相关错误
    #[error("Failed to lock mutex: {0}")]
    LockMutexError(String),

    // 数据处理相关错误
    #[error("Frame id not found in start_tims")]
    FrameIdNotFound(u64),

    #[error("Failed to get camera frame: {0}")]
    InvalidArmorClassIndex(usize),

    #[error("Cal yaw angle under other coordinate")]
    CalAngleDisUnderOtherCoord,

    // 执行相关错误
    #[error("Unsupported execution provider: {0}")]
    UnsupportedExecutionProvider(String),

    // 可视化相关错误
    #[error("Rerun Recording Stream Error: {0}")]
    RerunRecordingStreamError(#[from] rr::RecordingStreamError),

    // 通讯相关错误
    #[error("Communication error: {0}")]
    CommError(#[from] CommError),

    // 通用错误
    #[error("Some Other Error with message: {0}")]
    StringError(String),
}

/// 自定义 result 类型，简化函数签名
pub type RbtResult<T> = Result<T, RbtError>;

impl From<String> for RbtError {
    fn from(err: String) -> Self {
        RbtError::StringError(err)
    }
}

/// 定义通讯相关的错误
#[derive(thiserror::Error, Debug)]
pub enum CommError {
    #[error("串口被占用: 检查是否开启了多个程序")]
    PortOccupied,
    #[error("IoError")]
    IoError,
    #[error("CorruptedFrame")]
    CorruptedFrame,
    #[error("TimeOut")]
    TimeOut,
    #[error("找不到串口")]
    NoPort,
    #[error("SystemError")]
    SystemError,
}
