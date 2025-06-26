// 暂时不考虑实现 hot_reload
// 暂时没有思路

use serde::Deserialize;
use std::path::Path;

use crate::rbt_err::RbtResult;

#[derive(Deserialize, Debug, Clone)]
pub struct LoggerConfig {
    pub terminal_log_filter: String,
    pub file_log_filter: String,
    pub console_log_enable: bool,
    pub file_log_enable: bool,
}

#[derive(Deserialize, Debug, Clone)]
pub struct GeneralConfig {
    pub img_dbg: bool,
    pub bullet_speed: f64,
}

// 检测器相关配置
#[derive(Deserialize, Debug, Clone)]
pub struct DetectorConfig {
    pub armor_detect_model_path: String,
    pub armor_detect_engine_path: String,
    pub buff_detect_model_path: String,
    pub camera_img_width: u64,
    pub camera_img_height: u64,
    pub infer_img_width: u64,
    pub infer_img_height: u64,
    pub infer_full_height: u64,
    pub confidence_threshold: f32,
    pub ort_ep: String,
}

// 相机相关配置
#[derive(Deserialize, Debug, Clone)]
pub struct CameraConfig {
    camera_matrix: [f64; 9], // 设为私有，通过方法暴露
}

impl CameraConfig {
    pub fn cam_mtx(&self) -> nalgebra::Matrix3<f64> {
        nalgebra::Matrix3::from_row_slice(&self.camera_matrix)
    }
}

// 总配置
#[derive(Deserialize, Debug, Clone)]
pub struct RbtCfg {
    pub general_cfg: GeneralConfig,
    pub detector_cfg: DetectorConfig,
    pub camera_cfg: CameraConfig,
    pub logger_cfg: LoggerConfig,
}

impl RbtCfg {
    pub fn from_toml() -> RbtResult<Self> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("robot_config.toml");
        let param = std::fs::read_to_string(path)?;
        let param: Self = toml::from_str(&param)?;
        Ok(param)
    }

    pub async fn from_toml_async() -> RbtResult<Self> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("robot_config.toml");
        let param = tokio::fs::read_to_string(path).await?;
        let param: Self = toml::from_str(&param)?;
        Ok(param)
    }
}
