use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::rbt_bail_error;
use crate::rbt_err::{RbtError, RbtResult};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct LoggerCfg {
    pub console_log_filter: String,
    pub file_log_filter: String,
    pub console_log_enable: bool,
    pub file_log_enable: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GeneralCfg {
    pub img_dbg: bool,
    pub bullet_speed: f64,
}

// 检测器相关配置
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct DetectorCfg {
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

/// 相机相关配置
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CamCfg {
    cam_k: [f64; 9], // 设为私有，通过方法暴露
}

impl CamCfg {
    /// 将 [f64; 9] 数组转换成相机内参矩阵
    pub fn cam_k(&self) -> nalgebra::Matrix3<f64> {
        nalgebra::Matrix3::from_row_slice(&self.cam_k)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EstimatorConfig {
    top1_activate_w: f64,
    top2_activate_w: f64,
}


/// 总配置
#[derive(Deserialize, Debug, Clone, PartialEq)]
pub struct RbtCfg {
    pub general_cfg: GeneralCfg,
    pub detector_cfg: DetectorCfg,
    pub cam_cfg: CamCfg,
    pub logger_cfg: LoggerCfg,
}

impl RbtCfg {
    pub fn from_toml() -> RbtResult<Self> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("cfg")
            .join("rbt_cfg.toml");
        let cfg_str = std::fs::read_to_string(path)?;
        let cfg = toml::from_str::<Self>(&cfg_str)?;
        cfg.validation()?;
        Ok(cfg)
    }

    pub async fn from_toml_async() -> RbtResult<Self> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("cfg")
            .join("rbt_cfg.toml");
        let cfg_str = tokio::fs::read_to_string(path).await?;
        let cfg = toml::from_str::<Self>(&cfg_str)?;
        cfg.validation()?;
        Ok(cfg)
    }

    // 参数正确性校验
    pub fn validation(&self) -> RbtResult<()> {
        if self.general_cfg.bullet_speed > 25.0 {
            rbt_bail_error!(RbtError::InvalidConfig(
                format!("Bullet speed = {} > 25.0", self.general_cfg.bullet_speed).to_string()
            ));
        }
        Ok(())
    }
}

impl Default for RbtCfg {
    fn default() -> Self {
        tracing::warn!("Failed to find cfg file, use default cfg instead");
        RbtCfg {
            general_cfg: GeneralCfg {
                img_dbg: false,
                bullet_speed: 23.0,
            },
            detector_cfg: DetectorCfg {
                armor_detect_model_path: "model/armor/best_fp16_norm.onnx".to_string(),
                armor_detect_engine_path: "/model/armor/".to_string(),
                buff_detect_model_path: "model/buff/best.onnx".to_string(),
                camera_img_width: 1280,
                camera_img_height: 720,
                infer_img_width: 640,
                infer_img_height: 360,
                infer_full_height: 384,
                confidence_threshold: 0.8,
                ort_ep: "OpenVINO".to_string(),
            },
            cam_cfg: CamCfg {
                cam_k: [1800.0, 0.0, 320.0, 0.0, 1800.0, 162.0, 0.0, 0.0, 1.0],
            },
            logger_cfg: LoggerCfg {
                console_log_filter: "info,ort=warn".to_string(),
                file_log_filter: "debug,ort=info".to_string(),
                console_log_enable: true,
                file_log_enable: true,
            },
        }
    }
}
