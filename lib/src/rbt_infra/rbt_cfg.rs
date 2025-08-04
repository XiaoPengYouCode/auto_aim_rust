use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::rbt_bail_error;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_estimator::rbt_enemy_model::EnemyFaction;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GameCfg {
    enemy_fraction: String,
}

impl GameCfg {
    pub fn enemy_fraction(&self) -> Option<EnemyFaction> {
        if self.enemy_fraction.trim() == "B" {
            Some(EnemyFaction::B)
        } else if self.enemy_fraction.trim() == "R" {
            Some(EnemyFaction::R)
        } else {
            eprintln!("请检查 game_cfg/enemy_fraction 设置");
            None
        }
    }

    pub fn self_fraction(&self) -> Option<EnemyFaction> {
        if let Some(fraction) = self.enemy_fraction() {
            match fraction {
                EnemyFaction::B => Some(EnemyFaction::R),
                EnemyFaction::R => Some(EnemyFaction::B),
            }
        } else {
            None
        }
    }

    pub fn is_blue(&self) -> bool {
        if self.enemy_fraction.trim() == "blue" {
            true
        } else {
            false
        }
    }
}

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
pub struct EstimatorCfg {
    armor_lost_wait_duration_ms: u64,
    // top1_activate_w: f64,
    // top2_activate_w: f64,
}

impl EstimatorCfg {
    #[inline(always)]
    pub fn lost_wait_duration_ms(&self) -> tokio::time::Duration {
        tokio::time::Duration::from_millis(self.armor_lost_wait_duration_ms)
    }
}

/// 总配置
#[derive(Deserialize, Debug, Clone, PartialEq)]
pub struct RbtCfg {
    pub game_cfg: GameCfg,
    pub general_cfg: GeneralCfg,
    pub detector_cfg: DetectorCfg,
    pub cam_cfg: CamCfg,
    pub logger_cfg: LoggerCfg,
    pub estimator_cfg: EstimatorCfg,
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
