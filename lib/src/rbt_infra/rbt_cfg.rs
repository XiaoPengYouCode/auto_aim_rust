use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::rbt_bail_error;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_comm::rbt_comm_frame::TaskMode;
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyFaction;

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
        self.enemy_fraction.trim() == "blue"
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
    #[serde(default = "default_can_interface")]
    pub can_interface: String,
    #[serde(default = "default_can_enabled")]
    pub can_enabled: bool,
    #[serde(default = "default_offline_task_mode")]
    pub offline_task_mode: String,
}

fn default_can_interface() -> String {
    "can0".to_string()
}

fn default_can_enabled() -> bool {
    true
}

fn default_offline_task_mode() -> String {
    "AutoShot".to_string()
}

impl GeneralCfg {
    pub fn offline_task_mode(&self) -> TaskMode {
        match self.offline_task_mode.trim() {
            "AutoShot" => TaskMode::AutoShot,
            "HitBigBuff" => TaskMode::HitBigBuff,
            "HitSmallBuff" => TaskMode::HitSmallBuff,
            "HitOutpost" => TaskMode::HitOutpost,
            invalid => {
                eprintln!(
                    "请检查 general_cfg/offline_task_mode 设置，当前值 `{invalid}` 无效，回退到 AutoShot"
                );
                TaskMode::AutoShot
            }
        }
    }
}

// 检测器相关配置
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct DetectorCfg {
    pub camera_img_width: u64,
    pub camera_img_height: u64,
    pub infer_img_width: u64,
    pub infer_img_height: u64,
    pub ort_ep: String,
    pub armor: ArmorDetectorCfg,
    pub energy_mechanism: EnergyMechanismDetectorCfg,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ArmorDetectorCfg {
    pub model_path: String,
    pub engine_path: String,
    pub score_threshold: f32,
    pub confidence_threshold: f32,
    pub nms_iou_threshold: f32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EnergyMechanismDetectorCfg {
    pub model_path: String,
    pub engine_path: String,
    pub confidence_threshold: f32,
    pub nms_iou_threshold: f32,
    #[serde(default = "default_energy_target_switch_missing_timeout_s")]
    pub target_switch_missing_timeout_s: f64,
    #[serde(default = "default_energy_lost_timeout_s")]
    pub lost_timeout_s: f64,
    #[serde(default = "default_energy_fire_gap_s")]
    pub fire_gap_s: f64,
    #[serde(default)]
    pub yaw_offset_deg: f64,
    #[serde(default)]
    pub pitch_offset_deg: f64,
    #[serde(default)]
    pub predict_time_s: f64,
    #[serde(default)]
    pub pitch_velocity_lead_time_s: f64,
}

fn default_energy_target_switch_missing_timeout_s() -> f64 {
    0.2
}

fn default_energy_lost_timeout_s() -> f64 {
    0.35
}

fn default_energy_fire_gap_s() -> f64 {
    0.2
}

/// 能量机关顶层配置：tracker / aimer / mpc 三段。
///
/// 检测相关参数继续放在 `EnergyMechanismDetectorCfg`；这里只覆盖跟踪、瞄准与控制。
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Default)]
pub struct EnergyMechanismCfg {
    #[serde(default)]
    pub tracker: EnergyMechanismTrackerCfg,
    #[serde(default)]
    pub aimer: EnergyMechanismAimerCfg,
    #[serde(default)]
    pub mpc: EnergyMechanismMpcCfg,
}

/// 能量机关 tracker 配置（小符常速 + 大符曲线 EKF）。
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EnergyMechanismTrackerCfg {
    #[serde(default = "default_tracker_lost_timeout_s")]
    pub lost_timeout_s: f64,
    #[serde(default = "default_tracker_big_lost_timeout_s")]
    pub big_lost_timeout_s: f64,
    #[serde(default = "default_tracker_big_model_reset_timeout_s")]
    pub big_model_reset_timeout_s: f64,
    #[serde(default = "default_tracker_big_curve_ekf_fit_enabled")]
    pub big_curve_ekf_fit_enabled: bool,
    #[serde(default = "default_tracker_big_phase_process_noise")]
    pub big_phase_process_noise: f64,
    #[serde(default = "default_tracker_big_a_process_noise")]
    pub big_a_process_noise: f64,
    #[serde(default = "default_tracker_big_w_process_noise")]
    pub big_w_process_noise: f64,
    #[serde(default = "default_tracker_big_measurement_noise_scale")]
    pub big_measurement_noise_scale: f64,
    #[serde(default = "default_tracker_big_speed_measurement_enabled")]
    pub big_speed_measurement_enabled: bool,
    #[serde(default = "default_tracker_big_speed_measurement_noise")]
    pub big_speed_measurement_noise: f64,
    #[serde(default = "default_tracker_big_speed_measurement_gate")]
    pub big_speed_measurement_gate: f64,
    #[serde(default = "default_tracker_big_curve_speed_slew_limit")]
    pub big_curve_speed_slew_limit: f64,
    #[serde(default = "default_tracker_big_speed_measurement_window_samples")]
    pub big_speed_measurement_window_samples: usize,
    #[serde(default = "default_tracker_big_speed_measurement_window_s")]
    pub big_speed_measurement_window_s: f64,
    #[serde(default = "default_tracker_big_speed_measurement_min_history")]
    pub big_speed_measurement_min_history: usize,
    #[serde(default = "default_tracker_big_curve_phi_correction_limit")]
    pub big_curve_phi_correction_limit: f64,
    #[serde(default = "default_tracker_big_phi_seed_frames")]
    pub big_phi_seed_frames: usize,
}

impl Default for EnergyMechanismTrackerCfg {
    fn default() -> Self {
        Self {
            lost_timeout_s: default_tracker_lost_timeout_s(),
            big_lost_timeout_s: default_tracker_big_lost_timeout_s(),
            big_model_reset_timeout_s: default_tracker_big_model_reset_timeout_s(),
            big_curve_ekf_fit_enabled: default_tracker_big_curve_ekf_fit_enabled(),
            big_phase_process_noise: default_tracker_big_phase_process_noise(),
            big_a_process_noise: default_tracker_big_a_process_noise(),
            big_w_process_noise: default_tracker_big_w_process_noise(),
            big_measurement_noise_scale: default_tracker_big_measurement_noise_scale(),
            big_speed_measurement_enabled: default_tracker_big_speed_measurement_enabled(),
            big_speed_measurement_noise: default_tracker_big_speed_measurement_noise(),
            big_speed_measurement_gate: default_tracker_big_speed_measurement_gate(),
            big_curve_speed_slew_limit: default_tracker_big_curve_speed_slew_limit(),
            big_speed_measurement_window_samples:
                default_tracker_big_speed_measurement_window_samples(),
            big_speed_measurement_window_s: default_tracker_big_speed_measurement_window_s(),
            big_speed_measurement_min_history: default_tracker_big_speed_measurement_min_history(),
            big_curve_phi_correction_limit: default_tracker_big_curve_phi_correction_limit(),
            big_phi_seed_frames: default_tracker_big_phi_seed_frames(),
        }
    }
}

fn default_tracker_lost_timeout_s() -> f64 {
    0.35
}
fn default_tracker_big_lost_timeout_s() -> f64 {
    0.08
}
fn default_tracker_big_model_reset_timeout_s() -> f64 {
    0.35
}
fn default_tracker_big_curve_ekf_fit_enabled() -> bool {
    true
}
fn default_tracker_big_phase_process_noise() -> f64 {
    0.02
}
fn default_tracker_big_a_process_noise() -> f64 {
    1e-6
}
fn default_tracker_big_w_process_noise() -> f64 {
    3e-6
}
fn default_tracker_big_measurement_noise_scale() -> f64 {
    4.0
}
fn default_tracker_big_speed_measurement_enabled() -> bool {
    true
}
fn default_tracker_big_speed_measurement_noise() -> f64 {
    1.50
}
fn default_tracker_big_speed_measurement_gate() -> f64 {
    1.2
}
fn default_tracker_big_curve_speed_slew_limit() -> f64 {
    3.0
}
fn default_tracker_big_speed_measurement_window_samples() -> usize {
    16
}
fn default_tracker_big_speed_measurement_window_s() -> f64 {
    0.30
}
fn default_tracker_big_speed_measurement_min_history() -> usize {
    20
}
fn default_tracker_big_curve_phi_correction_limit() -> f64 {
    0.0
}
fn default_tracker_big_phi_seed_frames() -> usize {
    15
}

/// 能量机关 aimer 配置。
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EnergyMechanismAimerCfg {
    #[serde(default)]
    pub predict_time_s: f64,
    #[serde(default = "default_aimer_fire_gap_s")]
    pub fire_gap_s: f64,
    #[serde(default)]
    pub yaw_offset_deg: f64,
    #[serde(default)]
    pub pitch_offset_deg: f64,
    #[serde(default)]
    pub pitch_velocity_lead_time_s: f64,
    #[serde(default = "default_aimer_snapshot_stale_ms")]
    pub snapshot_stale_ms: f64,
}

impl Default for EnergyMechanismAimerCfg {
    fn default() -> Self {
        Self {
            predict_time_s: 0.0,
            fire_gap_s: default_aimer_fire_gap_s(),
            yaw_offset_deg: 0.0,
            pitch_offset_deg: 0.0,
            pitch_velocity_lead_time_s: 0.0,
            snapshot_stale_ms: default_aimer_snapshot_stale_ms(),
        }
    }
}

fn default_aimer_fire_gap_s() -> f64 {
    0.2
}
fn default_aimer_snapshot_stale_ms() -> f64 {
    180.0
}

/// 能量机关 MPC 配置（直接映射 `SecondOrderPositionMpcConfig` 关键字段）。
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EnergyMechanismMpcCfg {
    #[serde(default = "default_mpc_model_dt_s")]
    pub model_dt_s: f64,
    #[serde(default = "default_mpc_horizon")]
    pub horizon: usize,
    #[serde(default = "default_mpc_track_q")]
    pub track_q: f64,
    #[serde(default = "default_mpc_rate_q")]
    pub rate_q: f64,
    #[serde(default = "default_mpc_command_q")]
    pub command_q: f64,
    #[serde(default = "default_mpc_delta_r")]
    pub delta_r: f64,
}

impl Default for EnergyMechanismMpcCfg {
    fn default() -> Self {
        Self {
            model_dt_s: default_mpc_model_dt_s(),
            horizon: default_mpc_horizon(),
            track_q: default_mpc_track_q(),
            rate_q: default_mpc_rate_q(),
            command_q: default_mpc_command_q(),
            delta_r: default_mpc_delta_r(),
        }
    }
}

fn default_mpc_model_dt_s() -> f64 {
    0.004
}
fn default_mpc_horizon() -> usize {
    50
}
fn default_mpc_track_q() -> f64 {
    3_198.0
}
fn default_mpc_rate_q() -> f64 {
    0.0
}
fn default_mpc_command_q() -> f64 {
    1_000.0
}
fn default_mpc_delta_r() -> f64 {
    48_343.0
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
    enemy_lost_wait_duration_ms: u64,
    #[serde(default = "default_fire_block_on_armor_jump")]
    pub fire_block_on_armor_jump: bool,
    #[serde(default = "default_fire_armor_jump_block_frames")]
    pub fire_armor_jump_block_frames: usize,
    #[serde(default = "default_ypd_geometry_recovery_window_frames")]
    pub ypd_geometry_recovery_window_frames: usize,
    #[serde(default = "default_ypd_geometry_recovery_cooldown_frames")]
    pub ypd_geometry_recovery_cooldown_frames: usize,
    #[serde(default = "default_ypd_geometry_recovery_mismatch_required_streak")]
    pub ypd_geometry_recovery_mismatch_required_streak: usize,
    #[serde(default = "default_ypd_geometry_recovery_min_matched_count")]
    pub ypd_geometry_recovery_min_matched_count: usize,
    #[serde(default = "default_ypd_geometry_recovery_z_sigma_threshold")]
    pub ypd_geometry_recovery_z_sigma_threshold: f64,
    #[serde(default = "default_ypd_geometry_recovery_xy_sigma_threshold")]
    pub ypd_geometry_recovery_xy_sigma_threshold: f64,
    #[serde(default = "default_ypd_geometry_recovery_cov_inflation_scale")]
    pub ypd_geometry_recovery_cov_inflation_scale: f64,
    #[serde(default = "default_ypd_geometry_recovery_min_dr_variance")]
    pub ypd_geometry_recovery_min_dr_variance: f64,
    #[serde(default = "default_ypd_geometry_recovery_min_h_variance")]
    pub ypd_geometry_recovery_min_h_variance: f64,
    // top1_activate_w: f64,
    // top2_activate_w: f64,
}

fn default_fire_block_on_armor_jump() -> bool {
    true
}

fn default_fire_armor_jump_block_frames() -> usize {
    3
}

fn default_ypd_geometry_recovery_window_frames() -> usize {
    24
}

fn default_ypd_geometry_recovery_cooldown_frames() -> usize {
    12
}

fn default_ypd_geometry_recovery_mismatch_required_streak() -> usize {
    2
}

fn default_ypd_geometry_recovery_min_matched_count() -> usize {
    2
}

fn default_ypd_geometry_recovery_z_sigma_threshold() -> f64 {
    3.0
}

fn default_ypd_geometry_recovery_xy_sigma_threshold() -> f64 {
    2.0
}

fn default_ypd_geometry_recovery_cov_inflation_scale() -> f64 {
    48.0
}

fn default_ypd_geometry_recovery_min_dr_variance() -> f64 {
    2.5e-3
}

fn default_ypd_geometry_recovery_min_h_variance() -> f64 {
    6.25e-4
}

impl EstimatorCfg {
    #[inline(always)]
    pub fn lost_wait_duration_ms(&self) -> tokio::time::Duration {
        tokio::time::Duration::from_millis(self.armor_lost_wait_duration_ms)
    }

    #[inline(always)]
    pub fn enemy_lost_wait_duration_ms(&self) -> tokio::time::Duration {
        tokio::time::Duration::from_millis(self.enemy_lost_wait_duration_ms)
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
    #[serde(default)]
    pub energy_mechanism_cfg: EnergyMechanismCfg,
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
