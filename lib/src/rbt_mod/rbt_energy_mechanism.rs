//! Energy mechanism detection, solving, tracking, and control.
//!
//! This module mirrors the Armour stack shape while keeping the implementation
//! Rust-native. Protocol names such as `HitBigBuff` are handled only at the
//! route/communication boundary.

pub mod detected;
pub mod fire_control;
pub mod solved;
pub mod tracker;

pub use detected::{
    ENERGY_MECHANISM_INPUT_HEIGHT, ENERGY_MECHANISM_INPUT_WIDTH, ENERGY_MECHANISM_KEYPOINTS,
    ENERGY_MECHANISM_OUTPUT_MAX_CHANNELS, EnergyMechanismClass, EnergyMechanismFrame,
    EnergyMechanismMode, EnergyMechanismObject, EnergyMechanismYoloDecodeStats,
    EnergyMechanismYoloPostprocessCfg, decode_energy_mechanism_output_with_stats,
    preprocess_energy_mechanism_letterbox_f32,
};
pub use fire_control::{
    EnergyMechanismControlInput, EnergyMechanismControlStats, EnergyMechanismController,
};
pub use solved::{
    EnergyMechanismPose, EnergyMechanismSolvedFrame, EnergyMechanismSolvedTarget,
    solve_energy_mechanism,
};
pub use tracker::{EnergyMechanismTrackSnapshot, EnergyMechanismTracker};
