use lazy_static::lazy_static;
use std::sync::RwLock;
use std::sync::atomic::{AtomicBool, AtomicU32};

use crate::rbt_infra::rbt_cfg::RbtCfg;

pub static IS_RUNNING: AtomicBool = AtomicBool::new(true);
pub static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);
pub static FAILED_COUNT: AtomicU32 = AtomicU32::new(0);

lazy_static! {
    pub static ref GENERIC_RBT_CFG: RwLock<RbtCfg> = {
        let cfg = { RbtCfg::from_toml().unwrap_or_default() };
        RwLock::new(cfg)
    };
}
