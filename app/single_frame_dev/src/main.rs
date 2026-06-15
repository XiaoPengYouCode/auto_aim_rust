extern crate nalgebra as na;
extern crate rerun as rr;

use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_infra::rbt_err::RbtResult;
use lib::rbt_infra::rbt_log::{RbtLoggerGuard, logger_init};
use lib::rbt_mod::rbt_detector::pipeline;
use lib::rbt_mod::rbt_estimator::RbtHandlerPoll;
use lib::rbt_mod::rbt_solver::enemys_solver;
use std::path::Path;

struct AutoAimHandle {
    pub cfg: RbtCfg,
    pub rec: rr::RecordingStream,
    _logger_guard: Option<RbtLoggerGuard>,
}

/// 执行所有 init 步骤
async fn auto_aim_init() -> RbtResult<AutoAimHandle> {
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init()?;
    let rerun_path = Path::new("rerun-log").join("test.rrd");
    if let Some(parent) = rerun_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let rec = rr::RecordingStreamBuilder::new("AutoAim").save(rerun_path)?;
    // let rec = rr::RecordingStreamBuilder::new("AutoAim").spawn()?;
    let _enemy_fraction = cfg.game_cfg.enemy_fraction().unwrap();

    Ok(AutoAimHandle {
        cfg,
        rec,
        _logger_guard,
    })
}

/// 虽为 tokio 异步运行时
/// 但是该函数内所有代码都是同步执行
#[tokio::main]
async fn main() -> RbtResult<()> {
    // 0. 初始化
    let auto_aim_handle = auto_aim_init().await?;
    let mut estimator_poll = RbtHandlerPoll::new();

    loop {
        // 1. 执行 detector，使用神经网络模型，寻找所有的装甲板
        let detector_result = pipeline(&auto_aim_handle.cfg.detector_cfg)?;

        // 2. 执行 solver
        // 获取相机内参
        let cam_k = auto_aim_handle.cfg.cam_cfg.cam_k();
        // 解算检测到的所有装甲板，得到所有地方单位的解算结果
        let enemys = enemys_solver(detector_result, &cam_k, None, &auto_aim_handle.rec)?;

        // 3. 执行 estimator
        estimator_poll.update(&auto_aim_handle.cfg.estimator_cfg, enemys);
    }
}
