extern crate nalgebra as na;
extern crate rerun as rr;

use tracing_appender::non_blocking::WorkerGuard;

use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_infra::rbt_err::RbtResult;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_mod::rbt_detector::pipeline;
use lib::rbt_mod::rbt_estimator::RbtEstimator;
use lib::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyId;
use lib::rbt_mod::rbt_solver::enemys_solver;

struct AutoAimHandle {
    pub cfg: RbtCfg,
    pub rec: rr::RecordingStream,
    _logger_guard: Option<WorkerGuard>,
}

/// 执行所有 init 步骤
async fn auto_aim_init() -> RbtResult<AutoAimHandle> {
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init().await?;
    let rec = rr::RecordingStreamBuilder::new("AutoAim").save("rerun-log/test.rrd")?;
    // let rec = rr::RecordingStreamBuilder::new("AutoAim").spawn()?;
    let enemy_fraction = cfg.game_cfg.enemy_fraction().unwrap();

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
    let mut auto_aim_handle = auto_aim_init().await?;

    loop {
        // 1. 执行 detector，使用神经网络模型，寻找所有的装甲板
        let detector_result = pipeline(&auto_aim_handle.cfg.detector_cfg)?;

        // 2. 执行 solver
        // 获取相机内参
        let cam_k = auto_aim_handle.cfg.cam_cfg.cam_k();
        // 解算检测到的所有装甲板，得到所有地方单位的解算结果
        let enemys = enemys_solver(detector_result, &cam_k, &auto_aim_handle.rec)?;

        // 3. 执行 estimator
        // 创建对 3 号步兵的估计器
        let mut estimator = RbtEstimator::new(EnemyId::Infantry3);
        // 从全部解算结果中获取 3 号步兵的解算结果
        let target_enemy_solved_result = enemys.get(&estimator.enemy_id).unwrap();
        // 基于解算结果，更新估计器
        estimator.update(
            &auto_aim_handle.cfg.estimator_cfg,
            target_enemy_solved_result,
        );
    }
}
