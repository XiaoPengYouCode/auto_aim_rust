use tracing::info;
use tracing_appender::non_blocking::WorkerGuard;

use crate::rbt_err::RbtResult;
use crate::rbt_infra::rbt_cfg::RbtCfg;
use crate::rbt_mod::rbt_detector;
// use crate::rbt_mod::rbt_ctrl;
use crate::rbt_infra::rbt_log;
// use crate::rbt_mod::rbt_solver;

pub struct RbtSingleThreadApp {
    rbt_cfg: RbtCfg,
    _logger_guard: Option<WorkerGuard>,
}

impl RbtSingleThreadApp {
    pub async fn new() -> RbtResult<Self> {
        // read config file
        let rbt_cfg = RbtCfg::from_toml_async().await?;

        // init logger
        let _logger_guard = rbt_log::logger_init().await?;

        Ok(RbtSingleThreadApp {
            rbt_cfg,
            _logger_guard,
        })
    }

    pub fn run(&self) -> RbtResult<()> {
        // armor detect using Yolo
        let armors = rbt_detector::pipeline(&self.rbt_cfg.detector_cfg)?;
        info!("detect_armors_num: {}", armors.len());
        for armor in &armors {
            info!("armor: {:?}", armor);
        }

        // rbt_solver
        // let mut rbt_solver = rbt_solver::RbtSolver::new()?;
        // let _rbt_solver_result =
        //     rbt_solver.solve_pnp(&self.rbt_cfg.camera_cfg, &img_dbg, &armors)?;

        info!("RbtApp run finished");
        Ok(())
    }
}
