use auto_aim_rust::config::RobotConfig;
use auto_aim_rust::module::armor_detector;
use auto_aim_rust::module::rbt_solver;

#[tokio::main]
async fn main() {
    single_thread().await.unwrap();
}

async fn single_thread() -> Result<(), Box<dyn std::error::Error>> {
    // read config file
    let robot_config = RobotConfig::read_from_toml().await?;

    // init logger
    let filter = tracing_subscriber::EnvFilter::new(robot_config.log_filter);
    tracing_subscriber::fmt().with_env_filter(filter).init();

    // for debug use
    // Stream log data to an awaiting `rerun` process.
    let dbg = if robot_config.img_dbg_open {
        let rec = rerun::RecordingStreamBuilder::new("rerun_example_app").connect_grpc()?;
        Some(rec)
    } else {
        None
    };

    // armor detect using Yolo
    let armors = armor_detector::pipeline(
        &dbg, // for debug_use
        robot_config.armor_detect_model_path.into(),
        robot_config.armor_detect_engine_path.into(),
    )?;

    let mut robot_solver = rbt_solver::RbtSolver::new()?;
    let rbt_solver_result = robot_solver.solve_pnp(&dbg, armors)?;
    tracing::info!("robot_solver: {:?}", rbt_solver_result);

    Ok(())
}
