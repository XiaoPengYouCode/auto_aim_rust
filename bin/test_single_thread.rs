use auto_aim_rust::config::RobotConfig;
use auto_aim_rust::module::armor_detector;
use auto_aim_rust::module::robot_solver;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt().init();

    let robot_config = RobotConfig::read_from_toml().await?;

    // Stream log data to an awaiting `rerun` process.
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_app").connect_grpc()?;

    let armors = armor_detector::pipeline(&rec, robot_config.armor_detect_model_path.into())?;
    // let armor_class =
    //     armor_num_classify::pipeline(&rec, robot_config.armor_num_classify_model_path.into());
    // let robot_solver = robot_solver::RobotSolver::from_armor_static_msg(armors)?;
    // tracing::info!("robot_solver: {:?}", robot_solver.armor());

    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    Ok(())
}
