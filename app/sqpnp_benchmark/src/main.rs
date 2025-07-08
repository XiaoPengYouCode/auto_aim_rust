use nalgebra as na;
use tracing::info;

use lib::rbt_base::rbt_math::sqpnp::PnpSolver;
use lib::rbt_err::RbtResult;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_base::rbt_cam_model::adjust_coordinates;
use lib::rbt_mod::rbt_armor::SMALL_ARMOR_POINT3;

#[tokio::main]
async fn main() -> RbtResult<()> {
    let _logger_guard = logger_init().await?;

    let k = na::SMatrix::<f64, 3, 3>::new(1600.0, 0.0, 320.0, 0.0, 1705.7, 192.0, 0.0, 0.0, 1.0);

    let points = SMALL_ARMOR_POINT3.to_vec();

    let mut projections = vec![
        na::Point2::new(197.125, 203.125), // 左上点
        na::Point2::new(191.25, 231.625), // 左下点
        na::Point2::new(235.875, 236.375), // 右下点
        na::Point2::new(241.5, 207.375), // 右上点
    ];

    adjust_coordinates(k, &mut projections);

    let tim = tokio::time::Instant::now();
    let mut pnp_solver = PnpSolver::new(points, projections, None).unwrap();
    pnp_solver.solve().unwrap();

    for (i, solution) in pnp_solver.solutions.iter().enumerate() {
        let rotation_matrix = na::Matrix3::from_row_slice(solution.r_hat.as_slice());
        info!("Rotation matrix: {:?}", rotation_matrix);

        let translation =
            na::Translation3::new(solution.t[0], solution.t[1], solution.t[2]);
        info!(
            "Translation: x={}, y={}, z={}",
            translation.x,
            translation.y,
            translation.z
        );
    }

    Ok(())
}

