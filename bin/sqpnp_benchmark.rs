use auto_aim_rust::rbt_base::rbt_math::sqpnp::sqpnp_def::{Point, Projection};
use auto_aim_rust::rbt_base::rbt_math::sqpnp::PnpSolver;

use nalgebra as na;

#[tokio::main]
async fn main() -> auto_aim_rust::rbt_err::RbtResult<()> {
    let _logger_guard = auto_aim_rust::rbt_infra::rbt_log::logger_init().await?;    

    let k = na::SMatrix::<f64, 3, 3>::new(1600.0, 0.0, 320.0, 0.0, 1705.7, 192.0, 0.0, 0.0, 1.0);

    let points = vec![
        Point {
            vector: na::Vector3::new(-135.0 / 2.0, 30.0, 0.01),
        },
        Point {
            vector: na::Vector3::new(-135.0 / 2.0, -30.0, 0.0),
        },
        Point {
            vector: na::Vector3::new(135.0 / 2.0, -30.0, 0.01),
        },
        Point {
            vector: na::Vector3::new(135.0 / 2.0, 30.0, 0.01),
        },
    ];

    let mut projections = vec![
        Projection {
            vector: na::Vector2::new(197.125, 203.125), // 左上点
        },
        Projection {
            vector: na::Vector2::new(191.25, 231.625), // 左下点
        },
        Projection {
            vector: na::Vector2::new(235.875, 236.375), // 右下点
        },
        Projection {
            vector: na::Vector2::new(241.5, 207.375), // 右上点
        },
    ];

    adjust_coordinates(k, &mut projections);

    let tim = tokio::time::Instant::now();
    let mut pnp_solver = PnpSolver::new(points, projections, None).unwrap();
    pnp_solver.solve().unwrap();
    tracing::info!("time used: {} us", tim.elapsed().as_micros());
    tracing::info!("Number of solutions found: {}", pnp_solver.solutions.len());

    // find the solution which has the minist sq error
    let best_solution = pnp_solver
        .solutions
        .iter()
        .min_by(|a, b| {
            a.sq_error
                .partial_cmp(&b.sq_error)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap();
    tracing::info!("best_solution = {}", best_solution.r);

    let rotation_matrix = na::Matrix3::from_column_slice(best_solution.r_hat.as_slice());
    let rotation = na::Rotation3::from_matrix(&rotation_matrix); // 提取欧拉角 (roll, pitch, yaw)
    let euler = rotation.euler_angles();

    tracing::info!(
        "eular_angle: roll={}, pitch={}, yaw={}",
        euler.0.to_degrees(),
        euler.1.to_degrees(),
        euler.2.to_degrees()
    );

    let translation =
        na::Translation3::new(best_solution.t[0], best_solution.t[1], best_solution.t[2]);
    tracing::info!(
        "Translation: x={}, y={}, z={}",
        translation.x, translation.y, translation.z
    );

    Ok(())
}

/// 使用内参归一化图像坐标系点到相机坐标系点
fn adjust_coordinates(k: na::SMatrix<f64, 3, 3>, projections: &mut Vec<Projection>) {
    for projection in projections.iter_mut() {
        let adjusted = k.try_inverse().unwrap()
            * na::Vector3::new(projection.vector.x, projection.vector.y, 1.0);
        projection.vector = na::Vector2::new(adjusted.x / adjusted.z, adjusted.y / adjusted.z);
    }
}
