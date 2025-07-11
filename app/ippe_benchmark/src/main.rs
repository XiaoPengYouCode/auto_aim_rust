use nalgebra as na;
use rerun as rr;

use lib::rbt_base::rbt_ippe::{ARMOR_LIGHT_HEIGHT, ARMOR_LIGHT_WEIGHT, ArmorPnpSolver};
use std::error::Error;
use tracing::{debug, error, info};

fn main() -> Result<(), Box<dyn Error>> {
    let rec = rr::RecordingStreamBuilder::new("pnp_visualizer")
        // .save("test.rrd")?;
        .spawn()?;
    let timer = std::time::Instant::now();
    let pnp_solver = ArmorPnpSolver::new().ok_or("Failed to create ArmorPnpSolver Instant")?;

    // 相机内参 (示例，单位要和装甲板尺寸一致，这里是毫米)
    let k_matrix = na::Matrix3::new(1600.0, 0.0, 320.0, 0.0, 1705.7, 192.0, 0.0, 0.0, 1.0);

    // 假设的观测图像点 (像素单位)
    let image_points_0 = [
        na::Point2::new(197.125, 203.125),
        na::Point2::new(191.25, 231.625),
        na::Point2::new(235.875, 236.375),
        na::Point2::new(241.5, 207.375),
    ];

    let image_points_1 = [
        na::Point2::new(361.5, 241.5),
        na::Point2::new(366.25, 270.0),
        na::Point2::new(416.5, 268.0),
        na::Point2::new(411.5, 239.5),
    ];

    let armor_points = vec![image_points_0, image_points_1];

    for (idx, image_points) in armor_points.iter().enumerate() {
        // 调用求解器
        if let Some(pose) = pnp_solver.solve(&image_points, &k_matrix) {
            debug!("Translation Vector: {}", pose.translation.vector);
            debug!("Rotation Matrix: {}", pose.rotation.to_rotation_matrix());

            let armor_translation_rr = [
                pose.translation.vector.x as f32,
                pose.translation.vector.y as f32,
                pose.translation.vector.z as f32,
            ];
            let armor_rotation_q_rr = [
                pose.rotation.i as f32,
                pose.rotation.j as f32,
                pose.rotation.k as f32,
                pose.rotation.w as f32,
            ];

            let _distance = pose.translation.vector.norm();

            rec.log(
                format!("armor_{}", idx),
                &[
                    &rerun::Boxes3D::from_half_sizes([(
                        ARMOR_LIGHT_WEIGHT as f32 / 2.0,
                        (ARMOR_LIGHT_HEIGHT as f32 + 30.0) / 2.0,
                        10.0,
                    )])
                    .with_fill_mode(rerun::FillMode::Solid)
                    .with_colors([rr::Color::from_unmultiplied_rgba(20, 20, 240, 100)])
                        as &dyn rerun::AsComponents,
                    &rerun::Transform3D::default()
                        .with_axis_length(100.0)
                        .with_translation(armor_translation_rr)
                        .with_rotation(rr::Rotation3D::Quaternion(
                            rr::components::RotationQuat::from(armor_rotation_q_rr),
                        )),
                ],
            )
            .unwrap();
        } else {
            error!("❌ PnP solving failed!");
        }
    }
    info!(
        "armor_pnp_solver time_used: {}",
        timer.elapsed().as_micros()
    );

    rec.log(
        "base_link",
        &rerun::Transform3D::default().with_axis_length(100.0),
    )?;

    Ok(())
}
