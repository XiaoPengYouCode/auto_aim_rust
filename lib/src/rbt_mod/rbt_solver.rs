use crate::rbt_base::rbt_geometry::rbt_cylindrical3::RbtCylindricalPoint3;
use crate::rbt_base::rbt_geometry::rbt_pose3::{RbtPose3, RbtPoseCoordSys};
use crate::rbt_base::rbt_geometry::rbt_line2::{RbtLine2, find_intersection};
use crate::rbt_base::rbt_ippe::ArmorPnpSolver;
use crate::rbt_infra::rbt_cfg;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
use crate::rbt_mod::rbt_enemy::EnemyId;
use nalgebra::Vector2;
use std::collections::HashMap;
use tracing::{debug, error, info, warn};

pub mod rbt_orientation_solver;
// mod rbt_sqpnp;

#[derive(Debug)]
enum RbtClass {
    // Hero,
    Engineer,
    // Infantry,
    // Sentry,
}

// #[derive(Debug)]
// pub struct RbtMsg {
//     _class: RbtClass,
// }
//
// impl RbtMsg {
//     fn new(class: RbtClass) -> RbtMsg {
//         RbtMsg { _class: class }
//     }
// }

#[derive(Debug)]
pub struct RbtSolver {
    armors: Vec<DetectedArmor>,
}

impl RbtSolver {
    // 根据 ArmorStaticMsg, 构建 RobotSolver
    pub fn new() -> RbtResult<Self> {
        Ok(Self { armors: vec![] })
    }
}

/// enemys_solver全流程
pub fn enemys_solver(
    detector_result: HashMap<EnemyId, Vec<DetectedArmor>>,
    cam_k: &na::Matrix3<f64>,
    rec: &rr::RecordingStream,
) -> RbtResult<Vec<(EnemyId, RbtCylindricalPoint3)>> {
    let mut enemys = Vec::<(EnemyId, RbtCylindricalPoint3)>::with_capacity(6);
    for (enemy_id, enemy_armors) in detector_result.into_iter() {
        let detected_enemy_armors_num = enemy_armors.len();
        if detected_enemy_armors_num == 0 {
            continue; // 该帧没有看到该兵种的装甲板，提前退出
        }

        // 针对每一块装甲板求解pnp
        let mut pnp_results = Vec::with_capacity(detected_enemy_armors_num);
        for armor in enemy_armors {
            let armor_key_points = armor.cornet_points();
            let armor_key_points_na = armor_key_points.map(|p| p.into());

            let pnp_solver = ArmorPnpSolver::new().ok_or(RbtError::StringError(
                "Failed to create ArmorPnpSolver Instant".to_string(),
            ))?;
            if let Some(pose) = pnp_solver.solve(&armor_key_points_na, &cam_k) {
                pnp_results.push(pose);
            } else {
                error!("❌ PnP solving failed!")
            };
        }

        let mut armors_base_pose_xyz = vec![];
        for pose in pnp_results.iter() {
            let armor_cam_pose = RbtPose3::new(
                <na::Point3<f64>>::from(pose.translation.vector),
                pose.rotation.to_rotation_matrix(),
                RbtPoseCoordSys::Camera,
            );
            let armor_base_pose = armor_cam_pose.coord_trans(RbtPoseCoordSys::BaseXyz);
            armors_base_pose_xyz.push(armor_base_pose);
        }

        let armors_line_2d = armors_base_pose_xyz
            .iter()
            .map(|armor_pose| {
                let [armor_x, armor_y] = [armor_pose.translation.x, armor_pose.translation.y];
                let [armor_2d_pose_a, armor_2d_pose_b] = [
                    armor_pose.rotation.matrix().m13,
                    armor_pose.rotation.matrix().m23,
                ];
                RbtLine2 {
                    point: na::Point2::new(armor_x, armor_y),
                    direction: na::Vector2::new(armor_2d_pose_a, armor_2d_pose_b),
                }
            })
            .collect::<Vec<RbtLine2>>();

        let enemy_center_xy = solve_enemy_center(&armors_line_2d)
            .ok_or(RbtError::StringError("Failed to solve enemy center".into()))?;

        let armor_base_cylindrical = RbtCylindricalPoint3::from_xy(enemy_center_xy);
        enemys.push((enemy_id, armor_base_cylindrical));

        // --- 可视化 ---
        rec.log(
            "world/base_link",
            &rerun::Transform3D::default().with_axis_length(300.0),
        )?;
        rec.log(
            "world/enemy_link",
            &rerun::Transform3D::default()
                .with_axis_length(300.0)
                .with_translation([enemy_center_xy.x as f32, enemy_center_xy.y as f32, 0.0f32]),
        )?;
        for (idx, armor_pose) in armors_base_pose_xyz.iter().enumerate() {
            armor_pose.armor_visualize(&rec, idx)?
        }

        let image =
            image::open("/home/flamingo/Project/robomaster/auto_aim_rust/imgs/test_resize.jpg")
                .unwrap();
        rec.log("world/image", &rerun::Image::from_image(image).unwrap())
            .expect("failed to show img in rerun");

        info!("App finished successfully");
        // --- 结束可视化 ---
    }

    Ok(enemys)
}

fn solve_enemy_center(armors_line_2d: &[RbtLine2]) -> Option<na::Point2<f64>> {
    let armors_line_num = armors_line_2d.len();
    if armors_line_num == 0 {
        warn!("未能成功解算出装甲板，跳过");
        None
    } else if armors_line_num == 1 {
        Some(handle_single_armor(&armors_line_2d[0]))
    } else if armors_line_num == 2 {
        Some(handle_multi_armor(&armors_line_2d))
    } else {
        warn!("解算出两块以上的装甲板, 跳过");
        None
    }
}

fn handle_single_armor(armors_line_2d: &RbtLine2) -> na::Point2<f64> {
    let r = 200.0;
    dbg!(&armors_line_2d);
    dbg!(armors_line_2d.point - armors_line_2d.direction * r);
    armors_line_2d.point - armors_line_2d.direction * r
}

fn handle_multi_armor(armors_line_2d: &[RbtLine2]) -> na::Point2<f64> {
    find_intersection(&armors_line_2d[0], &armors_line_2d[1])
}
