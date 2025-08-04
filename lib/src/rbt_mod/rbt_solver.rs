use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
use crate::rbt_base::rbt_geometry::rbt_line2::{RbtLine2, find_intersection};
use crate::rbt_base::rbt_geometry::rbt_pose3::{RbtPose3, RbtPoseCoordSys};
use crate::rbt_base::rbt_ippe::ArmorPnpSolver;
use crate::rbt_infra::rbt_cfg;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
use crate::rbt_mod::rbt_armor::solved_armor;
use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
use crate::rbt_mod::rbt_estimator::rbt_enemy_model::EnemyId;
use nalgebra::Vector2;
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use tracing::{debug, error, info, warn};

#[derive(Debug, Clone)]
pub struct RbtSolver {
    armors: Vec<DetectedArmor>,
}

impl RbtSolver {
    // 根据 ArmorStaticMsg, 构建 RobotSolver
    pub fn new() -> RbtResult<Self> {
        Ok(Self { armors: vec![] })
    }
}

#[derive(Debug, Clone)]
pub struct RbtSolvedResult {
    coord: RbtCylindricalPoint2,
    armors: Vec<SolvedArmor>,
}

pub struct RbtSolvedResults {
    result: HashMap<EnemyId, Option<RbtSolvedResult>>,
}

impl Default for RbtSolvedResults {
    fn default() -> Self {
        let mut result = HashMap::with_capacity(6);
        result.insert(EnemyId::Hero1, None);
        result.insert(EnemyId::Engineer2, None);
        result.insert(EnemyId::Infantry3, None);
        result.insert(EnemyId::Infantry4, None);
        result.insert(EnemyId::Sentry7, None);
        result.insert(EnemyId::Outpost8, None);
        RbtSolvedResults { result }
    }
}

impl Deref for RbtSolvedResults {
    type Target = HashMap<EnemyId, Option<RbtSolvedResult>>;

    fn deref(&self) -> &Self::Target {
        &self.result
    }
}

impl DerefMut for RbtSolvedResults {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.result
    }
}

/// enemys_solver全流程
pub fn enemys_solver(
    detector_result: HashMap<EnemyId, Vec<DetectedArmor>>,
    cam_k: &na::Matrix3<f64>,
    rec: &rr::RecordingStream,
) -> RbtResult<RbtSolvedResults> {
    // 构建解算结果，内部是一个HashMap
    let mut enemys = RbtSolvedResults::default();
    for (enemy_id, enemy_armors) in detector_result.into_iter() {
        let detected_enemy_armors_num = enemy_armors.len();
        if detected_enemy_armors_num == 0 {
            continue; // 该帧没有看到该兵种的装甲板，提前退出
        }

        // 针对每一块装甲板求解pnp
        let mut enemy_solved_armors = Vec::with_capacity(detected_enemy_armors_num);
        for armor in enemy_armors {
            let armor_key_points = armor.cornet_points();
            let armor_key_points_na = armor_key_points.map(|p| p.into());

            let pnp_solver = ArmorPnpSolver::new().ok_or(RbtError::StringError(
                "Failed to create ArmorPnpSolver Instant".to_string(),
            ))?;
            if let Some(camera_pose) = pnp_solver.solve(&armor_key_points_na, &cam_k) {
                let solved_armor = SolvedArmor::new(armor, camera_pose, 0.0, 0.0, 0.0);
                enemy_solved_armors.push(solved_armor);
            } else {
                error!("❌ PnP solving failed!");
                todo!("Solve失败应该后续使用ESKF纯预测进行一次更新");
            };
        }

        // 将 pnp 结果转换为机体坐标系
        for solved_armor in enemy_solved_armors.iter_mut() {
            solved_armor
                .pose_mut()
                .coord_trans_mut(RbtPoseCoordSys::BaseXyz);
        }

        // 根据装甲板的连线计算敌人中心坐标
        let armors_line_2d = enemy_solved_armors
            .iter()
            .map(|solved_armor| {
                let armor_pose = solved_armor.pose();
                let [armor_x, armor_y] = [armor_pose.translation.x, armor_pose.translation.y];
                let rot_mat = armor_pose.rotation.to_rotation_matrix().matrix().clone();
                let [armor_2d_pose_a, armor_2d_pose_b] = [rot_mat.m13, rot_mat.m23];
                RbtLine2 {
                    point: na::Point2::new(armor_x, armor_y),
                    direction: na::Vector2::new(armor_2d_pose_a, armor_2d_pose_b),
                }
            })
            .collect::<Vec<RbtLine2>>();
        let enemy_center_xy = solve_enemy_center(&armors_line_2d)
            .ok_or(RbtError::StringError("Failed to solve enemy center".into()))?;

        // 得到的base坐标系下敌人中心坐标
        let enemy_base_cylindrical = RbtCylindricalPoint2::from_xy(enemy_center_xy);
        // 根据该中心坐标，求解装甲板其他参数
        for solved_armor in enemy_solved_armors.iter_mut() {
            let armor_pose = solved_armor.pose();
            let [armor_x, armor_y] = [armor_pose.translation.x, armor_pose.translation.y];
            let dx = armor_x - enemy_center_xy.x;
            let dy = armor_y - enemy_center_xy.y;
            let radius = (dx * dx + dy * dy).sqrt();
            solved_armor.update_measurement(radius);
        }

        /* --- 可视化 --- */
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
        for (idx, armor_pose) in enemy_solved_armors.iter().enumerate() {
            armor_pose.pose().armor_visualize(&rec, idx)?
        }

        let image =
            image::open("/home/flamingo/Project/robomaster/auto_aim_rust/imgs/test_resize.jpg")
                .unwrap();
        rec.log("world/image", &rerun::Image::from_image(image).unwrap())
            .expect("failed to show img in rerun");
        /* --- 结束可视化 --- */

        let solved_result = RbtSolvedResult {
            coord: enemy_base_cylindrical,
            armors: enemy_solved_armors,
        };
        enemys
            .entry(enemy_id)
            .and_modify(|result| *result = Some(solved_result));

        info!("App finished successfully");
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

/// 处理只看到一块装甲板的情况
/// 使用估计出的半径进行中心求解
fn handle_single_armor(armors_line_2d: &RbtLine2) -> na::Point2<f64> {
    let r = 200.0;
    dbg!(&armors_line_2d);
    dbg!(armors_line_2d.point - armors_line_2d.direction * r);
    armors_line_2d.point - armors_line_2d.direction * r
}

/// 处理看到两块装甲板的情况
/// 使用反向延长线求解中心
fn handle_multi_armor(armors_line_2d: &[RbtLine2]) -> na::Point2<f64> {
    find_intersection(&armors_line_2d[0], &armors_line_2d[1])
}
