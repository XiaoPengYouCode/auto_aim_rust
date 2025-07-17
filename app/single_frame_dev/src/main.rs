use nalgebra as na;
use rerun as rr;

use lib::rbt_base::rbt_geometry::rbt_coord_dev::CAMERA_AXES_TO_BODY_AXES_ROTATION;
use rerun::external::image;
use tracing::{error, info};

use lib::rbt_base::rbt_cam_model::adjust_coordinates;
use lib::rbt_base::rbt_geometry::rbt_coord_dev::{RbtPose, RbtPoseCoordSys};
use lib::rbt_base::rbt_geometry::rbt_coord_2d::RbtCylindricalCoord2;
use lib::rbt_base::rbt_geometry::rbt_point_dev::{RbtImgPoint2, RbtImgPoint2Coord};
use lib::rbt_base::rbt_ippe::{ARMOR_LIGHT_HEIGHT, ARMOR_LIGHT_WEIGHT, ArmorPnpSolver};
use lib::rbt_err::{RbtError, RbtResult};
use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_mod::rbt_armor::ArmorType;
use lib::rbt_mod::rbt_detector::pipeline;
use lib::rbt_base::rbt_geometry::rbt_line_2d::{RbtLine2, find_intersection};

#[tokio::main]
async fn main() -> RbtResult<()> {
    // 初始化流程
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init().await?;
    // let rec = rr::RecordingStreamBuilder::new("AutoAim").save("rerun-log/test.rrd")?;
    let rec = rr::RecordingStreamBuilder::new("AutoAim").spawn()?;

    let cam_k = cfg.cam_cfg.cam_k();

    let detector_result = pipeline(&cfg.detector_cfg)?;

    // 得到两块装甲板的 pnp 解算结果
    let mut pnp_results = Vec::with_capacity(detector_result.len());
    for armor in detector_result {
        let armor_key_points = armor.cornet_points();
        info!("armor_corner_points: {:?}", armor_key_points);
        let armor_key_points_na = [
            armor_key_points[0].to_na_point(),
            armor_key_points[1].to_na_point(),
            armor_key_points[2].to_na_point(),
            armor_key_points[3].to_na_point(),
        ];

        let pnp_solver = ArmorPnpSolver::new().ok_or(RbtError::StringError(
            "Failed to create ArmorPnpSolver Instant".to_string(),
        ))?;
        if let Some(pose) = pnp_solver.solve(&armor_key_points_na, &cam_k) {
            pnp_results.push(pose);
        } else {
            error!("❌ PnP solving failed!")
        };
    }

    info!("pnps number: {}", pnp_results.len());

    let mut armors_base_pose_xyz = vec![];
    for pose in pnp_results.iter() {
        let armor_cam_pose = RbtPose::new(
            <na::Point3<f64>>::from(pose.translation.vector),
            pose.rotation.to_rotation_matrix(),
            RbtPoseCoordSys::Camera,
        );
        let armor_base_pose = armor_cam_pose.coord_trans(RbtPoseCoordSys::BaseXyz);
        armors_base_pose_xyz.push(armor_base_pose);
    }

    let [armor1_x, armor1_y, _armor1_z] = [
        armors_base_pose_xyz[0].translation.x,
        armors_base_pose_xyz[0].translation.y,
        armors_base_pose_xyz[0].translation.z,
    ];
    let [armor2_x, armor2_y, _armor2_z] = [
        armors_base_pose_xyz[1].translation.x,
        armors_base_pose_xyz[1].translation.y,
        armors_base_pose_xyz[1].translation.z,
    ];

    let armor1_2d_pose = [
        armors_base_pose_xyz[0].rotation.matrix().m13,
        armors_base_pose_xyz[0].rotation.matrix().m23,
    ];
    let armor2_2d_pose = [
        armors_base_pose_xyz[1].rotation.matrix().m13,
        armors_base_pose_xyz[1].rotation.matrix().m23,
    ];

    // 求解中心点
    let line1 = RbtLine2 {
        point: na::Point2::new(armor1_x, armor1_y),
        direction: na::Vector2::new(armor1_2d_pose[0], armor1_2d_pose[1]),
    };
    let line2 = RbtLine2 {
        point: na::Point2::new(armor2_x, armor2_y),
        direction: na::Vector2::new(armor2_2d_pose[0], armor2_2d_pose[1]),
    };
    let enemy_center_xy = find_intersection(&line1, &line2);

    let armor_base_cylindrical =
        RbtCylindricalCoord2::from_xy(enemy_center_xy);

    dbg!(armor_base_cylindrical.dist);
    dbg!(armor_base_cylindrical.angle_yaw_d);

    rec.log(
        "world/base_link",
        &rerun::Transform3D::default().with_axis_length(300.0),
    )?;
    rec.log(
        "world/base_link",
        &rerun::Transform3D::default()
            .with_axis_length(300.0)
            .with_translation([enemy_center_xy.x as f32, enemy_center_xy.y as f32, 0.0f32]),
    )?;
    for (idx, armor_pose) in armors_base_pose_xyz.iter().enumerate() {
        armor_pose.armor_visualize(&rec, idx)?
    }

    let image = image::open("/home/flamingo/Project/robomaster/auto_aim_rust/imgs/test_resize.jpg")
        .unwrap();
    rec.log("world/image", &rerun::Image::from_image(image).unwrap())
        .expect("failed to show img in rerun");

    info!("App finished successfully");

    Ok(())
}

