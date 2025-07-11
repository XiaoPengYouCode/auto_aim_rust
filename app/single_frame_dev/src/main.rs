use nalgebra as na;
use rerun as rr;

use lib::rbt_base::rbt_geometry::rbt_coord_dev::CAMERA_AXES_TO_BODY_AXES_ROTATION;
use rerun::external::image;
use tracing::{error, info};

use lib::rbt_base::rbt_cam_model::adjust_coordinates;
use lib::rbt_base::rbt_geometry::rbt_coord_dev::{RbtCylindricalCoord, RbtPose, RbtPoseCoordSys};
use lib::rbt_base::rbt_geometry::rbt_point_dev::{RbtImgPoint2, RbtImgPoint2Coord};
use lib::rbt_base::rbt_ippe::{ARMOR_LIGHT_HEIGHT, ARMOR_LIGHT_WEIGHT, ArmorPnpSolver};
use lib::rbt_base::rbt_math::sqpnp::PnpSolver;
use lib::rbt_err::{RbtError, RbtResult};
use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_mod::rbt_armor::ArmorType;
use lib::rbt_mod::rbt_detector::pipeline;

#[tokio::main]
async fn main() -> RbtResult<()> {
    // 初始化流程
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init().await?;
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
    let mut armor_base_pose_cylindrical = vec![];
    let mut distance_mean = 0.0;
    for pose in pnp_results.iter() {
        let armor_cam_pose = RbtPose::new(
            <na::Point3<f64>>::from(pose.translation.vector),
            pose.rotation.to_rotation_matrix(),
            RbtPoseCoordSys::Camera,
        );
        let armor_base_pose = armor_cam_pose.coord_trans(RbtPoseCoordSys::BaseXyz);

        info!("armor_base_pose: t: {}", armor_base_pose.translation);
        info!("armor_base_pose: r: {}", armor_base_pose.rotation);

        let yaw_angle = armor_base_pose.cal_yaw()?;
        let distance = armor_base_pose.cal_distance()?;
        info!("Armor yaw angle: {}", yaw_angle);
        info!("Armor distance: {}", distance);

        let armor_base_cylindrical =
            RbtCylindricalCoord::new(distance, yaw_angle, armor_base_pose.translation.z);
        armors_base_pose_xyz.push(armor_base_pose);
        armor_base_pose_cylindrical.push(armor_base_cylindrical);
        distance_mean += distance / 2.0;
    }

    rec.log(
        "world/base_link",
        &rerun::Transform3D::default().with_axis_length(300.0),
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
