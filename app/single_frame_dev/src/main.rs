use nalgebra as na;
use rerun as rr;

use rerun::external::image;
use std::f32::consts::TAU;
use tracing::info;

use lib::rbt_base::rbt_cam_model::adjust_coordinates;
use lib::rbt_base::rbt_geometry::rbt_coord_dev::{RbtCylindricalCoord, RbtPose, RbtPoseCoordSys};
use lib::rbt_base::rbt_geometry::rbt_point_dev::{RbtImgPoint2, RbtImgPoint2Coord};
use lib::rbt_base::rbt_math::sqpnp::PnpSolver;
use lib::rbt_err::RbtResult;
use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_mod::rbt_armor::SMALL_ARMOR_POINT3;
use lib::rbt_mod::rbt_detector::pipeline;

#[tokio::main]
async fn main() -> RbtResult<()> {
    // 初始化流程
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init().await?;
    let rec = rr::RecordingStreamBuilder::new("AutoAim").save("rerun-log/test.rrd")?;

    let cam_k = cfg.cam_cfg.cam_k();

    let detector_result = pipeline(&cfg.detector_cfg)?;

    let mut pnp_results = vec![];
    for armor in detector_result {
        let mut armor_key_points = armor.cornet_points();

        let mut armor_key_points_na = Vec::with_capacity(armor_key_points.len());
        for point in armor_key_points.iter_mut() {
            let na_corner_point = point.img_to_cam(&cam_k)?.to_na_point();
            armor_key_points_na.push(na_corner_point);
        }

        let mut pnp = PnpSolver::new(SMALL_ARMOR_POINT3.to_vec(), armor_key_points_na, None)?;
        pnp.solve()?;
        pnp_results.push(pnp);
    }

    info!("pnps number: {}", pnp_results.len());

    let mut armors_base_pose = vec![];
    for r in pnp_results.iter() {
        let solution = r.solutions[0];
        let r_hat = solution.r_hat;
        let rotation_matrix =
            na::Rotation3::from_matrix(&na::Matrix3::from_column_slice(r_hat.as_slice()));
        let t = solution.t;

        let armor_cam_pose = RbtPose::new(
            <na::Point3<f64>>::from(t),
            rotation_matrix,
            RbtPoseCoordSys::Camera,
        );
        let armor_base_pose = armor_cam_pose.coord_trans(RbtPoseCoordSys::BaseXyz);

        // todo!("装甲板中心需要最低高度限幅，但是还要考虑高打低处的情况“)
        info!("armor_base_pose: t: {}", armor_base_pose.translation);
        info!("armor_base_pose: r: {}", armor_base_pose.rotation);

        let yaw_angle = armor_base_pose.cal_yaw()?;
        let distance = armor_base_pose.cal_distance()?;
        info!("Armor yaw angle: {}", yaw_angle);
        info!("Armor distance: {}", distance);

        let armor_base_cylindrical =
            RbtCylindricalCoord::new(distance, yaw_angle, armor_base_pose.translation.z);
        armors_base_pose.push(armor_base_pose);
    }
    rec.log(
        "base_link",
        &rerun::Transform3D::default().with_axis_length(100.0),
    )?;
    for (idx, armor) in armors_base_pose.iter().enumerate() {
        let armor_translation_rr = [
            armor.translation.x as f32,
            armor.translation.y as f32,
            armor.translation.z as f32,
        ];
        let armor_rotation_q = na::UnitQuaternion::from_rotation_matrix(&armor.rotation);
        let armor_rotation_q_rr = [
            armor_rotation_q.i as f32,
            armor_rotation_q.j as f32,
            armor_rotation_q.k as f32,
            armor_rotation_q.w as f32,
        ];
        let dir_in_world = armor_rotation_q * na::Vector3::x_axis();
        let yaw= dir_in_world.y.atan2(dir_in_world.x).to_degrees(); // yaw = atan2(y, x)
        rec.log(
            format!("armor_{}", idx),
            &[
                &rerun::Boxes3D::from_half_sizes([(10.0, 85.0 / 2.0, 135.0 / 2.0)])
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
        )?;
    }

    let image = image::open("/home/flamingo/Project/robomaster/auto_aim_rust/imgs/test_resize.jpg")
        .unwrap();
    rec.log("img", &rerun::Image::from_image(image).unwrap())
        .expect("failed to show img in rerun");

    info!("App finished successfully");

    Ok(())
}
