use lib::rbt_base::rbt_cam_model::RbtCamExtrinsics;
use lib::rbt_base::rbt_geometry::rbt_coord::RbtWorldPoint3;

use nalgebra as na;

fn main() {
    // 把相机坐标轴（右-下-前）表示为在机体坐标系（前-左-上）中的方向
    // 注意 Rotation3 是按列定义三个轴（X, Y, Z）
    let camera_axes_to_body_axes_rotation =
        na::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
            0.0, 0.0, 1.0, // X_cam → Z_body
            -1.0, 0.0, 0.0, // Y_cam → -X_body
            0.0, -1.0, 0.0, // Z_cam → -Y_body
        ));

    let pitch_rad = 9_f64.to_radians();
    let pitch_rotation = na::Rotation3::from_euler_angles(0.0, pitch_rad, 0.0);

    // 总旋转：先变换坐标系，再添加俯仰角（注意乘法顺序！先右乘坐标轴转换，再左乘pitch旋转）
    let total_rotation = pitch_rotation * camera_axes_to_body_axes_rotation;

    // 基于base的移动
    let translation = na::Translation3::new(0.05, 0.01, 0.32);
    let cam_extrinsics = RbtCamExtrinsics::new(total_rotation, translation);

    println!("Camera Extrinsics: {:?}", cam_extrinsics);

    let p_camera = RbtWorldPoint3::new(-0.230, 0.072, 0.361);
    let p_world: RbtWorldPoint3 = cam_extrinsics
        .isometry()
        .transform_point(&p_camera.point())
        .coords
        .into();

    println!("Transformed World Point: {:?}", p_world);
}
