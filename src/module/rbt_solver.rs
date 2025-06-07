// 需要完成的事情，

// 1. 解算画面中每一块装甲板的位姿
// 2. 计算车体中心的坐标

// 坐标系定义，以当前车体的中心点的地面投影为原点

use crate::module::armor::ArmorStaticMsg;
use crate::module::rbt_solver::pnp::PnpResult;
use crate::rbt_err::RbtError;
use nalgebra::Vector2;
use pnp::solve_pnp;
use rerun::Vec3D;

mod pnp;

#[derive(Debug)]
enum RbtClass {
    // Hero,
    Engineer,
    // Infantry,
    // Sentry,
}

#[derive(Debug)]
pub struct RbtMsg {
    _class: RbtClass,
}

impl RbtMsg {
    fn new(class: RbtClass) -> RbtMsg {
        RbtMsg { _class: class }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct RbtSolver {
    armors: Vec<ArmorStaticMsg>,
}

impl RbtSolver {
    // 根据 ArmorStaticMsg, 构建 RobotSolver
    pub fn new() -> Result<Self, RbtError> {
        // // let center = armors.center();
        // let left_top = armors.left_top();
        // let right_top = armors.right_top();
        // let left_bottom = armors.left_bottom();
        // let right_bottom = armors.right_bottom();
        //
        // let distance_left =
        //     (left_top.0 as f64 - right_top.0 as f64).hypot(left_top.1 as f64 - right_top.1 as f64);
        // let distance_right = (left_bottom.0 as f64 - right_bottom.0 as f64)
        //     .hypot(left_bottom.1 as f64 - right_bottom.1 as f64);

        // let x = (distance_left + distance_right) / 2.0;
        // let y = (left_top.1 + right_top.1 + left_bottom.1 + right_bottom.1) as f64 / 4.0;

        // // 融合数据，构建整车模型
        // let rbt_center = 5000.0; // 车体中心
        // let rbt_armors_hit_msg = Vec::<(f64, f64)>::with_capacity(4); // 四块装甲板中心的Z轴高度和距离车体中心的角度
        // let rbt_armors_angle = 0.0f64; // 0.0对应正对（选板需更新）

        Ok(RbtSolver { armors: vec![] })
    }

    pub fn solve_pnp(
        &mut self,
        dbg: &Option<rerun::RecordingStream>,
        armors: Vec<ArmorStaticMsg>,
    ) -> Result<Vec<PnpResult>, RbtError> {
        let mut results = vec![];
        // pnp solver
        for armor in armors.iter() {
            let image_points: Vec<Vector2<f64>> = vec![
                Vector2::new(armor.left_top().x(), armor.left_top().y()),
                Vector2::new(armor.left_bottom().x(), armor.left_bottom().y()),
                Vector2::new(armor.right_bottom().x(), armor.right_bottom().y()),
                Vector2::new(armor.right_top().x(), armor.right_top().y()),
            ];

            tracing::debug!("image points {:?}", image_points);

            results.push(solve_pnp(image_points).unwrap()); // todo: convert argmin error to RbtError
        }

        if let Some(rec) = dbg {
            for (idx, result) in results.iter().enumerate() {
                rec.log(
                    "armor/rbt_solver",
                    &rerun::Points3D::new([result.point_3d()]),
                )?;
                rec.log(
                    format!("armor/rbt_solver_armor_{}", idx),
                    &rerun::Arrows3D::from_vectors([Vec3D::new(
                        1000.0 * f32::sin(15.0) * (result.world_yaw().cos() as f32),
                        1000.0 * f32::sin(15.0) * (result.world_yaw().sin() as f32),
                        1000.0 * f32::cos(15.0),
                    )])
                    .with_origins([result.point_3d()]),
                )?;
            }
        }

        Ok(results)
    }
}
