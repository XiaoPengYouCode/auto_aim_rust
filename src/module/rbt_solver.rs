// 需要完成的事情，

// 1. 解算画面中每一块装甲板的位姿
// 2. 计算车体中心的坐标

// 坐标系定义，以当前车体的中心点的地面投影为原点

use crate::module::armor::ArmorStaticMsg;
use crate::module::pnp::solve_pnp;
use crate::robot_error::RbtError;
use nalgebra::Vector2;

#[derive(Debug)]
enum RobotClass {
    // Hero,
    Engineer,
    // Infantry,
    // Sentry,
}

#[derive(Debug)]
pub struct RobotMsg {
    _class: RobotClass,
}

impl RobotMsg {
    fn new(class: RobotClass) -> RobotMsg {
        RobotMsg { _class: class }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct RobotSolver {
    armors: Vec<ArmorStaticMsg>,
    robot_msg: RobotMsg,
}

impl RobotSolver {
    // 根据 ArmorStaticMsg, 构建 RobotSolver
    pub fn from_armors(armors: Vec<ArmorStaticMsg>) -> Result<Self, RbtError> {
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

        // pnp solver
        for armor in armors.iter() {
            let image_points: Vec<Vector2<f64>> = vec![
                Vector2::new(armor.left_top().x(), armor.left_top().y()),
                Vector2::new(armor.left_bottom().x(), armor.left_bottom().y()),
                Vector2::new(armor.right_bottom().x(), armor.right_bottom().y()),
                Vector2::new(armor.right_top().x(), armor.right_top().y()),
            ];

            let result = solve_pnp(image_points);
            tracing::info!("Result {:?}", result);
        }

        // // 融合数据，构建整车模型
        // let rbt_center = 5000.0; // 车体中心
        // let rbt_armors_hit_msg = Vec::<(f64, f64)>::with_capacity(4); // 四块装甲板中心的Z轴高度和距离车体中心的角度
        // let rbt_armors_angle = 0.0f64; // 0.0对应正对（选板需更新）

        Ok(RobotSolver {
            armors,
            robot_msg: RobotMsg::new(RobotClass::Engineer),
        })
    }

    pub fn armor(&self) -> &RobotMsg {
        &self.robot_msg
    }
}
