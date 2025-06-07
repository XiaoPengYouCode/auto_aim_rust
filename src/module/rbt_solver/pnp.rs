use argmin::core::Executor;
use argmin::solver::neldermead::NelderMead;
use nalgebra::{Matrix3, Vector2, Vector3};

#[derive(Debug)]
pub struct PnpResult {
    yaw: f64,
    x: f64,
    y: f64,
    z: f64,
    cost: f64,
}

impl PnpResult {
    pub fn point_3d(&self) -> (f32, f32, f32) {
        (self.x as f32, self.y as f32, self.z as f32)
    }

    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    pub fn world_yaw(&self) -> f64 {
        180.0 - self.yaw
    }
}

/// 建模，采用上海交通大学交龙战队降自由度PNP解算方案
/// 默认Roll=0,Pitch=15,以提高Yaw轴解算精度，并降低算法复杂度
/// from: 肖皓宇: 但是在坡上可能需要问题，需要测试
fn rotation_matrix(yaw: f64) -> Matrix3<f64> {
    let pitch = 15f64.to_radians();
    let roll: f64 = 0.0;

    let rx = Matrix3::new(
        1.0,
        0.0,
        0.0,
        0.0,
        roll.cos(),
        -roll.sin(),
        0.0,
        roll.sin(),
        roll.cos(),
    );

    let ry = Matrix3::new(
        pitch.cos(),
        0.0,
        pitch.sin(),
        0.0,
        1.0,
        0.0,
        -pitch.sin(),
        0.0,
        pitch.cos(),
    );

    let rz = Matrix3::new(
        yaw.cos(),
        -yaw.sin(),
        0.0,
        yaw.sin(),
        yaw.cos(),
        0.0,
        0.0,
        0.0,
        1.0,
    );

    rz * ry * rx
}

/// Argmin优化结构体
pub struct PoseEstimationProblem {
    pub object_points: Vec<Vector3<f64>>,
    pub image_points: Vec<Vector2<f64>>,
    pub camera_matrix: Matrix3<f64>,
}

impl argmin::core::CostFunction for PoseEstimationProblem {
    type Param = Vec<f64>;
    type Output = f64;
    // 优化目标函数
    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        let yaw = param[0];
        let translation = Vector3::new(param[1], param[2], param[3]);

        // 计算旋转矩阵
        let rotation = rotation_matrix(yaw);

        // 计算重投影误差
        let mut total_error = 0.0;
        for (i, (p3d, p2d)) in self
            .object_points
            .iter()
            .zip(self.image_points.iter())
            .enumerate()
        {
            let pc = rotation * p3d + translation;
            let projected = self.camera_matrix * pc;
            let projected = Vector2::new(projected.x / projected.z, projected.y / projected.z);
            let error = (projected - p2d).norm_squared();
            total_error += error;
            // 打印每个点的实际位置和重投影位置
            tracing::debug!(
                "点 {}: 图像点 = {:?}, 重投影点 = {:?}, 误差 = {}",
                i,
                p2d,
                projected,
                error
            );
        } // --- 调试关键 ---

        tracing::debug!("当前参数: {:?}, 总误差: {}", param, total_error);

        Ok(total_error)
    }
}

pub fn solve_pnp(img_points: Vec<Vector2<f64>>) -> Result<PnpResult, Box<dyn std::error::Error>> {
    let object_points: Vec<Vector3<f64>> = vec![
        Vector3::new(-135.0 / 2.0, 30.0, 0.0),
        Vector3::new(-135.0 / 2.0, -30.0, 0.0),
        Vector3::new(135.0 / 2.0, -30.0, 0.0),
        Vector3::new(135.0 / 2.0, 30.0, 0.0),
    ];

    let camera_matrix: Matrix3<f64> = Matrix3::new(
        1600.0, 0.0, 320.0, // fx, 0, cx
        0.0, 1705.7, 192.0, // 0, fy, cy
        0.0, 0.0, 1.0,
    );

    let problem = PoseEstimationProblem {
        object_points,
        image_points: img_points.clone(),
        camera_matrix,
    };

    let init_param = if img_points[0].x < (640.0 / 2.0) {
        vec![15.0_f64.to_radians(), 100.0, 100.0, 3000.0]
    } else {
        vec![-15.0_f64.to_radians(), 100.0, 100.0, 3000.0]
    };

    let delta = [0.1, 1.0, 1.0, 100.0]; // 每个参数的小扰动

    let simplex = vec![
        init_param.clone(),
        vec![
            init_param[0] + delta[0],
            init_param[1],
            init_param[2],
            init_param[3],
        ],
        vec![
            init_param[0],
            init_param[1] + delta[1],
            init_param[2],
            init_param[3],
        ],
        vec![
            init_param[0],
            init_param[1],
            init_param[2] + delta[2],
            init_param[3],
        ],
        vec![
            init_param[0],
            init_param[1],
            init_param[2],
            init_param[3] + delta[3],
        ],
    ];

    let solver = NelderMead::new(simplex);

    let res = Executor::new(problem, solver)
        .configure(|state| state.max_iters(100))
        .run()?;

    tracing::debug!("最优参数: {:?}", res.state().best_param);
    tracing::debug!("最小误差: {:?}", res.state().best_cost);

    let rst = if let Some(param) = &res.state().best_param {
        PnpResult {
            yaw: param[0],
            x: param[1],
            y: param[2],
            z: param[3],
            cost: res.state.best_cost,
        }
    } else {
        return Err("Failed to solve pnp".into());
    };

    Ok(rst)
}
