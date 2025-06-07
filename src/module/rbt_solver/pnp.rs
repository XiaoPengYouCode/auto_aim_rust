use argmin::core::Executor;
use argmin::solver::neldermead::NelderMead;
use nalgebra::{Matrix3, Vector2, Vector3};

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
        for (p3d, p2d) in self.object_points.iter().zip(self.image_points.iter()) {
            let pc = rotation * p3d + translation;
            let projected = self.camera_matrix * pc;
            let projected = Vector2::new(projected.x / projected.z, projected.y / projected.z);
            total_error += (projected - p2d).norm_squared();
        }

        Ok(total_error)
    }
}

pub fn solve_pnp(img_points: Vec<Vector2<f64>>) -> Result<Vec<f64>, Box<dyn std::error::Error>> {
    let object_points: Vec<Vector3<f64>> = vec![
        Vector3::new(-135.0 / 2.0, 30.0, 0.0),
        Vector3::new(-135.0 / 2.0, -30.0, 0.0),
        Vector3::new(135.0 / 2.0, -30.0, 0.0),
        Vector3::new(135.0 / 2.0, 30.0, 0.0),
    ];

    let camera_matrix: Matrix3<f64> =
        Matrix3::new(4000.0, 0.0, 960.0, 0.0, 4000.0, 540.0, 0.0, 0.0, 1.0);

    let problem = PoseEstimationProblem {
        object_points,
        image_points: img_points,
        camera_matrix,
    };

    // 初始参数猜测
    let init_param = vec![0.0, 3000.0, 0.0, 100.0];

    // 构建初始单纯形
    let simplex = vec![
        init_param.clone(),
        vec![0.1, 0.0, 0.0, 5.0],
        vec![0.0, 0.1 * 3000.0, 0.0, 5.0 * 100.0],
        vec![0.0, 0.0, 0.1, 5.0 * 100.0],
        vec![0.0, 0.0, 0.0, 5.1 * 100.0],
    ];

    let solver = NelderMead::new(simplex);

    let res = Executor::new(problem, solver)
        .configure(|state| state.max_iters(100))
        .run()?;

    tracing::debug!("最优参数: {:?}", res.state().best_param);
    tracing::debug!("最小误差: {:?}", res.state().best_cost);

    Ok(res.state().clone().best_param.unwrap())
}
