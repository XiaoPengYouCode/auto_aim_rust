use crate::rbt_base::rbt_math::eskf::{Eskf, EskfDynamic};

pub struct Armor {
    pub position: ArmorPosition,
    pub yaw: f64,
    pub number: u8,
}

impl Armor {
    pub const OUTPOST: u8 = 8;
}

pub struct ArmorPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

// 跟踪状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum TrackerState {
    Detecting, // 检测中
    Tracking,  // 跟踪中
    TempLost,  // 暂时丢失
    Lost,      // 完全丢失
}

// 运动模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum MovementMode {
    Static,      // 静止
    Translation, // 平移
    Spinning,    // 旋转
}

// 机器人状态结构体
struct RobotState {
    pub num: u8,               // 装甲板编号
    pub idx: usize,            // 当前装甲板索引
    pub direction: f64,        // 旋转方向
    pub armor_rs: [f64; 4],    // 各装甲板半径
    pub armor_zs: [f64; 4],    // 各装甲板高度
    pub determined: [bool; 4], // 装甲板是否已确定
}

const OUTPOST_ROTATE_R: f64 = 1800.0;

pub struct JunSolver {}

impl EskfDynamic<9, 4> for JunSolver {
    type Input = f64;
    type Measurement = na::SVector<f64, 4>;
    type NominalState = na::SVector<f64, 9>;

    fn inject_error(
        &self,
        nominal_state: Self::NominalState,
        error_estimate: &na::SVector<f64, 9>,
    ) -> Self::NominalState {
        na::SVector::zeros()
    }

    fn measurement_matrix_h(&self, nominal_state: &Self::NominalState) -> na::SMatrix<f64, 4, 9> {
        na::SMatrix::<f64, 4, 9>::zeros()
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
    ) -> na::SVector<f64, 4> {
        na::SVector::<f64, 4>::zeros()
    }

    #[rustfmt::skip] // 提高矩阵的可读性
    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) -> na::SMatrix<f64, 9, 9> {
        na::SMatrix::<f64, 9, 9>::from_row_slice(&[
            1.0,   dt,    0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   1.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   1.0,   dt,    0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   1.0,   dt,    0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   dt,    0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,
        ])
    }

    // 使用 ESKF 维护的 v_yaw 来更新装甲板的 yaw
    fn update_nominal_state(
        &self,
        nominal_state: &mut Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) {
        nominal_state[6] += u * dt;
        nominal_state[6] = nominal_state[6].nominalize();
    }
}

struct RobotEstimator {
    // 名义状态 (xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r)
    pub state: na::SVector<f64, 9>,
    pub dynamic: JunSolver, // solver
    pub eskf: Eskf<9, 4>,

    pub last_state: na::SVector<f64, 9>,
    pub tracker_state: TrackerState, // 跟踪状态
    pub movement: MovementMode,      // 运动模式

    // 时间相关
    pub dt: f64,        // 时间步长
    pub last_time: f64, // 上次更新时间

    // 机器人特定状态
    pub robot: RobotState,

    // 测量相关
    pub measurement: na::SVector<f64, 4>, // 测量值 (x, y, z, yaw)

    // 阈值参数
    pub max_match_distance: f64, // 最大匹配距离
    pub max_match_yaw_diff: f64, // 最大匹配角度差
    pub lost_threshold: usize,   // 丢失阈值
}

impl RobotEstimator {
    pub fn new(
        initial_armor: Armor,              // 初始装甲板
        dynamic: JunSolver,                // ESKF求解器实现
        initial_p: na::SMatrix<f64, 9, 9>, // 初始协方差
        q: na::SMatrix<f64, 9, 9>,         // 过程噪声
        r: na::SMatrix<f64, 4, 4>,         // 测量噪声
        dt: f64,
    ) -> Self {
        // 初始化名义状态
        let mut state = na::SVector::<f64, 9>::zeros();

        // 从装甲板提取初始状态
        let xa = initial_armor.position.x;
        let ya = initial_armor.position.y;
        let za = initial_armor.position.z;
        let yaw = initial_armor.yaw;

        // 设置初始状态 (根据C++的initEKF逻辑)
        let r_init = 0.26; // 初始半径
        state[0] = xa + r_init * yaw.cos(); // xc
        state[1] = 0.0; // v_xc
        state[2] = ya + r_init * yaw.sin(); // yc
        state[3] = 0.0; // v_yc
        state[4] = za; // za
        state[5] = 0.0; // v_za
        state[6] = yaw; // yaw
        state[7] = 0.0; // v_yaw
        state[8] = r_init; // r

        // 初始化ESKF
        let eskf = Eskf::new(initial_p, q, r, dt);

        // 初始化机器人状态
        let mut robot = RobotState {
            num: initial_armor.number,
            idx: 0,
            direction: 1.0,
            armor_rs: [0.26; 4], // 初始半径
            armor_zs: [za; 4],   // 初始高度
            determined: [false; 4],
        };
        robot.determined[0] = true;

        let last_state = na::SVector::<f64, 9>::zeros();

        Self {
            state,
            last_state,
            dynamic,
            eskf,
            tracker_state: TrackerState::Detecting,
            movement: MovementMode::Static,
            dt,
            last_time: 0.0,
            robot,
            measurement: na::SVector::zeros(),
            max_match_distance: 0.1, // 默认值
            max_match_yaw_diff: 0.2, // 默认值 (弧度)
            lost_threshold: 5,       // 默认值
        }
    }

    pub fn process_frame(&mut self, frame: &Frame) {
        let input = self.state[7];
        // 预测步骤完全依靠动力学模型，不会采纳控制输入，所以随便写一个
        self.last_state = self.state.clone();
        self.dynamic
            .update_nominal_state(&mut self.state, self.dt, &input);

        // ESKF预测
        self.eskf.predict(&self.dynamic, &self.state, &input);

        // ESKF更新
        let mut nominal_state = self.state.clone();
        self.eskf
            .update(&self.dynamic, &mut nominal_state, &frame.measurement);

        // 更新名义状态
        self.state = nominal_state;

        // 应用物理约束 (根据C++中的限幅逻辑)
        if self.state[8] < 0.12 {
            // 半径下限
            self.state[8] = 0.12;
        } else if self.state[8] > 0.4 {
            // 半径上限
            self.state[8] = 0.4;
        }

        // 更新机器人方向
        self.robot.direction = self.state[7].signum(); // v_yaw的符号
    }

    /// 处理装甲板跳变
    pub fn handle_armor_jump(&mut self, current_armor: &Armor) {
        let yaw = self.update_yaw(current_armor.yaw);
        self.state[6] = yaw; // 更新yaw

        self.robot.idx = (self.robot.idx + 1) % 4;
        self.state[4] = current_armor.position.z; // 更新高度
        self.state[8] = self.robot.armor_rs[self.robot.idx]; // 更新半径
    }

    /// 角度归一化处理
    fn update_yaw(&mut self, armor_yaw: f64) -> f64 {
        // 角度归一化逻辑 (根据C++中的normalize函数)
        let normalized = |angle: f64| {
            let result = (angle + std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
            if result <= 0.0 {
                result + std::f64::consts::PI
            } else {
                result - std::f64::consts::PI
            }
        };

        let yaw = normalized(armor_yaw - self.last_yaw()) + self.last_yaw();
        yaw
    }

    /// 从状态获取装甲板位置
    pub fn get_armor_position(&self) -> na::Vector3<f64> {
        let xc = self.state[0];
        let yc = self.state[2];
        let za = self.state[4];
        let yaw = self.state[6] + self.dt * self.state[7]; // 预测角度
        let r = self.state[8];

        na::Vector3::new(xc - r * yaw.cos(), yc - r * yaw.sin(), za)
    }

    pub fn last_yaw(&self) -> f64 {
        self.last_state[6]
    }
}

pub struct Frame {
    measurement: na::Vector4<f64>,
}

impl Frame {
    pub fn new(measurement: na::Vector4<f64>) -> Self {
        Frame { measurement }
    }
}
trait AngleYaw {
    fn nominalize(&self) -> f64;
}

impl AngleYaw for f64 {
    // nominalize f64 to [-PI, PI]
    fn nominalize(&self) -> f64 {
        self.rem_euclid(std::f64::consts::PI * 2.0) - std::f64::consts::PI
    }
}
