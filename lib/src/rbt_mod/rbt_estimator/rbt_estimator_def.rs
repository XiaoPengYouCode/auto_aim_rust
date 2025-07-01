// 使用类型别名提高代码可读性
pub type State = na::SVector<f64, 9>; // xc, v_xc, yc, v_yc, za, v_za, orient, v_yaw, r
pub type Observation = na::SVector<f64, 4>; // yaw, pitch, dis, orientation_yaw

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TopLevel {
    Stationary,  // 等级 0: 基本跟随
    IndirectAim, // 等级 1: 开启间接瞄准
    HighSpeed,   // 等级 2: 超高速模式
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShootMode {
    Idle,
    Tracking,
    ShootNow,
}

#[derive(Debug, Clone, Copy)]
pub struct YpdCoord {
    pub yaw: f64,
    pub pitch: f64,
    pub dis: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ShootParam {
    pub v0: f64,
    pub aim_angle: f64,
    // ... 其他字段
}

#[derive(Debug, Clone)]
pub struct AimInfo {
    pub ypd: YpdCoord,
    pub ypd_v: YpdCoord,
    pub shoot_param: ShootParam,
    pub shoot_mode: ShootMode,
}

impl AimInfo {
    pub fn idle() -> Self {
        Self {
            ypd: YpdCoord {
                yaw: 0.0,
                pitch: 0.0,
                dis: 0.0,
            },
            ypd_v: YpdCoord {
                yaw: 0.0,
                pitch: 0.0,
                dis: 0.0,
            },
            shoot_param: ShootParam {
                v0: 0.0,
                aim_angle: 0.0,
            },
            shoot_mode: ShootMode::Idle,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AimAndState {
    pub aim: AimInfo,
    pub state: State,
}
