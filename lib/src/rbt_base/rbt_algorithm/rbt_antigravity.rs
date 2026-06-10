/* 重力补偿算法 */

/// RoboMaster 场地重力加速度近似值 (m/s^2)。
///
/// 与参考工程 `tools::Trajectory`/`Planner::BallisticTrajectory` 保持一致。
pub const GRAVITY_MPS2: f64 = 9.7833;

const EPSILON: f64 = 1e-6;

/// 不考虑空气阻力的弹道解。
///
/// `pitch_rad` 为发射仰角，抬头为正；`fly_time_s` 为命中目标所需飞行时间。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallisticSolution {
    pub pitch_rad: f64,
    pub fly_time_s: f64,
}

impl BallisticSolution {
    pub fn pitch_deg(self) -> f64 {
        self.pitch_rad.to_degrees()
    }
}

/// 使用无空气阻力抛物线模型求低弧线弹道。
///
/// - `bullet_speed_mps`: 弹速，单位 m/s
/// - `horizontal_distance_m`: 目标水平距离，单位 m
/// - `height_m`: 目标相对枪口高度，单位 m，向上为正
pub fn solve_ballistic_trajectory(
    bullet_speed_mps: f64,
    horizontal_distance_m: f64,
    height_m: f64,
) -> Result<BallisticSolution, &'static str> {
    if !bullet_speed_mps.is_finite() || bullet_speed_mps <= EPSILON {
        return Err("弹丸速度必须为正数");
    }
    if !horizontal_distance_m.is_finite() || horizontal_distance_m <= EPSILON {
        return Err("目标距离必须为正数");
    }
    if !height_m.is_finite() {
        return Err("目标高度必须为有限值");
    }

    let a = GRAVITY_MPS2 * horizontal_distance_m * horizontal_distance_m
        / (2.0 * bullet_speed_mps * bullet_speed_mps);
    let b = -horizontal_distance_m;
    let c = a + height_m;
    let delta = b * b - 4.0 * a * c;

    if delta < 0.0 {
        return Err("目标超出当前弹速可达范围");
    }

    let sqrt_delta = delta.sqrt();
    let tan_pitch_1 = (-b + sqrt_delta) / (2.0 * a);
    let tan_pitch_2 = (-b - sqrt_delta) / (2.0 * a);
    let pitch_1 = tan_pitch_1.atan();
    let pitch_2 = tan_pitch_2.atan();
    let time_1 = horizontal_distance_m / (bullet_speed_mps * pitch_1.cos());
    let time_2 = horizontal_distance_m / (bullet_speed_mps * pitch_2.cos());

    let (pitch_rad, fly_time_s) = if time_1 < time_2 {
        (pitch_1, time_1)
    } else {
        (pitch_2, time_2)
    };

    if !pitch_rad.is_finite() || !fly_time_s.is_finite() || fly_time_s <= 0.0 {
        return Err("弹道解无效");
    }

    Ok(BallisticSolution {
        pitch_rad,
        fly_time_s,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_low_arc_for_level_target() {
        let solution = solve_ballistic_trajectory(20.0, 10.0, 0.0).unwrap();

        assert!((0.0..10.0).contains(&solution.pitch_deg()));
        assert!((0.4..0.6).contains(&solution.fly_time_s));
    }

    #[test]
    fn returns_error_when_target_is_unreachable() {
        let result = solve_ballistic_trajectory(5.0, 100.0, 0.0);

        assert!(result.is_err());
    }

    #[test]
    fn rejects_invalid_distance() {
        let result = solve_ballistic_trajectory(20.0, -5.0, 0.0);

        assert_eq!(result.err().unwrap(), "目标距离必须为正数");
    }

    #[test]
    fn rejects_invalid_bullet_speed() {
        let result = solve_ballistic_trajectory(0.0, 10.0, 0.0);

        assert_eq!(result.err().unwrap(), "弹丸速度必须为正数");
    }

    #[test]
    fn solves_target_far_below_muzzle() {
        let solution = solve_ballistic_trajectory(20.0, 10.0, -5.0).unwrap();

        assert!(solution.pitch_deg() < 0.0);
        assert!(solution.fly_time_s > 0.0);
    }
}
