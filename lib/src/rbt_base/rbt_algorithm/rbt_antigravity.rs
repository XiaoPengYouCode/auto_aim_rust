/* 重力补偿算法 */

/// 重力加速度常数 (m/s²)
const GRAVITY: f64 = 9.81;

/// 弹丸发射重力补偿算法
///
/// 根据发射角度(pitch)、目标距离(distance)和弹丸初速度(bullet_speed)计算需要补偿的角度
/// 算法基于抛物线轨迹和简化的空气阻力模型（阻力与速度的平方成正比）
///
/// # 参数
/// - `pitch`: 发射仰角（角度制，正数表示向上）
/// - `distance`: 水平距离到目标（米）
/// - `bullet_speed`: 弹丸初速度（米/秒）
///
/// # 返回值
/// - `Ok(adj_pitch)`: 补偿后的发射仰角（角度制）
/// - `Err(&str)`: 错误信息
///
/// # 错误处理
/// 当输入参数不合理或计算过程中出现数学错误时，会返回错误信息
pub fn calculate_compensated_pitch(
    pitch_deg: f64,
    distance_mm: f64,
    bullet_speed_mps: f64,
) -> Result<f64, &'static str> {
    // 参数校验
    if distance_mm <= 0.0 {
        return Err("目标距离必须为正数");
    }

    if bullet_speed_mps <= 0.0 {
        return Err("弹丸速度必须为正数");
    }

    // 将角度转换为弧度
    let pitch_rad = pitch_deg.to_radians();

    // 计算飞行时间的初始估计
    let time_of_flight = distance_mm / (bullet_speed_mps * pitch_rad.cos());

    // 计算目标高度
    let target_height = distance_mm * pitch_rad.tan();

    // 简化的阻力因子，这里使用一个经验系数
    // 系数可以根据实际测试进行调整
    let drag_coefficient = 0.01;

    // 计算重力影响下的补偿角度
    let gravity_compensation = (GRAVITY * time_of_flight * time_of_flight)
        / (2.0 * bullet_speed_mps * time_of_flight * pitch_rad.cos());

    // 计算阻力影响（简化模型，阻力与速度平方成正比）
    let drag_effect = drag_coefficient * time_of_flight * time_of_flight * bullet_speed_mps;

    // 计算补偿角度（弧度）
    let compensation_rad = gravity_compensation + drag_effect;

    // 转换回角度
    let compensation_deg = compensation_rad.to_degrees();
    if compensation_deg < 0.0 {
        return Err("补偿角度不能小于0度");
    }

    // 计算最终角度
    let mut adj_pitch = pitch_deg + compensation_deg;

    // 对补偿后的角度进行验证
    // 限制最大仰角为75度，避免过于极端的角度
    if adj_pitch > 75.0 {
        adj_pitch = 75.0;
    }

    // 限制最小仰角为-45度
    if adj_pitch < -45.0 {
        adj_pitch = -45.0;
    }

    Ok(adj_pitch)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_conditions() {
        let result = calculate_compensated_pitch(0.0, 10.0, 20.0);
        assert!(result.is_ok());
        let adj_pitch = result.unwrap();
        // 在这个例子中，补偿角度应该为正，因为需要克服重力
        assert!(adj_pitch >= 0.0);
    }

    #[test]
    fn test_negative_pitch() {
        let result = calculate_compensated_pitch(-10.0, 10.0, 20.0);
        assert!(result.is_ok());
    }

    #[test]
    fn test_invalid_distance() {
        let result = calculate_compensated_pitch(0.0, -5.0, 20.0);
        assert!(result.is_err());
        assert_eq!(result.err().unwrap(), "目标距离必须为正数");
    }

    #[test]
    fn test_invalid_bullet_speed() {
        let result = calculate_compensated_pitch(0.0, 10.0, 0.0);
        assert!(result.is_err());
        assert_eq!(result.err().unwrap(), "弹丸速度必须为正数");
    }

    #[test]
    fn test_extreme_pitch_up() {
        let result = calculate_compensated_pitch(80.0, 10.0, 20.0);
        assert!(result.is_ok());
        // 即使输入角度很大，输出也应该被限制在75度以内
        assert!(result.unwrap() <= 75.0);
    }

    #[test]
    fn test_extreme_pitch_down() {
        let result = calculate_compensated_pitch(-50.0, 10.0, 20.0);
        assert!(result.is_ok());
        // 即使输入角度很小，输出也不应该低于-45度
        assert!(result.unwrap() >= -45.0);
    }
}
