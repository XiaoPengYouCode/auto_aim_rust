//! 角度相关的扩展 trait
//!
//! 为 f32 类型添加角度归一化功能

/// 角度 trait，提供归一化方法
pub trait Angle {
    /// 将角度归一化到 0-360 度范围
    fn norm_deg(&self) -> Self;
    /// 将弧度归一化到 0-2π 范围
    fn norm_rad(&self) -> Self;
}

impl Angle for f32 {
    fn norm_deg(&self) -> Self {
        // 处理负数角度，确保结果在 [0, 360) 范围内
        (self % 360.0 + 360.0) % 360.0
    }

    fn norm_rad(&self) -> Self {
        // 处理负数弧度，确保结果在 [0, 2π) 范围内
        const TWO_PI: f32 = std::f32::consts::PI * 2.0;
        (self % TWO_PI + TWO_PI) % TWO_PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_norm_deg_positive() {
        let angle = 450.0_f32;
        assert_eq!(angle.norm_deg(), 90.0);
    }

    #[test]
    fn test_norm_deg_negative() {
        let angle = -90.0_f32;
        assert_eq!(angle.norm_deg(), 270.0);
    }

    #[test]
    fn test_norm_rad_positive() {
        let angle = 3.0 * std::f32::consts::PI;
        assert!((angle.norm_rad() - std::f32::consts::PI).abs() < 1e-6);
    }

    #[test]
    fn test_norm_rad_negative() {
        let angle = -std::f32::consts::PI;
        assert!((angle.norm_rad() - std::f32::consts::PI).abs() < 1e-6);
    }
}
