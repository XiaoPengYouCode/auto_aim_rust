#[derive(Debug, Clone)]
pub struct RbtCylindricalPoint2 {
    pub rho: f64,     // 相对与世界坐标系原点的距离
    pub theta_d: f64, // 在世界坐标系下的角度
}

impl RbtCylindricalPoint2 {
    pub fn new(dist: f64, angle_yaw_d: f64) -> Self {
        Self {
            rho: dist,
            theta_d: angle_yaw_d,
        }
    }

    pub fn from_xy(xy: impl Into<na::Point2<f64>>) -> Self {
        let p = xy.into();
        let (x, y) = (p.x, p.y);
        let rho = (x * x + y * y).sqrt();
        let theta_d = y.atan2(x).to_degrees();
        Self::new(rho, theta_d)
    }

    pub fn to_xy(&self) -> na::Point2<f64> {
        let theta = self.theta_d.to_radians();
        na::Point2::new(self.rho * theta.cos(), self.rho * theta.sin())
    }
}
