pub struct RbtCylindricalCoord2 {
    pub dist: f64, // 相对与世界坐标系原点的距离
    pub angle_yaw_d: f64, // 在世界坐标系下的角度
}

impl RbtCylindricalCoord2 {
    pub fn new(dist: f64, angle_yaw_d: f64) -> Self {
        Self { dist, angle_yaw_d }
    }

    pub fn from_xy(xy: impl Into<na::Point2<f64>>) -> Self {
        let p = xy.into();
        let (x, y) = (p.x, p.y);
        let distance = (x * x + y * y).sqrt();
        let yaw = y / x
            .atan()
            .to_degrees();
        Self::new(distance, yaw)
    }
}
