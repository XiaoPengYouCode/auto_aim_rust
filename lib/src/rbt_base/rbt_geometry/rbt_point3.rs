use std::ops::Deref;

#[derive(Clone, Debug, PartialEq)]
pub enum RbtPoint3CoordSys {
    Camera, // 原点位于相机中心，以相机向前方向为 Z 轴
    Muzzle, // 原点位于测速中心，以向前方向为 Z 轴
    Base,   // 以云台向前方向为 X 轴，向上为 Z 轴的右手坐标系
    World,  // 根据电控端维护的绝对位资
}

/// 3-d point
#[derive(Clone, Debug)]
pub struct RbtPoint3 {
    point: na::Point3<f64>,
    coord_sys: RbtPoint3CoordSys,
}

impl RbtPoint3 {
    pub fn new(x: f64, y: f64, z: f64, coord_sys: RbtPoint3CoordSys) -> Self {
        Self {
            point: na::Point3::new(x, y, z),
            coord_sys,
        }
    }

    pub fn coord_sys(&self) -> &RbtPoint3CoordSys {
        &self.coord_sys
    }

    pub fn from_point(point: na::Point3<f64>, coord_sys: RbtPoint3CoordSys) -> Self {
        (point, coord_sys).into()
    }

    pub fn trans_to(&mut self, target_coord_sys: &RbtPoint3CoordSys) {
        self.point = point_tansfer(&self.point, &self.coord_sys, &target_coord_sys);
        self.coord_sys = target_coord_sys.clone();
    }
}

impl Deref for RbtPoint3 {
    type Target = na::Point3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

// 实现 From trait (核心逻辑)
impl From<(na::Point3<f64>, RbtPoint3CoordSys)> for RbtPoint3 {
    fn from((point, coord_sys): (na::Point3<f64>, RbtPoint3CoordSys)) -> Self {
        Self { point, coord_sys }
    }
}

// 将 RbtPoint3 转换回 nalgebra点 (丢失坐标系信息)
impl From<RbtPoint3> for na::Point3<f64> {
    fn from(rbt_point: RbtPoint3) -> Self {
        rbt_point.point
    }
}

fn point_tansfer(
    point: &na::Point3<f64>,
    source_coord_sys: &RbtPoint3CoordSys,
    target_coord_sys: &RbtPoint3CoordSys,
) -> na::Point3<f64> {
    match (source_coord_sys, target_coord_sys) {
        (_, _) => na::Point3::new(point.x, point.y, point.z),
    }
}
