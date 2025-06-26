extern crate nalgebra as na;

#[derive(Debug, Clone)]
pub struct RbtImgPoint2 {
    point: na::Point2<f64>,
}

impl RbtImgPoint2 {
    pub fn new(x: f64, y: f64) -> Self {
        RbtImgPoint2 {
            point: na::Point2::new(x, y),
        }
    }

    pub fn x(&self) -> f64 {
        self.point.x
    }

    pub fn y(&self) -> f64 {
        self.point.y
    }

    pub fn from_vec(vec: &[(f64, f64)]) -> Vec<Self> {
        vec.iter().map(|&(x, y)| Self::new(x, y)).collect()
    }

    pub fn point(&self) -> &na::Point2<f64> {
        &self.point
    }
}

impl Into<RbtImgPoint2> for na::Vector2<f64> {
    fn into(self) -> RbtImgPoint2 {
        RbtImgPoint2::new(self.x, self.y)
    }
}

#[derive(Debug, Clone)]
pub struct RbtWorldPoint3 {
    point: na::Point3<f64>,
}

impl RbtWorldPoint3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        RbtWorldPoint3 {
            point: na::Point3::new(x, y, z),
        }
    }

    pub fn x(&self) -> f64 {
        self.point.x
    }

    pub fn y(&self) -> f64 {
        self.point.y
    }

    pub fn z(&self) -> f64 {
        self.point.z
    }

    pub fn point(&self) -> &na::Point3<f64> {
        &self.point
    }

    pub fn from_vec(vec: &[(f64, f64, f64)]) -> Vec<Self> {
        vec.iter().map(|&(x, y, z)| Self::new(x, y, z)).collect()
    }
}

impl Into<RbtWorldPoint3> for na::Vector3<f64> {
    fn into(self) -> RbtWorldPoint3 {
        RbtWorldPoint3::new(self.x, self.y, self.z)
    }
}
