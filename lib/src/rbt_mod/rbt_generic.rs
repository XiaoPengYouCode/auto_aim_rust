#[derive(Debug, Copy, Clone)]
pub struct ImgCoord(f64, f64);

impl ImgCoord {
    pub fn to_f32(&self) -> (f32, f32) {
        (self.0 as f32, self.1 as f32)
    }

    pub fn from_f32(x: f32, y: f32) -> Self {
        Self(x as f64, y as f64)
    }

    pub fn x(&self) -> f64 {
        self.0
    }
    pub fn y(&self) -> f64 {
        self.1
    }
}
