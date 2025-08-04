use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;

/// 作为 Detector 的输出和 Solver 的输入
#[derive(Debug, Clone)]
pub struct DetectedArmor {
    key_points: [RbtImgPoint2; 5],
    id: usize, // 当前帧画面唯一 id，用于区分每一块装甲板
}

impl DetectedArmor {
    pub fn new(
        center: RbtImgPoint2,
        lt: RbtImgPoint2,
        lb: RbtImgPoint2,
        rb: RbtImgPoint2,
        rt: RbtImgPoint2,
        id: usize,
    ) -> Self {
        DetectedArmor {
            key_points: [center, lt, lb, rb, rt],
            id,
        }
    }

    #[inline(always)]
    pub fn center(&self) -> RbtImgPoint2 {
        self.key_points[0]
    }

    #[inline(always)]
    pub fn lt(&self) -> RbtImgPoint2 {
        self.key_points[1]
    }

    #[inline(always)]
    pub fn lb(&self) -> RbtImgPoint2 {
        self.key_points[2]
    }

    #[inline(always)]
    pub fn rb(&self) -> RbtImgPoint2 {
        self.key_points[3]
    }

    #[inline(always)]
    pub fn rt(&self) -> RbtImgPoint2 {
        self.key_points[4]
    }

    pub fn cornet_points(&self) -> [RbtImgPoint2; 4] {
        [self.lt(), self.lb(), self.rb(), self.rt()]
    }
}
