#![allow(unused)]

use crate::rbt_base::rbt_geometry::rbt_point_dev::{RbtImgPoint2, RbtImgPoint2Coord};
use crate::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_enemy_dev::EnemyId;
use crate::rbt_mod::rbt_generic::ImgCoord;
use nalgebra as na;
use tracing::error;
pub type ArmorId = EnemyId;

#[derive(Debug, Clone)]
pub struct ArmorStaticMsg {
    center: ImgCoord,
    left_top: ImgCoord,
    left_bottom: ImgCoord,
    right_bottom: ImgCoord,
    right_top: ImgCoord,
}

impl ArmorStaticMsg {
    pub fn new(
        center: ImgCoord,
        left_top: ImgCoord,
        left_bottom: ImgCoord,
        right_bottom: ImgCoord,
        right_top: ImgCoord,
        // image: ImageBuffer<Rgba<u8>, Vec<u8>>,
    ) -> Self {
        ArmorStaticMsg {
            center,
            left_top,
            right_top,
            left_bottom,
            right_bottom,
            // image,
        }
    }

    pub fn center(&self) -> ImgCoord {
        self.center
    }

    pub fn left_top(&self) -> ImgCoord {
        self.left_top
    }

    pub fn right_top(&self) -> ImgCoord {
        self.right_top
    }

    pub fn left_bottom(&self) -> ImgCoord {
        self.left_bottom
    }

    pub fn right_bottom(&self) -> ImgCoord {
        self.right_bottom
    }

    pub fn corner_points(&self) -> Vec<ImgCoord> {
        vec![
            self.left_top,
            self.left_bottom,
            self.right_bottom,
            self.right_top,
        ]
    }

    /// 给部分点一个z轴高度 1e-6 给共面点加一个小小的扰动，提高数值稳定性
    pub fn corner_points_na(&self) -> Vec<na::Point2<f64>> {
        self.corner_points()
            .iter()
            .enumerate()
            .map(|(idx, p)| na::Point2::new(p.x(), p.y()))
            .collect()
    }

    pub fn cornet_points(&self) -> [RbtImgPoint2; 4] {
        let lt = RbtImgPoint2::new(
            self.left_top().x(),
            self.left_top().y(),
            RbtImgPoint2Coord::Screen,
        );
        let lb = RbtImgPoint2::new(
            self.left_bottom().x(),
            self.left_bottom().y(),
            RbtImgPoint2Coord::Screen,
        );
        let rb = RbtImgPoint2::new(
            self.right_bottom().x(),
            self.right_bottom().y(),
            RbtImgPoint2Coord::Screen,
        );
        let rt = RbtImgPoint2::new(
            self.right_top().x(),
            self.right_top().y(),
            RbtImgPoint2Coord::Screen,
        );
        let points = [lt, lb, rb, rt];
        points
    }

    pub fn fmt(&self) -> String {
        self.center().x().to_string()
    }
}

#[derive(Debug)]
pub enum ArmorClass {
    Red(u8),
    Blue(u8),
}

impl ArmorClass {
    pub fn from_yolo_output_idx(idx: usize) -> RbtResult<Self> {
        match idx {
            0..=17 => Ok(Self::Blue((idx) as u8)),
            18..=35 => Ok(Self::Red((idx - 17) as u8)),
            _ => {
                error!("Invalid armor class index: {}", idx);
                Err(RbtError::InvalidArmorClassIndex(idx))
            }
        }
    }

    fn armor_color(&self) -> ArmorColor {
        match self {
            ArmorClass::Blue(_) => ArmorColor::Blue,
            ArmorClass::Red(_) => ArmorColor::Red,
        }
    }

    fn armor_type(&self) -> ArmorType {
        match self {
            ArmorClass::Blue(idx) | ArmorClass::Red(idx) => {
                if *idx == 1 {
                    ArmorType::Large
                } else {
                    ArmorType::Small
                }
            }
        }
    }
}

impl std::fmt::Display for ArmorClass {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ArmorClass::Blue(u8) => write!(f, "Blue{}", u8),
            ArmorClass::Red(u8) => write!(f, "Red{}", u8),
        }
    }
}

pub struct ArmorPointMsg {
    center: ImgCoord,
    left_top: ImgCoord,
    left_bottom: ImgCoord,
    right_bottom: ImgCoord,
    right_top: ImgCoord,
}

impl ArmorPointMsg {
    pub fn new(
        center: ImgCoord,
        left_top: ImgCoord,
        left_bottom: ImgCoord,
        right_bottom: ImgCoord,
        right_top: ImgCoord,
    ) -> Self {
        ArmorPointMsg {
            center,
            left_top,
            right_top,
            left_bottom,
            right_bottom,
        }
    }

    pub fn from_vec(coords: Vec<ImgCoord>) -> Self {
        Self {
            center: coords[0],
            left_top: coords[1],
            right_top: coords[2],
            left_bottom: coords[3],
            right_bottom: coords[4],
        }
    }

    pub fn center(&self) -> ImgCoord {
        self.center
    }

    pub fn left_top(&self) -> ImgCoord {
        self.left_top
    }

    pub fn right_top(&self) -> ImgCoord {
        self.right_top
    }

    pub fn left_bottom(&self) -> ImgCoord {
        self.left_bottom
    }

    pub fn right_bottom(&self) -> ImgCoord {
        self.right_bottom
    }
}

pub struct ArmorRaceMsg {
    armor_class: ArmorClass,
}

pub enum ArmorColor {
    Red,
    Blue,
}

pub enum ArmorType {
    Small,
    Large,
}

impl ArmorType {
    /// 小装甲板灯条关键点尺寸，用于输入 pnp
    pub const SMALL_ARMOR_POINT3: [na::Point3<f64>; 4] = [
        na::Point3::new(-135.0 / 2.0, 55.0 / 2.0, 5.1),
        na::Point3::new(-135.0 / 2.0, -55.0 / 2.0, 5.2),
        na::Point3::new(135.0 / 2.0, -55.0 / 2.0, 5.3),
        na::Point3::new(135.0 / 2.0, 55.0 / 2.0, 5.4),
    ];

    /// 大装甲板灯条关键点尺寸，用于输入 pnp
    pub const LARGE_ARMOR_POINT3: [na::Point3<f64>; 4] = [
        na::Point3::new(-135.0 / 2.0, 55.0 / 2.0, 1e-6),
        na::Point3::new(-135.0 / 2.0, -55.0 / 2.0, 1e-6),
        na::Point3::new(135.0 / 2.0, -55.0 / 2.0, 1e-6),
        na::Point3::new(135.0 / 2.0, 55.0 / 2.0, 1e-6),
    ];

    pub fn armor_corner_points(&self) -> [na::Point3<f64>; 4] {
        match self {
            Self::Large => Self::LARGE_ARMOR_POINT3,
            Self::Small => Self::SMALL_ARMOR_POINT3,
        }
    }
}

impl std::fmt::Display for ArmorColor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Red => f.write_str("Red"),
            Self::Blue => f.write_str("Blue"),
        }
    }
}

impl std::fmt::Display for ArmorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ArmorType::Small => f.write_str("Small"),
            ArmorType::Large => f.write_str("Large"),
        }
    }
}
