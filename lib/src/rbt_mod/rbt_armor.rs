#![allow(unused)]

use nalgebra as na;
use tracing::error;

use crate::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_generic::ImgCoord;

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

    // pub fn image(&self) -> &ImageBuffer<Rgba<u8>, Vec<u8>> {
    //     &self.image
    // }

    pub fn corner_points_na(&self) -> Vec<na::Vector2<f64>> {
        self.corner_points()
            .iter()
            .map(|p| na::Vector2::new(p.x(), p.y()))
            .collect()
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

#[derive(Debug)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Sentry7,
}

impl EnemyId {
    pub fn to_usize(&self) -> usize {
        match self {
            Self::Hero1 => 1_usize,
            Self::Engineer2 => 2_usize,
            Self::Infantry3 => 3_usize,
            Self::Infantry4 => 4_usize,
            Self::Sentry7 => 7_usize,
        }
    }
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

enum ArmorType {
    Small,
    Large,
}

// impl std::fmt::Display for ArmorColor {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         match self {
//             Self::Red => f.write_str("Red"),
//             Self::Blue => f.write_str("Blue"),
//             Self::Gray => f.write_str("Gray"),
//             Self::Purple => f.write_str("Purple"),
//         }
//     }
// }

// impl std::fmt::Display for ArmorType {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         match self {
//             ArmorType::Small => f.write_str("Small"),
//             ArmorType::Large => f.write_str("Large"),
//             ArmorType::Invalid => f.write_str("Invalid"),
//         }
//     }
// }

// #[derive(Debug, Copy, Clone)]
// struct LightBar {
//     light: cv::core::RotatedRect,
//     color: i32,
//     top: na::Vector2<f64>,
//     bottom: na::Vector2<f64>,
//     length: f64,
//     width: f64,
//     tilt_angle: f64, //倾斜角
// }

// pub struct ArmorForDetect {
//     num: ArmorNum,
//     left_light_bar: LightBar,
//     right_light_bar: LightBar,
//     center: na::Point2<f64>,
//     armor_type: ArmorType,
//     number_img: cv::core::Mat,
//     confidence: f64,
//     armor_position: na::Vector3<f64>,
//     position: Vec<na::Point2<f64>>,
//     yaw: f64,
//     dis: f64,
//     distance_to_img_center: f64,
// }

// enum Label {
//     Hero,
//     Sentry,
//     Base,
//     Outpost,
// }

// type ArmorNum = u8;

// struct Armor {
//     num: ArmorNum,
//     armor_type: ArmorType,
//     label: Label,
//     center: na::Point2<f64>,
//     hit_point_r: na::Point2<f64>,
//     hit_point_l: na::Point2<f64>,
//     hit_point_u: na::Point2<f64>,
//     hit_point_d: na::Point2<f64>,
//     armor_r: na::Point2<f64>,
//     armor_l: na::Point2<f64>,
//     armor_u: na::Point2<f64>,
//     armor_d: na::Point2<f64>,
//     position: na::Vector3<f64>,
//     vertex: Vec<na::Point2<f64>>,
// }

// impl Armor {
//     fn new() -> Self {
//         Self {
//             num: 0,
//             armor_type: ArmorType::Small,
//             label: Label::Outpost,
//             center: na::Point2::<f64>::origin(),
//             hit_point_r: na::Point2::<f64>::origin(),
//             hit_point_l: na::Point2::<f64>::origin(),
//             hit_point_u: na::Point2::<f64>::origin(),
//             hit_point_d: na::Point2::<f64>::origin(),
//             armor_r: na::Point2::<f64>::origin(),
//             armor_l: na::Point2::<f64>::origin(),
//             armor_u: na::Point2::<f64>::origin(),
//             armor_d: na::Point2::<f64>::origin(),
//             position: na::Vector3::<f64>::zeros(),
//             vertex: Vec::new(),
//         }
//     }

//     fn get_dis(p: na::Point2<f64>, q: na::Point2<f64>) -> f64 {
//         nalgebra::distance::<f64, 2>(&p, &q)
//     }

//     fn judge_hit(&self, p: na::Point2<f64>) -> bool {
//         let dis = Armor::get_dis(self.vertex[0], self.vertex[1]);
//         let dis_p_center = Armor::get_dis(p, self.center);
//         dis_p_center <= dis
//     }

//     fn from_armor_detect(armor_detected: &ArmorForDetect) {
//         let mut armor = Armor::new();
//         armor.armor_type = ArmorType::Small;
//         //number
//         armor.num = armor_detected.num;
//         match armor_detected.num {
//             1 => armor.armor_type = ArmorType::Large,
//             0 => armor.num = 8,
//             7 => armor.num = 8,
//             6 => armor.num = 3,
//             _ => {}
//         }

//         //pose
//         // vertex.emplace_back(armor.left_light.bottom);
//         // vertex.emplace_back(armor.left_light.top);
//         // vertex.emplace_back(armor.right_light.top);
//         // vertex.emplace_back(armor.right_light.bottom);

//         armor.center = from_cv_point_2_na_point(
//             armor_detected.right_light_bar.light.center
//                 + armor_detected.left_light_bar.light.center,
//         ) / 2.0;
//         armor.hit_point_r = from_cv_point_2_na_point(armor_detected.right_light_bar.light.center)
//             + 4.0 / 5.0
//                 * from_cv_point_2_na_point(
//                     armor_detected.right_light_bar.light.center
//                         - armor_detected.left_light_bar.light.center,
//                 )
//                 .coords;
//         armor.hit_point_l = from_cv_point_2_na_point(armor_detected.left_light_bar.light.center)
//             + 1.0 / 5.0
//                 * from_cv_point_2_na_point(
//                     armor_detected.right_light_bar.light.center
//                         - armor_detected.left_light_bar.light.center,
//                 )
//                 .coords;
//         armor.hit_point_u = armor.center
//             + (armor_detected.left_light_bar.top - armor_detected.left_light_bar.bottom) * 0.8;
//         armor.hit_point_d = armor.center
//             - (armor_detected.left_light_bar.top - armor_detected.left_light_bar.bottom) * 0.8;

//         // armorR = armor.right_light.center;
//         // armorL = armor.left_light.center;
//         // armorU = center + (armor.left_light.top - armor.left_light.bottom) * 0.8;
//         // armorD = center - (armor.left_light.top - armor.left_light.bottom) * 0.8;
//         armor.armor_r = from_cv_point_2_na_point(armor_detected.right_light_bar.light.center);
//         armor.armor_l = from_cv_point_2_na_point(armor_detected.left_light_bar.light.center);
//         armor.armor_u = armor.center
//             + (armor_detected.left_light_bar.top - armor_detected.left_light_bar.bottom) * 0.8;
//         armor.armor_d = armor.center
//             - (armor_detected.left_light_bar.top - armor_detected.left_light_bar.bottom) * 0.8;
//     }
// }

// struct ArmorMsg {
//     radius: f64,
//     height: f64,
//     armor_determined: bool,
// }

// impl Default for ArmorMsg {
//     fn default() -> Self {
//         ArmorMsg {
//             radius: 0.2,
//             height: 0.2,
//             armor_determined: false,
//         }
//     }
// }

// // 观测目标的运动信息
// struct RobotMsg {
//     num: ArmorNum,
//     armor_msg: [ArmorMsg; 4],
//     idx: usize,
//     rotate_direction: bool,
// }

// impl RobotMsg {
//     /// 看到一个新的目标，初始化
//     pub fn from_armor(armor: &Armor) -> Self {
//         RobotMsg {
//             num: armor.num,
//             armor_msg: [
//                 ArmorMsg::default(),
//                 ArmorMsg::default(),
//                 ArmorMsg::default(),
//                 ArmorMsg::default(),
//             ],
//             idx: 0,
//             rotate_direction: false,
//         }
//     }

//     fn set_r(&mut self, r: f64) {
//         self.armor_msg[self.idx].radius = r;
//     }

//     fn update_robot_msg(&mut self, armor: &Armor) {
//         self.armor_msg[self.idx].armor_determined = true;
//         self.armor_msg[self.idx].height = armor.position[2];
//     }

//     fn next_armor_idx(&self) -> usize {
//         (self.idx + if self.rotate_direction { 1 } else { 3 }) % 4
//     }

//     fn next_armor_from_idx(&self, i: i32) -> usize {
//         (i as usize + if self.rotate_direction { 1 } else { 3 }) % 4
//     }

//     fn pre_armor_idx(&self) -> usize {
//         (self.idx + if self.rotate_direction { 3 } else { 1 }) % 4
//     }

//     fn pre_armor_from_idx(&self, i: i32) -> usize {
//         (i as usize + if self.rotate_direction { 3 } else { 1 }) % 4
//     }
// }

// fn from_cv_point_2_na_point(p: cv::core::Point_<f32>) -> na::Point2<f64> {
//     na::Point2::<f64>::new(p.x as f64, p.y as f64)
// }
