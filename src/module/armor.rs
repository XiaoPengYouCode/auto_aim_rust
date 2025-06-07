// #include <opencv2/opencv.hpp>
// #include <eigen3/Eigen/Eigen>
// #include <memory>
// #include <string>
// //#include <map>
// use nalgebra::{self as na, zero};
// use opencv as cv;
//
// use cv::core::CV_PI;

use image::{ImageBuffer, Rgba};

pub const ARMOR_LIGHT_BAR_SIZE: f64 = 10.0;

pub type ImgCoordinate = (f32, f32);

#[derive(Debug)]
pub struct ArmorStaticMsg {
    center: ImgCoordinate,
    left_top: ImgCoordinate,
    left_bottom: ImgCoordinate,
    right_bottom: ImgCoordinate,
    right_top: ImgCoordinate,
    image: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

impl ArmorStaticMsg {
    pub fn new(
        center: ImgCoordinate,
        left_top: ImgCoordinate,
        left_bottom: ImgCoordinate,
        right_bottom: ImgCoordinate,
        right_top: ImgCoordinate,
        image: ImageBuffer<Rgba<u8>, Vec<u8>>,
    ) -> Self {
        ArmorStaticMsg {
            center,
            left_top,
            right_top,
            left_bottom,
            right_bottom,
            image,
        }
    }

    pub fn center(&self) -> ImgCoordinate {
        self.center
    }

    pub fn left_top(&self) -> ImgCoordinate {
        self.left_top
    }

    pub fn right_top(&self) -> ImgCoordinate {
        self.right_top
    }

    pub fn left_bottom(&self) -> ImgCoordinate {
        self.left_bottom
    }

    pub fn right_bottom(&self) -> ImgCoordinate {
        self.right_bottom
    }

    pub fn corner_points(&self) -> Vec<ImgCoordinate> {
        vec![
            self.left_top,
            self.right_top,
            self.right_bottom,
            self.left_bottom,
        ]
    }

    pub fn image(&self) -> &ImageBuffer<Rgba<u8>, Vec<u8>> {
        &self.image
    }
}

// const OUTPOST_ROTATE_RL: f64 = 553.0;
// 目标运动模式，用于辅助响应
pub enum MovementMethod {
    Static(Vec<ArmorStaticMsg>), // 静止状态
    Trans,                       // 横移
    Spin,                        // 旋转
    TransSpin,                   // 平移+旋转
}

// enum ArmorColor {
//     Red,
//     Blue,
//     Gray,   // 熄灭灯条
//     Purple, // 无敌灯条
// }

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

// const D2R: f64 = CV_PI / 180.0;
// const R2D: f64 = 180.0 / CV_PI;

// struct AngleWithTime {
//     angle: f64, // 角度
//     t: f64,     // 秒
// }

// struct PosWithTime {
//     pos: na::Vector3<f64>, // 三维坐标/m
//     t: f64,                // 时间戳/秒
// }

// enum ArmorType {
//     Small,
//     Large,
//     Invalid,
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
