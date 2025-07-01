// Control module for the robot.

use crate::rbt_mod::generic_def::armor::ArmorStaticMsg;

mod armor_select;

pub struct RbtController {
    // armor_choice
    armor_select: armor_select::ArmorSelector,
    dt: std::time::Duration,
}

impl RbtController {
    pub fn new(dt: std::time::Duration, armors: &[ArmorStaticMsg]) -> Self {
        RbtController {
            armor_select: armor_select::ArmorSelector::from_armors(armors.to_owned()),
            dt,
        }
    }

    pub fn dt(&self) -> &std::time::Duration {
        &self.dt
    }

    /// fanhuijidazhuangjiaban index
    pub fn select_armor(&self) -> usize {
        self.armor_select.select()
    }
}
