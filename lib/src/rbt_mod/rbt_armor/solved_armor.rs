use std::ops::Deref;
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;

pub(crate) struct SolvedArmor {
    detected_armor: DetectedArmor,
    enemy_yaw: f64,
    base_yaw: f64
}

impl Deref for SolvedArmor {
    type Target = DetectedArmor;

    fn deref(&self) -> &Self::Target {
        &self.detected_armor
    }
}
