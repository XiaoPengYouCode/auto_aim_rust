use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
use std::ops::Deref;

#[derive(Debug, Clone)]
pub(crate) struct TrackedArmor {
    solved_armor: SolvedArmor,
    balalbala: f64,
}

// 在 tracked_armor.rs 中添加
impl TrackedArmor {
    pub fn new(solved_armor: SolvedArmor, balalbala: f64) -> Self {
        TrackedArmor {
            solved_armor,
            balalbala,
        }
    }
}

impl Deref for TrackedArmor {
    type Target = SolvedArmor;

    fn deref(&self) -> &Self::Target {
        &self.solved_armor
    }
}
