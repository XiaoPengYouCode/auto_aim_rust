use std::ops::Deref;
use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;

pub(crate) struct TrackedArmor {
    solved_armor: SolvedArmor,
    balalbala: f64
}

impl Deref for TrackedArmor {
    type Target = SolvedArmor;

    fn deref(&self) -> &Self::Target {
        &self.solved_armor
    }
}
