use crate::rbt_mod::{generic_def::armor::ArmorStaticMsg, rbt_estimator::RbtMovementState};

pub struct ArmorSelector {
    armors: Vec<ArmorStaticMsg>,
    rbt_state: RbtMovementState,
}

impl ArmorSelector {
    pub fn from_armors(armors: Vec<ArmorStaticMsg>) -> ArmorSelector {
        ArmorSelector {
            armors,
            rbt_state: RbtMovementState::Static,
        }
    }

    pub fn select(&self) -> usize {
        let mut scores = Vec::new();
        for (i, armor) in self.armors.iter().enumerate() {
            scores.push((
                handle_movement_state(&self.rbt_state, &armor.cal_score()),
                i,
            ));
        }
        scores
            .iter()
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
            .unwrap()
            .1
    }
}

pub struct ArmorSelectScore {
    angle_score: f64,
    history_score: f64,
    jump_score: f64,
}

impl ArmorSelectScore {}

pub fn handle_movement_state(state: &RbtMovementState, score: &ArmorSelectScore) -> f64 {
    match state {
        RbtMovementState::Static => score.angle_score + score.history_score + score.jump_score,
        _ => 0.0_f64,
    }
}

trait ScoredArmor {
    fn cal_score(&self) -> ArmorSelectScore;
}

impl ScoredArmor for ArmorStaticMsg {
    fn cal_score(&self) -> ArmorSelectScore {
        ArmorSelectScore {
            angle_score: (self.center().x() / self.center().y()).atan(),
            history_score: self.center().y(),
            jump_score: self.center().x(),
        }
    }
}
