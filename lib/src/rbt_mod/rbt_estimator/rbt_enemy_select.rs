use crate::rbt_infra::rbt_cfg::EstimatorCfg;
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyId;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};

pub(super) const TRACKED_ENEMY_IDS: [EnemyId; 6] = [
    EnemyId::Hero1,
    EnemyId::Engineer2,
    EnemyId::Infantry3,
    EnemyId::Infantry4,
    EnemyId::Sentry7,
    EnemyId::Outpost8,
];

const DEFAULT_IMAGE_CENTER_X: f64 = 320.0;
const DEFAULT_IMAGE_CENTER_Y: f64 = 192.0;

#[derive(Debug, Clone)]
enum EnemySelectState {
    Idle,
    Locked {
        enemy_id: EnemyId,
    },
    Lost {
        enemy_id: EnemyId,
        time_stamp: tokio::time::Instant,
    },
}

/// Selects one enemy from all enemies visible in the current frame.
#[derive(Debug, Clone)]
pub(super) struct EnemySelectHandler {
    state: EnemySelectState,
    image_center: na::Point2<f64>,
}

impl EnemySelectHandler {
    fn new(image_center: na::Point2<f64>) -> Self {
        Self {
            state: EnemySelectState::Idle,
            image_center,
        }
    }

    pub(super) fn select(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemies: &RbtSolvedResults,
    ) -> Option<EnemyId> {
        match self.state.clone() {
            EnemySelectState::Idle => self.lock_closest_visible(solved_enemies),
            EnemySelectState::Locked { enemy_id } => {
                if Self::is_visible(solved_enemies, enemy_id) {
                    Some(enemy_id)
                } else {
                    self.state = EnemySelectState::Lost {
                        enemy_id,
                        time_stamp: tokio::time::Instant::now(),
                    };
                    Some(enemy_id)
                }
            }
            EnemySelectState::Lost {
                enemy_id,
                time_stamp,
            } => {
                if Self::is_visible(solved_enemies, enemy_id) {
                    self.state = EnemySelectState::Locked { enemy_id };
                    Some(enemy_id)
                } else if time_stamp.elapsed() <= cfg.enemy_lost_wait_duration_ms() {
                    Some(enemy_id)
                } else {
                    self.lock_closest_visible(solved_enemies)
                }
            }
        }
    }

    pub(super) fn selected_enemy_id(&self) -> Option<EnemyId> {
        match self.state {
            EnemySelectState::Idle => None,
            EnemySelectState::Locked { enemy_id } | EnemySelectState::Lost { enemy_id, .. } => {
                Some(enemy_id)
            }
        }
    }

    fn lock_closest_visible(&mut self, solved_enemies: &RbtSolvedResults) -> Option<EnemyId> {
        let enemy_id = self.closest_visible_enemy(solved_enemies);
        self.state = match enemy_id {
            Some(enemy_id) => EnemySelectState::Locked { enemy_id },
            None => EnemySelectState::Idle,
        };
        enemy_id
    }

    fn closest_visible_enemy(&self, solved_enemies: &RbtSolvedResults) -> Option<EnemyId> {
        let mut best: Option<(EnemyId, f64)> = None;

        for enemy_id in TRACKED_ENEMY_IDS {
            let Some(Some(solved_enemy)) = solved_enemies.get(&enemy_id) else {
                continue;
            };
            let Some(score) = self.enemy_center_score(solved_enemy) else {
                continue;
            };

            if best.is_none_or(|(_, best_score)| score < best_score) {
                best = Some((enemy_id, score));
            }
        }

        best.map(|(enemy_id, _)| enemy_id)
    }

    fn enemy_center_score(&self, solved_enemy: &RbtSolvedResult) -> Option<f64> {
        solved_enemy
            .armors
            .iter()
            .map(|armor| {
                let center = armor.center();
                let dx = center.x - self.image_center.x;
                let dy = center.y - self.image_center.y;
                dx * dx + dy * dy
            })
            .min_by(|a, b| a.total_cmp(b))
    }

    fn is_visible(solved_enemies: &RbtSolvedResults, enemy_id: EnemyId) -> bool {
        solved_enemies
            .get(&enemy_id)
            .is_some_and(|solved_enemy| solved_enemy.is_some())
    }
}

impl Default for EnemySelectHandler {
    fn default() -> Self {
        Self::new(na::Point2::new(
            DEFAULT_IMAGE_CENTER_X,
            DEFAULT_IMAGE_CENTER_Y,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
    use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;
    use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
    use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
    use na::Isometry3;
    use tokio::time::Duration;

    fn estimator_cfg(enemy_lost_wait_duration_ms: u64) -> EstimatorCfg {
        toml::from_str(&format!(
            "\
armor_lost_wait_duration_ms = 100
enemy_lost_wait_duration_ms = {enemy_lost_wait_duration_ms}
"
        ))
        .unwrap()
    }

    fn solved_enemy(center_x: f32, center_y: f32) -> RbtSolvedResult {
        let detected_armor = DetectedArmor::new(
            RbtImgPoint2::new_screen_pixel(center_x, center_y),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y - 5.0),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y - 5.0),
            0,
        );

        RbtSolvedResult {
            coord: RbtCylindricalPoint2::new(1_000.0, 0.0),
            armors: vec![SolvedArmor::new(
                detected_armor,
                Isometry3::identity(),
                0.0,
                0.0,
                200.0,
            )],
        }
    }

    fn frame(targets: &[(EnemyId, (f32, f32))]) -> RbtSolvedResults {
        let mut solved_enemies = RbtSolvedResults::default();
        for (enemy_id, (x, y)) in targets {
            solved_enemies.insert(*enemy_id, Some(solved_enemy(*x, *y)));
        }
        solved_enemies
    }

    fn selector() -> EnemySelectHandler {
        EnemySelectHandler::new(na::Point2::new(320.0, 192.0))
    }

    #[test]
    fn returns_none_when_no_enemy_is_visible() {
        let cfg = estimator_cfg(1_000);
        let mut selector = selector();

        let selected = selector.select(&cfg, &RbtSolvedResults::default());

        assert_eq!(selected, None);
        assert_eq!(selector.selected_enemy_id(), None);
    }

    #[test]
    fn locks_closest_enemy_on_first_visible_frame() {
        let cfg = estimator_cfg(1_000);
        let mut selector = selector();
        let solved_enemies = frame(&[
            (EnemyId::Hero1, (520.0, 192.0)),
            (EnemyId::Infantry3, (322.0, 193.0)),
            (EnemyId::Sentry7, (260.0, 192.0)),
        ]);

        let selected = selector.select(&cfg, &solved_enemies);

        assert_eq!(selected, Some(EnemyId::Infantry3));
        assert_eq!(selector.selected_enemy_id(), Some(EnemyId::Infantry3));
    }

    #[test]
    fn keeps_locked_enemy_while_it_is_still_visible() {
        let cfg = estimator_cfg(1_000);
        let mut selector = selector();
        selector.state = EnemySelectState::Locked {
            enemy_id: EnemyId::Hero1,
        };
        let solved_enemies = frame(&[
            (EnemyId::Hero1, (600.0, 192.0)),
            (EnemyId::Infantry3, (320.0, 192.0)),
        ]);

        let selected = selector.select(&cfg, &solved_enemies);

        assert_eq!(selected, Some(EnemyId::Hero1));
        assert_eq!(selector.selected_enemy_id(), Some(EnemyId::Hero1));
    }

    #[test]
    fn holds_lost_enemy_before_timeout_even_when_another_enemy_is_visible() {
        let cfg = estimator_cfg(1_000);
        let mut selector = selector();
        selector.state = EnemySelectState::Locked {
            enemy_id: EnemyId::Hero1,
        };

        let selected = selector.select(&cfg, &frame(&[(EnemyId::Infantry3, (320.0, 192.0))]));

        assert_eq!(selected, Some(EnemyId::Hero1));
        assert_eq!(selector.selected_enemy_id(), Some(EnemyId::Hero1));
        assert!(matches!(selector.state, EnemySelectState::Lost { .. }));
    }

    #[test]
    fn relocks_lost_enemy_when_it_reappears_before_timeout() {
        let cfg = estimator_cfg(1_000);
        let mut selector = selector();
        selector.state = EnemySelectState::Lost {
            enemy_id: EnemyId::Hero1,
            time_stamp: tokio::time::Instant::now(),
        };

        let selected = selector.select(
            &cfg,
            &frame(&[
                (EnemyId::Hero1, (610.0, 192.0)),
                (EnemyId::Infantry3, (320.0, 192.0)),
            ]),
        );

        assert_eq!(selected, Some(EnemyId::Hero1));
        assert!(matches!(
            selector.state,
            EnemySelectState::Locked {
                enemy_id: EnemyId::Hero1
            }
        ));
    }

    #[test]
    fn switches_after_lost_timeout() {
        let cfg = estimator_cfg(1);
        let mut selector = selector();
        selector.state = EnemySelectState::Lost {
            enemy_id: EnemyId::Hero1,
            time_stamp: tokio::time::Instant::now() - Duration::from_millis(2),
        };

        let selected = selector.select(&cfg, &frame(&[(EnemyId::Infantry3, (320.0, 192.0))]));

        assert_eq!(selected, Some(EnemyId::Infantry3));
        assert_eq!(selector.selected_enemy_id(), Some(EnemyId::Infantry3));
    }
}
