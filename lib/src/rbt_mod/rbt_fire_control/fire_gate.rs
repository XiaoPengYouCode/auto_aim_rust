const FIRE_IMPACT_DIRECTION_EPSILON_RAD_S: f64 = 0.2;

/// Automatic-fire gate parameters aligned with vivsionn's fire control defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FireGateConfig {
    pub yaw_miss_tolerance_m: f64,
    pub yaw_tolerance_min_deg: f64,
    pub yaw_tolerance_max_deg: f64,
    pub armor_impact_enter_angle_deg: f64,
    pub armor_impact_leave_angle_deg: f64,
    pub command_stable_ratio: f64,
    pub fire_rate_hz: f64,
    pub shot_window_pre_ms: f64,
    pub shot_window_post_ms: f64,
    pub model_dt_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactAngleCheck {
    pub delta_deg: f64,
    pub in_window: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotSlotGate<'a> {
    pub predicted_yaw_deg: &'a [f64],
    pub reference_yaw_deg: &'a [f64],
    pub impact_delta_angle_ref_deg: Option<&'a [f64]>,
    pub tolerance_deg: f64,
    pub first_slot_time_s: f64,
    pub target_omega_rad_s: f64,
    pub require_impact_angle_gate: bool,
    pub mcu_fire_permit: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotSlotGateResult {
    pub viable_slot_count: usize,
    pub first_slot_index: Option<usize>,
    pub first_slot_error_deg: Option<f64>,
    pub first_viable_slot_time_s: Option<f64>,
    pub first_slot_impact: Option<ImpactAngleCheck>,
}

impl Default for FireGateConfig {
    fn default() -> Self {
        Self {
            yaw_miss_tolerance_m: 0.055,
            yaw_tolerance_min_deg: 0.9,
            yaw_tolerance_max_deg: 1.5,
            armor_impact_enter_angle_deg: 50.0,
            armor_impact_leave_angle_deg: 30.0,
            command_stable_ratio: 0.9,
            fire_rate_hz: 20.0,
            shot_window_pre_ms: 10.0,
            shot_window_post_ms: 0.0,
            model_dt_s: 0.004,
        }
    }
}

impl FireGateConfig {
    /// Compute the allowed yaw error from target distance.
    ///
    /// This mirrors vivsionn's `fire_yaw_tolerance_deg`: a fixed miss radius is
    /// converted to angular tolerance and clamped by configured bounds.
    pub fn yaw_tolerance_deg(&self, distance_m: f64) -> f64 {
        let default = FireGateConfig::default();
        let min_deg = if self.yaw_tolerance_min_deg.is_finite() {
            self.yaw_tolerance_min_deg
        } else {
            default.yaw_tolerance_min_deg
        };
        let max_deg = if self.yaw_tolerance_max_deg.is_finite() {
            self.yaw_tolerance_max_deg
        } else {
            default.yaw_tolerance_max_deg
        };
        let lower = min_deg.min(max_deg);
        let upper = min_deg.max(max_deg);
        let miss_tolerance_m =
            if self.yaw_miss_tolerance_m.is_finite() && self.yaw_miss_tolerance_m > 0.0 {
                self.yaw_miss_tolerance_m
            } else {
                default.yaw_miss_tolerance_m
            };

        if !distance_m.is_finite() || distance_m <= 1e-3 {
            return upper;
        }

        (miss_tolerance_m / distance_m)
            .atan()
            .to_degrees()
            .clamp(lower, upper)
    }

    /// Check whether the predicted armor impact angle is in the fire window.
    ///
    /// For a fast spinning target the enter side is wider than the leave side,
    /// matching vivsionn's asymmetric window.
    pub fn impact_angle_in_window(&self, impact_delta_angle_rad: f64, omega_rad_s: f64) -> bool {
        if !impact_delta_angle_rad.is_finite() {
            return false;
        }

        let configured_enter_deg = self.armor_impact_enter_angle_deg.max(0.0);
        let configured_leave_deg = self.armor_impact_leave_angle_deg.max(0.0);
        let enter_rad = configured_enter_deg.max(configured_leave_deg).to_radians();
        let leave_rad = configured_enter_deg.min(configured_leave_deg).to_radians();

        if omega_rad_s.abs() <= FIRE_IMPACT_DIRECTION_EPSILON_RAD_S {
            impact_delta_angle_rad.abs() <= enter_rad
        } else if omega_rad_s > 0.0 {
            impact_delta_angle_rad >= -enter_rad && impact_delta_angle_rad <= leave_rad
        } else {
            impact_delta_angle_rad <= enter_rad && impact_delta_angle_rad >= -leave_rad
        }
    }

    pub fn impact_angle_ref_in_window(
        &self,
        impact_delta_angle_ref_deg: &[f64],
        index: usize,
        omega_rad_s: f64,
    ) -> Option<ImpactAngleCheck> {
        let delta_deg = *impact_delta_angle_ref_deg.get(index)?;
        Some(ImpactAngleCheck {
            delta_deg,
            in_window: self.impact_angle_in_window(delta_deg.to_radians(), omega_rad_s),
        })
    }

    /// Max yaw tracking error around a shot slot.
    ///
    /// The yaw trajectories are expected to already be unwrapped, as in
    /// vivsionn's MPC preview output.
    pub fn shot_slot_window_max_error_deg(
        &self,
        predicted_yaw_deg: &[f64],
        reference_yaw_deg: &[f64],
        slot_index: usize,
    ) -> Option<f64> {
        let horizon = predicted_yaw_deg.len().min(reference_yaw_deg.len());
        if horizon <= 1 || !self.model_dt_s.is_finite() || self.model_dt_s <= 0.0 {
            return None;
        }

        let clamped_slot_index = slot_index.clamp(1, horizon - 1);
        let pre_steps = ((self.shot_window_pre_ms.max(0.0) * 1e-3) / self.model_dt_s).ceil();
        let post_steps = ((self.shot_window_post_ms.max(0.0) * 1e-3) / self.model_dt_s).ceil();
        let pre_steps = pre_steps.max(0.0) as usize;
        let post_steps = post_steps.max(0.0) as usize;
        let start_index = clamped_slot_index
            .saturating_sub(pre_steps)
            .clamp(1, horizon - 1);
        let end_index = (clamped_slot_index + post_steps).clamp(start_index, horizon - 1);

        let mut max_error_deg: f64 = 0.0;
        for index in start_index..=end_index {
            let slot_error_deg = (reference_yaw_deg[index] - predicted_yaw_deg[index]).abs();
            if !slot_error_deg.is_finite() {
                return None;
            }
            max_error_deg = max_error_deg.max(slot_error_deg);
        }
        Some(max_error_deg)
    }

    /// Count consecutive future shot slots that pass preview, impact, and MCU gates.
    pub fn count_viable_shot_slots(&self, gate: ShotSlotGate<'_>) -> ShotSlotGateResult {
        let mut result = ShotSlotGateResult {
            viable_slot_count: 0,
            first_slot_index: None,
            first_slot_error_deg: None,
            first_viable_slot_time_s: None,
            first_slot_impact: None,
        };

        if !gate.tolerance_deg.is_finite() || gate.tolerance_deg <= 0.0 {
            return result;
        }

        let horizon = gate
            .predicted_yaw_deg
            .len()
            .min(gate.reference_yaw_deg.len());
        if horizon <= 1 {
            return result;
        }

        let model_dt_s = if self.model_dt_s.is_finite() {
            self.model_dt_s.max(1e-3)
        } else {
            FireGateConfig::default().model_dt_s
        };
        let fire_rate_hz = if self.fire_rate_hz.is_finite() {
            self.fire_rate_hz.max(1e-3)
        } else {
            FireGateConfig::default().fire_rate_hz
        };
        let slot_step = ((1.0 / fire_rate_hz) / model_dt_s).round().max(1.0) as usize;
        let first_slot_index = ((gate.first_slot_time_s.max(model_dt_s) / model_dt_s).round()
            as usize)
            .clamp(1, horizon - 1);
        result.first_slot_index = Some(first_slot_index);

        let mut index = first_slot_index;
        while index < horizon {
            let impact = gate.impact_delta_angle_ref_deg.and_then(|impact_ref| {
                self.impact_angle_ref_in_window(impact_ref, index, gate.target_omega_rad_s)
            });
            let impact_angle_ok = if gate.require_impact_angle_gate {
                impact.is_some_and(|impact| impact.in_window)
            } else {
                true
            };

            if index == first_slot_index {
                result.first_slot_impact = impact;
            }
            if !impact_angle_ok {
                break;
            }

            let Some(slot_error_deg) = self.shot_slot_window_max_error_deg(
                gate.predicted_yaw_deg,
                gate.reference_yaw_deg,
                index,
            ) else {
                break;
            };
            if index == first_slot_index {
                result.first_slot_error_deg = Some(slot_error_deg);
            }
            if slot_error_deg >= gate.tolerance_deg || !gate.mcu_fire_permit {
                break;
            }

            if result.viable_slot_count == 0 {
                result.first_viable_slot_time_s = Some(index as f64 * model_dt_s);
            }
            result.viable_slot_count += 1;
            index += slot_step;
        }

        result
    }

    pub fn command_is_stable(
        &self,
        yaw_delta_deg: f64,
        pitch_delta_deg: f64,
        tolerance_deg: f64,
    ) -> bool {
        let stable_tolerance = tolerance_deg * self.command_stable_ratio;
        yaw_delta_deg.is_finite()
            && pitch_delta_deg.is_finite()
            && yaw_delta_deg < stable_tolerance
            && pitch_delta_deg < stable_tolerance
    }

    pub fn follow_is_ready(
        &self,
        yaw_error_deg: f64,
        pitch_error_deg: f64,
        tolerance_deg: f64,
    ) -> bool {
        yaw_error_deg.is_finite()
            && pitch_error_deg.is_finite()
            && yaw_error_deg < tolerance_deg
            && pitch_error_deg < tolerance_deg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yaw_tolerance_matches_vivsionn_distance_gate() {
        let cfg = FireGateConfig::default();
        let mid_distance_tolerance = (0.055_f64 / 3.0).atan().to_degrees();

        assert!((cfg.yaw_tolerance_deg(1.0) - 1.5).abs() < 1e-12);
        assert!((cfg.yaw_tolerance_deg(5.0) - 0.9).abs() < 1e-12);
        assert!((cfg.yaw_tolerance_deg(3.0) - mid_distance_tolerance).abs() < 1e-12);
        assert!((cfg.yaw_tolerance_deg(f64::NAN) - 1.5).abs() < 1e-12);
    }

    #[test]
    fn yaw_tolerance_tolerates_reversed_or_invalid_limits() {
        let reversed = FireGateConfig {
            yaw_tolerance_min_deg: 1.5,
            yaw_tolerance_max_deg: 0.9,
            ..Default::default()
        };
        let invalid_miss = FireGateConfig {
            yaw_miss_tolerance_m: -1.0,
            ..Default::default()
        };
        let mid_distance_tolerance = (0.055_f64 / 3.0).atan().to_degrees();

        assert!((reversed.yaw_tolerance_deg(5.0) - 0.9).abs() < 1e-12);
        assert!((invalid_miss.yaw_tolerance_deg(3.0) - mid_distance_tolerance).abs() < 1e-12);
    }

    #[test]
    fn impact_angle_window_uses_spin_direction() {
        let cfg = FireGateConfig::default();

        assert!(cfg.impact_angle_in_window(49.0_f64.to_radians(), 0.0));
        assert!(!cfg.impact_angle_in_window(51.0_f64.to_radians(), 0.0));

        assert!(cfg.impact_angle_in_window((-45.0_f64).to_radians(), 1.0));
        assert!(cfg.impact_angle_in_window(30.0_f64.to_radians(), 1.0));
        assert!(!cfg.impact_angle_in_window(35.0_f64.to_radians(), 1.0));

        assert!(cfg.impact_angle_in_window(45.0_f64.to_radians(), -1.0));
        assert!(cfg.impact_angle_in_window((-30.0_f64).to_radians(), -1.0));
        assert!(!cfg.impact_angle_in_window((-35.0_f64).to_radians(), -1.0));
    }

    #[test]
    fn shot_slot_window_max_error_checks_configured_window() {
        let cfg = FireGateConfig {
            model_dt_s: 0.01,
            shot_window_pre_ms: 10.0,
            shot_window_post_ms: 20.0,
            ..Default::default()
        };
        let predicted = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let reference = [0.0, 0.2, 0.7, 0.4, 1.3, 0.9];

        let max_error = cfg
            .shot_slot_window_max_error_deg(&predicted, &reference, 3)
            .unwrap();

        assert!((max_error - 1.3).abs() < 1e-12);
    }

    #[test]
    fn count_viable_shot_slots_stops_on_first_failed_slot() {
        let cfg = FireGateConfig {
            model_dt_s: 0.01,
            fire_rate_hz: 20.0,
            shot_window_pre_ms: 0.0,
            shot_window_post_ms: 0.0,
            ..Default::default()
        };
        let predicted = [0.0; 18];
        let mut reference = [0.0; 18];
        reference[11] = 2.0;
        let impact_ref = [0.0; 18];

        let result = cfg.count_viable_shot_slots(ShotSlotGate {
            predicted_yaw_deg: &predicted,
            reference_yaw_deg: &reference,
            impact_delta_angle_ref_deg: Some(&impact_ref),
            tolerance_deg: 1.0,
            first_slot_time_s: 0.01,
            target_omega_rad_s: 1.0,
            require_impact_angle_gate: true,
            mcu_fire_permit: true,
        });

        assert_eq!(result.first_slot_index, Some(1));
        assert_eq!(result.first_slot_error_deg, Some(0.0));
        assert_eq!(result.first_viable_slot_time_s, Some(0.01));
        assert_eq!(result.viable_slot_count, 2);
        assert_eq!(
            result.first_slot_impact,
            Some(ImpactAngleCheck {
                delta_deg: 0.0,
                in_window: true
            })
        );
    }

    #[test]
    fn count_viable_shot_slots_respects_impact_and_mcu_gates() {
        let cfg = FireGateConfig {
            model_dt_s: 0.01,
            fire_rate_hz: 20.0,
            shot_window_pre_ms: 0.0,
            shot_window_post_ms: 0.0,
            ..Default::default()
        };
        let predicted = [0.0; 8];
        let reference = [0.0; 8];
        let impact_ref = [0.0, 35.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let blocked_by_impact = cfg.count_viable_shot_slots(ShotSlotGate {
            predicted_yaw_deg: &predicted,
            reference_yaw_deg: &reference,
            impact_delta_angle_ref_deg: Some(&impact_ref),
            tolerance_deg: 1.0,
            first_slot_time_s: 0.01,
            target_omega_rad_s: 1.0,
            require_impact_angle_gate: true,
            mcu_fire_permit: true,
        });
        let blocked_by_mcu = cfg.count_viable_shot_slots(ShotSlotGate {
            predicted_yaw_deg: &predicted,
            reference_yaw_deg: &reference,
            impact_delta_angle_ref_deg: None,
            tolerance_deg: 1.0,
            first_slot_time_s: 0.01,
            target_omega_rad_s: 0.0,
            require_impact_angle_gate: false,
            mcu_fire_permit: false,
        });

        assert_eq!(blocked_by_impact.viable_slot_count, 0);
        assert_eq!(
            blocked_by_impact.first_slot_impact,
            Some(ImpactAngleCheck {
                delta_deg: 35.0,
                in_window: false
            })
        );
        assert_eq!(blocked_by_mcu.viable_slot_count, 0);
        assert_eq!(blocked_by_mcu.first_slot_error_deg, Some(0.0));
    }

    #[test]
    fn command_and_follow_gates_use_fire_tolerance() {
        let cfg = FireGateConfig::default();
        let tolerance = cfg.yaw_tolerance_deg(3.0);

        assert!(cfg.command_is_stable(0.5, 0.5, tolerance));
        assert!(!cfg.command_is_stable(tolerance, 0.5, tolerance));
        assert!(cfg.follow_is_ready(0.8, 0.8, tolerance));
        assert!(!cfg.follow_is_ready(1.1, 0.8, tolerance));
    }
}
