use crate::rbt_mod::rbt_comm::rbt_comm_frame::ShotMode;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotPhaseConfig {
    pub model_dt_s: f64,
    pub fire_rate_hz: f64,
    pub first_shot_advance_ms: f64,
    pub auto_enter_slot_count: usize,
    pub auto_hold_slot_count: usize,
    pub auto_min_burst_ms: f64,
    pub auto_restart_cooldown_ms: f64,
}

impl Default for ShotPhaseConfig {
    fn default() -> Self {
        Self {
            model_dt_s: 0.004,
            fire_rate_hz: 20.0,
            first_shot_advance_ms: 10.0,
            auto_enter_slot_count: 2,
            auto_hold_slot_count: 1,
            auto_min_burst_ms: 20.0,
            auto_restart_cooldown_ms: 40.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShotPhaseMode {
    None,
    Auto,
    Single,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotPhaseInput {
    pub hard_gate_ok: bool,
    pub viable_slot_count: usize,
    pub dt_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShotPhaseDecision {
    pub shot_mode: ShotMode,
    pub phase_mode: ShotPhaseMode,
    pub next_slot_delay_ms: f64,
    pub mechanical_hold_active: bool,
    pub burst_active: bool,
}

impl Default for ShotPhaseDecision {
    fn default() -> Self {
        Self {
            shot_mode: ShotMode::AimOnly,
            phase_mode: ShotPhaseMode::None,
            next_slot_delay_ms: f64::NAN,
            mechanical_hold_active: false,
            burst_active: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ShotPhaseController {
    config: ShotPhaseConfig,
    phase_mode: ShotPhaseMode,
    now_s: f64,
    next_slot_at_s: Option<f64>,
    auto_hold_until_s: Option<f64>,
    auto_restart_cooldown_until_s: Option<f64>,
    single_pulse_sent: bool,
    burst_active: bool,
}

impl ShotPhaseController {
    pub fn new(config: ShotPhaseConfig) -> Self {
        Self {
            config,
            phase_mode: ShotPhaseMode::None,
            now_s: 0.0,
            next_slot_at_s: None,
            auto_hold_until_s: None,
            auto_restart_cooldown_until_s: None,
            single_pulse_sent: false,
            burst_active: false,
        }
    }

    pub fn config(&self) -> ShotPhaseConfig {
        self.config
    }

    pub fn reset(&mut self) {
        self.clear_phase();
        self.auto_restart_cooldown_until_s = None;
    }

    pub fn first_slot_time_s(&self) -> f64 {
        self.phase_first_slot_time_s(self.now_s)
    }

    pub fn update(&mut self, input: ShotPhaseInput) -> ShotPhaseDecision {
        self.advance_clock(input.dt_s);
        self.advance_phase();

        let first_slot_time_s = self.phase_first_slot_time_s(self.now_s);
        let enter_slot_count = self.config.auto_enter_slot_count.max(1);
        let hold_slot_count = self.config.auto_hold_slot_count.max(1);
        let burst_enter_ready = input.viable_slot_count >= enter_slot_count;
        let burst_hold_ready = input.viable_slot_count >= hold_slot_count;
        let single_window_ready = input.viable_slot_count == 1;
        let mechanical_hold_ready = self.phase_mode == ShotPhaseMode::Auto
            && self
                .auto_hold_until_s
                .is_some_and(|hold_until| self.now_s < hold_until);

        let mut decision = ShotPhaseDecision {
            next_slot_delay_ms: if input.hard_gate_ok {
                first_slot_time_s * 1_000.0
            } else {
                f64::NAN
            },
            ..Default::default()
        };

        if !input.hard_gate_ok {
            self.clear_phase();
        } else if self.phase_mode == ShotPhaseMode::Auto {
            if burst_hold_ready {
                self.burst_active = true;
                self.auto_hold_until_s = Some(self.auto_hold_deadline_from_next_slot());
                decision.shot_mode = ShotMode::AutoFire;
            } else if mechanical_hold_ready {
                self.burst_active = true;
                decision.shot_mode = ShotMode::AutoFire;
                decision.mechanical_hold_active = true;
            } else {
                self.clear_phase();
                self.auto_restart_cooldown_until_s =
                    Some(self.now_s + self.config.auto_restart_cooldown_ms.max(0.0) * 1e-3);
            }
        } else if self.phase_mode == ShotPhaseMode::Single {
            if !self.single_pulse_sent {
                self.single_pulse_sent = true;
                decision.shot_mode = ShotMode::ShotOnce;
            }
        } else if self.cooling_down() {
            decision.next_slot_delay_ms = self.cooldown_delay_ms();
        } else if burst_enter_ready {
            self.start_phase(ShotPhaseMode::Auto, first_slot_time_s);
            self.auto_hold_until_s = Some(self.auto_hold_deadline_from_next_slot());
            decision.shot_mode = ShotMode::AutoFire;
        } else if single_window_ready {
            self.start_phase(ShotPhaseMode::Single, first_slot_time_s);
            self.single_pulse_sent = true;
            decision.shot_mode = ShotMode::ShotOnce;
        } else {
            self.clear_phase();
        }

        if self.phase_mode != ShotPhaseMode::None {
            decision.next_slot_delay_ms = self.phase_first_slot_time_s(self.now_s) * 1_000.0;
        } else if input.hard_gate_ok && !self.cooling_down() {
            decision.next_slot_delay_ms = first_slot_time_s * 1_000.0;
        }

        decision.phase_mode = self.phase_mode;
        decision.burst_active = self.burst_active;
        decision
    }

    fn advance_clock(&mut self, dt_s: f64) {
        let dt_s = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            self.model_dt_s()
        };
        self.now_s += dt_s.max(0.0);
    }

    fn advance_phase(&mut self) {
        let Some(next_slot_at_s) = self.next_slot_at_s else {
            return;
        };

        if self.phase_mode == ShotPhaseMode::Single && self.now_s >= next_slot_at_s {
            self.clear_phase();
            return;
        }

        let slot_period_s = self.slot_period_s();
        while self.phase_mode == ShotPhaseMode::Auto {
            let Some(next_slot_at_s) = self.next_slot_at_s else {
                break;
            };
            if self.now_s < next_slot_at_s {
                break;
            }
            self.next_slot_at_s = Some(next_slot_at_s + slot_period_s);
        }
    }

    fn start_phase(&mut self, mode: ShotPhaseMode, first_slot_time_s: f64) {
        let first_slot_time_s = first_slot_time_s.max(self.model_dt_s());
        self.phase_mode = mode;
        self.next_slot_at_s = Some(self.now_s + first_slot_time_s);
        self.single_pulse_sent = false;
        self.burst_active = mode == ShotPhaseMode::Auto;
    }

    fn clear_phase(&mut self) {
        self.phase_mode = ShotPhaseMode::None;
        self.next_slot_at_s = None;
        self.auto_hold_until_s = None;
        self.single_pulse_sent = false;
        self.burst_active = false;
    }

    fn phase_first_slot_time_s(&self, now_s: f64) -> f64 {
        if self.phase_mode == ShotPhaseMode::None {
            return self.compute_shot_check_start_time_s(true);
        }

        self.next_slot_at_s
            .map(|next_slot_at_s| (next_slot_at_s - now_s).max(self.model_dt_s()))
            .unwrap_or_else(|| self.compute_shot_check_start_time_s(true))
    }

    fn compute_shot_check_start_time_s(&self, first_shot_candidate: bool) -> f64 {
        let model_dt_s = self.model_dt_s();
        let slot_period_s = self.slot_period_s();
        if !first_shot_candidate {
            return slot_period_s;
        }

        let max_advance_s = (slot_period_s - model_dt_s).max(0.0);
        let advance_s = (self.config.first_shot_advance_ms * 1e-3).clamp(0.0, max_advance_s);
        (slot_period_s - advance_s).max(model_dt_s)
    }

    fn auto_hold_deadline_from_next_slot(&self) -> f64 {
        let committed_slot_s = self.next_slot_at_s.unwrap_or(self.now_s);
        committed_slot_s + self.config.auto_min_burst_ms.max(0.0) * 1e-3
    }

    fn cooling_down(&self) -> bool {
        self.auto_restart_cooldown_until_s
            .is_some_and(|cooldown_until| self.now_s < cooldown_until)
    }

    fn cooldown_delay_ms(&self) -> f64 {
        self.auto_restart_cooldown_until_s
            .map(|cooldown_until| (cooldown_until - self.now_s).max(0.0) * 1_000.0)
            .unwrap_or(f64::NAN)
    }

    fn model_dt_s(&self) -> f64 {
        if self.config.model_dt_s.is_finite() && self.config.model_dt_s > 0.0 {
            self.config.model_dt_s.max(1e-3)
        } else {
            ShotPhaseConfig::default().model_dt_s
        }
    }

    fn slot_period_s(&self) -> f64 {
        let fire_rate_hz = if self.config.fire_rate_hz.is_finite() {
            self.config.fire_rate_hz.max(1e-3)
        } else {
            ShotPhaseConfig::default().fire_rate_hz
        };
        1.0 / fire_rate_hz
    }
}

impl Default for ShotPhaseController {
    fn default() -> Self {
        Self::new(ShotPhaseConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn controller() -> ShotPhaseController {
        ShotPhaseController::new(ShotPhaseConfig {
            model_dt_s: 0.004,
            fire_rate_hz: 20.0,
            first_shot_advance_ms: 10.0,
            auto_enter_slot_count: 2,
            auto_hold_slot_count: 1,
            auto_min_burst_ms: 20.0,
            auto_restart_cooldown_ms: 40.0,
        })
    }

    #[test]
    fn first_slot_advance_stays_inside_fire_period() {
        let phase = controller();
        let first_slot = phase.first_slot_time_s();

        assert!(first_slot >= phase.config().model_dt_s);
        assert!(first_slot <= 1.0 / phase.config().fire_rate_hz);
        assert!((first_slot - 0.04).abs() < 1e-12);
    }

    #[test]
    fn single_window_emits_one_shot_once() {
        let mut phase = controller();

        let first = phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 1,
            dt_s: 0.004,
        });
        let second = phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 1,
            dt_s: 0.004,
        });

        assert_eq!(first.shot_mode, ShotMode::ShotOnce);
        assert_eq!(second.shot_mode, ShotMode::AimOnly);
    }

    #[test]
    fn consecutive_slots_enter_auto_fire() {
        let mut phase = controller();

        let decision = phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 2,
            dt_s: 0.004,
        });

        assert_eq!(decision.shot_mode, ShotMode::AutoFire);
        assert_eq!(decision.phase_mode, ShotPhaseMode::Auto);
        assert!(decision.burst_active);
    }

    #[test]
    fn auto_phase_uses_mechanical_hold_then_exits() {
        let mut phase = controller();
        phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 2,
            dt_s: 0.004,
        });

        let held = phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 0,
            dt_s: 0.004,
        });
        let exited = phase.update(ShotPhaseInput {
            hard_gate_ok: true,
            viable_slot_count: 0,
            dt_s: 0.060,
        });

        assert_eq!(held.shot_mode, ShotMode::AutoFire);
        assert!(held.mechanical_hold_active);
        assert_eq!(exited.shot_mode, ShotMode::AimOnly);
        assert_eq!(exited.phase_mode, ShotPhaseMode::None);
    }
}
