//! Runtime task-mode context.
//!
//! This mirrors vivsionn's mainline mode routing in
//! `ThreadManager::applyTaskModeState`: feedback task mode decides which vision
//! pipeline is active, and route changes ask the caller to clear stale queues.

use std::time::Instant;

use crate::rbt_mod::rbt_comm::rbt_comm_frame::{SensData, TaskMode};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeRoute {
    AutoShot,
    EnergyMechanism,
    Outpost,
}

impl ModeRoute {
    pub fn from_task_mode(task_mode: TaskMode) -> Self {
        match task_mode {
            TaskMode::AutoShot => Self::AutoShot,
            TaskMode::HitBigBuff | TaskMode::HitSmallBuff => Self::EnergyMechanism,
            TaskMode::HitOutpost => Self::Outpost,
        }
    }

    pub fn yolo_preprocess_active(self) -> bool {
        matches!(self, Self::AutoShot | Self::Outpost)
    }

    pub fn yolo_active(self) -> bool {
        matches!(self, Self::AutoShot | Self::Outpost)
    }

    pub fn fire_control_active(self) -> bool {
        matches!(self, Self::AutoShot | Self::Outpost)
    }

    pub fn energy_mechanism_preprocess_active(self) -> bool {
        matches!(self, Self::EnergyMechanism)
    }

    pub fn energy_mechanism_active(self) -> bool {
        matches!(self, Self::EnergyMechanism)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeTransition {
    Unchanged,
    EnterAutoShot,
    EnterEnergyMechanism,
    EnterOutpost,
}

impl ModeTransition {
    pub fn changed(self) -> bool {
        !matches!(self, Self::Unchanged)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeRuntimeSwitch {
    pub clear_mode_queues: bool,
    pub yolo_preprocess_active: bool,
    pub yolo_active: bool,
    pub fire_control_active: bool,
    pub energy_mechanism_preprocess_active: bool,
    pub energy_mechanism_active: bool,
}

impl ModeRuntimeSwitch {
    fn new(route: ModeRoute, transition: ModeTransition) -> Self {
        Self {
            clear_mode_queues: transition.changed(),
            yolo_preprocess_active: route.yolo_preprocess_active(),
            yolo_active: route.yolo_active(),
            fire_control_active: route.fire_control_active(),
            energy_mechanism_preprocess_active: route.energy_mechanism_preprocess_active(),
            energy_mechanism_active: route.energy_mechanism_active(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeUpdate {
    pub transition: ModeTransition,
    pub previous_task_mode: Option<TaskMode>,
    pub task_mode: TaskMode,
    pub raw_task_mode: u8,
    pub mapped_task_mode: TaskMode,
    pub route: ModeRoute,
    pub runtime_switch: ModeRuntimeSwitch,
    pub transition_seq: u64,
}

impl ModeUpdate {
    pub fn changed(&self) -> bool {
        self.transition.changed()
    }
}

#[derive(Debug, Clone)]
pub struct ModeContext {
    current_task_mode: Option<TaskMode>,
    active_route: Option<ModeRoute>,
    latest_raw_task_mode: u8,
    latest_mapped_task_mode: Option<TaskMode>,
    transition_seq: u64,
    last_transition_at: Option<Instant>,
}

impl Default for ModeContext {
    fn default() -> Self {
        Self::new()
    }
}

impl ModeContext {
    pub fn new() -> Self {
        Self {
            current_task_mode: None,
            active_route: None,
            latest_raw_task_mode: 0,
            latest_mapped_task_mode: None,
            transition_seq: 0,
            last_transition_at: None,
        }
    }

    pub fn with_initial_task_mode(task_mode: TaskMode) -> Self {
        Self {
            current_task_mode: Some(task_mode),
            active_route: Some(ModeRoute::from_task_mode(task_mode)),
            latest_raw_task_mode: task_mode.into(),
            latest_mapped_task_mode: Some(task_mode),
            transition_seq: 0,
            last_transition_at: None,
        }
    }

    pub fn apply_feedback(&mut self, feedback: &SensData) -> ModeUpdate {
        let task_mode = feedback.task_mode;
        let route = ModeRoute::from_task_mode(task_mode);
        let previous_task_mode = self.current_task_mode;
        let transition = if self.active_route == Some(route) {
            ModeTransition::Unchanged
        } else {
            match route {
                ModeRoute::AutoShot => ModeTransition::EnterAutoShot,
                ModeRoute::EnergyMechanism => ModeTransition::EnterEnergyMechanism,
                ModeRoute::Outpost => ModeTransition::EnterOutpost,
            }
        };

        if transition.changed() {
            self.active_route = Some(route);
            self.transition_seq = self.transition_seq.saturating_add(1);
            self.last_transition_at = Some(Instant::now());
        }

        self.current_task_mode = Some(task_mode);
        self.latest_raw_task_mode = raw_task_mode_or_feedback_value(feedback);
        self.latest_mapped_task_mode = Some(feedback.mapped_task_mode);

        ModeUpdate {
            transition,
            previous_task_mode,
            task_mode,
            raw_task_mode: self.latest_raw_task_mode,
            mapped_task_mode: feedback.mapped_task_mode,
            route,
            runtime_switch: ModeRuntimeSwitch::new(route, transition),
            transition_seq: self.transition_seq,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn current_task_mode(&self) -> Option<TaskMode> {
        self.current_task_mode
    }

    pub fn active_route(&self) -> Option<ModeRoute> {
        self.active_route
    }

    pub fn latest_raw_task_mode(&self) -> u8 {
        self.latest_raw_task_mode
    }

    pub fn latest_mapped_task_mode(&self) -> Option<TaskMode> {
        self.latest_mapped_task_mode
    }

    pub fn transition_seq(&self) -> u64 {
        self.transition_seq
    }

    pub fn last_transition_at(&self) -> Option<Instant> {
        self.last_transition_at
    }
}

fn raw_task_mode_or_feedback_value(feedback: &SensData) -> u8 {
    if feedback.raw_task_mode == 0 {
        feedback.task_mode.into()
    } else {
        feedback.raw_task_mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_comm::rbt_comm_frame::{DEFAULT_BULLET_SPEED_MPS, SelfFraction};

    fn feedback(task_mode: TaskMode) -> SensData {
        SensData {
            task_mode,
            self_fraction: SelfFraction::Blue,
            bullet_speed: DEFAULT_BULLET_SPEED_MPS,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            yaw_speed: 0.0,
            mcu_fire_permit: false,
            raw_task_mode: task_mode.into(),
            mapped_task_mode: task_mode,
        }
    }

    #[test]
    fn first_auto_shot_feedback_enters_armor_pipeline() {
        let mut context = ModeContext::new();

        let update = context.apply_feedback(&feedback(TaskMode::AutoShot));

        assert_eq!(update.transition, ModeTransition::EnterAutoShot);
        assert!(update.changed());
        assert!(update.runtime_switch.clear_mode_queues);
        assert!(update.runtime_switch.yolo_preprocess_active);
        assert!(update.runtime_switch.yolo_active);
        assert!(update.runtime_switch.fire_control_active);
        assert!(!update.runtime_switch.energy_mechanism_preprocess_active);
        assert!(!update.runtime_switch.energy_mechanism_active);
        assert_eq!(context.active_route(), Some(ModeRoute::AutoShot));
        assert_eq!(context.transition_seq(), 1);
        assert!(context.last_transition_at().is_some());
    }

    #[test]
    fn repeated_same_route_does_not_clear_queues() {
        let mut context = ModeContext::new();
        context.apply_feedback(&feedback(TaskMode::AutoShot));

        let update = context.apply_feedback(&feedback(TaskMode::AutoShot));

        assert_eq!(update.transition, ModeTransition::Unchanged);
        assert!(!update.changed());
        assert!(!update.runtime_switch.clear_mode_queues);
        assert_eq!(update.transition_seq, 1);
    }

    #[test]
    fn entering_energy_mechanism_disables_armor_pipeline_and_resets_energy_mechanism_state() {
        let mut context = ModeContext::new();
        context.apply_feedback(&feedback(TaskMode::AutoShot));

        let update = context.apply_feedback(&feedback(TaskMode::HitBigBuff));

        assert_eq!(update.transition, ModeTransition::EnterEnergyMechanism);
        assert_eq!(update.previous_task_mode, Some(TaskMode::AutoShot));
        assert!(update.runtime_switch.clear_mode_queues);
        assert!(!update.runtime_switch.yolo_active);
        assert!(!update.runtime_switch.fire_control_active);
        assert!(update.runtime_switch.energy_mechanism_preprocess_active);
        assert!(update.runtime_switch.energy_mechanism_active);
        assert_eq!(context.active_route(), Some(ModeRoute::EnergyMechanism));
    }

    #[test]
    fn large_and_small_energy_mechanism_are_same_runtime_route() {
        let mut context = ModeContext::new();
        context.apply_feedback(&feedback(TaskMode::HitBigBuff));

        let update = context.apply_feedback(&feedback(TaskMode::HitSmallBuff));

        assert_eq!(update.transition, ModeTransition::Unchanged);
        assert!(!update.runtime_switch.clear_mode_queues);
        assert_eq!(context.current_task_mode(), Some(TaskMode::HitSmallBuff));
        assert_eq!(context.active_route(), Some(ModeRoute::EnergyMechanism));
        assert_eq!(context.transition_seq(), 1);
    }

    #[test]
    fn entering_outpost_uses_armor_pipeline_with_outpost_route() {
        let mut context = ModeContext::new();
        context.apply_feedback(&feedback(TaskMode::HitSmallBuff));

        let update = context.apply_feedback(&feedback(TaskMode::HitOutpost));

        assert_eq!(update.transition, ModeTransition::EnterOutpost);
        assert!(update.runtime_switch.clear_mode_queues);
        assert!(update.runtime_switch.yolo_preprocess_active);
        assert!(update.runtime_switch.yolo_active);
        assert!(update.runtime_switch.fire_control_active);
        assert!(!update.runtime_switch.energy_mechanism_preprocess_active);
        assert_eq!(context.active_route(), Some(ModeRoute::Outpost));
    }

    #[test]
    fn raw_mode_falls_back_to_task_mode_when_feedback_was_constructed_by_hand() {
        let mut context = ModeContext::new();
        let mut data = feedback(TaskMode::HitOutpost);
        data.raw_task_mode = 0;

        let update = context.apply_feedback(&data);

        assert_eq!(update.raw_task_mode, TaskMode::HitOutpost as u8);
        assert_eq!(context.latest_raw_task_mode(), TaskMode::HitOutpost as u8);
        assert_eq!(
            context.latest_mapped_task_mode(),
            Some(TaskMode::HitOutpost)
        );
    }

    #[test]
    fn reset_forgets_current_route() {
        let mut context = ModeContext::new();
        context.apply_feedback(&feedback(TaskMode::AutoShot));

        context.reset();

        assert_eq!(context.current_task_mode(), None);
        assert_eq!(context.active_route(), None);
        assert_eq!(context.transition_seq(), 0);
        assert_eq!(context.last_transition_at(), None);
    }

    #[test]
    fn initial_task_mode_starts_without_transition_action() {
        let mut context = ModeContext::with_initial_task_mode(TaskMode::AutoShot);

        let update = context.apply_feedback(&feedback(TaskMode::AutoShot));

        assert_eq!(update.transition, ModeTransition::Unchanged);
        assert!(!update.runtime_switch.clear_mode_queues);
        assert_eq!(context.active_route(), Some(ModeRoute::AutoShot));
        assert_eq!(context.transition_seq(), 0);
    }
}
