//! Runtime route switch state for the async mainline.
//!
//! `ModeContext` answers "which route does this feedback select"; this module
//! keeps the answer in a shared, cheap-to-read state that long-running tasks can
//! query before accepting stale work from another route.

use std::sync::{Arc, RwLock};

use crate::rbt_mod::rbt_comm::rbt_comm_frame::{SensData, TaskMode};
use crate::rbt_mod::rbt_mode_context::{ModeContext, ModeRoute, ModeUpdate};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuntimeRouteState {
    pub route: ModeRoute,
    pub task_mode: TaskMode,
    pub transition_seq: u64,
}

impl RuntimeRouteState {
    fn new(route: ModeRoute, task_mode: TaskMode, transition_seq: u64) -> Self {
        Self {
            route,
            task_mode,
            transition_seq,
        }
    }

    pub fn armor_pipeline_active(self) -> bool {
        matches!(self.route, ModeRoute::AutoShot | ModeRoute::Outpost)
    }

    pub fn fire_control_active(self) -> bool {
        matches!(self.route, ModeRoute::AutoShot | ModeRoute::Outpost)
    }

    pub fn energy_mechanism_active(self) -> bool {
        matches!(self.route, ModeRoute::EnergyMechanism)
    }
}

#[derive(Debug)]
struct RuntimeRouterInner {
    mode_context: ModeContext,
    state: RuntimeRouteState,
}

#[derive(Debug, Clone)]
pub struct RuntimeRouter {
    inner: Arc<RwLock<RuntimeRouterInner>>,
}

impl Default for RuntimeRouter {
    fn default() -> Self {
        Self::new(TaskMode::AutoShot)
    }
}

impl RuntimeRouter {
    pub fn new(initial_task_mode: TaskMode) -> Self {
        let route = ModeRoute::from_task_mode(initial_task_mode);
        Self {
            inner: Arc::new(RwLock::new(RuntimeRouterInner {
                mode_context: ModeContext::with_initial_task_mode(initial_task_mode),
                state: RuntimeRouteState::new(route, initial_task_mode, 0),
            })),
        }
    }

    pub fn apply_feedback(&self, feedback: &SensData) -> ModeUpdate {
        let mut inner = self.inner.write().expect("runtime router lock poisoned");
        let update = inner.mode_context.apply_feedback(feedback);
        inner.state = RuntimeRouteState {
            route: update.route,
            task_mode: update.task_mode,
            transition_seq: update.transition_seq,
        };
        update
    }

    pub fn state(&self) -> RuntimeRouteState {
        self.inner
            .read()
            .expect("runtime router lock poisoned")
            .state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
        DEFAULT_BULLET_SPEED_MPS, SelfFraction, TaskMode,
    };
    use crate::rbt_mod::rbt_mode_context::{ModeRoute, ModeTransition};

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
    fn default_route_is_active_armor_without_transition() {
        let router = RuntimeRouter::default();

        let state = router.state();

        assert_eq!(state.route, ModeRoute::AutoShot);
        assert_eq!(state.task_mode, TaskMode::AutoShot);
        assert!(state.armor_pipeline_active());
        assert!(state.fire_control_active());
        assert!(!state.energy_mechanism_active());
        assert_eq!(state.transition_seq, 0);
    }

    #[test]
    fn applying_energy_mechanism_feedback_disables_armor_and_fire_control() {
        let router = RuntimeRouter::default();

        let update = router.apply_feedback(&feedback(TaskMode::HitBigBuff));
        let state = router.state();

        assert_eq!(update.transition, ModeTransition::EnterEnergyMechanism);
        assert!(update.runtime_switch.clear_mode_queues);
        assert_eq!(state.route, ModeRoute::EnergyMechanism);
        assert_eq!(state.task_mode, TaskMode::HitBigBuff);
        assert!(!state.armor_pipeline_active());
        assert!(!state.fire_control_active());
        assert!(state.energy_mechanism_active());
        assert_eq!(state.transition_seq, 1);
    }

    #[test]
    fn large_and_small_energy_mechanism_share_route_without_new_transition() {
        let router = RuntimeRouter::default();

        router.apply_feedback(&feedback(TaskMode::HitBigBuff));
        let update = router.apply_feedback(&feedback(TaskMode::HitSmallBuff));

        assert_eq!(update.transition, ModeTransition::Unchanged);
        assert!(!update.runtime_switch.clear_mode_queues);
        assert_eq!(router.state().task_mode, TaskMode::HitSmallBuff);
        assert_eq!(router.state().transition_seq, 1);
    }
}
