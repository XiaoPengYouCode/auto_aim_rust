use std::f64::consts::PI;

use crate::rbt_base::rbt_algorithm::rbt_antigravity::solve_ballistic_trajectory;
use crate::rbt_mod::rbt_estimator::EnemyTrackSnapshot;

pub const PLANNER_STATE_LEN: usize = 11;
const XC: usize = 0;
const VXC: usize = 1;
const YC: usize = 2;
const VYC: usize = 3;
const ZA: usize = 4;
const VZA: usize = 5;
const YAW: usize = 6;
const VYAW: usize = 7;
const R: usize = 8;
const EXTRA_0: usize = 9;
const EXTRA_1: usize = 10;

const MIN_ARMOR_RADIUS_M: f64 = 0.05;
const MAX_ARMOR_RADIUS_M: f64 = 0.60;
const MAX_OUTPOST_HEIGHT_OFFSET_M: f64 = 0.60;
const OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD: f64 = 153.0 * PI / 180.0;
const LOW_SPEED_SELECTION_OMEGA_RAD_S: f64 = 4.0;
const LOW_SPEED_MAX_DELTA_RAD: f64 = 60.0 * PI / 180.0;
const OUTPOST_COMING_ANGLE_RAD: f64 = 70.0 * PI / 180.0;
const OUTPOST_LEAVING_ANGLE_RAD: f64 = 30.0 * PI / 180.0;
const AIM_SWITCH_PENALTY_RAD: f64 = 10.0 * PI / 180.0;
const AIM_CONTINUITY_PENALTY_WEIGHT: f64 = 1.0;
const AIM_DIRECTIONAL_PENALTY_WEIGHT: f64 = 0.2;
const AIM_EXCESS_ANGLE_PENALTY_WEIGHT: f64 = 4.0;
const AIM_RELAXED_ANGLE_EXTRA_RAD: f64 = 12.0 * PI / 180.0;
const AIM_SOLVE_TOLERANCE_S: f64 = 1e-3;
const AIM_SOLVE_MAX_ITERATIONS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawPlannerConfig {
    pub yaw_offset_rad: f64,
    pub execution_delay_s: f64,
    pub preview_dt_s: f64,
    pub preview_horizon: usize,
    pub armor_enter_angle_deg: f64,
    pub armor_leave_angle_deg: f64,
}

impl Default for YawPlannerConfig {
    fn default() -> Self {
        Self {
            yaw_offset_rad: 0.0,
            execution_delay_s: 0.0,
            preview_dt_s: 0.01,
            preview_horizon: 16,
            armor_enter_angle_deg: 50.0,
            armor_leave_angle_deg: 30.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannerTarget {
    armor_count: usize,
    state: [f64; PLANNER_STATE_LEN],
}

impl PlannerTarget {
    pub fn new(armor_count: usize, state: [f64; PLANNER_STATE_LEN]) -> Self {
        let mut target = Self {
            armor_count: normalize_armor_count(armor_count),
            state,
        };
        target.clamp_geometry_state();
        target
    }

    pub fn from_snapshot(snapshot: &EnemyTrackSnapshot) -> Self {
        Self::new(snapshot.armor_count, snapshot.planner_state())
    }

    pub fn predict(&mut self, dt_s: f64) {
        if !dt_s.is_finite() {
            return;
        }
        self.state[XC] += self.state[VXC] * dt_s;
        self.state[YC] += self.state[VYC] * dt_s;
        self.state[ZA] += self.state[VZA] * dt_s;
        self.state[YAW] = limit_rad(self.state[YAW] + self.state[VYAW] * dt_s);
        self.clamp_geometry_state();
    }

    pub fn state(&self) -> &[f64; PLANNER_STATE_LEN] {
        &self.state
    }

    pub fn armor_count(&self) -> usize {
        self.armor_count
    }

    fn armor_xyza_list(&self) -> Vec<ArmorPose> {
        (0..self.armor_count)
            .map(|id| {
                let radial_yaw =
                    limit_rad(self.state[YAW] + id as f64 * 2.0 * PI / self.armor_count as f64);
                let radius = self.radius_for_armor(id);
                let z = self.state[ZA] + self.height_offset_for_armor(id);
                let radial_sign = if self.armor_count == 3 { 1.0 } else { -1.0 };
                let position = na::Point3::new(
                    self.state[XC] + radial_sign * radius * radial_yaw.cos(),
                    self.state[YC] + radial_sign * radius * radial_yaw.sin(),
                    z,
                );
                let yaw_rad = if self.armor_count == 3 {
                    limit_rad(radial_yaw - OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD)
                } else {
                    radial_yaw
                };
                ArmorPose { position, yaw_rad }
            })
            .collect()
    }

    fn radius_for_armor(&self, id: usize) -> f64 {
        let use_secondary = self.armor_count == 4 && (id == 1 || id == 3);
        let radius = if use_secondary {
            self.state[R] + self.state[EXTRA_0]
        } else {
            self.state[R]
        };
        radius.clamp(MIN_ARMOR_RADIUS_M, MAX_ARMOR_RADIUS_M)
    }

    fn height_offset_for_armor(&self, id: usize) -> f64 {
        if self.armor_count == 4 {
            if id == 1 || id == 3 {
                self.state[EXTRA_1]
            } else {
                0.0
            }
        } else if self.armor_count == 3 {
            match id {
                1 => self.state[EXTRA_0],
                2 => self.state[EXTRA_1],
                _ => 0.0,
            }
        } else {
            0.0
        }
    }

    fn clamp_geometry_state(&mut self) {
        self.state[R] = self.state[R].clamp(MIN_ARMOR_RADIUS_M, MAX_ARMOR_RADIUS_M);
        if self.armor_count == 4 {
            let secondary_radius =
                (self.state[R] + self.state[EXTRA_0]).clamp(MIN_ARMOR_RADIUS_M, MAX_ARMOR_RADIUS_M);
            self.state[EXTRA_0] = secondary_radius - self.state[R];
        } else if self.armor_count == 3 {
            self.state[EXTRA_0] = self.state[EXTRA_0]
                .clamp(-MAX_OUTPOST_HEIGHT_OFFSET_M, MAX_OUTPOST_HEIGHT_OFFSET_M);
            self.state[EXTRA_1] = self.state[EXTRA_1]
                .clamp(-MAX_OUTPOST_HEIGHT_OFFSET_M, MAX_OUTPOST_HEIGHT_OFFSET_M);
        } else {
            self.state[EXTRA_0] = 0.0;
            self.state[EXTRA_1] = 0.0;
        }
    }
}

impl From<&EnemyTrackSnapshot> for PlannerTarget {
    fn from(snapshot: &EnemyTrackSnapshot) -> Self {
        Self::from_snapshot(snapshot)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct YawPlan {
    pub control: bool,
    pub target_yaw_rad: f64,
    pub target_yaw_rate_rad_s: f64,
    pub selected_armor_index: isize,
    pub estimated_fly_time_s: f64,
    pub impact_delta_angle_rad: f64,
    pub impact_delta_angle_ref_rad: Vec<f64>,
    pub yaw_ref_rad: Vec<f64>,
    pub yaw_rate_ref_rad_s: Vec<f64>,
    pub target_position_m: na::Point3<f64>,
}

impl Default for YawPlan {
    fn default() -> Self {
        Self {
            control: false,
            target_yaw_rad: 0.0,
            target_yaw_rate_rad_s: 0.0,
            selected_armor_index: -1,
            estimated_fly_time_s: 0.0,
            impact_delta_angle_rad: 0.0,
            impact_delta_angle_ref_rad: Vec::new(),
            yaw_ref_rad: Vec::new(),
            yaw_rate_ref_rad_s: Vec::new(),
            target_position_m: na::Point3::origin(),
        }
    }
}

#[derive(Debug, Clone)]
struct YawReference {
    yaw_ref_rad: Vec<f64>,
    yaw_rate_ref_rad_s: Vec<f64>,
    impact_delta_angle_ref_rad: Vec<f64>,
    first_target_pos_m: na::Point3<f64>,
    first_selected_index: isize,
    first_fly_time_s: f64,
    first_impact_delta_angle_rad: f64,
}

#[derive(Debug, Clone)]
pub struct YawPlanner {
    config: YawPlannerConfig,
    last_selected_armor_index: isize,
    last_fly_time_s: Option<f64>,
    last_yaw_ref_rad: Vec<f64>,
    last_armor_count: Option<usize>,
}

impl Default for YawPlanner {
    fn default() -> Self {
        Self::new(YawPlannerConfig::default())
    }
}

impl YawPlanner {
    pub fn new(config: YawPlannerConfig) -> Self {
        Self {
            config,
            last_selected_armor_index: -1,
            last_fly_time_s: None,
            last_yaw_ref_rad: Vec::new(),
            last_armor_count: None,
        }
    }

    pub fn configure(&mut self, config: YawPlannerConfig) {
        if self.config != config {
            self.config = config;
        }
    }

    pub fn config(&self) -> YawPlannerConfig {
        self.config
    }

    pub fn reset(&mut self) {
        self.last_selected_armor_index = -1;
        self.last_fly_time_s = None;
        self.last_yaw_ref_rad.clear();
        self.last_armor_count = None;
    }

    pub fn plan(
        &mut self,
        target: impl Into<Option<PlannerTarget>>,
        bullet_speed_mps: f64,
    ) -> YawPlan {
        let Some(mut target) = target.into() else {
            self.reset();
            return YawPlan::default();
        };

        if self
            .last_armor_count
            .is_some_and(|count| count != target.armor_count())
        {
            self.reset();
        }
        self.last_armor_count = Some(target.armor_count());

        if self.config.execution_delay_s > 1e-6 {
            target.predict(self.config.execution_delay_s);
        }

        let bullet_speed_mps = if (10.0..=25.0).contains(&bullet_speed_mps) {
            bullet_speed_mps
        } else {
            22.0
        };

        let Some(reference) = self.build_yaw_reference(target, bullet_speed_mps) else {
            return YawPlan::default();
        };

        let plan = YawPlan {
            control: true,
            target_yaw_rad: reference.yaw_ref_rad[0],
            target_yaw_rate_rad_s: reference.yaw_rate_ref_rad_s[0],
            selected_armor_index: reference.first_selected_index,
            estimated_fly_time_s: reference.first_fly_time_s,
            impact_delta_angle_rad: reference.first_impact_delta_angle_rad,
            impact_delta_angle_ref_rad: reference.impact_delta_angle_ref_rad,
            yaw_ref_rad: reference.yaw_ref_rad,
            yaw_rate_ref_rad_s: reference.yaw_rate_ref_rad_s,
            target_position_m: reference.first_target_pos_m,
        };
        self.last_yaw_ref_rad = plan.yaw_ref_rad.clone();
        self.last_selected_armor_index = plan.selected_armor_index;
        if plan.estimated_fly_time_s > 1e-6 {
            self.last_fly_time_s = Some(plan.estimated_fly_time_s);
        }
        plan
    }

    fn build_yaw_reference(
        &self,
        mut target: PlannerTarget,
        bullet_speed_mps: f64,
    ) -> Option<YawReference> {
        let horizon = self.config.preview_horizon.max(4);
        let dt = self.config.preview_dt_s.max(1e-3);
        let mut yaw_ref_rad = vec![0.0; horizon];
        let mut yaw_rate_ref_rad_s = vec![0.0; horizon];
        let mut impact_delta_angle_ref_rad = vec![0.0; horizon];
        let mut first_target_pos_m = na::Point3::origin();
        let mut first_selected_index = self.last_selected_armor_index;
        let mut first_fly_time_s = 0.0;
        let mut first_impact_delta_angle_rad = 0.0;

        let mut preferred_index = first_selected_index;
        let mut fly_time_hint = self.last_fly_time_s;
        let mut continuity_yaw = self
            .last_yaw_ref_rad
            .get(1)
            .copied()
            .or_else(|| self.last_yaw_ref_rad.first().copied());

        for index in 0..horizon {
            if index > 0 {
                target.predict(dt);
            }

            let solution = self.solve_aim(
                target,
                bullet_speed_mps,
                preferred_index,
                fly_time_hint,
                continuity_yaw,
            )?;

            preferred_index = solution.selected_index;
            fly_time_hint = Some(solution.fly_time_s);
            impact_delta_angle_ref_rad[index] = solution.impact_delta_angle_rad;

            if index == 0 {
                yaw_ref_rad[0] = continuity_yaw
                    .map(|yaw| closest_equivalent_rad(yaw, solution.yaw_rad))
                    .unwrap_or(solution.yaw_rad);
                continuity_yaw = Some(yaw_ref_rad[0]);
                first_target_pos_m = solution.aim_pos_m;
                first_selected_index = solution.selected_index;
                first_fly_time_s = solution.fly_time_s;
                first_impact_delta_angle_rad = solution.impact_delta_angle_rad;
            } else {
                yaw_ref_rad[index] =
                    closest_equivalent_rad(yaw_ref_rad[index - 1], solution.yaw_rad);
                continuity_yaw = Some(yaw_ref_rad[index]);
            }
        }

        yaw_rate_ref_rad_s[0] = (yaw_ref_rad[1] - yaw_ref_rad[0]) / dt;
        for index in 1..horizon - 1 {
            yaw_rate_ref_rad_s[index] =
                (yaw_ref_rad[index + 1] - yaw_ref_rad[index - 1]) / (2.0 * dt);
        }
        yaw_rate_ref_rad_s[horizon - 1] =
            (yaw_ref_rad[horizon - 1] - yaw_ref_rad[horizon - 2]) / dt;

        Some(YawReference {
            yaw_ref_rad,
            yaw_rate_ref_rad_s,
            impact_delta_angle_ref_rad,
            first_target_pos_m,
            first_selected_index,
            first_fly_time_s,
            first_impact_delta_angle_rad,
        })
    }

    fn solve_aim(
        &self,
        target: PlannerTarget,
        bullet_speed_mps: f64,
        preferred_index: isize,
        initial_fly_time: Option<f64>,
        continuity_yaw: Option<f64>,
    ) -> Option<AimSolution> {
        let armors = target.armor_xyza_list();
        if armors.is_empty() {
            return None;
        }

        let mut best_solution = None;
        let mut best_score = f64::INFINITY;
        let mut preferred_solution = None;
        let mut preferred_score = f64::INFINITY;

        for armor_index in 0..armors.len() {
            let Some(solution) =
                self.solve_aim_for_armor(target, bullet_speed_mps, armor_index, initial_fly_time)
            else {
                continue;
            };
            let score = self.score_aim_solution(target, &solution, preferred_index, continuity_yaw);
            if score + 1e-9 < best_score {
                best_score = score;
                best_solution = Some(solution);
            }
            if armor_index as isize == preferred_index {
                preferred_score = score;
                preferred_solution = Some(solution);
            }
        }

        if preferred_solution.is_some()
            && preferred_score <= best_score + 0.5 * AIM_SWITCH_PENALTY_RAD
        {
            preferred_solution
        } else {
            best_solution
        }
    }

    fn solve_aim_for_armor(
        &self,
        target: PlannerTarget,
        bullet_speed_mps: f64,
        armor_index: usize,
        initial_fly_time: Option<f64>,
    ) -> Option<AimSolution> {
        let initial_armors = target.armor_xyza_list();
        let initial_armor = initial_armors.get(armor_index)?;
        let mut fly_time_s = if let Some(fly_time) = initial_fly_time.filter(|time| *time > 1e-6) {
            fly_time
        } else {
            fly_time_for_armor(bullet_speed_mps, initial_armor)?
        };

        for _ in 0..AIM_SOLVE_MAX_ITERATIONS {
            let solution = solve_at_time(
                &target,
                bullet_speed_mps,
                armor_index,
                fly_time_s,
                self.config.yaw_offset_rad,
            )?;
            if (solution.fly_time_s - fly_time_s).abs() < AIM_SOLVE_TOLERANCE_S {
                return Some(solution);
            }
            fly_time_s = solution.fly_time_s;
        }

        solve_at_time(
            &target,
            bullet_speed_mps,
            armor_index,
            fly_time_s,
            self.config.yaw_offset_rad,
        )
        .map(|mut solution| {
            solution.yaw_rad = limit_rad(solution.yaw_rad);
            solution
        })
    }

    fn score_aim_solution(
        &self,
        target: PlannerTarget,
        solution: &AimSolution,
        preferred_index: isize,
        continuity_yaw: Option<f64>,
    ) -> f64 {
        let state = target.state();
        let omega = state[VYAW];
        let is_outpost = target.armor_count() == 3;
        let low_speed = omega.abs() <= LOW_SPEED_SELECTION_OMEGA_RAD_S && !is_outpost;

        let configured_enter = self.config.armor_enter_angle_deg.max(0.0).to_radians();
        let configured_leave = self.config.armor_leave_angle_deg.max(0.0).to_radians();
        let normal_coming_angle = configured_enter.max(configured_leave);
        let normal_leaving_angle = configured_enter.min(configured_leave);
        let coming_angle = if is_outpost {
            OUTPOST_COMING_ANGLE_RAD
        } else {
            normal_coming_angle
        };
        let leaving_angle = if is_outpost {
            OUTPOST_LEAVING_ANGLE_RAD
        } else {
            normal_leaving_angle
        };
        let preferred_angle_limit = (if low_speed {
            LOW_SPEED_MAX_DELTA_RAD
        } else {
            coming_angle
        }) + AIM_RELAXED_ANGLE_EXTRA_RAD;

        let abs_delta = solution.impact_delta_angle_rad.abs();
        let mut score = abs_delta;
        if abs_delta > preferred_angle_limit {
            score += AIM_EXCESS_ANGLE_PENALTY_WEIGHT * (abs_delta - preferred_angle_limit);
        }
        if preferred_index >= 0 && solution.selected_index != preferred_index {
            score += AIM_SWITCH_PENALTY_RAD;
        }
        if let Some(continuity_yaw) = continuity_yaw {
            score +=
                AIM_CONTINUITY_PENALTY_WEIGHT * limit_rad(solution.yaw_rad - continuity_yaw).abs();
        }
        if !low_speed {
            if omega > 0.0 && solution.impact_delta_angle_rad > leaving_angle {
                score += AIM_DIRECTIONAL_PENALTY_WEIGHT
                    * (solution.impact_delta_angle_rad - leaving_angle);
            } else if omega < 0.0 && solution.impact_delta_angle_rad < -leaving_angle {
                score += AIM_DIRECTIONAL_PENALTY_WEIGHT
                    * (-leaving_angle - solution.impact_delta_angle_rad);
            }
        }
        score + 1e-3 * solution.planar_distance_m
    }
}

#[derive(Debug, Clone, Copy)]
struct ArmorPose {
    position: na::Point3<f64>,
    yaw_rad: f64,
}

#[derive(Debug, Clone, Copy)]
struct AimSolution {
    yaw_rad: f64,
    fly_time_s: f64,
    impact_delta_angle_rad: f64,
    planar_distance_m: f64,
    selected_index: isize,
    aim_pos_m: na::Point3<f64>,
}

fn solve_at_time(
    target: &PlannerTarget,
    bullet_speed_mps: f64,
    armor_index: usize,
    fly_time_s: f64,
    yaw_offset_rad: f64,
) -> Option<AimSolution> {
    let mut impact_target = *target;
    if fly_time_s > 1e-6 {
        impact_target.predict(fly_time_s);
    }
    let armor = *impact_target.armor_xyza_list().get(armor_index)?;
    let planar_distance_m = armor.position.x.hypot(armor.position.y);
    let fly_time_s = fly_time_for_armor(bullet_speed_mps, &armor)?;
    let state = impact_target.state();
    let center_yaw = state[YC].atan2(state[XC]);

    Some(AimSolution {
        yaw_rad: limit_rad((-armor.position.y).atan2(armor.position.x) + yaw_offset_rad),
        fly_time_s,
        impact_delta_angle_rad: limit_rad(armor.yaw_rad - center_yaw),
        planar_distance_m,
        selected_index: armor_index as isize,
        aim_pos_m: armor.position,
    })
}

fn fly_time_for_armor(bullet_speed_mps: f64, armor: &ArmorPose) -> Option<f64> {
    solve_ballistic_trajectory(
        bullet_speed_mps,
        armor.position.x.hypot(armor.position.y),
        armor.position.z,
    )
    .ok()
    .map(|solution| solution.fly_time_s)
}

fn normalize_armor_count(armor_count: usize) -> usize {
    if armor_count == 3 { 3 } else { 4 }
}

fn limit_rad(angle: f64) -> f64 {
    let mut result = (angle + PI) % (2.0 * PI);
    if result < 0.0 {
        result += 2.0 * PI;
    }
    result - PI
}

fn closest_equivalent_rad(reference: f64, angle: f64) -> f64 {
    reference + limit_rad(angle - reference)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target_at(x: f64, y: f64) -> PlannerTarget {
        PlannerTarget::new(4, [x, 0.0, y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0])
    }

    #[test]
    fn static_target_selects_front_armor() {
        let mut planner = YawPlanner::default();

        let plan = planner.plan(Some(target_at(3.0, 0.0)), 24.0);

        assert!(plan.control);
        assert_eq!(plan.selected_armor_index, 0);
        assert!(plan.target_yaw_rad.abs() < 1e-6);
        assert_eq!(
            plan.yaw_ref_rad.len(),
            YawPlannerConfig::default().preview_horizon
        );
    }

    #[test]
    fn spinning_target_builds_nonzero_rate_reference() {
        let mut planner = YawPlanner::default();
        let mut state = [0.0; PLANNER_STATE_LEN];
        state[XC] = 3.0;
        state[VYAW] = 2.0;
        state[R] = 0.20;

        let plan = planner.plan(Some(PlannerTarget::new(4, state)), 24.0);

        assert!(plan.control);
        assert!(plan.yaw_rate_ref_rad_s.iter().any(|rate| rate.abs() > 1e-3));
    }

    #[test]
    fn yaw_reference_unwraps_across_pi_boundary() {
        let mut planner = YawPlanner::default();
        let first = planner.plan(Some(target_at(3.0, -0.01)), 24.0);
        let second = planner.plan(Some(target_at(3.0, 0.01)), 24.0);

        assert!(first.control);
        assert!(second.control);
        assert!((second.yaw_ref_rad[0] - first.yaw_ref_rad[0]).abs() < 0.02);
    }

    #[test]
    fn unreachable_low_speed_target_returns_no_control() {
        let mut planner = YawPlanner::default();

        let plan = planner.plan(Some(target_at(100.0, 0.0)), 5.0);

        assert!(!plan.control);
    }
}
