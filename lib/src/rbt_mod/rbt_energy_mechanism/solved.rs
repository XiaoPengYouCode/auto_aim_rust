use crate::rbt_infra::rbt_err::{RbtError, RbtResult};

use super::detected::{EnergyMechanismMode, EnergyMechanismObject};

const R_CENTER_KEYPOINT_INDEX: usize = 4;
const MODEL_TOP_M: na::Point3<f64> = na::Point3::new(0.0, 0.0, 0.827);
const MODEL_LEFT_M: na::Point3<f64> = na::Point3::new(0.0, -0.127, 0.700);
const MODEL_BOTTOM_M: na::Point3<f64> = na::Point3::new(0.0, 0.0, 0.573);
const MODEL_RIGHT_M: na::Point3<f64> = na::Point3::new(0.0, 0.127, 0.700);
const MODEL_BLADE_CENTER_M: na::Point3<f64> = na::Point3::new(0.0, 0.0, 0.700);
const MODEL_R_CENTER_M: na::Point3<f64> = na::Point3::new(0.0, 0.0, 0.0);
const MODEL_BLADE_RADIUS_M: f32 = 0.700;
const R_CENTER_GEOMETRY_MAX_ERROR_RATIO: f32 = 0.35;

#[derive(Debug, Clone, PartialEq)]
pub struct EnergyMechanismPose {
    pub rune_center_world_m: na::Point3<f64>,
    pub target_center_world_m: na::Point3<f64>,
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub roll_rad: f64,
    pub reprojection_error_px: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EnergyMechanismSolvedTarget {
    pub mode: EnergyMechanismMode,
    pub pose: EnergyMechanismPose,
    pub image_r_center: na::Point2<f32>,
    pub image_target_center: na::Point2<f32>,
    pub image_r_center_corrected: bool,
    pub confidence: f32,
    pub selected_phase_index: usize,
    pub observed_roll_rad: f64,
    pub switch_deferred: bool,
    pub target_switched: bool,
    pub selected_roll_offset_rad: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EnergyMechanismSolvedFrame {
    pub mode: EnergyMechanismMode,
    pub target: Option<EnergyMechanismSolvedTarget>,
    pub candidates: Vec<EnergyMechanismObject>,
}

pub fn solve_energy_mechanism(
    mode: EnergyMechanismMode,
    candidates: Vec<EnergyMechanismObject>,
    cam_k: &na::Matrix3<f64>,
) -> RbtResult<EnergyMechanismSolvedFrame> {
    let mut best = None;
    let mut corrected_candidates = Vec::with_capacity(candidates.len());
    for candidate in &candidates {
        let corrected = correct_candidate_r_center(candidate);
        let Some(pose) = solve_candidate_pose(&corrected.object, cam_k) else {
            corrected_candidates.push(corrected.object);
            continue;
        };
        let observed_roll_rad = observed_roll(&corrected.object);
        let selected_phase_index = phase_index_from_roll(observed_roll_rad);
        let target = EnergyMechanismSolvedTarget {
            mode,
            observed_roll_rad,
            pose,
            image_r_center: corrected.object.r_center(),
            image_target_center: corrected.object.target_center(),
            image_r_center_corrected: corrected.corrected,
            confidence: candidate.confidence,
            selected_phase_index,
            switch_deferred: false,
            target_switched: false,
            selected_roll_offset_rad: Some(phase_offset_rad(
                observed_roll_rad,
                selected_phase_index,
            )),
        };
        if best
            .as_ref()
            .is_none_or(|current: &EnergyMechanismSolvedTarget| {
                candidate.confidence > current.confidence
                    && target.pose.reprojection_error_px <= current.pose.reprojection_error_px * 2.0
            })
        {
            best = Some(target);
        }
        corrected_candidates.push(corrected.object);
    }

    Ok(EnergyMechanismSolvedFrame {
        mode,
        target: best,
        candidates: corrected_candidates,
    })
}

#[derive(Debug, Clone)]
struct CorrectedCandidate {
    object: EnergyMechanismObject,
    corrected: bool,
}

fn correct_candidate_r_center(candidate: &EnergyMechanismObject) -> CorrectedCandidate {
    let target = candidate.target_center();
    let observed_r = candidate.r_center();
    let geometry_r = geometry_r_center(candidate);
    let observed_radius = (observed_r - target).norm();
    let geometry_radius = geometry_radius(candidate);
    let corrected = observed_radius <= 1e-3
        || (observed_radius - geometry_radius).abs()
            > geometry_radius * R_CENTER_GEOMETRY_MAX_ERROR_RATIO;
    if corrected {
        let mut object = candidate.clone();
        object.keypoints[R_CENTER_KEYPOINT_INDEX] = geometry_r;
        CorrectedCandidate { object, corrected }
    } else {
        CorrectedCandidate {
            object: candidate.clone(),
            corrected,
        }
    }
}

fn geometry_r_center(candidate: &EnergyMechanismObject) -> na::Point2<f32> {
    let target = candidate.target_center();
    let observed_r = candidate.r_center();
    let radius = geometry_radius(candidate);
    let direction = observed_r - target;
    let direction = if direction.norm() > 1e-3 {
        direction.normalize()
    } else {
        na::Vector2::new(0.0, -1.0)
    };
    target + direction * radius
}

fn geometry_radius(candidate: &EnergyMechanismObject) -> f32 {
    let top = candidate.keypoints[0];
    let left = candidate.keypoints[1];
    let bottom = candidate.keypoints[2];
    let right = candidate.keypoints[3];
    let vertical = (top - bottom).norm();
    let horizontal = (left - right).norm();
    let blade_span_px = ((vertical + horizontal) * 0.5).max(1.0);
    blade_span_px * MODEL_BLADE_RADIUS_M / 0.254
}

fn solve_candidate_pose(
    candidate: &EnergyMechanismObject,
    cam_k: &na::Matrix3<f64>,
) -> Option<EnergyMechanismPose> {
    let image_points = [
        to_f64(candidate.keypoints[0]),
        to_f64(candidate.keypoints[1]),
        to_f64(candidate.keypoints[2]),
        to_f64(candidate.keypoints[3]),
        to_f64(candidate.keypoints[4]),
    ];
    if !image_points
        .iter()
        .all(|point| point.x.is_finite() && point.y.is_finite())
    {
        return None;
    }

    let object_points = [
        MODEL_TOP_M,
        MODEL_LEFT_M,
        MODEL_BOTTOM_M,
        MODEL_RIGHT_M,
        MODEL_R_CENTER_M,
    ];
    let pose = estimate_pose_planar(&object_points, &image_points, cam_k)?;
    if !pose
        .translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        || pose.translation.vector.z <= 0.05
    {
        return None;
    }

    let rune_center_camera = pose * MODEL_R_CENTER_M;
    let target_center_camera = pose * MODEL_BLADE_CENTER_M;
    let rune_center_world_m = camera_to_base_point_m(rune_center_camera);
    let target_center_world_m = camera_to_base_point_m(target_center_camera);
    let reprojection_error_px = reprojection_error(&pose, &object_points, &image_points, cam_k);
    if !reprojection_error_px.is_finite() || reprojection_error_px > 150.0 {
        return None;
    }

    let (_, pitch_rad, yaw_rad) = pose.rotation.euler_angles();
    Some(EnergyMechanismPose {
        rune_center_world_m,
        target_center_world_m,
        yaw_rad,
        pitch_rad,
        roll_rad: observed_roll(candidate),
        reprojection_error_px,
    })
}

fn estimate_pose_planar(
    object_points: &[na::Point3<f64>; 5],
    image_points: &[na::Point2<f64>; 5],
    cam_k: &na::Matrix3<f64>,
) -> Option<na::Isometry3<f64>> {
    let k_inv = cam_k.try_inverse()?;
    let mut a = na::DMatrix::<f64>::zeros(object_points.len() * 2, 9);
    for (idx, (world, image)) in object_points.iter().zip(image_points.iter()).enumerate() {
        let normalized = k_inv * image.to_homogeneous();
        if normalized.z.abs() <= f64::EPSILON {
            return None;
        }
        let x = normalized.x / normalized.z;
        let y = normalized.y / normalized.z;
        let row = idx * 2;
        let p = [world.y, world.z, 1.0];
        for col in 0..3 {
            a[(row, col)] = p[col];
            a[(row, 6 + col)] = -x * p[col];
            a[(row + 1, 3 + col)] = p[col];
            a[(row + 1, 6 + col)] = -y * p[col];
        }
    }

    let svd = a.svd(true, true);
    let v_t = svd.v_t?;
    let h = v_t.row(v_t.nrows() - 1);
    let mut homography = na::Matrix3::<f64>::zeros();
    for row in 0..3 {
        for col in 0..3 {
            homography[(row, col)] = h[row * 3 + col];
        }
    }

    let scale = (homography.column(0).norm() + homography.column(1).norm()) * 0.5;
    if !scale.is_finite() || scale <= 1e-9 {
        return None;
    }
    let mut r_y = homography.column(0).into_owned() / scale;
    let mut r_z = homography.column(1).into_owned() / scale;
    let mut t = homography.column(2).into_owned() / scale;
    if t.z < 0.0 {
        r_y = -r_y;
        r_z = -r_z;
        t = -t;
    }

    let r_x = r_y.cross(&r_z).normalize();
    r_y = r_z.cross(&r_x).normalize();
    r_z = r_x.cross(&r_y).normalize();
    let r_tilde = na::Matrix3::from_columns(&[r_x, r_y, r_z]);
    let svd_r = r_tilde.svd(true, true);
    let u = svd_r.u?;
    let v_t = svd_r.v_t?;
    let mut r = u * v_t;
    if r.determinant() < 0.0 {
        r = -r;
        t = -t;
    }

    Some(na::Isometry3::from_parts(
        na::Translation3::from(t),
        na::UnitQuaternion::from_matrix(&r),
    ))
}

fn reprojection_error(
    pose: &na::Isometry3<f64>,
    object_points: &[na::Point3<f64>; 5],
    image_points: &[na::Point2<f64>; 5],
    cam_k: &na::Matrix3<f64>,
) -> f64 {
    let mut error_sum = 0.0;
    for (object, image) in object_points.iter().zip(image_points.iter()) {
        let point_camera = pose * object;
        if point_camera.z <= 1e-9 {
            return f64::INFINITY;
        }
        let projected = cam_k * point_camera.coords;
        let predicted = na::Point2::new(projected.x / projected.z, projected.y / projected.z);
        error_sum += (predicted - image).norm();
    }
    error_sum / object_points.len() as f64
}

fn camera_to_base_point_m(point: na::Point3<f64>) -> na::Point3<f64> {
    na::Point3::new(point.z, -point.x, -point.y)
}

fn observed_roll(candidate: &EnergyMechanismObject) -> f64 {
    let target = candidate.target_center();
    let r_center = candidate.r_center();
    (target.y - r_center.y).atan2(target.x - r_center.x) as f64
}

fn phase_index_from_roll(roll_rad: f64) -> usize {
    let normalized = roll_rad.rem_euclid(std::f64::consts::TAU);
    ((normalized / (std::f64::consts::TAU / 5.0)).round() as usize) % 5
}

fn phase_offset_rad(roll_rad: f64, phase_index: usize) -> f64 {
    normalize_angle(roll_rad - phase_index as f64 * std::f64::consts::TAU / 5.0)
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

fn to_f64(point: na::Point2<f32>) -> na::Point2<f64> {
    na::Point2::new(point.x as f64, point.y as f64)
}

#[allow(dead_code)]
fn invalid_pose(message: impl Into<String>) -> RbtError {
    RbtError::StringError(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_energy_mechanism::detected::{
        EnergyMechanismBBox, EnergyMechanismClass,
    };

    fn camera_matrix() -> na::Matrix3<f64> {
        na::Matrix3::new(800.0, 0.0, 320.0, 0.0, 800.0, 320.0, 0.0, 0.0, 1.0)
    }

    fn project(point: na::Point3<f64>, cam_k: &na::Matrix3<f64>) -> na::Point2<f32> {
        let projected = cam_k * point.coords;
        na::Point2::new(
            (projected.x / projected.z) as f32,
            (projected.y / projected.z) as f32,
        )
    }

    fn image_object_with_r_center(r_center: na::Point2<f32>) -> EnergyMechanismObject {
        EnergyMechanismObject {
            bbox: EnergyMechanismBBox::from_center_size(320.0, 320.0, 140.0, 140.0),
            class: EnergyMechanismClass::RedTarget,
            confidence: 0.9,
            keypoints: [
                na::Point2::new(320.0, 260.0),
                na::Point2::new(290.0, 320.0),
                na::Point2::new(320.0, 380.0),
                na::Point2::new(350.0, 320.0),
                r_center,
            ],
        }
    }

    #[test]
    fn solves_valid_energy_mechanism_target() {
        let cam_k = camera_matrix();
        let rotation = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
            na::Vector3::new(0.0, 0.0, 1.0),
            na::Vector3::new(1.0, 0.0, 0.0),
            na::Vector3::new(0.0, 1.0, 0.0),
        ]));
        let pose = na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, 3.0),
            na::UnitQuaternion::from_rotation_matrix(&rotation),
        );
        let keypoints = [
            project(pose * MODEL_TOP_M, &cam_k),
            project(pose * MODEL_LEFT_M, &cam_k),
            project(pose * MODEL_BOTTOM_M, &cam_k),
            project(pose * MODEL_RIGHT_M, &cam_k),
            project(pose * MODEL_R_CENTER_M, &cam_k),
        ];
        let object = EnergyMechanismObject {
            bbox: EnergyMechanismBBox::from_center_size(320.0, 320.0, 100.0, 100.0),
            class: EnergyMechanismClass::RedTarget,
            confidence: 0.9,
            keypoints,
        };

        let solved = solve_energy_mechanism(EnergyMechanismMode::Small, vec![object], &cam_k)
            .unwrap()
            .target
            .unwrap();

        assert!(solved.pose.reprojection_error_px < 1.0);
        assert!((solved.pose.rune_center_world_m.x - 3.0).abs() < 1e-3);
    }

    #[test]
    fn rejects_degenerate_points() {
        let cam_k = camera_matrix();
        let object = EnergyMechanismObject {
            bbox: EnergyMechanismBBox::from_center_size(320.0, 320.0, 100.0, 100.0),
            class: EnergyMechanismClass::RedTarget,
            confidence: 0.9,
            keypoints: [na::Point2::new(1.0, 1.0); 5],
        };

        let solved =
            solve_energy_mechanism(EnergyMechanismMode::Large, vec![object], &cam_k).unwrap();

        assert!(solved.target.is_none());
    }

    #[test]
    fn corrects_bad_r_center_to_geometry_radius() {
        let object = image_object_with_r_center(na::Point2::new(320.0, 319.0));
        let corrected = correct_candidate_r_center(&object);
        let target = object.target_center();
        let expected_radius = geometry_radius(&object);

        assert!(corrected.corrected);
        assert!(((corrected.object.r_center() - target).norm() - expected_radius).abs() < 1e-3);
    }

    #[test]
    fn keeps_consistent_r_center_without_correction() {
        let object = image_object_with_r_center(na::Point2::new(320.0, 650.0));
        let corrected = correct_candidate_r_center(&object);

        assert!(!corrected.corrected);
        assert_eq!(corrected.object.r_center(), object.r_center());
    }
}
