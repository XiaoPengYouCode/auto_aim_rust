/// 针对装甲板场景高度特化的 PnpSolver
/// 使用 IPPE 方法，四个特征点

use tracing::error;

// 硬编码的世界坐标，满足 IPPE 的规范坐标系要求 (Z=0, 中心在原点)
pub const ARMOR_LIGHT_WEIGHT: f64 = 135.0;
pub const ARMOR_LIGHT_HEIGHT: f64 = 55.0;

// 世界坐标系点，原点在装甲板中心，Z=0平面，省去归一化
const ARMOR_WORLD_POINTS: [na::Point3<f64>; 4] = [
    na::Point3::new(-ARMOR_LIGHT_WEIGHT / 2.0, ARMOR_LIGHT_HEIGHT / 2.0, 0.0), // 左上
    na::Point3::new(-ARMOR_LIGHT_WEIGHT / 2.0, -ARMOR_LIGHT_HEIGHT / 2.0, 0.0), // 左下
    na::Point3::new(ARMOR_LIGHT_WEIGHT / 2.0, -ARMOR_LIGHT_HEIGHT / 2.0, 0.0), // 右下
    na::Point3::new(ARMOR_LIGHT_WEIGHT / 2.0, ARMOR_LIGHT_HEIGHT / 2.0, 0.0),  // 右上
];

// 世界坐标点，但只有X, Y分量，用于平移计算
const ARMOR_WORLD_POINTS_2D: [na::Point2<f64>; 4] = [
    na::Point2::new(-ARMOR_LIGHT_WEIGHT / 2.0, ARMOR_LIGHT_HEIGHT / 2.0), // 左上
    na::Point2::new(-ARMOR_LIGHT_WEIGHT / 2.0, -ARMOR_LIGHT_HEIGHT / 2.0), // 左下
    na::Point2::new(ARMOR_LIGHT_WEIGHT / 2.0, -ARMOR_LIGHT_HEIGHT / 2.0), // 右下
    na::Point2::new(ARMOR_LIGHT_WEIGHT / 2.0, ARMOR_LIGHT_HEIGHT / 2.0),  // 右上
];

/// 专为已知尺寸的平面4点目标设计的 pnp 求解器
/// 基于 IPPE PnP 求解器
#[derive(Debug, Clone)]
pub struct ArmorPnpSolver {
    pws_iso_mat_pinv: na::SMatrix<f64, 4, 3>,
    pws_iso_t_inv: na::Matrix3<f64>,
}

impl ArmorPnpSolver {
    /// 构建一个新的装甲板求解器
    /// 提前预计算 point_world_matrix 伪逆
    pub fn new() -> Option<Self> {
        if let Some((iso_norm_armor_world_points, iso_norm_pws_t_inv)) =
            isotropic_normalize(&ARMOR_WORLD_POINTS_2D)
        {
            let pws_matrix = na::SMatrix::<f64, 3, 4>::from_columns(&[
                iso_norm_armor_world_points[0].to_homogeneous().xyz(),
                iso_norm_armor_world_points[1].to_homogeneous().xyz(),
                iso_norm_armor_world_points[2].to_homogeneous().xyz(),
                iso_norm_armor_world_points[3].to_homogeneous().xyz(),
            ]);

            let svd = pws_matrix.svd(true, true);
            if let Ok(pws_mat_pinv) = svd.pseudo_inverse(1e-8) {
                Some(Self {
                    pws_iso_mat_pinv: pws_mat_pinv,
                    pws_iso_t_inv: iso_norm_pws_t_inv,
                })
            } else {
                None
            }
        } else {
            None
        }
    }

    /// 执行解算全部流程
    pub fn solve(
        &self,
        img_coord: &[na::Point2<f64>; 4],
        cam_k: &na::Matrix3<f64>,
    ) -> Option<na::Isometry3<f64>> {
        // 使用 IPPE 算法求解出两个可能解
        if let Some((pose1, pose2)) = self.solve_ippe(img_coord, cam_k) {
            // 简单判断解的合理性
            let pose1_valid = if self.is_pose_valid(&pose1) {
                Some(pose1)
            } else {
                None
            };
            let pose2_valid = if self.is_pose_valid(&pose2) {
                Some(pose2)
            } else {
                None
            };

            // 根据解的合理性情况返回最终解
            match (pose1_valid, pose2_valid) {
                (Some(p1), Some(p2)) => {
                    let err1 = self.eval_reproj_err(&p1, img_coord, cam_k);
                    let err2 = self.eval_reproj_err(&p2, img_coord, cam_k);
                    if err1 < err2 { Some(p1) } else { Some(p2) }
                }
                (Some(p1), None) => Some(p1),
                (None, Some(p2)) => Some(p2),
                (None, None) => None,
            }
        } else {
            None
        }
    }

    /// 核心求解步骤
    /// 1、根据相机内参对点进行归一化
    /// 2、根据 IPPE 方法计算出两个可能解
    fn solve_ippe(
        &self,
        uvs: &[na::Point2<f64>; 4],
        k_matrix: &na::Matrix3<f64>,
    ) -> Option<(na::Isometry3<f64>, na::Isometry3<f64>)> {
        let k_inv = k_matrix.try_inverse()?;
        let p_norm_homo_vec: Vec<_> = uvs.iter().map(|uv| k_inv * uv.to_homogeneous()).collect();

        let mut p_norm: [na::Point2<f64>; 4] = [na::Point2::origin(); 4];
        for (i, p_homo) in p_norm_homo_vec.iter().enumerate() {
            p_norm[i] = na::Point2::from(p_homo.xy() / p_homo.z);
        }

        if let Some((p_iso_norm, img_p_iso_t_inv)) = isotropic_normalize(&p_norm) {
            // 构造归一化图像点的齐次坐标矩阵
            let p_norm_matrix = na::SMatrix::<f64, 3, 4>::from_columns(&[
                p_iso_norm[0].to_homogeneous().xyz(),
                p_iso_norm[1].to_homogeneous().xyz(),
                p_iso_norm[2].to_homogeneous().xyz(),
                p_iso_norm[3].to_homogeneous().xyz(),
            ]);

            // 计算单应性矩阵 h
            let mut h = p_norm_matrix * self.pws_iso_mat_pinv;

            h = img_p_iso_t_inv * h * self.pws_iso_t_inv;

            // 归一化单应性矩阵
            let h_2_2 = h[(2, 2)];
            if h_2_2.abs() < 1e-9 {
                // 如果 h[2,2] 接近于0，表明这是一个退化或不稳定的情况
                return None;
            }
            h /= h_2_2;

            // 现在 h[2,2] 等于 1，可以安全地使用以下公式计算雅可比矩阵
            let j00 = h[(0, 0)] - h[(2, 0)] * h[(0, 2)];
            let j01 = h[(0, 1)] - h[(2, 1)] * h[(0, 2)];
            let j10 = h[(1, 0)] - h[(2, 0)] * h[(1, 2)];
            let j11 = h[(1, 1)] - h[(2, 1)] * h[(1, 2)];

            let v0 = h[(0, 2)];
            let v1 = h[(1, 2)];

            let (r1, r2) = self.ippe_compute_rotations(j00, j01, j10, j11, v0, v1)?;

            // 平移计算使用原始的归一化点
            let t1 = self.ippe_compute_translation(&p_norm.to_vec(), &r1)?;
            let t2 = self.ippe_compute_translation(&p_norm.to_vec(), &r2)?;

            let pose1 = na::Isometry3::from_parts(t1.into(), na::UnitQuaternion::from_matrix(&r1));
            let pose2 = na::Isometry3::from_parts(t2.into(), na::UnitQuaternion::from_matrix(&r2));

            Some((pose1, pose2))
        } else {
            None
        }
    }

    /// 计算 rotation
    fn ippe_compute_rotations(
        &self,
        j00: f64,
        j01: f64,
        j10: f64,
        j11: f64,
        p: f64,
        q: f64,
    ) -> Option<(na::Matrix3<f64>, na::Matrix3<f64>)> {
        let v = na::Vector3::new(p, q, 1.0);
        let rv = rotate_vec_to_z_axis(&v);
        let rv_t = rv.transpose();

        let b00 = rv_t[(0, 0)] - p * rv_t[(2, 0)];
        let b01 = rv_t[(0, 1)] - p * rv_t[(2, 1)];
        let b10 = rv_t[(1, 0)] - q * rv_t[(2, 0)];
        let b11 = rv_t[(1, 1)] - q * rv_t[(2, 1)];

        let dtinv = (b00 * b11 - b01 * b10).recip();
        if dtinv.is_infinite() || dtinv.is_nan() {
            return None;
        }

        let binv00 = dtinv * b11;
        let binv01 = -dtinv * b01;
        let binv10 = -dtinv * b10;
        let binv11 = dtinv * b00;

        let a00 = binv00 * j00 + binv01 * j10;
        let a01 = binv00 * j01 + binv01 * j11;
        let a10 = binv10 * j00 + binv11 * j10;
        let a11 = binv10 * j01 + binv11 * j11;

        let ata00 = a00 * a00 + a01 * a01;
        let ata01 = a00 * a10 + a01 * a11;
        let ata11 = a10 * a10 + a11 * a11;

        let sqrt_expr = (ata00 - ata11).powi(2) + 4.0 * ata01 * ata01;
        if sqrt_expr < 0.0 {
            return None;
        }

        let gamma = (0.5 * (ata00 + ata11 + sqrt_expr.sqrt())).sqrt();
        if gamma == 0.0 {
            return None;
        }

        let gamma_inv = 1.0 / gamma;
        let r_tilde_00 = a00 * gamma_inv;
        let r_tilde_01 = a01 * gamma_inv;
        let r_tilde_10 = a10 * gamma_inv;
        let r_tilde_11 = a11 * gamma_inv;

        let b0_sq = 1.0 - r_tilde_00.powi(2) - r_tilde_10.powi(2);
        let b1_sq = 1.0 - r_tilde_01.powi(2) - r_tilde_11.powi(2);

        // Clamp to 0.0 to prevent NaNs from floating point inaccuracies
        let b0 = b0_sq.max(0.0).sqrt();
        let mut b1 = b1_sq.max(0.0).sqrt();

        let sp = -(r_tilde_00 * r_tilde_01 + r_tilde_10 * r_tilde_11);
        if sp < 0.0 {
            b1 = -b1;
        }

        let mut r1_tilde = na::Matrix3::zeros();
        r1_tilde.m11 = r_tilde_00;
        r1_tilde.m12 = r_tilde_01;
        r1_tilde.m21 = r_tilde_10;
        r1_tilde.m22 = r_tilde_11;
        r1_tilde.m31 = b0;
        r1_tilde.m32 = b1;
        r1_tilde.set_column(2, &r1_tilde.column(0).cross(&r1_tilde.column(1)));

        let mut r2_tilde = na::Matrix3::zeros();
        r2_tilde.m11 = r_tilde_00;
        r2_tilde.m12 = r_tilde_01;
        r2_tilde.m21 = r_tilde_10;
        r2_tilde.m22 = r_tilde_11;
        r2_tilde.m31 = -b0;
        r2_tilde.m32 = -b1;
        r2_tilde.set_column(2, &r2_tilde.column(0).cross(&r2_tilde.column(1)));

        let r1 = rv_t * r1_tilde;
        let r2 = rv_t * r2_tilde;

        Some((r1, r2))
    }

    /// 计算平移
    fn ippe_compute_translation(
        &self,
        p_norm: &Vec<na::Point2<f64>>,
        r_mat: &na::Matrix3<f64>,
    ) -> Option<na::Vector3<f64>> {
        let n_f64 = p_norm.len() as f64;
        let mut ata = na::Matrix3::<f64>::zeros();
        let mut atb = na::Vector3::<f64>::zeros();

        ata[(0, 0)] = n_f64;
        ata[(1, 1)] = n_f64;

        for i in 0..4 {
            let u = p_norm[i].x;
            let v = p_norm[i].y;
            let p_world_2d = ARMOR_WORLD_POINTS_2D[i];

            let rx = r_mat[(0, 0)] * p_world_2d.x + r_mat[(0, 1)] * p_world_2d.y;
            let ry = r_mat[(1, 0)] * p_world_2d.x + r_mat[(1, 1)] * p_world_2d.y;
            let rz = r_mat[(2, 0)] * p_world_2d.x + r_mat[(2, 1)] * p_world_2d.y;

            ata[(0, 2)] -= u;
            ata[(1, 2)] -= v;
            ata[(2, 2)] += u * u + v * v;

            let bx = u * rz - rx;
            let by = v * rz - ry;

            atb[0] += bx;
            atb[1] += by;
            atb[2] += -u * bx - v * by;
        }

        ata[(2, 0)] = ata[(0, 2)];
        ata[(2, 1)] = ata[(1, 2)];

        ata.try_inverse().map(|inv| inv * atb)
    }

    /// 检查姿态是否有效
    /// 逻辑较为简单，检查点是否在相机前方
    fn is_pose_valid(&self, pose: &na::Isometry3<f64>) -> bool {
        // 首先检查平移向量的 Z 分量，这是一个快速的初步筛选
        if pose.translation.vector.z <= 0.0 {
            return false;
        }
        // 确保所有点变换后都在相机前方
        for point in &ARMOR_WORLD_POINTS {
            if (pose * point).z <= 0.0 {
                return false;
            }
        }
        true
    }

    /// 计算重投影误差
    /// 用于对 IPPE 产生的两个解进行排序
    fn eval_reproj_err(
        &self,
        pose: &na::Isometry3<f64>,
        uvs: &[na::Point2<f64>; 4],
        k: &na::Matrix3<f64>,
    ) -> f64 {
        let mut sum_sq_err = 0.0;
        for i in 0..4 {
            let pw = &ARMOR_WORLD_POINTS[i];
            let pc = pose * pw; // 将世界点变换到相机坐标系

            // 检查点是否在相机前方
            if pc.z <= 1e-7 {
                // 使用一个小的epsilon避免除零
                return f64::MAX;
            }

            // 正确的投影步骤：k(3x3) * pc(3x1) -> projected_h(3x1)
            let projected_h = k * pc;

            // 执行透视除法
            let u_repro = projected_h.x / projected_h.z;
            let v_repro = projected_h.y / projected_h.z;

            sum_sq_err += (uvs[i].x - u_repro).powi(2) + (uvs[i].y - v_repro).powi(2);
        }
        let ass_err = (sum_sq_err / 4.0).sqrt();
        ass_err
    }
}

fn rotate_vec_to_z_axis(a: &na::Vector3<f64>) -> na::Matrix3<f64> {
    match nalgebra::Rotation3::rotation_between(&a.normalize(), &na::Vector3::z_axis()) {
        Some(rot) => rot.matrix().into_owned(),
        None => na::Matrix3::identity(), // 如果向量已经是Z轴，返回单位阵
    }
}

/// 各向同性归一化
///
/// ```md
/// - 输入：2D点集合
/// - 输出：归一化后数据(DataN)以及归一化变换矩阵(T)及其逆矩阵(Ti)
///
/// 执行步骤：
/// 1. 对输入的数据点进行中心化(减去均值)
/// 2. 计算缩放因子使得归一化后的点的平均欧氏距离为√2
/// 3. 计算并返回归一化后数据(DataN)以及归一化变换矩阵(T)及其逆矩阵(Ti)
/// ```
fn isotropic_normalize(
    input_points: &[na::Point2<f64>],
) -> Option<(Vec<na::Point2<f64>>, na::Matrix3<f64>)> {
    // 1. 处理空输入：如果点集为空，无法进行归一化，返回 None。
    if input_points.is_empty() {
        error!("the input points is empty");
        return None;
    }

    let n_f64 = input_points.len() as f64;

    // 计算质心
    let sum_coords = input_points
        .iter()
        .fold(na::Vector2::<f64>::zeros(), |acc, p| acc + p.coords);
    let center_point = na::Point2::from(sum_coords / n_f64);

    // 中心化点并计算 `kappa` (中心化后所有点到原点距离的平方和)
    let mut kappa = 0.0;
    let mut centered_points = input_points
        .iter()
        .map(|p| {
            let centered_p: na::Point2<f64> = na::Point2::from(*p - center_point);
            kappa += centered_p.coords.norm_squared();
            centered_p
        })
        .collect::<Vec<na::Point2<f64>>>();

    if kappa.abs() < f64::EPSILON {
        // 使用 epsilon 比较浮点数是否接近于零
        error!("检查输入点是否重合");
        return None;
    }

    // 计算缩放因子 `beta`，使得归一化后点的平方欧氏距离之和为 `2N`
    let beta = (2.0 * n_f64 / kappa).sqrt();
    centered_points.iter_mut().for_each(|p| *p *= beta);

    // 构建归一化变换矩阵 `T`
    // T 将归一化齐次坐标点 `P_normalized = (x_n, y_n, 1)`映射到原始齐次坐标点 `P_original = (x, y, 1)` 。
    // 对应的 3x3 齐次变换矩阵为：
    // T = [ 1 / beta     0     center_point.x ]
    //     [    0      1/ beta  center_point.y ]
    //     [    0         0           1        ]
    let transformation_matrix = na::Matrix3::new(
        1.0 / beta,
        0.0,
        center_point.x,
        0.0,
        1.0 / beta,
        center_point.y,
        0.0,
        0.0,
        1.0,
    );
    Some((centered_points, transformation_matrix))
}
