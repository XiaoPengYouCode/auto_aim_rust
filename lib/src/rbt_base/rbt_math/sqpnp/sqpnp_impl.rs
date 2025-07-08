use tracing::debug;
use super::sqpnp_def::SQPSolution;
use super::PnpSolver;

/// const变量
impl PnpSolver {
    pub const SQRT3: f64 = 1.7320508075688772; // sqrt(3)
    pub const NORM_THRESHOLD: f64 = 0.1;
}

/// public 实例方法
impl PnpSolver {
    pub fn solve(&mut self) -> Result<(), String> {
        let mut min_sq_error = f64::MAX;
        let num_eigen_points = if self.num_null_vectors > 0 {
            self.num_null_vectors
        } else {
            1
        };

        self.solutions.clear();
        self.num_solutions = 0;

        for i in (9 - num_eigen_points)..9 {
            let e = Self::SQRT3 * self.u.column(i as usize);
            let orthogonality_sq_error = Self::orthogonality_error(&e);

            let mut solution = [SQPSolution::default(), SQPSolution::default()];

            if orthogonality_sq_error < self.parameters.orthogonality_squared_error_threshold {
                solution[0].r_hat = Self::determinant9x1(&e) * e;
                solution[0].t = self.p * solution[0].r_hat.clone();
                solution[0].num_iterations = 0;
                self.handle_solution(&mut solution[0], &mut min_sq_error);
            } else {
                self.parameters
                    .nearest_rotation_method
                    .nearest_rotation_matrix(&e, &mut solution[0].r);
                solution[0] = self.run_sqp(&solution[0].r);
                solution[0].t = self.p * solution[0].r_hat.clone();
                self.handle_solution(&mut solution[0], &mut min_sq_error);

                self.parameters
                    .nearest_rotation_method
                    .nearest_rotation_matrix(&(-e), &mut solution[1].r);
                solution[1] = self.run_sqp(&solution[1].r);
                solution[1].t = self.p * solution[1].r_hat.clone();
                self.handle_solution(&mut solution[1], &mut min_sq_error);
            }
        }

        let mut c = 1;
        let mut index = 9 - num_eigen_points - c;
        while index > 0 && min_sq_error > (3.0 * self.s[index as usize]) {
            let e = self.u.column(index as usize).clone_owned();
            let mut solution = [SQPSolution::default(), SQPSolution::default()];

            self.parameters
                .nearest_rotation_method
                .nearest_rotation_matrix(&e, &mut solution[0].r);
            solution[0] = self.run_sqp(&solution[0].r);
            solution[0].t = self.p * solution[0].r_hat.clone();
            self.handle_solution(&mut solution[0], &mut min_sq_error);

            self.parameters
                .nearest_rotation_method
                .nearest_rotation_matrix(&(-e), &mut solution[1].r);
            solution[1] = self.run_sqp(&solution[1].r);
            solution[1].t = self.p * solution[1].r_hat.clone();
            self.handle_solution(&mut solution[1], &mut min_sq_error);

            c += 1;
            index = 9 - num_eigen_points - c;
        }

        Ok(())
    }
}

/// 实例方法
impl PnpSolver {
    /// Run sequential quadratic programming on orthogonal matrices
    fn run_sqp(&mut self, r0: &na::SVector<f64, 9>) -> SQPSolution {
        let mut r = r0.clone();

        let mut delta_squared_norm = f64::MAX;
        let mut delta = na::SVector::<f64, 9>::zeros();
        let mut step = 0;
        // step += 1;

        while delta_squared_norm > self.parameters.sqp_squared_tolerance
            && step < self.parameters.sqp_max_iteration
        {
            step += 1;
            self.solve_sqp_system(&r, &mut delta);
            r += delta;
            delta_squared_norm = delta.norm_squared();
        }

        let mut solution = SQPSolution::default();
        solution.num_iterations = step;
        solution.r = r.clone();

        // Clear the estimate and/or flip the matrix sign if necessary
        let mut det_r = Self::determinant9x1(&solution.r);
        if det_r < 0.0 {
            solution.r = -r;
            det_r = -det_r;
        }

        if det_r > self.parameters.sqp_det_threshold {
            // 使用函数指针计算结果
            self.parameters
                .nearest_rotation_method
                .nearest_rotation_matrix(&solution.r, &mut solution.r_hat);
        } else {
            solution.r_hat = solution.r.clone();
        }

        solution
    }

    // Solve the SQP system efficiently
    fn solve_sqp_system(&mut self, r: &na::SVector<f64, 9>, delta: &mut na::SVector<f64, 9>) {
        let sqnorm_r1 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        let sqnorm_r2 = r[3] * r[3] + r[4] * r[4] + r[5] * r[5];
        let sqnorm_r3 = r[6] * r[6] + r[7] * r[7] + r[8] * r[8];
        let dot_r1r2 = r[0] * r[3] + r[1] * r[4] + r[2] * r[5];
        let dot_r1r3 = r[0] * r[6] + r[1] * r[7] + r[2] * r[8];
        let dot_r2r3 = r[3] * r[6] + r[4] * r[7] + r[5] * r[8];

        // Obtain 6D normal (H) and 3D null space of the constraint Jacobian-J at the estimate (r)
        // NOTE: This is done via Gram-Schmidt orthogonalization
        let mut n = na::SMatrix::<f64, 9, 3>::zeros(); // Null space of J
        let mut h = na::SMatrix::<f64, 9, 6>::zeros(); // Row space of J
        let mut jh = na::SMatrix::<f64, 6, 6>::zeros(); // The lower triangular matrix J*Q

        self.row_and_null_space(r, &mut h, &mut n, &mut jh);

        let mut g = na::SVector::<f64, 6>::zeros();
        g[0] = 1.0 - sqnorm_r1;
        g[1] = 1.0 - sqnorm_r2;
        g[2] = 1.0 - sqnorm_r3;
        g[3] = -dot_r1r2;
        g[4] = -dot_r2r3;
        g[5] = -dot_r1r3;

        let mut x = na::SVector::<f64, 6>::zeros();
        x[0] = g[0] / jh[(0, 0)];
        x[1] = g[1] / jh[(1, 1)];
        x[2] = g[2] / jh[(2, 2)];
        x[3] = (g[3] - jh[(3, 0)] * x[0] - jh[(3, 1)] * x[1]) / jh[(3, 3)];
        x[4] = (g[4] - jh[(4, 1)] * x[1] - jh[(4, 2)] * x[2] - jh[(4, 3)] * x[3]) / jh[(4, 4)];
        x[5] =
            (g[5] - jh[(5, 0)] * x[0] - jh[(5, 2)] * x[2] - jh[(5, 3)] * x[3] - jh[(5, 4)] * x[4])
                / jh[(5, 5)];

        // Now obtain the component of delta in the row space of E as delta_h = H*x and assign straight into delta
        *delta = h * x;

        // Then, solve for y from W*y = ksi, where matrix W and vector ksi are :
        //
        // W = N'*Omega*N and ksi = -N'*Omega*( r + delta_h );
        let nt_omega = n.transpose() * self.omega;
        let w = nt_omega * n;
        let rhs = -(nt_omega * (*delta + r));
        let mut y = na::SVector::<f64, 3>::zeros();

        // Solve with LDLt and if it fails, use inverse
        if Self::axb_solve_ldlt_3x3(&w, &rhs, &mut y).is_err() {
            debug!("LDLt solve failed, falling back to matrix inverse.");
            // println!("W:\n{}", w);
            let w_inv = w.try_inverse().expect("Matrix inversion failed");
            y = w_inv * rhs;
        }

        // Finally, accumulate delta with component in tangent space (delta_n)
        *delta += n * y;
    }

    fn handle_solution(&mut self, solution: &mut SQPSolution, min_sq_error: &mut f64) {
        let cheirok =
            self.test_positive_depth(solution) || self.test_positive_majority_depths(solution);

        if cheirok {
            solution.sq_error = (self.omega * solution.r_hat).dot(&solution.r_hat);

            if (*min_sq_error - solution.sq_error).abs() > self.parameters.equal_squared_errors_diff
            {
                if *min_sq_error > solution.sq_error {
                    *min_sq_error = solution.sq_error;
                    // self.solutions[0] = solution.clone();
                    self.solutions.clear();
                    self.solutions.push(solution.clone());
                    self.num_solutions = 1;
                }
            } else {
                let mut found = false;

                for existing_solution in self.solutions.iter_mut() {
                    if (existing_solution.r_hat - solution.r_hat).norm_squared()
                        < self.parameters.equal_vectors_squared_diff
                    {
                        if existing_solution.sq_error > solution.sq_error {
                            *existing_solution = solution.clone();
                        }
                        found = true;
                        break;
                    }
                }

                if !found {
                    // self.solutions.push(solution.clone());
                    self.solutions[self.num_solutions as usize] = solution.clone();
                    self.num_solutions += 1;
                }

                if *min_sq_error > solution.sq_error {
                    *min_sq_error = solution.sq_error;
                }
            }
        }
    }

    // Average squared projection error of a given solution
    #[allow(unused)]
    #[inline(always)]
    fn average_squared_projection_error(&self, index: usize) -> f64 {
        let mut avg = 0.0;
        let r = &self.solutions[index].r_hat;
        let t = &self.solutions[index].t;

        for i in 0..self.points.len() {
            let m = &self.points[i];
            let x_c = r[0] * m[0] + r[1] * m[1] + r[2] * m[2] + t[0];
            let y_c = r[3] * m[0] + r[4] * m[1] + r[5] * m[2] + t[1];
            let inv_z_c = 1.0 / (r[6] * m[0] + r[7] * m[1] + r[8] * m[2] + t[2]);

            let m = &self.projections[i];
            let dx = x_c * inv_z_c - m[0];
            let dy = y_c * inv_z_c - m[1];
            avg += dx * dx + dy * dy;
        }

        avg / self.points.len() as f64
    }

    // Test cheirality on the mean point for a given solution
    fn test_positive_depth(self: &PnpSolver, solution: &SQPSolution) -> bool {
        let r = &solution.r_hat;
        let t = &solution.t;
        let m = self.point_mean;
        r[6] * m[0] + r[7] * m[1] + r[8] * m[2] + t[2] > 0.0
    }

    // Test cheirality on the majority of points for a given solution
    fn test_positive_majority_depths(self: &PnpSolver, solution: &SQPSolution) -> bool {
        let r = &solution.r_hat;
        let t = &solution.t;
        let mut npos = 0;
        let mut nneg = 0;

        for (i, point) in self.points.iter().enumerate() {
            if self.weights[i] == 0.0 {
                continue;
            }
            let m = &point;
            if r[6] * m[0] + r[7] * m[1] + r[8] * m[2] + t[2] > 0.0 {
                npos += 1;
            } else {
                nneg += 1;
            }
        }

        npos >= nneg
    }
}

/// 关联方法
impl PnpSolver {
    // Invert a 3x3 symmetric matrix (using low triangle values only)
    #[allow(unused)]
    fn invert_symmetric3x3(
        q: na::SMatrix<f64, 3, 3>,
        q_inv: &mut na::SMatrix<f64, 3, 3>,
        det_threshold: f64,
    ) -> bool {
        // 1. Get the elements of the matrix
        let a = q[(0, 0)];
        let b = q[(1, 0)];
        let d = q[(1, 1)];
        let c = q[(2, 0)];
        let e = q[(2, 1)];
        let f = q[(2, 2)];

        // 2. Determinant
        let t2 = e * e;
        let t4 = a * d;
        let t7 = b * b;
        let t9 = b * c;
        let t12 = c * c;
        let det = -t4 * f + a * t2 + t7 * f - 2.0 * t9 * e + t12 * d;

        if det.abs() < det_threshold {
            let svd = na::SVD::new(q.clone(), true, true);
            *q_inv = svd.pseudo_inverse(det_threshold).unwrap();
            return false;
        } // fall back to pseudoinverse

        // 3. Inverse
        let t15 = 1.0 / det;
        let t20 = (-b * f + c * e) * t15;
        let t24 = (b * e - c * d) * t15;
        let t30 = (a * e - t9) * t15;

        q_inv[(0, 0)] = (-d * f + t2) * t15;
        q_inv[(0, 1)] = -t20;
        q_inv[(1, 0)] = -t20;
        q_inv[(0, 2)] = -t24;
        q_inv[(2, 0)] = -t24;
        q_inv[(1, 1)] = -(a * f - t12) * t15;
        q_inv[(1, 2)] = t30;
        q_inv[(2, 1)] = t30;
        q_inv[(2, 2)] = -(t4 - t7) * t15;

        return true;
    }

    // Simple SVD-based nearest rotation matrix. Argument should be a *row-major* matrix representation.
    // Returns a row-major vector representation of the nearest rotation matrix.
    pub fn nearest_rotation_matrix_svd(e: &na::SVector<f64, 9>, r: &mut na::SVector<f64, 9>) {
        let e_matrix = na::SMatrix::<f64, 3, 3>::from_column_slice(e.as_slice());
        let svd = e_matrix.svd(true, true);
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let v = v_t.transpose();

        let det_uv = u.determinant() * v.determinant();
        let s = na::SMatrix::<f64, 3, 3>::from_diagonal(&na::Vector3::new(1.0, 1.0, det_uv));
        let r_matrix = u * s * v_t;
        *r = na::SVector::<f64, 9>::from_column_slice(r_matrix.as_slice());
    }

    // Faster nearest rotation computation based on FOAM.
    pub fn nearest_rotation_matrix_foam(e: &na::SVector<f64, 9>, r: &mut na::SVector<f64, 9>) {
        let b = e.as_slice();
        let mut adj_b = [0.0; 9];
        let mut l = 0.5 * (b.iter().map(|x| x * x).sum::<f64>() + 3.0);
        let det_b = b[0] * b[4] * b[8] - b[0] * b[5] * b[7] - b[1] * b[3] * b[8]
            + b[2] * b[3] * b[7]
            + b[1] * b[6] * b[5]
            - b[2] * b[6] * b[4];

        if det_b.abs() < 1e-4 {
            Self::nearest_rotation_matrix_svd(e, r);
            return;
        }

        adj_b[0] = b[4] * b[8] - b[5] * b[7];
        adj_b[1] = b[2] * b[7] - b[1] * b[8];
        adj_b[2] = b[1] * b[5] - b[2] * b[4];
        adj_b[3] = b[5] * b[6] - b[3] * b[8];
        adj_b[4] = b[0] * b[8] - b[2] * b[6];
        adj_b[5] = b[2] * b[3] - b[0] * b[5];
        adj_b[6] = b[3] * b[7] - b[4] * b[6];
        adj_b[7] = b[1] * b[6] - b[0] * b[7];
        adj_b[8] = b[0] * b[4] - b[1] * b[3];

        let b_sq = b.iter().map(|x| x * x).sum::<f64>();
        let adj_b_sq = adj_b.iter().map(|x| x * x).sum::<f64>();

        if det_b < 0.0 {
            l = -l;
        }

        for _ in 0..15 {
            let tmp = l * l - b_sq;
            let p = tmp * tmp - 8.0 * l * det_b - 4.0 * adj_b_sq;
            let pp = 8.0 * (0.5 * tmp * l - det_b);
            let l_prev = l;
            l -= p / pp;
            if (l - l_prev).abs() < 1e-12 * l_prev.abs() {
                break;
            }
        }

        let a = l * l + b_sq;
        let mut bb_t = [0.0; 9];
        bb_t[0] = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
        bb_t[1] = b[0] * b[3] + b[1] * b[4] + b[2] * b[5];
        bb_t[2] = b[0] * b[6] + b[1] * b[7] + b[2] * b[8];
        bb_t[3] = bb_t[1];
        bb_t[4] = b[3] * b[3] + b[4] * b[4] + b[5] * b[5];
        bb_t[5] = b[3] * b[6] + b[4] * b[7] + b[5] * b[8];
        bb_t[6] = bb_t[2];
        bb_t[7] = bb_t[5];
        bb_t[8] = b[6] * b[6] + b[7] * b[7] + b[8] * b[8];

        let mut tmp = [0.0; 9];
        tmp[0] = bb_t[0] * b[0] + bb_t[1] * b[3] + bb_t[2] * b[6];
        tmp[1] = bb_t[0] * b[1] + bb_t[1] * b[4] + bb_t[2] * b[7];
        tmp[2] = bb_t[0] * b[2] + bb_t[1] * b[5] + bb_t[2] * b[8];
        tmp[3] = bb_t[3] * b[0] + bb_t[4] * b[3] + bb_t[5] * b[6];
        tmp[4] = bb_t[3] * b[1] + bb_t[4] * b[4] + bb_t[5] * b[7];
        tmp[5] = bb_t[3] * b[2] + bb_t[4] * b[5] + bb_t[5] * b[8];
        tmp[6] = bb_t[6] * b[0] + bb_t[7] * b[3] + bb_t[8] * b[6];
        tmp[7] = bb_t[6] * b[1] + bb_t[7] * b[4] + bb_t[8] * b[7];
        tmp[8] = bb_t[6] * b[2] + bb_t[7] * b[5] + bb_t[8] * b[8];

        let denom = 1.0 / (l * (l * l - b_sq) - 2.0 * det_b);
        r[0] = (a * b[0] + 2.0 * (l * adj_b[0] - tmp[0])) * denom;
        r[1] = (a * b[1] + 2.0 * (l * adj_b[3] - tmp[1])) * denom;
        r[2] = (a * b[2] + 2.0 * (l * adj_b[6] - tmp[2])) * denom;
        r[3] = (a * b[3] + 2.0 * (l * adj_b[1] - tmp[3])) * denom;
        r[4] = (a * b[4] + 2.0 * (l * adj_b[4] - tmp[4])) * denom;
        r[5] = (a * b[5] + 2.0 * (l * adj_b[7] - tmp[5])) * denom;
        r[6] = (a * b[6] + 2.0 * (l * adj_b[2] - tmp[6])) * denom;
        r[7] = (a * b[7] + 2.0 * (l * adj_b[5] - tmp[7])) * denom;
        r[8] = (a * b[8] + 2.0 * (l * adj_b[8] - tmp[8])) * denom;
    }

    // /// Solve A*x=b for 3x3 SPD A.
    // /// The solution involves computing a lower triangular sqrt-free Cholesky factor
    // /// A=L*D*L' (L has ones on its diagonal, D is diagonal).
    // ///
    // /// Only the lower triangular part of A is accessed.
    // ///
    // /// The function returns 0 if successful, non-zero otherwise
    // ///
    // /// see http://euler.nmt.edu/~brian/ldlt.html
    // ///
    // fn axb_solve_ldlt_3x3(
    //     a: &na::SMatrix<f64, 3, 3>,
    //     b: &na::SVector<f64, 3>,
    //     x: &mut na::SVector<f64, 3>,
    // ) -> Result<(), &'static str> {
    //     let mut l = na::SMatrix::<f64, 3, 3>::zeros();

    //     // D is stored in L's diagonal, i.e., l[(0, 0)], l[(1, 1)], l[(2, 2)]
    //     // its elements should be positive
    //     l[(0, 0)] = a[(0, 0)];
    //     if l[(0, 0)] < 1e-10 {
    //         return Err("Matrix is not positive definite");
    //     }
    //     let inv_l00 = 1.0 / l[(0, 0)];
    //     l[(1, 0)] = a[(1, 0)] * inv_l00;
    //     l[(2, 0)] = a[(2, 0)] * inv_l00;

    //     l[(1, 1)] = a[(1, 1)] - l[(1, 0)] * l[(1, 0)] * l[(0, 0)];
    //     if l[(1, 1)] < 1e-10 {
    //         return Err("Matrix is not positive definite");
    //     }
    //     let inv_l11 = 1.0 / l[(1, 1)];
    //     l[(2, 1)] = (a[(2, 1)] - l[(2, 0)] * l[(1, 0)] * l[(0, 0)]) * inv_l11;

    //     l[(2, 2)] =
    //         a[(2, 2)] - l[(2, 0)] * l[(2, 0)] * l[(0, 0)] - l[(2, 1)] * l[(2, 1)] * l[(1, 1)];
    //     if l[(2, 2)] < 1e-10 {
    //         return Err("Matrix is not positive definite");
    //     }

    //     // Forward solve L*x = b
    //     x[0] = b[0];
    //     x[1] = b[1] - l[(1, 0)] * x[0];
    //     x[2] = b[2] - l[(2, 0)] * x[0] - l[(2, 1)] * x[1];

    //     // Backward solve D*L'*x = y
    //     x[2] /= l[(2, 2)];
    //     x[1] = x[1] / l[(1, 1)] - l[(2, 1)] * x[2];
    //     x[0] = x[0] / l[(0, 0)] - l[(1, 0)] * x[1] - l[(2, 0)] * x[2];

    //     Ok(())
    // }

    // Determinant of 3x3 matrix stored as a 9x1 vector in *row-major* order
    #[inline(always)]
    fn determinant9x1(r: &na::SVector<f64, 9>) -> f64 {
        return (r[0] * r[4] * r[8] + r[1] * r[5] * r[6] + r[2] * r[3] * r[7])
            - (r[6] * r[4] * r[2] + r[7] * r[5] * r[0] + r[8] * r[3] * r[1]);
    }

    // Determinant of 3x3 matrix
    #[allow(unused)]
    #[inline(always)]
    fn determinant3x3(m: &na::SMatrix<f64, 3, 3>) -> f64 {
        return m[(0, 0)] * (m[(1, 1)] * m[(2, 2)] - m[(1, 2)] * m[(2, 1)])
            - m[(0, 1)] * (m[(1, 0)] * m[(2, 2)] - m[(1, 2)] * m[(2, 0)])
            + m[(0, 2)] * (m[(1, 0)] * m[(2, 1)] - m[(1, 1)] * m[(2, 0)]);
    }

    /// 计算正交性误差
    #[inline(always)]
    fn orthogonality_error(a: &na::SMatrix<f64, 9, 1>) -> f64 {
        let sq_norm_a1 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        let sq_norm_a2 = a[3] * a[3] + a[4] * a[4] + a[5] * a[5];
        let sq_norm_a3 = a[6] * a[6] + a[7] * a[7] + a[8] * a[8];
        let dot_a1a2 = a[0] * a[3] + a[1] * a[4] + a[2] * a[5];
        let dot_a1a3 = a[0] * a[6] + a[1] * a[7] + a[2] * a[8];
        let dot_a2a3 = a[3] * a[6] + a[4] * a[7] + a[5] * a[8];

        ((sq_norm_a1 - 1.0) * (sq_norm_a1 - 1.0) + (sq_norm_a2 - 1.0) * (sq_norm_a2 - 1.0))
            + ((sq_norm_a3 - 1.0) * (sq_norm_a3 - 1.0)
                + 2.0 * (dot_a1a2 * dot_a1a2 + dot_a1a3 * dot_a1a3 + dot_a2a3 * dot_a2a3))
    }

    // Compute the 3D null space (N) and 6D normal space (H) of the constraint Jacobian at a 9D vector r
    // (r is not necessarily a rotation but it must represent a rank-3 matrix)
    // NOTE: K is lower-triangular, so upper triangle may contain trash (is not filled by the function)...
    fn row_and_null_space(
        self: &Self,
        r: &na::SVector<f64, 9>,
        h: &mut na::SMatrix<f64, 9, 6>, // Row space
        n: &mut na::SMatrix<f64, 9, 3>, // Null space
        k: &mut na::SMatrix<f64, 6, 6>, // J*Q (J - Jacobian of constraints)
    ) {
        // Applying Gram-Schmidt orthogonalization on the Jacobian.
        // The steps are fixed here to take advantage of the sparse form of the matrix
        *h = na::SMatrix::<f64, 9, 6>::zeros();

        // 1. q1
        let norm_r1 = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        let inv_norm_r1 = if norm_r1 > 1e-5 { 1.0 / norm_r1 } else { 0.0 };
        h[(0, 0)] = r[0] * inv_norm_r1;
        h[(1, 0)] = r[1] * inv_norm_r1;
        h[(2, 0)] = r[2] * inv_norm_r1;
        k[(0, 0)] = 2.0 * norm_r1;

        // 2. q2
        let norm_r2 = (r[3] * r[3] + r[4] * r[4] + r[5] * r[5]).sqrt();
        let inv_norm_r2 = 1.0 / norm_r2;
        h[(3, 1)] = r[3] * inv_norm_r2;
        h[(4, 1)] = r[4] * inv_norm_r2;
        h[(5, 1)] = r[5] * inv_norm_r2;
        k[(1, 0)] = 0.0;
        k[(1, 1)] = 2.0 * norm_r2;

        // 3. q3
        let norm_r3 = (r[6] * r[6] + r[7] * r[7] + r[8] * r[8]).sqrt();
        let inv_norm_r3 = 1.0 / norm_r3;
        h[(6, 2)] = r[6] * inv_norm_r3;
        h[(7, 2)] = r[7] * inv_norm_r3;
        h[(8, 2)] = r[8] * inv_norm_r3;
        k[(2, 0)] = 0.0;
        k[(2, 1)] = 0.0;
        k[(2, 2)] = 2.0 * norm_r3;

        // 4. q4
        let dot_j4q1 = r[3] * h[(0, 0)] + r[4] * h[(1, 0)] + r[5] * h[(2, 0)];
        let dot_j4q2 = r[0] * h[(3, 1)] + r[1] * h[(4, 1)] + r[2] * h[(5, 1)];

        h[(0, 3)] = r[3] - dot_j4q1 * h[(0, 0)];
        h[(1, 3)] = r[4] - dot_j4q1 * h[(1, 0)];
        h[(2, 3)] = r[5] - dot_j4q1 * h[(2, 0)];
        h[(3, 3)] = r[0] - dot_j4q2 * h[(3, 1)];
        h[(4, 3)] = r[1] - dot_j4q2 * h[(4, 1)];
        h[(5, 3)] = r[2] - dot_j4q2 * h[(5, 1)];
        let inv_norm_j4 = 1.0
            / (h[(0, 3)] * h[(0, 3)]
                + h[(1, 3)] * h[(1, 3)]
                + h[(2, 3)] * h[(2, 3)]
                + h[(3, 3)] * h[(3, 3)]
                + h[(4, 3)] * h[(4, 3)]
                + h[(5, 3)] * h[(5, 3)])
                .sqrt();

        h[(0, 3)] *= inv_norm_j4;
        h[(1, 3)] *= inv_norm_j4;
        h[(2, 3)] *= inv_norm_j4;
        h[(3, 3)] *= inv_norm_j4;
        h[(4, 3)] *= inv_norm_j4;
        h[(5, 3)] *= inv_norm_j4;

        k[(3, 0)] = r[3] * h[(0, 0)] + r[4] * h[(1, 0)] + r[5] * h[(2, 0)];
        k[(3, 1)] = r[0] * h[(3, 1)] + r[1] * h[(4, 1)] + r[2] * h[(5, 1)];
        k[(3, 2)] = 0.0;
        k[(3, 3)] = r[3] * h[(0, 3)]
            + r[4] * h[(1, 3)]
            + r[5] * h[(2, 3)]
            + r[0] * h[(3, 3)]
            + r[1] * h[(4, 3)]
            + r[2] * h[(5, 3)];

        // 5. q5
        let dot_j5q2 = r[6] * h[(3, 1)] + r[7] * h[(4, 1)] + r[8] * h[(5, 1)];
        let dot_j5q3 = r[3] * h[(6, 2)] + r[4] * h[(7, 2)] + r[5] * h[(8, 2)];
        let dot_j5q4 = r[6] * h[(3, 3)] + r[7] * h[(4, 3)] + r[8] * h[(5, 3)];

        h[(0, 4)] = -dot_j5q4 * h[(0, 3)];
        h[(1, 4)] = -dot_j5q4 * h[(1, 3)];
        h[(2, 4)] = -dot_j5q4 * h[(2, 3)];
        h[(3, 4)] = r[6] - dot_j5q2 * h[(3, 1)] - dot_j5q4 * h[(3, 3)];
        h[(4, 4)] = r[7] - dot_j5q2 * h[(4, 1)] - dot_j5q4 * h[(4, 3)];
        h[(5, 4)] = r[8] - dot_j5q2 * h[(5, 1)] - dot_j5q4 * h[(5, 3)];
        h[(6, 4)] = r[3] - dot_j5q3 * h[(6, 2)];
        h[(7, 4)] = r[4] - dot_j5q3 * h[(7, 2)];
        h[(8, 4)] = r[5] - dot_j5q3 * h[(8, 2)];

        let _ = h.column_mut(4).normalize();

        k[(4, 0)] = 0.0;
        k[(4, 1)] = r[6] * h[(3, 1)] + r[7] * h[(4, 1)] + r[8] * h[(5, 1)];
        k[(4, 2)] = r[3] * h[(6, 2)] + r[4] * h[(7, 2)] + r[5] * h[(8, 2)];
        k[(4, 3)] = r[6] * h[(3, 3)] + r[7] * h[(4, 3)] + r[8] * h[(5, 3)];
        k[(4, 4)] = r[6] * h[(3, 4)]
            + r[7] * h[(4, 4)]
            + r[8] * h[(5, 4)]
            + r[3] * h[(6, 4)]
            + r[4] * h[(7, 4)]
            + r[5] * h[(8, 4)];

        // 6. q6
        let dot_j6q1 = r[6] * h[(0, 0)] + r[7] * h[(1, 0)] + r[8] * h[(2, 0)];
        let dot_j6q3 = r[0] * h[(6, 2)] + r[1] * h[(7, 2)] + r[2] * h[(8, 2)];
        let dot_j6q4 = r[6] * h[(0, 3)] + r[7] * h[(1, 3)] + r[8] * h[(2, 3)];
        let dot_j6q5 = r[0] * h[(6, 4)]
            + r[1] * h[(7, 4)]
            + r[2] * h[(8, 4)]
            + r[6] * h[(0, 4)]
            + r[7] * h[(1, 4)]
            + r[8] * h[(2, 4)];

        h[(0, 5)] = r[6] - dot_j6q1 * h[(0, 0)] - dot_j6q4 * h[(0, 3)] - dot_j6q5 * h[(0, 4)];
        h[(1, 5)] = r[7] - dot_j6q1 * h[(1, 0)] - dot_j6q4 * h[(1, 3)] - dot_j6q5 * h[(1, 4)];
        h[(2, 5)] = r[8] - dot_j6q1 * h[(2, 0)] - dot_j6q4 * h[(2, 3)] - dot_j6q5 * h[(2, 4)];

        h[(3, 5)] = -dot_j6q5 * h[(3, 4)] - dot_j6q4 * h[(3, 3)];
        h[(4, 5)] = -dot_j6q5 * h[(4, 4)] - dot_j6q4 * h[(4, 3)];
        h[(5, 5)] = -dot_j6q5 * h[(5, 4)] - dot_j6q4 * h[(5, 3)];

        h[(6, 5)] = r[0] - dot_j6q3 * h[(6, 2)] - dot_j6q5 * h[(6, 4)];
        h[(7, 5)] = r[1] - dot_j6q3 * h[(7, 2)] - dot_j6q5 * h[(7, 4)];
        h[(8, 5)] = r[2] - dot_j6q3 * h[(8, 2)] - dot_j6q5 * h[(8, 4)];

        let _ = h.column_mut(5).normalize();

        k[(5, 0)] = r[6] * h[(0, 0)] + r[7] * h[(1, 0)] + r[8] * h[(2, 0)];
        k[(5, 1)] = 0.0;
        k[(5, 2)] = r[0] * h[(6, 2)] + r[1] * h[(7, 2)] + r[2] * h[(8, 2)];
        k[(5, 3)] = r[6] * h[(0, 3)] + r[7] * h[(1, 3)] + r[8] * h[(2, 3)];
        k[(5, 4)] = r[6] * h[(0, 4)]
            + r[7] * h[(1, 4)]
            + r[8] * h[(2, 4)]
            + r[0] * h[(6, 4)]
            + r[1] * h[(7, 4)]
            + r[2] * h[(8, 4)];
        k[(5, 5)] = r[6] * h[(0, 5)]
            + r[7] * h[(1, 5)]
            + r[8] * h[(2, 5)]
            + r[0] * h[(6, 5)]
            + r[1] * h[(7, 5)]
            + r[2] * h[(8, 5)];

        // Great! Now H is an orthogonalized, sparse basis of the Jacobian row space and K is filled.
        //
        // Now get a projector onto the null space of H:
        let pn = na::SMatrix::<f64, 9, 9>::identity() - (*h * h.transpose());

        // Now we need to pick 3 columns of P with non-zero norm (> 0.3) and some angle between them (> 0.3).
        //
        // Find the 3 columns of Pn with largest norms
        let mut index1: i32 = -1;
        let mut index2: i32 = -1;
        let mut index3: i32 = -1;

        let mut max_norm1 = f64::MIN_POSITIVE;
        let mut min_dot12 = f64::MAX;
        let mut min_dot1323 = f64::MAX;

        let mut col_norms: [f64; 9] = [0.0; 9];

        for i in 0..9 {
            col_norms[i] = pn.column(i).norm();
            if col_norms[i] >= Self::NORM_THRESHOLD {
                if max_norm1 < col_norms[i] {
                    max_norm1 = col_norms[i];
                    index1 = i as i32;
                }
            }
        }

        let v1 = pn.column(index1 as usize);
        n.column_mut(0).copy_from(&(v1 * (1.0 / max_norm1)));
        col_norms[index1 as usize] = -1.0; // Mark to avoid reuse

        // Second pass: find second column with smallest cosine angle to v1
        for i in 0..9 {
            if col_norms[i] >= Self::NORM_THRESHOLD {
                let cos_v1_x_col = (pn.column(i).dot(&v1) / col_norms[i]).abs();
                if cos_v1_x_col <= min_dot12 {
                    index2 = i as i32;
                    min_dot12 = cos_v1_x_col;
                }
            }
        }
        let v2 = pn.column(index2 as usize);
        let dot_v2_n0 = v2.dot(&n.column(0));
        let v2_ortho = v2 - dot_v2_n0 * n.column(0);
        n.column_mut(1).copy_from(&v2_ortho);
        n.column_mut(1).normalize_mut();
        col_norms[index2 as usize] = -1.0; // Mark to avoid reuse

        // Third pass: find third column minimizing sum of cosine angles to v1 and v2
        for i in 0..9 {
            if col_norms[i] >= Self::NORM_THRESHOLD {
                let inv_norm = 1.0 / col_norms[i];
                let cos_v1_x_col = (pn.column(i).dot(&v1) * inv_norm).abs();
                let cos_v2_x_col = (pn.column(i).dot(&v2) * inv_norm).abs();
                if (cos_v1_x_col + cos_v2_x_col) <= min_dot1323 {
                    index3 = i as i32;
                    min_dot1323 = cos_v1_x_col + cos_v2_x_col;
                }
            }
        }

        // Orthogonalize v3 against v1 and v2
        // Third column
        let v3 = pn.column(index3 as usize);
        // Compute dot products and intermediate vector before mutable borrow
        let dot_v3_n0 = v3.dot(&n.column(0));
        let dot_v3_n1 = v3.dot(&n.column(1));
        let v3_ortho = v3 - (dot_v3_n1 * n.column(1)) - (dot_v3_n0 * n.column(0));
        n.column_mut(2).copy_from(&v3_ortho);

        n.column_mut(2).normalize_mut();
    }

    #[inline(always)]
    fn axb_solve_ldlt_3x3(
        a: &na::SMatrix<f64, 3, 3>,
        b: &na::SVector<f64, 3>,
        x: &mut na::SVector<f64, 3>,
    ) -> Result<(), &'static str> {
        let mut l = na::SMatrix::<f64, 3, 3>::zeros();

        // D is stored in L's diagonal, i.e., l[(0, 0)], l[(1, 1)], l[(2, 2)]
        // its elements should be positive
        l[(0, 0)] = a[(0, 0)];
        if l[(0, 0)] < 1e-10 {
            return Err("Matrix is not positive definite");
        }
        let inv_l00 = 1.0 / l[(0, 0)];
        l[(1, 0)] = a[(1, 0)] * inv_l00;
        l[(2, 0)] = a[(2, 0)] * inv_l00;

        l[(1, 1)] = a[(1, 1)] - l[(1, 0)] * l[(1, 0)] * l[(0, 0)];
        if l[(1, 1)] < 1e-10 {
            return Err("Matrix is not positive definite");
        }
        let inv_l11 = 1.0 / l[(1, 1)];
        l[(2, 1)] = (a[(2, 1)] - l[(2, 0)] * l[(1, 0)] * l[(0, 0)]) * inv_l11;

        l[(2, 2)] =
            a[(2, 2)] - l[(2, 0)] * l[(2, 0)] * l[(0, 0)] - l[(2, 1)] * l[(2, 1)] * l[(1, 1)];
        if l[(2, 2)] < 1e-10 {
            return Err("Matrix is not positive definite");
        }

        // Forward solve L*x = b
        x[0] = b[0];
        x[1] = b[1] - l[(1, 0)] * x[0];
        x[2] = b[2] - l[(2, 0)] * x[0] - l[(2, 1)] * x[1];

        // Backward solve D*L'*x = y
        x[2] /= l[(2, 2)];
        x[1] = x[1] / l[(1, 1)] - l[(2, 1)] * x[2];
        x[0] = x[0] / l[(0, 0)] - l[(1, 0)] * x[1] - l[(2, 0)] * x[2];

        Ok(())
    }
}
