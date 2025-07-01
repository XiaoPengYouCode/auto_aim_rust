pub mod sqpnp_def;
pub mod sqpnp_impl;

use tracing::error;
use sqpnp_def::OmegaNullspaceMethod;
use sqpnp_def::SQPSolution;
use sqpnp_def::SolverParameters;
use sqpnp_def::{Point, Projection};

pub struct PnpSolver {
    pub projections: Vec<Projection>,
    pub points: Vec<Point>,
    pub weights: Vec<f64>,
    pub parameters: SolverParameters,

    pub omega: na::SMatrix<f64, 9, 9>,
    pub s: na::SVector<f64, 9>,
    pub u: na::SMatrix<f64, 9, 9>,
    pub p: na::SMatrix<f64, 3, 9>,
    pub point_mean: na::SVector<f64, 3>, // For the positive depth test

    pub num_null_vectors: i32,

    pub solutions: Vec<SQPSolution>,
    pub num_solutions: i32,
}

impl Default for PnpSolver {
    fn default() -> Self {
        Self {
            projections: Vec::new(),
            points: Vec::new(),
            weights: Vec::new(),
            parameters: SolverParameters::default(),
            omega: na::SMatrix::<f64, 9, 9>::zeros(),
            s: na::SVector::<f64, 9>::zeros(),
            u: na::SMatrix::<f64, 9, 9>::zeros(),
            p: na::SMatrix::<f64, 3, 9>::zeros(),
            point_mean: na::SVector::<f64, 3>::zeros(),
            num_null_vectors: 0,
            solutions: Vec::with_capacity(18),
            num_solutions: 0,
        }
    }
}

impl PnpSolver {
    /// 注意这里的函数参数少了 parameters
    /// 在PnpSolver::default()中会调用SolverParameters::default()
    pub fn new(
        points_3d: Vec<Point>,
        projections: Vec<Projection>,
        weights: Option<&[f64]>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut solver = PnpSolver::default();

        let n = points_3d.len();
        let n_projections = projections.len();

        if n != n_projections {
            error!("Number of projections does not match number of points: points_num {} != projection_num {}", n, projections.len());
            return Err("Number of projections does not match number of points".into());
        }

        if n != projections.len() || n < 3 {
            return Err("Number of points and projections must match and be at least 3".into());
        }

        solver.weights = if let Some(weights) = weights {
            if n != weights.len() {
                return Err("Number of weights must match the number of points".into());
            }
            weights.to_vec()
        } else {
            vec![1.0; n] // 如果没有给定权重，则默认每个点的权重为1.0
        };

        solver.points = Vec::with_capacity(n);
        solver.projections = Vec::with_capacity(n);
        solver.num_null_vectors = -1;
        solver.omega = na::SMatrix::<f64, 9, 9>::zeros();

        let mut sum_wx = 0.0;
        let mut sum_wy = 0.0;
        let mut sum_wx2_plus_wy2 = 0.0;
        let mut sum_w = 0.0;

        let mut sum_w_x = 0.0;
        let mut sum_w_y = 0.0;
        let mut sum_w_z = 0.0;

        let mut qa = na::SMatrix::<f64, 3, 9>::zeros();

        // let mut frame_count = 0;
        for (i, (point, projection)) in points_3d
            .into_iter()
            .zip(projections.into_iter())
            .enumerate()
        {
            let w = solver.weights[i];

            solver.points.push(point);
            solver.projections.push(projection);

            if w == 0.0 {
                continue; // Skip zero-weight points
            }

            let proj = solver.projections[i].vector;
            let wx = proj[0] * w;
            let wy = proj[1] * w;
            let wsq_norm_m = w * proj.norm_squared();

            sum_wx += wx;
            sum_wy += wy;
            sum_wx2_plus_wy2 += wsq_norm_m;
            sum_w += w;

            let pt = solver.points[i].vector;
            let x = pt[0];
            let y = pt[1];
            let z = pt[2];

            let w_x = w * x;
            let w_y = w * y;
            let w_z = w * z;

            sum_w_x += w_x;
            sum_w_y += w_y;
            sum_w_z += w_z;

            let x2 = x * x;
            let xy = x * y;
            let xz = x * z;
            let y2 = y * y;
            let yz = y * z;
            let z2 = z * z;

            solver.omega[(0, 0)] += w * x2;
            solver.omega[(0, 1)] += w * xy;
            solver.omega[(0, 2)] += w * xz;
            solver.omega[(1, 1)] += w * y2;
            solver.omega[(1, 2)] += w * yz;
            solver.omega[(2, 2)] += w * z2;

            solver.omega[(0, 6)] -= wx * x2;
            solver.omega[(0, 7)] -= wx * xy;
            solver.omega[(0, 8)] -= wx * xz;
            solver.omega[(1, 7)] -= wx * y2;
            solver.omega[(1, 8)] -= wx * yz;
            solver.omega[(2, 8)] -= wx * z2;

            solver.omega[(3, 6)] -= wy * x2;
            solver.omega[(3, 7)] -= wy * xy;
            solver.omega[(3, 8)] -= wy * xz;
            solver.omega[(4, 7)] -= wy * y2;
            solver.omega[(4, 8)] -= wy * yz;
            solver.omega[(5, 8)] -= wy * z2;

            solver.omega[(6, 6)] += wsq_norm_m * x2;
            solver.omega[(6, 7)] += wsq_norm_m * xy;
            solver.omega[(6, 8)] += wsq_norm_m * xz;
            solver.omega[(7, 7)] += wsq_norm_m * y2;
            solver.omega[(7, 8)] += wsq_norm_m * yz;
            solver.omega[(8, 8)] += wsq_norm_m * z2;

            qa[(0, 0)] += w_x;
            qa[(0, 1)] += w_y;
            qa[(0, 2)] += w_z;

            qa[(0, 6)] -= wx * x;
            qa[(0, 7)] -= wx * y;
            qa[(0, 8)] -= wx * z;
            qa[(1, 6)] -= wy * x;
            qa[(1, 7)] -= wy * y;
            qa[(1, 8)] -= wy * z;

            qa[(2, 6)] += wsq_norm_m * x;
            qa[(2, 7)] += wsq_norm_m * y;
            qa[(2, 8)] += wsq_norm_m * z;

            // println!("frame {}: omega = {}", frame_count, solver.omega);

            // frame_count += 1;
        }

        // Complete QA
        qa[(1, 3)] = qa[(0, 0)];
        qa[(1, 4)] = qa[(0, 1)];
        qa[(1, 5)] = qa[(0, 2)];
        qa[(2, 0)] = qa[(0, 6)];
        qa[(2, 1)] = qa[(0, 7)];
        qa[(2, 2)] = qa[(0, 8)];
        qa[(2, 3)] = qa[(1, 6)];
        qa[(2, 4)] = qa[(1, 7)];
        qa[(2, 5)] = qa[(1, 8)];

        // Fill-in lower triangles of off-diagonal blocks (0:2, 6:8), (3:5, 6:8) and (6:8, 6:8)
        solver.omega[(1, 6)] = solver.omega[(0, 7)];
        solver.omega[(2, 6)] = solver.omega[(0, 8)];
        solver.omega[(2, 7)] = solver.omega[(1, 8)];
        solver.omega[(4, 6)] = solver.omega[(3, 7)];
        solver.omega[(5, 6)] = solver.omega[(3, 8)];
        solver.omega[(5, 7)] = solver.omega[(4, 8)];
        solver.omega[(7, 6)] = solver.omega[(6, 7)];
        solver.omega[(8, 6)] = solver.omega[(6, 8)];
        solver.omega[(8, 7)] = solver.omega[(7, 8)];

        // Fill-in upper triangle of block (3:5, 3:5)
        solver.omega[(3, 3)] = solver.omega[(0, 0)];
        solver.omega[(3, 4)] = solver.omega[(0, 1)];
        solver.omega[(3, 5)] = solver.omega[(0, 2)];
        solver.omega[(4, 4)] = solver.omega[(1, 1)];
        solver.omega[(4, 5)] = solver.omega[(1, 2)];
        solver.omega[(5, 5)] = solver.omega[(2, 2)];

        // Fill lower triangle of Omega; elements (7, 6), (8, 6) & (8, 7) have already been assigned above
        solver.omega[(1, 0)] = solver.omega[(0, 1)];
        solver.omega[(2, 0)] = solver.omega[(0, 2)];
        solver.omega[(2, 1)] = solver.omega[(1, 2)];
        solver.omega[(3, 0)] = solver.omega[(0, 3)];
        solver.omega[(3, 1)] = solver.omega[(1, 3)];
        solver.omega[(3, 2)] = solver.omega[(2, 3)];
        solver.omega[(4, 0)] = solver.omega[(0, 4)];
        solver.omega[(4, 1)] = solver.omega[(1, 4)];
        solver.omega[(4, 2)] = solver.omega[(2, 4)];
        solver.omega[(4, 3)] = solver.omega[(3, 4)];
        solver.omega[(5, 0)] = solver.omega[(0, 5)];
        solver.omega[(5, 1)] = solver.omega[(1, 5)];
        solver.omega[(5, 2)] = solver.omega[(2, 5)];
        solver.omega[(5, 3)] = solver.omega[(3, 5)];
        solver.omega[(5, 4)] = solver.omega[(4, 5)];
        solver.omega[(6, 0)] = solver.omega[(0, 6)];
        solver.omega[(6, 1)] = solver.omega[(1, 6)];
        solver.omega[(6, 2)] = solver.omega[(2, 6)];
        solver.omega[(6, 3)] = solver.omega[(3, 6)];
        solver.omega[(6, 4)] = solver.omega[(4, 6)];
        solver.omega[(6, 5)] = solver.omega[(5, 6)];
        solver.omega[(7, 0)] = solver.omega[(0, 7)];
        solver.omega[(7, 1)] = solver.omega[(1, 7)];
        solver.omega[(7, 2)] = solver.omega[(2, 7)];
        solver.omega[(7, 3)] = solver.omega[(3, 7)];
        solver.omega[(7, 4)] = solver.omega[(4, 7)];
        solver.omega[(7, 5)] = solver.omega[(5, 7)];
        solver.omega[(8, 0)] = solver.omega[(0, 8)];
        solver.omega[(8, 1)] = solver.omega[(1, 8)];
        solver.omega[(8, 2)] = solver.omega[(2, 8)];
        solver.omega[(8, 3)] = solver.omega[(3, 8)];
        solver.omega[(8, 4)] = solver.omega[(4, 8)];
        solver.omega[(8, 5)] = solver.omega[(5, 8)];

        let mut q = na::Matrix3::zeros();
        q[(0, 0)] = sum_w;
        q[(0, 1)] = 0.0;
        q[(0, 2)] = -sum_wx;
        q[(1, 0)] = 0.0;
        q[(1, 1)] = sum_w;
        q[(1, 2)] = -sum_wy;
        q[(2, 0)] = -sum_wx;
        q[(2, 1)] = -sum_wy;
        q[(2, 2)] = sum_wx2_plus_wy2;

        let q_inv = q.try_inverse().ok_or("Matrix inversion failed")?;
        solver.p = -q_inv * qa; // t = p * r
        solver.omega += qa.transpose() * solver.p; // final omega

        // find candidate solution
        let (u, s) = match solver.parameters.omega_nullspace_method {
            OmegaNullspaceMethod::RRQR | OmegaNullspaceMethod::CPRRQR => {
                // nalgebra没有全主元QR分解的实现，使用ColPivQR代替
                let rrqr = na::ColPivQR::new(solver.omega);
                (rrqr.q().clone_owned(), rrqr.r().diagonal().to_owned())
            }
            OmegaNullspaceMethod::SVD => {
                let svd = solver.omega.svd(true, false); // 只需要计算完整的U，不需要计算V
                (svd.u.unwrap(), svd.singular_values)
            }
        };
        solver.u = u;
        solver.s = s;

        while (7 - solver.num_null_vectors) >= 0
            && solver.s[(7 - solver.num_null_vectors) as usize] < solver.parameters.rank_tolerance
        {
            solver.num_null_vectors += 1;
        }

        solver.num_null_vectors += 1;
        if solver.num_null_vectors > 6 {
            return Err("Number of null vectors exceeds 6".into());
        }

        let inv_sum_w = 1.0 / sum_w;
        solver.point_mean = na::Vector3::new(sum_w_x, sum_w_y, sum_w_z) * inv_sum_w;

        // solver.nearest_rotation_matrix = match solver.parameters.nearest_rotation_method {
        //     NearestRotationMethod::FOAM => Self::nearest_rotation_matrix_foam,
        //     NearestRotationMethod::SVD => Self::nearest_rotation_matrix_svd,
        // };

        Ok(solver)
    }

    pub fn omega(&self) -> &na::SMatrix<f64, 9, 9> {
        &self.omega
    }

    pub fn eigen_vectors(&self) -> &na::SMatrix<f64, 9, 9> {
        &self.u
    }

    pub fn eigen_values(&self) -> &na::SVector<f64, 9> {
        &self.s
    }

    pub fn null_space_dimension(&self) -> i32 {
        self.num_null_vectors
    }

    pub fn number_of_solutions(&self) -> i32 {
        self.num_solutions
    }

    pub fn solution_ptr(&mut self, idx: usize) -> Option<&mut SQPSolution> {
        if idx >= self.solutions.len() {
            return None;
        }
        Some(&mut self.solutions[idx])
    }

    pub fn weights(&self) -> &[f64] {
        &self.weights
    }
}
