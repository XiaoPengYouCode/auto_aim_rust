const DEFAULT_RANK_TOLERANCE: f64 = 1e-7;
const DEFAULT_SQP_SQUARED_TOLERANCE: f64 = 1e-10;
const DEFAULT_SQP_DET_THRESHOLD: f64 = 1.001;
const DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD: f64 = 1e-8;
const DEFAULT_EQUAL_VECTORS_SQUARED_DIFF: f64 = 1e-10;
const DEFAULT_EQUAL_SQUARED_ERRORS_DIFF: f64 = 1e-6;
const DEFAULT_POINT_VARIANCE_THRESHOLD: f64 = 1e-5;

pub enum OmegaNullspaceMethod {
    RRQR,
    CPRRQR,
    SVD,
}

impl Default for OmegaNullspaceMethod {
    fn default() -> Self {
        Self::SVD
    }
}

pub enum NearestRotationMethod {
    FOAM,
    SVD,
}

impl Default for NearestRotationMethod {
    fn default() -> Self {
        Self::SVD
    }
}

pub struct SolverParameters {
    pub rank_tolerance: f64,
    pub sqp_squared_tolerance: f64,
    pub sqp_det_threshold: f64,
    pub sqp_max_iteration: usize,
    pub omega_nullspace_method: OmegaNullspaceMethod,
    pub nearest_rotation_method: NearestRotationMethod,
    pub orthogonality_squared_error_threshold: f64,
    pub equal_vectors_squared_diff: f64,
    pub equal_squared_errors_diff: f64,
    pub point_variance_threshold: f64,
}

impl Default for SolverParameters {
    fn default() -> Self {
        SolverParameters {
            rank_tolerance: DEFAULT_RANK_TOLERANCE,
            sqp_squared_tolerance: DEFAULT_SQP_SQUARED_TOLERANCE,
            sqp_det_threshold: DEFAULT_SQP_DET_THRESHOLD,
            sqp_max_iteration: 15, // 默认值
            omega_nullspace_method: OmegaNullspaceMethod::default(),
            nearest_rotation_method: NearestRotationMethod::default(),
            orthogonality_squared_error_threshold: DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD,
            equal_vectors_squared_diff: DEFAULT_EQUAL_VECTORS_SQUARED_DIFF,
            equal_squared_errors_diff: DEFAULT_EQUAL_SQUARED_ERRORS_DIFF,
            point_variance_threshold: DEFAULT_POINT_VARIANCE_THRESHOLD,
        }
    }
}

pub struct SolverParametersBuilder {
    rank_tolerance: f64,
    sqp_squared_tolerance: f64,
    sqp_det_threshold: f64,
    sqp_max_iteration: usize,
    omega_nullspace_method: OmegaNullspaceMethod,
    nearest_rotation_method: NearestRotationMethod,
    orthogonality_squared_error_threshold: f64,
    equal_vectors_squared_diff: f64,
    equal_squared_errors_diff: f64,
    point_variance_threshold: f64,
}

impl SolverParametersBuilder {
    pub fn new() -> Self {
        SolverParametersBuilder {
            rank_tolerance: DEFAULT_RANK_TOLERANCE,
            sqp_squared_tolerance: DEFAULT_SQP_SQUARED_TOLERANCE,
            sqp_det_threshold: DEFAULT_SQP_DET_THRESHOLD,
            sqp_max_iteration: 100,                                  // 默认值
            omega_nullspace_method: OmegaNullspaceMethod::default(), // 假设有一个默认值
            nearest_rotation_method: NearestRotationMethod::default(), // 假设有一个默认值
            orthogonality_squared_error_threshold: DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD,
            equal_vectors_squared_diff: DEFAULT_EQUAL_VECTORS_SQUARED_DIFF,
            equal_squared_errors_diff: DEFAULT_EQUAL_SQUARED_ERRORS_DIFF,
            point_variance_threshold: DEFAULT_POINT_VARIANCE_THRESHOLD,
        }
    }

    pub fn rank_tolerance(mut self, value: f64) -> Self {
        self.rank_tolerance = value;
        self
    }

    pub fn sqp_squared_tolerance(mut self, value: f64) -> Self {
        self.sqp_squared_tolerance = value;
        self
    }

    pub fn sqp_det_threshold(mut self, value: f64) -> Self {
        self.sqp_det_threshold = value;
        self
    }

    pub fn sqp_max_iteration(mut self, value: usize) -> Self {
        self.sqp_max_iteration = value;
        self
    }

    pub fn omega_nullspace_method(mut self, value: OmegaNullspaceMethod) -> Self {
        self.omega_nullspace_method = value;
        self
    }

    pub fn nearest_rotation_method(mut self, value: NearestRotationMethod) -> Self {
        self.nearest_rotation_method = value;
        self
    }

    pub fn orthogonality_squared_error_threshold(mut self, value: f64) -> Self {
        self.orthogonality_squared_error_threshold = value;
        self
    }

    pub fn equal_vectors_squared_diff(mut self, value: f64) -> Self {
        self.equal_vectors_squared_diff = value;
        self
    }

    pub fn equal_squared_errors_diff(mut self, value: f64) -> Self {
        self.equal_squared_errors_diff = value;
        self
    }

    pub fn point_variance_threshold(mut self, value: f64) -> Self {
        self.point_variance_threshold = value;
        self
    }

    pub fn build(self) -> SolverParameters {
        SolverParameters {
            rank_tolerance: self.rank_tolerance,
            sqp_squared_tolerance: self.sqp_squared_tolerance,
            sqp_det_threshold: self.sqp_det_threshold,
            sqp_max_iteration: self.sqp_max_iteration,
            omega_nullspace_method: self.omega_nullspace_method,
            nearest_rotation_method: self.nearest_rotation_method,
            orthogonality_squared_error_threshold: self.orthogonality_squared_error_threshold,
            equal_vectors_squared_diff: self.equal_vectors_squared_diff,
            equal_squared_errors_diff: self.equal_squared_errors_diff,
            point_variance_threshold: self.point_variance_threshold,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Projection {
    pub vector: na::SVector<f64, 2>,
}

impl Projection {
    pub fn new(x: f64, y: f64) -> Self {
        Projection {
            vector: na::SVector::<f64, 2>::new(x, y),
        }
    }
}

impl Default for Projection {
    fn default() -> Self {
        Projection {
            vector: na::SVector::<f64, 2>::zeros(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Point {
    pub vector: na::SVector<f64, 3>,
}

impl Point {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Point {
            vector: na::SVector::<f64, 3>::new(x, y, z),
        }
    }
}

impl Default for Point {
    fn default() -> Self {
        Point {
            vector: na::SVector::<f64, 3>::zeros(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SQPSolution {
    pub r: na::SVector<f64, 9>,     // Actual matrix upon convergence
    pub r_hat: na::SVector<f64, 9>, // "Clean" (nearest) rotation matrix
    pub t: na::SVector<f64, 3>,     // Translation vector
    pub num_iterations: usize,      // Number of iterations
    pub sq_error: f64,              // Squared error
}

impl Default for SQPSolution {
    fn default() -> Self {
        SQPSolution {
            r: na::SVector::<f64, 9>::zeros(),
            r_hat: na::SVector::<f64, 9>::zeros(),
            t: na::SVector::<f64, 3>::zeros(),
            num_iterations: 0,
            sq_error: f64::MAX,
        }
    }
}
