// outpost model

use super::{EnemyModel, EskfDynamic};

/// 3 块装甲板为前哨站特化版本
/// some variable no need to estimator
/// three armor model
impl EskfDynamic<6, 3> for EnemyModel<3> {
    type Input = f64;
    type NominalState = f64;
    type Measurement = f64;

    fn update_nominal_state(
        &self,
        nominal_state: &mut Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) {
    }

    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) -> na::SMatrix<f64, 6, 6> {
        na::SMatrix::<f64, 6, 6>::zeros()
    }

    fn measurement_matrix_h(&self, nominal_state: &Self::NominalState) -> na::SMatrix<f64, 3, 6> {
        na::SMatrix::<f64, 3, 6>::zeros()
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
    ) -> na::SVector<f64, 3> {
        na::SVector::<f64, 3>::zeros()
    }

    fn inject_error(
        &self,
        nominal_state: Self::NominalState,
        error_estimate: &na::SVector<f64, 6>,
    ) -> Self::NominalState {
        0.0_f64
    }
}
