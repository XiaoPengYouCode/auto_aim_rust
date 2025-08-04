// outpost 3-Armor model

use crate::rbt_base::rbt_math::eskf::StrategyESKFDynamic;
use crate::rbt_mod::rbt_estimator::enemy_model::EnemyModel;

/// 3 块装甲板为前哨站特化版本
/// some variable no need to estimator
/// three armor model
impl StrategyESKFDynamic<6, 3> for EnemyModel<3> {
    type Input = f64;
    type NominalState = f64;
    type Measurement = f64;
    type Strategy = ();

    fn update_nominal_state(
        &self,
        nominal_state: &mut Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: Self::Strategy
    ) {
    }

    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: Self::Strategy
    ) -> na::SMatrix<f64, 6, 6> {
        na::SMatrix::<f64, 6, 6>::zeros()
    }

    fn measurement_matrix_h(
        &self,
        nominal_state: &Self::NominalState,
        strategy: &Self::Strategy
    ) -> na::SMatrix<f64, 3, 6> {
        na::SMatrix::<f64, 3, 6>::zeros()
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
        strategy: Self::Strategy
    ) -> na::SVector<f64, 3> {
        na::SVector::<f64, 3>::zeros()
    }

    fn inject_error(
        &self,
        nominal_state: &mut Self::NominalState,
        error_estimate: &na::SVector<f64, 6>,
        strategy: Self::Strategy
    ) {
        *nominal_state += error_estimate[0]
    }
}
