use auto_aim_rust::rbt_base::rbt_math::eskf;
use nalgebra as na;

fn main() {
    let init_p = na::Matrix3::<f64>::zeros();
    let s = na::Matrix3::zeros();
    let m = na::Matrix3::zeros();
    let dt = tokio::time::Duration::from_secs(10);
    let dt_as_millis_f64 = dt.as_micros() as f64 / 1000.0;
    let eskf = eskf::Eskf::new(init_p, s, m, dt_as_millis_f64);
}
