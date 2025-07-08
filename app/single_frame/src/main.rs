use tracing::info;
use nalgebra as na;

use lib::rbt_base::rbt_cam_model::adjust_coordinates;
// use lib::rbt_base::rbt_geometry::rbt_rigid::RbtRigidPose;
use lib::rbt_base::rbt_math::sqpnp::PnpSolver;
use lib::rbt_err::RbtResult;
use lib::rbt_infra::rbt_cfg::RbtCfg;
use lib::rbt_mod::rbt_detector::pipeline;
use lib::rbt_infra::rbt_log::logger_init;
use lib::rbt_mod::rbt_armor::SMALL_ARMOR_POINT3;

#[tokio::main]
async fn main() -> RbtResult<()> {
    // 初始化流程
    let cfg = RbtCfg::from_toml()?;
    // todo!(这里直接使用了 lazy_static 中读取的配置，还没有替换成最新的 rbt_cfg)
    let _logger_guard = logger_init().await?;

    let k = cfg.cam_cfg.cam_k();

    // FromIterator trait
    // 输入：一个迭代器，其中每个元素都是 Result<T, E>（即 Iterator<Item = Result<T, E>>）
    // - 遍历所有元素, 如果所有元素都是 Ok(T)，则收集所有 T 值到集合 V 中，返回 Ok(V)
    // - 如果遇到任何一个 Err(E)，立即停止处理并返回这个 Err(E)
    // 输出：一个 Result<V, E>，其中 V 是包含所有 T 的集合
    let pnps = pipeline(&cfg.detector_cfg)?
        .into_iter()
        .map(|armor| {
            let mut armor_point = armor.corner_points_na();
            adjust_coordinates(&k, &mut armor_point);
            let mut pnp = PnpSolver::new(SMALL_ARMOR_POINT3.to_vec(), armor_point, None)?;
            pnp.solve()?;
            Ok(pnp)
        }).collect::<RbtResult<Vec<PnpSolver>>>()?;

    info!("pnps number: {}", pnps.len());
    for pnp in pnps.iter() {
        // let armor_point3d_in_cam = na::Point3::from(pnp.solutions[0].t);
        // let armor_rotation_matrix_in_cam = na::Matrix3::<f64>::from(pnp.solutions[1].r_hat);
        // let armor_rotation_in_cam = na::UnitQuaternion::from_matrix(&armor_rotation_matrix_in_cam);
        info!("armor_translation: {}", pnp.solutions[0].t);
    }


    // angle_solver
    // Armor armor(armor_jun);
    //
    // armor.rect = cvex::scaleRect(boundingRect(armor.vertex), Vec2f(1.5, 1.5));
    //
    // armor.dis = calculateDistance(armor, armor.type);
    //
    // armor.armorPosition = calculateGimblePoint(armor.center, armor.dis) / 1000.0; // m
    //
    // armor.hitPosRight = calculateGimblePoint(armor.hitPointR, armor.dis) / 1000.0;
    // armor.hitPosLeft = calculateGimblePoint(armor.hitPointL, armor.dis) / 1000.0;
    // armor.hitPosUp = calculateGimblePoint(armor.hitPointU, armor.dis) / 1000.0;
    // armor.hitPosDown = calculateGimblePoint(armor.hitPointD, armor.dis) / 1000.0;
    //
    // armor.yaw_absolute = calculateYaw(armor.vertex, armor.type);
    //
    // armor.yaw = armor.yaw_absolute - _gimbal_pose.yaw * D2R;
    //
    // //        cout << "calcuYaw: " << calculateYaw(armor.vertex, armor.type) << endl;
    // //        cout << "gimbal_yaw: " << _gimbal_pose.yaw << endl;
    //
    // armor.distanceToImageCenter = calculateDistanceToCenter(armor.center);
    //
    // _armors.push_back(make_shared<Armor>(armor));


    // estimator
    // _angleSolverPtr = &angle_solver;
    // _aim_mode = aim_mode;

    /*
     * 卡尔曼滤波器可以通过观测值更新目标的状态
     * 观测值：装甲板的三维坐标和相对与相机系的角度
     * 目标的状态：包括中心的xy坐标和速度，装甲板的xy坐标，旋转速度和旋转半径等
     * 更新：可以通过以前的观测，预测出当前的状态，然后比较观测到的当前实际位置和预测的位置的误差，来修正卡尔曼滤波器
     */

    // bool init_flag = false;
    // if (tracker_state == State::LOST)
    // {
    //     init(armors); // 初始化过程
    //     init_flag = true;
    // }
    // else
    // {
    //     update(armors); // 上面提到的更新过程
    // }
    //
    // if ((tracker_state == State::LOST) || init_flag)
    // {
    //     // lost状态或者刚初始化完没有_trackedArmor
    //     // 因此要通过 _detectedFlag 提示一下
    //     // 以免show_result中使用空指针
    //     _detectedFlag = false;
    // }
    // else
    // {
    //     _detectedFlag = true;
    //     getHitPoint(); //计算瞄点
    //     _lastTime = _timeStamp;
    // }

    if pnps.len() == 1 {
        // handle_single_armor();
    } else if pnps.len() == 2 {
        // handle_double_armor();
    }

    info!("App finished successfully");

    Ok(())
}
