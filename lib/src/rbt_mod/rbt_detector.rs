use image::{DynamicImage, ImageReader};
use log::info;
use ndarray as nd;
use ort::{
    inputs,
    session::{Session, SessionOutputs},
    value::TensorRef,
};
use std::collections::HashMap;

use crate::rbt_infra::rbt_cfg;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_infra::rbt_global::GENERIC_RBT_CFG;
use crate::rbt_infra::rbt_ort_ep::configure_session_builder;
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
use crate::rbt_mod::rbt_detector::rbt_frame::{
    ARMOR_INPUT_HEIGHT, ARMOR_INPUT_WIDTH, ARMOR_OUTPUT_COLS, ARMOR_OUTPUT_ROWS,
};
pub use crate::rbt_mod::rbt_detector::rbt_yolo::BBox;
use crate::rbt_mod::rbt_detector::rbt_yolo::{
    ArmorYoloPostprocessCfg, LetterboxTransform, decode_armor_output, preprocess_letterbox_f16,
};
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyId;

pub mod rbt_frame;
pub mod rbt_yolo;

pub struct ArmorDetector {
    img: DynamicImage,
    input: nd::Array4<half::f16>,
    letterbox: LetterboxTransform,
}

impl ArmorDetector {
    fn init(_cfg: &rbt_cfg::DetectorCfg) -> RbtResult<ArmorDetector> {
        let img = ImageReader::open("./imgs/test_resize.jpg")?
            .decode()
            .map_err(|err| RbtError::StringError(format!("failed to decode test image: {err}")))?;

        Ok(Self {
            img,
            input: nd::Array4::zeros((1, 3, ARMOR_INPUT_HEIGHT, ARMOR_INPUT_WIDTH)),
            letterbox: LetterboxTransform::default(),
        })
    }

    /// 前处理：等比例 resize、左上角 letterbox、RGB CHW、归一化到 FP16。
    fn pre_process(&mut self) {
        self.letterbox = preprocess_letterbox_f16(self.input.view_mut(), &self.img);
    }

    pub fn post_process(
        &self,
        outputs: &SessionOutputs,
        cfg: &rbt_cfg::DetectorCfg,
    ) -> RbtResult<HashMap<EnemyId, Vec<DetectedArmor>>> {
        let output = outputs
            .get("output")
            .ok_or_else(|| RbtError::StringError("armor model output `output` not found".into()))?
            .try_extract_array::<f32>()?;
        let output = output
            .as_standard_layout()
            .to_owned()
            .into_shape_with_order((ARMOR_OUTPUT_ROWS, ARMOR_OUTPUT_COLS))
            .map_err(|err| {
                RbtError::StringError(format!(
                    "failed to reshape armor output to [{ARMOR_OUTPUT_ROWS},{ARMOR_OUTPUT_COLS}]: {err}"
                ))
            })?;
        let self_fraction = GENERIC_RBT_CFG
            .read()
            .ok()
            .and_then(|cfg| cfg.game_cfg.self_fraction());
        let post_cfg = ArmorYoloPostprocessCfg::from_armor_cfg(&cfg.armor, self_fraction);

        Ok(decode_armor_output(
            &output.view(),
            self.letterbox,
            &post_cfg,
        ))
    }
}

/// 不需要使用cudarc主动将数据拷贝，这个过程ort-rs会自己完成
/// 但是要通过log察觉到是否在节点间新增了memcpy操作，发现问题并解决，这会严重影响性能
/// 目前尚不支持动态量化，所以使用动态量化的模型会引入很多memcpy操作
/// 直观速度对比
/// CPU型号:12500H
/// iGPU型号:Intel Iris Xe Graphics
/// GPU型号:RTX 2050
/// CPU: FP16 26ms
/// CPU+OPENVINO: FP16 19ms
/// iGPU + OPENVINO + oneAPI + oneDNN: FP16 10ms
/// CUDA 12.6: FP16 5ms
/// TensorRT 10: FP16 2.5ms
pub fn pipeline(cfg: &rbt_cfg::DetectorCfg) -> RbtResult<HashMap<EnemyId, Vec<DetectedArmor>>> {
    let session_builder = Session::builder()?;
    let (session_builder, ort_ep) = configure_session_builder(
        session_builder,
        cfg.ort_ep.as_str(),
        cfg.armor.engine_path.as_str(),
    )?;
    info!("using ONNX Runtime execution provider: {}", ort_ep.as_str());
    let mut session = session_builder
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(16)?
        .commit_from_file(cfg.armor.model_path.as_str())?;

    let tim = std::time::Instant::now();
    let mut detector = ArmorDetector::init(cfg)?;
    let elapsed = tim.elapsed();
    info!("Initialization time elapsed: {:?}", elapsed);

    detector.pre_process();

    let tim2 = std::time::Instant::now();
    let outputs: SessionOutputs<'_> =
        session.run(inputs![TensorRef::from_array_view(detector.input.view())?])?;
    let elapsed = tim2.elapsed();
    info!("Inference time elapsed: {:?}", elapsed);

    let tim = std::time::Instant::now();
    let result = detector.post_process(&outputs, cfg)?;
    let elapsed = tim.elapsed();
    info!("Postprocessing time elapsed: {:?}", elapsed);

    Ok(result)
}
