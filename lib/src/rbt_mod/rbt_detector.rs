use image::{DynamicImage, GenericImageView, ImageReader};
use ndarray as nd;
use ort::{
    execution_providers, inputs,
    session::{Session, SessionOutputs},
    value::TensorRef,
};
use tracing::{error, info};

use crate::{rbt_err::RbtError, rbt_mod::rbt_armor::ArmorClass};
use crate::{
    rbt_infra::rbt_cfg,
    rbt_mod::rbt_armor::ArmorStaticMsg,
    rbt_mod::rbt_generic::ImgCoord, // Import ImgCoord for image coordinates
};

pub mod rbt_detect_proc;

pub struct ArmorDetector {
    img: DynamicImage,
    input: nd::Array<f32, nd::Dim<[usize; 4]>>,
}

impl ArmorDetector {
    fn init(_cfg: &rbt_cfg::DetectorCfg) -> ArmorDetector {
        Self {
            img: ImageReader::open("./imgs/test_resize.jpg")
                .unwrap()
                .decode()
                .unwrap(),
            input: nd::Array::zeros((1, 3, 384, 640)),
        }
    }

    /// 前处理
    /// 主要包含：
    /// 1. 调整图片大小（主要耗时操作）
    /// 2. 填充灰色
    fn pre_process(&mut self) {
        // self.img = self.img.resize(640, 360, FilterType::Triangle);
        let gray = 114.0;

        for pixel in self.img.pixels() {
            let x = pixel.0 as _;
            let y = pixel.1 as _;
            let [r, g, b, _] = pixel.2.0;

            if (12..360).contains(&y) {
                self.input[[0, 0, y, x]] = r as f32;
                self.input[[0, 1, y, x]] = g as f32;
                self.input[[0, 2, y, x]] = b as f32;
            } else {
                self.input[[0, 0, y, x]] = gray;
                self.input[[0, 1, y, x]] = gray;
                self.input[[0, 2, y, x]] = gray;
            }
        }
    }

    /// 后处理
    /// 1. 筛选0.8以上置信度的装甲板
    /// 2. 利用IOU筛选装甲板
    /// 3. 统计装甲板信息
    /// 4. 切片装甲板图片
    pub fn post_process(&self, outputs: &SessionOutputs) -> ort::Result<Vec<ArmorStaticMsg>> {
        // // f32
        let output = outputs["output0"]
            .try_extract_array::<f32>()?
            .t()
            .into_owned();

        let mut boxes = Vec::new();
        let output = output.slice(nd::s![.., .., 0]);
        for (idx, row) in output.axis_iter(nd::Axis(0)).enumerate() {
            let row: Vec<_> = row.iter().copied().collect();
            let (class_id, prob) = row[4..40]
                .iter()
                .enumerate()
                .map(|(index, value)| (index, *value))
                .reduce(|accum, row| if row.1 > accum.1 { row } else { accum })
                .unwrap();
            if prob < 0.8 {
                continue;
            }
            let xc = row[0];
            let yc = row[1];
            let w = row[2];
            let h = row[3];

            let half_w = w / 2.0;
            let half_h = h / 2.0;

            boxes.push((
                BBox::new(xc - half_w, yc - half_h, xc + half_w, yc + half_h),
                // label,
                class_id,
                prob,
                idx,
            ));
        }

        // 非极大值抑制
        // 作用是寻找最准确的目标检测框
        boxes.sort_by(|box1, box2| box2.2.total_cmp(&box1.2));
        let mut result = Vec::new();

        while !boxes.is_empty() {
            result.push(boxes[0]);
            boxes = boxes
                .iter()
                .filter(|box1| {
                    intersection(&boxes[0].0, &box1.0) / union(&boxes[0].0, &box1.0) < 0.7
                })
                .copied()
                .collect();
        }

        // 收集结果
        let mut armors = Vec::<ArmorStaticMsg>::with_capacity(boxes.len());
        for (_, class_id, _, idx) in result {
            let armor = ArmorStaticMsg::new(
                ImgCoord::from_f32(output[[idx, 0]], output[[idx, 1]]),
                ImgCoord::from_f32(output[[idx, 40]], output[[idx, 41]]),
                ImgCoord::from_f32(output[[idx, 42]], output[[idx, 43]]),
                ImgCoord::from_f32(output[[idx, 44]], output[[idx, 45]]),
                ImgCoord::from_f32(output[[idx, 46]], output[[idx, 47]]),
            );

            let armor_class = ArmorClass::from_yolo_output_idx(class_id).unwrap();

            info!(
                "Armor {} detected: center: {:?}",
                armor_class,
                armor.center()
            );

            // 收集装甲板图片
            armors.push(armor);
        }
        Ok(armors)
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
pub fn pipeline(cfg: &rbt_cfg::DetectorCfg) -> Result<Vec<ArmorStaticMsg>, RbtError> {
    // build session
    let session_builder = Session::builder()?;
    let mut session = if cfg.ort_ep == "TensorRT" {
        session_builder.with_execution_providers([
            execution_providers::TensorRTExecutionProvider::default()
                .with_engine_cache(true)
                .with_engine_cache_path(cfg.armor_detect_engine_path.as_str())
                .with_fp16(true)
                .build()
                .error_on_failure(),
        ])?
    } else if cfg.ort_ep == "OpenVINO" {
        session_builder.with_execution_providers([
            execution_providers::OpenVINOExecutionProvider::default()
                .with_device_type("GPU")
                .build()
                .error_on_failure(),
        ])?
    } else {
        error!("Unsupported execution provider: {}", cfg.ort_ep);
        return Err(RbtError::UnsupportedExecutionProvider(cfg.ort_ep.clone()));
    }
    .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
    .with_inter_threads(16)?
    .commit_from_file(cfg.armor_detect_model_path.as_str())?;

    // init armor detector
    let tim = std::time::Instant::now();
    let mut detector = ArmorDetector::init(cfg);
    let elapsed = tim.elapsed();
    info!("Initialization time elapsed: {:?}", elapsed);

    // preprocessing
    detector.pre_process();

    // inference
    let tim2 = std::time::Instant::now();
    let outputs: SessionOutputs<'_> = session.run(inputs![
        TensorRef::from_array_view(&detector.input).unwrap()
    ])?;
    let elapsed = tim2.elapsed();
    info!("Inference time elapsed: {:?}", elapsed);

    // postprocessing
    let tim = std::time::Instant::now();
    let result = detector.post_process(&outputs)?;
    let elapsed = tim.elapsed();
    info!("Postprocessing time elapsed: {:?}", elapsed);

    Ok(result)
}

/// BoundingBox yolo模型候选框
/// 因为目前跟神经网络交互的部分暂时还是 f32，所以暂时没有提供泛型实现
#[derive(Debug, Clone, Copy)]
pub struct BBox(f32, f32, f32, f32);

impl BBox {
    pub fn new(x1: f32, y1: f32, x2: f32, y2: f32) -> Self {
        BBox(x1, y1, x2, y2)
    }

    #[inline(always)]
    fn x1(&self) -> f32 {
        self.0
    }

    #[inline(always)]
    fn y1(&self) -> f32 {
        self.1
    }

    #[inline(always)]
    fn x2(&self) -> f32 {
        self.2
    }

    #[inline(always)]
    fn y2(&self) -> f32 {
        self.3
    }
}

pub fn intersection(box1: &BBox, box2: &BBox) -> f32 {
    (box1.x2().min(box2.x2()) - box1.x1().max(box2.x1()))
        * (box1.y2().min(box2.y2()) - box1.y1().max(box2.y1()))
}

pub fn union(box1: &BBox, box2: &BBox) -> f32 {
    ((box1.x2() - box1.x1()) * (box1.y2() - box1.y1()))
        + ((box2.x2() - box2.x1()) * (box2.y2() - box2.y1()))
        - intersection(box1, box2)
}
