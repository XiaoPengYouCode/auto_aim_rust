use super::armor::{ArmorStaticMsg, ImgCoordinate};
use crate::rbt_err::RbtError;
use crate::util::img_dbg::{BoundingBox, intersection, union};
use half::f16;
use image::{DynamicImage, GenericImageView, ImageReader, imageops::FilterType};
use ndarray as nd;
use ort::{
    execution_providers, inputs,
    session::{Session, SessionOutputs},
    value::TensorRef,
};
use rerun::RecordingStream;
use std::path::PathBuf;

pub struct ArmorDetector {
    img: DynamicImage,
    input: nd::Array<f32, nd::Dim<[usize; 4]>>,
}

impl ArmorDetector {
    fn init() -> ArmorDetector {
        Self {
            img: ImageReader::open("./imgs/140.jpg")
                .unwrap()
                .decode()
                .unwrap(),
            input: nd::Array::zeros((1, 3, 384, 640)),
        }
    }

    /// 前处理操作
    /// 主要包含：
    /// 1. 调整图片大小（主要耗时操作）
    /// 2. 填充灰色
    /// 3. 归一化图片亮度
    fn pre_process(&mut self) {
        self.img = self.img.resize(640, 360, FilterType::Triangle);
        let inv_255 = 1.0 / 255.0;
        let [gray_r, gray_g, gray_b] = [114.0 / 255.0; 3];

        for pixel in self.img.pixels() {
            let x = pixel.0 as _;
            let y = pixel.1 as _;
            let [r, g, b, _] = pixel.2.0;

            if (12..360).contains(&y) {
                self.input[[0, 0, y, x]] = r as f32 * inv_255;
                self.input[[0, 1, y, x]] = g as f32 * inv_255;
                self.input[[0, 2, y, x]] = b as f32 * inv_255;
            } else {
                self.input[[0, 0, y, x]] = gray_r;
                self.input[[0, 1, y, x]] = gray_g;
                self.input[[0, 2, y, x]] = gray_b;
            }
        }
    }

    /// 后处理部分
    /// 1. 筛选0.8以上置信度的装甲板
    /// 2. 利用IOU筛选装甲板
    /// 3. 统计装甲板信息
    /// 4. 切片装甲板图片
    pub fn post_process(&self, outputs: &SessionOutputs) -> ort::Result<Vec<ArmorStaticMsg>> {
        let output = outputs["output0"]
            .try_extract_array::<f16>()?
            .t()
            .into_owned();
        let output_f32 = output.iter().map(|&x| x.to_f32()).collect();
        let output = nd::Array::from_shape_vec(output.shape(), output_f32).unwrap();
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

            boxes.push((
                BoundingBox {
                    x1: xc - w / 2.,
                    y1: yc - h / 2.,
                    x2: xc + w / 2.,
                    y2: yc + h / 2.,
                },
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

        // let _guard = self.img_lock.lock().unwrap();

        // 收集结果
        let mut armors = Vec::<ArmorStaticMsg>::with_capacity(boxes.len());
        for (bounding_box, _, _, idx) in result {
            // 获取装甲板图像切片
            let armor_img = self
                .img
                .view(
                    bounding_box.x1 as u32 + 5,
                    bounding_box.y1 as u32 - 20,
                    (bounding_box.x2 - bounding_box.x1) as u32 - 10,
                    (bounding_box.y2 - bounding_box.y1) as u32 + 40,
                )
                .to_image();
            armor_img.save(format!("imgs/armor_{idx}.png")).unwrap();
            // 收集中心点和特征点坐标信息
            let armor = ArmorStaticMsg::new(
                ImgCoordinate::from_f32(output[[idx, 0]], output[[idx, 1]]),
                ImgCoordinate::from_f32(output[[idx, 40]], output[[idx, 41]]),
                ImgCoordinate::from_f32(output[[idx, 42]], output[[idx, 43]]),
                ImgCoordinate::from_f32(output[[idx, 44]], output[[idx, 45]]),
                ImgCoordinate::from_f32(output[[idx, 46]], output[[idx, 47]]),
                armor_img,
            );

            // 收集装甲板图片
            armors.push(armor);
        }
        Ok(armors)
    }
}

/// 不需要使用cudarc主动将数据拷贝，这个过程ort-rs会自己完成，但是要通过log察觉到是否在节点间新增了memcpy操作，这会严重影响性能
pub fn pipeline(
    rec: &Option<RecordingStream>,
    model_path: PathBuf,
    engine_path: PathBuf,
) -> Result<Vec<ArmorStaticMsg>, RbtError> {
    // build session
    let mut session = Session::builder()?
        .with_execution_providers([execution_providers::TensorRTExecutionProvider::default()
            .with_engine_cache(true)
            .with_engine_cache_path(engine_path.to_str().unwrap())
            .with_fp16(true)
            .build()
            .error_on_failure()])?
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(16)?
        .commit_from_file(model_path)?;

    // init armor detector
    let tim = std::time::Instant::now();
    let mut detector = ArmorDetector::init();
    let elapsed = tim.elapsed();
    tracing::info!("Initialization time elapsed: {:?}", elapsed);

    // preprocessing
    let tim = std::time::Instant::now();
    detector.pre_process();
    let elapsed = tim.elapsed();
    tracing::info!("Preprocessing time elapsed: {:?}", elapsed);

    let tim = std::time::Instant::now();
    let half_input: Vec<f16> = detector.input.iter().map(|&x| f16::from_f32(x)).collect();
    let half_input_array = nd::Array::from_shape_vec(detector.input.shape(), half_input).unwrap();
    tracing::info!("preprocessing time elapsed: {:?}", tim.elapsed());

    // inference
    let tim2 = std::time::Instant::now();
    let outputs = session.run(inputs![
        // TensorRef::from_array_view(&detector.input).unwrap() // tensor
        TensorRef::from_array_view(&half_input_array)? // tensor
    ])?;
    let elapsed = tim2.elapsed();
    tracing::info!("Inference time elapsed: {:?}", elapsed);

    // postprocessing
    let tim = std::time::Instant::now();
    let result = detector.post_process(&outputs)?;
    let elapsed = tim.elapsed();
    tracing::info!("Postprocessing time elapsed: {:?}", elapsed);

    // debug
    if let Some(rec) = rec {
        rec.log(
            "img",
            &rerun::Image::from_color_model_and_tensor(rerun::ColorModel::RGB, detector.img)
                .unwrap(),
        )
        .unwrap();

        for (idx, armor) in result.iter().enumerate() {
            rec.log(
                format!("img/armor_{}_line", idx),
                &rerun::LineStrips2D::new([
                    [armor.left_top().to_f32(), armor.right_bottom().to_f32()],
                    [armor.left_bottom().to_f32(), armor.right_top().to_f32()],
                ])
                .with_radii([1.0])
                .with_colors([rerun::Color::from_rgb(255, 0, 0)]),
            )
            .unwrap();

            rec.log(
                format!("img/armor_{}_center", idx),
                &rerun::Points2D::new([armor.center().to_f32()])
                    .with_radii([3.0])
                    .with_colors([rerun::Color::from_rgb(200, 255, 0)]),
            )
            .unwrap();
        }
    }

    Ok(result)
}
