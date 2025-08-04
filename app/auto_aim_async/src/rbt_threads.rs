use ort::inputs;
use ort::value::TensorRef;
use std::sync::Arc;
use tokio::task::JoinHandle;
use tracing::{error, info, warn};

// use crate::rbt_cfg::{self, DetectorConfig, RbtCfg};
use lib::rbt_mod::rbt_armor::ArmorKeyPoints;
use lib::rbt_mod::rbt_detector::BBox;
use lib::rbt_mod::rbt_detector::rbt_frame::{RbtFrame, RbtFrameStage};
use lib::rbt_mod::rbt_detector::rbt_yolo::{letterbox, nms};

use lib::rbt_infra::rbt_global::{FAILED_COUNT, GENERIC_RBT_CFG, IS_RUNNING};
use lib::rbt_infra::rbt_queue_async::RbtQueueAsync;

pub mod rbt_cfg_thread;

/// 图像预处理阶段：读取图像并通过通道发送到下一阶段。
/// 此函数负责读取图像、调整图像大小、转换为归一化格式，并为推理阶段准备数据。
pub fn pre_process(queue: Arc<RbtQueueAsync<RbtFrame>>) -> JoinHandle<()> {
    tokio::spawn(async move {
        for frame_id in 1..=1000 {
            let mut rbt_frame = RbtFrame::new();
            // 在阻塞线程中执行图像处理操作，以避免阻塞异步运行时
            let result = tokio::task::spawn_blocking(move || {
                // 从磁盘加载图像
                let resized_img =
                    image::open("../../../imgs/test_resize.jpg").expect("无法打开图像");

                // 创建一个 4D 数组以存储处理后的图像数据。
                let mut input_array = nd::Array4::zeros((1, 3, 384, 640));
                letterbox(&mut input_array, &resized_img);

                rbt_frame.pre_data().assign(&input_array);
                rbt_frame.set_id(frame_id);
                rbt_frame.set_state(RbtFrameStage::Pre);
                rbt_frame
            })
            .await;

            // 处理阻塞任务的结果。
            if let Ok(frame) = result {
                // 通过通道将处理后的帧发送到下一阶段
                let id = frame.id(); // 获取帧 ID，用于日志记录
                info!(
                    "预处理阶段：图像 {} 处理完成，耗时 {:?}",
                    id,
                    frame.time_used()
                );
                let _ = queue.force_push(frame);
                if id == 1000 {
                    IS_RUNNING.store(false, std::sync::atomic::Ordering::SeqCst);
                    info!(
                        "Failed count: {}",
                        FAILED_COUNT.load(std::sync::atomic::Ordering::SeqCst)
                    );
                    info!("预处理阶段：已处理完所有图像，停止处理");
                    break;
                }
            } else {
                error!("预处理阶段：图像 {} 处理失败", frame_id);
            }
        }
    })
}

/// 推理阶段：接收预处理后的数据，执行模型推理，并将结果发送到后续处理阶段
pub fn infer(
    pre_infer_queue: Arc<RbtQueueAsync<RbtFrame>>, // 接收预处理阶段的输出
    mut session: ort::session::Session,            // ONNX Runtime 推理会话
    infer_post_queue: Arc<RbtQueueAsync<RbtFrame>>, // 发送推理结果到后续处理阶段
) -> JoinHandle<()> {
    tokio::spawn(async move {
        loop {
            if IS_RUNNING.load(std::sync::atomic::Ordering::SeqCst) == false {
                info!("infer: Stopping processing as IS_RUNNING is false");
                break;
            }
            if let Some(mut frame) = pre_infer_queue.pop().await {
                info!(
                    "infer: Frame ID {} received form processing, time used: {:?}",
                    frame.id(),
                    frame.time_used()
                );
                frame.set_state(RbtFrameStage::Infer);
                let id = frame.id(); // 获取帧 ID，用于日志记录
                // 在阻塞线程中执行推理操作
                let output_result = tokio::task::spawn_blocking(move || {
                    // 执行模型推理
                    let output_array = {
                        let outputs = session
                            .run(inputs![
                                TensorRef::from_array_view(frame.pre_data()).unwrap()
                            ])
                            .unwrap();
                        outputs["output0"]
                            .try_extract_array::<f32>()
                            .unwrap()
                            .t()
                            .into_owned()
                            .as_standard_layout()
                            .into_shape_with_order((5040, 48, 1)) // 重塑输出形状，基于先验的模型尺寸
                            .expect("Failed to reshape output")
                            .to_owned()
                    };
                    frame.infer_data().assign(&output_array);
                    (session, frame) // 返回会话和处理后的帧
                })
                .await;

                // 处理推理结果
                if let Ok((session_return, output)) = output_result {
                    let _ = infer_post_queue.force_push(output); // 将推理结果发送到后处理阶段
                    session = session_return; // 确保会话在闭包外部可用
                } else {
                    warn!("infer: Failed to process frame ID: {}", id);
                    break;
                }
            }
        }
    })
}

/// 后处理阶段：接收推理结果，执行目标检测框处理，并提取装甲板信息
pub async fn post_process(frame: Arc<RbtQueueAsync<RbtFrame>>) -> JoinHandle<()> {
    tokio::spawn(async move {
        loop {
            if IS_RUNNING.load(std::sync::atomic::Ordering::SeqCst) == false {
                info!("post_process: Stopping processing as IS_RUNNING is false");
                break;
            }
            let detector_cfg = GENERIC_RBT_CFG.read().unwrap().detector_cfg.clone();
            if let Some(mut frame) = frame.pop().await {
                let time_used = frame.time_used(); // 获取处理时间
                info!(
                    "post_process: Frame ID {} received in {:?}",
                    frame.id(),
                    time_used
                );
                frame.set_state(RbtFrameStage::Post); // 更新状态为后处理
                let id = frame.id(); // 获取帧 ID，用于日志记录
                // 在阻塞线程中执行后处理操作
                let result = tokio::task::spawn_blocking(move || {
                    let binding = frame.infer_data();
                    let output = binding.slice(nd::s![.., .., 0]);

                    let mut boxes = Vec::new(); // 存储目标检测框

                    // 遍历每一行输出，提取目标检测框信息
                    for (idx, row) in output.axis_iter(nd::Axis(0)).enumerate() {
                        let row: Vec<_> = row.iter().copied().collect();
                        let (class_id, prob) = row[4..40]
                            .iter()
                            .enumerate()
                            .map(|(index, value)| (index, *value))
                            .reduce(|accum, row| if row.1 > accum.1 { row } else { accum })
                            .unwrap();

                        // 如果置信度低于阈值，跳过该检测框
                        if prob < detector_cfg.confidence_threshold {
                            continue;
                        }

                        let xc = row[0]; // 中心点 x 坐标
                        let yc = row[1]; // 中心点 y 坐标
                        let w = row[2]; // 检测框宽度
                        let h = row[3]; // 检测框高度

                        let half_w = w / 2.0;
                        let half_h = h / 2.0;

                        boxes.push((
                            BBox::new(xc - half_w, yc - half_h, xc + half_w, yc + half_h),
                            class_id,
                            prob,
                            idx,
                        ));
                    }

                    // 非极大值抑制：去除重叠的检测框，保留最优框
                    let result = nms(boxes);

                    // 收集装甲板信息
                    let mut armors = Vec::<ArmorKeyPoints>::with_capacity(result.len());
                    for (_, _, _, idx) in result {
                        let armor = ArmorKeyPoints::new(
                            ImgCoord::from_f32(output[[idx, 0]], output[[idx, 1]]), // 中心点坐标
                            ImgCoord::from_f32(output[[idx, 40]], output[[idx, 41]]), // 特征点 1
                            ImgCoord::from_f32(output[[idx, 42]], output[[idx, 43]]), // 特征点 2
                            ImgCoord::from_f32(output[[idx, 44]], output[[idx, 45]]), // 特征点 3
                            ImgCoord::from_f32(output[[idx, 46]], output[[idx, 47]]), // 特征点 4
                        );
                        armors.push(armor); // 添加到装甲板列表
                    }
                    (frame, armors) // 返回装甲板信息
                })
                .await;

                if let Ok((_frame, _armors)) = result {
                    // 处理装甲板信息
                    // for armor in armors {
                    //     info!("Detected armor: {:?}", armor);
                    // }
                    // 将处理后的帧发送到下一阶段或存储
                    // 这里可以添加代码将处理后的帧存储或发送到其他组件
                    let time_used = _frame.time_used(); // 获取处理时间
                    info!(
                        "post_process: Frame ID {} processed successfully, time used: {:?}",
                        id, time_used
                    );
                } else {
                    warn!("post_process: Failed to process frame ID: {}", id);
                }
            } else {
                warn!("post_process: No frame available for processing");
                continue; // 如果没有数据，继续等待
            }
        }
    })
}
