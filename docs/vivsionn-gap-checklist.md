# vivsionn Gap Checklist

This checklist tracks the functional gaps found while comparing this Rust workspace with `/Users/flamingo/Projects/robomaster/vivsionn`.

| feature | 目标仓库 | 当前仓库 | 备注 |
|---|---|---|---|
| [x] P0 真 CAN 收发闭环 | `Serial + SocketCAN` 真读 `0x203/0x204`、真发 `0x100` | SocketCAN runtime task 打开 `can0`，配对 feedback 帧，生产 `SensData`，并发送序列化后的 `CtrlData` | 已补齐主线闭环，协议格式仍由 `rbt_comm_frame.rs` 锁定 |
| [ ] P0 相机/视频源 | 海康相机、模式切曝光、离线 `.avi + .csv` 回放 | 固定 ffmpeg 读离线视频 | 必补。不上这个很难常驻上车 |
| [ ] P0 常驻机器人入口 | `supervisor + YoloDetect()` 常驻运行 | 视频结束进程退出 | 实车稳定性缺口 |
| [x] P0 装甲板 pitch 弹道控制 | 发控会算重力补偿并下发 pitch | 装甲板路线已根据 planner 目标位置计算弹道 pitch，并随 yaw 一起下发 | 已补齐装甲板 3D 发控输出 |
| [x] P0 YPD geometry recovery | 有 armor jump 后几何恢复、协方差膨胀 | tracker 在多装甲板观测后打开 recovery window，并在连续几何 mismatch 后膨胀 `dr/h` covariance | 已补齐跳板、错配、重获相关基础恢复逻辑 |
| [x] P0 前哨站特化 | outpost 高度相位锁定、半径先验、yaw 恢复 | tracker 已做 outpost observed/radial yaw 转换、高度相位锁定、锁定高度冻结、半径先验和 rejected update 门控 | 已补齐前哨站专用 tracker 路径 |
| [x] P0 能量机关 R 圆心/切换门控 | `Buff_Detector` 有 R 圆心修正、模板/轮廓 fallback、锁定门控 | solve stage 已修正不一致 R 圆心几何，tracker 已对大符目标切换做 defer/rebind phase gate | 本轮补了 R 圆心和切换门控；模板/轮廓 fallback 仍未纳入 |
| [x] P0 能量机关 tracker/aimer | 相位 EKF、大小符曲线模型、相位化预瞄、pitch lead | 大符曲线 EKF（基于共享不定长 EKF）+ 两轮飞行时间迭代 + yaw preview horizon + pitch lead + 配置化偏置 | 大符走 `BigBuffCurveEskf` 曲线预测（`speed=a·sin(phase)+base-a`），小符保留常速；aimer 两轮弹道迭代、yaw MPC horizon 由 tracker 预瞄生成 |
| [ ] P1 主线热更新调参 | `param.yaml` 每秒 reload，曝光/发控/MPC 可调 | 只有实验入口，主线没接 watcher | 上车调参效率会差。本轮不做热更新，配置仅启动加载 |
| [x] P1 配置面补齐 | 大量曝光、门控、MPC、buff 参数 | `rbt_cfg.toml` 主要是 detector/cam/estimator | 新增顶层 `energy_mechanism_cfg`（tracker/aimer/mpc），补齐大符曲线 EKF 全部 knob，serde 默认值保证旧配置兼容 |
| [x] P1 PnP 稳态保护 | 角点细化、位姿 sanity gate | 主线解码后随 `RbtFrame` 流一份原尺寸灰度帧到 solver；Rust IPPE 输入前做灰度灯条端点细化 + 几何规整，输出后做深度/有限值/重投影 RMSE sanity gate | 已补齐 Rust IPPE 路径的稳态保护和灰度帧上下文；未引入 OpenCV/C++ PnP 旁路 |
| [ ] P1 离线录制/回放 | 可录 `.avi + .csv`，强制 task mode 回放 | 缺主链路复盘工具 | 调现场问题很关键 |
| [ ] P1 通信/MPC smoke 工具 | `testSerial`、`can_mpc_yaw_test` 工具链完整 | `comm_test` 基本空 | 接 CAN 后应尽快补 |
| [ ] P2 显示/HUD/录制旁路 | MJPEG/Rerun/HUD/CSV/plot 脚本多 | 有 Rerun 和日志，但观测面较薄 | 影响调试效率 |
| [ ] P2 TRT/ROI/CUDA 性能路径 | TensorRT/CUDA 预处理更贴 Jetson | ORT + ONNX EP | 不是功能缺口，先看实测延迟 |
| [x] 已基本对齐：模式路由 | `AutoShot`/`Outpost`/`Buff` 路由切换 | `RuntimeRouter`/`ModeContext` 已有 | 这块方向对 |
| [x] 已基本对齐：yaw 发控主链 | yaw planner、二阶 MPC、shot phase | Rust 已迁主干 | 主要剩验证/调参 |
| [x] 已基本对齐：CAN 协议格式 | `0x100`/`0x203`/`0x204` 协议 | `rbt_comm_frame.rs` 有实现和单测 | 协议定义不是短板 |
