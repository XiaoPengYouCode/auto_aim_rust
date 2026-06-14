# Repository Instructions

## 项目定位

- 这是 RoboMaster 自瞄 Rust 工作区，主目标是常驻在机器人上的视觉与发控二进制应用。
- 主运行入口是 `app/auto_aim_async`，公共算法和业务模块放在 `lib` crate。
- 当前主线是单进程、多 Tokio task 的异步流水线：视频/相机帧源、检测、解算、估计、发控和控制帧发送按 latest-queue 串联。
- 架构和运行时主线以 `docs/系统架构与发控主线.md` 为准；改主线时同步更新这份文档。

## 目录边界

- `lib/src/rbt_base`：数学、几何、PnP、弹道等底层算法。
- `lib/src/rbt_infra`：配置、日志、错误、ONNX Runtime EP 选择、异步队列等基础设施。
- `lib/src/rbt_mod`：业务模块，包括装甲板、能量机关、通信协议、检测器、估计器、发控、运行时路由。
- `app/auto_aim_async`：真实异步主线。这里应只做调度、队列、session 初始化、route 分发和控制循环，不把算法本体堆进 task 函数。
- `app/single_frame_dev`、`app/ippe_benchmark`、`app/comm_test`：开发、基准和通信验证入口，不要反向影响主二进制架构。
- `cfg/rbt_cfg.toml`：主配置入口。可调参数优先进入配置文件，并在 `lib/src/rbt_infra/rbt_cfg.rs` 建类型。

## 运行时主线

- `AutoShot` 和 `HitOutpost` 走装甲板 pipeline；`HitBigBuff` 和 `HitSmallBuff` 在内部统一映射到 `EnergyMechanism` route。
- `RuntimeRouter` / `ModeContext` 是任务模式切换的唯一入口。不要在各 task 内自行判断协议枚举并绕过 route state。
- route 切换必须清理 Armour、EnergyMechanism 两侧队列，并重置对应 estimator/controller，避免旧帧进入新模式。
- 控制输出统一走 `CtrlData` 和既有 CAN payload 序列化。协议里的 `HitBigBuff`、`HitSmallBuff`、`ShotBuffMode` 是线协议兼容名，内部模块命名使用 `EnergyMechanism` / `energy_mechanism`。

## 配置与入口

- 这个仓库面向常驻机器人二进制，运行入口必须清晰、稳定、可审计。
- 环境变量入口必须精心设计，默认不要新增。肆意添加环境变量会导致代码仓库入口混乱，也会让真实部署行为难以追踪。
- 除非存在明确、长期、不可替代的运行时需求，并且已经在配置、协议或命令行入口之外论证过，否则不要通过环境变量改变主线行为。
- 临时调试、smoke test、离线验证需求应优先使用测试、配置文件、固定样例数据或显式工具入口，不要塞进主二进制的环境变量分支。
- README 中用于 native library loading 的系统级环境变量说明不属于应用行为开关；不要把它扩展成主线控制入口。

## 模型与推理

- Armour 和 EnergyMechanism 是两套独立模型、tensor contract 和后处理流程。不要用一个 reshape 或 decode contract 同时假设两类输出。
- ONNX metadata 相关假设需要有测试锁住输入/输出名、dtype 和 shape。
- 推理优先使用 `ort` 和现有 `configure_session_builder`，不要引入 C++/OpenCV/FFI 旁路来绕过 Rust 主线。
- 模型路径和 EP 配置来自 `cfg/rbt_cfg.toml`，不要硬编码到算法模块里。

## 编码规则

- 优先保持简单、低复杂度、可调试。新增抽象必须降低真实重复或隔离明确边界。
- 新业务能力优先按现有分层落位：detected / solved / estimator or tracker / fire_control / route wiring / tests。
- 配置字段使用业务语义命名。能量机关相关字段使用 `energy_mechanism`，不要新增旧称呼。
- 代码注释只解释不显然的约束、坐标系、协议兼容或数学假设；不要写重复代码表面的注释。
- 不要在主线 task 里做大块同步阻塞计算；需要阻塞推理或重计算时使用现有 `spawn_blocking` 模式。

## 验证要求

- PR 前至少运行：
  - `cargo fmt --all -- --check`
  - `cargo clippy --locked --workspace --all-targets -- -D warnings`
  - `cargo test --locked --workspace --all-targets`
- CI 使用 `.github/workflows/ci.yml` 中同一套命令；本地结论应以这些命令为准。
- 改模型 contract、decode、PnP、tracker、route 或发控时，需要补对应单元测试。
- 改 `app/auto_aim_async` 主线时，确认 Armour route 不回归，并验证 EnergyMechanism route 的状态切换、队列清理和控制输出路径。
