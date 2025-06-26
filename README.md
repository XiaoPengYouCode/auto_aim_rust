# 🤖 RoboMaster 3SE 战队自瞄

<p align="center">
  <img src="https://img.shields.io/badge/Language-Rust-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Platform-RoboMaster-blue?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/SEU-3SE%20Team-red?style=for-the-badge"/>
</p>

🚀 面向 RoboMaster 赛场的高性能自瞄

**模块化架构、实时并发、全 Rust 编写、性能炸裂！**

---

## 🧠 系统特色 Highlights

- 🦀 **全 Rust 实现**：零成本抽象，安全而强大
- 🚦 **勉强能用的多线程任务调度**：支持并行图像处理、控制策略与通讯任务
- 🎯 **毫无精度可言 PnP 求解器**：自己实现的 SQPnP 模块，支持误差优化与姿态恢复
- ⚙️ **唐完了的状态估计算法**：内置装甲板选择与跟踪模块
- 📡 **毫无技术力的异步消息通信队列**：基于 tokio 构建的高性能事件系统
- 🏆 **面向比赛优化**：针对 RoboMaster 赛场需求深度定制，兼顾实时性与可靠性
- 🏗️ **基础设施完善**：结构化日志（tracing）与错误处理（thiserror）高度成熟，保障系统健壮高效

---

## 🧱 项目结构一览

```text
src/
├── lib.rs                 # 顶层 crate 入口
├── rbt_app/               # 启动器，支持多线程与单线程模式
│   ├── multi_threads.rs
│   └── single_thread.rs
├── rbt_base/              # 底层数据结构、数学库、相机模型、姿态估计
│   ├── rbt_cam.rs
│   ├── rbt_frame.rs
│   ├── rbt_geometry/
│   │   ├── rbt_coord.rs
│   │   └── rbt_rigid.rs
│   ├── rbt_math/
│   │   ├── eskf.rs
│   │   ├── sqpnp/
│   │   │   ├── sqpnp_def.rs
│   │   │   └── sqpnp_impl.rs
│   │   └── sqpnp.rs
├── rbt_infra/             # 基础设施配置、日志、异步队列、工具函数
│   ├── rbt_cfg.rs
│   ├── rbt_log.rs
│   ├── rbt_queue_async.rs
│   └── rbt_utils.rs
├── rbt_mod/               # 功能模块：检测、估计、控制、通讯等
│   ├── rbt_armor.rs
│   ├── rbt_comm.rs
│   ├── rbt_ctrl/
│   │   └── armor_select.rs
│   ├── rbt_detector/
│   │   └── rbt_detect_proc.rs
│   ├── rbt_estimator/
│   │   ├── armor_model.rs
│   │   └── enemy_model.rs
│   ├── rbt_solver/
│   └── rbt_threads.rs
```

---

## 📡 核心模块简介

| 模块名              | 说明                             |
| ---------------- | ------------------------------ |
| `rbt_app`        | 系统主入口，支持多种运行模式                 |
| `rbt_base`       | 几何、数学、姿态估计核心模块                 |
| `rbt_infra`      | 配置、日志、工具库、异步通信支持               |
| `rbt_mod`        | 包含视觉识别、装甲板选择、控制策略等上层业务逻辑       |

---

## 💻 开发与运行

### 🛠️ 环境要求

- Rust ≥ 1.87.0（建议使用 stable）
- 构建工具：`cargo`，支持自动并发编译

### 🚀 快速运行

```bash
cargo build --release
cargo run --bin auto_aim_async
```

---

<p align="center">
  <img src="assets/3se-logo.png" width="150" alt="3SE Logo"/>
  <p align="center">❤️爱来自东南大学3SE战队❤️</p>
</p>
