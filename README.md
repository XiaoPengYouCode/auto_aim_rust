<p align="center">
  <h1 align="center">🤖 一次对自瞄的尝试 🎯</h1>
  <p align="center">
    <img src="https://img.shields.io/badge/Language-Rust-orange?style=for-the-badge"/ alt="language=rust">
    <img src="https://img.shields.io/badge/Platform-RoboMaster-blue?style=for-the-badge"/ alt="platform=robomaster">
    <img src="https://img.shields.io/badge/Team-3SE-orange?style=for-the-badge"/ alt="team=3SE">
  </p>
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

## 📡 核心模块简介

| 模块名                | 说明                                   |
| ------------------- | -------------------------------------- |
| `lib/rbt_app`       | 上层APP                     |
| `lib/rbt_base`      | 几何、数学、姿态估计核心模块                     |
| `lib/rbt_infra`     | 配置、日志、工具库、异步通信支持                   |
| `lib/rbt_mod`       | 包含视觉识别、装甲板选择、控制策略等上层业务逻辑           |

---

## 💻 开发与运行

### 🛠️ 环境要求

- Rust Stable(没有测试过MSRV)

### 🚀 快速运行

```bash
cargo build --release
cargo run -p auto_aim_async --release
```

<p align="center">
  <img src="assets/3se-logo.png" width="150" alt="3SE Logo"/>
  <p align="center">❤️爱来自东南大学3SE战队❤️</p>
</p>
