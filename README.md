<div align="center">
  <h1>🤖 自 瞄 🎯</h1>
</div>

[![Language: Rust](https://img.shields.io/badge/Language-Rust-orange?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/ "Rust官方")
[![Platform: RoboMaster](https://img.shields.io/badge/Platform-RoboMaster-blue?style=for-the-badge&logo=robot&logoColor=white)](https://www.robomaster.com/zh-CN/ "RoboMaster官网")
[![Team: 3SE](https://img.shields.io/badge/Team-3SE-red?style=for-the-badge&logo=steam&logoColor=white)](https://space.bilibili.com/1911835891 "3SE战队B站")
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge&logo=opensourceinitiative&logoColor=white)](https://opensource.org/licenses/MIT "MIT协议")
[![ONNX: ort-rs](https://img.shields.io/badge/ONNX-ort--rs-informational?style=for-the-badge&logo=onnx&logoColor=white)](https://github.com/pykeio/ort "Rust ONNX运行时")

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
  <img src="assets/3se-logo.png" width="150" alt="3SE Logo"/>&nbsp;&nbsp;&nbsp;
  <img src="assets/robo-rust-logo.svg" width="130" alt="RoboRust Logo"/>
  <p align="center">❤️爱来自东南大学3SE战队❤️</p>
</p>

</p>
