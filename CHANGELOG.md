# Change Log

## 2025.6.13下午 - 2025.6.13凌晨

1. 大致实现了 `rbt_log` 文件输出系统，可以根据配置来选择是否在命令行和文件中输出，并根据当前时间自动创建和归类文件夹，创建文件并写入日志信息（学到了很多 `tracing` 相关的知识）
2. 将 `rbt_cfg` 进行了重构，将配置文件进行分层，每个模块拥有自己的配置信息，同时也有一个 `general` 配置负责一些公共配置（这部分怎么写实话还没想好，先按照 Gemini 的建议写了，后面看使用下来有没有什么槽点）
3. 下次请优先扩展多帧运的能力

## 2025.6.14晚上 - 2025.6.15凌晨

1. 实现了一个概念版的 `rbt_benchmark` 用来记录 latency 和 fps，但是现在写完感觉好蠢，因为用了 `Mutex<HashMap>` 做实现

记录一下今天的 prompt，考虑考完模电之后把这个更高效地实现一下

> 请帮我审查我的代码，指出不够合理的地方，不要生成代码，而是用文字进行描述
>
> **需求**：用Rust实现CV流水线监控模块（目标150fps），检测延迟和FPS。流水线含三个顺序线程：
>
> - 取图+前处理（CPU，~20ms）
> - 推理（GPU，~30ms）
> - 后处理+输出（CPU，~20ms）
>
> **延迟检测**：
>
> - 线程1记录帧开始时间（start_time）
> - 线程3计算延迟（当前时间 - start_time）后丢弃帧数据
> - 使用process_id_counter跟踪处理进度
>
> **FPS统计**：
>
> - 1秒周期的轻量级tokio任务计算FPS
> - 统计后重置计数器
>
> **内存管理**：
>
> 可选方案：
>
> - 1秒周期清理未完成帧的时间戳
> - 或在Drop时检查残留数据
>
> 以及一些疑问，虽然我这里用到了三个线程，但是其实每一帧都是顺序执行的，我使用多线程的唯一目的是使用单槽口缓冲区来提高吞吐量（因为第二步要用到GPU，我可以在运行2的同时运行下一步的1或者上一步的3），那么是否使用环形缓冲区等数据结构要相比使用HaspMap加锁，来得更高

## 6.15 下午 6:00 开始 - 6.17下午四点结束

实现了基于 watch 的多线程, 简单记录了一下 FPS 和 lantency，结果如下

- lantency: 40
- FPS: 60

具体实现

使用了 tokio 异步运行时，一共三个运行时，pre, infer和 post，然后一起 join，每个运行时中，所有操作都放在了 blocking 块中（背后应该是线程池），避免阻塞主异步任务，符合 tokio 的设计模式，后续打算一直从这个方向走下去

另外我发现统计 FPS 和 lantency 没有我之前想象中的那么难，之前的工作可能要推倒重来了，我也不确定，再观望使用一段时间

期望下次打开之后，先完成摄像头取帧的工作，用 USB 摄像头来完成任务，而不是一直从硬盘读

## 6.21 以来

前几天一直发现基于Watch的方案可能不是很靠谱，因为需要clone，但我更需要的可能是所有权转移，最后兜兜转转到了 crossbeam-ArrayQueue，并且给帧加上了生命周期检测，可以根据自己的状态生成对应的log，可以体现出生产者速度过快，消费者丢失帧的情况

根据社区的反馈和AI的帮助，在量化模型的前后增加了两个数据转换的节点，这样我就不需要自己处理half了，输入f32之后GPU自动cast为f16，这真是太棒了，感谢社区，感谢 onnx 如此好的生态，再见Half

另外我突发奇想，其实归一化操作也可以放在 onnx 模型中来完成，所以我就把归一化操作给放进去了，这样我的代码又变简单了，很有收获的一天

## 6.22下午-6.23

1. 使用 lazy_static 统一config变量，加入 RwLock 便于之后扩展config热重载
2. 把 TensorRT 支持暂时移除了（todo: 后面可以使用feature的方式加回来）

## 6.27凌晨

> 时间居然过的这么快呢，好几天没更新了

这几天一直在尝试抄 sqpnp 算法的源码，今天总算跑出来一个正确的数字，而且性能也是不到ms级别，虽然值还是不对（主要原因是我对背后算法一无所知啊，完全纯抄的，能对就怪咯），但是我依旧很高兴，于是将代码往 github 上 push 了一下

这几天还发现这个 vscode 和 rust-analyzer 一起开，内存占用太高了，于是我重新配置了一下 helix，用起来居然还行，后面会更多尝试使用 helix 进行开发，而且暂时没有 ai，也能一定程度上提高我的基础编程能力（虽然会降低效率，但是我这个人最大特点就是浮，需要沉淀一下）

## 6.30 凌晨

找到了几个 sqpnp 潜在的错误，但是改正之后效果依然没有变好，有点沮丧就放弃了

基于之前的想法实现了以下配置文件的热更新，然后在不断迭代更新中，感觉 gemini 的建议都非常有启发性，也非常贴合实际，于是在这里粘贴一下

```shell
当前代码可完善之处
以下是一些针对现有代码的直接改进建议，目标是增强其健壮性。

事件防抖 (Debounce)

问题: 某些编辑器（如 Vim）在保存文件时可能会触发多个事件（例如，Modify 然后是 Remove 再 Create）。你当前的 sleep(500ms) 可以缓解一部分问题，但它是在处理之后等待，无法防止短时间内的连续多次重载。

建议: 实现一个真正的防抖逻辑。当收到第一个文件变更事件后，不要立即处理，而是等待一个短暂的窗口期（比如 200ms）。如果在此期间没有新的事件到来，才执行配置重载。如果来了新事件，则重置等待计时器。这样可以确保一系列密集的写操作只触发一次重载。

错误处理

问题: unwrap() 会在出错时导致整个线程 panic。例如，如果启动时 config/rbt_cfg.toml 文件不存在，watcher.watch(...).unwrap() 就会使程序崩溃。

建议: 妥善处理 Result。将 unwrap() 替换为 match 或 ? (如果函数返回 Result)，并在出错时记录日志，然后优雅地决定是退出线程还是重试。

重载失败的处理

问题: 如果新的配置文件 rbt_cfg.toml 存在语法错误，RbtCfg::from_toml_async().await 会失败。当前的 ? 会导致整个 tokio::spawn 的任务中止。

建议: 在大多数场景下，配置热更新失败时不应该让整个服务停机。更好的做法是捕获这个错误，打印日志，并继续使用旧的、有效的配置，同时继续监听文件变化，等待用户修复错误后再次尝试重载。

避免不必要的重载

问题: touch 命令或者内容完全没有变化的保存操作，依然会触发重载逻辑。

建议: 在解析文件前，先判断文件内容是否真的发生了变化。可以比较文件的哈希值 (hash) 或者直接比较新旧两个 RbtCfg 对象（需要为 RbtCfg 实现 PartialEq）。tokio::sync::watch 通道自身会在新旧值相同时阻止通知下游，但我们可以在此之前就避免昂贵的 IO 和反序列化操作。

未来发展方向
以下是一些可以让该系统功能更强大、更通用的扩展方向。

配置来源抽象化

将配置的来源抽象成一个 Trait (例如 ConfigurationProvider)。这样你的系统就不再局限于本地 TOML 文件。可以轻松扩展，以支持从 环境变量、JSON/YAML 文件、远程配置中心 (如 etcd, Consul, Nacos) 或 数据库 中加载配置。

支持监控整个目录

当前只监控单个文件。可以扩展为监控一个配置目录，当目录中任何 .toml 文件发生变化时，重新加载所有配置。这对于复杂的、分模块的配置很有用。

配置校验 (Validation)

在成功解析配置并发送给其他模块之前，增加一个校验层。即使 TOML 格式正确，业务逻辑上也可能存在无效值（如端口号为0，密码为空等）。如果校验失败，则不应用新配置，并打印详细错误。

度量和可观测性 (Observability)

集成 Prometheus 等监控系统。暴露一些关键指标，例如：

配置更新成功/失败的次数。

上一次成功/失败更新的时间戳。

当前使用的配置版本号或哈希值。

模块化订阅

对于大型应用，不同模块可能只关心配置中的一部分。可以设计一种机制，让模块只订阅自己关心的那部分配置的变更，而不是每次都接收完整的 RbtCfg。

```

一直干到了早上八点半，实现了一版非常完善了热更新机制，爽

## 7.2 凌晨

今天发现使用编译的时候因为宏展开的问题非常容易出现内存不足的情况，需要将库和二进制分开编译，于是我选择了使用 WorkSpace 重新组织目录

顺便今天还修复了显卡的bug，原因是之前使用 cuda 12.9 的时候显卡驱动的版本太高了，也不知道为什么一直没有爆雷，但是最近突然开始卡，发现渲染卡居然是 Nvidia，于是对 Nvidia-driver进行了降级来减少损失

## 7.4 凌晨

我决定暂时放弃继续使用helix进行开发，原因是 GUI 的代码编辑器真的很方便，除了性能之外，没有不选择用他的道理。

## 7.9 下午开始

使用rerun进行了装甲板的可视化

## 7.12 凌晨

实现了一版稳定的 IPPE-PnP 求解器。硬编码装甲板尺寸，省去平面化操作，预计算各向同性归一化，相比 OpenCV 标准实现具有更好性能。

## 7.17夜晚 - 7.18凌晨

1. 构建了 `RbtLine2` 数据结构，使用参数方程来描述二维平面中的直线
2. 通过反向延长装甲板并求交点，求解被检测目标的中心
