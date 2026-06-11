# `rbt_estimator` 估计器模块

估计器主线现在以 `YpdAngleTracker` 为核心，不再维护旧的敌方状态镜像。

## 主线流程

1. `RbtHandlerPoll` 从 `RbtSolvedResults` 中选择当前要跟踪的敌方 ID。
2. 对被选中的 `RbtEstimator`，有观测时将 `SolvedArmor` 转成 `YpdObservation`。
3. `YpdAngleTracker` 执行预测；有观测时再做 batch update 和多装甲板 ID 匹配。
4. tracker 输出 `YpdTrackerSnapshot`，作为当前中心、速度、yaw、半径、高度差和预测装甲板列表的唯一估计结果。
5. 发控目标点从 `YpdTrackerSnapshot` 直接生成，不再维护额外的装甲板包装状态。

## 状态机

- `Init` / `Sleep`: 清空 tracker 和当前 snapshot。
- `WakeUp` / `Recovery` / `Track`: 预测后用观测修正。
- `Lost` / `Switching`: 无观测纯预测。
