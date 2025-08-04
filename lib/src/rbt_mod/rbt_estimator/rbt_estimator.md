# `rbt_estimator` 估计器模块

## `enemy_estimator` 

- 状态变量
  1. `theta` 车体中心相对于世界坐标系的角度 `degree`
  2. `distance` 敌方车体中心相对于己方中心的距离 `mm`
  3. `velocity_tang` 切向速度 `mm/s`
  4. `velocity_norm` 法向速度 `mm/s`
  5. `spin_velocity` 陀螺速度 `degree/s`
- 测量变量
  1. `theta`
  2. `distance`
  3. `angle_diff`

首先，比赛一开始，直接创建针对六辆车的观测器

观测器需要有一个状态机，来描述自己当前是什么状态，根据上一帧和当前帧的状态

观测器状态机

- `lost`
  - 维护一个唤醒的时间戳
  - 如果当前帧没有，开始计时 2 s
  - 这个过程用于过滤装甲板被打灭的情况(更好的做法是模型可以识别灰色装甲板)

- `sleep` 
  - 如果 `lost` + 计时之后没有等到这个敌人，观测器进入 `lost`，不再进行相关计算

- `wake`
  - 如果上一帧没有，但是这一帧有了，激活
  - 该帧内需要初始化状态
- `aim`
  - 自瞄工作状态

- `init`
  - 开局初始化

# `ESKF` 模块

