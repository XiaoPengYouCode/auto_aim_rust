# Armor 部分

在每次程序运动的时候，直接创建所有可能会遇到的 `Enemy`，持续化地存储 `Enemy` 所有信息

每个 `Enemy` 需要存储的信息结构体如下

```rust
/// A_N 代表装甲板数量，其他兵种为 4, 前哨站为 3
pub struct Enemy<A_N: usize> {
    // 装甲板类型（大小装甲板）
    armor_type: ArmorType,
    armor_id: ArmorID,
    // 选择第一次看到该车的第一块装甲板为 idx = 0
    // z 轴高度需要做特殊处理
    armor_rads_and_zs: [(f64, f64); A_N], 
}
```