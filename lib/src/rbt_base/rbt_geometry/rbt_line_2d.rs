// 数学上的参数方程 L(t) = point + t * direction
#[derive(Debug, Clone, Copy)]
pub struct RbtLine2 {
    pub point: na::Point2<f64>,      // 直线上的一个点 (P_start)
    pub direction: na::Vector2<f64>, // 直线的方向向量 (V_dir)
}

// 主函数：寻找交点
pub fn find_intersection(l1: &RbtLine2, l2: &RbtLine2) -> na::Point2<f64> {
    let p1 = l1.point;
    let d1 = l1.direction;
    let p2 = l2.point;
    let d2 = l2.direction;

    // --- 第2步：计算行列式/2D叉乘 ---
    let denominator = d1.perp(&d2); // nalgebra的perp()就是计算2D叉乘，非常方便！

    // 我们需要解出 t: l1(t) = p1 + t*d1
    // t = ((p2 - p1) x d2) / (d1 x d2)
    // nalgebra 没有直接的点向量减法，但我们可以转换
    let p2_minus_p1 = p2.coords - p1.coords;
    let t_numerator = p2_minus_p1.perp(&d2);

    let t = t_numerator / denominator;

    // 用 t 计算交点
    let intersection_point = p1 + t * d1;

    intersection_point
}
