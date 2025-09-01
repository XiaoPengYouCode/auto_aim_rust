//! 排序和匹配算法模块
//!
//! 该模块包含用于目标匹配的算法实现，如匈牙利算法等。

/// 匈牙利算法实现，用于解决分配问题
///
/// 该算法用于在多项任务和多项资源之间找到最优的一对一匹配，
/// 使得总成本最小化或总收益最大化。
///
/// # 参数
/// * `cost_matrix` - 成本矩阵，其中 cost_matrix[i][j] 表示第 i 个任务分配给第 j 个资源的成本
///
/// # 返回值
/// 返回一个元组 (assignments, total_cost)，其中 assignments 是任务到资源的分配映射，
/// total_cost 是总成本。
pub fn hungarian_algorithm(cost_matrix: &[Vec<f64>]) -> (Vec<Option<usize>>, f64) {
    let rows = cost_matrix.len();
    if rows == 0 {
        return (Vec::new(), 0.0);
    }
    let cols = cost_matrix[0].len();
    if cols == 0 {
        return (vec![None; rows], 0.0);
    }

    // 创建可变的成本矩阵副本
    let mut matrix: Vec<Vec<f64>> = cost_matrix.to_vec();

    // 步骤1: 每行减去该行的最小值
    for i in 0..rows {
        let min_val = matrix[i].iter().cloned().fold(f64::INFINITY, f64::min);
        if min_val.is_finite() {
            for j in 0..cols {
                matrix[i][j] -= min_val;
            }
        }
    }

    // 步骤2: 每列减去该列的最小值
    for j in 0..cols {
        let min_val = (0..rows)
            .map(|i| matrix[i][j])
            .fold(f64::INFINITY, f64::min);
        if min_val.is_finite() {
            for i in 0..rows {
                matrix[i][j] -= min_val;
            }
        }
    }

    // 初始化标记矩阵
    let mut starred: Vec<Vec<bool>> = vec![vec![false; cols]; rows];
    let mut primed: Vec<Vec<bool>> = vec![vec![false; cols]; rows];
    let mut row_covered: Vec<bool> = vec![false; rows];
    let mut col_covered: Vec<bool> = vec![false; cols];

    // 步骤3: 对每行找零元素并标记
    for i in 0..rows {
        for j in 0..cols {
            if matrix[i][j].abs() < f64::EPSILON && !row_covered[i] && !col_covered[j] {
                starred[i][j] = true;
                row_covered[i] = true;
                col_covered[j] = true;
            }
        }
    }

    // 重置覆盖行和列
    row_covered.fill(false);
    col_covered.fill(false);

    // 主循环
    loop {
        // 步骤4: 标记所有独立零元素所在的列
        for i in 0..rows {
            for j in 0..cols {
                if starred[i][j] {
                    col_covered[j] = true;
                }
            }
        }

        // 检查是否所有列都被覆盖（找到最优解）
        if col_covered.iter().filter(|&&c| c).count() == cols.min(rows) {
            break;
        }

        // 步骤5: 找到未覆盖的零元素
        let mut found_zero = false;
        let mut zero_row = 0;
        let mut zero_col = 0;

        'find_zero: for i in 0..rows {
            if !row_covered[i] {
                for j in 0..cols {
                    if !col_covered[j] && matrix[i][j].abs() < f64::EPSILON {
                        zero_row = i;
                        zero_col = j;
                        found_zero = true;
                        break 'find_zero;
                    }
                }
            }
        }

        if found_zero {
            // 标记找到的零元素
            primed[zero_row][zero_col] = true;

            // 检查该行是否有标记的星号零元素
            let mut starred_col = None;
            for j in 0..cols {
                if starred[zero_row][j] {
                    starred_col = Some(j);
                    break;
                }
            }

            if let Some(col) = starred_col {
                // 覆盖该行，取消覆盖该列
                row_covered[zero_row] = true;
                col_covered[col] = false;
            } else {
                // 找到增广路径
                let path = find_augmenting_path(
                    zero_row,
                    zero_col,
                    &starred,
                    &primed,
                    &row_covered,
                    &col_covered,
                );

                // 更新标记
                for (i, j) in path {
                    if starred[i][j] {
                        starred[i][j] = false;
                    } else {
                        starred[i][j] = true;
                    }
                }

                // 清除所有标记
                primed.iter_mut().for_each(|row| row.fill(false));
                row_covered.fill(false);
                col_covered.fill(false);
            }
        } else {
            // 步骤6: 不存在未覆盖的零元素，调整矩阵
            let mut min_val = f64::INFINITY;
            for i in 0..rows {
                if !row_covered[i] {
                    for j in 0..cols {
                        if !col_covered[j] {
                            min_val = min_val.min(matrix[i][j]);
                        }
                    }
                }
            }

            // 从未覆盖行中减去最小值
            for i in 0..rows {
                if !row_covered[i] {
                    for j in 0..cols {
                        matrix[i][j] -= min_val;
                    }
                }
            }

            // 向覆盖列中添加最小值
            for j in 0..cols {
                if col_covered[j] {
                    for i in 0..rows {
                        matrix[i][j] += min_val;
                    }
                }
            }
        }
    }

    // 构造结果
    let mut assignments = vec![None; rows];
    let mut total_cost = 0.0;

    for i in 0..rows {
        for j in 0..cols {
            if starred[i][j] {
                assignments[i] = Some(j);
                total_cost += cost_matrix[i][j];
                break;
            }
        }
    }

    (assignments, total_cost)
}

/// 寻找增广路径
fn find_augmenting_path(
    start_row: usize,
    start_col: usize,
    starred: &[Vec<bool>],
    primed: &[Vec<bool>],
    row_covered: &[bool],
    col_covered: &[bool],
) -> Vec<(usize, usize)> {
    let rows = starred.len();
    let cols = starred[0].len();
    let mut path = vec![(start_row, start_col)];
    let mut done = false;

    while !done {
        // 查找标记的星号零元素在当前列
        let mut starred_row = None;
        for i in 0..rows {
            if starred[i][path.last().unwrap().1] {
                starred_row = Some(i);
                break;
            }
        }

        if let Some(row) = starred_row {
            path.push((row, path.last().unwrap().1));
        } else {
            done = true;
            break;
        }

        // 查找标记的撇号零元素在当前行
        let col = (0..cols)
            .find(|&j| primed[path.last().unwrap().0][j])
            .unwrap();
        path.push((path.last().unwrap().0, col));
    }

    path
}
