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

    let assignments = if rows <= cols {
        rectangular_hungarian(cost_matrix)
    } else {
        let mut transposed = vec![vec![0.0; rows]; cols];
        for (i, row) in cost_matrix.iter().enumerate() {
            for (j, cost) in row.iter().enumerate() {
                transposed[j][i] = *cost;
            }
        }

        let transposed_assignments = rectangular_hungarian(&transposed);
        let mut assignments = vec![None; rows];
        for (col, row) in transposed_assignments.into_iter().enumerate() {
            if let Some(row) = row {
                assignments[row] = Some(col);
            }
        }
        assignments
    };

    let total_cost = assignments
        .iter()
        .enumerate()
        .filter_map(|(row, col)| col.map(|col| cost_matrix[row][col]))
        .sum();

    (assignments, total_cost)
}

fn rectangular_hungarian(cost_matrix: &[Vec<f64>]) -> Vec<Option<usize>> {
    let rows = cost_matrix.len();
    let cols = cost_matrix[0].len();
    debug_assert!(rows <= cols);

    let mut row_potential = vec![0.0; rows + 1];
    let mut col_potential = vec![0.0; cols + 1];
    let mut matching_row_by_col = vec![0usize; cols + 1];
    let mut previous_col = vec![0usize; cols + 1];

    for row in 1..=rows {
        matching_row_by_col[0] = row;
        let mut current_col = 0;
        let mut min_reduced_cost = vec![f64::INFINITY; cols + 1];
        let mut used_col = vec![false; cols + 1];

        loop {
            used_col[current_col] = true;
            let current_row = matching_row_by_col[current_col];
            let mut delta = f64::INFINITY;
            let mut next_col = 0;

            for col in 1..=cols {
                if used_col[col] {
                    continue;
                }

                let reduced_cost = cost_matrix[current_row - 1][col - 1]
                    - row_potential[current_row]
                    - col_potential[col];
                if reduced_cost < min_reduced_cost[col] {
                    min_reduced_cost[col] = reduced_cost;
                    previous_col[col] = current_col;
                }
                if min_reduced_cost[col] < delta {
                    delta = min_reduced_cost[col];
                    next_col = col;
                }
            }

            for col in 0..=cols {
                if used_col[col] {
                    row_potential[matching_row_by_col[col]] += delta;
                    col_potential[col] -= delta;
                } else {
                    min_reduced_cost[col] -= delta;
                }
            }

            current_col = next_col;
            if matching_row_by_col[current_col] == 0 {
                break;
            }
        }

        while current_col != 0 {
            let next_col = previous_col[current_col];
            matching_row_by_col[current_col] = matching_row_by_col[next_col];
            current_col = next_col;
        }
    }

    let mut assignments = vec![None; rows];
    for (col, row) in matching_row_by_col.into_iter().enumerate().skip(1) {
        if row != 0 {
            assignments[row - 1] = Some(col - 1);
        }
    }

    assignments
}

#[cfg(test)]
mod tests {
    use super::hungarian_algorithm;

    #[test]
    fn test_hungarian_square_matrix() {
        let cost_matrix = vec![
            vec![4.0, 1.0, 3.0],
            vec![2.0, 0.0, 5.0],
            vec![3.0, 2.0, 2.0],
        ];

        let (assignments, total_cost) = hungarian_algorithm(&cost_matrix);

        assert_eq!(assignments, vec![Some(1), Some(0), Some(2)]);
        assert_eq!(total_cost, 5.0);
    }

    #[test]
    fn test_hungarian_rectangular_matrix() {
        let cost_matrix = vec![vec![10.0, 2.0, 8.0], vec![7.0, 5.0, 6.0]];

        let (assignments, total_cost) = hungarian_algorithm(&cost_matrix);

        assert_eq!(assignments, vec![Some(1), Some(2)]);
        assert_eq!(total_cost, 8.0);
    }

    #[test]
    fn test_hungarian_more_rows_than_cols() {
        let cost_matrix = vec![vec![8.0, 7.0], vec![2.0, 4.0], vec![5.0, 1.0]];

        let (assignments, total_cost) = hungarian_algorithm(&cost_matrix);

        assert_eq!(assignments, vec![None, Some(0), Some(1)]);
        assert_eq!(total_cost, 3.0);
    }
}
