// 定义了许多有用的宏

/// 该宏用于将生成一个标准字符串错误
/// 所有的错误都应该被记录，所以将log信息封装了进来
#[macro_export]
macro_rules! rbt_bail_error {
    ($err: expr) => {
        let e = $err;
        tracing::error!("{}", $err);
        return Err(e.into());
    };
}
