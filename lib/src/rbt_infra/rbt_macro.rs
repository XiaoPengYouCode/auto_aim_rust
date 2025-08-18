/// 该宏用于将生成一个标准字符串错误
/// 并同时记录 log 信息
#[macro_export]
macro_rules! rbt_bail_error {
    ($err: expr) => {
        let e = $err;
        tracing::error!("{}", $err);
        return Err(e.into());
    };
}
