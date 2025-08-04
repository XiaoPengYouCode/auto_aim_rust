/// 用于测试电控通讯
use lib::rbt_infra::rbt_comm;
use tracing::info;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .init();

    // 接收线程
    let sens_handle = tokio::spawn(async move {
        // 10ms 定时器
        let mut ticker = tokio::time::interval(tokio::time::Duration::from_millis(10));
        loop {
            ticker.tick().await;
            info!("received");
        }
    });

    // 发送线程
    let ctrl_handle = tokio::spawn(async move {
        // 10ms 定时器
        let mut ticker = tokio::time::interval(tokio::time::Duration::from_millis(10));
        loop {
            ticker.tick().await;
            info!("sended");
        }
    });

    tokio::join!(sens_handle, ctrl_handle,);

    info!("main task exiting");
}
