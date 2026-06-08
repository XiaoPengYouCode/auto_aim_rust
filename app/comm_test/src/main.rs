mod serial_test;
mod udp_test;
mod usb_test;

/// 用于测试电控通讯
use lib::rbt_mod::rbt_comm::rbt_comm_frame;
use log::info;

#[tokio::main]
async fn main() {
    logforth::starter_log::stdout().apply();

    // 接收线程
    let sens_handle = tokio::spawn(async move {
        // 2ms 定时器
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
