use auto_aim_rust::module::rbt_serial_async;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .init();
    if let Err(e) = rbt_serial_async::async_serial_example().await {
        tracing::error!("{}", e);
    }
}
