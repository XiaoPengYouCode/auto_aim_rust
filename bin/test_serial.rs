use auto_aim_rust::rbt_mod::rbt_comm;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .init();
    if let Err(e) = rbt_comm::async_serial_example().await {
        tracing::error!("{}", e);
    }
}
