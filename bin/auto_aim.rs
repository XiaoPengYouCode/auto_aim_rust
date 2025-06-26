use auto_aim_rust::rbt_app::single_thread::RbtSingleThreadApp;
use auto_aim_rust::rbt_err::RbtResult;

#[tokio::main]
async fn main() -> RbtResult<()> {
    let app = RbtSingleThreadApp::new().await?;
    app.run()?;

    Ok(())
}
