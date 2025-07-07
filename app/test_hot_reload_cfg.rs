use auto_aim_rust::rbt_err::RbtResult;
use auto_aim_rust::rbt_infra::rbt_cfg::RbtCfg;
use auto_aim_rust::rbt_mod::rbt_threads::rbt_cfg_thread::rbt_cfg_thread;
use auto_aim_rust::rbt_infra::rbt_log::logger_init;

#[tokio::main]
async fn main() -> RbtResult<()> { 
    let cfg = RbtCfg::from_toml().unwrap_or_default();
    let _logger_guard = logger_init().await?;

    let (boardcast_tx, boardcast_rx) = tokio::sync::broadcast::channel::<()>(1);
    let (cfg_watch_tx, cfg_watch_rx) = tokio::sync::watch::channel(cfg.clone());
    let rbt_cfg_thread_handle = rbt_cfg_thread(boardcast_rx, cfg_watch_tx, cfg);

    tokio::join!(rbt_cfg_thread_handle);

    let exit_signal = tokio::signal::ctrl_c().await.unwrap();
    boardcast_tx.send(()).unwrap();

    tokio::time::sleep(tokio::time::Duration::from_secs(3)).await;
    println!("exit");

    Ok(())
}
