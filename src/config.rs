use std::path::Path;

#[derive(serde::Deserialize, Debug)]
pub struct RobotConfig {
    pub armor_detect_model_path: String,
    pub armor_detect_engine_path: String,
    pub buff_detect_model_path: String,
    pub log_filter: String,
    pub img_dbg_open: bool,
}

impl RobotConfig {
    pub async fn read_from_toml() -> Result<Self, Box<dyn std::error::Error>> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("robot_config.toml");
        tracing::info!(?path, "loaded config");
        let param = tokio::fs::read_to_string(path).await?;
        let param: Self = toml::from_str(&param)?;
        Ok(param)
    }
}
