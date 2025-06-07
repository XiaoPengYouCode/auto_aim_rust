use std::path::Path;

const config_file_path: &str = "/config/robot_config.toml";

#[derive(serde::Deserialize, Debug)]

pub struct RobotConfig {
    armor_detect_model_path: String,
    armor_num_classify_model_path: String,
    buff_detect_model_path: String,
}

impl RobotConfig {
    pub fn read_from_toml() -> Result<Self, Box<dyn std::error::Error>> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(config_file_path);
        let param = std::fs::read_to_string(path)?;
        let param: Self = toml::from_str(&param)?;
        Ok(param)
    }
}
