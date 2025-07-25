use std::path::PathBuf;

#[derive(Debug)]
pub struct Config {
    pub policy_scale: f64,
    pub kp_scale: f64,
    pub kd_scale: f64,
    pub log_path: PathBuf,
}
