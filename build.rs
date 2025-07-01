use std::env;
use std::io::Result;

fn main() -> Result<()> {
    let mut config = prost_build::Config::new();
    config.type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]");
    config.compile_protos(
        &[
            "proto-public-api/public_api_types.proto",
            "proto-public-api/public_api_up.proto",
            "proto-public-api/public_api_down.proto",
        ],
        &["proto-public-api/"],
    )?;

    // Export build time and git commit hash to project
    let out_dir = env::var("OUT_DIR").unwrap();
    let git_hash = std::process::Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
        .expect("Failed to execute git command")
        .stdout;
    let git_hash = std::str::from_utf8(&git_hash).expect("Failed to parse git hash");
    // Create file called hex_metadata.rs in OUT_DIR
    std::fs::write(
        format!("{}/hex_metadata.rs", out_dir),
        format!(
            r#"pub const BUILD_TIME: &str = "{}";
            pub const GIT_HASH: &str = "{}";
            "#,
            chrono::Local::now().to_rfc3339(),
            git_hash
        ),
    )?;
    Ok(())
}
