use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Provide app_main stub for ESP-IDF
    embuild::espidf::sysenv::output();

    // Declare custom cfg conditions to avoid warnings
    println!("cargo:rustc-check-cfg=cfg(esp_idf_httpd_ws_support)");

    // Copy partition table to the esp-idf-sys output directory
    // This is needed because CONFIG_PARTITION_TABLE_CUSTOM_FILENAME is resolved
    // relative to the cmake build directory, not the source directory
    if let Ok(out_dir) = env::var("OUT_DIR") {
        let src = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
            .join("partitions.csv");
        let dst = PathBuf::from(&out_dir).join("partitions.csv");

        if src.exists() {
            if let Err(e) = fs::copy(&src, &dst) {
                println!("cargo:warning=Failed to copy partitions.csv: {}", e);
            } else {
                println!("cargo:rerun-if-changed=partitions.csv");
            }
        }
    }
}
