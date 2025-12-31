fn main() {
    // Provide app_main stub for ESP-IDF
    embuild::espidf::sysenv::output();

    // Declare custom cfg conditions to avoid warnings
    println!("cargo:rustc-check-cfg=cfg(esp_idf_httpd_ws_support)");
}
