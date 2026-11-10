fn main() {
    println!("cargo::rustc-check-cfg=cfg(esp_idf_comp_espressif__mdns_enabled)");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("espidf") {
        embuild::espidf::sysenv::output();
    }
}
