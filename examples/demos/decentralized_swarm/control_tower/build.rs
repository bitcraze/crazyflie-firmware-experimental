fn main() {
    slint_build::compile("ui/app.slint").expect("Slint build failed");

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let settings_path = std::path::Path::new(&manifest_dir)
        .join("../src/common_files/settings.h");

    println!("cargo:rerun-if-changed={}", settings_path.display());

    let text = std::fs::read_to_string(&settings_path)
        .expect("Failed to read ../src/common_files/settings.h");

    println!("cargo:rustc-env=LIGHTHOUSE_DIR=../config");

    for (define, env_key) in &[
        ("MIN_X_BOUND", "SETTINGS_MIN_X_BOUND"),
        ("MAX_X_BOUND", "SETTINGS_MAX_X_BOUND"),
        ("MIN_Y_BOUND", "SETTINGS_MIN_Y_BOUND"),
        ("MAX_Y_BOUND", "SETTINGS_MAX_Y_BOUND"),
    ] {
        let value = parse_define(&text, define)
            .unwrap_or_else(|| panic!("Could not find {} in settings.h", define));
        println!("cargo:rustc-env={}={}", env_key, value);
    }
}

fn parse_define(text: &str, name: &str) -> Option<String> {
    let prefix = format!("#define {} ", name);
    for line in text.lines() {
        if let Some(rest) = line.trim().strip_prefix(&prefix) {
            // Strip trailing C float suffix 'f'
            return Some(rest.trim().trim_end_matches('f').to_string());
        }
    }
    None
}
