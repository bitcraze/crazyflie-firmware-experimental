use std::collections::BTreeMap;
use std::path::Path;

#[derive(Clone, Copy)]
pub struct ActiveArea {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
}

#[derive(Clone, Copy)]
pub struct BaseStation {
    pub origin: [f32; 3],
}

/// Active area bounds parsed from settings.h at build time.
pub fn settings_active_area() -> ActiveArea {
    ActiveArea {
        min_x: env!("SETTINGS_MIN_X_BOUND").parse().expect("invalid MIN_X_BOUND"),
        max_x: env!("SETTINGS_MAX_X_BOUND").parse().expect("invalid MAX_X_BOUND"),
        min_y: env!("SETTINGS_MIN_Y_BOUND").parse().expect("invalid MIN_Y_BOUND"),
        max_y: env!("SETTINGS_MAX_Y_BOUND").parse().expect("invalid MAX_Y_BOUND"),
    }
}

/// Scan `config_dir` for any YAML file that contains a `geos:` lighthouse section.
/// Returns the path if exactly one match is found; prints an error and returns None if multiple are found.
pub fn find_lighthouse_yaml(config_dir: &Path) -> Option<std::path::PathBuf> {
    let dir = std::fs::read_dir(config_dir).ok()?;
    let mut candidates: Vec<std::path::PathBuf> = dir
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("yaml"))
        .collect();
    candidates.sort(); // deterministic order
    let matches: Vec<std::path::PathBuf> = candidates.into_iter()
        .filter(|p| std::fs::read_to_string(p).map(|t| t.contains("geos:")).unwrap_or(false))
        .collect();
    match matches.len() {
        0 => None,
        1 => Some(matches.into_iter().next().unwrap()),
        _ => {
            eprintln!("ERROR: Multiple lighthouse YAML files found in '{}'. Please keep only one:", config_dir.display());
            for p in &matches {
                eprintln!("  - {}", p.display());
            }
            None
        }
    }
}

pub fn load_lighthouse_geometry(yaml_path: &str, config_dir: &Path) -> Vec<BaseStation> {
    let full_path = if Path::new(yaml_path).is_absolute() {
        yaml_path.to_string()
    } else {
        config_dir.join(yaml_path).to_string_lossy().to_string()
    };

    let text = match std::fs::read_to_string(&full_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("Failed to read lighthouse geometry {}: {}", full_path, e);
            return Vec::new();
        }
    };

    #[derive(serde::Deserialize)]
    struct GeoEntry {
        origin: [f64; 3],
    }

    #[derive(serde::Deserialize)]
    struct LighthouseFile {
        #[serde(default)]
        geos: BTreeMap<u32, GeoEntry>,
    }

    match serde_yaml::from_str::<LighthouseFile>(&text) {
        Ok(lh) => {
            let stations: Vec<BaseStation> = lh.geos.values().map(|g| {
                BaseStation {
                    origin: [g.origin[0] as f32, g.origin[1] as f32, g.origin[2] as f32],
                }
            }).collect();
            eprintln!("Loaded {} lighthouse base stations from {}", stations.len(), full_path);
            stations
        }
        Err(e) => {
            eprintln!("Failed to parse lighthouse geometry {}: {}", full_path, e);
            Vec::new()
        }
    }
}
