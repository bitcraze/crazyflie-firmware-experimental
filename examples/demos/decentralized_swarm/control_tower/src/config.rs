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
