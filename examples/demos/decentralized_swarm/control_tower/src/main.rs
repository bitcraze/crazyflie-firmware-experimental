mod config;
mod protocol;
mod renderer;

use std::sync::{Arc, Mutex};
use std::time::Instant;

use protocol::{CopterFullState, CopterState, WandState, ALIVE_TIMEOUT_MS, MAX_COPTERS, WAND_TIMEOUT_MS};
use renderer::{Scene3DRenderer, TrailSegment, UnitPos, WandViz};
use tokio::sync::{mpsc, watch};


slint::include_modules!();

#[derive(Clone)]
struct RadioConfig {
    radio_index: usize,
    channel: u8,
    datarate: crazyradio::Datarate,
    address: [u8; 5],
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            radio_index: 0,
            channel: 80,
            datarate: crazyradio::Datarate::Dr2M,
            address: [0xff, 0xe7, 0xe7, 0xe7, 0xe7],
        }
    }
}

fn parse_address(s: &str) -> Result<[u8; 5], String> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    if s.len() != 10 {
        return Err(format!("Address must be 10 hex chars (5 bytes), got '{}'", s));
    }
    let bytes: Result<Vec<u8>, _> = (0..5).map(|i| u8::from_str_radix(&s[i*2..i*2+2], 16)).collect();
    let bytes = bytes.map_err(|e| format!("Invalid hex in address: {}", e))?;
    Ok([bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]])
}

fn parse_datarate(s: &str) -> Result<crazyradio::Datarate, String> {
    match s {
        "250K" | "250k" => Ok(crazyradio::Datarate::Dr250K),
        "1M" | "1m" => Ok(crazyradio::Datarate::Dr1M),
        "2M" | "2m" => Ok(crazyradio::Datarate::Dr2M),
        _ => Err(format!("Invalid datarate '{}'. Use 250K, 1M, or 2M", s)),
    }
}

fn parse_args() -> RadioConfig {
    let mut config = RadioConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--radio" | "-r" => {
                i += 1;
                config.radio_index = args.get(i)
                    .and_then(|s| s.parse().ok())
                    .expect("--radio requires a number (e.g. 0)");
            }
            "--channel" | "-c" => {
                i += 1;
                config.channel = args.get(i)
                    .and_then(|s| s.parse().ok())
                    .filter(|&c: &u8| c <= 125)
                    .expect("--channel requires a number 0-125");
            }
            "--datarate" | "-d" => {
                i += 1;
                config.datarate = args.get(i)
                    .map(|s| parse_datarate(s))
                    .expect("--datarate requires a value")
                    .expect("Invalid datarate");
            }
            "--address" | "-a" => {
                i += 1;
                config.address = args.get(i)
                    .map(|s| parse_address(s))
                    .expect("--address requires a value")
                    .expect("Invalid address");
            }
            "--help" | "-h" => {
                eprintln!("Usage: swarm-control-tower [OPTIONS]");
                eprintln!();
                eprintln!("Options:");
                eprintln!("  -r, --radio <N>        Radio index (default: 0)");
                eprintln!("  -c, --channel <N>      Radio channel 0-125 (default: 80)");
                eprintln!("  -d, --datarate <RATE>   250K, 1M, or 2M (default: 2M)");
                eprintln!("  -a, --address <HEX>    5-byte address in hex (default: E7E7E7E7E7)");
                eprintln!("  -h, --help             Show this help");
                std::process::exit(0);
            }
            other => {
                eprintln!("Unknown argument: {}. Use --help for usage.", other);
                std::process::exit(1);
            }
        }
        i += 1;
    }
    config
}

#[derive(Clone)]
struct CopterData {
    state: CopterFullState,
    last_seen: Instant,
    rssi_dbm: i16,
    trail: Vec<([f32; 3], [f32; 3])>, // (position, state_color)
    takeoff_pos: Option<[f32; 3]>,
    goto_history: Vec<([f32; 3], [f32; 3])>, // (goto_position, state_color)
}

impl Default for CopterData {
    fn default() -> Self {
        Self {
            state: CopterFullState {
                id: 0,
                counter: 0,
                state: CopterState::Unknown,
                battery_voltage: 0.0,
                timestamp: 0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
                goto_x: 0.0,
                goto_y: 0.0,
                goto_z: 0.0,
                desired_flying: 0,
                max_wand_grasped: 255,
            },
            last_seen: Instant::now() - std::time::Duration::from_secs(10),
            rssi_dbm: -100,
            trail: Vec::new(),
            takeoff_pos: None,
            goto_history: Vec::new(),
        }
    }
}

#[derive(Clone)]
struct WandData {
    state: WandState,
    last_seen: Instant,
}

impl Default for WandData {
    fn default() -> Self {
        Self {
            state: WandState { id: 0, x: 0.0, y: 0.0, z: 0.0, dx: 0.0, dy: 0.0, dz: 0.0 },
            last_seen: Instant::now() - std::time::Duration::from_secs(10),
        }
    }
}

struct SharedState {
    copters: [CopterData; MAX_COPTERS],
    radio_connected: bool,
    wand: WandData,
}

type SharedCopterState = Arc<Mutex<SharedState>>;

struct BroadcastCmd {
    desired_flying: u8,
    force_takeoff: bool,
    max_wand_grasped: u8,
}

fn main() {
    let radio_config = parse_args();

    // Bounds and lighthouse path supplied by build.rs (repo-specific)
    let active_area = Some(config::settings_active_area());
    let base_stations = config::load_lighthouse_geometry(
        env!("LIGHTHOUSE_YAML"), std::path::Path::new("."));

    slint::BackendSelector::new()
        .require_opengl_es()
        .select()
        .expect("Unable to select OpenGL ES backend");

    let app = AppWindow::new().unwrap();
    let app_weak = app.as_weak();

    // Persistent copter model — updated in-place so Slint keeps list items stable for clicks
    let copter_model = std::rc::Rc::new(slint::VecModel::<CopterInfo>::default());
    app.set_copters(copter_model.clone().into());

    // Shared state between radio thread and UI
    let shared_state: SharedCopterState = Arc::new(Mutex::new(SharedState {
        copters: std::array::from_fn(|_| CopterData::default()),
        radio_connected: false,
        wand: WandData::default(),
    }));

    // Command channel for desired_flying broadcasts
    let (cmd_tx, cmd_rx) = mpsc::unbounded_channel::<BroadcastCmd>();

    // Wire up control buttons
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_more(move || {
            if let Some(app) = app_ref.upgrade() {
                let current = app.get_desired_flying() as u8;
                let new = current.saturating_add(1).min(MAX_COPTERS as u8);
                eprintln!("[UI] More: {} -> {}", current, new);
                let _ = tx.send(BroadcastCmd { desired_flying: new, force_takeoff: app.get_force_takeoff(), max_wand_grasped: app.get_max_wand_grasped() as u8 });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_less(move || {
            if let Some(app) = app_ref.upgrade() {
                let current = app.get_desired_flying() as u8;
                let new = current.saturating_sub(1);
                eprintln!("[UI] Less: {} -> {}", current, new);
                let _ = tx.send(BroadcastCmd { desired_flying: new, force_takeoff: app.get_force_takeoff(), max_wand_grasped: app.get_max_wand_grasped() as u8 });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_all(move || {
            if let Some(app) = app_ref.upgrade() {
                eprintln!("[UI] All: {}", MAX_COPTERS);
                let _ = tx.send(BroadcastCmd { desired_flying: MAX_COPTERS as u8, force_takeoff: app.get_force_takeoff(), max_wand_grasped: app.get_max_wand_grasped() as u8 });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_none(move || {
            if let Some(app) = app_ref.upgrade() {
                eprintln!("[UI] None: 0");
                let _ = tx.send(BroadcastCmd { desired_flying: 0, force_takeoff: app.get_force_takeoff(), max_wand_grasped: app.get_max_wand_grasped() as u8 });
            }
        });
    }

    // Wire up max wand grasped buttons
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_more_wand(move || {
            if let Some(app) = app_ref.upgrade() {
                let current = app.get_max_wand_grasped() as u8;
                let new = if current == 255 { 255 } else { current.saturating_add(1) };
                eprintln!("[UI] WandMore: {} -> {}", current, new);
                let _ = tx.send(BroadcastCmd { desired_flying: app.get_desired_flying() as u8, force_takeoff: app.get_force_takeoff(), max_wand_grasped: new });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_less_wand(move || {
            if let Some(app) = app_ref.upgrade() {
                let current = app.get_max_wand_grasped() as u8;
                let new = if current == 255 { 10 } else { current.saturating_sub(1) };
                eprintln!("[UI] WandLess: {} -> {}", current, new);
                let _ = tx.send(BroadcastCmd { desired_flying: app.get_desired_flying() as u8, force_takeoff: app.get_force_takeoff(), max_wand_grasped: new });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_all_wand(move || {
            if let Some(app) = app_ref.upgrade() {
                eprintln!("[UI] WandAll: 255");
                let _ = tx.send(BroadcastCmd { desired_flying: app.get_desired_flying() as u8, force_takeoff: app.get_force_takeoff(), max_wand_grasped: 255 });
            }
        });
    }
    {
        let tx = cmd_tx.clone();
        let app_ref = app.as_weak();
        app.on_request_none_wand(move || {
            if let Some(app) = app_ref.upgrade() {
                eprintln!("[UI] WandNone: 0");
                let _ = tx.send(BroadcastCmd { desired_flying: app.get_desired_flying() as u8, force_takeoff: app.get_force_takeoff(), max_wand_grasped: 0 });
            }
        });
    }

    // Wire up clear-trail callback
    {
        let state = shared_state.clone();
        app.on_clear_trail(move |copter_id| {
            let id = copter_id as usize;
            if id < MAX_COPTERS {
                let mut guard = state.lock().unwrap();
                guard.copters[id].trail.clear();
                eprintln!("[UI] Cleared trail for CF#{}", id);
            }
        });
    }

    // Start radio sniffer thread
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let radio_state = shared_state.clone();
    let radio_thread = std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(radio_sniffer_task(radio_state, radio_config, cmd_rx, shutdown_rx));
    });

    // Set up rendering notifier
    let render_state = shared_state.clone();
    let copter_model_for_render = copter_model.clone();
    app.window()
        .set_rendering_notifier(move |state, graphics_api| {
            use std::cell::RefCell;
            thread_local! {
                static RENDERER: RefCell<Option<Scene3DRenderer>> = const { RefCell::new(None) };
                static LAST_FRAME: RefCell<Option<Instant>> = const { RefCell::new(None) };
            }

            match state {
                slint::RenderingState::RenderingSetup => {
                    let context = match graphics_api {
                        slint::GraphicsAPI::NativeOpenGL { get_proc_address } => unsafe {
                            glow::Context::from_loader_function_cstr(|s| get_proc_address(s))
                        },
                        _ => return,
                    };
                    RENDERER.with(|r| {
                        *r.borrow_mut() = Some(Scene3DRenderer::new(context));
                    });
                }
                slint::RenderingState::BeforeRendering => {
                    let Some(app) = app_weak.upgrade() else { return };
                    let mut renderer_ref = RENDERER.with(|r| r.borrow_mut().take());
                    let Some(renderer) = renderer_ref.as_mut() else { return };

                    let width = app.get_viewport_width() as u32;
                    let height = app.get_viewport_height() as u32;
                    let distance = app.get_cam_distance();
                    let pan_x = app.get_cam_pan_x();
                    let pan_y = app.get_cam_pan_y();

                    let now = Instant::now();

                    // Auto-rotate: advance yaw based on elapsed time
                    if app.get_auto_rotate() {
                        let dt = LAST_FRAME.with(|lf| {
                            let mut lf = lf.borrow_mut();
                            let dt = lf.map(|prev| now.duration_since(prev).as_secs_f32()).unwrap_or(0.0);
                            *lf = Some(now);
                            dt
                        });
                        const ROTATION_SPEED: f32 = 0.15; // radians per second
                        app.set_cam_yaw(app.get_cam_yaw() - ROTATION_SPEED * dt);
                    } else {
                        LAST_FRAME.with(|lf| *lf.borrow_mut() = None);
                    }
                    let yaw = app.get_cam_yaw();
                    let pitch = app.get_cam_pitch();
                    let (copters, radio_connected, wand_data) = {
                        let guard = render_state.lock().unwrap();
                        (guard.copters.clone(), guard.radio_connected, guard.wand.clone())
                    };

                    let selected_id = app.get_selected_copter();

                    // Build unit positions for alive copters
                    let mut unit_positions = Vec::new();
                    let mut copter_infos: Vec<CopterInfo> = Vec::new();
                    let mut cf_labels: Vec<VizLabel> = Vec::new();
                    let mut trail_segments: Vec<TrailSegment> = Vec::new();
                    let mut takeoff_markers: Vec<renderer::TakeoffMarker> = Vec::new();
                    let mut goto_history_points: Vec<([f32; 3], [f32; 3])> = Vec::new();

                    let mvp = renderer::compute_mvp(yaw, pitch, distance, pan_x, pan_y,
                        width as f32 / height.max(1) as f32);

                    for (i, cd) in copters.iter().enumerate() {
                        let alive = now.duration_since(cd.last_seen).as_millis() < ALIVE_TIMEOUT_MS as u128;
                        let is_selected = selected_id == i as i32;

                        if alive {
                            let color = cd.state.state.color();
                            // Include goto only for selected copter, when non-zero
                            let goto = if is_selected && (cd.state.goto_x != 0.0 || cd.state.goto_y != 0.0 || cd.state.goto_z != 0.0) {
                                Some([cd.state.goto_x, cd.state.goto_y, cd.state.goto_z])
                            } else {
                                None
                            };
                            unit_positions.push(UnitPos {
                                x: cd.state.x,
                                y: cd.state.y,
                                z: cd.state.z,
                                color,
                                highlighted: is_selected,
                                goto,
                            });

                            if let Some(tp) = cd.takeoff_pos {
                                takeoff_markers.push(renderer::TakeoffMarker {
                                    x: tp[0],
                                    y: tp[1],
                                    color,
                                });
                            }

                            if app.get_show_cf_labels() {
                                let sc = cd.state.state.color();
                                if let Some((sx, sy)) = renderer::project_to_screen(
                                    [cd.state.x, cd.state.y, cd.state.z],
                                    &mvp, width, height,
                                ) {
                                    cf_labels.push(VizLabel {
                                        text: format!("CF#{} {}", i, cd.state.state.name()).into(),
                                        screen_x: sx + 8.0,
                                        screen_y: sy - 12.0,
                                        label_color: slint::Color::from_rgb_u8(
                                            (sc[0] * 255.0) as u8,
                                            (sc[1] * 255.0) as u8,
                                            (sc[2] * 255.0) as u8,
                                        ),
                                    });
                                }
                            }
                        }

                        // Build trail segments for selected copter, grouped by color
                        if is_selected && cd.trail.len() >= 2 {
                            let mut seg_points = vec![cd.trail[0].0];
                            let mut seg_color = cd.trail[0].1;
                            for &(pos, color) in &cd.trail[1..] {
                                if color != seg_color {
                                    // Start new segment, overlapping last point for continuity
                                    if seg_points.len() >= 2 {
                                        trail_segments.push(TrailSegment {
                                            points: seg_points,
                                            color: seg_color,
                                        });
                                    }
                                    seg_points = vec![pos];
                                    seg_color = color;
                                } else {
                                    seg_points.push(pos);
                                }
                            }
                            if seg_points.len() >= 2 {
                                trail_segments.push(TrailSegment {
                                    points: seg_points,
                                    color: seg_color,
                                });
                            }
                        }

                        // Collect goto history for selected copter
                        if is_selected && app.get_show_goto_history() && !cd.goto_history.is_empty() {
                            goto_history_points = cd.goto_history.clone();
                        }

                        // Only show copters that have been seen at least once
                        if cd.state.timestamp > 0 || alive {
                            let sc = cd.state.state.color();
                            // RSSI to link quality: -90 dBm (0%) to -20 dBm (100%)
                            let link_quality = ((cd.rssi_dbm as f32 + 90.0) / 70.0).clamp(0.0, 1.0);
                            copter_infos.push(CopterInfo {
                                id: i as i32,
                                state_name: cd.state.state.name().into(),
                                battery_voltage: format!("{:.2}V", cd.state.battery_voltage).into(),
                                battery_level: ((cd.state.battery_voltage - 3.0) / 1.2).clamp(0.0, 1.0),
                                rssi_text: format!("{}dBm", cd.rssi_dbm).into(),
                                link_quality,
                                alive,
                                selected: is_selected,
                                color: slint::Color::from_rgb_u8(
                                    (sc[0] * 255.0) as u8,
                                    (sc[1] * 255.0) as u8,
                                    (sc[2] * 255.0) as u8,
                                ),
                            });
                        }
                    }

                    // Axis labels (X, Y, Z at end of each axis) - colors match axis lines
                    let mut axis_labels: Vec<VizLabel> = Vec::new();
                    if app.get_show_axis_labels() {
                        let axis_defs: [([f32; 3], &str, [f32; 3]); 3] = [
                            ([2.1, 0.0, 0.0], "X", [0.94, 0.27, 0.27]),  // red
                            ([0.0, 2.1, 0.0], "Y", [0.29, 0.85, 0.50]),  // green
                            ([0.0, 0.0, 2.1], "Z", [0.38, 0.65, 0.98]),  // blue
                        ];
                        for (pos, name, color) in &axis_defs {
                            if let Some((sx, sy)) = renderer::project_to_screen(
                                *pos, &mvp, width, height,
                            ) {
                                axis_labels.push(VizLabel {
                                    text: (*name).into(),
                                    screen_x: sx + 4.0,
                                    screen_y: sy - 8.0,
                                    label_color: slint::Color::from_rgb_u8(
                                        (color[0] * 255.0) as u8,
                                        (color[1] * 255.0) as u8,
                                        (color[2] * 255.0) as u8,
                                    ),
                                });
                            }
                        }
                    }

                    // Grid measurement labels along X and Y axes - color matches grid lines
                    let grid_color = slint::Color::from_rgb_u8(
                        (0.3 * 255.0) as u8,
                        (0.3 * 255.0) as u8,
                        (0.35 * 255.0) as u8,
                    );
                    let mut grid_labels: Vec<VizLabel> = Vec::new();
                    if app.get_show_grid_labels() {
                        let (gx_min, gx_max, gy_min, gy_max) = if let Some(aa) = active_area {
                            (aa.min_x.floor() as i32, aa.max_x.ceil() as i32,
                             aa.min_y.floor() as i32, aa.max_y.ceil() as i32)
                        } else {
                            (-5, 5, -5, 5)
                        };
                        for i in gx_min..=gx_max {
                            if i == 0 { continue; }
                            if let Some((sx, sy)) = renderer::project_to_screen(
                                [i as f32, 0.0, 0.0], &mvp, width, height,
                            ) {
                                grid_labels.push(VizLabel {
                                    text: format!("{}m", i).into(),
                                    screen_x: sx + 2.0,
                                    screen_y: sy + 4.0,
                                    label_color: grid_color,
                                });
                            }
                        }
                        for i in gy_min..=gy_max {
                            if i == 0 { continue; }
                            if let Some((sx, sy)) = renderer::project_to_screen(
                                [0.0, i as f32, 0.0], &mvp, width, height,
                            ) {
                                grid_labels.push(VizLabel {
                                    text: format!("{}m", i).into(),
                                    screen_x: sx + 2.0,
                                    screen_y: sy + 4.0,
                                    label_color: grid_color,
                                });
                            }
                        }
                    }

                    // Build wand visualization if recently seen
                    let wand_viz = if now.duration_since(wand_data.last_seen).as_millis() < WAND_TIMEOUT_MS as u128 {
                        let w = &wand_data.state;
                        let wpos = [w.x, w.y, w.z];
                        // Ray length = projection of the furthest copter goto onto the wand direction
                        let dir = [w.dx, w.dy, w.dz];
                        let dir_len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
                        let max_proj = if dir_len > 1e-6 {
                            copters.iter().filter_map(|cd| {
                                let alive = now.duration_since(cd.last_seen).as_millis() < ALIVE_TIMEOUT_MS as u128;
                                if !alive { return None; }
                                let g = [cd.state.goto_x, cd.state.goto_y, cd.state.goto_z];
                                if g[0] == 0.0 && g[1] == 0.0 && g[2] == 0.0 { return None; }
                                let dx = g[0] - wpos[0];
                                let dy = g[1] - wpos[1];
                                let dz = g[2] - wpos[2];
                                // Project (goto - wand_pos) onto wand direction
                                let proj = (dx * dir[0] + dy * dir[1] + dz * dir[2]) / dir_len;
                                if proj > 0.0 { Some(proj) } else { None }
                            }).fold(0.0_f32, f32::max)
                        } else {
                            0.0
                        };
                        let ray_len = if max_proj > 0.1 { max_proj } else { 3.0 };
                        let ndir = if dir_len > 1e-6 {
                            [dir[0] / dir_len, dir[1] / dir_len, dir[2] / dir_len]
                        } else {
                            [1.0, 0.0, 0.0]
                        };
                        Some(WandViz {
                            pos: wpos,
                            dir: ndir,
                            ray_len,
                        })
                    } else {
                        None
                    };

                    let show_base_stations = app.get_show_base_stations();
                    let show_axes = app.get_show_axes();
                    let show_grid = app.get_show_grid();
                    let show_fog = app.get_show_fog();
                    let texture = renderer.render(
                        width, height, yaw, pitch, distance, pan_x, pan_y,
                        &unit_positions, &trail_segments, &takeoff_markers, &goto_history_points,
                        active_area, if show_base_stations { &base_stations } else { &[] },
                        show_axes, show_grid, show_fog,
                        wand_viz.as_ref(),
                    );

                    app.set_texture(texture);
                    app.set_cf_labels(std::rc::Rc::new(slint::VecModel::from(cf_labels)).into());
                    app.set_axis_labels(std::rc::Rc::new(slint::VecModel::from(axis_labels)).into());
                    app.set_grid_labels(std::rc::Rc::new(slint::VecModel::from(grid_labels)).into());

                    // Update copter model in-place to preserve Slint list item state (click handlers)
                    {
                        use slint::Model;
                        let model = &copter_model_for_render;
                        // Update existing rows and add new ones
                        for (idx, info) in copter_infos.iter().enumerate() {
                            if idx < model.row_count() {
                                model.set_row_data(idx, info.clone());
                            } else {
                                model.push(info.clone());
                            }
                        }
                        // Remove excess rows
                        while model.row_count() > copter_infos.len() {
                            model.remove(model.row_count() - 1);
                        }
                    }
                    // Extract desired_flying and max_wand_grasped from any alive copter
                    let alive_copter = copters.iter()
                        .find(|cd| now.duration_since(cd.last_seen).as_millis() < ALIVE_TIMEOUT_MS as u128);
                    let desired = alive_copter.map(|cd| cd.state.desired_flying).unwrap_or(0);
                    let max_wand = alive_copter.map(|cd| cd.state.max_wand_grasped).unwrap_or(255);
                    app.set_desired_flying(desired as i32);
                    app.set_max_wand_grasped(max_wand as i32);

                    app.set_radio_status(if radio_connected {
                        "Receiving".into()
                    } else {
                        "Disconnected".into()
                    });

                    RENDERER.with(|r| {
                        *r.borrow_mut() = renderer_ref.take();
                    });

                    app.window().request_redraw();
                }
                slint::RenderingState::RenderingTeardown => {
                    RENDERER.with(|r| {
                        *r.borrow_mut() = None;
                    });
                }
                _ => {}
            }
        })
        .expect("Failed to set rendering notifier");

    app.run().unwrap();

    // Signal the radio task to stop and wait for it to exit sniffer mode cleanly
    let _ = shutdown_tx.send(true);
    radio_thread.join().ok();
}

async fn radio_sniffer_task(
    state: SharedCopterState,
    config: RadioConfig,
    mut cmd_rx: mpsc::UnboundedReceiver<BroadcastCmd>,
    mut shutdown: watch::Receiver<bool>,
) {
    loop {
        if *shutdown.borrow() { break; }

        eprintln!("Opening Crazyradio #{}...", config.radio_index);
        let cr = match crazyradio::Crazyradio::open_nth_async(config.radio_index).await {
            Ok(cr) => cr,
            Err(e) => {
                eprintln!("Failed to open Crazyradio: {:?}. Retrying in 2s...", e);
                tokio::select! {
                    _ = tokio::time::sleep(std::time::Duration::from_secs(2)) => {}
                    _ = shutdown.changed() => break,
                }
                continue;
            }
        };

        let res = run_sniffer(cr, &state, &config, &mut cmd_rx, &mut shutdown).await;
        state.lock().unwrap().radio_connected = false;
        if let Err(e) = res {
            eprintln!("Sniffer error: {:?}. Reconnecting in 2s...", e);
            tokio::select! {
                _ = tokio::time::sleep(std::time::Duration::from_secs(2)) => {}
                _ = shutdown.changed() => break,
            }
        }
    }
    eprintln!("Radio task stopped.");
}

async fn run_sniffer(
    mut cr: crazyradio::Crazyradio,
    state: &SharedCopterState,
    config: &RadioConfig,
    cmd_rx: &mut mpsc::UnboundedReceiver<BroadcastCmd>,
    shutdown: &mut watch::Receiver<bool>,
) -> Result<(), crazyradio::Error> {
    cr.set_channel(crazyradio::Channel::from_number(config.channel)?)?;
    cr.set_datarate(config.datarate)?;
    cr.set_address(&config.address)?;

    let rate_str = match config.datarate {
        crazyradio::Datarate::Dr250K => "250K",
        crazyradio::Datarate::Dr1M => "1M",
        crazyradio::Datarate::Dr2M => "2M",
    };
    eprintln!(
        "Entering sniffer mode on channel {}, {}, address {:02X?}...",
        config.channel, rate_str, config.address
    );
    let (receiver, sender) = cr.enter_sniffer_mode_async().await?;

    state.lock().unwrap().radio_connected = true;
    eprintln!("Sniffer mode active. Listening for swarm broadcasts...");

    let mut pkt_count: u64 = 0;
    let mut parsed_count: u64 = 0;
    let mut result: Result<(), crazyradio::Error> = Ok(());

    'sniffer: loop {
        tokio::select! {
            pkt_result = receiver.recv() => {
                match pkt_result {
                    Some(Ok(pkt)) => {
                        pkt_count += 1;
                        if pkt_count <= 10 || pkt_count % 100 == 0 {
                            eprintln!(
                                "[pkt #{}] pipe:{} rssi:{}dBm len:{} data:{:02X?}",
                                pkt_count, pkt.pipe, pkt.rssi_dbm, pkt.payload.len(), &pkt.payload
                            );
                        }
                        let rssi = pkt.rssi_dbm;
                        if let Some(copter) = protocol::parse_sniffer_payload(&pkt.payload) {
                            parsed_count += 1;
                            if parsed_count <= 10 || parsed_count % 100 == 0 {
                                eprintln!(
                                    "  -> CF#{} state={} pos=({:.2}, {:.2}, {:.2}) bat={:.2}V rssi={}dBm desired={} [parsed #{}]",
                                    copter.id, copter.state.name(),
                                    copter.x, copter.y, copter.z,
                                    copter.battery_voltage, rssi, copter.desired_flying, parsed_count
                                );
                            }
                            let id = copter.id as usize;
                            if id < MAX_COPTERS {
                                let mut guard = state.lock().unwrap();
                                let cd = &mut guard.copters[id];
                                // Record takeoff position and clear trail/goto history on takeoff
                                if copter.state == CopterState::TakingOff && cd.state.state != CopterState::TakingOff {
                                    cd.trail.clear();
                                    cd.goto_history.clear();
                                    cd.takeoff_pos = Some([copter.x, copter.y, copter.z]);
                                }
                                let pos = [copter.x, copter.y, copter.z];
                                let color = copter.state.color();
                                // Record goto point when it changes
                                let new_goto = [copter.goto_x, copter.goto_y, copter.goto_z];
                                let old_goto = [cd.state.goto_x, cd.state.goto_y, cd.state.goto_z];
                                if new_goto != old_goto && (new_goto[0] != 0.0 || new_goto[1] != 0.0 || new_goto[2] != 0.0) {
                                    cd.goto_history.push((new_goto, color));
                                }
                                if copter.state != CopterState::WaitForPositionLock {
                                    cd.trail.push((pos, color));
                                }
                                cd.state = copter;
                                cd.rssi_dbm = rssi;
                                cd.last_seen = Instant::now();
                            }
                        } else if let Some(wand) = protocol::parse_wand_payload(&pkt.payload) {
                            if pkt_count <= 10 || pkt_count % 100 == 0 {
                                eprintln!(
                                    "  -> Wand#{} pos=({:.2}, {:.2}, {:.2}) dir=({:.2}, {:.2}, {:.2})",
                                    wand.id, wand.x, wand.y, wand.z, wand.dx, wand.dy, wand.dz
                                );
                            }
                            let mut guard = state.lock().unwrap();
                            guard.wand.state = wand;
                            guard.wand.last_seen = Instant::now();
                        } else if pkt_count <= 10 {
                            eprintln!("  -> parse failed");
                        }
                    }
                    Some(Err(e)) => {
                        result = Err(e);
                        break 'sniffer;
                    }
                    None => {
                        break 'sniffer;
                    }
                }
            }
            Some(cmd) = cmd_rx.recv() => {
                let packet = protocol::build_control_packet(cmd.desired_flying, cmd.force_takeoff, cmd.max_wand_grasped);
                eprintln!("Broadcasting desired_flying={} force_takeoff={} len:{} data:{:02X?}",
                    cmd.desired_flying, cmd.force_takeoff, packet.len(), &packet);
                // Send burst for reliability (10 packets over ~1s)
                for i in 0..10 {
                    if let Err(e) = sender.send_broadcast(&config.address, &packet).await {
                        eprintln!("Broadcast error at attempt {}: {:?}", i, e);
                        break;
                    }
                    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                }
                eprintln!("Broadcast burst complete");
            }
            _ = shutdown.changed() => {
                break 'sniffer;
            }
        }
    }

    // Explicitly close so the RX thread calls exit_sniffer_mode() before we return.
    // Dropping receiver without closing races against process exit.
    drop(sender);
    let _ = receiver.close().await;
    eprintln!("Sniffer mode exited.");

    result
}
