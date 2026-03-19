# Swarm Control Tower

A real-time 3D visualization and control application for [Crazyflie](https://www.bitcraze.io/products/crazyflie-2-1/) drone swarms. It uses a [Crazyradio](https://www.bitcraze.io/products/crazyradio-2-0/) in sniffer mode to listen to P2P broadcast packets from a decentralized swarm and displays each drone's position, state, battery level, and signal strength in an interactive 3D view.

![Rust](https://img.shields.io/badge/language-Rust-orange)

## Features

- **3D Visualization** -- Interactive OpenGL viewport showing drone positions, flight trails, goto targets, and drop lines for depth perception
- **Swarm Monitoring** -- Live state, battery voltage, and RSSI for up to 10 drones
- **Flight Control** -- Buttons to set the number of drones that should fly (+, -, All, None) with a force-takeoff override
- **Lighthouse Integration** -- Displays Lighthouse base station positions loaded from calibration geometry
- **Configurable Camera** -- Orbit, pan, and zoom with mouse controls
- **Wand Support** -- Visualizes wand position and pointing direction from wand P2P packets

## Prerequisites

- Rust toolchain (edition 2021)
- A Crazyradio USB dongle
- OpenGL ES 2.0 capable system
- USB permissions for the Crazyradio (on Linux, udev rules)

## Setup

### 1. Install Rust

If you don't have Rust installed:

```bash
curl --proto '=https' --tlsv8 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### 2. USB permissions (Linux)

Allow your user to access the Crazyradio without root. You only need to do this once:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-crazyradio.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug and replug the Crazyradio dongle.


### 3. Crazyradio firmware

Requires [Crazyradio 2.0 firmware 5.4 or later](https://github.com/bitcraze/crazyradio2-firmware/releases/tag/5.4).

Follow the [getting started guide](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/#enter-bootloader-mode) to flash the firmware to your Crazyradio 2.0.


### 4. Build

```bash
cargo build --release
```

### 5. Run

```bash
# Default: channel 80, 2M datarate, address E7E7E7E7E7
cargo run --release

# Custom radio settings
cargo run --release -- --radio 0 --channel 90 --address FFE7E7E7E7
```

### Command-line options

| Option | Short | Default | Description |
|---|---|---|---|
| `--radio <N>` | `-r` | `0` | Crazyradio device index |
| `--channel <N>` | `-c` | `80` | Radio channel (0-125) |
| `--datarate <RATE>` | `-d` | `2M` | Data rate: `250K`, `1M`, or `2M` |
| `--address <HEX>` | `-a` | `E7E7E7E7E7` | 5-byte radio address in hex |

## Configuration

### config.toml

The runtime configuration file defines the Lighthouse geometry file and the active flight area boundaries:

```toml
lighthouse_geometry = "Lighthouse_Cage.yaml"

[active_area]
min_x = -1.36
max_x = 1.72
min_y = -1.66
max_y = 1.35
```

### Lighthouse geometry

The `Lighthouse_Cage.yaml` file contains Lighthouse base station calibration data including sweep parameters and origin positions. This is the same format used by the Crazyflie Python client.

## Controls

### Left panel

- **+** / **-** -- Increase or decrease the number of drones that should fly
- **All** / **None** -- Request all or no drones to fly
- **Force takeoff** -- Override low-battery takeoff protection
- Click a drone in the list to select it and show its flight trail

### 3D viewport

- **Left-drag** -- Orbit camera (yaw/pitch)
- **Right-drag** -- Pan camera
- **Scroll wheel** -- Zoom in/out
- **Right-click** -- Context menu with display toggles (labels, grid, axes, Lighthouse stations)

## Architecture

| Module | Purpose |
|---|---|
| `main.rs` | Application entry point, UI setup, radio thread, frame updates |
| `protocol.rs` | P2P packet parsing/building matching the decentralized swarm firmware protocol |
| `renderer.rs` | OpenGL 3D rendering engine (ground grid, drones, trails, base stations) |
| `config.rs` | TOML/YAML configuration loading |
| `ui/app.slint` | UI layout using the [Slint](https://slint.dev/) framework |

The application runs two threads: the main thread drives the Slint UI event loop, while a background Tokio async task handles Crazyradio sniffer I/O and packet parsing. They communicate through `Arc<Mutex<SharedState>>`.
