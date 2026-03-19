/// Decentralized swarm P2P protocol definitions.
///
/// Matches the C structs from:
/// crazyflie-firmware-experimental/examples/demos/decentralized_swarm/src/common_files/

pub const P2P_PORT: u8 = 5;
pub const WAND_P2P_PORT: u8 = 0x01;
pub const MAGIC_NUMBER: u32 = 0xbc471117;
pub const MAX_COPTERS: usize = 10;
pub const ALIVE_TIMEOUT_MS: u64 = 1000;
pub const WAND_TIMEOUT_MS: u64 = 1500;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CopterState {
    Idle = 0,
    WaitForPositionLock = 1,
    WaitForTakeOff = 2,
    QueuedForTakeOff = 3,
    PreparingForTakeOff = 4,
    TakingOff = 5,
    Hovering = 6,
    GoingToTrajectoryStart = 7,
    ExecutingTrajectory = 8,
    GoingToRandomPoint = 9,
    PreparingForLand = 10,
    GoingToPad = 11,
    WaitingAtPad = 12,
    Landing = 13,
    CheckCharging = 14,
    RepositionOnPad = 15,
    Crashed = 16,
    Sniffing = 17,
    WandGrasped = 18,
    WandReleased = 19,
    Unknown = 255,
}

impl CopterState {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::WaitForPositionLock,
            2 => Self::WaitForTakeOff,
            3 => Self::QueuedForTakeOff,
            4 => Self::PreparingForTakeOff,
            5 => Self::TakingOff,
            6 => Self::Hovering,
            7 => Self::GoingToTrajectoryStart,
            8 => Self::ExecutingTrajectory,
            9 => Self::GoingToRandomPoint,
            10 => Self::PreparingForLand,
            11 => Self::GoingToPad,
            12 => Self::WaitingAtPad,
            13 => Self::Landing,
            14 => Self::CheckCharging,
            15 => Self::RepositionOnPad,
            16 => Self::Crashed,
            17 => Self::Sniffing,
            18 => Self::WandGrasped,
            19 => Self::WandReleased,
            _ => Self::Unknown,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::WaitForPositionLock => "Wait Pos Lock",
            Self::WaitForTakeOff => "Wait Takeoff",
            Self::QueuedForTakeOff => "Queued",
            Self::PreparingForTakeOff => "Prep Takeoff",
            Self::TakingOff => "Taking Off",
            Self::Hovering => "Hovering",
            Self::GoingToTrajectoryStart => "To Traj Start",
            Self::ExecutingTrajectory => "Trajectory",
            Self::GoingToRandomPoint => "To Random Pt",
            Self::PreparingForLand => "Prep Land",
            Self::GoingToPad => "To Pad",
            Self::WaitingAtPad => "At Pad",
            Self::Landing => "Landing",
            Self::CheckCharging => "Check Charge",
            Self::RepositionOnPad => "Reposition",
            Self::Crashed => "CRASHED",
            Self::Sniffing => "Sniffer",
            Self::WandGrasped => "Wand Grasped",
            Self::WandReleased => "Wand Released",
            Self::Unknown => "Unknown",
        }
    }

    /// RGB color for this state.
    pub fn color(&self) -> [f32; 3] {
        match self {
            // Ground/idle states - gray/blue
            Self::Idle | Self::WaitForPositionLock => [0.5, 0.5, 0.5],
            Self::WaitForTakeOff | Self::QueuedForTakeOff => [0.3, 0.5, 0.8],
            Self::PreparingForTakeOff => [0.4, 0.7, 1.0],

            // Flying states - green shades
            Self::TakingOff => [0.2, 0.9, 0.4],
            Self::Hovering => [0.1, 0.8, 0.3],
            Self::GoingToTrajectoryStart => [0.0, 0.7, 0.7],
            Self::ExecutingTrajectory => [0.0, 0.9, 0.9],
            Self::GoingToRandomPoint => [0.3, 0.9, 0.5],

            // Landing states - orange/yellow
            Self::PreparingForLand => [1.0, 0.8, 0.0],
            Self::GoingToPad => [1.0, 0.65, 0.0],
            Self::WaitingAtPad => [0.8, 0.6, 0.0],
            Self::Landing => [1.0, 0.5, 0.0],

            // Charging states - purple
            Self::CheckCharging => [0.6, 0.3, 0.9],
            Self::RepositionOnPad => [0.7, 0.4, 0.8],

            // Error - red
            Self::Crashed => [1.0, 0.1, 0.1],

            // Wand states - cyan/teal
            Self::WandGrasped => [0.0, 0.9, 0.7],
            Self::WandReleased => [0.0, 0.6, 0.5],

            // Special
            Self::Sniffing => [0.3, 0.3, 0.3],
            Self::Unknown => [0.4, 0.4, 0.4],
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct CopterFullState {
    pub id: u8,
    pub counter: u8,
    pub state: CopterState,
    pub battery_voltage: f32,
    pub timestamp: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub goto_x: f32,
    pub goto_y: f32,
    pub goto_z: f32,
    pub desired_flying: u8,
    pub max_wand_grasped: u8,
}

/// Build a P2P broadcast packet to set desiredFlyingCopters and maxWandGrasped.
///
/// Format matches real firmware: [0xFF, 0x80|P2P_PORT, copter_message_t(44 bytes)]
/// copter_message_t is naturally aligned (no packed attribute).
pub fn build_control_packet(desired_flying: u8, force_takeoff: bool, max_wand_grasped: u8) -> Vec<u8> {
    let mut pkt = Vec::with_capacity(46);

    // ESB P2P header: 0xFF = CRTP P2P marker, 0x80 = P2P flag on port byte
    pkt.push(0xFF);
    pkt.push(0x80 | P2P_PORT);

    // copter_full_state_t (32 bytes)
    pkt.push(0);    // id
    pkt.push(0);    // counter
    pkt.push(CopterState::Sniffing as u8); // state
    pkt.push(0);    // battery_voltage (compressed)
    pkt.extend_from_slice(&0u32.to_le_bytes()); // timestamp
    // position x, y, z
    pkt.extend_from_slice(&0.0f32.to_le_bytes());
    pkt.extend_from_slice(&0.0f32.to_le_bytes());
    pkt.extend_from_slice(&0.0f32.to_le_bytes());
    // goto_position x, y, z
    pkt.extend_from_slice(&0.0f32.to_le_bytes());
    pkt.extend_from_slice(&0.0f32.to_le_bytes());
    pkt.extend_from_slice(&0.0f32.to_le_bytes());

    // ageOfControlDataMs (i32)
    pkt.extend_from_slice(&0i32.to_le_bytes());
    // isControlDataValid (u8)
    pkt.push(1);
    // desiredFlyingCopters (u8)
    pkt.push(desired_flying);
    // forceTakeoff (u8)
    pkt.push(if force_takeoff { 1 } else { 0 });
    // maxWandGrasped (u8)
    pkt.push(max_wand_grasped);
    // magicNumber (u32)
    pkt.extend_from_slice(&MAGIC_NUMBER.to_le_bytes());

    pkt
}

/// Parse a copter_message_t from sniffer payload.
///
/// Supports two over-the-air formats:
///
/// Real firmware: [0xFF, P2P_port_byte, copter_message_t...]
///   copter_message_t is naturally aligned (32 bytes), magic at offset 28.
///
/// Fake/sim swarm: [P2P_PORT, copter_message_t...]
///   copter_message_t is packed (30 bytes), magic at offset 26.
///
/// copter_full_state_t fields (first 20 or 32 bytes of message):
///   offset 0:  id (u8)
///   offset 1:  counter (u8)
///   offset 2:  state (u8)
///   offset 3:  battery_voltage (u8, compressed)
///   offset 4:  timestamp (u32 LE)
///   offset 8:  position.x (f32 LE)
///   offset 12: position.y (f32 LE)
///   offset 16: position.z (f32 LE)
///   offset 20: goto_position.x (f32 LE)  [if present]
///   offset 24: goto_position.y (f32 LE)  [if present]
///   offset 28: goto_position.z (f32 LE)  [if present]
pub fn parse_sniffer_payload(payload: &[u8]) -> Option<CopterFullState> {
    // Need at least 4 bytes for magic check
    if payload.len() < 4 {
        return None;
    }

    // Validate magic number at end of payload
    let magic_start = payload.len() - 4;
    let magic = u32::from_le_bytes([
        payload[magic_start],
        payload[magic_start + 1],
        payload[magic_start + 2],
        payload[magic_start + 3],
    ]);
    if magic != MAGIC_NUMBER {
        return None;
    }

    // Determine header length:
    // Real firmware: 0xFF marker + P2P port byte = 2 bytes
    // Fake/sim: single P2P_PORT byte = 1 byte
    let header_len = if payload[0] == 0xFF {
        2
    } else if payload[0] == P2P_PORT {
        1
    } else {
        0
    };

    let msg_data = &payload[header_len..];
    if msg_data.len() < 20 {
        return None;
    }

    let id = msg_data[0];
    if id as usize >= MAX_COPTERS {
        return None;
    }

    let compressed_voltage = msg_data[3];
    let battery_voltage = (compressed_voltage as f32 / 255.0) * (4.2 - 3.0) + 3.0;

    // goto_position present if message has at least 32 bytes of copter_full_state_t
    let has_goto = msg_data.len() >= 32;

    // desiredFlyingCopters and maxWandGrasped offsets depend on alignment:
    //   Aligned (44-byte msg): desiredFlyingCopters=37, forceTakeoff=38, maxWandGrasped=39, magic=40
    //   Packed  (30-byte msg): desiredFlyingCopters=25, magic=26 (no forceTakeoff/maxWandGrasped)
    let (desired_flying, max_wand_grasped) = if has_goto {
        // Naturally aligned
        let d = if msg_data.len() > 37 { msg_data[37] } else { 0 };
        let m = if msg_data.len() > 39 { msg_data[39] } else { 255 };
        (d, m)
    } else {
        // Packed: no maxWandGrasped field, default to unlimited
        let d = if msg_data.len() > 25 { msg_data[25] } else { 0 };
        (d, 255u8)
    };

    Some(CopterFullState {
        id,
        counter: msg_data[1],
        state: CopterState::from_u8(msg_data[2]),
        battery_voltage,
        timestamp: u32::from_le_bytes([msg_data[4], msg_data[5], msg_data[6], msg_data[7]]),
        x: f32::from_le_bytes([msg_data[8], msg_data[9], msg_data[10], msg_data[11]]),
        y: f32::from_le_bytes([msg_data[12], msg_data[13], msg_data[14], msg_data[15]]),
        z: f32::from_le_bytes([msg_data[16], msg_data[17], msg_data[18], msg_data[19]]),
        goto_x: if has_goto { f32::from_le_bytes([msg_data[20], msg_data[21], msg_data[22], msg_data[23]]) } else { 0.0 },
        goto_y: if has_goto { f32::from_le_bytes([msg_data[24], msg_data[25], msg_data[26], msg_data[27]]) } else { 0.0 },
        goto_z: if has_goto { f32::from_le_bytes([msg_data[28], msg_data[29], msg_data[30], msg_data[31]]) } else { 0.0 },
        desired_flying,
        max_wand_grasped,
    })
}

/// Wand line packet: position + direction vector.
///
/// Over the air: [0xFF, 0x80|WAND_P2P_PORT, id(u8), x,y,z,dx,dy,dz (6 x f32 LE)]
#[derive(Debug, Clone, Copy)]
pub struct WandState {
    pub id: u8,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub dx: f32,
    pub dy: f32,
    pub dz: f32,
}

/// Parse a wand line packet from sniffer payload.
pub fn parse_wand_payload(payload: &[u8]) -> Option<WandState> {
    // Real firmware: [0xFF, 0x80|port, data...]
    if payload.len() < 2 {
        return None;
    }

    let (port_byte, header_len) = if payload[0] == 0xFF && payload.len() >= 2 {
        (payload[1] & 0x0F, 2usize)
    } else {
        (payload[0], 1usize)
    };

    if port_byte != WAND_P2P_PORT {
        return None;
    }

    let data = &payload[header_len..];
    // WandLinePacket: 1 byte id + 6 floats = 25 bytes
    if data.len() < 25 {
        return None;
    }

    let f = |off: usize| f32::from_le_bytes([data[off], data[off+1], data[off+2], data[off+3]]);

    Some(WandState {
        id: data[0],
        x: f(1),
        y: f(5),
        z: f(9),
        dx: f(13),
        dy: f(17),
        dz: f(21),
    })
}
