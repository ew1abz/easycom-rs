/// Commands that can be sent to an Easycom-compatible device.
///
/// All angle values are in whole degrees. Speed values are in
/// millidegrees per second.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Command {
    /// Rotate to the given azimuth (0–360°).
    Azimuth(u16),
    /// Rotate to the given elevation (0–180°).
    Elevation(u16),
    /// Rotate to the given azimuth and elevation simultaneously.
    AzimuthElevation { az: u16, el: u16 },
    /// Query the current azimuth and elevation position (GS-232: `C`).
    QueryPosition,
    /// Query the current azimuth only (Easycomm II: `AZ`).
    QueryAzimuth,
    /// Query the current elevation only (Easycomm II: `EL`).
    QueryElevation,
    /// Query the current device status (GS-232: `GS`, returns [`Response::Status`]).
    QueryStatus,
    /// Stop all movement immediately.
    Stop,
    /// Move by a relative offset from the current position.
    Offset { az: i16, el: i16 },
    /// Send a keep-alive ping (`?`). The device responds with an Ack.
    KeepAlive,
    /// Velocity left — rotate azimuth in the decreasing direction (Easycomm III: `VL`).
    VelocityLeft(u32),
    /// Velocity right — rotate azimuth in the increasing direction (Easycomm III: `VR`).
    VelocityRight(u32),
    /// Velocity up — rotate elevation upward (Easycomm III: `VU`).
    VelocityUp(u32),
    /// Velocity down — rotate elevation downward (Easycomm III: `VD`).
    VelocityDown(u32),
    /// Get the status register bitmask (Easycomm III: `GS`).
    GetStatusRegister,
    /// Get the error register bitmask (Easycomm III: `GE`).
    GetErrorRegister,
    /// Read a configuration register (Easycomm III: `CR<reg>`).
    ReadConfig(u16),
    /// Write a configuration register (Easycomm III: `CW<reg>,<value>`).
    WriteConfig { register: u16, value: i32 },
    /// Reset the device (Easycomm III: `RESET`).
    Reset,
    /// Move to the configured park position (Easycomm III: `PARK`).
    Park,
}

/// Responses returned by an Easycom-compatible device.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Response {
    /// The device acknowledged the command without extra data.
    Ack,
    /// The current position reported by the device (both axes).
    Position { az: u16, el: u16 },
    /// The current azimuth only (Easycomm II: `AZ=NNN.N`).
    AzimuthPosition(u16),
    /// The current elevation only (Easycomm II: `EL=NNN.N`).
    ElevationPosition(u16),
    /// The current device status (GS-232: `ST=<status>`).
    Status(DeviceStatus),
    /// Status register bitmask (Easycomm III: `GS<value>`).
    StatusRegister(u16),
    /// Error register bitmask (Easycomm III: `GE<value>`).
    ErrorRegister(u16),
    /// Configuration register value (Easycomm III: `CR<reg>,<value>`).
    ConfigValue { register: u16, value: i32 },
    /// The device reported an error (returned `?`).
    Error,
}

/// Status codes reported by the device in response to a [`Command::QueryStatus`].
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DeviceStatus {
    /// Homed and idle, ready for commands.
    Idle,
    /// Currently moving to a target position.
    Moving,
    /// Performing homing / calibration sequence.
    Homing,
    /// Power-on state, homing not yet performed.
    NotHomed,
    /// Azimuth homing failed.
    HomingAzError,
    /// Elevation homing failed.
    HomingElError,
}
