/// Commands that can be sent to an Easycom-compatible device.
///
/// All angle values are in whole degrees.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Command {
    /// Rotate to the given azimuth (0–360°).
    Azimuth(u16),
    /// Rotate to the given elevation (0–180°).
    Elevation(u16),
    /// Rotate to the given azimuth and elevation simultaneously.
    AzimuthElevation { az: u16, el: u16 },
    /// Query the current azimuth and elevation position.
    QueryPosition,
    /// Stop all movement immediately.
    Stop,
    /// Move by a relative offset from the current position.
    Offset { az: i16, el: i16 },
    /// Send a keep-alive ping (`?`). The device responds with an Ack.
    KeepAlive,
}

/// Responses returned by an Easycom-compatible device.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Response {
    /// The device acknowledged the command without extra data.
    Ack,
    /// The current position reported by the device.
    Position { az: u16, el: u16 },
    /// The device reported an error (returned `?`).
    Error,
}
