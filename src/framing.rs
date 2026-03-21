//! Frame encoding and decoding for the Easycom protocol.
//!
//! Supports GS-232A/B, Easycomm II, and Easycomm III command formats:
//!
//! GS-232 commands:
//! - `A<NNN>\r`  — set azimuth, 3 zero-padded decimal digits
//! - `E<NNN>\r`  — set elevation, 3 zero-padded decimal digits
//! - `W<NNN> <NNN>\r` — set azimuth and elevation (space-separated)
//! - `C\r`       — query current position (both axes)
//! - `S\r`       — stop
//!
//! Easycomm II commands:
//! - `AZ\r`      — query azimuth
//! - `EL\r`      — query elevation
//! - `AZnnn.n\r` — set azimuth (decimal degrees)
//! - `ELnnn.n\r` — set elevation (decimal degrees)
//! - `AZnnn.n ELnnn.n\r` — set both axes
//! - `SA SE\r`   — stop all axes
//!
//! Easycomm III commands:
//! - `VL<speed>\r` / `VR<speed>\r` — velocity left/right (mdeg/s)
//! - `VU<speed>\r` / `VD<speed>\r` — velocity up/down (mdeg/s)
//! - `GS\r`      — get status register (bitmask)
//! - `GE\r`      — get error register (bitmask)
//! - `CR<reg>\r` — read configuration register
//! - `CW<reg>,<val>\r` — write configuration register
//! - `RESET\r`   — reset device
//! - `PARK\r`    — move to park position
//!
//! An optional XOR checksum may be appended as `*XX` (ASCII hex) before `\r`.
//!
//! Response formats:
//! - `AZ=NNN EL=NNN\r` — position response to a `C` query
//! - `AZ=NNN.N\r`      — azimuth-only response (Easycomm II)
//! - `EL=NNN.N\r`      — elevation-only response (Easycomm II)
//! - `ST=<status>\r`   — status response (`idle`, `moving`, `homing`,
//!   `not_homed`, `homing_az_error`, `homing_el_error`)
//! - `GS<val>\n`       — status register bitmask (Easycomm III)
//! - `GE<val>\n`       — error register bitmask (Easycomm III)
//! - `CR<reg>,<val>\n` — configuration register value (Easycomm III)
//! - `+\r` or an empty `\r` — acknowledgement
//! - `?\r` — device error

use crate::command::{Command, DeviceStatus, Response};
use crate::error::ParseError;

// ── Encoding ─────────────────────────────────────────────────────────────────

/// Compute the XOR checksum of `data`.
fn xor_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// Write a two-hex-digit checksum suffix `*XX` into `buf` starting at `pos`.
/// Returns the new position after the suffix.
fn write_checksum(buf: &mut [u8], pos: usize, checksum: u8) -> usize {
    const HEX: &[u8] = b"0123456789ABCDEF";
    buf[pos] = b'*';
    buf[pos + 1] = HEX[(checksum >> 4) as usize];
    buf[pos + 2] = HEX[(checksum & 0x0F) as usize];
    pos + 3
}

/// Encode a [`Command`] into a byte frame and return it as a `Vec<u8>`.
///
/// The returned bytes include the trailing `\r` terminator.
///
/// # Errors
/// Returns [`crate::error::Error::InvalidParam`] if an angle value is out of
/// the valid range (azimuth 0–360, elevation 0–180).
#[cfg(feature = "std")]
pub fn encode(cmd: &Command) -> Result<Vec<u8>, crate::error::Error<core::convert::Infallible>> {
    use crate::error::Error;

    // Maximum frame length: "W360 180*XX\r" = 13 bytes — a 32-byte buf is safe.
    let mut buf = [0u8; 32];
    let len = encode_into(cmd, &mut buf).map_err(Error::InvalidParam)?;
    Ok(buf[..len].to_vec())
}

/// Encode a [`Command`] into `buf` (no-std / fixed-buffer variant).
///
/// Returns the number of bytes written, or an error message if a parameter is
/// out of range.
pub fn encode_into(cmd: &Command, buf: &mut [u8]) -> Result<usize, &'static str> {
    match cmd {
        Command::Azimuth(az) => {
            if *az > 360 {
                return Err("azimuth must be 0–360");
            }
            let payload = format_cmd_3digit(b'A', *az, buf);
            Ok(payload)
        }
        Command::Elevation(el) => {
            if *el > 180 {
                return Err("elevation must be 0–180");
            }
            let payload = format_cmd_3digit(b'E', *el, buf);
            Ok(payload)
        }
        Command::AzimuthElevation { az, el } => {
            if *az > 360 {
                return Err("azimuth must be 0–360");
            }
            if *el > 180 {
                return Err("elevation must be 0–180");
            }
            // "WAZZ ELL\r"
            buf[0] = b'W';
            write_3digits(buf, 1, *az);
            buf[4] = b' ';
            write_3digits(buf, 5, *el);
            buf[8] = b'\r';
            Ok(9)
        }
        Command::QueryPosition => {
            buf[0] = b'C';
            buf[1] = b'\r';
            Ok(2)
        }
        Command::QueryAzimuth => {
            buf[0] = b'A';
            buf[1] = b'Z';
            buf[2] = b'\r';
            Ok(3)
        }
        Command::QueryElevation => {
            buf[0] = b'E';
            buf[1] = b'L';
            buf[2] = b'\r';
            Ok(3)
        }
        Command::QueryStatus => {
            buf[0] = b'G';
            buf[1] = b'S';
            buf[2] = b'\r';
            Ok(3)
        }
        Command::Stop => {
            buf[0] = b'S';
            buf[1] = b'\r';
            Ok(2)
        }
        Command::KeepAlive => {
            buf[0] = b'?';
            buf[1] = b'\r';
            Ok(2)
        }
        Command::Offset { az, el } => {
            // "O+AZZ+ELL\r" — sign-prefixed 3-digit values
            buf[0] = b'O';
            let pos = write_signed_3digits(buf, 1, *az);
            let pos = write_signed_3digits(buf, pos, *el);
            buf[pos] = b'\r';
            Ok(pos + 1)
        }
        Command::VelocityLeft(speed) => write_velocity_cmd(buf, b"VL", *speed),
        Command::VelocityRight(speed) => write_velocity_cmd(buf, b"VR", *speed),
        Command::VelocityUp(speed) => write_velocity_cmd(buf, b"VU", *speed),
        Command::VelocityDown(speed) => write_velocity_cmd(buf, b"VD", *speed),
        Command::GetStatusRegister => {
            buf[..2].copy_from_slice(b"GS");
            buf[2] = b'\r';
            Ok(3)
        }
        Command::GetErrorRegister => {
            buf[..2].copy_from_slice(b"GE");
            buf[2] = b'\r';
            Ok(3)
        }
        Command::ReadConfig(reg) => {
            buf[..2].copy_from_slice(b"CR");
            let pos = write_u32(buf, 2, *reg as u32);
            buf[pos] = b'\r';
            Ok(pos + 1)
        }
        Command::WriteConfig { register, value } => {
            buf[..2].copy_from_slice(b"CW");
            let pos = write_u32(buf, 2, *register as u32);
            buf[pos] = b',';
            let pos = write_i32(buf, pos + 1, *value);
            buf[pos] = b'\r';
            Ok(pos + 1)
        }
        Command::Reset => {
            buf[..5].copy_from_slice(b"RESET");
            buf[5] = b'\r';
            Ok(6)
        }
        Command::Park => {
            buf[..4].copy_from_slice(b"PARK");
            buf[4] = b'\r';
            Ok(5)
        }
    }
}

/// Encode a [`Command`] with an XOR checksum into `buf`.
///
/// Returns the number of bytes written, or an error message if a parameter is
/// out of range.
pub fn encode_with_checksum(cmd: &Command, buf: &mut [u8]) -> Result<usize, &'static str> {
    // Build the payload (everything before the terminator).
    let payload_end = encode_into(cmd, buf)? - 1; // exclude the trailing \r
    let checksum = xor_checksum(&buf[..payload_end]);
    let pos = write_checksum(buf, payload_end, checksum);
    buf[pos] = b'\r';
    Ok(pos + 1)
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Write a zero-padded 3-digit decimal number at `buf[pos..]`. Returns `pos + 3`.
fn write_3digits(buf: &mut [u8], pos: usize, n: u16) -> usize {
    buf[pos] = b'0' + ((n / 100) % 10) as u8;
    buf[pos + 1] = b'0' + ((n / 10) % 10) as u8;
    buf[pos + 2] = b'0' + (n % 10) as u8;
    pos + 3
}

/// Write a sign character followed by a zero-padded 3-digit decimal. Returns new pos.
fn write_signed_3digits(buf: &mut [u8], pos: usize, n: i16) -> usize {
    buf[pos] = if n >= 0 { b'+' } else { b'-' };
    write_3digits(buf, pos + 1, n.unsigned_abs())
}

/// Write `<letter><NNN>\r` and return total length.
fn format_cmd_3digit(letter: u8, value: u16, buf: &mut [u8]) -> usize {
    buf[0] = letter;
    write_3digits(buf, 1, value);
    buf[4] = b'\r';
    5
}

/// Write an unsigned 32-bit integer as decimal digits at `buf[pos..]`.
/// Returns the new position.
fn write_u32(buf: &mut [u8], pos: usize, mut n: u32) -> usize {
    if n == 0 {
        buf[pos] = b'0';
        return pos + 1;
    }
    let mut digits = [0u8; 10];
    let mut len = 0;
    while n > 0 {
        digits[len] = b'0' + (n % 10) as u8;
        n /= 10;
        len += 1;
    }
    for i in 0..len {
        buf[pos + i] = digits[len - 1 - i];
    }
    pos + len
}

/// Write a signed 32-bit integer as decimal digits (with leading `-`) at `buf[pos..]`.
/// Returns the new position.
fn write_i32(buf: &mut [u8], pos: usize, n: i32) -> usize {
    if n < 0 {
        buf[pos] = b'-';
        write_u32(buf, pos + 1, n.unsigned_abs())
    } else {
        write_u32(buf, pos, n as u32)
    }
}

/// Encode a velocity command: `<prefix><speed>\r`.
fn write_velocity_cmd(buf: &mut [u8], prefix: &[u8; 2], speed: u32) -> Result<usize, &'static str> {
    buf[0] = prefix[0];
    buf[1] = prefix[1];
    let pos = write_u32(buf, 2, speed);
    buf[pos] = b'\r';
    Ok(pos + 1)
}

// ── Decoding ──────────────────────────────────────────────────────────────────

/// Decode a complete response frame (including the trailing `\r`).
///
/// Handles optional `*XX` checksums and trims trailing `\r`/`\n` whitespace.
pub fn decode(input: &[u8]) -> Result<Response, ParseError> {
    if input.is_empty() {
        return Err(ParseError::UnexpectedEof);
    }

    // Strip trailing CR/LF.
    let trimmed = input
        .iter()
        .rposition(|&b| b != b'\r' && b != b'\n')
        .map(|i| &input[..=i])
        .unwrap_or(&[]);

    if trimmed.is_empty() {
        // A lone \r is treated as an Ack.
        return Ok(Response::Ack);
    }

    // Check for and strip optional `*XX` checksum.
    let (payload, expected_checksum) = if let Some(star) = trimmed
        .windows(3)
        .rposition(|w| w[0] == b'*')
    {
        let cs_bytes = &trimmed[star + 1..star + 3];
        if cs_bytes.len() == 2 {
            let hi = parse_hex_nibble(cs_bytes[0]).ok_or(ParseError::InvalidUtf8)?;
            let lo = parse_hex_nibble(cs_bytes[1]).ok_or(ParseError::InvalidUtf8)?;
            (
                &trimmed[..star],
                Some((hi << 4) | lo),
            )
        } else {
            (trimmed, None)
        }
    } else {
        (trimmed, None)
    };

    if let Some(expected) = expected_checksum {
        let actual = xor_checksum(payload);
        if actual != expected {
            return Err(ParseError::BadChecksum);
        }
    }

    // Device error.
    if payload == b"?" {
        return Ok(Response::Error);
    }

    // Acknowledgement (empty frame or just `+`).
    if payload.is_empty() || payload == b"+" {
        return Ok(Response::Ack);
    }

    // Position: "AZ=NNN EL=NNN"
    if let Some(pos_resp) = try_parse_position(payload) {
        return Ok(pos_resp);
    }

    // Single-axis position: "AZ=NNN.N" or "EL=NNN.N"
    if let Some(axis_resp) = try_parse_single_axis(payload) {
        return Ok(axis_resp);
    }

    // Status: "ST=<status>"
    if let Some(status_resp) = try_parse_status(payload) {
        return Ok(status_resp);
    }

    // Easycomm III registers: "GS<val>", "GE<val>", "CR<reg>,<val>"
    if let Some(reg_resp) = try_parse_register(payload) {
        return Ok(reg_resp);
    }

    Err(ParseError::UnknownCommand)
}

/// Attempt to parse `AZ=NNN EL=NNN` (with optional leading label letters).
fn try_parse_position(data: &[u8]) -> Option<Response> {
    // Expected: "AZ=NNN EL=NNN"  (13 bytes minimum)
    if data.len() < 13 {
        return None;
    }
    // Tolerate case differences and whitespace variation by scanning.
    let s = core::str::from_utf8(data).ok()?;
    let az = parse_labeled_value(s, "AZ=")?;
    let el = parse_labeled_value(s, "EL=")?;
    Some(Response::Position { az, el })
}

/// Find `label` in `s` and parse the 3 decimal digits following it.
fn parse_labeled_value(s: &str, label: &str) -> Option<u16> {
    let idx = s.find(label)?;
    let digits = s.get(idx + label.len()..idx + label.len() + 3)?;
    digits.parse().ok()
}

/// Attempt to parse a single-axis position: `AZNNN.N` or `ELNNN.N` (with or without `=`).
fn try_parse_single_axis(data: &[u8]) -> Option<Response> {
    let s = core::str::from_utf8(data).ok()?;
    // Accept both "AZ=NNN.N" and "AZNNN.N"
    if let Some(rest) = s.strip_prefix("AZ") {
        let val = rest.strip_prefix('=').unwrap_or(rest);
        let deg = parse_float_u16(val)?;
        return Some(Response::AzimuthPosition(deg));
    }
    if let Some(rest) = s.strip_prefix("EL") {
        let val = rest.strip_prefix('=').unwrap_or(rest);
        let deg = parse_float_u16(val)?;
        return Some(Response::ElevationPosition(deg));
    }
    None
}

/// Parse a decimal number (possibly with fractional part) and truncate to u16.
fn parse_float_u16(s: &str) -> Option<u16> {
    if s.is_empty() {
        return None;
    }
    let int_part = if let Some(dot) = s.find('.') { &s[..dot] } else { s };
    int_part.parse().ok()
}

/// Attempt to parse `ST=<status>`.
fn try_parse_status(data: &[u8]) -> Option<Response> {
    let s = core::str::from_utf8(data).ok()?;
    let value = s.strip_prefix("ST=")?;
    let status = status_from_str(value)?;
    Some(Response::Status(status))
}

/// Map a status string to a [`DeviceStatus`] variant.
fn status_from_str(s: &str) -> Option<DeviceStatus> {
    match s {
        "idle" => Some(DeviceStatus::Idle),
        "moving" => Some(DeviceStatus::Moving),
        "homing" => Some(DeviceStatus::Homing),
        "not_homed" => Some(DeviceStatus::NotHomed),
        "homing_az_error" => Some(DeviceStatus::HomingAzError),
        "homing_el_error" => Some(DeviceStatus::HomingElError),
        _ => None,
    }
}

/// Attempt to parse Easycomm III register responses: `GS<val>`, `GE<val>`, `CR<reg>,<val>`.
fn try_parse_register(data: &[u8]) -> Option<Response> {
    let s = core::str::from_utf8(data).ok()?;
    if let Some(val) = s.strip_prefix("GS") {
        let v: u16 = val.parse().ok()?;
        return Some(Response::StatusRegister(v));
    }
    if let Some(val) = s.strip_prefix("GE") {
        let v: u16 = val.parse().ok()?;
        return Some(Response::ErrorRegister(v));
    }
    if let Some(rest) = s.strip_prefix("CR") {
        let comma = rest.find(',')?;
        let reg: u16 = rest[..comma].parse().ok()?;
        let val: i32 = rest[comma + 1..].parse().ok()?;
        return Some(Response::ConfigValue { register: reg, value: val });
    }
    None
}

/// Return the wire-format string for a [`DeviceStatus`].
fn status_to_str(status: &DeviceStatus) -> &'static str {
    match status {
        DeviceStatus::Idle => "idle",
        DeviceStatus::Moving => "moving",
        DeviceStatus::Homing => "homing",
        DeviceStatus::NotHomed => "not_homed",
        DeviceStatus::HomingAzError => "homing_az_error",
        DeviceStatus::HomingElError => "homing_el_error",
    }
}

fn parse_hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

// ── Response encoding ─────────────────────────────────────────────────────────

/// Encode a [`Response`] into `buf` (no-std / fixed-buffer variant).
///
/// Returns the number of bytes written.  `buf` must be at least 32 bytes.
pub fn encode_response_into(resp: &Response, buf: &mut [u8]) -> usize {
    match resp {
        Response::Ack => {
            buf[0] = b'+';
            buf[1] = b'\r';
            2
        }
        Response::Error => {
            buf[0] = b'?';
            buf[1] = b'\r';
            2
        }
        Response::Position { az, el } => {
            // "AZ=NNN EL=NNN\r" — 14 bytes
            buf[0] = b'A';
            buf[1] = b'Z';
            buf[2] = b'=';
            write_3digits(buf, 3, *az);
            buf[6] = b' ';
            buf[7] = b'E';
            buf[8] = b'L';
            buf[9] = b'=';
            write_3digits(buf, 10, *el);
            buf[13] = b'\r';
            14
        }
        Response::AzimuthPosition(az) => {
            // "AZNNN.0\n" — 8 bytes (Easycomm II, no '=')
            buf[0] = b'A';
            buf[1] = b'Z';
            write_3digits(buf, 2, *az);
            buf[5] = b'.';
            buf[6] = b'0';
            buf[7] = b'\n';
            8
        }
        Response::ElevationPosition(el) => {
            // "ELNNN.0\n" — 8 bytes (Easycomm II, no '=')
            buf[0] = b'E';
            buf[1] = b'L';
            write_3digits(buf, 2, *el);
            buf[5] = b'.';
            buf[6] = b'0';
            buf[7] = b'\n';
            8
        }
        Response::Status(status) => {
            // "ST=<status>\r"
            let prefix = b"ST=";
            buf[..3].copy_from_slice(prefix);
            let value = status_to_str(status).as_bytes();
            buf[3..3 + value.len()].copy_from_slice(value);
            buf[3 + value.len()] = b'\r';
            4 + value.len()
        }
        Response::StatusRegister(val) => {
            // "GS<val>\n"
            buf[0] = b'G';
            buf[1] = b'S';
            let pos = write_u32(buf, 2, *val as u32);
            buf[pos] = b'\n';
            pos + 1
        }
        Response::ErrorRegister(val) => {
            // "GE<val>\n"
            buf[0] = b'G';
            buf[1] = b'E';
            let pos = write_u32(buf, 2, *val as u32);
            buf[pos] = b'\n';
            pos + 1
        }
        Response::ConfigValue { register, value } => {
            // "CR<reg>,<val>\n"
            buf[0] = b'C';
            buf[1] = b'R';
            let pos = write_u32(buf, 2, *register as u32);
            buf[pos] = b',';
            let pos = write_i32(buf, pos + 1, *value);
            buf[pos] = b'\n';
            pos + 1
        }
    }
}

// ── Device-side command parsing ───────────────────────────────────────────────

/// Maximum raw command frame length (bytes before the trailing `\r`).
/// Longest command: `CW65535,-2147483648` = 19 bytes; 32 leaves headroom.
const MAX_CMD_FRAME: usize = 32;

/// Byte-by-byte accumulator that parses incoming *host commands* from frames
/// terminated with `\r`.
///
/// This is the device-side counterpart to [`Parser`], which decodes *device
/// responses*.  Use `CommandParser` in rotator firmware; use `Parser` in
/// host-side controller software.
pub struct CommandParser {
    buf: [u8; MAX_CMD_FRAME],
    len: usize,
}

impl CommandParser {
    /// Create a new, empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_CMD_FRAME],
            len: 0,
        }
    }

    /// Feed one byte.
    ///
    /// Returns `Some(command)` when a complete `\r`-terminated frame has been
    /// assembled and successfully decoded, `None` otherwise.
    /// Malformed or overlong frames are silently discarded.
    pub fn feed(&mut self, byte: u8) -> Option<Command> {
        if byte == b'\r' || byte == b'\n' {
            let cmd = decode_command(&self.buf[..self.len]);
            self.len = 0;
            cmd
        } else if self.len < MAX_CMD_FRAME {
            self.buf[self.len] = byte;
            self.len += 1;
            None
        } else {
            self.len = 0; // overlong — discard
            None
        }
    }
}

impl Default for CommandParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Parse a raw frame (bytes *before* the `\r`) as an Easycom host command.
///
/// Supports GS-232 fixed-length commands, Easycomm II, and Easycomm III
/// variable-length commands.
///
/// Returns `None` for unknown or out-of-range frames.
pub fn decode_command(frame: &[u8]) -> Option<Command> {
    let frame = strip_cmd_checksum(frame);

    // Exact-match commands (unambiguous, checked first).
    match frame {
        [] => return None,
        [b'C'] => return Some(Command::QueryPosition),
        [b'S'] => return Some(Command::Stop),
        [b'?'] => return Some(Command::KeepAlive),
        [b'G', b'S'] => return Some(Command::GetStatusRegister),
        [b'G', b'E'] => return Some(Command::GetErrorRegister),
        [b'A', b'Z'] => return Some(Command::QueryAzimuth),
        [b'E', b'L'] => return Some(Command::QueryElevation),
        _ => {}
    }

    // GS-232 fixed-length patterns.
    match frame {
        [b'A', d0, d1, d2] => {
            if let Some(cmd) = parse_u16_3digit(*d0, *d1, *d2)
                .filter(|&az| az <= 360)
                .map(Command::Azimuth)
            {
                return Some(cmd);
            }
        }
        [b'E', d0, d1, d2] => {
            if let Some(cmd) = parse_u16_3digit(*d0, *d1, *d2)
                .filter(|&el| el <= 180)
                .map(Command::Elevation)
            {
                return Some(cmd);
            }
        }
        [b'W', a0, a1, a2, b' ', e0, e1, e2] => {
            let az = parse_u16_3digit(*a0, *a1, *a2).filter(|&v| v <= 360);
            let el = parse_u16_3digit(*e0, *e1, *e2).filter(|&v| v <= 180);
            if let (Some(az), Some(el)) = (az, el) {
                return Some(Command::AzimuthElevation { az, el });
            }
        }
        [b'O', s1, a0, a1, a2, s2, e0, e1, e2] => {
            if let (Some(az), Some(el)) = (
                parse_signed_3digit(*s1, *a0, *a1, *a2),
                parse_signed_3digit(*s2, *e0, *e1, *e2),
            ) {
                return Some(Command::Offset { az, el });
            }
        }
        _ => {}
    }

    // Easycomm II/III variable-length patterns.
    try_parse_easycomm(frame)
}

/// Parse Easycomm II/III variable-length commands.
fn try_parse_easycomm(frame: &[u8]) -> Option<Command> {
    let s = core::str::from_utf8(frame).ok()?;

    // Multi-character keywords.
    if s == "RESET" {
        return Some(Command::Reset);
    }
    if s == "PARK" {
        return Some(Command::Park);
    }

    // Stop variants: SA SE, SA, SE
    if s == "SA SE" || s == "SA" || s == "SE" {
        return Some(Command::Stop);
    }

    // Velocity commands: VL/VR/VU/VD<speed>
    if let Some(val) = s.strip_prefix("VL") {
        return val.parse().ok().map(Command::VelocityLeft);
    }
    if let Some(val) = s.strip_prefix("VR") {
        return val.parse().ok().map(Command::VelocityRight);
    }
    if let Some(val) = s.strip_prefix("VU") {
        return val.parse().ok().map(Command::VelocityUp);
    }
    if let Some(val) = s.strip_prefix("VD") {
        return val.parse().ok().map(Command::VelocityDown);
    }

    // Config write: CW<reg>,<val>
    if let Some(rest) = s.strip_prefix("CW") {
        let comma = rest.find(',')?;
        let reg: u16 = rest[..comma].parse().ok()?;
        let val: i32 = rest[comma + 1..].parse().ok()?;
        return Some(Command::WriteConfig { register: reg, value: val });
    }

    // Config read: CR<reg>
    if let Some(rest) = s.strip_prefix("CR") {
        let reg: u16 = rest.parse().ok()?;
        return Some(Command::ReadConfig(reg));
    }

    // Status register with value: GS<val> (bare GS handled in exact match)
    if let Some(val) = s.strip_prefix("GS")
        && !val.is_empty()
    {
        return Some(Command::GetStatusRegister);
    }

    // Combined set: "AZnnn.n ELnnn.n"
    if let Some(space_idx) = s.find(' ') {
        let left = &s[..space_idx];
        let right = &s[space_idx + 1..];
        if let (Some(az), Some(el)) = (
            left.strip_prefix("AZ").and_then(parse_float_u16),
            right.strip_prefix("EL").and_then(parse_float_u16),
        )
            && az <= 360 && el <= 180
        {
            return Some(Command::AzimuthElevation { az, el });
        }
    }

    // Single-axis set: "AZnnn.n" or "ELnnn.n"
    if let Some(val) = s.strip_prefix("AZ") {
        let az = parse_float_u16(val)?;
        if az <= 360 {
            return Some(Command::Azimuth(az));
        }
    }
    if let Some(val) = s.strip_prefix("EL") {
        let el = parse_float_u16(val)?;
        if el <= 180 {
            return Some(Command::Elevation(el));
        }
    }

    None
}

/// Strip an optional `*XX` XOR-checksum suffix from a command frame.
fn strip_cmd_checksum(frame: &[u8]) -> &[u8] {
    if frame.len() >= 3
        && let Some(pos) = frame.iter().rposition(|&b| b == b'*')
        && pos + 3 <= frame.len()
    {
        return &frame[..pos];
    }
    frame
}

fn parse_u16_3digit(d0: u8, d1: u8, d2: u8) -> Option<u16> {
    let a = ascii_digit(d0)? as u16;
    let b = ascii_digit(d1)? as u16;
    let c = ascii_digit(d2)? as u16;
    Some(a * 100 + b * 10 + c)
}

fn parse_signed_3digit(sign: u8, d0: u8, d1: u8, d2: u8) -> Option<i16> {
    let mag = parse_u16_3digit(d0, d1, d2)? as i16;
    match sign {
        b'+' => Some(mag),
        b'-' => Some(-mag),
        _ => None,
    }
}

fn ascii_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        _ => None,
    }
}

// ── Incremental response Parser ───────────────────────────────────────────────

/// Maximum size of a single Easycom frame (generous bound for a fixed buffer).
const MAX_FRAME: usize = 64;

/// Incremental byte-by-byte parser.
///
/// Feed bytes with [`Parser::feed`]; each call returns `Some(result)` when a
/// complete frame has been assembled, or `None` if more data is needed.
pub struct Parser {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Parser {
    /// Create a new, empty parser.
    pub fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME],
            len: 0,
        }
    }

    /// Feed one byte to the parser.
    ///
    /// Returns `Some(Ok(response))` when a complete frame ending with `\r` is
    /// received, `Some(Err(_))` if the frame is malformed, or `None` if more
    /// bytes are needed.
    ///
    /// Overlong frames (exceeding the internal buffer) are discarded and a
    /// [`ParseError::UnexpectedEof`] is returned.
    pub fn feed(&mut self, byte: u8) -> Option<Result<Response, ParseError>> {
        if byte == b'\r' || byte == b'\n' {
            // Frame complete — decode what we have.
            let result = decode(&self.buf[..self.len]);
            self.len = 0;
            Some(result)
        } else if self.len < MAX_FRAME {
            self.buf[self.len] = byte;
            self.len += 1;
            None
        } else {
            // Buffer overflow — discard frame.
            self.len = 0;
            Some(Err(ParseError::UnexpectedEof))
        }
    }

    /// Feed a slice of bytes, collecting any complete frames.
    ///
    /// Returns a `Vec<Result<Response, ParseError>>` for each complete frame
    /// found in `data`.
    #[cfg(feature = "std")]
    pub fn feed_slice(&mut self, data: &[u8]) -> Vec<Result<Response, ParseError>> {
        data.iter().filter_map(|&b| self.feed(b)).collect()
    }
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::{Command, DeviceStatus, Response};

    // ── encode helpers ────────────────────────────────────────────────────────

    fn enc(cmd: &Command) -> Vec<u8> {
        encode(cmd).unwrap()
    }

    // ── encode tests ──────────────────────────────────────────────────────────

    #[test]
    fn encode_azimuth() {
        assert_eq!(enc(&Command::Azimuth(90)), b"A090\r");
        assert_eq!(enc(&Command::Azimuth(0)), b"A000\r");
        assert_eq!(enc(&Command::Azimuth(360)), b"A360\r");
    }

    #[test]
    fn encode_elevation() {
        assert_eq!(enc(&Command::Elevation(45)), b"E045\r");
        assert_eq!(enc(&Command::Elevation(180)), b"E180\r");
    }

    #[test]
    fn encode_azimuth_elevation() {
        assert_eq!(enc(&Command::AzimuthElevation { az: 180, el: 90 }), b"W180 090\r");
    }

    #[test]
    fn encode_query_position() {
        assert_eq!(enc(&Command::QueryPosition), b"C\r");
    }

    #[test]
    fn encode_query_azimuth() {
        assert_eq!(enc(&Command::QueryAzimuth), b"AZ\r");
    }

    #[test]
    fn encode_query_elevation() {
        assert_eq!(enc(&Command::QueryElevation), b"EL\r");
    }

    #[test]
    fn encode_stop() {
        assert_eq!(enc(&Command::Stop), b"S\r");
    }

    #[test]
    fn encode_keep_alive() {
        assert_eq!(enc(&Command::KeepAlive), b"?\r");
    }

    #[test]
    fn encode_offset() {
        let mut buf = [0u8; 32];
        let n = encode_into(&Command::Offset { az: 10, el: -5 }, &mut buf).unwrap();
        assert_eq!(&buf[..n], b"O+010-005\r");
    }

    #[test]
    fn encode_rejects_out_of_range() {
        assert!(encode(&Command::Azimuth(361)).is_err());
        assert!(encode(&Command::Elevation(181)).is_err());
        assert!(encode(&Command::AzimuthElevation { az: 400, el: 0 }).is_err());
        assert!(encode(&Command::AzimuthElevation { az: 0, el: 200 }).is_err());
    }

    // ── checksum encoding ─────────────────────────────────────────────────────

    #[test]
    fn encode_with_checksum_query() {
        let mut buf = [0u8; 32];
        let n = encode_with_checksum(&Command::QueryPosition, &mut buf).unwrap();
        let frame = &buf[..n];
        // Payload is b"C", checksum = b'C' = 0x43
        assert_eq!(&frame[frame.len() - 4..], b"*43\r");
    }

    // ── decode tests ──────────────────────────────────────────────────────────

    #[test]
    fn decode_position() {
        let r = decode(b"AZ=180 EL=090\r").unwrap();
        assert_eq!(r, Response::Position { az: 180, el: 90 });
    }

    #[test]
    fn decode_ack_empty() {
        assert_eq!(decode(b"\r").unwrap(), Response::Ack);
        assert_eq!(decode(b"+\r").unwrap(), Response::Ack);
    }

    #[test]
    fn decode_device_error() {
        assert_eq!(decode(b"?\r").unwrap(), Response::Error);
    }

    #[test]
    fn decode_bad_checksum() {
        // "AZ=090 EL=045*FF\r" — wrong checksum
        let r = decode(b"AZ=090 EL=045*FF\r");
        assert_eq!(r, Err(ParseError::BadChecksum));
    }

    #[test]
    fn decode_good_checksum() {
        // Build a frame with a correct checksum and verify it decodes.
        let payload = b"AZ=090 EL=045";
        let cs = xor_checksum(payload);
        let mut frame: Vec<u8> = payload.to_vec();
        frame.push(b'*');
        frame.push(b"0123456789ABCDEF"[(cs >> 4) as usize]);
        frame.push(b"0123456789ABCDEF"[(cs & 0xF) as usize]);
        frame.push(b'\r');
        assert_eq!(decode(&frame).unwrap(), Response::Position { az: 90, el: 45 });
    }

    #[test]
    fn decode_unexpected_eof() {
        assert_eq!(decode(b""), Err(ParseError::UnexpectedEof));
    }

    // ── round-trip tests ──────────────────────────────────────────────────────

    #[test]
    fn round_trip_position_response() {
        // A QueryPosition command produces C\r; device responds AZ=... EL=...
        let cmd = enc(&Command::QueryPosition);
        assert_eq!(cmd, b"C\r");
        let resp = decode(b"AZ=270 EL=030\r").unwrap();
        assert_eq!(resp, Response::Position { az: 270, el: 30 });
    }

    // ── encode_response_into tests ────────────────────────────────────────────

    #[test]
    fn encode_response_ack() {
        let mut buf = [0u8; 16];
        let n = encode_response_into(&Response::Ack, &mut buf);
        assert_eq!(&buf[..n], b"+\r");
    }

    #[test]
    fn encode_response_error() {
        let mut buf = [0u8; 16];
        let n = encode_response_into(&Response::Error, &mut buf);
        assert_eq!(&buf[..n], b"?\r");
    }

    #[test]
    fn encode_response_position() {
        let mut buf = [0u8; 16];
        let n = encode_response_into(&Response::Position { az: 270, el: 45 }, &mut buf);
        assert_eq!(&buf[..n], b"AZ=270 EL=045\r");
    }

    #[test]
    fn encode_response_azimuth_position() {
        let mut buf = [0u8; 16];
        let n = encode_response_into(&Response::AzimuthPosition(270), &mut buf);
        assert_eq!(&buf[..n], b"AZ270.0\n");
    }

    #[test]
    fn encode_response_elevation_position() {
        let mut buf = [0u8; 16];
        let n = encode_response_into(&Response::ElevationPosition(45), &mut buf);
        assert_eq!(&buf[..n], b"EL045.0\n");
    }

    // ── decode_command tests ──────────────────────────────────────────────────

    #[test]
    fn decode_command_query_position() {
        assert_eq!(decode_command(b"C"), Some(Command::QueryPosition));
    }

    #[test]
    fn decode_command_stop() {
        assert_eq!(decode_command(b"S"), Some(Command::Stop));
    }

    #[test]
    fn decode_command_keep_alive() {
        assert_eq!(decode_command(b"?"), Some(Command::KeepAlive));
    }

    #[test]
    fn decode_command_azimuth() {
        assert_eq!(decode_command(b"A090"), Some(Command::Azimuth(90)));
        assert_eq!(decode_command(b"A360"), Some(Command::Azimuth(360)));
        assert_eq!(decode_command(b"A361"), None); // out of range
    }

    #[test]
    fn decode_command_elevation() {
        assert_eq!(decode_command(b"E045"), Some(Command::Elevation(45)));
        assert_eq!(decode_command(b"E180"), Some(Command::Elevation(180)));
        assert_eq!(decode_command(b"E181"), None);
    }

    #[test]
    fn decode_command_azimuth_elevation() {
        assert_eq!(
            decode_command(b"W180 090"),
            Some(Command::AzimuthElevation { az: 180, el: 90 })
        );
    }

    #[test]
    fn decode_command_offset() {
        assert_eq!(
            decode_command(b"O+010-005"),
            Some(Command::Offset { az: 10, el: -5 })
        );
    }

    #[test]
    fn decode_command_strips_checksum() {
        // "C*43" — checksum stripped, then decoded as QueryPosition
        assert_eq!(decode_command(b"C*43"), Some(Command::QueryPosition));
    }

    // ── Easycomm II command tests ─────────────────────────────────────────────

    #[test]
    fn decode_command_query_azimuth() {
        assert_eq!(decode_command(b"AZ"), Some(Command::QueryAzimuth));
    }

    #[test]
    fn decode_command_query_elevation() {
        assert_eq!(decode_command(b"EL"), Some(Command::QueryElevation));
    }

    #[test]
    fn decode_command_easycomm2_set_azimuth() {
        assert_eq!(decode_command(b"AZ180.0"), Some(Command::Azimuth(180)));
        assert_eq!(decode_command(b"AZ45"), Some(Command::Azimuth(45)));
        assert_eq!(decode_command(b"AZ361.0"), None); // out of range
    }

    #[test]
    fn decode_command_easycomm2_set_elevation() {
        assert_eq!(decode_command(b"EL90.0"), Some(Command::Elevation(90)));
        assert_eq!(decode_command(b"EL45"), Some(Command::Elevation(45)));
        assert_eq!(decode_command(b"EL181.0"), None);
    }

    #[test]
    fn decode_command_easycomm2_set_both() {
        assert_eq!(
            decode_command(b"AZ180.0 EL90.0"),
            Some(Command::AzimuthElevation { az: 180, el: 90 })
        );
    }

    #[test]
    fn decode_command_easycomm2_stop() {
        assert_eq!(decode_command(b"SA SE"), Some(Command::Stop));
        assert_eq!(decode_command(b"SA"), Some(Command::Stop));
        assert_eq!(decode_command(b"SE"), Some(Command::Stop));
    }

    // ── decode response tests (single-axis) ───────────────────────────────────

    #[test]
    fn decode_azimuth_position() {
        assert_eq!(
            decode(b"AZ=270.0\r").unwrap(),
            Response::AzimuthPosition(270)
        );
    }

    #[test]
    fn decode_elevation_position() {
        assert_eq!(
            decode(b"EL=045.0\r").unwrap(),
            Response::ElevationPosition(45)
        );
    }

    // ── Easycomm III encode tests ────────────────────────────────────────────

    #[test]
    fn encode_velocity_left() {
        assert_eq!(enc(&Command::VelocityLeft(100)), b"VL100\r");
    }

    #[test]
    fn encode_velocity_right() {
        assert_eq!(enc(&Command::VelocityRight(0)), b"VR0\r");
    }

    #[test]
    fn encode_velocity_up() {
        assert_eq!(enc(&Command::VelocityUp(5000)), b"VU5000\r");
    }

    #[test]
    fn encode_velocity_down() {
        assert_eq!(enc(&Command::VelocityDown(250)), b"VD250\r");
    }

    #[test]
    fn encode_get_status_register() {
        assert_eq!(enc(&Command::GetStatusRegister), b"GS\r");
    }

    #[test]
    fn encode_get_error_register() {
        assert_eq!(enc(&Command::GetErrorRegister), b"GE\r");
    }

    #[test]
    fn encode_read_config() {
        assert_eq!(enc(&Command::ReadConfig(0)), b"CR0\r");
        assert_eq!(enc(&Command::ReadConfig(5)), b"CR5\r");
    }

    #[test]
    fn encode_write_config() {
        assert_eq!(enc(&Command::WriteConfig { register: 0, value: 1000 }), b"CW0,1000\r");
        assert_eq!(enc(&Command::WriteConfig { register: 3, value: -50 }), b"CW3,-50\r");
    }

    #[test]
    fn encode_reset() {
        assert_eq!(enc(&Command::Reset), b"RESET\r");
    }

    #[test]
    fn encode_park() {
        assert_eq!(enc(&Command::Park), b"PARK\r");
    }

    // ── Easycomm III decode command tests ─────────────────────────────────────

    #[test]
    fn decode_command_get_status_register() {
        assert_eq!(decode_command(b"GS"), Some(Command::GetStatusRegister));
    }

    #[test]
    fn decode_command_get_error_register() {
        assert_eq!(decode_command(b"GE"), Some(Command::GetErrorRegister));
    }

    #[test]
    fn decode_command_velocity_left() {
        assert_eq!(decode_command(b"VL100"), Some(Command::VelocityLeft(100)));
    }

    #[test]
    fn decode_command_velocity_right() {
        assert_eq!(decode_command(b"VR0"), Some(Command::VelocityRight(0)));
    }

    #[test]
    fn decode_command_velocity_up() {
        assert_eq!(decode_command(b"VU5000"), Some(Command::VelocityUp(5000)));
    }

    #[test]
    fn decode_command_velocity_down() {
        assert_eq!(decode_command(b"VD250"), Some(Command::VelocityDown(250)));
    }

    #[test]
    fn decode_command_read_config() {
        assert_eq!(decode_command(b"CR0"), Some(Command::ReadConfig(0)));
        assert_eq!(decode_command(b"CR5"), Some(Command::ReadConfig(5)));
    }

    #[test]
    fn decode_command_write_config() {
        assert_eq!(
            decode_command(b"CW0,1000"),
            Some(Command::WriteConfig { register: 0, value: 1000 })
        );
        assert_eq!(
            decode_command(b"CW3,-50"),
            Some(Command::WriteConfig { register: 3, value: -50 })
        );
    }

    #[test]
    fn decode_command_reset() {
        assert_eq!(decode_command(b"RESET"), Some(Command::Reset));
    }

    #[test]
    fn decode_command_park() {
        assert_eq!(decode_command(b"PARK"), Some(Command::Park));
    }

    // ── Easycomm III decode response tests ────────────────────────────────────

    #[test]
    fn decode_status_register() {
        assert_eq!(decode(b"GS2\n").unwrap(), Response::StatusRegister(2));
    }

    #[test]
    fn decode_error_register() {
        assert_eq!(decode(b"GE4\n").unwrap(), Response::ErrorRegister(4));
    }

    #[test]
    fn decode_config_value() {
        assert_eq!(
            decode(b"CR0,1000\n").unwrap(),
            Response::ConfigValue { register: 0, value: 1000 }
        );
        assert_eq!(
            decode(b"CR3,-50\n").unwrap(),
            Response::ConfigValue { register: 3, value: -50 }
        );
    }

    // ── Easycomm III encode response tests ────────────────────────────────────

    #[test]
    fn encode_response_status_register() {
        let mut buf = [0u8; 32];
        let n = encode_response_into(&Response::StatusRegister(2), &mut buf);
        assert_eq!(&buf[..n], b"GS2\n");
    }

    #[test]
    fn encode_response_error_register() {
        let mut buf = [0u8; 32];
        let n = encode_response_into(&Response::ErrorRegister(4), &mut buf);
        assert_eq!(&buf[..n], b"GE4\n");
    }

    #[test]
    fn encode_response_config_value() {
        let mut buf = [0u8; 32];
        let n = encode_response_into(&Response::ConfigValue { register: 0, value: 1000 }, &mut buf);
        assert_eq!(&buf[..n], b"CR0,1000\n");
    }

    #[test]
    fn command_parser_incremental() {
        let mut p = CommandParser::new();
        let frame = b"A090\r";
        let mut result = None;
        for &b in frame {
            result = p.feed(b);
        }
        assert_eq!(result, Some(Command::Azimuth(90)));
    }

    // ── status tests ──────────────────────────────────────────────────────────

    #[test]
    fn encode_query_status() {
        assert_eq!(enc(&Command::QueryStatus), b"GS\r");
    }

    #[test]
    fn decode_status_idle() {
        assert_eq!(decode(b"ST=idle\r").unwrap(), Response::Status(DeviceStatus::Idle));
    }

    #[test]
    fn decode_status_moving() {
        assert_eq!(decode(b"ST=moving\r").unwrap(), Response::Status(DeviceStatus::Moving));
    }

    #[test]
    fn decode_status_homing() {
        assert_eq!(decode(b"ST=homing\r").unwrap(), Response::Status(DeviceStatus::Homing));
    }

    #[test]
    fn decode_status_not_homed() {
        assert_eq!(decode(b"ST=not_homed\r").unwrap(), Response::Status(DeviceStatus::NotHomed));
    }

    #[test]
    fn decode_status_homing_az_error() {
        assert_eq!(
            decode(b"ST=homing_az_error\r").unwrap(),
            Response::Status(DeviceStatus::HomingAzError)
        );
    }

    #[test]
    fn decode_status_homing_el_error() {
        assert_eq!(
            decode(b"ST=homing_el_error\r").unwrap(),
            Response::Status(DeviceStatus::HomingElError)
        );
    }

    #[test]
    fn decode_status_unknown() {
        assert_eq!(decode(b"ST=bogus\r"), Err(ParseError::UnknownCommand));
    }

    #[test]
    fn encode_response_status() {
        let mut buf = [0u8; 32];
        let n = encode_response_into(&Response::Status(DeviceStatus::HomingAzError), &mut buf);
        assert_eq!(&buf[..n], b"ST=homing_az_error\r");
    }

    #[test]
    fn decode_command_gs_maps_to_get_status_register() {
        // Bare "GS" on the wire maps to Easycomm III GetStatusRegister.
        assert_eq!(decode_command(b"GS"), Some(Command::GetStatusRegister));
    }

    #[test]
    fn round_trip_status_response() {
        let resp = Response::Status(DeviceStatus::Idle);
        let mut buf = [0u8; 32];
        let n = encode_response_into(&resp, &mut buf);
        assert_eq!(decode(&buf[..n]).unwrap(), resp);
    }

    // ── Parser tests ──────────────────────────────────────────────────────────

    #[test]
    fn parser_incremental() {
        let mut p = Parser::new();
        let data = b"AZ=045 EL=010\r";
        let mut results = Vec::new();
        for &b in data {
            if let Some(r) = p.feed(b) {
                results.push(r);
            }
        }
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].as_ref().unwrap(), &Response::Position { az: 45, el: 10 });
    }

    #[test]
    fn parser_multiple_frames() {
        let mut p = Parser::new();
        let data = b"?\rAZ=090 EL=000\r";
        let results = p.feed_slice(data);
        assert_eq!(results.len(), 2);
        assert_eq!(results[0].as_ref().unwrap(), &Response::Error);
        assert_eq!(results[1].as_ref().unwrap(), &Response::Position { az: 90, el: 0 });
    }
}
