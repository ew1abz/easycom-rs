//! Frame encoding and decoding for the Easycom protocol.
//!
//! Encoding rules (GS-232A/B compatible):
//! - `A<NNN>\r`  — set azimuth, 3 zero-padded decimal digits
//! - `E<NNN>\r`  — set elevation, 3 zero-padded decimal digits
//! - `W<NNN> <NNN>\r` — set azimuth and elevation (space-separated)
//! - `C\r`       — query current position
//! - `S\r`       — stop
//!
//! An optional XOR checksum may be appended as `*XX` (ASCII hex) before `\r`.
//!
//! Response formats:
//! - `AZ=NNN EL=NNN\r` — position response to a `C` query
//! - `+\r` or an empty `\r` — acknowledgement
//! - `?\r` — device error

use crate::command::{Command, Response};
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
/// Returns the number of bytes written.  `buf` must be at least 16 bytes.
///
/// | Response          | Frame             | Bytes |
/// |-------------------|-------------------|-------|
/// | `Ack`             | `+\r`             | 2     |
/// | `Error`           | `?\r`             | 2     |
/// | `Position{az,el}` | `AZ=NNN EL=NNN\r` | 14    |
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
    }
}

// ── Device-side command parsing ───────────────────────────────────────────────

/// Maximum raw command frame length (bytes before the trailing `\r`).
/// Longest command: `O+360-180` = 9 bytes; 16 leaves headroom.
const MAX_CMD_FRAME: usize = 16;

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
/// Returns `None` for unknown or out-of-range frames.
pub fn decode_command(frame: &[u8]) -> Option<Command> {
    let frame = strip_cmd_checksum(frame);
    match frame {
        [] => None,
        [b'C'] => Some(Command::QueryPosition),
        [b'S'] => Some(Command::Stop),
        [b'?'] => Some(Command::KeepAlive),
        [b'A', d0, d1, d2] => parse_u16_3digit(*d0, *d1, *d2)
            .filter(|&az| az <= 360)
            .map(Command::Azimuth),
        [b'E', d0, d1, d2] => parse_u16_3digit(*d0, *d1, *d2)
            .filter(|&el| el <= 180)
            .map(Command::Elevation),
        [b'W', a0, a1, a2, b' ', e0, e1, e2] => {
            let az = parse_u16_3digit(*a0, *a1, *a2).filter(|&v| v <= 360)?;
            let el = parse_u16_3digit(*e0, *e1, *e2).filter(|&v| v <= 180)?;
            Some(Command::AzimuthElevation { az, el })
        }
        [b'O', s1, a0, a1, a2, s2, e0, e1, e2] => {
            let az = parse_signed_3digit(*s1, *a0, *a1, *a2)?;
            let el = parse_signed_3digit(*s2, *e0, *e1, *e2)?;
            Some(Command::Offset { az, el })
        }
        _ => None,
    }
}

/// Strip an optional `*XX` XOR-checksum suffix from a command frame.
fn strip_cmd_checksum(frame: &[u8]) -> &[u8] {
    if frame.len() >= 3 {
        if let Some(pos) = frame.iter().rposition(|&b| b == b'*') {
            if pos + 3 <= frame.len() {
                return &frame[..pos];
            }
        }
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
    use crate::command::{Command, Response};

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
