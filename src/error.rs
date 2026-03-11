/// Errors that can occur while encoding, decoding, or transporting Easycom frames.
#[non_exhaustive]
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// An I/O error from the underlying transport.
    Transport(E),
    /// The received frame could not be parsed.
    Parse(ParseError),
    /// The operation timed out waiting for a response.
    Timeout,
    /// A command parameter is out of its valid range.
    InvalidParam(&'static str),
}

impl<E: core::fmt::Display> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Transport(e) => write!(f, "transport error: {e}"),
            Error::Parse(p) => write!(f, "parse error: {p}"),
            Error::Timeout => write!(f, "timeout"),
            Error::InvalidParam(s) => write!(f, "invalid parameter: {s}"),
        }
    }
}

/// Specific parse failures when decoding a response frame.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ParseError {
    /// Input ended before a complete frame was received.
    UnexpectedEof,
    /// A field that was expected to be ASCII/UTF-8 was not.
    InvalidUtf8,
    /// The command letter in a frame was not recognised.
    UnknownCommand,
    /// The optional XOR checksum did not match the payload.
    BadChecksum,
}

impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParseError::UnexpectedEof => write!(f, "unexpected end of input"),
            ParseError::InvalidUtf8 => write!(f, "invalid UTF-8 in frame"),
            ParseError::UnknownCommand => write!(f, "unknown command letter"),
            ParseError::BadChecksum => write!(f, "checksum mismatch"),
        }
    }
}
