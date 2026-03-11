//! Abstract I/O transport used by [`crate::session::Session`].
//!
//! Implement this trait on any type that can send and receive raw bytes, such
//! as a serial port, TCP stream, or an in-memory mock.

/// A synchronous byte-oriented I/O channel.
///
/// Implementations cover anything from a real serial port to a TCP socket or
/// an in-memory test double.
pub trait Transport {
    /// The error type produced by transport operations.
    type Error;

    /// Write all bytes in `frame` to the transport.
    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error>;

    /// Read up to `buf.len()` bytes from the transport into `buf`.
    ///
    /// Returns the number of bytes actually read.  A return value of `0` may
    /// indicate that the connection was closed or that no data was available
    /// at this time, depending on the implementation.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
}

// ── In-memory mock transport (available under std for testing) ────────────────

/// A loopback transport that stores writes in an internal buffer and replays
/// pre-programmed responses on read, one byte at a time.
///
/// Useful for unit tests that exercise [`crate::session::Session`] without
/// real hardware.
#[cfg(feature = "std")]
pub struct MockTransport {
    /// Bytes captured by calls to [`Transport::write`].
    pub written: Vec<u8>,
    /// Pending response bytes not yet consumed by [`Transport::read`].
    pub rx_buf: std::collections::VecDeque<u8>,
}

#[cfg(feature = "std")]
impl MockTransport {
    /// Create an empty mock with no pre-programmed responses.
    pub fn new() -> Self {
        Self {
            written: Vec::new(),
            rx_buf: std::collections::VecDeque::new(),
        }
    }

    /// Queue a response frame whose bytes will be drained by subsequent `read`
    /// calls.
    pub fn enqueue_response(&mut self, frame: Vec<u8>) {
        self.rx_buf.extend(frame);
    }
}

#[cfg(feature = "std")]
impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "std")]
impl Transport for MockTransport {
    type Error = std::io::Error;

    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.written.extend_from_slice(frame);
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = buf.len().min(self.rx_buf.len());
        for slot in buf.iter_mut().take(n) {
            *slot = self.rx_buf.pop_front().unwrap();
        }
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_write_and_read() {
        let mut t = MockTransport::new();
        t.enqueue_response(b"AZ=090 EL=045\r".to_vec());

        t.write(b"C\r").unwrap();
        assert_eq!(&t.written, b"C\r");

        let mut buf = [0u8; 32];
        let n = t.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"AZ=090 EL=045\r");
    }

    #[test]
    fn mock_empty_read() {
        let mut t = MockTransport::new();
        let mut buf = [0u8; 8];
        let n = t.read(&mut buf).unwrap();
        assert_eq!(n, 0);
    }
}
