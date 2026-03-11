//! High-level synchronous session over a [`Transport`].
//!
//! [`Session`] encodes a [`Command`], writes it to the transport, reads bytes
//! until a complete response frame arrives, decodes it, and returns the
//! [`Response`].

use crate::command::{Command, Response};
use crate::error::Error;
use crate::framing::{encode_into, Parser};
use crate::transport::Transport;

/// A synchronous Easycom session.
///
/// Wraps a [`Transport`] and an incremental [`Parser`] to provide a simple
/// request/response interface.
///
/// # Example
/// ```rust,no_run
/// use easycom::{Session, Command};
/// use easycom::transport::MockTransport;
///
/// let mut mock = MockTransport::new();
/// mock.enqueue_response(b"AZ=090 EL=045\r".to_vec());
///
/// let mut session = Session::new(mock);
/// let response = session.send(Command::QueryPosition).unwrap();
/// ```
pub struct Session<T> {
    transport: T,
    parser: Parser,
}

impl<T: Transport> Session<T> {
    /// Create a new session backed by `transport`.
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            parser: Parser::new(),
        }
    }

    /// Consume `self` and return the underlying transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// Encode `cmd`, send it over the transport, and wait for a complete
    /// response frame.
    ///
    /// Bytes are read one at a time until the internal [`Parser`] signals that
    /// a complete frame has been received.
    ///
    /// # Errors
    /// - [`Error::InvalidParam`] — a command parameter was out of its valid
    ///   range.
    /// - [`Error::Transport`] — an I/O error occurred.
    /// - [`Error::Parse`] — the device returned a malformed frame.
    pub fn send(&mut self, cmd: Command) -> Result<Response, Error<T::Error>> {
        // Encode.
        let mut frame_buf = [0u8; 32];
        let len = encode_into(&cmd, &mut frame_buf)
            .map_err(Error::InvalidParam)?;

        // Send.
        self.transport
            .write(&frame_buf[..len])
            .map_err(Error::Transport)?;

        // Receive: read byte-by-byte until the parser emits a non-echo frame.
        // `rx_frame` accumulates the current frame's bytes so we can detect
        // whether the device echoed our command back before its real response.
        let mut byte = [0u8; 1];
        let mut rx_frame = [0u8; 32];
        let mut rx_len = 0usize;
        loop {
            let n = self
                .transport
                .read(&mut byte)
                .map_err(Error::Transport)?;

            if n == 0 {
                // Transport returned no data — treat as EOF / timeout.
                return Err(Error::Timeout);
            }

            let b = byte[0];
            if b == b'\r' || b == b'\n' {
                if let Some(result) = self.parser.feed(b) {
                    // Echo suppression: skip frames that are byte-identical
                    // to the command we sent (excluding the terminator).
                    let sent = &frame_buf[..len.saturating_sub(1)]; // strip \r
                    if rx_frame[..rx_len] == *sent {
                        rx_len = 0;
                        continue;
                    }
                    return result.map_err(Error::Parse);
                }
                rx_len = 0;
            } else {
                if rx_len < rx_frame.len() {
                    rx_frame[rx_len] = b;
                    rx_len += 1;
                }
                self.parser.feed(b);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MockTransport;

    #[test]
    fn session_query_position() {
        let mut mock = MockTransport::new();
        mock.enqueue_response(b"AZ=180 EL=090\r".to_vec());

        let mut session = Session::new(mock);
        let resp = session.send(Command::QueryPosition).unwrap();
        assert_eq!(resp, Response::Position { az: 180, el: 90 });
    }

    #[test]
    fn session_send_written_bytes() {
        let mut mock = MockTransport::new();
        mock.enqueue_response(b"+\r".to_vec());

        let mut session = Session::new(mock);
        session.send(Command::Azimuth(90)).unwrap();

        let transport = session.into_transport();
        assert_eq!(&transport.written, b"A090\r");
    }

    #[test]
    fn session_device_error_response() {
        let mut mock = MockTransport::new();
        mock.enqueue_response(b"?\r".to_vec());

        let mut session = Session::new(mock);
        let resp = session.send(Command::Stop).unwrap();
        assert_eq!(resp, Response::Error);
    }

    #[test]
    fn session_echo_suppression() {
        let mut mock = MockTransport::new();
        // Device echoes "C\r" before the real response.
        mock.enqueue_response(b"C\rAZ=045 EL=010\r".to_vec());

        let mut session = Session::new(mock);
        let resp = session.send(Command::QueryPosition).unwrap();
        assert_eq!(resp, Response::Position { az: 45, el: 10 });
    }

    #[test]
    fn session_keep_alive() {
        let mut mock = MockTransport::new();
        mock.enqueue_response(b"+\r".to_vec());

        let mut session = Session::new(mock);
        let resp = session.send(Command::KeepAlive).unwrap();
        assert_eq!(resp, Response::Ack);

        let t = session.into_transport();
        assert_eq!(&t.written, b"?\r");
    }

    #[test]
    fn session_timeout_on_empty_read() {
        let mock = MockTransport::new(); // no responses queued
        let mut session = Session::new(mock);
        let err = session.send(Command::QueryPosition).unwrap_err();
        assert!(matches!(err, Error::Timeout));
    }
}
