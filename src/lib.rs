//! # easycom
//!
//! A Rust library implementing the Easycom protocol, a variant of the Yaesu
//! GS-232A/B antenna rotor control protocol.
//!
//! ## Quick start (std)
//!
//! ```rust,no_run
//! use easycom::{Session, Command, Response};
//! use easycom::transport::MockTransport;
//!
//! let mut transport = MockTransport::new();
//! transport.enqueue_response(b"AZ=270 EL=045\r".to_vec());
//!
//! let mut session = Session::new(transport);
//! match session.send(Command::QueryPosition).unwrap() {
//!     Response::Position { az, el } => println!("AZ={az} EL={el}"),
//!     Response::Ack => println!("ack"),
//!     Response::Error => println!("device error"),
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod command;
pub mod error;
pub mod framing;
pub mod session;
pub mod transport;

pub use command::{Command, Response};
pub use error::Error;
pub use framing::CommandParser;
pub use session::Session;
pub use transport::Transport;
