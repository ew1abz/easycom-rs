//! # easycom
//!
//! A Rust library implementing the Easycom antenna rotator control protocol,
//! supporting the Yaesu GS-232A/B command set, Easycomm II, and Easycomm III.
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
//!     Response::AzimuthPosition(az) => println!("AZ={az}"),
//!     Response::ElevationPosition(el) => println!("EL={el}"),
//!     Response::Status(status) => println!("status: {status:?}"),
//!     Response::StatusRegister(val) => println!("status register: {val}"),
//!     Response::ErrorRegister(val) => println!("error register: {val}"),
//!     Response::ConfigValue { register, value } => println!("CR{register}={value}"),
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

pub use command::{Command, DeviceStatus, Response};
pub use error::Error;
pub use framing::{CommandParser, decode_command};
pub use session::Session;
pub use transport::Transport;
