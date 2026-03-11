//! Connect to a TCP Easycom adapter and poll the rotator position.
//!
//! Usage:
//!   cargo run --example tcp_poll -- 192.168.1.100:4533 5

use easycom::{Command, Response, Session, Transport};
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::thread;
use std::time::Duration;

struct TcpTransport(TcpStream);

impl Transport for TcpTransport {
    type Error = io::Error;

    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.0.write_all(frame)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let addr = args.get(1).map(String::as_str).unwrap_or("127.0.0.1:4533");
    let interval_secs: u64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    let stream = TcpStream::connect(addr).unwrap_or_else(|e| {
        eprintln!("Failed to connect to {addr}: {e}");
        std::process::exit(1);
    });
    stream
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();

    let mut session = Session::new(TcpTransport(stream));

    println!("Polling {addr} every {interval_secs}s. Press Ctrl-C to stop.");
    loop {
        match session.send(Command::QueryPosition) {
            Ok(Response::Position { az, el }) => {
                println!("AZ={az:03}  EL={el:03}");
            }
            Ok(Response::Error) => eprintln!("Device error"),
            Ok(other) => eprintln!("Unexpected: {other:?}"),
            Err(e) => {
                eprintln!("Error: {e}");
                break;
            }
        }
        thread::sleep(Duration::from_secs(interval_secs));
    }
}
