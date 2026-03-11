//! Send the rotator to the home position (AZ=0, EL=0) over a serial port.
//!
//! Usage:
//!   cargo run --example serial_home -- /dev/ttyUSB0 9600

use easycom::{Command, Response, Session, Transport};
use serialport::SerialPort;
use std::io;
use std::time::Duration;

struct SerialTransport(Box<dyn SerialPort>);

impl Transport for SerialTransport {
    type Error = io::Error;

    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        use std::io::Write;
        self.0.write_all(frame)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use std::io::Read;
        self.0.read(buf)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let port_name = args.get(1).map(String::as_str).unwrap_or("/dev/ttyUSB0");
    let baud: u32 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(9600);

    let port = serialport::new(port_name, baud)
        .timeout(Duration::from_secs(2))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open {port_name}: {e}");
            std::process::exit(1);
        });

    let mut session = Session::new(SerialTransport(port));

    println!("Stopping any ongoing movement...");
    match session.send(Command::Stop) {
        Ok(_) => println!("Stopped."),
        Err(e) => eprintln!("Stop failed: {e}"),
    }

    println!("Sending to home position (AZ=000, EL=000)...");
    match session.send(Command::AzimuthElevation { az: 0, el: 0 }) {
        Ok(Response::Ack) => println!("Homing command accepted."),
        Ok(Response::Error) => eprintln!("Device rejected the command."),
        Ok(other) => println!("Unexpected response: {other:?}"),
        Err(e) => eprintln!("Error: {e}"),
    }

    println!("Querying position...");
    match session.send(Command::QueryPosition) {
        Ok(Response::Position { az, el }) => println!("Position: AZ={az:03} EL={el:03}"),
        Ok(other) => println!("Unexpected response: {other:?}"),
        Err(e) => eprintln!("Error: {e}"),
    }
}
