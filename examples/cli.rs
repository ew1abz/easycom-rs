//! Interactive command-line interface for an Easycom rotator.
//!
//! Connects over TCP (default) or serial and lets you type commands manually.
//!
//! Usage:
//!   cargo run --example cli -- tcp 192.168.1.100:4533
//!   cargo run --example cli -- serial /dev/ttyUSB0 9600
//!
//! Commands:
//!   az <0-360>          Set azimuth
//!   el <0-180>          Set elevation
//!   pos <az> <el>       Set azimuth and elevation
//!   query               Query current position
//!   stop                Stop movement
//!   keepalive           Send keep-alive ping
//!   help                Show this help
//!   quit                Exit

use easycom::{Command, Response, Session, Transport};
use std::io::{self, BufRead, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

// ── Transport wrappers ────────────────────────────────────────────────────────

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

#[cfg(feature = "std")]
struct SerialTransport(Box<dyn serialport::SerialPort>);

#[cfg(feature = "std")]
impl Transport for SerialTransport {
    type Error = io::Error;
    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.0.write_all(frame)
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

// ── REPL ──────────────────────────────────────────────────────────────────────

fn run_repl<T: Transport>(mut session: Session<T>)
where
    T::Error: std::fmt::Display,
{
    let stdin = io::stdin();
    print_help();
    print!("> ");
    io::stdout().flush().ok();

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                eprintln!("Read error: {e}");
                break;
            }
        };

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            print!("> ");
            io::stdout().flush().ok();
            continue;
        }

        let cmd = match parts[0] {
            "az" => {
                let n: u16 = parse_arg(&parts, 1, "az <0-360>");
                Command::Azimuth(n)
            }
            "el" => {
                let n: u16 = parse_arg(&parts, 1, "el <0-180>");
                Command::Elevation(n)
            }
            "pos" => {
                let az: u16 = parse_arg(&parts, 1, "pos <az> <el>");
                let el: u16 = parse_arg(&parts, 2, "pos <az> <el>");
                Command::AzimuthElevation { az, el }
            }
            "query" => Command::QueryPosition,
            "stop" => Command::Stop,
            "keepalive" => Command::KeepAlive,
            "help" => {
                print_help();
                print!("> ");
                io::stdout().flush().ok();
                continue;
            }
            "quit" | "exit" | "q" => break,
            other => {
                eprintln!("Unknown command: {other}. Type 'help'.");
                print!("> ");
                io::stdout().flush().ok();
                continue;
            }
        };

        match session.send(cmd) {
            Ok(Response::Ack) => println!("OK"),
            Ok(Response::Position { az, el }) => println!("AZ={az:03}  EL={el:03}"),
            Ok(Response::Error) => println!("Device error (?)"),
            Err(e) => eprintln!("Error: {e}"),
        }

        print!("> ");
        io::stdout().flush().ok();
    }
}

fn parse_arg<T: std::str::FromStr>(parts: &[&str], idx: usize, usage: &str) -> T {
    parts
        .get(idx)
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| {
            eprintln!("Usage: {usage}");
            std::process::exit(1);
        })
}

fn print_help() {
    println!(
        "Commands: az <n>  el <n>  pos <az> <el>  query  stop  keepalive  help  quit"
    );
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mode = args.get(1).map(String::as_str).unwrap_or("tcp");

    match mode {
        "tcp" => {
            let addr = args.get(2).map(String::as_str).unwrap_or("127.0.0.1:4533");
            let stream = TcpStream::connect(addr).unwrap_or_else(|e| {
                eprintln!("Failed to connect to {addr}: {e}");
                std::process::exit(1);
            });
            stream
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            println!("Connected to {addr}");
            run_repl(Session::new(TcpTransport(stream)));
        }
        "serial" => {
            let port_name = args.get(2).map(String::as_str).unwrap_or("/dev/ttyUSB0");
            let baud: u32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(9600);
            let port = serialport::new(port_name, baud)
                .timeout(Duration::from_secs(2))
                .open()
                .unwrap_or_else(|e| {
                    eprintln!("Failed to open {port_name}: {e}");
                    std::process::exit(1);
                });
            println!("Opened {port_name} at {baud} baud");
            run_repl(Session::new(SerialTransport(port)));
        }
        other => {
            eprintln!("Unknown mode: {other}. Use 'tcp' or 'serial'.");
            std::process::exit(1);
        }
    }
}
