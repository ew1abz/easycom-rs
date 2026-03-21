# easycom

A Rust library implementing the **Easycom** antenna rotator control protocol,
supporting the Yaesu GS-232A/B command set, Easycomm II, and Easycomm III.

Works on `std` targets (desktop, server) and `no_std` targets (embedded firmware).

## Features

- Encode and decode all standard Easycom commands
- Optional XOR checksum support
- Incremental `Parser` for byte-at-a-time UART feeds
- Generic `Transport` trait — plug in serial, TCP, or any I/O channel
- Echo suppression and keep-alive (`?`) handling
- `no_std` compatible with a fixed-size stack buffer

## Quick start

Add to `Cargo.toml`:

```toml
[dependencies]
easycom = "0.1"
```

### Query position over TCP

```rust
use easycom::{Session, Command, Response, Transport};
use std::io::{self, Read, Write};
use std::net::TcpStream;

struct TcpTransport(TcpStream);

impl Transport for TcpTransport {
    type Error = io::Error;
    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error> { self.0.write_all(frame) }
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> { self.0.read(buf) }
}

fn main() {
    let stream = TcpStream::connect("192.168.1.100:4533").unwrap();
    let mut session = Session::new(TcpTransport(stream));

    match session.send(Command::QueryPosition).unwrap() {
        Response::Position { az, el } => println!("AZ={az:03} EL={el:03}"),
        Response::Status(status) => println!("Status: {status:?}"),
        Response::StatusRegister(val) => println!("Status register: {val}"),
        Response::ErrorRegister(val) => println!("Error register: {val}"),
        Response::ConfigValue { register, value } => println!("CR{register}={value}"),
        Response::AzimuthPosition(az) => println!("AZ={az:03}"),
        Response::ElevationPosition(el) => println!("EL={el:03}"),
        Response::Error => eprintln!("Device error"),
        Response::Ack => {}
    }
}
```

### Send rotator to home position over serial

```rust,no_run
use easycom::{Session, Command};
// (SerialTransport setup omitted — see examples/serial_home.rs)

session.send(Command::Stop).unwrap();
session.send(Command::AzimuthElevation { az: 0, el: 0 }).unwrap();
```

## Supported protocols

The library understands three wire protocols and accepts commands from any of
them interchangeably. Responses are encoded in the format matching the original query.

### GS-232A/B (Yaesu)

| Variant | Wire frame | Description |
|---|---|---|
| `Azimuth(az)` | `Annn\r` | Set azimuth (0–360°) |
| `Elevation(el)` | `Ennn\r` | Set elevation (0–180°) |
| `AzimuthElevation { az, el }` | `Wnnn nnn\r` | Set both simultaneously |
| `QueryPosition` | `C\r` | Query current position |
| `QueryStatus` | `GS\r` | Query device status (returns `ST=…`) |
| `Stop` | `S\r` | Stop all movement |
| `KeepAlive` | `?\r` | Keep-alive ping |
| `Offset { az, el }` | `O±nnn±nnn\r` | Relative move |

### Easycomm II

| Variant | Wire frame | Description |
|---|---|---|
| `Azimuth(az)` | `AZnnn.n\n` | Set azimuth (0–360°) |
| `Elevation(el)` | `ELnnn.n\n` | Set elevation (0–180°) |
| `AzimuthElevation { az, el }` | `AZnnn.n ELnnn.n\n` | Set both simultaneously |
| `QueryAzimuth` | `AZ\n` | Query current azimuth |
| `QueryElevation` | `EL\n` | Query current elevation |
| `Stop` | `SA SE\n` | Stop all axes |

Easycomm II is the protocol used by **hamlib** (`rotctl -m 204`).

### Easycomm III

| Variant | Wire frame | Description |
|---|---|---|
| `VelocityLeft(speed)` | `VL<speed>\n` | Velocity left (mdeg/s) |
| `VelocityRight(speed)` | `VR<speed>\n` | Velocity right (mdeg/s) |
| `VelocityUp(speed)` | `VU<speed>\n` | Velocity up (mdeg/s) |
| `VelocityDown(speed)` | `VD<speed>\n` | Velocity down (mdeg/s) |
| `GetStatusRegister` | `GS\n` | Get status register (bitmask) |
| `GetErrorRegister` | `GE\n` | Get error register (bitmask) |
| `ReadConfig(reg)` | `CR<reg>\n` | Read configuration register |
| `WriteConfig { register, value }` | `CW<reg>,<val>\n` | Write configuration register |
| `Reset` | `RESET\n` | Reset device |
| `Park` | `PARK\n` | Move to park position |

Easycomm III extends II with velocity control, device status/error registers,
and configuration read/write. Used by **SatNOGS** rotator firmware.

## Feature flags

| Flag | Default | Description |
|---|---|---|
| `std` | yes | Enables `Vec`-based API and `MockTransport` |
| `alloc` | no | Heap support without full `std` |

Disable default features for `no_std`:

```toml
[dependencies]
easycom = { version = "0.1", default-features = false }
```

## Examples

```sh
# Send rotator to home via serial
cargo run --example serial_home -- /dev/ttyUSB0 9600

# Poll position from a TCP adapter every 5 seconds
cargo run --example tcp_poll -- 192.168.1.100:4533 5

# Interactive CLI (TCP or serial)
cargo run --example cli -- tcp 192.168.1.100:4533
cargo run --example cli -- serial /dev/ttyUSB0 9600
```

## Publishing to crates.io

1. Update `authors`, `repository`, and `homepage` in `Cargo.toml`
2. Log in: `cargo login`
3. Dry run: `cargo publish --dry-run`
4. Publish: `cargo publish`

## License

MIT — see [LICENSE](LICENSE)
