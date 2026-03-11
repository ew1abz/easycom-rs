# easycom

A Rust library implementing the **Easycom** antenna rotator control protocol,
a variant of the Yaesu GS-232A/B standard.

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

## Commands

| Variant | Wire frame | Description |
|---|---|---|
| `Azimuth(az)` | `Annn\r` | Set azimuth (0–360°) |
| `Elevation(el)` | `Ennn\r` | Set elevation (0–180°) |
| `AzimuthElevation { az, el }` | `Wnnn nnn\r` | Set both simultaneously |
| `QueryPosition` | `C\r` | Query current position |
| `Stop` | `S\r` | Stop all movement |
| `KeepAlive` | `?\r` | Keep-alive ping |
| `Offset { az, el }` | `O±nnn±nnn\r` | Relative move |

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
