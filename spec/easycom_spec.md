# Easycom-rs Specification

This document describes the design of a Rust library implementing the Easycom
protocol, a variant of the Yaesu GS-232A/B antenna rotor control protocol.
The goal is to produce an idiomatic, extensible, and testable crate suitable
for both desktop and embedded use.

---

## 1. Background & Context

### 1.1 Protocol Overview

Easycom is a simple ASCII-based command/response protocol originally used to
control ham radio antenna rotators and other gear. It is derived from the
Yaesu GS-232A/B standard but differs in framing, parameter sets, and
extensions for network use. Commands are sent as bytes terminated by a `\r` or
`\n` and responses echo the command followed by status data.

### 1.2 Use Cases

- Desktop applications controlling transceivers or rotators via USB-to-serial
  adapters.
- Embedded firmware communicating over UART with Yaesu-compatible hardware.
- Remote control via TCP/IP or WebSocket adapters implementing Easycom.

### 1.3 Protocol Characteristics

- ASCII encoding, magnetic carriage returns and optional newlines.
- Checksum is optional but common on serial links.
- Commands are typically one ASCII character followed by optional parameters.
- The protocol is synchronous; a response is expected for each command.
- Timeouts and error conditions (e.g. `?` response for invalid command) must
  be handled.

---

## 2. Architecture

### 2.1 Crate Layout

```text
crate: easycom-rs
├── src/lib.rs         # public API re-exports
├── framing.rs        # message framing/parsing
├── command.rs        # command/response types
├── transport.rs      # traits & implementations for I/O
├── error.rs          # error definitions
└── tests/            # unit and integration tests
```

### 2.2 Core Traits

```rust
pub trait Transport {
    type Error;
    fn write(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
}
```

The crate will provide `SerialTransport` (wrapping `embedded_hal::serial`) and a
`TcpTransport` for networked adapters.

Parsing and framing is separated into its own module with functions to encode
commands and decode responses. A `Parser` struct can be used for incremental
input.

### 2.3 Data Structures

- `enum Command { Azimuth(u16), Elevation(u16), Stop, ... }`
- `enum Response { Ack, Position { az: u16, el: u16 }, Error(String) }`

Builders and newtypes ensure only valid values are constructed.

---

## 3. API Requirements

### 3.1 Feature Flags

- `std` (default) vs `no_std` for embedded environments.
- `alloc` for heap usage.

### 3.2 High-Level Interface

Provide an async-friendly `Session<T: Transport>` struct:

```rust
pub struct Session<T> { transport: T, parser: Parser }

impl<T: Transport> Session<T> {
    pub fn new(transport: T) -> Self { ... }

    pub fn send(&mut self, cmd: Command) -> Result<Response, Error<T::Error>> {
        ...
    }
}
```

Support optional futures (`async-std`/`tokio`) behind cargo features.

### 3.3 Error Handling

Errors encapsulate transport errors, parse errors, timeouts, and invalid
parameters. Use a `#[non_exhaustive]` `enum Error`.

### 3.4 Examples

- Opening a serial port and sending a `Home` command.
- Connecting to a TCP adapter and polling status.

Example code shall be included in doc comments and README.

---

## 4. Protocol Details

### 4.1 Command Set

List all easycom commands (A, B, C, ...), parameters ranges and meanings.

### 4.2 Encoding Rules

- Commands encoded as ASCII digits/letters; numbers padded with leading zeros.
- Frame start: optional `STX`; terminator: `CR` (0x0D).
- Checksum: simple XOR of payload followed by `*XX` ASCII hex when enabled.

### 4.3 Special Handling

- Echo suppression: if the device repeats the command, library may ignore it.
- Keep-alive: send `?` periodically to maintain connection.

---

## 5. Testing Strategy

- **Unit tests**: each command's encoder and parser; invalid frames; edge cases.
- **Mock transport** implementing `Transport` that records writes and returns
  predefined responses.
- **Integration tests**: use `std::net::TcpListener` to simulate an adapter.
- Fuzz properties using `quickcheck`/`proptest` to ensure parser robustness.

---

## 6. Documentation & Examples

- README with quick-start code and explanation of features.
- Doc comments for every public type/function, with `rustdoc` examples.
- Example `cli.rs` binary demonstrating manual control via command line.

---

## 7. Extensibility

- Support for additional protocol variants by adding features or new command
  enums.
- Maintain compatibility with GS-232A by reusing frame/parsing logic and
  providing a `gs232` feature.
- Use semantic versioning; bump minor version for protocol extensions.

---

## 8. Optional Extras

- `cli` crate within workspace for a command-line interface.
- `embedded-hal` support via generic serial transport.
- Bindings/ffi for languages such as Python or C if requested.

---

*End of specification.*

This spec can be distributed alongside the library to guide contributors and
ensure implementation completeness.
