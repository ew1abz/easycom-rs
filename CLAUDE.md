# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Build
cargo build

# Run all tests
cargo test

# Run a single test
cargo test <test_name>           # e.g. cargo test encode_azimuth
cargo test <module>::tests::<name>  # e.g. cargo test framing::tests::encode_azimuth

# Lint
cargo clippy -- -D warnings

# Build examples
cargo build --examples

# Run an example
cargo run --example serial_home -- /dev/ttyUSB0 9600
cargo run --example tcp_poll -- 192.168.1.100:4533 5
cargo run --example cli -- tcp 192.168.1.100:4533

# Check no_std compatibility
cargo build --no-default-features
cargo build --no-default-features --features alloc

# Lint markdown
markdownlint spec/*.md
```

## Architecture

The library is a synchronous, `no_std`-compatible implementation of the Easycom rotator control protocol (ASCII, CR-terminated, request/response).

**Data flow:** `Command` → `framing::encode_into` → bytes over `Transport` → bytes → `framing::Parser`/`decode` → `Response`

**Core modules:**

- `command.rs` — `Command` and `Response` enums. All protocol-level types live here.
- `framing.rs` — stateless `encode`/`encode_into`/`decode` functions plus the stateful `Parser` (fixed `[u8; 64]` buffer, byte-at-a-time). This is the protocol heart.
- `transport.rs` — `Transport` trait (write + read). `MockTransport` (std-only, `VecDeque`-backed) is the test double used across all session tests.
- `session.rs` — `Session<T>` composes a `Transport` + `Parser`. Its `send()` encodes, writes, then reads byte-by-byte until `Parser` emits a frame, with built-in echo suppression (skips frames identical to the sent command).
- `error.rs` — `Error<E>` (`#[non_exhaustive]`) wrapping transport, parse, timeout, and invalid-param errors.

**`no_std` strategy:** `encode_into` writes into a caller-supplied `[u8; 32]`; `std` feature adds the `encode` wrapper returning `Vec<u8>`. `Parser` uses a fixed internal buffer. The `alloc` feature enables heap usage without full `std`.

**Feature flags:** `std` (default), `alloc`. Disable `std` for embedded targets.

**Examples** (`examples/`) each define their own `Transport` wrapper inline (serial via `serialport` crate, TCP via `std::net::TcpStream`) — there are no built-in concrete transport implementations in the library itself.
