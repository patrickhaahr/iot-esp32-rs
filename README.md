# IoT ESP32 Rust Project

A Rust-based embedded project for the ESP32 using `esp-hal`.

## Development

To build, upload the code, and monitor the ESP32 serial output:

```sh
cargo run --release
```

## Documentation

The project documentation is written in [Typst](https://typst.app/).

### Building Documentation

Ensure you have `typst` installed.

To compile the logbook or portfolio:

```sh
# Compile a specific file
typst compile docs/logbog.typst
typst compile docs/arbejds-portfolio.typst
```

### Watch Mode

To automatically recompile when the file changes:

```sh
typst watch docs/logbog.typst docs/logbog.pdf
```
