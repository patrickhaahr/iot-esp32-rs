# Build & Test Commands
- **Build:** `cargo build` (target: xtensa-esp32-none-elf)
- **Flash & Monitor:** `cargo run` (uses espflash via .cargo/config.toml)
- **Lint:** `cargo clippy` (fix with `cargo clippy --fix`)
- **Format:** `cargo fmt`
- **Test:** `cargo test`
- **Single Test:** `cargo test -- <test_name>`

# Code Style & Guidelines
- **Environment:** `#![no_std]` and `#![no_main]` are required.
- **Imports:** Group `core` first, then external crates (`esp_hal`, `log`), then internal modules.
- **Formatting:** Strict adherence to `rustfmt`.
- **Naming:** snake_case for variables/functions, PascalCase for types.
- **Logging:** Use `log::info!`, `warn!`, etc. (initialized via `esp_println`).
- **Error Handling:** Prefer `Result` over panics. Avoid `unwrap()` unless initialization is critical.
- **Peripherals:** Initialize `esp_hal::init` and configure pins/clocks early in `main`.
- **Async/Tasks:** If using async, ensure executor is properly set up (currently using sync main).
- **Memory:** Be mindful of stack usage; avoid large allocations.
