# relay_controller

ESP32-C3 firmware in Rust that monitors pump current via a split-core sensor and pulses a relay to prevent water hammer when flow drops below threshold — async Embassy tasks, no RTOS overhead.

## Key Technologies

- **MCU:** ESP32-C3
- **Language:** Rust 2021
- **Framework:** esp-hal 1.0.0 + Embassy (async executor, timers, GPIO)
- **Sensor:** Split-core current sensor on ADC GPIO0
- **Build:** Cargo + ESP-IDF toolchain (`export-esp.sh`)

## How It Works

ADC samples the current sensor every 200 ms. When current rises above `ON_THRESHOLD` (2400) the pump is flagged as running. When it then drops below `OFF_THRESHOLD` (1800), a relay pulse task fires for 5 seconds — creating a settling period that prevents rapid cycling and mechanical stress.

Two Embassy tasks run concurrently: ADC monitoring signals the relay task via a lock-free `Signal` primitive.

## Getting Started

```bash
. $HOME/export-esp.sh
cargo run --release
```

Thresholds and relay GPIO pin are configured in `src/bin/main.rs`.

---

Built by Owen O'Hehir — embedded Linux, IoT, Matter & Rust consulting at [electronicsconsult.com](https://electronicsconsult.com). Available for contract and consulting work.
