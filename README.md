# Relay Controller

## Description
This project is for an ESP32C3, it monitors the current flow to a pump & when the current flow is below a threshold it will turn off a relay for a specified time period. This is to stop a pump cycling on & off continuously due to water hammer.

## Hardware
- ESP32C3
- Relay Module
- Current Sensor (split core)

## Software
- esp-hal for ESP32C3
- Rust

## Usage
1. Clone the repository.
2. Install the required Rust toolchain for ESP32C3.
3. Source the rust environment, build & flash:
```bash
. $HOME/export-esp.sh
cargo run --release
```