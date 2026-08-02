# Wavin AHC9000 / Jablotron AC-116 ESPHome Component

Production ESPHome External Component for integration with **Wavin AHC 9000** and **Jablotron AC-116** Underfloor Heating Controllers via RS485 Modbus RTU interface.

Supports modern ESPHome (2024+/2026+) architecture, including:
- Native **Sub-Devices** for room/zone separation in Home Assistant
- High-efficiency **Modbus RTU Broker Engine** (custom Function Codes 0x43, 0x44, 0x45)
- Automated **Area assignment**
- Diagnostic channel paired status binary sensors
- Climate entities with heating demand, battery percentage, RSSI signal strength, and offline status.

## Installation & Configuration Example

In your :



## Repository Structure



## Features

- **Sub-Device Architecture**: Physical thermostats are grouped into their respective Home Assistant device pages under designated Areas.
- **Smart Modbus Polling**: Automatically filters out unpaired channels (9-16) from detailed packet polling to eliminate Modbus timeouts and optimize bus throughput.
- **Non-blocking Execution**: Full async C++ architecture designed for ESP8266 & ESP32.
