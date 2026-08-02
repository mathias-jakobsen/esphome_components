# ESPHome Integration for Wavin AHC9000 / Jablotron AC-116 Underfloor Heating Controller

A modern, highly optimized, and feature-complete ESPHome integration for the **Wavin AHC9000** and **Jablotron AC-116** underfloor heating control units.

This component implements non-blocking Modbus RTU communication over RS-485 using Wavin's custom function codes, natively supporting **ESPHome Sub-Devices** (`esphome: devices:`), multi-channel **Zone Consolidation**, auto-detection of paired thermostats, battery diagnostics, and signal strength (RSSI) monitoring.

---

## Key Features

- **Asynchronous & Non-Blocking Modbus RTU**: Custom low-level C++ state machine operating at 38400 baud (8N1). Gracefully handles communication timeouts and offline/lost thermostats without blocking serial polling cycles.
- **Wavin Protocol Function Codes**: Full support for non-standard function codes `0x43` (Read Register), `0x44` (Write Register), `0x45` (Masked Write Index), and `0x46` (Masked Write Address).
- **ESPHome Native Sub-Devices Architecture**: Exposes each physical room/zone as a separate Home Assistant device tied to Home Assistant Areas directly via YAML (`device_id`).
- **Zone Consolidation & Sync Group Support**: Multi-channel rooms (where multiple heating loops are bound to one thermostat or room) are consolidated into **one unified climate entity** with aggregate heating demand monitoring.
- **Auto-Detection Channel Discovery**: 16 root-level diagnostic binary sensors automatically indicate which Wavin channels have active paired thermostats.
- **Comprehensive Sub-Device Diagnostics**:
  - **Climate Entity**: Target temperature control, current room temperature, HVAC action (`HEATING` / `IDLE`), mode (`HEAT` / `OFF`).
  - **Battery Percentage Sensor**: 0–100% battery level status.
  - **Low Battery Binary Sensor**: Early warning diagnostic for low thermostat battery.
  - **Thermostat Offline/Lost Binary Sensor**: Real-time diagnostic when a thermostat stops communicating.
  - **Wireless Signal Strength (RSSI)**: Exact dBm signal strength values for RF link troubleshooting.
  - **Heating Demand Binary Sensor**: Monitors whether the underfloor heating loop/actuator valve is currently open and demanding heat.

---

## Hardware & RS-485 Wiring

The Wavin AHC9000 / Jablotron AC-116 controller provides an **RJ45 port** for RS-485 Modbus RTU communication. 

### RS-485 to RJ45 Pinout

Using a standard Ethernet patch cable (T568B standard wiring):

| RJ45 Pin | T568B Wire Color | Function | RS-485 Adapter Connection |
| :--- | :--- | :--- | :--- |
| **Pin 3** | Green / White | **RS-485 B (D-)** | Connect to **B-** / **Data-** |
| **Pin 4** | Blue | **RS-485 A (D+)** | Connect to **A+** / **Data+** |
| **Pin 1** or **Pin 8** | Orange/White or Brown | **GND** | Connect to **GND** (Common Ground) |

> [!IMPORTANT]
> - **Serial Bus Settings**: 38400 baud, 8 data bits, 1 stop bit, no parity (8N1).
> - **Flow Control**: Auto-direction RS-485 modules (e.g. MAX13487 or XY-017) require no extra GPIO. For standard MAX485 boards with DE/RE pins, un-comment `hub->set_flow_control_pin()` in `wavin.yaml`.

---

## Recommended Deployment Workflow

### Step 1: Deploy Base Gateway Configuration
Flash your ESP32 or ESP8266 node with the base `wavin.yaml` configuration.

### Step 2: Auto-Discover Active Channels
Open **Home Assistant** -> **Devices & Services** -> **ESPHome** -> **Wavin AHC9000 Gateway**.
Check the **16 Root Diagnostic Binary Sensors** (`Wavin Channel 1 Paired Status` through `Wavin Channel 16 Paired Status`):
- **ON (Connected)**: Indicates a physical thermostat is actively paired to this channel (Category 0x01 Element Address is non-zero).
- **OFF (Unpaired)**: Channel is empty.

### Step 3: Configure Sub-Devices & Zone Consolidation
Edit `wavin.yaml` to define your physical rooms as sub-devices in `esphome: devices:` and map their respective climate and diagnostic entities:

```yaml
esphome:
  name: wavin-ahc9000-gateway
  includes:
    - wavin_ahc9000.h

  devices:
    - id: subdev_living_room
      name: "Living Room Underfloor Heating"
      area: "Living Room"
    - id: subdev_kitchen
      name: "Kitchen Underfloor Heating"
      area: "Kitchen"

# ... (UART and custom_component configuration) ...

# Single Channel Room (e.g. Kitchen = Channel 3)
climate:
  - platform: custom
    lambda: |-
      auto *c = new wavin_ahc9000::WavinZoneClimate(id(wavin_hub), {3});
      App.register_component(c);
      return {c};
    climates:
      - name: "Kitchen Climate"
        id: climate_kitchen
        device_id: subdev_living_room

# Consolidated Multi-Loop Zone (e.g. Living Room = Channels 1 & 2)
  - platform: custom
    lambda: |-
      auto *c = new wavin_ahc9000::WavinZoneClimate(id(wavin_hub), {1, 2});
      App.register_component(c);
      return {c};
    climates:
      - name: "Living Room Climate"
        id: climate_living_room
        device_id: subdev_living_room
```

---

## Exposed Entities Reference

### Root Gateway Device (Parent ESP Node)
- **Binary Sensors**: `Wavin Channel 1 Paired Status` .. `Wavin Channel 16 Paired Status` (Diagnostic auto-detection).

### Room Sub-Devices
Each sub-device registered in YAML exposes the following entities:

| Entity Type | Platform | Description | Values / Range |
| :--- | :--- | :--- | :--- |
| **Climate** | `climate` | Primary zone thermostat control | 5.0 °C – 35.0 °C, Modes: `HEAT`, `OFF`, Action: `HEATING`, `IDLE` |
| **Battery Level** | `sensor` | Thermostat battery percentage | 0 % – 100 % (10% increments per Wavin spec) |
| **Signal Strength** | `sensor` | Wireless RF link quality | dBm (calculated from `RSSIEL` / `RSSICU` registers) |
| **Low Battery Alert** | `binary_sensor` | Diagnostic battery warning | `ON` (Low Battery) / `OFF` (Normal) |
| **Thermostat Offline** | `binary_sensor` | Diagnostic connection status | `ON` (Lost / Offline) / `OFF` (Online) |
| **Heating Demand** | `binary_sensor` | Actuator valve status | `ON` (Demand Heat / Valve Open) / `OFF` (Idle) |

---

## Technical Modbus Protocol Specification

The Wavin AHC9000 / Jablotron AC-116 controller uses a proprietary Modbus RTU register mapping organized into **Categories**, **Pages**, and **Indexes**:

- **Category 0x01 (ELEMENTS)**: 48 pages (elements 1..48). Reg 0x00 & 0x01 store 32-bit physical address, Reg 0x04 stores air temperature (0.1 °C), Reg 0x05 stores floor temperature, Reg 0x08 stores status bits (ALIVE, LOST, LOW BATT), Reg 0x09 stores RSSI, Reg 0x0A stores battery level (0-10), Reg 0x0B stores Sync Group.
- **Category 0x02 (PACKED DATA)**: 17 pages (channels 1..16). Reg 0x00 stores manual setpoint (0.1 °C), Reg 0x07 stores configuration & mode bits.
- **Category 0x03 (CHANNELS)**: 17 pages (channels 1..16). Reg 0x00 stores Timer Event (`OUTP ON` bit indicates active valve output), Reg 0x02 stores Primary Element index.

### Custom Function Codes
- `0x43`: Read Register from Category/Page/Index.
- `0x44`: Write Register to Category/Page/Index.
- `0x45`: Masked Write to Category/Page/Index (used for updating mode bits safely without altering child lock).
- `0x46`: Masked Write to Address.

---

## License

MIT License. Designed for ESPHome & Home Assistant.
