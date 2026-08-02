#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <cmath>

namespace esphome {
namespace wavin_ahc9000 {

static const char *const TAG = "wavin_ahc9000";

// Wavin Modbus Non-Standard Function Codes
static const uint8_t WAVIN_FC_READ_REGISTER           = 0x43;
static const uint8_t WAVIN_FC_WRITE_REGISTER          = 0x44;
static const uint8_t WAVIN_FC_WRITE_MASKED_INDEX     = 0x45;
static const uint8_t WAVIN_FC_WRITE_MASKED_ADDRESS   = 0x46;

// Wavin Modbus Register Categories
static const uint8_t CAT_MAIN                         = 0x00;
static const uint8_t CAT_ELEMENTS                     = 0x01;
static const uint8_t CAT_PACKED                       = 0x02;
static const uint8_t CAT_CHANNELS                     = 0x03;
static const uint8_t CAT_RELAYS                       = 0x04;
static const uint8_t CAT_CLOCK                        = 0x05;
static const uint8_t CAT_SCHEDULES                    = 0x06;
static const uint8_t CAT_INFO                         = 0x07;

// Category 0x01 (ELEMENTS) Register Indexes
static const uint8_t ELEM_REG_ADDRESS_L               = 0x00;
static const uint8_t ELEM_REG_ADDRESS_H               = 0x01;
static const uint8_t ELEM_REG_AIR_TEMP                = 0x04;
static const uint8_t ELEM_REG_FLOOR_TEMP              = 0x05;
static const uint8_t ELEM_REG_HUMIDITY                = 0x07;
static const uint8_t ELEM_REG_STATUS                  = 0x08;
static const uint8_t ELEM_REG_RSSI                    = 0x09;
static const uint8_t ELEM_REG_BATTERY                 = 0x0A;
static const uint8_t ELEM_REG_SYNC_GROUP              = 0x0B;

// Category 0x02 (PACKED DATA) Register Indexes
static const uint8_t PACKED_REG_MANUAL_TEMP           = 0x00;
static const uint8_t PACKED_REG_COMFORT_TEMP          = 0x01;
static const uint8_t PACKED_REG_ECO_TEMP              = 0x02;
static const uint8_t PACKED_REG_CONFIGURATION         = 0x07;
static const uint8_t PACKED_REG_DESIRED_TEMP          = 0x10;

// Category 0x03 (CHANNELS) Register Indexes
static const uint8_t CH_REG_TIMER_EVENT               = 0x00;
static const uint8_t CH_REG_PRIMARY_ELEMENT           = 0x02;

// Bitmasks
static const uint16_t ELEM_STATUS_ALIVE_MASK          = 0x8000; // Bit 15
static const uint16_t ELEM_STATUS_LOST_MASK           = 0x0800; // Bit 11
static const uint16_t ELEM_STATUS_LOW_BATT_MASK       = 0x0400; // Bit 10
static const uint16_t CH_TIMER_EVENT_OUTP_ON          = 0x0010; // Bit 4
static const uint16_t CH_PRIMARY_ELEM_ALL_LOST        = 0x0800; // Bit 11

// Compute Standard Modbus CRC16 (0xA001 polynomial)
inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Channel state structure maintained by central data broker
struct ChannelState {
  uint32_t address{0};
  bool paired{false};
  float air_temp{NAN};
  float floor_temp{NAN};
  float target_temp{20.0f};
  uint8_t battery_pct{0};
  bool low_battery{false};
  bool lost{false};
  float rssi{NAN};
  bool heating_active{false};
  uint8_t sync_group{0};
  uint8_t primary_element{0};
  climate::ClimateMode mode{climate::CLIMATE_MODE_HEAT};
  climate::ClimateAction action{climate::CLIMATE_ACTION_IDLE};
};

enum RequestType {
  REQ_READ_ELEMENT,
  REQ_READ_CHANNEL,
  REQ_READ_PACKED,
  REQ_WRITE_SETPOINT,
  REQ_WRITE_MODE
};

struct ModbusPacket {
  RequestType type;
  uint8_t channel; // 1..16
  uint8_t category;
  uint8_t index;
  uint8_t page;
  uint8_t quantity;
  std::vector<uint8_t> payload;
};


class WavinUpdatableEntity {
 public:
  virtual ~WavinUpdatableEntity() = default;
  virtual void update_state() = 0;
};

// Forward declaration
class WavinZoneClimate;
class WavinZoneBatterySensor;
class WavinZoneRSSISensor;
class WavinZoneLowBatterySensor;
class WavinZoneLostSensor;
class WavinZoneHeatingDemandSensor;

class WavinAHC9000Component : public PollingComponent, public uart::UARTDevice {
 public:
  WavinAHC9000Component(uart::UARTComponent *parent) : uart::UARTDevice(parent) {
    this->channel_paired_sensors_.fill(nullptr);
  }

  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_receive_timeout_ms(uint32_t timeout) { this->timeout_ms_ = timeout; }

  void set_channel_paired_sensor(uint8_t channel, binary_sensor::BinarySensor *sensor) {
    if (channel >= 1 && channel <= 16) {
      this->channel_paired_sensors_[channel - 1] = sensor;
    }
  }

  void register_updatable(WavinUpdatableEntity *e) { this->updatable_entities_.push_back(e); }

  const ChannelState &get_channel_data(uint8_t channel) const {
    static ChannelState empty_state;
    if (channel >= 1 && channel <= 16) {
      return this->channels_[channel - 1];
    }
    return empty_state;
  }

  void setup() override {
    ESP_LOGCONFIG(TAG, "Initializing Wavin AHC9000 Modbus RTU Component...");
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->setup();
      this->flow_control_pin_->digital_write(false);
    }
  }

  void update() override {
    // Queue polling requests for all 16 channels in background
    for (uint8_t ch = 1; ch <= 16; ch++) {
      uint8_t page = ch - 1;

      // 1. Read Category 0x01 (ELEMENTS) Page ch-1, Index 0x00, Quantity 12
      ModbusPacket p1;
      p1.type = REQ_READ_ELEMENT;
      p1.channel = ch;
      p1.category = CAT_ELEMENTS;
      p1.index = ELEM_REG_ADDRESS_L;
      p1.page = page;
      p1.quantity = 12;
      this->tx_queue_.push_back(p1);

      // 2. Read Category 0x03 (CHANNELS) Page ch-1, Index 0x00, Quantity 3
      ModbusPacket p2;
      p2.type = REQ_READ_CHANNEL;
      p2.channel = ch;
      p2.category = CAT_CHANNELS;
      p2.index = CH_REG_TIMER_EVENT;
      p2.page = page;
      p2.quantity = 3;
      this->tx_queue_.push_back(p2);

      // 3. Read Category 0x02 (PACKED DATA) Page ch-1, Index 0x00, Quantity 8
      ModbusPacket p3;
      p3.type = REQ_READ_PACKED;
      p3.channel = ch;
      p3.category = CAT_PACKED;
      p3.index = PACKED_REG_MANUAL_TEMP;
      p3.page = page;
      p3.quantity = 8;
      this->tx_queue_.push_back(p3);
    }
  }

  void loop() override {
    uint32_t now = millis();

    switch (this->fsm_state_) {
      case FSM_IDLE: {
        if (!this->tx_queue_.empty()) {
          this->current_packet_ = this->tx_queue_.front();
          this->tx_queue_.pop_front();
          this->send_packet_(this->current_packet_);
          this->last_request_time_ = now;
          this->rx_buffer_.clear();
          this->fsm_state_ = FSM_WAITING_RESPONSE;
        }
        break;
      }
      case FSM_WAITING_RESPONSE: {
        while (this->available()) {
          uint8_t b;
          this->read_byte(&b);
          this->rx_buffer_.push_back(b);
        }

        if (this->process_rx_buffer_()) {
          this->fsm_state_ = FSM_IDLE;
        } else if (now - this->last_request_time_ > this->timeout_ms_) {
          ESP_LOGW(TAG, "Timeout waiting for Modbus response (Cat: 0x%02X, Ch: %u)",
                   this->current_packet_.category, this->current_packet_.channel);
          this->handle_packet_timeout_(this->current_packet_);
          this->fsm_state_ = FSM_IDLE;
        }
        break;
      }
    }
  }

  void write_setpoint(const std::vector<uint8_t> &channels, float celsius) {
    uint16_t raw_temp = static_cast<uint16_t>(celsius * 10.0f + 0.5f);
    for (uint8_t ch : channels) {
      if (ch < 1 || ch > 16) continue;
      ModbusPacket p;
      p.type = REQ_WRITE_SETPOINT;
      p.channel = ch;
      p.category = CAT_PACKED;
      p.index = PACKED_REG_MANUAL_TEMP;
      p.page = ch - 1;
      p.quantity = 1;
      p.payload.push_back(static_cast<uint8_t>(raw_temp >> 8));
      p.payload.push_back(static_cast<uint8_t>(raw_temp & 0xFF));
      this->tx_queue_.push_front(p); // High priority write
    }
  }

  void write_mode(const std::vector<uint8_t> &channels, climate::ClimateMode mode) {
    // Mode 0 = MANUAL (HEAT), Mode 1 = PERMANENT STANDBY (OFF)
    uint16_t mode_val = (mode == climate::CLIMATE_MODE_OFF) ? 0x0001 : 0x0000;
    uint16_t mask_val = 0xFFF8; // Keep all higher bits, replace mode bits 2..0

    for (uint8_t ch : channels) {
      if (ch < 1 || ch > 16) continue;
      ModbusPacket p;
      p.type = REQ_WRITE_MODE;
      p.channel = ch;
      p.category = CAT_PACKED;
      p.index = PACKED_REG_CONFIGURATION;
      p.page = ch - 1;
      p.quantity = 1;
      // FC 0x45 payload: Data (2 bytes) + Mask (2 bytes)
      p.payload.push_back(static_cast<uint8_t>(mode_val >> 8));
      p.payload.push_back(static_cast<uint8_t>(mode_val & 0xFF));
      p.payload.push_back(static_cast<uint8_t>(mask_val >> 8));
      p.payload.push_back(static_cast<uint8_t>(mask_val & 0xFF));
      this->tx_queue_.push_front(p); // High priority write
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Wavin AHC9000 Custom Component");
    ESP_LOGCONFIG(TAG, "  Timeout: %u ms", this->timeout_ms_);
  }

 protected:
  enum FSMState { FSM_IDLE, FSM_WAITING_RESPONSE };

  void send_packet_(const ModbusPacket &p) {
    std::vector<uint8_t> frame;
    frame.push_back(0x01); // Wavin Slave Address

    if (p.type == REQ_READ_ELEMENT || p.type == REQ_READ_CHANNEL || p.type == REQ_READ_PACKED) {
      frame.push_back(WAVIN_FC_READ_REGISTER);
      frame.push_back(p.category);
      frame.push_back(p.index);
      frame.push_back(p.page);
      frame.push_back(p.quantity);
    } else if (p.type == REQ_WRITE_SETPOINT) {
      frame.push_back(WAVIN_FC_WRITE_REGISTER);
      frame.push_back(p.category);
      frame.push_back(p.index);
      frame.push_back(p.page);
      frame.push_back(p.quantity);
      frame.insert(frame.end(), p.payload.begin(), p.payload.end());
    } else if (p.type == REQ_WRITE_MODE) {
      frame.push_back(WAVIN_FC_WRITE_MASKED_INDEX);
      frame.push_back(p.category);
      frame.push_back(p.index);
      frame.push_back(p.page);
      frame.push_back(p.quantity);
      frame.insert(frame.end(), p.payload.begin(), p.payload.end());
    }

    uint16_t crc = crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc >> 8));

    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(true);
    }
    this->write_array(frame.data(), frame.size());
    this->flush();
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(false);
    }
  }

  bool process_rx_buffer_() {
    if (this->rx_buffer_.size() < 5) return false;

    uint8_t fc = this->rx_buffer_[1];

    // Exception handling
    if ((fc & 0x80) != 0) {
      uint16_t frame_crc = this->rx_buffer_[3] | (this->rx_buffer_[4] << 8);
      if (crc16(this->rx_buffer_.data(), 3) == frame_crc) {
        ESP_LOGW(TAG, "Wavin Modbus exception 0x%02X for FC 0x%02X", this->rx_buffer_[2], fc);
        return true;
      }
    }

    uint8_t byte_count = this->rx_buffer_[2];
    size_t expected_len = 5 + byte_count;

    if (this->rx_buffer_.size() < expected_len) {
      return false;
    }

    uint16_t frame_crc = this->rx_buffer_[expected_len - 2] | (this->rx_buffer_[expected_len - 1] << 8);
    if (crc16(this->rx_buffer_.data(), expected_len - 2) != frame_crc) {
      ESP_LOGW(TAG, "CRC Checksum failed on response frame!");
      return true; // Discard invalid frame
    }

    std::vector<uint16_t> registers;
    for (size_t i = 0; i < byte_count; i += 2) {
      uint16_t val = (this->rx_buffer_[3 + i] << 8) | this->rx_buffer_[3 + i + 1];
      registers.push_back(val);
    }

    this->handle_response_registers_(this->current_packet_, registers);
    return true;
  }

  void handle_response_registers_(const ModbusPacket &p, const std::vector<uint16_t> &regs) {
    if (p.channel < 1 || p.channel > 16) return;
    auto &st = this->channels_[p.channel - 1];

    if (p.type == REQ_READ_ELEMENT) {
      if (regs.size() >= 12) {
        uint32_t addr = (static_cast<uint32_t>(regs[ELEM_REG_ADDRESS_H]) << 16) | regs[ELEM_REG_ADDRESS_L];
        st.address = addr;
        st.paired = (addr != 0);

        // Update root diagnostic paired status binary sensor
        if (this->channel_paired_sensors_[p.channel - 1] != nullptr) {
          this->channel_paired_sensors_[p.channel - 1]->publish_state(st.paired);
        }

        if (st.paired) {
          // Air Temp
          if (regs[ELEM_REG_AIR_TEMP] != 0x7FFF) {
            st.air_temp = static_cast<int16_t>(regs[ELEM_REG_AIR_TEMP]) / 10.0f;
          } else {
            st.air_temp = NAN;
          }

          // Floor Temp
          if (regs[ELEM_REG_FLOOR_TEMP] != 0x7FFF) {
            st.floor_temp = static_cast<int16_t>(regs[ELEM_REG_FLOOR_TEMP]) / 10.0f;
          } else {
            st.floor_temp = NAN;
          }

          // Status Flags
          uint16_t status = regs[ELEM_REG_STATUS];
          st.lost = ((status & ELEM_STATUS_LOST_MASK) != 0) || ((status & ELEM_STATUS_ALIVE_MASK) == 0);
          st.low_battery = ((status & ELEM_STATUS_LOW_BATT_MASK) != 0);

          // RSSI
          int8_t signed_rssi = static_cast<int8_t>(regs[ELEM_REG_RSSI] & 0xFF);
          st.rssi = -74.0f + (signed_rssi * 0.5f);

          // Battery %
          uint8_t batt_raw = regs[ELEM_REG_BATTERY] & 0x0F;
          st.battery_pct = (batt_raw > 10 ? 10 : batt_raw) * 10;

          // Sync Group
          st.sync_group = regs[ELEM_REG_SYNC_GROUP] & 0xFF;
        }
      }
    } else if (p.type == REQ_READ_CHANNEL) {
      if (regs.size() >= 3) {
        st.heating_active = ((regs[CH_REG_TIMER_EVENT] & CH_TIMER_EVENT_OUTP_ON) != 0);
        st.primary_element = regs[CH_REG_PRIMARY_ELEMENT] & 0x3F;
        if ((regs[CH_REG_PRIMARY_ELEMENT] & CH_PRIMARY_ELEM_ALL_LOST) != 0) {
          st.lost = true;
        }
      }
    } else if (p.type == REQ_READ_PACKED) {
      if (regs.size() >= 8) {
        st.target_temp = static_cast<int16_t>(regs[PACKED_REG_MANUAL_TEMP]) / 10.0f;
        uint16_t config = regs[PACKED_REG_CONFIGURATION];
        uint8_t mode_bits = config & 0x0007;
        st.mode = (mode_bits == 1) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
      }
    }

    this->notify_sub_device_entities_();
  }

  void handle_packet_timeout_(const ModbusPacket &p) {
    if (p.channel >= 1 && p.channel <= 16) {
      auto &st = this->channels_[p.channel - 1];
      st.lost = true; // Flag channel as lost gracefully on timeout
      this->notify_sub_device_entities_();
    }
  }

  void notify_sub_device_entities_();

  FSMState fsm_state_{FSM_IDLE};
  std::deque<ModbusPacket> tx_queue_;
  ModbusPacket current_packet_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_request_time_{0};
  uint32_t timeout_ms_{100};
  GPIOPin *flow_control_pin_{nullptr};

  std::array<ChannelState, 16> channels_;
  std::array<binary_sensor::BinarySensor *, 16> channel_paired_sensors_;

  std::vector<WavinUpdatableEntity *> updatable_entities_;
};

class WavinZoneClimate : public climate::Climate, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneClimate(WavinAHC9000Component *parent, std::vector<uint8_t> channels)
      : parent_(parent), channels_(channels) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();
    traits.set_visual_min_temperature(5.0f);
    traits.set_visual_max_temperature(35.0f);
    traits.set_visual_target_temperature_step(0.5f);
    traits.set_supported_modes({climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_OFF});
    return traits;
  }

  void control(const climate::ClimateCall &call) override {
    if (call.get_target_temperature().has_value()) {
      float temp = *call.get_target_temperature();
      this->target_temperature = temp;
      this->parent_->write_setpoint(this->channels_, temp);
    }
    if (call.get_mode().has_value()) {
      climate::ClimateMode mode = *call.get_mode();
      this->mode = mode;
      this->parent_->write_mode(this->channels_, mode);
    }
    this->publish_state();
  }

  void update_state() {
    if (this->channels_.empty() || this->parent_ == nullptr) return;

    uint8_t primary_ch = this->channels_[0];
    const auto &st = this->parent_->get_channel_data(primary_ch);

    this->current_temperature = st.air_temp;
    this->target_temperature = st.target_temp;
    this->mode = st.mode;

    if (this->mode == climate::CLIMATE_MODE_OFF) {
      this->action = climate::CLIMATE_ACTION_OFF;
    } else {
      bool any_heating = false;
      for (uint8_t ch : this->channels_) {
        if (this->parent_->get_channel_data(ch).heating_active) {
          any_heating = true;
          break;
        }
      }
      this->action = any_heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
    }

    this->publish_state();
  }

 protected:
  WavinAHC9000Component *parent_;
  std::vector<uint8_t> channels_;
};

class WavinZoneBatterySensor : public sensor::Sensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneBatterySensor(WavinAHC9000Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired && !st.lost) {
      this->publish_state(st.battery_pct);
    } else {
      this->publish_state(NAN);
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  uint8_t channel_;
};

class WavinZoneRSSISensor : public sensor::Sensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneRSSISensor(WavinAHC9000Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired && !std::isnan(st.rssi)) {
      this->publish_state(st.rssi);
    } else {
      this->publish_state(NAN);
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  uint8_t channel_;
};

class WavinZoneLowBatterySensor : public binary_sensor::BinarySensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneLowBatterySensor(WavinAHC9000Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired) {
      this->publish_state(st.low_battery);
    } else {
      this->publish_state(false);
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  uint8_t channel_;
};

class WavinZoneLostSensor : public binary_sensor::BinarySensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneLostSensor(WavinAHC9000Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired) {
      this->publish_state(st.lost);
    } else {
      this->publish_state(true);
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  uint8_t channel_;
};

class WavinZoneHeatingDemandSensor : public binary_sensor::BinarySensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneHeatingDemandSensor(WavinAHC9000Component *parent, std::vector<uint8_t> channels)
      : parent_(parent), channels_(channels) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() {
    if (this->parent_ == nullptr) return;
    bool any_heating = false;
    for (uint8_t ch : this->channels_) {
      if (this->parent_->get_channel_data(ch).heating_active) {
        any_heating = true;
        break;
      }
    }
    this->publish_state(any_heating);
  }

 protected:
  WavinAHC9000Component *parent_;
  std::vector<uint8_t> channels_;
};

inline void WavinAHC9000Component::notify_sub_device_entities_() {
  for (auto *e : this->updatable_entities_) {
    e->update_state();
  }
}

}  // namespace wavin_ahc9000
}  // namespace esphome
