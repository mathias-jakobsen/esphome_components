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
#include <set>
#include <algorithm>
#include <cmath>
#include <array>

namespace esphome {
namespace wavin_ahc9000 {

static const char *const TAG = "wavin_ahc9000";

// Wavin Modbus Protocol Constants
static constexpr uint8_t DEVICE_ADDR = 0x01;
static constexpr uint8_t FC_READ     = 0x43;
static constexpr uint8_t FC_WRITE    = 0x44;
static constexpr uint8_t FC_WRITE_MASKED = 0x45;

static constexpr uint8_t CAT_ELEMENTS = 0x01;
static constexpr uint8_t CAT_PACKED   = 0x02;
static constexpr uint8_t CAT_CHANNELS = 0x03;

static constexpr uint8_t CH_TIMER_EVENT = 0x00;
static constexpr uint16_t CH_TIMER_EVENT_OUTP_ON_MASK = 0x0010;
static constexpr uint8_t CH_PRIMARY_ELEMENT = 0x02;
static constexpr uint16_t CH_PRIMARY_ELEMENT_ELEMENT_MASK = 0x003F;
static constexpr uint16_t CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK = 0x0400;

static constexpr uint8_t ELEM_AIR_TEMPERATURE = 0x04;
static constexpr uint8_t ELEM_FLOOR_TEMPERATURE = 0x05;
static constexpr uint8_t ELEM_STATUS = 0x08;
static constexpr uint8_t ELEM_RSSI = 0x09;
static constexpr uint8_t ELEM_BATTERY_STATUS = 0x0A;

static constexpr uint8_t PACKED_MANUAL_TEMPERATURE = 0x00;
static constexpr uint8_t PACKED_CONFIGURATION = 0x07;
static constexpr uint16_t PACKED_CONFIGURATION_MODE_MASK = 0x0007;
static constexpr uint16_t PACKED_CONFIGURATION_MODE_STANDBY = 0x0001;

// Modbus CRC16
inline uint16_t crc16(const uint8_t *frame, size_t len) {
  uint16_t temp = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    temp ^= frame[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }
  return temp;
}

struct ChannelState {
  uint8_t primary_index{0};
  bool paired{false};
  bool all_tp_lost{false};
  float current_temp_c{NAN};
  float floor_temp_c{NAN};
  float setpoint_c{20.0f};
  uint8_t battery_pct{100};
  bool low_battery{false};
  float rssi{-60.0f};
  climate::ClimateMode mode{climate::CLIMATE_MODE_HEAT};
  climate::ClimateAction action{climate::CLIMATE_ACTION_IDLE};
};

class WavinUpdatableEntity {
 public:
  virtual ~WavinUpdatableEntity() = default;
  virtual void update_state() = 0;
};

class WavinAHC9000Component : public PollingComponent, public uart::UARTDevice {
 public:
  WavinAHC9000Component(uart::UARTComponent *parent) : uart::UARTDevice(parent) {
    this->channel_paired_sensors_.fill(nullptr);
  }

  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; }
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
    ESP_LOGCONFIG(TAG, "Setting up Wavin AHC9000 Modbus RTU Component...");
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->setup();
      this->flow_control_pin_->digital_write(false);
    }
  }

  void loop() override {}

  void update() override {
    std::vector<uint16_t> regs;

    for (uint8_t ch = 1; ch <= 16; ch++) {
      uint8_t ch_page = ch - 1;
      auto &st = this->channels_[ch - 1];

      // 1. Read Channel Primary Element (Category 0x03, Page ch-1, Index 0x02)
      if (this->read_registers(CAT_CHANNELS, ch_page, CH_PRIMARY_ELEMENT, 1, regs) && !regs.empty()) {
        uint16_t v = regs[0];
        st.primary_index = v & CH_PRIMARY_ELEMENT_ELEMENT_MASK;
        st.all_tp_lost = (v & CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK) != 0;
        st.paired = (st.primary_index > 0 && !st.all_tp_lost);
      } else {
        st.paired = false;
      }

      // Update root diagnostic paired status binary sensor
      if (this->channel_paired_sensors_[ch - 1] != nullptr) {
        this->channel_paired_sensors_[ch - 1]->publish_state(st.paired);
      }

      if (!st.paired) continue;

      // 2. Read Configuration & Mode (Category 0x02, Page ch-1, Index 0x07)
      if (this->read_registers(CAT_PACKED, ch_page, PACKED_CONFIGURATION, 1, regs) && !regs.empty()) {
        uint16_t raw_cfg = regs[0];
        uint16_t mode_bits = raw_cfg & PACKED_CONFIGURATION_MODE_MASK;
        st.mode = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
      }

      // 3. Read Setpoint Temperature (Category 0x02, Page ch-1, Index 0x00)
      if (this->read_registers(CAT_PACKED, ch_page, PACKED_MANUAL_TEMPERATURE, 1, regs) && !regs.empty()) {
        st.setpoint_c = regs[0] / 10.0f;
      }

      // 4. Read Timer Event / Heating Output Status (Category 0x03, Page ch-1, Index 0x00)
      if (this->read_registers(CAT_CHANNELS, ch_page, CH_TIMER_EVENT, 1, regs) && !regs.empty()) {
        bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
        st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
      }

      // 5. Read Element Data (Category 0x01, Page primary_index - 1, Index 0x00, Quantity 11)
      uint8_t elem_page = st.primary_index - 1;
      if (this->read_registers(CAT_ELEMENTS, elem_page, 0x00, 11, regs) && regs.size() > ELEM_AIR_TEMPERATURE) {
        st.current_temp_c = regs[ELEM_AIR_TEMPERATURE] / 10.0f;
        if (regs.size() > ELEM_FLOOR_TEMPERATURE) {
          float ft = regs[ELEM_FLOOR_TEMPERATURE] / 10.0f;
          st.floor_temp_c = (ft > 1.0f && ft < 90.0f) ? ft : NAN;
        }
        if (regs.size() > ELEM_STATUS) {
          uint16_t status_reg = regs[ELEM_STATUS];
          st.low_battery = (status_reg & 0x0400) != 0;
        }
        if (regs.size() > ELEM_RSSI) {
          int8_t signed_rssi = static_cast<int8_t>(regs[ELEM_RSSI] & 0xFF);
          st.rssi = -74.0f + (signed_rssi * 0.5f);
        }
        if (regs.size() > ELEM_BATTERY_STATUS) {
          uint16_t raw_batt = regs[ELEM_BATTERY_STATUS];
          uint8_t steps = (raw_batt > 10) ? 10 : static_cast<uint8_t>(raw_batt);
          st.battery_pct = steps * 10;
        }
      }
    }

    this->notify_sub_device_entities_();
  }

  void write_setpoint(const std::vector<uint8_t> &channels, float celsius) {
    uint16_t raw_temp = static_cast<uint16_t>(celsius * 10.0f + 0.5f);
    for (uint8_t ch : channels) {
      if (ch < 1 || ch > 16) continue;
      this->write_register(CAT_PACKED, ch - 1, PACKED_MANUAL_TEMPERATURE, raw_temp);
    }
  }

  void write_mode(const std::vector<uint8_t> &channels, climate::ClimateMode mode) {
    uint16_t mode_val = (mode == climate::CLIMATE_MODE_OFF) ? PACKED_CONFIGURATION_MODE_STANDBY : 0x0000;
    for (uint8_t ch : channels) {
      if (ch < 1 || ch > 16) continue;
      this->write_masked_register(CAT_PACKED, ch - 1, PACKED_CONFIGURATION, 0xFFF8, mode_val);
    }
  }

 protected:
  bool read_registers(uint8_t category, uint8_t page, uint8_t index, uint8_t count, std::vector<uint16_t> &out) {
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
      while (this->available()) {
        uint8_t dummy;
        this->read_byte(&dummy);
      }

      uint8_t msg[8];
      msg[0] = DEVICE_ADDR;
      msg[1] = FC_READ;
      msg[2] = category;
      msg[3] = index;
      msg[4] = page;
      msg[5] = count;
      uint16_t crc = crc16(msg, 6);
      msg[6] = crc & 0xFF;
      msg[7] = crc >> 8;

      if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(true);
      this->write_array(msg, 8);
      this->flush();
      delayMicroseconds(250);
      if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);

      std::vector<uint8_t> buf;
      uint32_t start = millis();
      while (millis() - start < this->receive_timeout_ms_) {
        while (this->available()) {
          int c = this->read();
          if (c < 0) break;
          buf.push_back(static_cast<uint8_t>(c));
          if (buf.size() >= 5) {
            uint8_t expected = static_cast<uint8_t>(buf[2] + 5);
            if (buf[0] == DEVICE_ADDR && buf[1] == FC_READ && buf.size() == expected) {
              if (crc16(buf.data(), buf.size()) != 0) {
                goto next_attempt;
              }
              uint8_t bytes = buf[2];
              out.clear();
              for (uint8_t i = 0; i + 1 < bytes; i += 2) {
                uint16_t w = static_cast<uint16_t>((buf[3 + i] << 8) | buf[3 + i + 1]);
                out.push_back(w);
              }
              return true;
            }
          }
        }
        delay(1);
      }
    next_attempt:;
    }
    return false;
  }

  bool write_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value) {
    uint8_t msg[10];
    msg[0] = DEVICE_ADDR;
    msg[1] = FC_WRITE;
    msg[2] = category;
    msg[3] = index;
    msg[4] = page;
    msg[5] = 1;
    msg[6] = value >> 8;
    msg[7] = value & 0xFF;
    uint16_t crc = crc16(msg, 8);
    msg[8] = crc & 0xFF;
    msg[9] = crc >> 8;

    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(true);
    this->write_array(msg, 10);
    this->flush();
    delayMicroseconds(250);
    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);
    delay(50);
    return true;
  }

  bool write_masked_register(uint8_t category, uint8_t page, uint8_t index, uint16_t and_mask, uint16_t or_mask) {
    uint8_t msg[12];
    msg[0] = DEVICE_ADDR;
    msg[1] = FC_WRITE_MASKED;
    msg[2] = category;
    msg[3] = index;
    msg[4] = page;
    msg[5] = 1;
    msg[6] = or_mask >> 8;
    msg[7] = or_mask & 0xFF;
    msg[8] = and_mask >> 8;
    msg[9] = and_mask & 0xFF;
    uint16_t crc = crc16(msg, 10);
    msg[10] = crc & 0xFF;
    msg[11] = crc >> 8;

    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(true);
    this->write_array(msg, 12);
    this->flush();
    delayMicroseconds(250);
    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);
    delay(50);
    return true;
  }

  void notify_sub_device_entities_() {
    for (auto *e : this->updatable_entities_) {
      e->update_state();
    }
  }

  uint32_t receive_timeout_ms_{1000};
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
    this->target_temperature = 20.0f;
    this->current_temperature = NAN;
    this->mode = climate::CLIMATE_MODE_HEAT;
    this->action = climate::CLIMATE_ACTION_IDLE;
    this->publish_state();
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

  void update_state() override {
    if (this->channels_.empty() || this->parent_ == nullptr) return;

    uint8_t primary_ch = this->channels_[0];
    const auto &st = this->parent_->get_channel_data(primary_ch);

    this->current_temperature = st.current_temp_c;
    this->target_temperature = st.setpoint_c;
    this->mode = st.mode;

    if (this->mode == climate::CLIMATE_MODE_OFF) {
      this->action = climate::CLIMATE_ACTION_OFF;
    } else {
      bool any_heating = false;
      for (uint8_t ch : this->channels_) {
        if (this->parent_->get_channel_data(ch).action == climate::CLIMATE_ACTION_HEATING) {
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

  void update_state() override {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired) {
      this->publish_state(st.battery_pct);
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

  void update_state() override {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired && !std::isnan(st.rssi)) {
      this->publish_state(st.rssi);
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
    this->publish_state(false);
  }

  void update_state() override {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    this->publish_state(st.paired && st.low_battery);
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
    this->publish_state(false);
  }

  void update_state() override {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    this->publish_state(st.paired ? st.all_tp_lost : false);
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
    this->publish_state(false);
  }

  void update_state() override {
    if (this->parent_ == nullptr) return;
    bool any_heating = false;
    for (uint8_t ch : this->channels_) {
      if (this->parent_->get_channel_data(ch).action == climate::CLIMATE_ACTION_HEATING) {
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

}  // namespace wavin_ahc9000
}  // namespace esphome
