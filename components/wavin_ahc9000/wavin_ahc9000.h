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

struct ModbusReq {
  uint8_t ch;
  uint8_t category;
  uint8_t page;
  uint8_t index;
  uint8_t count{1};
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

  void update() override {
    // Non-blocking background queue creation: populate 1-register requests for channels 1..16
    for (uint8_t ch = 1; ch <= 16; ch++) {
      uint8_t page = ch - 1;
      this->queue_req_(ch, CAT_CHANNELS, page, CH_PRIMARY_ELEMENT, 1);
      this->queue_req_(ch, CAT_PACKED, page, PACKED_CONFIGURATION, 1);
      this->queue_req_(ch, CAT_PACKED, page, PACKED_MANUAL_TEMPERATURE, 1);
      this->queue_req_(ch, CAT_CHANNELS, page, CH_TIMER_EVENT, 1);
    }
  }

  void loop() override {
    // Process serial incoming bytes
    while (this->available()) {
      uint8_t b;
      this->read_byte(&b);
      this->rx_buf_.push_back(b);
    }

    uint32_t now = millis();

    switch (this->fsm_state_) {
      case FSM_IDLE: {
        if (!this->req_queue_.empty() && (now - this->last_tx_time_ > 35)) {
          this->current_req_ = this->req_queue_.front();
          this->req_queue_.pop_front();

          // Send 8-byte Modbus read frame
          uint8_t msg[8];
          msg[0] = DEVICE_ADDR;
          msg[1] = FC_READ;
          msg[2] = this->current_req_.category;
          msg[3] = this->current_req_.index;
          msg[4] = this->current_req_.page;
          msg[5] = this->current_req_.count;
          uint16_t crc = crc16(msg, 6);
          msg[6] = crc & 0xFF;
          msg[7] = crc >> 8;

          // Clear RX buffer and flush UART
          this->rx_buf_.clear();

          if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(true);
          this->write_array(msg, 8);
          this->flush();
          delayMicroseconds(200);
          if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);

          this->last_tx_time_ = now;
          this->fsm_state_ = FSM_WAITING;
        }
        break;
      }
      case FSM_WAITING: {
        // Strip leading garbage bytes until 0x01 0x43 header
        while (this->rx_buf_.size() >= 2) {
          if (this->rx_buf_[0] == DEVICE_ADDR && (this->rx_buf_[1] == FC_READ || (this->rx_buf_[1] & 0x80) != 0)) {
            break;
          }
          this->rx_buf_.erase(this->rx_buf_.begin());
        }

        if (this->rx_buf_.size() >= 5) {
          uint8_t byte_cnt = this->rx_buf_[2];
          size_t expected_len = 5 + byte_cnt;

          if (this->rx_buf_.size() >= expected_len) {
            uint16_t frame_crc = crc16(this->rx_buf_.data(), expected_len);
            if (frame_crc == 0) { // Valid CRC
              std::vector<uint16_t> registers;
              for (size_t i = 0; i + 1 < byte_cnt; i += 2) {
                uint16_t val = (this->rx_buf_[3 + i] << 8) | this->rx_buf_[3 + i + 1];
                registers.push_back(val);
              }
              this->process_response_(this->current_req_, registers);
            }
            this->rx_buf_.clear();
            this->fsm_state_ = FSM_IDLE;
          }
        }

        // Timeout handler (250ms max per packet)
        if (this->fsm_state_ == FSM_WAITING && (now - this->last_tx_time_ > 250)) {
          this->rx_buf_.clear();
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
      this->send_write_reg_(CAT_PACKED, ch - 1, PACKED_MANUAL_TEMPERATURE, raw_temp);
    }
  }

  void write_mode(const std::vector<uint8_t> &channels, climate::ClimateMode mode) {
    uint16_t mode_val = (mode == climate::CLIMATE_MODE_OFF) ? PACKED_CONFIGURATION_MODE_STANDBY : 0x0000;
    for (uint8_t ch : channels) {
      if (ch < 1 || ch > 16) continue;
      this->send_write_masked_reg_(CAT_PACKED, ch - 1, PACKED_CONFIGURATION, 0xFFF8, mode_val);
    }
  }

 protected:
  void queue_req_(uint8_t ch, uint8_t category, uint8_t page, uint8_t index, uint8_t count = 1) {
    ModbusReq r;
    r.ch = ch;
    r.category = category;
    r.page = page;
    r.index = index;
    r.count = count;
    this->req_queue_.push_back(r);
  }

  void process_response_(const ModbusReq &req, const std::vector<uint16_t> &regs) {
    if (req.ch < 1 || req.ch > 16 || regs.empty()) return;
    auto &st = this->channels_[req.ch - 1];

    if (req.category == CAT_CHANNELS && req.index == CH_PRIMARY_ELEMENT) {
      uint16_t v = regs[0];
      st.primary_index = v & CH_PRIMARY_ELEMENT_ELEMENT_MASK;
      st.all_tp_lost = (v & CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK) != 0;
      st.paired = (st.primary_index > 0 && !st.all_tp_lost);

      if (this->channel_paired_sensors_[req.ch - 1] != nullptr) {
        this->channel_paired_sensors_[req.ch - 1]->publish_state(st.paired);
      }

      // If paired, dynamically queue element air temp and battery queries for the element page
      if (st.paired) {
        uint8_t elem_page = st.primary_index - 1;
        this->queue_req_(req.ch, CAT_ELEMENTS, elem_page, ELEM_AIR_TEMPERATURE, 1);
        this->queue_req_(req.ch, CAT_ELEMENTS, elem_page, ELEM_BATTERY_STATUS, 1);
      }
    } else if (req.category == CAT_PACKED && req.index == PACKED_CONFIGURATION) {
      uint16_t mode_bits = regs[0] & PACKED_CONFIGURATION_MODE_MASK;
      st.mode = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
    } else if (req.category == CAT_PACKED && req.index == PACKED_MANUAL_TEMPERATURE) {
      if (regs[0] != 0x7FFF && regs[0] != 0) {
        st.setpoint_c = regs[0] / 10.0f;
      }
    } else if (req.category == CAT_CHANNELS && req.index == CH_TIMER_EVENT) {
      bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
      st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
    } else if (req.category == CAT_ELEMENTS && req.index == ELEM_AIR_TEMPERATURE) {
      if (regs[0] != 0x7FFF) {
        st.current_temp_c = static_cast<int16_t>(regs[0]) / 10.0f;
        ESP_LOGI(TAG, "Channel %u Air Temp: %.1f C", req.ch, st.current_temp_c);
      }
    } else if (req.category == CAT_ELEMENTS && req.index == ELEM_BATTERY_STATUS) {
      uint16_t raw_b = regs[0];
      uint8_t steps = (raw_b > 10) ? 10 : static_cast<uint8_t>(raw_b);
      st.battery_pct = steps * 10;
      ESP_LOGI(TAG, "Channel %u Battery: %u pct", req.ch, (unsigned)st.battery_pct);
    }

    this->notify_sub_device_entities_();
  }

  void send_write_reg_(uint8_t category, uint8_t page, uint8_t index, uint16_t value) {
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
    delayMicroseconds(200);
    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);
  }

  void send_write_masked_reg_(uint8_t category, uint8_t page, uint8_t index, uint16_t and_mask, uint16_t or_mask) {
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
    delayMicroseconds(200);
    if (this->flow_control_pin_ != nullptr) this->flow_control_pin_->digital_write(false);
  }

  void notify_sub_device_entities_() {
    for (auto *e : this->updatable_entities_) {
      e->update_state();
    }
  }

  enum FSMState { FSM_IDLE, FSM_WAITING };

  FSMState fsm_state_{FSM_IDLE};
  std::deque<ModbusReq> req_queue_;
  ModbusReq current_req_;
  std::vector<uint8_t> rx_buf_;
  uint32_t last_tx_time_{0};
  uint32_t receive_timeout_ms_{250};
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

    bool changed = false;

    if (!std::isnan(st.current_temp_c)) {
      if (std::isnan(this->current_temperature) || std::abs(this->current_temperature - st.current_temp_c) > 0.01f) {
        this->current_temperature = st.current_temp_c;
        changed = true;
      }
    }
    if (!std::isnan(st.setpoint_c) && this->target_temperature != st.setpoint_c) {
      this->target_temperature = st.setpoint_c;
      changed = true;
    }
    if (this->mode != st.mode) {
      this->mode = st.mode;
      changed = true;
    }

    climate::ClimateAction new_action;
    if (this->mode == climate::CLIMATE_MODE_OFF) {
      new_action = climate::CLIMATE_ACTION_OFF;
    } else {
      bool any_heating = false;
      for (uint8_t ch : this->channels_) {
        if (this->parent_->get_channel_data(ch).action == climate::CLIMATE_ACTION_HEATING) {
          any_heating = true;
          break;
        }
      }
      new_action = any_heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
    }

    if (this->action != new_action) {
      this->action = new_action;
      changed = true;
    }

    if (changed || std::isnan(this->current_temperature)) {
      this->publish_state();
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  std::vector<uint8_t> channels_;
};

class WavinZoneTemperatureSensor : public sensor::Sensor, public Component, public WavinUpdatableEntity {
 public:
  WavinZoneTemperatureSensor(WavinAHC9000Component *parent, uint8_t channel)
      : parent_(parent), channel_(channel) {}

  void setup() override {
    if (this->parent_ != nullptr) {
      this->parent_->register_updatable(this);
    }
  }

  void update_state() override {
    if (this->parent_ == nullptr) return;
    const auto &st = this->parent_->get_channel_data(this->channel_);
    if (st.paired && !std::isnan(st.current_temp_c)) {
      if (!this->has_state() || std::abs(this->state - st.current_temp_c) > 0.01f) {
        this->publish_state(st.current_temp_c);
      }
    }
  }

 protected:
  WavinAHC9000Component *parent_;
  uint8_t channel_;
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
      if (!this->has_state() || this->state != (float)st.battery_pct) {
        this->publish_state(st.battery_pct);
      }
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
