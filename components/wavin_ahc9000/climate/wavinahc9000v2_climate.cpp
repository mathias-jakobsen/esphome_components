#include "wavinahc9000v2_climate.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wavinahc9000v2 {
static const char *TAG = "wavinahc9000v2.climate";

void Wavinahc9000v2Climate::setup() {
  current_temp_sensor_->add_on_state_callback([this](float state) {
    current_temperature = state;
    publish_state();
  });
  temp_setpoint_number_->add_on_state_callback([this](float state) {
    target_temperature = state;
    publish_state();
  });
  mode_switch_->add_on_state_callback([this](bool state) {
    ESP_LOGD(TAG, "MODE SWITCH CALLBACK: %s", ONOFF(state));
    if (state) {
      this->mode = climate::CLIMATE_MODE_OFF;
    } else {
      this->mode = climate::CLIMATE_MODE_HEAT;
    }
    publish_state();
  });
  
  // RETTELSE: Denne callback-funktion styrer nu både 'action' og 'mode'
  hvac_action_->add_on_state_callback([this](bool state) {
    ESP_LOGD(TAG, "HVAC ACTION CALLBACK: %s", ONOFF(state));
    if (state) {
      this->action = climate::CLIMATE_ACTION_HEATING;
      // Hvis enheden begynder at varme, skal tilstanden (mode) tvinges til HEAT.
      if (this->mode != climate::CLIMATE_MODE_HEAT) {
        ESP_LOGD(TAG, "Action is HEATING, forcing mode to HEAT.");
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
    publish_state();
  });

  // Sæt start-værdier
  current_temperature = current_temp_sensor_->state;
  target_temperature  = temp_setpoint_number_->state;
  if (mode_switch_->state) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    this->mode = climate::CLIMATE_MODE_HEAT;
  }
}

void Wavinahc9000v2Climate::control(const climate::ClimateCall& call) {
  if (call.get_target_temperature().has_value())
  {
    this->target_temperature = *call.get_target_temperature();
    float target = ((roundf(this->target_temperature * 2.0) / 2.0));
    ESP_LOGD(TAG, "Target temperature changed to: %f", target);
    temp_setpoint_number_->make_call().set_value(target).perform();
  }

  if (call.get_mode().has_value())
  {
    auto new_mode = *call.get_mode();
    if (new_mode == climate::CLIMATE_MODE_HEAT) {
      ESP_LOGD(TAG, "Turning OFF thermostat standby mode (setting mode to HEAT)");
      mode_switch_->turn_off();
    } else if (new_mode == climate::CLIMATE_MODE_OFF) {
      ESP_LOGD(TAG, "Turning ON thermostat standby mode (setting mode to OFF)");
      mode_switch_->turn_on();
    }
    // Mode opdateres via callback fra switchen, så vi behøver ikke sætte den her.
  }
  this->publish_state();
}

climate::ClimateTraits Wavinahc9000v2Climate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supported_modes({
    climate::ClimateMode::CLIMATE_MODE_OFF,
    climate::ClimateMode::CLIMATE_MODE_HEAT,
  });
  traits.set_supports_action(true);
  traits.set_supports_current_temperature(true);
  traits.set_visual_temperature_step(0.5);
  traits.set_visual_min_temperature(6);
  traits.set_visual_max_temperature(40);
  return traits;
}

void Wavinahc9000v2Climate::dump_config() {
  LOG_CLIMATE("", "Wavinahc9000v2 Climate", this);
}

} // namespace wavinahc9000v2
} // namespace esphome
