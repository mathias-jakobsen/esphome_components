import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, CONF_TYPE

from . import wavin_ns, WavinAHC9000Component

CONF_WAVIN_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"
CONF_CHANNELS = "channels"

WavinZoneLowBatterySensor = wavin_ns.class_("WavinZoneLowBatterySensor", binary_sensor.BinarySensor, cg.Component)
WavinZoneLostSensor = wavin_ns.class_("WavinZoneLostSensor", binary_sensor.BinarySensor, cg.Component)
WavinZoneHeatingDemandSensor = wavin_ns.class_("WavinZoneHeatingDemandSensor", binary_sensor.BinarySensor, cg.Component)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_WAVIN_ID): cv.use_id(WavinAHC9000Component),
        cv.Optional(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Optional(CONF_CHANNELS): cv.ensure_list(cv.int_range(min=1, max=16)),
        cv.Required(CONF_TYPE): cv.one_of("low_battery", "lost", "heating_demand", "channel_paired", lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_WAVIN_ID])
    sens_type = config[CONF_TYPE]
    if sens_type == "channel_paired":
        ch = config[CONF_CHANNEL]
        var = await binary_sensor.new_binary_sensor(config)
        cg.add(hub.set_channel_paired_sensor(ch, var))
        return

    if sens_type == "low_battery":
        var = cg.new_Pvariable(config[CONF_ID], hub, config[CONF_CHANNEL])
    elif sens_type == "lost":
        var = cg.new_Pvariable(config[CONF_ID], hub, config[CONF_CHANNEL])
    elif sens_type == "heating_demand":
        channels = [config[CONF_CHANNEL]] if CONF_CHANNEL in config else config[CONF_CHANNELS]
        var = cg.new_Pvariable(config[CONF_ID], hub, channels)
    await binary_sensor.register_binary_sensor(var, config)
