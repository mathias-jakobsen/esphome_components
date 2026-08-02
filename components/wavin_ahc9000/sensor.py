import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_TYPE

from . import wavin_ns, WavinAHC9000Component

CONF_WAVIN_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"

WavinZoneBatterySensor = wavin_ns.class_("WavinZoneBatterySensor", sensor.Sensor, cg.Component)
WavinZoneRSSISensor = wavin_ns.class_("WavinZoneRSSISensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = cv.typed_schema(
    {
        "battery": sensor.sensor_schema(WavinZoneBatterySensor).extend(
            {
                cv.GenerateID(CONF_WAVIN_ID): cv.use_id(WavinAHC9000Component),
                cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
            }
        ).extend(cv.COMPONENT_SCHEMA),
        "rssi": sensor.sensor_schema(WavinZoneRSSISensor).extend(
            {
                cv.GenerateID(CONF_WAVIN_ID): cv.use_id(WavinAHC9000Component),
                cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
            }
        ).extend(cv.COMPONENT_SCHEMA),
    },
    lower=True,
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_WAVIN_ID])
    ch = config[CONF_CHANNEL]
    sens_type = config[CONF_TYPE]
    if sens_type == "battery":
        var = cg.new_Pvariable(config[CONF_ID], hub, ch)
    else:
        var = cg.new_Pvariable(config[CONF_ID], hub, ch)
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
