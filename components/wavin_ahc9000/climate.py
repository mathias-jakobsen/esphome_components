import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from . import wavin_ns, WavinAHC9000Component

CONF_WAVIN_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"
CONF_CHANNELS = "channels"

WavinZoneClimate = wavin_ns.class_("WavinZoneClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.climate_schema(WavinZoneClimate).extend(
    {
        cv.GenerateID(CONF_WAVIN_ID): cv.use_id(WavinAHC9000Component),
        cv.Optional(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Optional(CONF_CHANNELS): cv.ensure_list(cv.int_range(min=1, max=16)),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_WAVIN_ID])
    channels = []
    if CONF_CHANNEL in config:
        channels = [config[CONF_CHANNEL]]
    elif CONF_CHANNELS in config:
        channels = config[CONF_CHANNELS]

    var = cg.new_Pvariable(config[CONF_ID], hub, channels)
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
