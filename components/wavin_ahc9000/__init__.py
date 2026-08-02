import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@mathias-jakobsen"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["climate", "sensor", "binary_sensor"]

wavin_ns = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000Component = wavin_ns.class_(
    "WavinAHC9000Component", cg.PollingComponent, uart.UARTDevice
)

CONF_RECEIVE_TIMEOUT = "receive_timeout"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WavinAHC9000Component),
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default="250ms"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RECEIVE_TIMEOUT in config:
        cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))
