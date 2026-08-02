import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@mathias-jakobsen"]
AUTO_LOAD = ["uart", "climate", "sensor", "binary_sensor"]

wavin_ns = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000Component = wavin_ns.class_("WavinAHC9000Component", cg.PollingComponent, uart.UARTDevice)

CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WavinAHC9000Component),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("10s"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], await cg.get_variable(config[CONF_UART_ID]))
    await cg.register_component(var, config)
