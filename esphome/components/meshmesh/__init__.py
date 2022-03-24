from esphome.core import CORE, coroutine, coroutine_with_priority
from esphome.const import CONF_ID, CONF_BAUD_RATE, CONF_RX_BUFFER_SIZE, CONF_TX_BUFFER_SIZE, CONF_CHANNEL, CONF_PASSWORD
from esphome.components import logger
import esphome.config_validation as cv
import esphome.codegen as cg

AUTO_LOAD = ['network']

meshmesh_ns = cg.esphome_ns.namespace('meshmesh')
MeshmeshComponent = meshmesh_ns.class_('MeshmeshComponent', cg.Component)

CONF_TEST_MODE = "test_mode"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MeshmeshComponent),
    cv.Optional(CONF_BAUD_RATE, default=0): cv.positive_int,
    cv.Optional(CONF_RX_BUFFER_SIZE, default=2048): cv.validate_bytes,
    cv.Optional(CONF_TX_BUFFER_SIZE, default=0): cv.validate_bytes,
    cv.Optional(CONF_CHANNEL, default=99): cv.positive_int,
    cv.Required(CONF_PASSWORD): cv.string,
    cv.Optional(CONF_TEST_MODE): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)


@coroutine_with_priority(100.0)
def to_code(config):
    baud_rate = config[CONF_BAUD_RATE]
    cg.add_define('USE_MESH_MESH')
    cg.add_define('USE_POLITE_BROADCAST_PROTOCOL')
    cg.add_define('USE_MULTIPATH_PROTOCOL')
    cg.add_define('USE_CONNECTED_PROTOCOL')
    if CONF_TEST_MODE in config:
        cg.add_define('USE_TEST_PROCEDURE')
    #cg.add_build_flag('-DNO_GLOBAL_SERIAL')
    if CORE.is_esp8266:
    #elif CORE.is_esp32:
        cg.add_build_flag('-Wl,-wrap=ppEnqueueRxq')
    new_meshmesh = MeshmeshComponent.new(baud_rate, config[CONF_TX_BUFFER_SIZE], config[CONF_RX_BUFFER_SIZE])
    var = cg.Pvariable(config[CONF_ID], new_meshmesh)
    if CONF_CHANNEL in config:
        cg.add(var.setChannel(config[CONF_CHANNEL]))
    cg.add(var.setAesPassword(config[CONF_PASSWORD]))
    cg.add(var.pre_setup())

    if CORE.is_esp8266:
        cg.add_library("ESP8266WiFi", None)

    yield cg.register_component(var, config)
