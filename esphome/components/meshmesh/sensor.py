import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_ADDRESS, UNIT_DECIBEL, ICON_SIGNAL
from esphome.components import sensor
from . import meshmesh_ns, MeshmeshComponent

DEPENDENCIES = ['meshmesh']

RSSISensor = meshmesh_ns.class_('RssiSensor', sensor.Sensor, cg.PollingComponent)

CONF_MESHMESH_ID = 'meshmesh_id'

CONFIG_SCHEMA = sensor.sensor_schema(UNIT_DECIBEL, ICON_SIGNAL, 0).extend({
    cv.GenerateID(): cv.declare_id(RSSISensor),
    cv.GenerateID(CONF_MESHMESH_ID): cv.use_id(MeshmeshComponent),
    cv.Required(CONF_ADDRESS): cv.positive_int,
}).extend(cv.polling_component_schema('1s'))


def to_code(config):
    paren = yield cg.get_variable(config[CONF_MESHMESH_ID])
    var = cg.new_Pvariable(config[CONF_ID], paren)
    yield sensor.register_sensor(var, config)
    yield cg.register_component(var, config)

    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(paren.register_rssi_sensor(var))
