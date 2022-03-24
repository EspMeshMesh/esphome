#include "entities.h"
#include "commands.h"
#include "meshmesh.h"

#include <esphome/core/log.h>
#ifdef USE_BINARY_SENSOR
#include <esphome/components/binary_sensor/binary_sensor.h>
#ifdef USE_LIGHT_PRESENCE
#include <esphome/components/light_presence/light_presence.h>
#endif
#endif
namespace esphome {
namespace meshmesh {

#define ENTITY_PREFS_NUM_REQ  			0x00
#define ENTITY_PREFS_NUM_REP			0x01
#define ENTITY_GET_PREFS_REQ  			0x02
#define ENTITY_GET_PREFS_REP			0x03
#define ENTITY_SET_PREFS_REQ  			0x04
#define ENTITY_SET_PREFS_REP			0x05

static const char *TAG = "meshmesh.entities";

uint8_t Entities::handle_frame(uint8_t *buf, uint16_t len, MeshmeshComponent *parent) {
    uint8_t err = 1;
	switch(buf[0]) {

	case ENTITY_PREFS_NUM_REQ:
		if(len == 4) {
            MeshmeshComponent::EnityType type = (MeshmeshComponent::EnityType)buf[1];
			uint16_t hash = uint16FromBuffer(buf+2);
			ESP_LOGD(TAG, "ENTITY_PREFS_NUM_REQ type %d hash %04X", type, hash);
			bool valuefound = false;
            uint16_t value;

			switch(type) {
				case MeshmeshComponent::LightEntity: { } break;
				case MeshmeshComponent::BinarySensorEntity: {
#ifdef USE_BINARY_SENSOR
					//auto *state = parent->findBinarySensor(hash);
#ifdef USE_LIGHT_PRESENCE
					if(state->get_device_class() == "presence") {
						auto state_ = static_cast<lightpresence::LightPresence *>(state);
						value = state_->get_preferences_num();
						valuefound = true;
					}
#endif
#endif
				} break;
				default:
				  break;
			}

            if(valuefound) {
                uint8_t rep[4];
                rep[0] = CMD_ENTITY_REP;
                rep[1] = ENTITY_PREFS_NUM_REP;
                uint16toBuffer(rep+2, value);
                parent->commandReply(rep, 4);
                err = 0;
            }
		}
		break;

	case ENTITY_GET_PREFS_REQ:
		if(len == 6) {
            MeshmeshComponent::EnityType type = (MeshmeshComponent::EnityType)buf[1];
			uint16_t hash = uint16FromBuffer(buf+2);
			uint16_t num = uint16FromBuffer(buf+4);
			ESP_LOGD(TAG, "ENTITY_GET_PREFS_REQ type %d hash %04X num %d", type, hash, num);
			bool valuefound = false;
            uint16_t value;

			switch(type) {
				case MeshmeshComponent::LightEntity: { } break;
				case MeshmeshComponent::BinarySensorEntity: {
#ifdef USE_BINARY_SENSOR
					//auto *state = parent->findBinarySensor(hash);
#ifdef USE_LIGHT_PRESENCE
					if(state->get_device_class() == "presence") {
						auto state_ = static_cast<lightpresence::LightPresence *>(state);
						value = state_->get_preference(num);
						valuefound = true;
					}
#endif
#endif
				} break;
				default:
				  break;
			}

            if(valuefound) {
                uint8_t rep[4];
                rep[0] = CMD_ENTITY_REP;
                rep[1] = ENTITY_GET_PREFS_REP;
                uint16toBuffer(rep+2, value);
                parent->commandReply(rep, 4);
                err = 0;
            }
		}
		break;

    case ENTITY_SET_PREFS_REQ:
		if(len == 8) {
			MeshmeshComponent::EnityType type = (MeshmeshComponent::EnityType)buf[1];
			uint16_t hash = uint16FromBuffer(buf+2);
			uint16_t num = uint16FromBuffer(buf+4);
			uint16_t value = uint16FromBuffer(buf+6);
			ESP_LOGD(TAG, "CMD_SET_ENTITY_PREFS_REQ type %d hash %04X num %d value %d", type, hash, num, value);
			bool valuefound = false;

			switch(type) {
				case MeshmeshComponent::LightEntity: { } break;
				case MeshmeshComponent::BinarySensorEntity: {
#ifdef USE_BINARY_SENSOR
					//auto *state = parent->findBinarySensor(hash);
					//if(state->get_device_class() == "presence") {
					//	auto state_ = static_cast<lightpresence::LightPresence *>(state);
					//		state_->set_preference(num, value);
					//	valuefound = true;
					//}
#endif
				} break;
				default:
				  break;
			}
			if(valuefound) {
				uint8_t rep[2];
				rep[0] = CMD_ENTITY_REP;
				rep[1] = ENTITY_SET_PREFS_REP;
				parent->commandReply(rep, 2);
				err = 0;
			}
		}
		break;

	}
    return err;
}

}
}