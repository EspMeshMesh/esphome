#pragma once
#include <stdint.h>

namespace esphome {
namespace meshmesh {

class MeshmeshComponent;
class Entities {
public:
    static uint8_t handle_frame(uint8_t *buf, uint16_t len, MeshmeshComponent *parent);
};

}
}