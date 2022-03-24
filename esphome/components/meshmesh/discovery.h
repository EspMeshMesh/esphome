#pragma once
#include <stdint.h>

namespace esphome {
namespace meshmesh {

#define DISCOVERY_TABLE_SIZE 64

struct CmdStartCompat_st {
    uint8_t cmd1;
    uint8_t mask;
    uint8_t filter;
    uint8_t slotnum;
} __attribute__ ((packed));
typedef struct CmdStartCompat_st CmdStartCompat_t;

struct CmdStart_st {
    uint8_t cmd1;
    uint8_t cmd2;
    uint8_t mask;
    uint8_t filter;
    uint8_t slotnum;
} __attribute__ ((packed));
typedef struct CmdStart_st CmdStart_t;

struct BaconsDataCompat_st {
	uint8_t reply;
	uint32_t id;
	int16_t  rssi;
} __attribute__ ((packed));
typedef struct BaconsDataCompat_st BaconsDataCompat_t;

struct BaconsData_st {
	uint8_t reply1;
	uint8_t reply2;
	int16_t  rssi;
	uint32_t id;
} __attribute__ ((packed));
typedef struct BaconsData_st BaconsData_t;

struct DiscoveryItem_st {
	uint32_t id;
	int16_t rssi1;
    int16_t rssi2;
	uint16_t flags;
} __attribute__ ((packed));
typedef struct DiscoveryItem_st DiscoveryItem_t;

class MeshmeshComponent;
class Discovery {
public:
    void init();
    void loop(MeshmeshComponent *parent);
    bool isRunning() const { return mRunPhase>0; }
    void process_beacon(uint32_t id, int16_t  rssi1, int16_t  rssi2);
    void clear_table(void);
    uint8_t handle_frame(uint8_t *buf, uint16_t len, MeshmeshComponent *parent);
    static uint32_t chipId();
private:
    uint8_t discovery_table_index = 0;
    DiscoveryItem_t discovery_table[DISCOVERY_TABLE_SIZE];
    CmdStart_st mStart;
    CmdStartCompat_st mStartCompat;
    uint32_t mStartTime = 0;
    uint32_t mBeaconDelay;
    uint8_t mRunPhase = 0;
};

}
}