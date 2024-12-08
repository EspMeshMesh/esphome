#pragma once
#ifndef __MESHMESH_H__
#define __MESHMESH_H__

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "packetbuf.h"
#include "discovery.h"
#include "rssicheck.h"
#include "memringbuffer.h"

#include <string>

#ifdef USE_ARDUINO
#if defined(USE_ESP8266) || defined(USE_ESP32)
#include <HardwareSerial.h>
#endif  // USE_ESP8266 || USE_ESP32
#endif  // USE_ARDUINO

#ifdef USE_ESP_IDF
#include <driver/uart.h>
#endif  // USE_ESP_IDF

namespace esphome {

#ifdef USE_BINARY_SENSOR
namespace binary_sensor {
class BinarySensor;
}
#endif
#ifdef USE_SENSOR
namespace sensor {
class Sensor;
}
#endif
#ifdef USE_LIGHT
namespace light {
class LightState;
}
#endif
#ifdef USE_TEXT_SENSOR
namespace text_sensor {
class TextSensor;
}
#endif
#ifdef USE_SWITCH
namespace switch_ {
class Switch;
}
#endif
namespace meshmesh {

typedef void (*EspHomeDataReceivedCbFn)(uint16_t, uint8_t *, uint16_t);

typedef enum { WAIT_MAGICK, WAIT_DATA, WAIT_ESCAPE } RecvState;

enum UARTSelection {
  UART_SELECTION_UART0 = 0,
  UART_SELECTION_UART1,
  UART_SELECTION_UART2,
};

class Broadcast;
class Unicast;
#ifdef USE_MULTIPATH_PROTOCOL
class MultiPath;
#endif
class PoliteBroadcastProtocol;
#ifdef USE_CONNECTED_PROTOCOL
class ConnectedPath;
#endif

#define HANDLE_UART_OK 0
#define HANDLE_UART_ERROR 1

struct MeshmeshSettings {
  char devicetag[32];
  // Log destination
  uint32_t log_destination;
  uint8_t channel;
  uint8_t txPower;
  uint32_t groups;
  uint32_t bindedServer;
} __attribute__((packed));

class MeshmeshComponent : public Component {
 public:
  typedef enum {
    SRC_SERIAL,
    SRC_BROADCAST,
    SRC_UNICAST,
    SRC_MULTIPATH,
    SRC_POLITEBRD,
    SRC_CONNPATH,
    SRC_FILTER
  } DataSrc;
  typedef enum { UNKNOW = 0, LUX, LAST_SENSOR_TYPE } SensorTypes;
  typedef enum {
    AllEntities = 0,
    SensorEntity,
    BinarySensorEntity,
    SwitchEntity,
    LightEntity,
    TextSensorEntity,
    LastEntity
  } EnityType;

 public:
  static MeshmeshComponent *singleton;
  static MeshmeshComponent *getInstance();
#ifdef USE_CONNECTED_PROTOCOL
  ConnectedPath *getConnectedPath() const { return mConnectedPath; }
#endif
 public:
  MeshmeshComponent(int baud_rate, int tx_buffer, int rx_buffer);
  void preSetupPreferences();
  void pre_setup();
  bool isDisabled() const {
    return mPreferences.bindedServer == 0;
  }  // bindedServer can be == 0 only if is enabled and not binded
  uint32_t bindedServer() const { return mPreferences.bindedServer; }
  void setBindedServer(uint32_t server);
  void set_uart_selection(UARTSelection uart_selection) { /*uart_ = uart_selection;*/
  }
#ifdef USE_ESP32
  static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
  void setupIdfWifi();
#endif
  void setupWifi();
  void setup() override;
  void setChannel(int channel) { mConfigChannel = channel; }
  void setAesPassword(const char *password) { mAesPassword = password; }
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BEFORE_CONNECTION; }
  void register_rssi_sensor(RssiSensor *sensor) { mRssiCheck.registerRssiSensor(sensor); }
  void loop() override;

 public:
  static void wifiInitMacAddr(uint8_t index);
  void commandReply(const uint8_t *buff, uint16_t len);
  void uartSendData(const uint8_t *buff, uint16_t len);
  int16_t lastPacketRssi() const { return packetbuf->lastPacketRssi(); }
  uint32_t broadcastFromAddress() const { return mBroadcastFromAddress; }
  void broadCastSendData(const uint8_t *buff, uint16_t len);
  void uniCastSendData(const uint8_t *buff, uint16_t len, uint32_t addr);
#ifdef USE_MULTIPATH_PROTOCOL
  void multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint8_t *path);
#endif
#ifdef USE_CONNECTED_PROTOCOL
  void connectedpathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint8_t *path);
#endif
 public:
  static unsigned long elapsedMillis(unsigned long t2, unsigned long t1) {
    return t2 >= t1 ? t2 - t1 : (~(t1 - t2)) + 1;
  }

 private:
#ifdef USE_SENSOR
  sensor::Sensor *findSensorByUnit(const std::string &unit);
  sensor::Sensor *findSensor(uint16_t hash);
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *findBinarySensor(uint16_t hash);
#endif
#ifdef USE_LIGHT
  light::LightState *findLightState(uint16_t hash);
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *findTextSensor(uint16_t hash);
#endif
#ifdef USE_SWITCH
  switch_::Switch *findSwitch(uint16_t hash);
#endif
 private:
  EnityType findEntityTypeByHash(uint16_t hash);

 private:
#ifdef USE_ESP_IDF
  void initIdfUart();
#endif
  void user_uart_recv_data(uint8_t byte);
  void flushUartTxBuffer();

 private:
  void handleFrame(uint8_t *buf, uint16_t len, DataSrc src, uint32_t from);
  void replyHandleFrame(uint8_t *buf, uint16_t len, DataSrc src, uint32_t from);
  uint8_t customHandleFrame(uint8_t *buf, uint16_t len);
  uint8_t flashHandleFrame(uint8_t *buf, uint16_t len);
  void flashDealyedHandleFrame();

 private:
  static void user_broadcast_recv_cb(uint8_t *data, uint16_t size, uint8_t *from);
  void user_broadcast_recv(uint8_t *data, uint16_t size, uint8_t *from);
  static void unicastRecvCb(void *arg, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
  void unicastRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
  static void multipathRecvCb(void *arg, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path,
                              uint8_t pathSize);
  void multipathRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path, uint8_t pathSize);
  static void politeBroadcastReceive(void *arg, uint8_t *data, uint16_t size, uint32_t from);
  void politeBroadcastReceiveCb(uint8_t *data, uint16_t size, uint32_t from);
  static void onConnectedPathNewClientCb(void *arg, uint32_t from, uint16_t handle);
  void onConnectedPathNewClient(uint32_t from, uint16_t handle);
  static void onConnectedPathReceiveCb(void *arg, uint8_t *data, uint16_t size, uint8_t connid);
  void onConnectedPathReceive(uint8_t *data, uint16_t size, uint8_t connid);
  void sendLog(int level, const char *tag, const char *payload);

 private:
  int mBaudRate{460800};
  int mTxBuffer;
  int mRxBuffer;
  int mConfigChannel = 99;
  ESPPreferenceObject mPreferencesObject;
  MeshmeshSettings mPreferences;

 private:
  DataSrc commandSource = SRC_SERIAL;
  PacketBuf *packetbuf = nullptr;
  Broadcast *broadcast = nullptr;
  Unicast *unicast = nullptr;
#ifdef USE_MULTIPATH_PROTOCOL
  MultiPath *multipath = nullptr;
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  PoliteBroadcastProtocol *mPoliteBroadcast = nullptr;
#endif
  uint32_t mBroadcastFromAddress = 0;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  uint32_t mPoliteFromAddress = 0;
#endif
#ifdef USE_CONNECTED_PROTOCOL
  ConnectedPath *mConnectedPath = nullptr;
  uint8_t mConnectionId = 0;
#endif

#ifdef USE_ARDUINO
#if defined(USE_ESP8266) || defined(USE_ESP32)
  HardwareSerial *mHwSerial = nullptr;
#endif  // USE_ESP8266 || USE_ESP32
#endif  // USE_ARDUINO

  int mUartNum{0};
  RecvState mRecvState = WAIT_MAGICK;
  uint8_t *mRecvBuffer = nullptr;
  uint16_t mRecvBufferPos = 0;
  uint8_t mRecvFromId[4];
  uint8_t mRecvPath[32];
  uint8_t mRecvPathSize = 0;

  Discovery mDiscovery;
  RssiCheck mRssiCheck;

 private:
  uint32_t mDelayedCommandStart = 0;
  uint32_t mDelayedCommandTime = 0;
  uint8_t mDelayedCommand = 0;
  uint8_t mDelayedSubCommand = 0;
  uint16_t mDelayedArgSize = 0;
  uint8_t *mDelayedArg = nullptr;

 private:
  // UartRingBuffer;
  MemRingBuffer mUartTxBuffer;
  // Use serial
  bool mUseSerial = false;
  // Elapsed time for stats
  uint32_t mElapsed1 = 0;
  // RSSI for handle frame
  int16_t mRssiHandle = 0;
  // Encryption password
  std::string mAesPassword;
#if defined(USE_ESP8266)
  bool mWorkAround{false};
#endif
#ifdef USE_TEST_PROCEDURE
 private:
  void test_light_broadcast(uint16_t value);
  void loop_test_procedure(void);
  uint16_t mTestProcedureState = 0;
  uint32_t mTestProcedureTime = 0;
#endif
 private:
  friend class Entities;
};

}  // namespace meshmesh
}  // namespace esphome

#endif
