#include "meshmesh.h"
#include <algorithm>

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/md5/md5.h"
#ifdef USE_BIND_SERVER
#include "esphome/components/globals/globals_component.h"
#include "esphome/components/wifi/wifi_component.h"
#endif
#ifdef USE_LOGGER
#include "esphome/components/logger/logger.h"
#endif
#include "esphome/core/application.h"
#ifdef USE_LIGHT
#include "esphome/components/light/light_state.h"
#include "esphome/components/light/light_output.h"
#endif

#include "commands.h"
#include "packetbuf.h"
#include "entities.h"
#include "broadcast.h"
#include "unicast.h"
#ifdef USE_MULTIPATH_PROTOCOL
#include "multipath.h"
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
#include "polite.h"
#endif
#ifdef USE_CONNECTED_PROTOCOL
#include "connectedpath.h"
#endif

#ifdef USE_ARDUINO
#include <Esp.h>
#ifdef USE_ESP8266
#include <eboot_command.h>
#endif
#endif

#ifdef USE_ESP_IDF
#include <esp_wifi.h>
#include <esp_event.h>
#endif

extern "C" uint32_t _FS_start;
extern "C" uint32_t _SPIFFS_start;

#ifdef USE_BIND_SERVER
extern esphome::globals::RestoringGlobalsComponent<int> *binded_server;
extern esphome::wifi::WiFiComponent *wifi_wificomponent_id;
#endif

namespace esphome {
namespace meshmesh {

static const char *TAG = "meshmesh";

#define UNICAST_DEFAULT_PORT 0

MeshmeshComponent *MeshmeshComponent::singleton = nullptr;

MeshmeshComponent *MeshmeshComponent::getInstance() { return singleton; }

MeshmeshComponent::MeshmeshComponent(int baud_rate, int tx_buffer, int rx_buffer)
    : mBaudRate(baud_rate), mTxBuffer(tx_buffer), mRxBuffer(rx_buffer) {
  if (singleton == nullptr)
    singleton = this;
}

void MeshmeshComponent::pre_setup() {
#ifdef USE_ARDUINO
  mHwSerial = &Serial;
  ESP_LOGD(TAG, "pre_setup baudrate %d", mBaudRate);
  if (mBaudRate > 0) {
    mHwSerial->begin(mBaudRate);
    mHwSerial->setRxBufferSize(mRxBuffer);
  }
#endif  // USE_ARDUINO

#ifdef USE_ESP_IDF
  initIdfUart();
#endif  // USE_ESP_IDF

  if (mTxBuffer > 0)
    mUartTxBuffer.resize(mTxBuffer);
}

#ifdef USE_ESP32
void MeshmeshComponent::wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "wifi_event_handler %ld", event_id);
}
#endif

void MeshmeshComponent::setup() {
  mUseSerial = mBaudRate > 0 || (logger::global_logger != nullptr && logger::global_logger->get_baud_rate() > 0);

  mPreferencesObject = global_preferences->make_preference<MeshmeshSettings>(fnv1_hash("MeshmeshComponent"), true);
  if (!mPreferencesObject.load(&mPreferences)) {
    os_memset(mPreferences.devicetag, 0, 32);
    mPreferences.channel = 6;
    mPreferences.txPower = 82;
    mPreferences.log_destination = 0;
    mPreferences.groups = 0;
  }

#ifdef USE_LOGGER
  if (logger::global_logger != nullptr) {
    logger::global_logger->add_on_log_callback(
        [this](int level, const char *tag, const char *message) { sendLog(level, tag, message); });
  }
#endif

#ifdef USE_BIND_SERVER
  if (binded_server->value() > 0) {
#endif

    ESP_LOGCONFIG(TAG, "Setting up meshmesh wifi...");
#ifdef USE_ESP32
    esp_err_t res;
#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
    wifi_config_t wcfg;

    strcpy((char *) wcfg.ap.ssid, "esphome");
    strcpy((char *) wcfg.ap.password, "esphome");
    wcfg.ap.ssid_len = 0;
    wcfg.ap.channel = mConfigChannel > 13 ? mPreferences.channel : mConfigChannel;
    wcfg.ap.authmode = WIFI_AUTH_OPEN;
    wcfg.ap.ssid_hidden = 1;
    wcfg.ap.max_connection = 4;
    wcfg.ap.beacon_interval = 60000;
#else
    wifi_config_t wcfg = {.ap = {
                              "esphome",
                              "esphome",
                          }};

    wcfg.ap.ssid_len = 0;
    wcfg.ap.channel = mConfigChannel > 13 ? mPreferences.channel : mConfigChannel;
    wcfg.ap.authmode = WIFI_AUTH_OPEN;
    wcfg.ap.ssid_hidden = 1;
    wcfg.ap.max_connection = 4;
    wcfg.ap.beacon_interval = 60000;

#endif
    esp_netif_t *netif;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    const wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
    res = esp_netif_init();
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_netif_init error %d", res);
      goto wifi_error;
    }

    res = esp_event_loop_create_default();
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_event_loop_create_default error %d", res);
      goto wifi_error;
    }

    netif = esp_netif_create_default_wifi_ap();
    if (!netif) {
      ESP_LOGE(TAG, "%s wifi ap creation failed: %s", __func__, esp_err_to_name(res));
      goto wifi_error;
    }

    res = esp_wifi_init(&cfg);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_init error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_storage error %s", esp_err_to_name(res));
      goto wifi_error;
    }

    res = esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_event_handler_instance_register error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_storage error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_mode(WIFI_MODE_AP);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_mode error %d", res);
      goto wifi_error;
    }

    wifiInitMacAddr(ESP_IF_WIFI_AP);

    res = esp_wifi_set_config(WIFI_IF_AP, &wcfg);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_config error %d", res);
      goto wifi_error;
    }

    ESP_LOGI(TAG, "Selected channel %d", wcfg.ap.channel);

    ESP_LOGD(TAG, "Start!!!");
    res = esp_wifi_start();
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_start error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_promiscuous(true);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_promiscuous error %d", res);
      goto wifi_error;
    }
    res = esp_wifi_set_promiscuous_filter(&filt);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_promiscuous_filter error %d", res);
      goto wifi_error;
    }

    /*res = esp_wifi_set_channel(mConfigChannel > 13 ? mPreferences.channel : mConfigChannel, WIFI_SECOND_CHAN_NONE);
    if(res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_channel error %d", res);
      goto wifi_error;
    }*/

    res = esp_wifi_set_max_tx_power(84);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_max_tx_power error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_protocol error %d", res);
      goto wifi_error;
    }

    res = esp_wifi_set_ps(WIFI_PS_NONE);
    if (res != ESP_OK) {
      ESP_LOGD(TAG, "esp_wifi_set_ps error %d", res);
      goto wifi_error;
    }

  wifi_error:

#else
  wifi_station_set_hostname((char *) App.get_name().c_str());
  wifi_set_opmode(STATION_MODE);
  wifiInitMacAddr(STATION_IF);
  wifi_station_set_auto_connect(false);
  wifi_set_phy_mode(PHY_MODE_11B);
  wifi_set_channel(mConfigChannel > 13 ? mPreferences.channel : mConfigChannel);
  system_phy_set_max_tpw(mPreferences.txPower);
  ESP_LOGCONFIG(TAG, "Channel cfg:%d pref:%d", mConfigChannel, mPreferences.channel);
#endif
    ESP_LOGD(TAG, "Wifi succesful!!!!");

#ifdef USE_BIND_SERVER
  }
#endif

  char aespassword[16];
  if (mAesPassword.size() == 0) {
    memcpy(aespassword, "1234567890ABCDEF", 16);
  } else {
    md5::MD5Digest md5;
    md5.init();
    md5.add(mAesPassword.c_str(), mAesPassword.size());
    md5.calculate();
    md5.get_bytes((uint8_t *) aespassword);
  }

  packetbuf = PacketBuf::getInstance();
  packetbuf->setup(aespassword, 16);
  broadcast = new Broadcast(packetbuf);
  unicast = new Unicast(packetbuf);

#ifdef USE_MULTIPATH_PROTOCOL
  multipath = new MultiPath(packetbuf);
  multipath->setup();
  multipath->setReceiveCallback(multipathRecvCb, this);
#endif

#ifdef USE_POLITE_BROADCAST_PROTOCOL
  mPoliteBroadcast = new PoliteBroadcastProtocol(packetbuf);
  mPoliteBroadcast->setup();
  packetbuf->setPoliteBroadcast(mPoliteBroadcast);
  mPoliteBroadcast->setReceivedHandler(politeBroadcastReceive, this);
#endif

#ifdef USE_CONNECTED_PROTOCOL
  mConnectedPath = new ConnectedPath(this, packetbuf);
  mConnectedPath->setup();
  mConnectedPath->bindPort(onConnectedPathNewClientCb, this, 0);
#endif

  mDiscovery.init();
  mRssiCheck.setup();
  broadcast->open();
  broadcast->setRecv_cb(user_broadcast_recv_cb);
  unicast->setup();
  unicast->bindPort(unicastRecvCb, this, UNICAST_DEFAULT_PORT);
  dump_config();
  mElapsed1 = millis();

#ifdef USE_TEST_PROCEDURE
  mTestProcedureTime = millis();
#endif
}

void MeshmeshComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Meshmesh");
#ifdef USE_ESP32
  ESP_LOGCONFIG(TAG, "Sys cip ID: %08lX", Discovery::chipId());
#else
  ESP_LOGCONFIG(TAG, "Sys cip ID: %08X", system_get_chip_id());
  ESP_LOGCONFIG(TAG, "Channel: %d", wifi_get_channel());
#ifdef USE_BIND_SERVER
  ESP_LOGCONFIG(TAG, "Bind server mode active with server 0x%06X", binded_server->value());
#endif
#endif
#ifdef USE_SENSOR
  for (auto sensor : App.get_sensors()) {
    ESP_LOGCONFIG(TAG, "Found sensor %s with hash %08X", sensor->get_object_id().c_str(), sensor->get_object_id_hash());
  }
#endif
#ifdef USE_BINARY_SENSOR
  for (auto sensor : App.get_binary_sensors()) {
    ESP_LOGCONFIG(TAG, "Found sensor %s with hash %08X", sensor->get_object_id().c_str(), sensor->get_object_id_hash());
  }
#endif
}

void MeshmeshComponent::loop() {
  if (mBaudRate > 0) {
#ifdef USE_ARDUINO
    int avail = mHwSerial->available();
    if (avail)
      ESP_LOGD(TAG, "MeshmeshComponent::loop available %d", avail);
    while (avail--)
      user_uart_recv_data((uint8_t) mHwSerial->read());
#endif

#ifdef USE_ESP_IDF
    size_t avail;
    uart_get_buffered_data_len(mUartNum, &avail);
    uint8_t data;
    while (avail--) {
      uart_read_bytes(mUartNum, &data, 1, 0);
      user_uart_recv_data(data);
    }
#endif
  }

  if (mUartTxBuffer.filledSpace() > 0)
    flushUartTxBuffer();
#ifdef USE_ESP32
  packetbuf->loop();
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  if (mPoliteBroadcast)
    mPoliteBroadcast->loop();
#endif
#ifdef USE_MULTIPATH_PROTOCOL
  // execute multipath loop
  multipath->loop();
#endif
#ifdef USE_CONNECTED_PROTOCOL
  mConnectedPath->loop();
#endif
  // Execute discovery if is running
  if (mDiscovery.isRunning())
    mDiscovery.loop(this);
  // Execute rrsicheck if is running
  if (mRssiCheck.isRunning())
    mRssiCheck.loop(this);

  // Execute delayed commands
  if (mDelayedCommandTime > 0) {
    uint32_t now = millis();
    if (MeshmeshComponent::elapsedMillis(now, mDelayedCommandStart) > mDelayedCommandTime) {
      switch (mDelayedCommand) {
        case CMD_REBOOT_REQ:
          App.reboot();
          break;
        case CMD_FLASH_OPER_REQ:
          flashDealyedHandleFrame();
          break;
      }
      mDelayedCommand = 0;
      mDelayedSubCommand = 0;
      mDelayedCommandTime = 0;
      mDelayedArgSize = 0;
      if (mDelayedArg) {
        delete mDelayedArg;
        mDelayedArg = nullptr;
      }
    }
  }

  uint32_t now = millis();

#if USE_ESP8266
  if (!mWorkAround && elapsedMillis(now, mElapsed1) > 5000) {
    mWorkAround = true;
    uint8_t data[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    uniCastSendData(data, 0x6, 0x111111);
    ESP_LOGI(TAG, "Sent dummy workaround packet");
  }
#endif

  if (elapsedMillis(now, mElapsed1) > 60000) {
#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
    ESP_LOGI(TAG, "Free Heap %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
#else
    ESP_LOGI(TAG, "Free Heap %d", ESP.getFreeHeap());
#endif
    mElapsed1 = millis();
  }

#ifdef USE_TEST_PROCEDURE
  loop_test_procedure();
#endif
}

void MeshmeshComponent::uartSendData(const uint8_t *buff, uint16_t len) {
  if (!mUseSerial)
    return;

  uint16_t i;
  mUartTxBuffer.pushByte(0xFE);
  for (i = 0; i < len; i++) {
    if (buff[i] == 0xEA || buff[i] == 0xEF || buff[i] == 0xFE)
      mUartTxBuffer.pushByte(0xEA);
    mUartTxBuffer.pushByte(buff[i]);
  }
  mUartTxBuffer.pushByte(0xEF);
  flushUartTxBuffer();
}

void MeshmeshComponent::broadCastSendData(const uint8_t *buff, uint16_t len) {
  if (broadcast)
    broadcast->send(buff, len);
}

void MeshmeshComponent::uniCastSendData(const uint8_t *buff, uint16_t len, uint32_t addr) {
  if (unicast)
    unicast->send(buff, len, addr, UNICAST_DEFAULT_PORT);
}

#ifdef USE_MULTIPATH_PROTOCOL
void MeshmeshComponent::multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen,
                                          uint8_t *path) {
  if (!multipath)
    return;
  MultiPathPacket *pkt = new MultiPathPacket(nullptr, nullptr);
  pkt->allocClearData(len, pathlen);
  pkt->multipathHeader()->trargetAddress = addr;
  for (int i = 0; i < pathlen; i++)
    pkt->setPathItem(uint32FromBuffer(path) + i * sizeof(uint32_t), i);
  pkt->setPayload(buff);
  multipath->send(pkt, true);
}
#endif

#ifdef USE_SENSOR
sensor::Sensor *MeshmeshComponent::findSensorByUnit(const std::string &unit) {
  sensor::Sensor *result = nullptr;
  auto sensors = App.get_sensors();
  for (auto sensor : sensors) {
    result = sensor;
    if (unit == sensor->get_unit_of_measurement()) {
      result = sensor;
      break;
    }
  }
  return result;
}

sensor::Sensor *MeshmeshComponent::findSensor(uint16_t hash) {
  sensor::Sensor *result = nullptr;
  auto sensors = App.get_sensors();
  for (auto sensor : sensors) {
    result = sensor;
    if (hash == (sensor->get_object_id_hash() & 0xFFFF)) {
      result = sensor;
      break;
    }
  }
  return result;
}
#endif

#ifdef USE_BINARY_SENSOR
binary_sensor::BinarySensor *MeshmeshComponent::findBinarySensor(uint16_t hash) {
  binary_sensor::BinarySensor *result = nullptr;
  auto sensors = App.get_binary_sensors();
  for (auto sensor : sensors) {
    if (hash == (sensor->get_object_id_hash() & 0xFFFF)) {
      result = sensor;
      break;
    }
  }
  return result;
}
#endif

#ifdef USE_LIGHT
light::LightState *MeshmeshComponent::findLightState(uint16_t hash) {
  light::LightState *result = nullptr;
  auto lights = App.get_lights();
  for (auto light : lights) {
    if (hash == (light->get_object_id_hash() & 0xFFFF)) {
      result = light;
      break;
    }
  }
  return result;
}
#endif

#ifdef USE_TEXT_SENSOR
text_sensor::TextSensor *MeshmeshComponent::findTextSensor(uint16_t hash) {
  text_sensor::TextSensor *result = nullptr;
  auto sensors = App.get_text_sensors();
  for (auto sensor : sensors) {
    if (hash == (sensor->get_object_id_hash() & 0xFFFF)) {
      result = sensor;
      break;
    }
  }
  return result;
}
#endif

#ifdef USE_SWITCH
switch_::Switch *MeshmeshComponent::findSwitch(uint16_t hash) {
  switch_::Switch *result = nullptr;
  auto switches = App.get_switches();
  for (auto switch_ : switches) {
    if (hash == (switch_->get_object_id_hash() & 0xFFFF)) {
      result = switch_;
      break;
    }
  }
  return result;
}
#endif

MeshmeshComponent::EnityType MeshmeshComponent::findEntityTypeByHash(uint16_t hash) {
#ifdef USE_SENSOR
  if (findSensor(hash) != nullptr)
    return SensorEntity;
#endif
#ifdef USE_BINARY_SENSOR
  if (findBinarySensor(hash) != nullptr)
    return BinarySensorEntity;
#endif
#ifdef USE_SWITCH
    // if(findSwitch(hash) != nullptr) return SwitchEntity;
#endif
#ifdef USE_LIGHT
  if (findLightState(hash) != nullptr)
    return LightEntity;
#endif
  return AllEntities;
}

#define DEF_CMD_BUFFER_SIZE 0x440
#define MAX_CMD_BUFFER_SIZE 0x440

#ifdef USE_ESP_IDF
void MeshmeshComponent::initIdfUart() {
  if (!mBaudRate)
    return;

  uart_config_t uart_config{};
  uart_config.baud_rate = (int) mBaudRate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  uart_config.source_clk = UART_SCLK_DEFAULT;
#endif
  int uartNum = 0;
  uart_param_config(uartNum, &uart_config);  // FIXME
  uart_driver_install(uartNum, mTxBuffer, mTxBuffer, 10, nullptr, 0);
}
#endif

void MeshmeshComponent::user_uart_recv_data(uint8_t byte) {
  switch (mRecvState) {
    case WAIT_MAGICK:
      if (byte == CMD_MAGICK) {
        // ESP_LOGD(TAG, "MeshmeshComponent::user_uart_recv_data find start");
        mRecvState = WAIT_DATA;
        if (!mRecvBuffer) {
          mRecvBuffer = new uint8_t[DEF_CMD_BUFFER_SIZE];
        }
        os_memset(mRecvBuffer, 0, DEF_CMD_BUFFER_SIZE);
        mRecvBufferPos = 0;
      }
      break;
    case WAIT_DATA:
      if (byte == 0xEF) {
        handleFrame(mRecvBuffer, mRecvBufferPos, SRC_SERIAL, 0xFFFFFFFF);
        mRecvState = WAIT_MAGICK;
      } else {
        if (byte == 0xEA) {
          mRecvState = WAIT_ESCAPE;
        } else {
          mRecvBuffer[mRecvBufferPos++] = byte;
        }
      }
      break;
    case WAIT_ESCAPE:
      if (byte == 0xEA || byte == 0xEF || byte == 0xFE) {
        mRecvBuffer[mRecvBufferPos++] = byte;
      }
      mRecvState = WAIT_DATA;
      break;
  }
}

void MeshmeshComponent::flushUartTxBuffer() {
#ifdef USE_ARDUINO
  int avail = mHwSerial->availableForWrite() - 8;
#endif

#ifdef USE_ESP_IDF
  int avail = mUartTxBuffer.filledSpace();
#endif

  if (avail > 0) {
    uint8_t *buff = new uint8_t[avail];
    avail = mUartTxBuffer.popData(buff, avail);
#ifdef USE_ARDUINO
    mHwSerial->write(buff, avail);
#endif

#ifdef USE_ESP_IDF
    uart_write_bytes(mUartNum, buff, avail);
#endif

    delete[] buff;
  }
}

void MeshmeshComponent::commandReply(const uint8_t *buff, uint16_t len) {
  uint8_t err = 0;
  switch (commandSource) {
    case SRC_SERIAL:
      uartSendData(buff, len);
      break;
    case SRC_BROADCAST:
      err = broadcast->send(buff, len);
      break;
    case SRC_UNICAST:
      err = unicast->send(buff, len, uint32FromBuffer(mRecvFromId), UNICAST_DEFAULT_PORT);
      break;
    case SRC_MULTIPATH:
#ifdef USE_MULTIPATH_PROTOCOL
      err = multipath->send(buff, len, mRecvFromId, mRecvPath, mRecvPathSize, true);
#endif
      break;
    case SRC_POLITEBRD:
#ifdef USE_POLITE_BROADCAST_PROTOCOL
      if (mPoliteFromAddress != POLITE_DEST_BROADCAST)
        mPoliteBroadcast->send(buff, len, mPoliteFromAddress);
#endif
      break;
    case SRC_CONNPATH:
#ifdef USE_CONNECTED_PROTOCOL
      mConnectedPath->sendDataTo(buff, len, mConnectionId);
#endif
      break;
    case SRC_FILTER:
      break;
  }

  commandSource = SRC_SERIAL;
}

void MeshmeshComponent::handleFrame(uint8_t *buf, uint16_t len, DataSrc src, uint32_t from) {
  uint8_t err = HANDLE_UART_ERROR;

  // ESP_LOGD(TAG, "MeshmeshComponent::handleFrame src %d cmd %02X len %d", src, buf[0], len);
  // print_hex_array("handleFrame ", buf, len);
  commandSource = src;
  if (buf[0] & 0x01) {
    replyHandleFrame(buf, len, src, from);
    return;
  }

  switch (buf[0]) {
    case CMD_UART_ECHO_REQ:
      if (len > 1) {
        buf[0] = CMD_UART_ECHO_REP;
        uartSendData(buf, len);
        err = 0;
      }
      break;
    case CMD_NODE_ID_REQ:  // 04 --> 05 0000XXYY
      if (len == 1) {
        uint32_t id = Discovery::chipId();
        uint8_t rep[5] = {0};
        rep[0] = CMD_NODE_ID_REP;
        os_memcpy(rep + 1, (uint8_t *) &id, 4);
        commandReply(rep, 5);
        err = 0;
      }
      break;
    case CMD_NODE_TAG_REQ:
      if (len == 1) {
        uint8_t rep[33] = {0};
        rep[0] = CMD_NODE_TAG_REP;
        os_memcpy(rep + 1, mPreferences.devicetag, 32);
        commandReply(rep, 33);
        err = 0;
      }
      break;
    case CMD_NODE_TAG_SET_REQ:
      if (len > 1) {
        os_memcpy(mPreferences.devicetag, buf + 1, len - 1);
        mPreferences.devicetag[len - 1] = 0;
        mPreferencesObject.save(&mPreferences);
        buf[0] = CMD_NODE_TAG_SET_REP;
        commandReply(buf, 1);
        err = 0;
      }
      break;
    case CMD_BIND_CLEAR_REQ:
      if (len == 1) {
#ifdef USE_BIND_SERVER
        binded_server->value() = 0;
        buf[0] = CMD_BIND_CLEAR_REP;
        commandReply(buf, 1);
        err = 0;
#endif
      }
      break;
    case CMD_LOG_DEST_REQ:
      if (len == 1) {
        uint8_t rep[5] = {0};
        rep[0] = CMD_LOG_DEST_REP;
        uint32toBuffer(rep + 1, mPreferences.log_destination);
        commandReply(rep, 5);
        err = 0;
      }
      break;
    case CMD_LOG_DEST_SET_REQ:
      if (len == 5) {
        mPreferences.log_destination = uint32FromBuffer(buf + 1);
        mPreferencesObject.save(&mPreferences);
        buf[0] = CMD_LOG_DEST_SET_REP;
        commandReply(buf, 1);
        err = 0;
      }
      break;
    case CMD_GROUPS_REQ:
      if (len == 1) {
        uint8_t rep[5] = {0};
        rep[0] = CMD_GROUPS_REP;
        uint32toBuffer(rep + 1, mPreferences.groups);
        commandReply(rep, 5);
        err = 0;
      }
      break;
    case CMD_GROUPS_SET_REQ:
      if (len == 5) {
        mPreferences.groups = uint32FromBuffer(buf + 1);
        mPreferencesObject.save(&mPreferences);
        buf[0] = CMD_GROUPS_SET_REP;
        commandReply(buf, 1);
        err = 0;
      }
      break;
    case CMD_FIRMWARE_REQ:
      if (len == 1) {
        uint8_t rep[32];
        uint16_t lenstr = App.get_compilation_time().length();
        if (lenstr > 31)
          lenstr = 31;
        rep[0] = CMD_FIRMWARE_REP;
        strncpy((char *) rep + 1, App.get_compilation_time().c_str(), 31);
        commandReply((const uint8_t *) rep, lenstr + 1);
        err = 0;
      }
      break;
    case CMD_REBOOT_REQ:
      if (len == 1) {
        mDelayedCommand = CMD_REBOOT_REQ;
        mDelayedCommandStart = millis();
        mDelayedCommandTime = 250;
        buf[0] = CMD_REBOOT_REP;
        commandReply(buf, 1);
        err = 0;
      }
      break;
    case CMD_DISCOVERY_REQ:
      if (len > 1) {
        err = mDiscovery.handle_frame(buf + 1, len - 1, this);
      }
      break;
    case CMD_RSSICHECK_REQ:
      if (len > 1) {
        err = mRssiCheck.handleFrame(buf + 1, len - 1, from, mRssiHandle, this);
      }
      break;
    case CMD_FLASH_OPER_REQ:
      if (len > 1) {
        err = flashHandleFrame(buf + 1, len - 1);
      }
      break;
    case CMD_ENTITIES_COUNT_REQ:
      if (len == 1) {
        uint8_t rep[LastEntity + 1];
        rep[0] = CMD_ENTITIES_COUNT_REP;
        uint8_t *buf = rep + 1;
        os_memset(buf, 0, LastEntity);

#ifdef USE_SENSOR
        buf[SensorEntity] = (uint8_t) App.get_sensors().size();
        buf[AllEntities] += buf[SensorEntity];
#endif

#ifdef USE_BINARY_SENSOR
        buf[BinarySensorEntity] = (uint8_t) App.get_binary_sensors().size();
        buf[AllEntities] += buf[BinarySensorEntity];
#endif

#ifdef USE_SWITCH
        buf[SwitchEntity] = (uint8_t) App.get_switches().size();
        buf[AllEntities] += buf[SwitchEntity];
#endif

#ifdef USE_LIGHT
        buf[LightEntity] = (uint8_t) App.get_lights().size();
        buf[AllEntities] += buf[LightEntity];
#endif
#ifdef USE_TEXT_SENSOR
        buf[TextSensorEntity] = (uint8_t) App.get_text_sensors().size();
        buf[AllEntities] += buf[TextSensorEntity];
#endif
        commandReply(rep, LastEntity + 1);
        err = 0;
      }
      break;

    case CMD_ENTITY_HASH_REQ:
      if (len == 3) {
        uint8_t service = buf[1];
        uint8_t index = buf[2];
        uint16_t hash = 0;
        std::string info;
        bool hashfound = false;

        switch (service) {
          case SensorEntity:
#ifdef USE_SENSOR
            if (index < App.get_sensors().size()) {
              auto sensor = App.get_sensors().at(index);
              hash = sensor->get_object_id_hash() & 0xFFFF;
              info = sensor->get_name() + "," + sensor->get_object_id() + "," + sensor->get_unit_of_measurement();
              hashfound = true;
            }
#endif
            break;
          case BinarySensorEntity:
#ifdef USE_BINARY_SENSOR
            if (index < App.get_binary_sensors().size()) {
              auto binary = App.get_binary_sensors().at(index);
              hash = binary->get_object_id_hash() & 0xFFFF;
              info = binary->get_name() + "," + binary->get_object_id();
              hashfound = true;
            }
#endif
            break;
          case SwitchEntity:
#ifdef USE_SWITCH
            if (index < App.get_switches().size()) {
              auto switch_ = App.get_switches().at(index);
              hash = switch_->get_object_id_hash() & 0xFFFF;
              info = switch_->get_name() + "," + switch_->get_object_id();
              hashfound = true;
            }
#endif
            break;
          case LightEntity:
#ifdef USE_LIGHT
            if (index < App.get_lights().size()) {
              auto light = App.get_lights().at(index);
              hash = light->get_object_id_hash() & 0xFFFF;
              info = light->get_name() + "," + light->get_object_id();
              hashfound = true;
            }
#endif
            break;
          case TextSensorEntity:
#ifdef USE_TEXT_SENSOR
            if (index < App.get_text_sensors().size()) {
              auto texts = App.get_text_sensors().at(index);
              hash = texts->get_object_id_hash() & 0xFFFF;
              info = texts->get_name() + "," + texts->get_object_id();
              hashfound = true;
            }
#endif
            break;
          case AllEntities:
          case LastEntity:
            break;
        }

        if (hashfound) {
          auto rep = new uint8_t[info.length() + 3];
          rep[0] = CMD_ENTITY_HASH_REP;
          uint16toBuffer(rep + 1, hash);
          os_memcpy(rep + 3, info.c_str(), info.length());
          commandReply(rep, 3 + info.length());
          delete rep;
          err = 0;
        }
      }
      break;

    case CMD_GET_ENTITY_STATE_REQ:
      if (len == 4) {
        EnityType type = (EnityType) buf[1];
        uint16_t hash = uint16FromBuffer(buf + 2);
        ESP_LOGD(TAG, "CMD_GET_ENTITY_STATE_REQ %04X hash %d type", hash, type);
        int16_t value = 0;
        std::string value_str;
        uint8_t value_type = 0;

        switch (type) {
          case SensorEntity: {
#ifdef USE_SENSOR
            sensor::Sensor *sensor = findSensor(hash);
            value = (int16_t) (sensor->state * 10.0);
            value_type = 1;
#endif
          } break;
          case BinarySensorEntity: {
#ifdef USE_BINARY_SENSOR
            binary_sensor::BinarySensor *binary = findBinarySensor(hash);
            value = binary->state ? 10 : 0;
            value_type = 1;
#endif
          } break;
          case SwitchEntity: {
#ifdef USE_SWITCH
            auto switch_ = findSwitch(hash);
            value = switch_->state ? 1 : 0;
            value_type = 1;
#endif
          } break;
          case LightEntity: {
#ifdef USE_LIGHT
            light::LightState *state = findLightState(hash);
            if (state->current_values.get_state() == 0)
              value = 0;
            else
              value = (uint16_t) (state->current_values.get_brightness() * 1024.0);
            value_type = 1;
#endif
          } break;
          case TextSensorEntity: {
#ifdef USE_TEXT_SENSOR
            text_sensor::TextSensor *texts = findTextSensor(hash);
            value_str = texts->state;
            value_type = 2;
#endif
          } break;
          case AllEntities:
          case LastEntity:
            break;
        }

        if (value_type == 1) {
          uint8_t rep[3];
          rep[0] = CMD_GET_ENTITY_STATE_REP;
          uint16toBuffer(rep + 1, value);
          commandReply(rep, 3);
          err = 0;
        } else if (value_type == 2) {
          uint16_t rep_size = 2 + value_str.length();
          auto *rep = new uint8_t[rep_size];
          rep[0] = CMD_GET_ENTITY_STATE_REP;
          rep[1] = value_type;
          os_memcpy(rep + 2, value_str.data(), value_str.length());
          commandReply(rep, rep_size);
          err = 0;
        }
      }
      break;

    case CMD_SET_ENTITY_STATE_REQ:
      if (len == 6) {
        EnityType type = (EnityType) buf[1];
        uint16_t hash = uint16FromBuffer(buf + 2);
        uint16_t value = uint16FromBuffer(buf + 4);
        ESP_LOGD(TAG, "CMD_SET_ENTITY_STATE_REQ type %d hash %04X value %d", type, hash, value);
        bool valuefound = 0;

        switch (type) {
          case LightEntity: {
#ifdef USE_LIGHT
            light::LightState *state = findLightState(hash);
            if (state != nullptr) {
              auto call = state->make_call();
              call.set_state(value > 0 ? 1 : 0);
              call.set_brightness((float) value / 1024.0f);
              call.perform();
              valuefound = 1;
            }
#endif
          } break;
          case SwitchEntity: {
#ifdef USE_SWITCH
            auto *state = findSwitch(hash);
            if (state != nullptr) {
              if (value)
                state->turn_on();
              else
                state->turn_off();
              valuefound = 1;
            }
#endif
          } break;
          case SensorEntity:
          case BinarySensorEntity:
          case TextSensorEntity:
            break;
          case AllEntities:
          case LastEntity:
            break;
        }

        if (valuefound) {
          uint8_t rep[1];
          rep[0] = CMD_SET_ENTITY_STATE_REP;
          commandReply(rep, 1);
          err = 0;
        }
      }
      break;

    case CMD_ENTITY_REQ:  // 48
      if (len > 1) {
        ESP_LOGD(TAG, "CMD_ENTITY_REQ len %d", len);
        err = Entities::handle_frame(buf + 1, len - 1, this);
      }
      break;

    case CMD_BROADCAST_SEND:  // 70 AABBCCDDEE...ZZ
      if (len > 1) {
        broadcast->send(buf + 1, len - 1);
        err = 0;
      }
      break;
    case CMD_UNICAST_SEND:  // 72 0000XXYY AABBCCDDEE...ZZ
      if (len > 5) {
        unicast->send(buf + 5, len - 5, uint32FromBuffer(buf + 1), UNICAST_DEFAULT_PORT);
        err = 0;
      }
      break;
    case CMD_MULTIPATH_SEND:  // 76 AAAAAAAA BB CCCCCCCC........DDDDDDDD AABBCCDDEE...ZZ
#ifdef USE_MULTIPATH_PROTOCOL
      if (len > 5) {
        uint8_t pathlen = buf[5];
        if (len > 6 + pathlen * sizeof(uint32_t)) {
          uint16_t payloadsize = len - (6 + pathlen * sizeof(uint32_t));
          MultiPathPacket *pkt = new MultiPathPacket(nullptr, nullptr);
          pkt->allocClearData(payloadsize, pathlen);
          pkt->multipathHeader()->sourceAddress = packetbuf->nodeId();
          pkt->multipathHeader()->trargetAddress = uint32FromBuffer(buf + 1);
          for (int i = 0; i < pathlen; i++)
            pkt->setPathItem(uint32FromBuffer(buf + 6) + i * sizeof(uint32_t), i);
          pkt->setPayload(buf + 6 + sizeof(uint32_t) * pathlen);
          multipath->send(pkt, true);
          err = 0;
        }
      }
#endif
      break;
#ifdef USE_POLITE_BROADCAST_PROTOCOL
    case CMD_POLITEBRD_SEND:
      if (len > 5) {
        mPoliteBroadcast->send(buf + 5, len - 5, uint32FromBuffer(buf + 1));
        err = 0;
      }
      break;
#endif
#ifdef USE_CONNECTED_PROTOCOL
    case CMD_CONNPATH_REQUEST:
      if (len > 1 && src == SRC_SERIAL) {
        err = mConnectedPath->receiveUartPacket(buf + 1, len - 1);
      }
      break;
#endif
    case CMD_FILTERED_REQUEST:
      if (len > 5) {
        uint8_t i;
        uint8_t *groups = (uint8_t *) &mPreferences.groups;
        for (i = 0; i < 4; i++)
          if (buf[i + 1] != 0xFF && (groups[i] == 0 || (groups[i] != 0xFF && groups[i] != buf[i + 1])))
            break;
        if (i == 4) {
          handleFrame(buf + 5, len - 5, SRC_FILTER, 0);
        }
        err = 0;
      }
      break;
    default:
      // Silently discard unknow commands
      break;
  }

  if (err == HANDLE_UART_ERROR && commandSource != SRC_BROADCAST) {
    // Don't reply errors when commd came from broadcast
    uint8_t *rep = new uint8_t[len + 1];
    rep[0] = CMD_ERROR_REP;
    os_memcpy(rep + 1, buf, len);
    commandReply(rep, len + 1);
    ESP_LOGD(TAG, "MeshmeshComponent::handleFrame error frame %02X %02X size %d", buf[0], buf[1], len);
    delete[] rep;
  }
}

void MeshmeshComponent::replyHandleFrame(uint8_t *buf, uint16_t len, DataSrc src, uint32_t from) {
  // All replies go to the serial, if the serial is active
  switch (buf[0]) {
    case CMD_LOGEVENT_REP:
      // Add the source to the log essage
      os_memcpy(buf + 3, (uint8_t *) &from, 4);
      uartSendData(buf, len);
      break;
    case CMD_BEACONS_RECV:
      // Compatibility with previous discovery procedure
      if (len == sizeof(BaconsDataCompat_t)) {
        BaconsDataCompat_t *b = (BaconsDataCompat_t *) buf;
        mDiscovery.process_beacon(b->id, b->rssi, 0);
      }
      break;
    case CMD_RSSICHECK_REP:
      if (len > 1) {
        mRssiCheck.handleFrame(buf + 1, len - 1, from, mRssiHandle, this);
      }
      break;
    default:
      uartSendData(buf, len);
      break;
  }
}

#define CMD_FLASH_GETMD5 0x01
#define CMD_FLASH_ERASE 0x02
#define CMD_FLASH_WRITE 0x03
#define CMD_FLASH_EBOOT 0x04

uint8_t MeshmeshComponent::flashHandleFrame(uint8_t *buf, uint16_t len) {
  uint8_t err = 1;

  switch (buf[0]) {
    case CMD_FLASH_GETMD5:
    case CMD_FLASH_ERASE:
    case CMD_FLASH_WRITE:
      if (len > 5) {
        mDelayedCommand = CMD_FLASH_OPER_REQ;
        mDelayedSubCommand = buf[0];
        mDelayedCommandStart = millis();
        mDelayedCommandTime = 15;
        mDelayedArgSize = len;
        mDelayedArg = new uint8_t[mDelayedArgSize];
        os_memcpy(mDelayedArg, buf, mDelayedArgSize);
        err = 0;
      }
      break;
    case CMD_FLASH_EBOOT:
      if (len == 9) {
#ifdef ARDUINO_ARCH_ESP8266
        uint32_t address = uint32FromBuffer(buf + 1);
        uint32_t length = uint32FromBuffer(buf + 5);
        eboot_command ebcmd;
        ebcmd.action = ACTION_COPY_RAW;
        ebcmd.args[0] = address;
        ebcmd.args[1] = 0x00000;
        ebcmd.args[2] = length;
        eboot_command_write(&ebcmd);
#endif
        uint8_t rep[2];
        rep[0] = CMD_FLASH_OPER_REP;
        rep[1] = CMD_FLASH_EBOOT;
        commandReply(rep, 2);
        err = 0;
      }
  }

  return err;
}

void MeshmeshComponent::flashDealyedHandleFrame() {
  switch (mDelayedSubCommand) {
    case CMD_FLASH_GETMD5:
      if (mDelayedArgSize == 9) {
#ifdef ARDUINO_ARCH_ESP8266
        MD5Builder md5;
        uint32_t address = uint32FromBuffer(mDelayedArg + 1);
        uint32_t length = uint32FromBuffer(mDelayedArg + 5);
        uint32_t readsize;

        md5.begin();
        uint8_t buff[128];
        bool erased = true;
        while (length > 0) {
          readsize = length > 128 ? 128 : length;
          ESP.flashRead(address, (uint32_t *) buff, readsize);
          md5.add(buff, readsize);
          address += readsize;
          length -= readsize;
          if (erased)
            for (int i = 0; i < 128; i++)
              if (buff[i] != 0xFF) {
                erased = false;
                break;
              }
        }
        md5.calculate();
#endif
#ifdef USE_ESP32
        bool erased = true;
#endif
        uint8_t rep[19];
        rep[0] = CMD_FLASH_OPER_REP;
        rep[1] = CMD_FLASH_GETMD5;
        rep[2] = erased ? 1 : 0;
#ifdef ARDUINO_ARCH_ESP8266
        md5.getBytes(rep + 3);
#endif
        commandReply(rep, 19);
      }
      break;
    case CMD_FLASH_ERASE:
      if (mDelayedArgSize == 9) {
        uint8_t res = 0;
#ifdef ARDUINO_ARCH_ESP8266
        uint32_t address = uint32FromBuffer(mDelayedArg + 1);
        uint32_t length = uint32FromBuffer(mDelayedArg + 5);
        while (length > 0) {
          if (ESP.flashEraseSector(address / FLASH_SECTOR_SIZE))
            res++;
          else
            break;
          length = length > FLASH_SECTOR_SIZE ? length - FLASH_SECTOR_SIZE : 0;
          address += FLASH_SECTOR_SIZE;
        }
#endif
        uint8_t rep[3];
        rep[0] = CMD_FLASH_OPER_REP;
        rep[1] = CMD_FLASH_ERASE;
        rep[2] = res;
        commandReply(rep, 3);
      }
      break;
    case CMD_FLASH_WRITE:
      if (mDelayedArgSize > 5) {
        uint32_t length = mDelayedArgSize - 5;
        uint32_t *buffer = new uint32_t[length / 4];
        os_memcpy(buffer, mDelayedArg + 5, length);
#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
        // FIXME
        bool result = 0;
#else
        uint32_t address = uint32FromBuffer(mDelayedArg + 1);
        bool result = ESP.flashWrite(address, buffer, length);
#endif
        delete[] buffer;

        uint8_t rep[3];
        rep[0] = CMD_FLASH_OPER_REP;
        rep[1] = CMD_FLASH_WRITE;
        rep[2] = result ? 0 : 1;  // error = !result
        commandReply(rep, 3);
      }
      break;
  }
}

void MeshmeshComponent::user_broadcast_recv_cb(uint8_t *data, uint16_t size, uint8_t *from) {
  if (singleton) {
    singleton->user_broadcast_recv(data, size, from);
  }
}

void MeshmeshComponent::user_broadcast_recv(uint8_t *data, uint16_t size, uint8_t *from) {
  // Ignore error frame frmo broadcast
  if (size == 0 || data[0] == 0x7F)
    return;
  os_memcpy(&mBroadcastFromAddress, from, 4);
  uint32_t *addr = (uint32_t *) from;
  ESP_LOGD(TAG, "MeshmeshComponent::user_broadcast_recv from %06lX size %d cmd %02X", *addr, size, data[0]);
  handleFrame(data, size, SRC_BROADCAST, *(uint32_t *) from);
}

void MeshmeshComponent::unicastRecvCb(void *arg, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
  ((MeshmeshComponent *) arg)->unicastRecv(data, size, from, rssi);
}

void MeshmeshComponent::unicastRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
  // ESP_LOGD(TAG, "unicastRecv %d", size);
  os_memcpy(mRecvFromId, (uint8_t *) &from, 4);
  mRssiHandle = rssi;
  handleFrame(data, size, SRC_UNICAST, from);
  if (mRssiCheck.useRssiStatistics())
    mRssiCheck.newRssiData(from, rssi);
}

void MeshmeshComponent::multipathRecvCb(void *arg, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi,
                                        uint8_t *path, uint8_t pathSize) {
  ((MeshmeshComponent *) arg)->multipathRecv(data, size, from, rssi, path, pathSize);
}

void MeshmeshComponent::multipathRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path,
                                      uint8_t pathSize) {
  os_memcpy(mRecvFromId, (uint8_t *) &from, 4);
  mRecvPathSize = pathSize;
  if (mRecvPathSize)
    os_memcpy(mRecvPath, path, mRecvPathSize * sizeof(uint32_t));
  mRssiHandle = rssi;
  handleFrame(data, size, SRC_MULTIPATH, from);
}

void MeshmeshComponent::politeBroadcastReceive(void *arg, uint8_t *data, uint16_t size, uint32_t from) {
  ((MeshmeshComponent *) arg)->politeBroadcastReceiveCb(data, size, from);
}

void MeshmeshComponent::politeBroadcastReceiveCb(uint8_t *data, uint16_t size, uint32_t from) {
#ifdef USE_POLITE_BROADCAST_PROTOCOL
  if (size == 0 || data[0] == 0x7F)
    return;
  mPoliteFromAddress = from;
  handleFrame(data, size, SRC_POLITEBRD, from);
#endif
}

void MeshmeshComponent::onConnectedPathNewClientCb(void *arg, uint32_t from, uint16_t handle) {
  ((MeshmeshComponent *) arg)->onConnectedPathNewClient(from, handle);
}

void MeshmeshComponent::onConnectedPathNewClient(uint32_t from, uint16_t handle) {
#ifdef USE_CONNECTED_PROTOCOL
  ESP_LOGD(TAG, "MeshmeshComponent::onConnectedPathNewClien %06lX:%04X", from, handle);
  mConnectedPath->setReceiveCallback(onConnectedPathReceiveCb, nullptr, this, from, handle);
#endif
}

void MeshmeshComponent::onConnectedPathReceiveCb(void *arg, uint8_t *data, uint16_t size, uint8_t connid) {
  ((MeshmeshComponent *) arg)->onConnectedPathReceive(data, size, connid);
}

void MeshmeshComponent::onConnectedPathReceive(uint8_t *data, uint16_t size, uint8_t connid) {
#ifdef USE_CONNECTED_PROTOCOL
  mConnectionId = connid;
  handleFrame(data, size, SRC_CONNPATH, connid);
#endif
}

void MeshmeshComponent::sendLog(int level, const char *tag, const char *payload) {
  if (mBaudRate == 0 && mPreferences.log_destination == 0)
    return;

  // uint16_t taglen = os_strlen(tag);
  uint16_t payloadlen = os_strlen(payload);
  // uint16_t buffersize = 7+taglen+1+payloadlen;
  uint16_t buffersize = 7 + payloadlen;

  auto buffer = new uint8_t[buffersize];
  auto buffer_ptr = buffer;

  *buffer_ptr = CMD_LOGEVENT_REP;
  buffer_ptr++;
  uint16toBuffer(buffer_ptr, (uint16_t) level);
  buffer_ptr += 2;
  uint32toBuffer(buffer_ptr, 0);
  buffer_ptr += 4;

  if (payloadlen > 0)
    os_memcpy(buffer_ptr, payload, payloadlen);
  if (mBaudRate > 0)
    uartSendData(buffer, buffersize);
  if (mPreferences.log_destination == 1) {
    if (broadcast)
      broadcast->send(buffer, buffersize);
  } else if (mPreferences.log_destination > 1) {
    if (unicast)
      unicast->send(buffer, buffersize, mPreferences.log_destination, UNICAST_DEFAULT_PORT);
  }

  delete[] buffer;
}

#ifdef USE_TEST_PROCEDURE
void MeshmeshComponent::test_light_broadcast(uint16_t value) {
  uint8_t buffer[6];
  buffer[0] = CMD_SET_ENTITY_STATE_REQ;
  buffer[1] = (uint8_t) LightEntity;
  uint16toBuffer(buffer + 2, 0xe55f);
  uint16toBuffer(buffer + 4, value);
  broadCastSendData(buffer, 6);
}

void MeshmeshComponent::loop_test_procedure(void) {
  uint32_t now = millis();
  switch (mTestProcedureState) {
    case 0:
      if (elapsedMillis(now, mTestProcedureTime) > 3500) {
        test_light_broadcast(1024);
        mTestProcedureState++;
        mTestProcedureTime = now;
      }
      break;
    case 1:
      if (elapsedMillis(now, mTestProcedureTime) > 3000) {
        test_light_broadcast(0);
        mTestProcedureState++;
        mTestProcedureTime = now;
      }
      break;
    case 2:
      if (elapsedMillis(now, mTestProcedureTime) > 3000) {
        test_light_broadcast(512);
        mTestProcedureState++;
        mTestProcedureTime = now;
      }
      break;
    case 3:
      if (elapsedMillis(now, mTestProcedureTime) > 3000) {
        test_light_broadcast(1024);
        mTestProcedureState++;
        mTestProcedureTime = now;
      }
      break;
    case 4:
      if (elapsedMillis(now, mTestProcedureTime) > 3000) {
        test_light_broadcast(0);
        mTestProcedureState++;
        mTestProcedureTime = now;
      }
      break;
    default:
      break;
  }
}
#endif

void MeshmeshComponent::wifiInitMacAddr(uint8_t index) {
  uint32_t id = Discovery::chipId();
  uint8_t *idptr = (uint8_t *) &id;
  uint8_t mac[6] = {0};
  mac[0] = 0xFE;
  mac[1] = 0x7F;
  mac[2] = idptr[3];
  mac[3] = idptr[2];
  mac[4] = idptr[1];
  mac[5] = idptr[0];

  ESP_LOGD(TAG, "wifiInitMacAddr %02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

#ifdef USE_ESP32
  esp_err_t res = esp_wifi_set_mac((wifi_interface_t) index, mac);
  if (res != ESP_OK) {
    ESP_LOGD(TAG, "esp_wifi_set_mac error %d", res);
  }
#else
  wifi_set_macaddr(index, mac);
#endif
}

}  // namespace meshmesh
}  // namespace esphome
