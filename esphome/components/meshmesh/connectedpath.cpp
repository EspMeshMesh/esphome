
#include "connectedpath.h"
#ifdef USE_CONNECTED_PROTOCOL
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include "meshmesh.h"
#include "commands.h"

#define RES_OK 0
#define RES_ERROR 1
#define RES_INVALID_HANDLE 2
#define FORWARD2TXT(X) (X ? "-->" : "<--")

namespace esphome {
namespace meshmesh {

static const char *TAG = "meshmesh.ConnectedPath";

#define CONNPATH_MAX_RETRANSMISSIONS 0x04

#define CONNPATH_OPEN_CONNECTION_REQ 0x01
#define CONNPATH_OPEN_CONNECTION_REP 0x02
#define CONNPATH_DISCONNECT_ACK 0x03
#define CONNPATH_INVALID_HANDLE 0x04
#define CONNPATH_SEND_DATA 0x05
#define CONNPATH_OPEN_CONNECTION_ACK 0x06
#define CONNPATH_OPEN_CONNECTION_NACK 0x07
#define CONNPATH_DISCONNECT_REQ 0x08
#define CONNPATH_SEND_DATA_ERROR 0x09
#define CONNPATH_CLEAR_CONNECTIONS 0x0A

#define CONN_IS_VALID(X) X < CONNPATH_MAX_CONNECTIONS

void ConnectedPathPacket::allocClearData(uint16_t size) {
  RadioPacket::allocClearData(size + sizeof(ConnectedPathHeaderSt));
  getHeader()->dataLength = size;
}

void ConnectedPathPacket::setPayload(const uint8_t *payoad) {
  os_memcpy(clearData() + sizeof(ConnectedPathHeaderSt), payoad, getHeader()->dataLength);
}

void ConnectedPathPacket::setTarget(uint32_t target, uint16_t handle) {
  mTarget = target;
  if (clearData() != nullptr)
    getHeader()->sourceHandle = handle;
}

void ConnectedPath::setup(void) {
  os_memset((uint8_t *) mConnectsions, 0x0, sizeof(mConnectsions));
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++)
    connectionSetInvalid(i);
  mConnectionsCheckTime = millis();
}

void ConnectedPath::loop() {
  mRecvDups.loop();

  if (mRetransmitPacket != nullptr) {
    sendRawRadioPacket(mRetransmitPacket);
    mRetransmitPacket = nullptr;
  } else if (mRadioOutputBuffer.filledSpace() > 0 && mIsRadioBusy == false) {
    processOutputBuffer();
  }

  uint32_t now = millis();
  if (MeshmeshComponent::elapsedMillis(now, mConnectionsCheckTime) > 120000) {
    mConnectionsCheckTime = now;
    // debugConnection();
    for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
      if (mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS &&
          MeshmeshComponent::elapsedMillis(now, mConnectsions[i].lastTime) > 300000) {
        closeConnection_(mConnectsions + i);
        connectionSetInvalid(i);
      }
    }
  }
}

uint8_t ConnectedPath::sendRawRadioPacket(ConnectedPathPacket *pkt) {
  ESP_LOGD(TAG, "ConnectedPath::sendRawRadioPacket sending to %06X %d bytes with flags %d", pkt->getTarget(),
           pkt->clearDataSize(), pkt->getHeader()->flags);
  if (pkt->encryptClearData()) {
    mIsRadioBusy = true;
    uint32_t target = pkt->getTarget();
    pkt->fill80211((uint8_t *) &target, mPacketBuf->nodeIdPtr());
    uint8_t res = mPacketBuf->send(pkt);
    if (res == PKT_SEND_ERR)
      delete pkt;
    return res;
  } else {
    return PKT_SEND_ERR;
  }
}

uint8_t ConnectedPath::sendRadioPacket(ConnectedPathPacket *pkt, bool forward, bool initHeader) {
  ConnectedPathHeader_t *header = pkt->getHeader();
  // Fill protocol header...
  header->protocol = PROTOCOL_CONNPATH;
  // Optional fields
  if (initHeader) {
    // Add flags to this packet
    header->flags = forward ? 0x00 : CONNPATH_FLAG_REVERSEDIR;
    // If is an ACK i use the last seqno
    header->seqno = ++mLastSequenceNum;
  }
  // Set this class as destination sent callback for retransmisisons
  pkt->setCallback(radioPacketSentCb, this);

  return sendRawRadioPacket(pkt);
}

void ConnectedPath::sendRadioDataTo(const uint8_t *data, uint16_t size, uint8_t connid, bool forward) {
  if (connid >= CONNPATH_MAX_CONNECTIONS || mConnectsions[connid].sourceAddr == CONNPATH_INVALID_ADDRESS) {
    return;
  }

  ConnectedPathOutputBufferHeader header;
  header.pkttime = millis();
  header.connId = connid;
  header.forward = forward ? 1 : 0;
  header.dataSize = size;
  mRadioOutputBuffer.pushData((uint8_t *) &header, sizeof(header));
  mRadioOutputBuffer.pushData(data, size);

  ESP_LOGD(TAG, "ConnectedPath::sendRadioDataTo connid %d size %d from %06X:%04X %s to %06X:%04X", connid, size,
           mConnectsions[connid].sourceAddr, mConnectsions[connid].sourceHandle, FORWARD2TXT(forward),
           mConnectsions[connid].destAddr, mConnectsions[connid].destHandle);
}

void ConnectedPath::sendRadioDataTo(const uint8_t *data, uint16_t size, uint32_t from, uint16_t handle) {
  bool forward;
  uint8_t connid = findConnectionIndex(from, handle, &forward);
  if (connid < CONNPATH_MAX_CONNECTIONS)
    sendRadioDataTo(data, size, connid, forward);
}

void ConnectedPath::closeConnection_(ConnectedPathConnections *conn) {
  sendSimplePacket(CONNPATH_DISCONNECT_REQ, conn->sourceAddr, conn->sourceHandle, false);
  sendSimplePacket(CONNPATH_DISCONNECT_REQ, conn->destAddr, conn->destHandle, true);
  connectionSetInvalid(conn);
}

void ConnectedPath::closeConnection(uint32_t from, uint16_t handle) {
  ConnectedPathConnections *conn = findConnection(from, handle);
  if (conn != nullptr)
    closeConnection_(conn);
}

void ConnectedPath::closeAllConnections() {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
      closeConnection_(mConnectsions + i);
    }
  }
}

/**
 * @brief Receive a ConnectedPath packet from the UART and handle it using the correct action.
 *
 * @param data - The packet data to be processed.
 * @param size - The size of the packet data.
 * @return uint8_t - The result of the operation. 0 if successful, 1 if not.
 */
uint8_t ConnectedPath::receiveUartPacket(uint8_t *data, uint16_t size) {
  if (size >= sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) data;
    uint8_t *payload = data + sizeof(ConnectedPathHeaderSt);
    uint16_t payloadSize = size - sizeof(ConnectedPathHeaderSt);
    // ESP_LOGD(TAG, "ConnectedPath::receiveUartPacket size %d subp %d", size, header->subprotocol);

    if (header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
      openConnection(data, size, 0);
    } else if (header->subprotocol == CONNPATH_SEND_DATA) {
      if (sendData2(payload, payloadSize, 0, header->sourceHandle) == RES_INVALID_HANDLE) {
        sendSimplePacket(CONNPATH_INVALID_HANDLE, 0, header->sourceHandle, false);
      }
    } else if (header->subprotocol == CONNPATH_DISCONNECT_REQ) {
      disconnect(data, size, 0);
    } else if (header->subprotocol == CONNPATH_INVALID_HANDLE) {
      invalidHandle(0, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_CLEAR_CONNECTIONS) {
      closeAllConnections();
      mRecvDups.clear();
      ESP_LOGI(TAG, "All connection and duplicates tables has been cleared");
    } else {
      ESP_LOGE(TAG, "ConnectedPath::receiveUartPacket unknow sub protocol %d", header->subprotocol);
    }
  }
  return HANDLE_UART_OK;
}

/**
 * @brief Receive a ConnectedPath packet from the radio and handle it using the correct action.
 * @param buf - The packet data to be processed.
 * @param size - The size of the packet data. Must be greater than sizeof(ConnectedPathHeaderSt).
 * @param f - The source address of the packet.
 * @param r - The RSSI of the packet.
 */
void ConnectedPath::receiveRadioPacket(uint8_t *buf, uint16_t size, uint32_t f, int16_t r) {
  if (size >= sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) buf;
    // ESP_LOGD(TAG, "ConnectedPath::receiveRadioPacket cmd %02X from %06X with seq %d data %d", header->subprotocol, f,
    // header->seqno, header->dataLength);
    if (mRecvDups.checkDuplicateTable(f, header->sourceHandle, header->seqno)) {
      // ESP_LOGE(TAG, "ConnectedPath duplicated packet received from %06X:%02X with seq %d", f, header->nodeHandle,
      // header->seqno);
      return;
    }

    if (header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
      openConnection(buf, size, f);
    } else if (header->subprotocol == CONNPATH_OPEN_CONNECTION_NACK) {
      openConnectionNack(f, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_OPEN_CONNECTION_ACK) {
      openConnectionAck(f, header->sourceHandle, buf, size);
    } else if (header->subprotocol == CONNPATH_SEND_DATA) {
      sendData(buf, size, f);
    } else if (header->subprotocol == CONNPATH_INVALID_HANDLE) {
      invalidHandle(f, header->sourceHandle);
    } else if (header->subprotocol == CONNPATH_SEND_DATA_ERROR) {
      sendDataError(f, header->sourceHandle);
    }
  } else {
    ESP_LOGE(TAG, "ConnectedPath::recv invalid size %d but required at least %d", size, sizeof(ConnectedPathHeaderSt));
  }
}

void ConnectedPath::setReceiveCallback(ConnectedPathReceiveHandler recvCb, ConnectedPathDisconnectHandler discCb,
                                       void *arg, uint32_t from, uint16_t handle) {
  ConnectedPathConnections *conn = findConnection(from, handle);
  if (conn != nullptr) {
    conn->receive = recvCb;
    conn->disconnect = discCb;
    conn->arg = arg;
  } else {
    ESP_LOGE(TAG, "ConnectedPath::setReceiveCallback %06lX:%04X handle not found", from, handle);
  }
}

void ConnectedPath::bindPort(ConnectedPathNewConnectionHandler h, void *arg, uint16_t port) {
  ESP_LOGD(TAG, "ConnectedPath::bind port %d", port);
  ConnectedPathBindedPort_t newclient = {h, arg, port};
  mBindedPorts.push_back(newclient);
}

bool ConnectedPath::isConnectionActive(uint32_t from, uint16_t handle) const {
  return findConnection(from, handle) != nullptr;
}

void ConnectedPath::radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt) {
  ((ConnectedPath *) arg)->radioPacketSent(status, pkt);
}

void ConnectedPath::radioPacketSent(uint8_t status, RadioPacket *pkt) {
  if (status) {
    // Handle transmission error onyl with packets with clean data
    ConnectedPathPacket *oldpkt = (ConnectedPathPacket *) pkt;
    ConnectedPathHeader_t *header = oldpkt->getHeader();
    if (header != nullptr) {
      if ((header->flags & CONNPATH_FLAG_RETRANSMIT_MASK) < CONNPATH_MAX_RETRANSMISSIONS) {
        mRetransmitPacket = new ConnectedPathPacket(radioPacketSentCb, this);
        mRetransmitPacket->fromRawData(pkt->clearData(), pkt->clearDataSize());
        mRetransmitPacket->getHeader()->flags++;
        mRetransmitPacket->setTarget(oldpkt->getTarget(), oldpkt->getHandle());
        return;
      } else {
        radioPacketError(pkt->target8211(), header->sourceHandle, header->subprotocol);
        // FIXME: Signal error to packet creator
      }
    }
  }
  // Free Radio for next packet
  mIsRadioBusy = false;
}

void ConnectedPath::radioPacketError(uint32_t address, uint16_t handle, uint8_t subprot) {
  uint8_t connid;
  bool forward;
  uint32_t otherAddress;
  uint16_t otherHandle;
  connid = findConnection(address, handle, forward, otherAddress, otherHandle);
  if (CONN_IS_VALID(connid)) {
    if (subprot == CONNPATH_OPEN_CONNECTION_REQ) {
      sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, otherAddress, otherHandle, forward);
    } else if (subprot == CONNPATH_SEND_DATA) {
      sendSimplePacket(CONNPATH_SEND_DATA_ERROR, otherAddress, otherHandle, forward);
    }
    connectionSetInvalid(connid);
  } else {
    ESP_LOGE(TAG, "ConnectedPath::radioPacketError %06lX:%04X handle not found", address, handle);
  }
}

void ConnectedPath::openConnection(uint8_t *buffer, uint16_t size, uint32_t source) {
  if (size >= sizeof(ConnectedPathHeaderSt) + 3) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) buffer;
    uint8_t *buf = buffer + sizeof(ConnectedPathHeaderSt);
    uint16_t port = uint16FromBuffer(buf);
    uint8_t pathLen = buf[2];

    ESP_LOGD(TAG, "ConnectedPath::openConnection from %06lX:%04X port %d pathLen %d", source, header->sourceHandle,
             port, pathLen);
    if (size >= sizeof(ConnectedPathHeaderSt) + pathLen * sizeof(uint32_t) + 3) {
      uint8_t connid = connectionGetFirstInvalid();

      if (CONN_IS_VALID(connid)) {
        ConnectedPathConnections *conn = mConnectsions + connid;
        uint32_t nextAddress = pathLen > 0 ? uint32FromBuffer(buf + 3) : 0;
        conn->sourceAddr = source;
        conn->sourceHandle = header->sourceHandle;
        conn->lastTime = millis();

        if (pathLen > 0) {
          uint8_t newPathLen = pathLen - 1;
          conn->destAddr = nextAddress;
          conn->destHandle = mNextHandle++;

          // Forrward OPEN_CONNECTION request
          ESP_LOGD(TAG, "ConnectedPath::openConnection req %06lX:%04X[%02X]", conn->destAddr, conn->destHandle, connid);
          ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
          pkt->allocClearData(newPathLen * sizeof(uint32_t) + 3);
          pkt->getHeader()->subprotocol = CONNPATH_OPEN_CONNECTION_REQ;
          uint8_t *payload = pkt->getPayload();
          uint16toBuffer(payload, port);
          payload[2] = newPathLen;
          if (newPathLen)
            os_memcpy(payload + 3, buf + 7, newPathLen * sizeof(uint32_t));
          pkt->setTarget(conn->destAddr, conn->destHandle);
          sendRadioPacket(pkt, true, true);
        } else {
          openConnectionForMe(conn, port);
        }
      } else {
        ESP_LOGE(TAG, "ConnectedPath::openConnectionNack not enough connections for %06lX:%04X", source,
                 header->sourceHandle);
      }
    }
  }
}

void ConnectedPath::openConnectionForMe(ConnectedPathConnections *conn, uint16_t port) {
  conn->destAddr = 0;
  conn->destHandle = 0;

  // Forrward connectnio request
  ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
  pkt->allocClearData(0);
  pkt->getHeader()->subprotocol = CONNPATH_OPEN_CONNECTION_ACK;
  pkt->setTarget(conn->sourceAddr, conn->sourceHandle);
  sendRadioPacket(pkt, false, true);

  for (ConnectedPathBindedPort_t bp : mBindedPorts) {
    if (bp.port == port)
      bp.handler(bp.arg, conn->destAddr, conn->destHandle);
  }
}

void ConnectedPath::openConnectionNack(uint32_t from, uint16_t handle) {
  bool forward;
  uint32_t otherAddress;
  uint16_t otherHandle;
  int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
  ESP_LOGD(TAG, "ConnectedPath::openConnectionNack from %06lX:%04X connid %d forward %d addr:%06lX", from, handle,
           connid, forward, otherAddress);
  if (CONN_IS_VALID(connid)) {
    sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, otherAddress, otherHandle, forward);
    connectionSetInvalid(connid);
  } else {
    ESP_LOGE(TAG, "ConnectedPath::openConnectionNack invalid handle %06lX:%04X", from, handle);
  }
}

void ConnectedPath::openConnectionAck(uint32_t from, uint16_t handle, uint8_t *buffer, uint16_t size) {
  bool forward;
  uint32_t otherAddress;
  uint16_t otherHandle;
  int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
  ESP_LOGD(TAG, "ConnectedPath::openConnectionAck from %06lX:%04X[%02X] size %d", from, handle, connid, size);
  if (CONN_IS_VALID(connid)) {
    mConnectsions[connid].lastTime = millis();
    sendSimplePacket(CONNPATH_OPEN_CONNECTION_ACK, otherAddress, otherHandle, forward);
  } else {
    ESP_LOGE(TAG, "ConnectedPath::openConnectionAck invalid handle %06lX:%04X", from, handle);
    sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, from, handle, forward);
  }
}

void ConnectedPath::disconnect(uint8_t *buffer, uint16_t size, uint32_t from) {
  if (size > sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) buffer;
    ESP_LOGD(TAG, "ConnectedPath::disconnect flags %d size %d", header->flags, size);
    bool forward;
    uint32_t otherAddress;
    uint16_t otherHandle;
    int8_t connid = findConnection(from, header->sourceHandle, forward, otherAddress, otherHandle);
    if (CONN_IS_VALID(connid)) {
      ConnectedPathConnections *conn = mConnectsions + connid;
      sendSimplePacket(CONNPATH_DISCONNECT_REQ, otherAddress, otherHandle, forward);
      if (forward && mConnectsions[connid].destAddr == 0)
        conn->disconnect(conn->arg);
      connectionSetInvalid(connid);
    }
  }
}

/**
 * @brief Send data using the connected path protocol. The source of packet can be either UART or radio.
 * @param buffer - The packet data to be processed. This must contains the ConnectedPathHeaderSt structure.
 * @param size - The size of the packet data. This must be greater than sizeof(ConnectedPathHeaderSt).
 * @param from - The source address of the packet. 0 if the data is sent from the UART.
 */
void ConnectedPath::sendData(uint8_t *buffer, uint16_t size, uint32_t source) {
  if (size > sizeof(ConnectedPathHeaderSt)) {
    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *) buffer;
    // ESP_LOGD(TAG, "ConnectedPath::sendData flags %d size %d", header->flags, size);
    bool forward;
    uint32_t otherAddress;
    uint16_t otherHandle;
    int8_t connid = findConnection(source, header->sourceHandle, forward, otherAddress, otherHandle);
    if (CONN_IS_VALID(connid)) {
      mConnectsions[connid].lastTime = millis();
      // From source to target
      ConnectedPathConnections *conn = mConnectsions + connid;
      ESP_LOGD(TAG, "ConnectedPath::sendData target %06X:%04X dir %s", otherAddress, otherHandle, FORWARD2TXT(forward));
      if (otherAddress == 0) {
        if (forward) {
          if (conn->receive != nullptr)
            conn->receive(conn->arg, buffer + sizeof(ConnectedPathHeaderSt), header->dataLength, connid);
        } else
          sendUartPacket(CONNPATH_SEND_DATA, otherHandle, buffer + sizeof(ConnectedPathHeaderSt), header->dataLength);
      } else {
        ESP_LOGD(TAG, "ConnectedPath::sendData to %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
        ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
        pkt->fromRawData(buffer, size);
        pkt->setTarget(otherAddress, otherHandle);
        sendRadioPacket(pkt, forward, true);
      }
    } else {
      ESP_LOGE(TAG, "ConnectedPath::sendData invalid handle from %06lX:%04X", source, header->sourceHandle);
      sendSimplePacket(CONNPATH_INVALID_HANDLE, source, header->sourceHandle, true);
    }
  }
}

uint8_t ConnectedPath::sendData2(uint8_t *buffer, uint16_t size, uint32_t source, uint16_t handle) {
  // ESP_LOGD(TAG, "ConnectedPath::sendData2 flags %d size %d", header->flags, size);
  bool forward;
  int8_t connidx = findConnectionIndex(source, handle, &forward);

  if (CONN_IS_VALID(connidx)) {
    mConnectsions[connidx].lastTime = millis();

    uint32_t destAddress;
    uint16_t destHandle;
    findConnectionPeer(connidx, forward, destAddress, destHandle);
    ConnectedPathConnections *conn = mConnectsions + connidx;

    ESP_LOGD(TAG, "ConnectedPath::sendData2 target %06X:%04X dir %s", destAddress, destHandle, FORWARD2TXT(forward));

    if (destAddress == CONNPATH_COORDINATOR_ADDRESS) {
      if (forward) {
        if (conn->receive != nullptr)
          conn->receive(conn->arg, buffer, size, connidx);
      } else
        sendUartPacket(CONNPATH_SEND_DATA, destHandle, buffer, size);
    } else {
      // ESP_LOGD(TAG, "ConnectedPath::sendData to %02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
      ConnectedPathPacket *pkt = cratePacket(CONNPATH_SEND_DATA, size, destAddress, destHandle, buffer);
      sendRadioDataTo(buffer, size, connidx, forward);
    }
  } else {
    ESP_LOGE(TAG, "ConnectedPath::sendData2 request invalid handle from %06lX:%04X", source, handle);
    return RES_INVALID_HANDLE;
  }
  return RES_OK;
}

/**
 * @brief If an invalid handle is received from a radio packet, invalidate the connection if it exists and propagate the
 * invalid handle to the next node in the path.
 * @param source - The address of the source of the packet.
 * @param sourceHandle - The handle of the source of the packet.
 */
void ConnectedPath::invalidHandle(uint32_t source, uint16_t sourceHandle) {
  bool forward;
  uint32_t destination;
  uint16_t destinationHandle;
  int8_t connid = findConnection(source, sourceHandle, forward, destination, destinationHandle);
  ESP_LOGD(TAG, "ConnectedPath::invalidHandle connid %d source %06lX:%04X destination %06lX:%04X %s", connid, source,
           sourceHandle, destination, destinationHandle, forward ? "-->" : "<--");
  if (CONN_IS_VALID(connid)) {
    sendSimplePacket(CONNPATH_INVALID_HANDLE, destination, destinationHandle, forward);
    connectionSetInvalid(connid);
  }
}

void ConnectedPath::sendDataError(uint32_t from, uint16_t handle) {
  bool forward;
  uint32_t otherAddress;
  uint16_t otherHandle;
  int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
  ESP_LOGD(TAG, "ConnectedPath::sendDataError from %06lX:%04X connid %d. %d %06lX", from, handle, connid, forward,
           otherAddress);
  if (CONN_IS_VALID(connid)) {
    sendSimplePacket(CONNPATH_SEND_DATA_ERROR, otherAddress, otherHandle, forward);
    connectionSetInvalid(connid);
  }
}

void ConnectedPath::processOutputBuffer() {
  if (!mRadioOutputBuffer.filledSpace())
    return;

  bool processing = true;
  bool lastForward = false;
  uint8_t lastConnId = CONNPATH_MAX_CONNECTIONS;
  uint32_t lastPktTime = 0;
  uint32_t now = millis();

  uint8_t buffer[128];
  uint16_t bufferSize = 0;

  while (processing && mRadioOutputBuffer.filledSpace() > sizeof(ConnectedPathOutputBufferHeader)) {
    processing = false;
    ConnectedPathOutputBufferHeader header;
    mRadioOutputBuffer.viewData((uint8_t *) &header, sizeof(ConnectedPathOutputBufferHeader));

    if (MeshmeshComponent::elapsedMillis(now, header.pkttime) > 20 &&
        (!(CONN_IS_VALID(lastConnId)) || ((header.connId == lastConnId) && (header.forward == lastForward)))) {
      mRadioOutputBuffer.popData((uint8_t *) &header, sizeof(header));
      mRadioOutputBuffer.popData(buffer + bufferSize, header.dataSize);
      bufferSize += header.dataSize;
      lastConnId = header.connId;
      lastForward = header.forward;
      lastPktTime = header.pkttime;
      processing = true;
    }
  }

  if (bufferSize > 0 && CONN_IS_VALID(lastConnId)) {
    ConnectedPathConnections *conn = mConnectsions + lastConnId;
    ConnectedPathPacket *pkt =
        cratePacket(CONNPATH_SEND_DATA, bufferSize, lastForward ? conn->destAddr : conn->sourceAddr,
                    lastForward ? conn->destHandle : conn->sourceHandle, buffer);
    ESP_LOGD(TAG, "ConnectedPath::processOutputBuffer processed size %d after %dms", bufferSize,
             MeshmeshComponent::elapsedMillis(now, lastPktTime));
    sendRadioPacket(pkt, false, true);
  }
}

const ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) const {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if ((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) ||
        (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
      return mConnectsions + i;
    }
  }
  return nullptr;
}

ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if ((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) ||
        (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
      return mConnectsions + i;
    }
  }
  return nullptr;
}

/**
 * @brief Find an active connection by source node address and  the source handle.
 * @param source - The address of the source of the packet.
 * @param sourceHandle - The handle of the source of the packet.
 * @param forward - If is true the packet is travelling from source to destination. If false the packet is
 * travelling from destination to source.
 * @param destinationAddress - The destination address of the connection.
 * @param destinationHandle - The destination handle of the connection.
 * @return The index of the connection in the mConnectsions array or CONNPATH_MAX_CONNECTIONS if not found.
 */
uint8_t ConnectedPath::findConnection(uint32_t source, uint16_t sourceHandle, bool &forward,
                                      uint32_t &destinationAddress, uint16_t &destinationHandle) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (source == mConnectsions[i].sourceAddr && sourceHandle == mConnectsions[i].sourceHandle) {
      forward = true;
      destinationAddress = mConnectsions[i].destAddr;
      destinationHandle = mConnectsions[i].destHandle;
      return i;
    }
    if (source == mConnectsions[i].destAddr && sourceHandle == mConnectsions[i].destHandle) {
      forward = false;
      destinationAddress = mConnectsions[i].sourceAddr;
      destinationHandle = mConnectsions[i].sourceHandle;
      return i;
    }
  }
  return CONNPATH_MAX_CONNECTIONS;
}

/**
 * @brief Find an active connection by node address and  the connection handle.
 * @param from - The address of the source of the packet.
 * @param handle - The handle of the source of the packet.
 * @param forward - Returned value. If is true the packet is travelling from source to destination. If false the packet
 * is travelling from destination to source.
 * @return The index of the connection in the mConnectsions array or CONNPATH_MAX_CONNECTIONS if not found.
 */
uint8_t ConnectedPath::findConnectionIndex(uint32_t from, uint16_t handle, bool *forward) {
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) {
      if (forward)
        *forward = true;
      return i;
    }
    if (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle) {
      if (forward)
        *forward = false;
      return i;
    }
  }
  return CONNPATH_MAX_CONNECTIONS;
}

uint8_t ConnectedPath::findConnectionPeer(uint8_t connIdx, bool forward, uint32_t &peerAddress, uint16_t &peerHandle) {
  if (connIdx >= CONNPATH_MAX_CONNECTIONS) {
    return 1;
  }

  if (forward) {
    peerAddress = mConnectsions[connIdx].destAddr;
    peerHandle = mConnectsions[connIdx].destHandle;
  } else {
    peerAddress = mConnectsions[connIdx].sourceAddr;
    peerHandle = mConnectsions[connIdx].sourceHandle;
  }

  return 0;
}

void ConnectedPath::sendUartPacket(uint8_t command, uint16_t handle, uint8_t *data, uint16_t size) {
  if (size == 0) {
    uint8_t _data[4];
    _data[0] = CMD_CONNPATH_REPLY;
    _data[1] = command;
    uint16toBuffer(_data + 2, handle);
    mMeshMesh->uartSendData(_data, 4);
  } else {
    uint8_t *_data = new uint8_t[size + 4];
    _data[0] = CMD_CONNPATH_REPLY;
    _data[1] = command;
    uint16toBuffer(_data + 2, handle);
    memcpy(_data + 4, data, size);
    mMeshMesh->uartSendData(_data, size + 4);
    delete[] _data;
  }
}

ConnectedPathPacket *ConnectedPath::cratePacket(uint8_t subprot, uint16_t size, uint32_t target, uint16_t handle,
                                                const uint8_t *payload) {
  ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
  pkt->allocClearData(size);
  pkt->getHeader()->subprotocol = subprot;
  pkt->setTarget(target, handle);
  if (payload)
    pkt->setPayload(payload);
  return pkt;
}

void ConnectedPath::sendSimplePacket(uint8_t subprot, uint32_t destination, uint16_t destinationHandle, bool forward) {
  if (destination == CONNPATH_COORDINATOR_ADDRESS) {
    // Only send to UART if the packet is travelling from destination to source and the destination is 0
    // (coordinator), otherwise do nothing
    if (forward == false)
      sendUartPacket(subprot, destinationHandle, nullptr, 0);
  } else {
    sendRadioPacket(cratePacket(subprot, 0, destination, destinationHandle, nullptr), true, true);
  }
}

void ConnectedPath::debugConnection() const {
  uint32_t now = millis();
  for (int i = 0; i < CONNPATH_MAX_CONNECTIONS; i++) {
    if (mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
      const ConnectedPathConnections &c = mConnectsions[i];
      uint32_t t = MeshmeshComponent::elapsedMillis(now, c.lastTime);
      ESP_LOGD(TAG, "connections: %02X %06lX:%04X -> %06lX:%04X (%ld)", i, c.sourceAddr, c.sourceHandle, c.destAddr,
               c.destHandle, t);
    }
  }
}

}  // namespace meshmesh
}  // namespace esphome

#endif
