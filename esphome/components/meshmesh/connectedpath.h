#pragma once
#include "esphome/core/defines.h"
#ifdef USE_CONNECTED_PROTOCOL

#include "packetbuf.h"
#include "recvdups.h"
#include "memringbuffer.h"
#include "esphome/core/component.h"

#define CONNPATH_FLAG_REVERSEDIR 0x80
#define CONNPATH_FLAG_RETRANSMIT_MASK 0x0F
#define CONNPATH_INVALID_ADDRESS 0xFFFFFFFF
#define CONNPATH_MAX_CONNECTIONS 0x10

namespace esphome {
namespace meshmesh {

typedef std::function<void(void *arg, uint8_t *data, uint16_t size, uint8_t connid)> ConnectedPathReceiveHandler;
typedef std::function<void(void *arg)> ConnectedPathDisconnectHandler;
typedef std::function<void(void *arg, uint32_t from, uint16_t handle)> ConnectedPathNewConnectionHandler;

struct ConnectedPathHeaderSt {
  uint8_t protocol;       // ConnectedPath protocol identifier
  uint8_t subprotocol;    // Subprotocol identifier (CONNPATH_SEND_DATA, CONNPATH_INVALID_HANDLE, etc.)
  uint16_t sourceHandle;  // Handle for  this connection on the source
  uint16_t flags;         // Flags (CONNPATH_FLAG_REVERSEDIR, CONNPATH_FLAG_RETRANSMIT_MASK, etc.)
  uint16_t seqno;         // Sequence number of the packet this must be incremented for each packet sent
  uint16_t dataLength;    // Payload length
} __attribute__((packed));
typedef struct ConnectedPathHeaderSt ConnectedPathHeader_t;

struct ConnectedPathBindedPortSt {
  ConnectedPathNewConnectionHandler handler;
  void *arg;
  uint16_t port;
};
typedef ConnectedPathBindedPortSt ConnectedPathBindedPort_t;

struct ConnectedPathConnections {
  uint16_t sourceHandle;
  uint16_t destHandle;
  uint32_t sourceAddr;
  uint32_t destAddr;
  uint32_t lastTime;
  ConnectedPathReceiveHandler receive;
  ConnectedPathDisconnectHandler disconnect;
  void *arg;
};

struct ConnectedPathOutputBufferHeader {
  uint32_t pkttime;
  uint16_t forward : 1;
  uint8_t connId : 7;
  uint16_t dataSize;
};

constexpr uint32_t CONNPATH_COORDINATOR_ADDRESS = 0x00000000;

class ConnectedPathPacket : public RadioPacket {
 public:
  explicit ConnectedPathPacket(pktbufSentCbFn cb, void *arg) : RadioPacket(cb, arg) {}

 public:
  virtual void allocClearData(uint16_t size);

 public:
  const ConnectedPathHeader_t *getHeader() const { return (const ConnectedPathHeader_t *) clearData(); }
  ConnectedPathHeader_t *getHeader() { return (ConnectedPathHeader_t *) clearData(); }
  void setPayload(const uint8_t *payoad);
  uint8_t *getPayload() { return clearData() + sizeof(ConnectedPathHeaderSt); }
  void setTarget(uint32_t target, uint16_t handle);
  uint32_t getTarget() const { return mTarget; }
  uint16_t getHandle() const { return getHeader()->sourceHandle; }

 private:
  uint32_t mTarget = 0;
};

class MeshmeshComponent;
class ConnectedPath {
 public:
  ConnectedPath(MeshmeshComponent *meshmesh, PacketBuf *packetbuf)
      : mMeshMesh(meshmesh), mPacketBuf(packetbuf), mRecvDups(), mRadioOutputBuffer(256) {
    mPacketBuf->setConnectedPath(this);
  }
  void setup(void);
  void loop();
  uint8_t sendRawRadioPacket(ConnectedPathPacket *pkt);
  uint8_t sendRadioPacket(ConnectedPathPacket *pkt, bool forward, bool initHeader);
  void sendDataTo(const uint8_t *data, uint16_t size, uint8_t connid);
  void sendDataTo(const uint8_t *data, uint16_t size, uint32_t from, uint16_t handle);
  void closeConnection_(ConnectedPathConnections *conn);
  void closeConnection(uint32_t from, uint16_t handle);
  void closeAllConnections();
  uint8_t receiveUartPacket(uint8_t *data, uint16_t size);
  void receiveRadioPacket(uint8_t *p, uint16_t size, uint32_t f, int16_t r);
  void setReceiveCallback(ConnectedPathReceiveHandler recvCb, ConnectedPathDisconnectHandler discCb, void *arg,
                          uint32_t from, uint16_t handle);
  void bindPort(ConnectedPathNewConnectionHandler h, void *arg, uint16_t port);
  bool isConnectionActive(uint32_t from, uint16_t handle) const;

 private:
  static void radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt);
  void radioPacketSent(uint8_t status, RadioPacket *pkt);
  void radioPacketError(uint32_t address, uint16_t handle, uint8_t subprot);
  void openConnection(uint8_t *buffer, uint16_t size, uint32_t from);
  void openConnectionForMe(ConnectedPathConnections *conn, uint16_t port);
  void openConnectionNack(uint32_t from, uint16_t handle);
  void openConnectionAck(uint32_t from, uint16_t handle, uint8_t *buffer, uint16_t size);
  void disconnect(uint8_t *buffer, uint16_t size, uint32_t from);
  void sendData(uint8_t *buffer, uint16_t size, uint32_t from);
  void invalidHandle(uint32_t from, uint16_t handle);
  void sendDataError(uint32_t from, uint16_t handle);
  void processOutputBuffer();

 private:
  void connectionSetInvalid(uint8_t index) {
    if (index < CONNPATH_MAX_CONNECTIONS)
      connectionSetInvalid(mConnectsions + index);
  }
  void connectionSetInvalid(ConnectedPathConnections *conn) {
    conn->sourceAddr = CONNPATH_INVALID_ADDRESS;
    conn->destAddr = CONNPATH_INVALID_ADDRESS;
    conn->receive = nullptr;
    conn->disconnect = nullptr;
    conn->arg = nullptr;
  }
  uint8_t connectionGetFirstInvalid() {
    uint8_t i;
    for (i = 0; i < CONNPATH_MAX_CONNECTIONS; i++)
      if (mConnectsions[i].sourceAddr == CONNPATH_INVALID_ADDRESS)
        break;
    return i;
  }

 private:
  const ConnectedPathConnections *findConnection(uint32_t from, uint16_t handle) const;
  ConnectedPathConnections *findConnection(uint32_t from, uint16_t handle);
  uint8_t findConnection(uint32_t from, uint16_t handle, bool &forward, uint32_t &otherAddress, uint16_t &otherHandle);
  void sendUartPacket(uint8_t command, uint16_t handle, uint8_t *data, uint16_t size);
  ConnectedPathPacket *cratePacket(uint8_t subprot, uint16_t size, uint32_t to, uint16_t handle);
  void sendSimplePacket(uint8_t subprot, uint32_t to, uint16_t handle, bool forward);
  void debugConnection() const;

 private:
  MeshmeshComponent *mMeshMesh;
  PacketBuf *mPacketBuf;
  uint16_t mLastSequenceNum = 0;
  RecvDups mRecvDups;
  ConnectedPathConnections mConnectsions[CONNPATH_MAX_CONNECTIONS];
  uint32_t mConnectionsCheckTime = 0;
  std::list<ConnectedPathBindedPort_t> mBindedPorts;
  uint16_t mNextHandle = 1;

 private:
  bool mIsRadioBusy{false};
  ConnectedPathPacket *mRetransmitPacket = nullptr;
  std::list<ConnectedPathPacket *> mPendingPackets;
  MemRingBuffer mRadioOutputBuffer;
};

}  // namespace meshmesh
}  // namespace esphome

#endif
