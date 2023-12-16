
#include "connectedpath.h"
#ifdef USE_CONNECTED_PROTOCOL
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include "meshmesh.h"
#include "commands.h"

namespace esphome {
namespace meshmesh {

static const char *TAG = "meshmesh.ConnectedPath";


#define CONNPATH_MAX_RETRANSMISSIONS 0x04

#define CONNPATH_OPEN_CONNECTION_REQ  0x01
#define CONNPATH_OPEN_CONNECTION_REP  0x02
#define CONNPATH_DISCONNECT_ACK       0x03
#define CONNPATH_INVALID_HANDLE       0x04
#define CONNPATH_SEND_DATA            0x05
#define CONNPATH_OPEN_CONNECTION_ACK  0x06
#define CONNPATH_OPEN_CONNECTION_NACK 0x07
#define CONNPATH_DISCONNECT_REQ       0x08
#define CONNPATH_SEND_DATA_ERROR      0x09
#define CONNPATH_CLEAR_CONNECTIONS    0x0A


#define CONN_IS_VALID(X) X<CONNPATH_MAX_CONNECTIONS


void ConnectedPathPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(ConnectedPathHeaderSt));
	getHeader()->dataLength = size;
}

void ConnectedPathPacket::setPayload(const uint8_t *payoad) {
	os_memcpy(clearData()+sizeof(ConnectedPathHeaderSt), payoad, getHeader()->dataLength);
}

void ConnectedPathPacket::setTarget(uint32_t target, uint16_t handle) {
	mTarget = target;
	if(clearData() != nullptr) getHeader()->nodeHandle = handle;
}

void ConnectedPath::setup(void) {
	os_memset((uint8_t *)mConnectsions, 0x0, sizeof(mConnectsions));
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) connectionSetInvalid(i);
	mConnectionsCheckTime = millis();
}

void ConnectedPath::loop() {
	mRecvDups.loop();
    uint32_t now=millis();
    if(MeshmeshComponent::elapsedMillis(now, mConnectionsCheckTime) > 120000) {
        mConnectionsCheckTime = now;
		// debugConnection();
        for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) {
            if(mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS && MeshmeshComponent::elapsedMillis(now, mConnectsions[i].lastTime) > 300000) {
				closeConnection_(mConnectsions+i);
                connectionSetInvalid(i);
            }
        }
    }
}

uint8_t ConnectedPath::send(ConnectedPathPacket *pkt, bool forward, bool initHeader) {
	ConnectedPathHeader_t *header = pkt->getHeader();
	// Fill protocol header...
	header->protocol = PROTOCOL_CONNPATH;
	// Optional fields
	if(initHeader) {
		// Add flags to this packet
		header->flags = forward ? 0x00 : CONNPATH_FLAG_REVERSEDIR;
		// If is an ACK i use the last seqno
		header->seqno = ++mLastSequenceNum;
	}
	// Set this class as destination sent callback for retransmisisons
	pkt->setCallback(radioPacketSentCb, this);

    if(pkt->encryptClearData()) {
		uint32_t target = pkt->getTarget();
		pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
		uint8_t res = mPacketBuf->send(pkt);
		if(res == PKT_SEND_ERR) delete pkt;
	    return res;
	} else {
		return PKT_SEND_ERR;
	}
}

void ConnectedPath::sendDataTo(const uint8_t *data, uint16_t size, uint8_t connid) {
	if(connid >= CONNPATH_MAX_CONNECTIONS || mConnectsions[connid].sourceAddr == CONNPATH_INVALID_ADDRESS) {
		return;
	}

	ConnectedPathConnections *conn = mConnectsions + connid;
	ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->getHeader()->subprotocol = CONNPATH_SEND_DATA;
	pkt->setTarget(conn->sourceAddr, conn->sourceHandle);
	pkt->setPayload(data);
	send(pkt, false, true);
}

void ConnectedPath::sendDataTo(const uint8_t *data, uint16_t size, uint32_t from, uint16_t handle) {
	ConnectedPathConnections *conn = findConnection(from, handle);
	if(conn != nullptr) {
		ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
		pkt->allocClearData(size);
		pkt->getHeader()->subprotocol = CONNPATH_SEND_DATA;
		pkt->setTarget(conn->sourceAddr, conn->sourceHandle);
		pkt->setPayload(data);
		send(pkt, false, true);
	}
}

void ConnectedPath::closeConnection_(ConnectedPathConnections *conn) {
	sendSimplePacket(CONNPATH_DISCONNECT_REQ, conn->sourceAddr, conn->sourceHandle, false);
	sendSimplePacket(CONNPATH_DISCONNECT_REQ, conn->destAddr, conn->destHandle, true);
	connectionSetInvalid(conn);
}

void ConnectedPath::closeConnection(uint32_t from, uint16_t handle) {
	ConnectedPathConnections *conn = findConnection(from, handle);
	if(conn != nullptr) closeConnection_(conn);
}

void ConnectedPath::closeAllConnections() {
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) {
		if(mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
			closeConnection_(mConnectsions+i);
		}
	}
}

uint8_t ConnectedPath::receiveUartPacket(uint8_t *data, uint16_t size) {
	if(size >= sizeof(ConnectedPathHeaderSt)) {
		ConnectedPathHeader_t *header = (ConnectedPathHeader_t *)data;
		//ESP_LOGD(TAG, "ConnectedPath::receiveUartPacket size %d subp %d", size, header->subprotocol);
		if(header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
			openConnection(data, size, 0);
		} else if(header->subprotocol == CONNPATH_SEND_DATA) {
			sendData(data, size, 0);
		} else if(header->subprotocol == CONNPATH_DISCONNECT_REQ) {
			disconnect(data, size, 0);
		} else if(header->subprotocol == CONNPATH_INVALID_HANDLE) {
			invalidHandle(0, header->nodeHandle);
		} else if(header->subprotocol == CONNPATH_CLEAR_CONNECTIONS) {
			closeAllConnections();
			mRecvDups.clear();
			ESP_LOGI(TAG, "All connection and duplicates tables has been cleared");
		} else {
			ESP_LOGE(TAG, "ConnectedPath::receiveUartPacket unknow sub protocol %d", header->subprotocol);
		}
	}
	return HANDLE_UART_OK;
}

void ConnectedPath::receiveRadioPacket(uint8_t  *buf, uint16_t  size, uint32_t f, int16_t r) {
	if(size >= sizeof(ConnectedPathHeaderSt)) {
	    ConnectedPathHeader_t *header = (ConnectedPathHeader_t *)buf;
		// ESP_LOGD(TAG, "ConnectedPath::receiveRadioPacket cmd %02X from %06X with seq %d data %d", header->subprotocol, f, header->seqno, header->dataLength);
		if(mRecvDups.checkDuplicateTable(f, header->nodeHandle, header->seqno)) {
			// ESP_LOGE(TAG, "ConnectedPath duplicated packet received from %06X:%02X with seq %d", f, header->nodeHandle, header->seqno);
			return;
		}


		if(header->subprotocol == CONNPATH_OPEN_CONNECTION_REQ) {
			openConnection(buf, size, f);
		} else if(header->subprotocol == CONNPATH_OPEN_CONNECTION_NACK) {
			openConnectionNack(f, header->nodeHandle);
		} else if(header->subprotocol == CONNPATH_OPEN_CONNECTION_ACK) {
			openConnectionAck(f, header->nodeHandle, buf, size);
		} else if(header->subprotocol == CONNPATH_SEND_DATA) {
			sendData(buf, size, f);
		} else if(header->subprotocol == CONNPATH_INVALID_HANDLE) {
			invalidHandle(f, header->nodeHandle);
		} else if(header->subprotocol == CONNPATH_SEND_DATA_ERROR) {
			sendDataError(f, header->nodeHandle);
		}
	} else {
        ESP_LOGE(TAG, "ConnectedPath::recv invalid size %d but required at least %d", size, sizeof(ConnectedPathHeaderSt));
	}
}

void ConnectedPath::setReceiveCallback(ConnectedPathReceiveHandler recvCb, ConnectedPathDisconnectHandler discCb, void *arg, uint32_t from, uint16_t handle) {
	ConnectedPathConnections *conn = findConnection(from, handle);
	if(conn != nullptr) {
		conn->receive = recvCb;
		conn->disconnect = discCb;
		conn->arg = arg;
	} else {
		ESP_LOGE(TAG, "ConnectedPath::setReceiveCallback %06X:%04X handle not found", from, handle);
	}
}

void ConnectedPath::bindPort(ConnectedPathNewConnectionHandler h, void *arg, uint16_t port) {
    ESP_LOGD(TAG, "ConnectedPath::bindPort port %d", port);
    ConnectedPathBindedPort_t newclient = { h, arg, port };
    mBindedPorts.push_back(newclient);
}

bool ConnectedPath::isConnectionActive(uint32_t from, uint16_t handle) const {
	return findConnection(from, handle) != nullptr;
}

void ConnectedPath::radioPacketSentCb(void *arg, uint8_t  status, RadioPacket *pkt) {
    ((ConnectedPath *)arg)->radioPacketSent(status, pkt);
}

void ConnectedPath::radioPacketSent(uint8_t  status, RadioPacket *pkt) {
    if(status) {
        // Handle transmission error onyl with packets with clean data
        ConnectedPathPacket *oldpkt = (ConnectedPathPacket *)pkt;
		ConnectedPathHeader_t *header = oldpkt->getHeader();
        if(header != nullptr) {
			if((header->flags & CONNPATH_FLAG_RETRANSMIT_MASK) < CONNPATH_MAX_RETRANSMISSIONS) {
				ConnectedPathPacket *newpkt = new ConnectedPathPacket(radioPacketSentCb, this);
				newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
				newpkt->getHeader()->flags++;
				newpkt->setTarget(oldpkt->getTarget(), oldpkt->getHandle());
				send(newpkt, false, false);
			} else {
				ESP_LOGE(TAG, "ConnectedPath::radioPacketSent transmission error for %06X:%04X after %d try", pkt->target8211(), header->nodeHandle, header->flags & 0xF);
				radioPacketError(pkt->target8211(), header->nodeHandle, header->subprotocol);
				// FIXME: Signal error to packet creator
			}
        }
    }
}

void ConnectedPath::radioPacketError(uint32_t address, uint16_t handle, uint8_t subprot) {
	uint8_t connid; bool forward; uint32_t otherAddress; uint16_t otherHandle;
	connid = findConnection(address, handle, forward, otherAddress, otherHandle);
	if(CONN_IS_VALID(connid)) {
		if(subprot == CONNPATH_OPEN_CONNECTION_REQ) {
			sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, otherAddress, otherHandle, forward);
		} else if(subprot == CONNPATH_SEND_DATA) {
			sendSimplePacket(CONNPATH_SEND_DATA_ERROR, otherAddress, otherHandle, forward);
		}
		connectionSetInvalid(connid);
	} else {
		ESP_LOGE(TAG, "ConnectedPath::radioPacketError %06X:%04X handle not found", address, handle);
	}
}

void ConnectedPath::openConnection(uint8_t *buffer, uint16_t size, uint32_t from) {
	if(size >= sizeof(ConnectedPathHeaderSt)+3) {
		ConnectedPathHeader_t *header = (ConnectedPathHeader_t *)buffer;
		uint8_t *buf = buffer + sizeof(ConnectedPathHeaderSt);
		uint16_t port = uint16FromBuffer(buf);
		uint8_t pathLen = buf[2];

		ESP_LOGD(TAG, "ConnectedPath::openConnection from %06X:%04X port %d pathLen %d", from, header->nodeHandle, port, pathLen);
		if(size >= sizeof(ConnectedPathHeaderSt)+pathLen*sizeof(uint32_t)+3) {
			uint8_t connid = connectionGetFirstInvalid();

			if(CONN_IS_VALID(connid)) {
				ConnectedPathConnections *conn = mConnectsions+connid;
				uint32_t nextAddress = pathLen>0 ? uint32FromBuffer(buf+3) : 0;
				conn->sourceAddr = from;
				conn->sourceHandle = header->nodeHandle;
				conn->lastTime = millis();

				if(pathLen > 0) {
					uint8_t newPathLen = pathLen-1;
					conn->destAddr = nextAddress;
					conn->destHandle = mNextHandle++;

					// Forrward OPEN_CONNECTION request
					ESP_LOGD(TAG, "ConnectedPath::openConnection req %06X:%04X[%02X]", conn->destAddr, conn->destHandle, connid);
					ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
					pkt->allocClearData(newPathLen*sizeof(uint32_t)+3);
					pkt->getHeader()->subprotocol = CONNPATH_OPEN_CONNECTION_REQ;
					uint8_t *payload = pkt->getPayload();
					uint16toBuffer(payload, port);
					payload[2] = newPathLen;
					if(newPathLen) os_memcpy(payload+3, buf+7, newPathLen*sizeof(uint32_t));
					pkt->setTarget(conn->destAddr, conn->destHandle);
					send(pkt, true, true);
				} else {
					openConnectionForMe(conn, port);
				}
			} else {
				ESP_LOGE(TAG, "ConnectedPath::openConnectionNack not enough connections for %06X:%04X", from, header->nodeHandle);
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
	send(pkt, false, true);

	for(ConnectedPathBindedPort_t bp : mBindedPorts) {
		if(bp.port == port) bp.handler(bp.arg, conn->sourceAddr, conn->sourceHandle);
	}
}

void ConnectedPath::openConnectionNack(uint32_t from, uint16_t handle) {
	bool forward; uint32_t otherAddress; uint16_t otherHandle;
	int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
	ESP_LOGD(TAG, "ConnectedPath::openConnectionNack from %06X:%04X connid %d forward %d addr:%06X", from, handle, connid, forward, otherAddress);
	if(CONN_IS_VALID(connid)) {
		sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, otherAddress, otherHandle, forward);
		connectionSetInvalid(connid);
	} else {
		ESP_LOGE(TAG, "ConnectedPath::openConnectionNack invalid handle %06X:%04X", from, handle);
	}
}

void ConnectedPath::openConnectionAck(uint32_t from, uint16_t handle, uint8_t *buffer, uint16_t size) {
	bool forward; uint32_t otherAddress; uint16_t otherHandle;
	int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
	ESP_LOGD(TAG, "ConnectedPath::openConnectionAck from %06X:%04X[%02X] size %d", from, handle, connid, size);
	if(CONN_IS_VALID(connid)) {
		mConnectsions[connid].lastTime = millis();
		sendSimplePacket(CONNPATH_OPEN_CONNECTION_ACK, otherAddress, otherHandle, forward);
	} else {
		ESP_LOGE(TAG, "ConnectedPath::openConnectionAck invalid handle %06X:%04X", from, handle);
		sendSimplePacket(CONNPATH_OPEN_CONNECTION_NACK, from, handle, forward);
	}
}

void ConnectedPath::disconnect(uint8_t *buffer, uint16_t size, uint32_t from) {
	if(size > sizeof(ConnectedPathHeaderSt)) {
		ConnectedPathHeader_t *header = (ConnectedPathHeader_t *)buffer;
		ESP_LOGD(TAG, "ConnectedPath::disconnect flags %d size %d", header->flags, size);
		bool forward; uint32_t otherAddress; uint16_t otherHandle;
		int8_t connid = findConnection(from, header->nodeHandle, forward, otherAddress, otherHandle);
		if(CONN_IS_VALID(connid)) {
			ConnectedPathConnections *conn = mConnectsions + connid;
			sendSimplePacket(CONNPATH_DISCONNECT_REQ, otherAddress, otherHandle, forward);
			if(forward && mConnectsions[connid].destAddr == 0) conn->disconnect(conn->arg);
			connectionSetInvalid(connid);
		}
	}
}

void ConnectedPath::sendData(uint8_t *buffer, uint16_t size, uint32_t from) {
	if(size > sizeof(ConnectedPathHeaderSt)) {
		ConnectedPathHeader_t *header = (ConnectedPathHeader_t *)buffer;
		//ESP_LOGD(TAG, "ConnectedPath::sendData flags %d size %d", header->flags, size);
		bool forward; uint32_t otherAddress; uint16_t otherHandle;
		int8_t connid = findConnection(from, header->nodeHandle, forward, otherAddress, otherHandle);
		if(CONN_IS_VALID(connid)) {
			mConnectsions[connid].lastTime = millis();
			// From source to target
			ConnectedPathConnections *conn = mConnectsions+connid;
			// ESP_LOGD(TAG, "ConnectedPath::sendData target %06X:%04X dir %s", otherAddress, otherHandle, forward ? "-->" : "<--");
			if(otherAddress == 0) {
				if(forward) {
					if(conn->receive != nullptr)
						conn->receive(conn->arg, buffer+sizeof(ConnectedPathHeaderSt), header->dataLength, connid);
				} else sendUartPacket(CONNPATH_SEND_DATA, otherHandle, buffer+sizeof(ConnectedPathHeaderSt), header->dataLength);
			} else {
				ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
				pkt->fromRawData(buffer, size);
				pkt->setTarget(otherAddress, otherHandle);
				send(pkt, forward, true);
			}
		} else {
			ESP_LOGE(TAG, "ConnectedPath::sendData invalid handle from %06X:%04X", from, header->nodeHandle);
			sendSimplePacket(CONNPATH_INVALID_HANDLE, from, header->nodeHandle, true);
		}
	}
}

void ConnectedPath::invalidHandle(uint32_t from, uint16_t handle) {
	bool forward; uint32_t otherAddress; uint16_t otherHandle;
	int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
	ESP_LOGD(TAG, "ConnectedPath::invalidHandle from %06X:%04X connid %d. %06X:%04X %s", from, handle, connid, otherAddress, otherHandle, forward ? "-->" : "<--");
	if(CONN_IS_VALID(connid)) {
		sendSimplePacket(CONNPATH_INVALID_HANDLE, otherAddress, otherHandle, forward);
		connectionSetInvalid(connid);
	}
}

void ConnectedPath::sendDataError(uint32_t from, uint16_t handle) {
	bool forward; uint32_t otherAddress; uint16_t otherHandle;
	int8_t connid = findConnection(from, handle, forward, otherAddress, otherHandle);
	ESP_LOGD(TAG, "ConnectedPath::sendDataError from %06X:%04X connid %d. %d %06X", from, handle, connid, forward, otherAddress);
	if(CONN_IS_VALID(connid)) {
		sendSimplePacket(CONNPATH_SEND_DATA_ERROR, otherAddress, otherHandle, forward);
		connectionSetInvalid(connid);
	}
}

const ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) const {
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) {
		if((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) || (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
			return mConnectsions+i;
		}
	}
	return nullptr;
}

ConnectedPathConnections *ConnectedPath::findConnection(uint32_t from, uint16_t handle) {
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) {
		if((from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) || (from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle)) {
			return mConnectsions+i;
		}
	}
	return nullptr;
}

uint8_t ConnectedPath::findConnection(uint32_t from, uint16_t handle, bool &forward, uint32_t &otherAddress, uint16_t &otherHandle) {
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS; i++) {
		if(from == mConnectsions[i].sourceAddr && handle == mConnectsions[i].sourceHandle) {
			forward = true;
			otherAddress = mConnectsions[i].destAddr;
			otherHandle = mConnectsions[i].destHandle;
			return i;
		}
		if(from == mConnectsions[i].destAddr && handle == mConnectsions[i].destHandle) {
			forward = false;
			otherAddress = mConnectsions[i].sourceAddr;
			otherHandle = mConnectsions[i].sourceHandle;
			return i;
		}
	}
	return CONNPATH_MAX_CONNECTIONS;
}

void ConnectedPath::sendUartPacket(uint8_t command, uint16_t handle, uint8_t *data, uint16_t size) {
    if(size == 0) {
        uint8_t _data[4];
        _data[0] = CMD_CONNPATH_REPLY;
        _data[1] = command;
        uint16toBuffer(_data+2, handle);
        mMeshMesh->uartSendData(_data, 4);
    } else {
        uint8_t *_data = new uint8_t[size+4];
        _data[0] = CMD_CONNPATH_REPLY;
        _data[1] = command;
        uint16toBuffer(_data+2, handle);
        memcpy(_data+4, data, size);
        mMeshMesh->uartSendData(_data, size+4);
        delete _data;
    }
}

ConnectedPathPacket *ConnectedPath::cratePacket(uint8_t subprot, uint16_t size, uint32_t to, uint16_t handle) {
	ConnectedPathPacket *pkt = new ConnectedPathPacket(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->getHeader()->subprotocol = subprot;
	pkt->setTarget(to, handle);
	return pkt;
}

void ConnectedPath::sendSimplePacket(uint8_t subprot, uint32_t to, uint16_t handle, bool forward) {
	if(to == 0) {
		if(forward == false) sendUartPacket(subprot, handle, nullptr, 0);
	} else {
		send(cratePacket(subprot, 0, to, handle), true, true);
	}
}


void ConnectedPath::debugConnection() const {
	uint32_t now = millis();
	for(int i=0; i<CONNPATH_MAX_CONNECTIONS;i++) {
		if(mConnectsions[i].sourceAddr != CONNPATH_INVALID_ADDRESS) {
			const ConnectedPathConnections &c = mConnectsions[i];
			uint32_t t = MeshmeshComponent::elapsedMillis(now, c.lastTime);
			ESP_LOGD(TAG, "connections: %02X %06X:%04X -> %06X:%04X (%d)", i, c.sourceAddr, c.sourceHandle, c.destAddr, c.destHandle, t);
		}
	}
}


}
}

#endif