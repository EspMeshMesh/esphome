#include "broadcast.h"

namespace esphome {
namespace meshmesh {

void BroadCastPacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast_header_st));
}

uint8_t ICACHE_FLASH_ATTR Broadcast::send(const uint8_t *data, uint16_t size) {
	BroadCastPacket *pkt = new BroadCastPacket(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST;
	pkt->broadcastHeader()->lenght = size;
	os_memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, packetbuf->nodeIdPtr());
	uint8_t res = packetbuf->send(pkt);
	if(res == PKT_SEND_ERR) delete pkt;
    return res;
}

void ICACHE_FLASH_ATTR Broadcast::recv(uint8_t *p, uint16_t size, uint8_t *f) {
	broadcast_header_t *brdchead = (broadcast_header_t *)p;
	if (rx_func) rx_func(p+sizeof(broadcast_header_t), brdchead->lenght, f);
}

void ICACHE_FLASH_ATTR Broadcast::setRecv_cb(breadcast_recv_cb_fn rx_fn) {
	rx_func = rx_fn;
}

void ICACHE_FLASH_ATTR Broadcast::open() {
	seqno = 0;
}

}  // namespace meshmesh
}  // namespace esphome
