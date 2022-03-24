#include "memringbuffer.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP8266
#include <HardwareSerial.h>
#endif

#include <memory.h>

namespace esphome {
namespace meshmesh {

static const char *TAG = "meshmesh.memringbuffer";

MemRingBuffer::MemRingBuffer(uint16_t size) {
    if(size > 0) resize(size);
}

MemRingBuffer::~MemRingBuffer() {
    if(mBuffer) deallocateSpace();
}

void MemRingBuffer::pushData(uint8_t *data, uint16_t size) {
    //esphome::ESP_LOGD(TAG, "MemRingBuffer::pushData size %d alloc %d free %d", size, allocatedSpace(), freeSpace());
    if(size <= freeSpace()) {
        uint16_t contfreespace = continousFreeSpace();
        if(contfreespace > size) contfreespace = size;
        pushDataUnsafe(data, contfreespace);

        size -= contfreespace;
        if(size > 0) {
            mTail = mBuffer;
            pushDataUnsafe(data+contfreespace, size);
        }
    }
}

void MemRingBuffer::pushByte(uint8_t byte) {
    if(mSize < mAllocatedSize) {
        *mTail++ = byte;
        if(mTail == mBuffer + mAllocatedSize) mTail = mBuffer;
        mSize++;
    }
}

uint16_t MemRingBuffer::popData(uint8_t *data, uint16_t size) {
    //esphome::ESP_LOGD(TAG, "MemRingBuffer::popData size %d alloc %d fill %d", size, allocatedSpace(), filledSpace());
    if(filledSpace() < size || size == 0) size = filledSpace();
    uint16_t readed = size;

    int16_t ss = (mTail-mBuffer)-(mHead-mBuffer);
    if(ss<0) ss = mAllocatedSize+ss;
    //Serial1.printf("**%d %d %d==%d\n", mHead-mBuffer, mTail-mBuffer, ss, mSize);

    uint16_t contfillspace = continousFilledSpace();
    if(contfillspace > size) contfillspace = size;
    popDataUnsafe(data, contfillspace);

    //Serial1.printf("*%d %d %d\n",size, contfillspace, size-contfillspace);
    size -= contfillspace;
    if(size > 0) {
        mHead = mBuffer;
        popDataUnsafe(data+contfillspace, size);
    }

    return readed;
}

void MemRingBuffer::resize(uint16_t size) {
    ESP_LOGD(TAG, "MemRingBuffer::resize curr %d new %d", mSize, size);
    if(mSize == size) return;
    if(mSize) deallocateSpace();
    if(size) allocateSpace(size);
}

void MemRingBuffer::deallocateSpace() {
    ESP_LOGD(TAG, "MemRingBuffer::deallocateSpace free %d bytes", mSize);
    delete mBuffer;
    mBuffer = nullptr;
    mTail = mHead = nullptr;
    mSize = mAllocatedSize = 0;
}

void MemRingBuffer::allocateSpace(uint16_t size) {
    mBuffer = new uint8_t[size];
    memset(mBuffer, 0x00, size);
    mTail = mHead = mBuffer;
    mAllocatedSize = size;
    mSize = 0;
}


void MemRingBuffer::pushDataUnsafe(uint8_t *data, uint16_t size) {
    memcpy(mTail, data, size);
    mTail += size;
    mSize += size;
}

void MemRingBuffer::popDataUnsafe(uint8_t *data, uint16_t size) {
    memcpy(data, mHead, size);
    mHead += size;
    mSize -= size;
}

}
}