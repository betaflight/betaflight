
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ringbuf.h"

void rbufWriteU8(rbuf_t *dst, uint8_t val)
{
    if (dst->writeIdx == dst->end) {
        dst->writeIdx = dst->start;
    }
    *dst->writeIdx++ = val;
}

void rbufWriteU16(rbuf_t *dst, uint16_t val)
{
    rbufWriteU8(dst, val >> 0);
    rbufWriteU8(dst, val >> 8);
}

void rbufWriteU32(rbuf_t *dst, uint32_t val)
{
    rbufWriteU8(dst, val >> 0);
    rbufWriteU8(dst, val >> 8);
    rbufWriteU8(dst, val >> 16);
    rbufWriteU8(dst, val >> 24);
}

void rbufWriteU16BigEndian(rbuf_t *dst, uint16_t val)
{
    rbufWriteU8(dst, val >> 8);
    rbufWriteU8(dst, (uint8_t)val);
}

void rbufWriteU32BigEndian(rbuf_t *dst, uint32_t val)
{
    rbufWriteU8(dst, val >> 24);
    rbufWriteU8(dst, val >> 16);
    rbufWriteU8(dst, val >> 8);
    rbufWriteU8(dst, (uint8_t)val);
}

void rbufWriteData(rbuf_t *dst, const void *data, int len)
{
    uint8_t distToEnd = dst->end - dst->writeIdx;
    if (len <= distToEnd) {
        memcpy(dst->writeIdx, data, len);
        dst->writeIdx += len;
    } else {
        memcpy(dst->writeIdx, data, distToEnd); // write to the end
        memcpy(dst->start, (uint8_t *)data + distToEnd, len - distToEnd);  // write from the start
        dst->writeIdx = dst->start + (len - distToEnd);
    }
}

void rbufWriteString(rbuf_t *dst, const char *string)
{
    rbufWriteData(dst, string, strlen(string));
}

void rbufWriteStringWithZeroTerminator(rbuf_t *dst, const char *string)
{
    rbufWriteData(dst, string, strlen(string) + 1);
}

uint8_t rbufReadU8(rbuf_t *src)
{
    if(src->readIdx == src->end) {
        src->readIdx = src->start;
    }
    return *src->readIdx++;
}

uint16_t rbufReadU16(rbuf_t *src)
{
    uint16_t ret;
    ret = rbufReadU8(src);
    ret |= rbufReadU8(src) << 8;
    return ret;
}

uint32_t rbufReadU32(rbuf_t *src)
{
    uint32_t ret;
    ret = rbufReadU8(src);
    ret |= rbufReadU8(src) <<  8;
    ret |= rbufReadU8(src) << 16;
    ret |= rbufReadU8(src) << 24;
    return ret;
}

void rbufReadData(rbuf_t *src, void *data, int len)
{
    uint8_t distToEnd = src->end - src->readIdx;
    if (len <= distToEnd) {
        memcpy(data, src->readIdx, len);
        src->readIdx += len;
    } else {
        memcpy(data, src->readIdx, distToEnd);
        memcpy((uint8_t *)data + distToEnd, src->start, len - distToEnd);
        src->readIdx = src->start + (len - distToEnd);
    }
}

int rbufReadBytesRemaining(rbuf_t *buf)
{
    if (buf->writeIdx >= buf->readIdx) {
        return buf->writeIdx - buf->readIdx;
    } else {
        return (buf->end - buf->start) - (buf->readIdx - buf->writeIdx);
    }
}

int rbufWriteBytesRemaining(rbuf_t *buf)
{
    if (buf->writeIdx < buf->readIdx) {
        return buf->writeIdx - buf->readIdx;
    } else {
        return (buf->end - buf->start) - (buf->readIdx - buf->writeIdx);
    }
}

uint8_t *rbufReadPtr(rbuf_t *buf)
{
    return buf->readIdx;
}

const uint8_t *rbufConstPtr(const rbuf_t *buf)
{
    return buf->readIdx;
}

rbuf_t *rbufInit(rbuf_t *rbuf, uint8_t *start, uint8_t *end)
{
    rbuf->start = start;
    rbuf->end = end;
    rbuf->readIdx = start;
    rbuf->writeIdx = start;
    return rbuf;
}

