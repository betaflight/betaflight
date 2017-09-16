
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct rbuf_s {
    uint8_t *start;  
    uint8_t *end;
    uint8_t *readIdx;
    uint8_t *writeIdx;
} rbuf_t;

void rbufWriteU8(rbuf_t *dst, uint8_t val);
void rbufWriteU16(rbuf_t *dst, uint16_t val);
void rbufWriteU32(rbuf_t *dst, uint32_t val);
void rbufWriteU16BigEndian(rbuf_t *dst, uint16_t val);
void rbufWriteU32BigEndian(rbuf_t *dst, uint32_t val);
void rbufWriteData(rbuf_t *dst, const void *data, int len);
void rbufWriteString(rbuf_t *dst, const char *string);
void rbufWriteStringWithZeroTerminator(rbuf_t *dst, const char *string);
uint8_t rbufReadU8(rbuf_t *src);
uint16_t rbufReadU16(rbuf_t *src);
uint32_t rbufReadU32(rbuf_t *src);
void rbufReadData(rbuf_t *src, void *data, int len);
int rbufReadBytesRemaining(rbuf_t *buf);
int rbufWriteBytesRemaining(rbuf_t *buf);
uint8_t* rbufPtr(rbuf_t *buf);
const uint8_t* rbufConstPtr(const rbuf_t *buf);
rbuf_t *rbufInit(rbuf_t *rbuf, uint8_t *start, uint8_t *end);