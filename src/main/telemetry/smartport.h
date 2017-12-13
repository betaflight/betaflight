/*
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SMARTPORT_MSP_TX_BUF_SIZE 256
#define SMARTPORT_MSP_RX_BUF_SIZE 64

enum
{
    FSSP_START_STOP = 0x7E,

    FSSP_DLE        = 0x7D,
    FSSP_DLE_XOR    = 0x20,

    FSSP_DATA_FRAME = 0x10,
    FSSP_MSPC_FRAME_SMARTPORT = 0x30, // MSP client frame
    FSSP_MSPC_FRAME_FPORT = 0x31, // MSP client frame
    FSSP_MSPS_FRAME = 0x32, // MSP server frame

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67
    // there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
};

typedef struct smartPortPayload_s {
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
} __attribute__((packed)) smartPortPayload_t;

typedef void smartPortWriteFrameFn(const smartPortPayload_t *payload);
typedef bool smartPortCheckQueueEmptyFn(void);

bool initSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);
bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal);

void handleSmartPortTelemetry(void);
void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *hasRequest, const uint32_t *requestTimeout);

smartPortPayload_t *smartPortDataReceive(uint16_t c, bool *clearToSend, smartPortCheckQueueEmptyFn *checkQueueEmpty, bool withChecksum);

struct serialPort_s;
void smartPortWriteFrameSerial(const smartPortPayload_t *payload, struct serialPort_s *port, uint16_t checksum);
void smartPortSendByte(uint8_t c, uint16_t *checksum, struct serialPort_s *port);
