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

typedef struct smartPortPayload_s {
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
} __attribute__((packed)) smartPortPayload_t;

typedef void smartPortWriteFrameFn(const smartPortPayload_t *payload);

bool initSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);
bool initSmartPortTelemetryExternal(smartPortWriteFrameFn *smartPortWriteFrameExternal);

void handleSmartPortTelemetry(void);
void processSmartPortTelemetry(smartPortPayload_t *payload, volatile bool *hasRequest, const uint32_t *requestTimeout);

struct serialPort_s;
void smartPortWriteFrameSerial(const smartPortPayload_t *payload, struct serialPort_s *port, uint16_t checksum);
void smartPortSendByte(uint8_t c, uint16_t *checksum, struct serialPort_s *port);
