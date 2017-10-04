/*
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */

#pragma once

#define SMARTPORT_MSP_TX_BUF_SIZE 256
#define SMARTPORT_MSP_RX_BUF_SIZE 64

void initSmartPortTelemetry(void);

void handleSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);

void configureSmartPortTelemetryPort(void);
void freeSmartPortTelemetryPort(void);

