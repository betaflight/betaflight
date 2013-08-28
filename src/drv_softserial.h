/*
 * drv_softserial.h
 *
 *  Created on: 23 Aug 2013
 *      Author: Hydra
 */

#pragma once

#define SOFT_SERIAL_BUFFER_SIZE 256

typedef struct softSerial_s {
    const timerHardware_t *timerHardware;
    uint8_t timerIndex;
    serialPort_t port;
    volatile int state;
    volatile uint8_t rxBuffer[SOFT_SERIAL_BUFFER_SIZE];
} softSerial_t;
void setupSoftSerial1(uint32_t baud);

uint8_t serialReadByte(softSerial_t *softSerial);
bool serialAvailable(softSerial_t *softSerial);

extern timerHardware_t* serialTimerHardware;
extern softSerial_t softSerialPorts[];
