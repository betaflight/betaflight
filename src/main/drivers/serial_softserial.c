/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)

#include "build_config.h"

#include "common/utils.h"
#include "common/atomic.h"

#include "nvic.h"
#include "system.h"
#include "gpio.h"
#include "timer.h"

#include "serial.h"
#include "serial_softserial.h"

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

#if defined(USE_SOFTSERIAL1) && defined(USE_SOFTSERIAL2)
#define MAX_SOFTSERIAL_PORTS 2
#else
#define MAX_SOFTSERIAL_PORTS 1
#endif

typedef struct softSerial_s {
    serialPort_t     port;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[SOFTSERIAL_BUFFER_SIZE];

    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[SOFTSERIAL_BUFFER_SIZE];

    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxEdge;

    uint8_t          isTransmittingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;

    uint8_t          softSerialPortIndex;

    timerCCHandlerRec_t timerCb;
    timerCCHandlerRec_t edgeCb;
} softSerial_t;

extern timerHardware_t* serialTimerHardware;
extern softSerial_t softSerialPorts[];

extern const struct serialPortVTable softSerialVTable[];


softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

void onSerialTimer(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture);

void setTxSignal(softSerial_t *softSerial, uint8_t state)
{
    if (softSerial->port.options & SERIAL_INVERTED) {
        state = !state;
    }

    if (state) {
        digitalHi(softSerial->txTimerHardware->gpio, softSerial->txTimerHardware->pin);
    } else {
        digitalLo(softSerial->txTimerHardware->gpio, softSerial->txTimerHardware->pin);
    }
}

static void softSerialGPIOConfig(GPIO_TypeDef *gpio, uint16_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void serialInputPortConfig(const timerHardware_t *timerHardwarePtr)
{
#ifdef STM32F10X
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU);
#else
#ifdef STM32F303xC
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_AF_PP_PU);
#endif
#endif
}

static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

static void serialTimerTxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference, uint32_t baud)
{
    uint32_t clock = SystemCoreClock;
    uint32_t timerPeriod;
    do {
        timerPeriod = clock / baud;
        if (isTimerPeriodTooLarge(timerPeriod)) {
            if (clock > 1) {
                clock = clock / 2;   // this is wrong - mhz stays the same ... This will double baudrate until ok (but minimum baudrate is < 1200)
            } else {
                // TODO unable to continue, unable to determine clock and timerPeriods for the given baud
            }

        }
    } while (isTimerPeriodTooLarge(timerPeriod));

    uint8_t mhz = SystemCoreClock / 1000000;
    timerConfigure(timerHardwarePtr, timerPeriod, mhz);
    timerChCCHandlerInit(&softSerialPorts[reference].timerCb, onSerialTimer);
    timerChConfigCallbacks(timerHardwarePtr, &softSerialPorts[reference].timerCb, NULL);
}

static void serialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

static void serialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference, portOptions_t options)
{
    // start bit is usually a FALLING signal
    serialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, (options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    timerChCCHandlerInit(&softSerialPorts[reference].edgeCb, onSerialRxPinChange);
    timerChConfigCallbacks(timerHardwarePtr, &softSerialPorts[reference].edgeCb, NULL);
}

static void serialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_Out_PP);
}

static void resetBuffers(softSerial_t *softSerial)
{
    softSerial->port.rxBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;
}

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, uint32_t baud, portOptions_t options)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

#ifdef USE_SOFTSERIAL1
    if (portIndex == SOFTSERIAL1) {
        softSerial->rxTimerHardware = &(timerHardware[SOFTSERIAL_1_TIMER_RX_HARDWARE]);
        softSerial->txTimerHardware = &(timerHardware[SOFTSERIAL_1_TIMER_TX_HARDWARE]);
    }
#endif

#ifdef USE_SOFTSERIAL2
    if (portIndex == SOFTSERIAL2) {
        softSerial->rxTimerHardware = &(timerHardware[SOFTSERIAL_2_TIMER_RX_HARDWARE]);
        softSerial->txTimerHardware = &(timerHardware[SOFTSERIAL_2_TIMER_TX_HARDWARE]);
    }
#endif

    softSerial->port.vTable = softSerialVTable;
    softSerial->port.baudRate = baud;
    softSerial->port.mode = MODE_RXTX;
    softSerial->port.options = options;
    softSerial->port.callback = callback;

    resetBuffers(softSerial);

    softSerial->isTransmittingData = false;

    softSerial->isSearchingForStartBit = true;
    softSerial->rxBitIndex = 0;

    softSerial->transmissionErrors = 0;
    softSerial->receiveErrors = 0;

    softSerial->softSerialPortIndex = portIndex;

    serialOutputPortConfig(softSerial->txTimerHardware);
    serialInputPortConfig(softSerial->rxTimerHardware);

    setTxSignal(softSerial, ENABLE);
    delay(50);

    serialTimerTxConfig(softSerial->txTimerHardware, portIndex, baud);
    serialTimerRxConfig(softSerial->rxTimerHardware, portIndex, options);

    return &softSerial->port;
}

/*********************************************/

void processTxState(softSerial_t *softSerial)
{
    uint8_t mask;

    if (!softSerial->isTransmittingData) {
        char byteToSend;
        if (isSoftSerialTransmitBufferEmpty((serialPort_t *)softSerial)) {
            return;
        }

        // data to send
        byteToSend = softSerial->port.txBuffer[softSerial->port.txBufferTail++];
        if (softSerial->port.txBufferTail >= softSerial->port.txBufferSize) {
            softSerial->port.txBufferTail = 0;
        }

        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        softSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        softSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        softSerial->isTransmittingData = true;

        return;
    }

    if (softSerial->bitsLeftToTransmit) {
        mask = softSerial->internalTxBuffer & 1;
        softSerial->internalTxBuffer >>= 1;

        setTxSignal(softSerial, mask);
        softSerial->bitsLeftToTransmit--;
        return;
    }

    softSerial->isTransmittingData = false;
}



enum {
    TRAILING,
    LEADING
};

void applyChangedBits(softSerial_t *softSerial)
{
    if (softSerial->rxEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = softSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < softSerial->rxBitIndex; bitToSet++) {
            softSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

void prepareForNextRxByte(softSerial_t *softSerial)
{
    // prepare for next byte
    softSerial->rxBitIndex = 0;
    softSerial->isSearchingForStartBit = true;
    if (softSerial->rxEdge == LEADING) {
        softSerial->rxEdge = TRAILING;
        serialICConfig(
            softSerial->rxTimerHardware->tim,
            softSerial->rxTimerHardware->channel,
            (softSerial->port.options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling
        );
    }
}

#define STOP_BIT_MASK (1 << 0)
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))

void extractAndStoreRxByte(softSerial_t *softSerial)
{
    if ((softSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t haveStartBit = (softSerial->internalRxBuffer & START_BIT_MASK) == 0;
    uint8_t haveStopBit = (softSerial->internalRxBuffer & STOP_BIT_MASK) == 1;

    if (!haveStartBit || !haveStopBit) {
        softSerial->receiveErrors++;
        return;
    }

    uint8_t rxByte = (softSerial->internalRxBuffer >> 1) & 0xFF;

    if (softSerial->port.callback) {
        softSerial->port.callback(rxByte);
    } else {
        softSerial->port.rxBuffer[softSerial->port.rxBufferHead] = rxByte;
        softSerial->port.rxBufferHead = (softSerial->port.rxBufferHead + 1) % softSerial->port.rxBufferSize;
    }
}

void processRxState(softSerial_t *softSerial)
{
    if (softSerial->isSearchingForStartBit) {
        return;
    }

    softSerial->rxBitIndex++;

    if (softSerial->rxBitIndex == RX_TOTAL_BITS - 1) {
        applyChangedBits(softSerial);
        return;
    }

    if (softSerial->rxBitIndex == RX_TOTAL_BITS) {

        if (softSerial->rxEdge == TRAILING) {
            softSerial->internalRxBuffer |= STOP_BIT_MASK;
        }

        extractAndStoreRxByte(softSerial);
        prepareForNextRxByte(softSerial);
    }
}

void onSerialTimer(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    softSerial_t *softSerial = container_of(cbRec, softSerial_t, timerCb);

    processTxState(softSerial);
    processRxState(softSerial);
}

void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);

    softSerial_t *softSerial = container_of(cbRec, softSerial_t, edgeCb);
    bool inverted = softSerial->port.options & SERIAL_INVERTED;

    if ((softSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    if (softSerial->isSearchingForStartBit) {
        // synchronise bit counter
        // FIXME this reduces functionality somewhat as receiving breaks concurrent transmission on all ports because
        // the next callback to the onSerialTimer will happen too early causing transmission errors.
        TIM_SetCounter(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->tim->ARR / 2);
        if (softSerial->isTransmittingData) {
            softSerial->transmissionErrors++;
        }

        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
        softSerial->rxEdge = LEADING;

        softSerial->rxBitIndex = 0;
        softSerial->rxLastLeadingEdgeAtBitIndex = 0;
        softSerial->internalRxBuffer = 0;
        softSerial->isSearchingForStartBit = false;
        return;
    }

    if (softSerial->rxEdge == LEADING) {
        softSerial->rxLastLeadingEdgeAtBitIndex = softSerial->rxBitIndex;
    }

    applyChangedBits(softSerial);

    if (softSerial->rxEdge == TRAILING) {
        softSerial->rxEdge = LEADING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
    } else {
        softSerial->rxEdge = TRAILING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    }
}

uint8_t softSerialRxBytesWaiting(serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

uint8_t softSerialTxBytesFree(serialPort_t *instance)
{
    if ((instance->mode & MODE_TX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    uint8_t bytesUsed = (s->port.txBufferHead - s->port.txBufferTail) & (s->port.txBufferSize - 1);

    return (s->port.txBufferSize - 1) - bytesUsed;
}

uint8_t softSerialReadByte(serialPort_t *instance)
{
    uint8_t ch;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    if (softSerialRxBytesWaiting(instance) == 0) {
        return 0;
    }

    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

void softSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    softSerial_t *softSerial = (softSerial_t *)s;
    openSoftSerial(softSerial->softSerialPortIndex, s->callback, baudRate, softSerial->port.options);
}

void softSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->mode = mode;
}

bool isSoftSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

const struct serialPortVTable softSerialVTable[] = {
    {
        softSerialWriteByte,
        softSerialRxBytesWaiting,
        softSerialTxBytesFree,
        softSerialReadByte,
        softSerialSetBaudRate,
        isSoftSerialTransmitBufferEmpty,
        softSerialSetMode,
        .writeBuf = NULL,
    }
};

#endif
