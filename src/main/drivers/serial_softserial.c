/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Cleanflight (or Baseflight): original
 * jflyper: Mono-timer and single-wire half-duplex
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_SOFTSERIAL)

#include "build/build_config.h"
#include "build/atomic.h"

#include "build/debug.h"

#include "common/utils.h"

#include "io/serial.h"

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/timer.h"

#include "serial_softserial.h"

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

typedef enum {
    TIMER_MODE_SINGLE,
    TIMER_MODE_DUAL,
} timerMode_e;

#define ICPOLARITY_RISING true
#define ICPOLARITY_FALLING false

typedef struct softSerial_s {
    serialPort_t     port;

    IO_t rxIO;
    IO_t txIO;

    const timerHardware_t *timerHardware;
#ifdef USE_HAL_DRIVER
    const TIM_HandleTypeDef *timerHandle;
#endif
    const timerHardware_t *exTimerHardware;

    volatile uint8_t rxBuffer[SOFTSERIAL_BUFFER_SIZE];
    volatile uint8_t txBuffer[SOFTSERIAL_BUFFER_SIZE];

    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxEdge;
    uint8_t          rxActive;

    uint8_t          isTransmittingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;

    timerMode_e      timerMode;

    timerOvrHandlerRec_t overCb;
    timerCCHandlerRec_t edgeCb;
} softSerial_t;

static const struct serialPortVTable softSerialVTable; // Forward
// SERIAL_SOFTSERIAL_COUNT is fine, softserial ports must start from 1 and be continuous
static softSerial_t softSerialPorts[SERIAL_SOFTSERIAL_COUNT];

void onSerialTimerOverflow(timerOvrHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture);

typedef enum { IDLE = ENABLE, MARK = DISABLE } SerialTxState_e;
static void setTxSignal(softSerial_t *softSerial, SerialTxState_e state)
{
    IOWrite(softSerial->txIO, (softSerial->port.options & SERIAL_INVERTED) ? !state : state);
}

static void serialEnableCC(softSerial_t *softSerial)
{
#ifdef USE_HAL_DRIVER
    TIM_CCxChannelCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_ENABLE);
#else
    TIM_CCxCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_Enable);
#endif
}

// switch to receive mode
static void serialInputPortActivate(softSerial_t *softSerial)
{
    const serialPullMode_t pull = serialOptions_pull(softSerial->port.options);
    const uint8_t pinConfig = ((const uint8_t[]){IOCFG_AF_PP, IOCFG_AF_PP_PD, IOCFG_AF_PP_UP})[pull];
    // softserial can easily support opendrain mode, but it is not implemented
    IOConfigGPIOAF(softSerial->rxIO, pinConfig, softSerial->timerHardware->alternateFunction);

    softSerial->rxActive = true;
    softSerial->isSearchingForStartBit = true;
    softSerial->rxBitIndex = 0;

    // Enable input capture

    serialEnableCC(softSerial);
}

static void serialInputPortDeActivate(softSerial_t *softSerial)
{
    // Disable input capture

#ifdef USE_HAL_DRIVER
    TIM_CCxChannelCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_DISABLE);
#else
    TIM_CCxCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_Disable);
#endif
    IOConfigGPIO(softSerial->rxIO, IOCFG_IN_FLOATING); // leave AF mode; serialOutputPortActivate will follow immediately
    softSerial->rxActive = false;
}

static void serialOutputPortActivate(softSerial_t *softSerial)
{
    if (softSerial->exTimerHardware) {
        IOConfigGPIOAF(softSerial->txIO, IOCFG_OUT_PP, softSerial->exTimerHardware->alternateFunction);
    } else {
        IOConfigGPIO(softSerial->txIO, IOCFG_OUT_PP);
    }
}

static void serialOutputPortDeActivate(softSerial_t *softSerial)
{
    if (softSerial->exTimerHardware) {
        // TODO: there in no AF associated with input port
        IOConfigGPIOAF(softSerial->txIO, IOCFG_IN_FLOATING, softSerial->exTimerHardware->alternateFunction);
    } else {
        IOConfigGPIO(softSerial->txIO, IOCFG_IN_FLOATING);
    }
}

static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

static void serialTimerConfigureTimebase(const timerHardware_t *timerHardwarePtr, uint32_t baud)
{
    uint32_t baseClock = timerClock(timerHardwarePtr->tim);
    uint32_t clock = baseClock;
    uint32_t timerPeriod;

    while (timerPeriod = clock / baud, isTimerPeriodTooLarge(timerPeriod) && clock > 1) {
        clock = clock / 2;   // minimum baudrate is < 1200
    }
    timerConfigure(timerHardwarePtr, timerPeriod, clock);
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

softSerial_t* softSerialFromIdentifier(serialPortIdentifier_e identifier)
{
    if (identifier >= SERIAL_PORT_SOFTSERIAL1 && identifier < SERIAL_PORT_SOFTSERIAL1 + SERIAL_SOFTSERIAL_COUNT) {
        return &softSerialPorts[identifier - SERIAL_PORT_SOFTSERIAL1];
    }
    return NULL;
}

serialPort_t *softSerialOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baud, portMode_e mode, portOptions_e options)
{
    softSerial_t *softSerial = softSerialFromIdentifier(identifier);
    if (!softSerial) {
        return NULL;
    }
    // fill identifier early, so initialization code can use it
    softSerial->port.identifier = identifier;

    const int resourceIndex = serialResourceIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1
    const int ownerIndex = serialOwnerIndex(identifier);

    const ioTag_t tagRx = serialPinConfig()->ioTagRx[resourceIndex];
    const ioTag_t tagTx = serialPinConfig()->ioTagTx[resourceIndex];

    const timerHardware_t *timerTx = timerAllocate(tagTx, ownerTxRx, ownerIndex);
    const timerHardware_t *timerRx = (tagTx == tagRx) ? timerTx : timerAllocate(tagRx, ownerTxRx + 1, ownerIndex);

    IO_t rxIO = IOGetByTag(tagRx);
    IO_t txIO = IOGetByTag(tagTx);

    if (options & SERIAL_BIDIR) {
        // If RX and TX pins are both assigned, we CAN use either with a timer.
        // However, for consistency with hardware UARTs, we only use TX pin,
        // and this pin must have a timer, and it must not be N-Channel.
        if (!timerTx || (timerTx->output & TIMER_OUTPUT_N_CHANNEL)) {
            return NULL;
        }

        softSerial->timerHardware = timerTx;
        softSerial->txIO = txIO;
        softSerial->rxIO = txIO;
        IOInit(txIO, ownerTxRx, ownerIndex);
    } else {
        if (mode & MODE_RX) {
            // Need a pin & a timer on RX. Channel must not be N-Channel.
            if (!timerRx || (timerRx->output & TIMER_OUTPUT_N_CHANNEL)) {
                return NULL;
            }

            softSerial->rxIO = rxIO;
            softSerial->timerHardware = timerRx;
            if (!((mode & MODE_TX) && rxIO == txIO)) {
                // RX only on pin
                IOInit(rxIO, ownerTxRx + 1, ownerIndex);
            }
        }

        if (mode & MODE_TX) {
            // Need a pin on TX
            if (!txIO)
                return NULL;

            softSerial->txIO = txIO;

            if (!(mode & MODE_RX)) {
                // TX Simplex, must have a timer
                if (!timerTx) {
                    return NULL;
                }
                softSerial->timerHardware = timerTx;
            } else {
                // Duplex, use timerTx if available
                softSerial->exTimerHardware = timerTx;
            }
            IOInit(txIO, ownerTxRx, ownerIndex);
        }
    }

    softSerial->port.vTable = &softSerialVTable;
    softSerial->port.baudRate = baud;
    softSerial->port.mode = mode;
    softSerial->port.options = options;
    softSerial->port.rxCallback = rxCallback;
    softSerial->port.rxCallbackData = rxCallbackData;

    resetBuffers(softSerial);

    softSerial->transmissionErrors = 0;
    softSerial->receiveErrors = 0;

    softSerial->rxActive = false;
    softSerial->isTransmittingData = false;

    // Configure master timer (on RX); time base and input capture

    serialTimerConfigureTimebase(softSerial->timerHardware, baud);
    timerChConfigIC(softSerial->timerHardware, (options & SERIAL_INVERTED) ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);

    // Initialize callbacks
    timerChCCHandlerInit(&softSerial->edgeCb, onSerialRxPinChange);
    timerChOvrHandlerInit(&softSerial->overCb, onSerialTimerOverflow);

    // Configure bit clock interrupt & handler.
    // If we have an extra timer (on TX), it is initialized and configured
    // for overflow interrupt.
    // Receiver input capture is configured when input is activated.

    if ((mode & MODE_TX) && softSerial->exTimerHardware && softSerial->exTimerHardware->tim != softSerial->timerHardware->tim) {
        softSerial->timerMode = TIMER_MODE_DUAL;
        serialTimerConfigureTimebase(softSerial->exTimerHardware, baud);
        timerChConfigCallbacks(softSerial->exTimerHardware, NULL, &softSerial->overCb);
        timerChConfigCallbacks(softSerial->timerHardware, &softSerial->edgeCb, NULL);
    } else {
        softSerial->timerMode = TIMER_MODE_SINGLE;
        timerChConfigCallbacks(softSerial->timerHardware, &softSerial->edgeCb, &softSerial->overCb);
    }

#ifdef USE_HAL_DRIVER
    softSerial->timerHandle = timerFindTimerHandle(softSerial->timerHardware->tim);
#endif

    if (!(options & SERIAL_BIDIR)) {
        serialOutputPortActivate(softSerial);
        setTxSignal(softSerial, IDLE);
    }

    serialInputPortActivate(softSerial);

    return &softSerial->port;
}


/*
 * Serial Engine
 */

void processTxState(softSerial_t *softSerial)
{
    if (!softSerial->isTransmittingData) {
        if (isSoftSerialTransmitBufferEmpty((serialPort_t *)softSerial)) {
            // Transmit buffer empty.
            // Switch to RX mode if not already listening and running in half-duplex mode
            if (!softSerial->rxActive && softSerial->port.options & SERIAL_BIDIR) {
                serialOutputPortDeActivate(softSerial); // TODO: not necessary
                serialInputPortActivate(softSerial);
            }
            return;
        }

        // data to send
        uint8_t byteToSend = softSerial->port.txBuffer[softSerial->port.txBufferTail++];
        if (softSerial->port.txBufferTail >= softSerial->port.txBufferSize) {
            softSerial->port.txBufferTail = 0;
        }

        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        softSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1) | 0;
        softSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        softSerial->isTransmittingData = true;

        if (softSerial->rxActive && (softSerial->port.options & SERIAL_BIDIR)) {
            // Half-duplex: Deactivate receiver, activate transmitter
            serialInputPortDeActivate(softSerial);
            serialOutputPortActivate(softSerial);

            // Start sending on next bit timing, as port manipulation takes time,
            // and continuing here may cause bit period to decrease causing sampling errors
            // at the receiver under high rates.
            // Note that there will be (little less than) 1-bit delay; take it as "turn around time".
            // This time is important in noninverted pulldown bidir mode (SmartAudio).
            //   During this period, TX pin is in IDLE state so next startbit (MARK) can be detected
            // XXX Otherwise, we may be able to reload counter and continue. (Future work.)
            return;
        }
    }

    if (softSerial->bitsLeftToTransmit) {
        const bool bit = softSerial->internalTxBuffer & 1;
        softSerial->internalTxBuffer >>= 1;

        setTxSignal(softSerial, bit);
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
        for (unsigned bitToSet = softSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < softSerial->rxBitIndex; bitToSet++) {
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
        timerChConfigIC(softSerial->timerHardware, (softSerial->port.options & SERIAL_INVERTED) ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
        serialEnableCC(softSerial);
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

    if (softSerial->port.rxCallback) {
        softSerial->port.rxCallback(rxByte, softSerial->port.rxCallbackData);
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

void onSerialTimerOverflow(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    softSerial_t *self = container_of(cbRec, softSerial_t, overCb);

    if (self->port.mode & MODE_TX)
        processTxState(self);

    if (self->port.mode & MODE_RX)
        processRxState(self);
}

void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);

    softSerial_t *self = container_of(cbRec, softSerial_t, edgeCb);

    if ((self->port.mode & MODE_RX) == 0) {
        return;
    }

    const bool inverted = self->port.options & SERIAL_INVERTED;

    if (self->isSearchingForStartBit) {
        // Synchronize the bit timing so that it will interrupt at the center
        // of the bit period.

#ifdef USE_HAL_DRIVER
        __HAL_TIM_SetCounter(self->timerHandle, __HAL_TIM_GetAutoreload(self->timerHandle) / 2);
#else
        TIM_SetCounter(self->timerHardware->tim, self->timerHardware->tim->ARR / 2);
#endif

        // For a mono-timer full duplex configuration, this may clobber the
        // transmission because the next callback to the onSerialTimerOverflow
        // will happen too early causing transmission errors.
        // For a dual-timer configuration, there is no problem.

        if ((self->timerMode != TIMER_MODE_DUAL) && self->isTransmittingData) {
            self->transmissionErrors++;
        }

        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        serialEnableCC(self);
#endif
        self->rxEdge = LEADING;

        self->rxBitIndex = 0;
        self->rxLastLeadingEdgeAtBitIndex = 0;
        self->internalRxBuffer = 0;
        self->isSearchingForStartBit = false;
        return;
    }

    if (self->rxEdge == LEADING) {
        self->rxLastLeadingEdgeAtBitIndex = self->rxBitIndex;
    }

    applyChangedBits(self);

    if (self->rxEdge == TRAILING) {
        self->rxEdge = LEADING;
        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
    } else {
        self->rxEdge = TRAILING;
        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
    }
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    serialEnableCC(self);
#endif
}


/*
 * Standard serial driver API
 */

uint32_t softSerialRxBytesWaiting(const serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

uint32_t softSerialTxBytesFree(const serialPort_t *instance)
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

    softSerial->port.baudRate = baudRate;

    serialTimerConfigureTimebase(softSerial->timerHardware, baudRate);
}

void softSerialSetMode(serialPort_t *instance, portMode_e mode)
{
    instance->mode = mode;
}

bool isSoftSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

static const struct serialPortVTable softSerialVTable = {
    .serialWrite = softSerialWriteByte,
    .serialTotalRxWaiting = softSerialRxBytesWaiting,
    .serialTotalTxFree = softSerialTxBytesFree,
    .serialRead = softSerialReadByte,
    .serialSetBaudRate = softSerialSetBaudRate,
    .isSerialTransmitBufferEmpty = isSoftSerialTransmitBufferEmpty,
    .setMode = softSerialSetMode,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL
};

#endif
