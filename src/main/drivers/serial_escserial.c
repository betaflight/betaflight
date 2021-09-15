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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_ESCSERIAL)

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "flight/mixer.h"

#include "io/serial.h"

#include "pg/motor.h"


typedef enum {
    BAUDRATE_NORMAL = 19200,
    BAUDRATE_SIMONK = 28800, // = 9600 * 3
    BAUDRATE_KISS   = 38400,
    BAUDRATE_CASTLE = 18880
} escBaudRate_e;

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

#define MAX_ESCSERIAL_PORTS 1
static serialPort_t *escPort = NULL;
static serialPort_t *passPort = NULL;

#define ICPOLARITY_RISING true
#define ICPOLARITY_FALLING false

typedef struct escSerial_s {
    serialPort_t     port;

    IO_t rxIO;
    IO_t txIO;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[ESCSERIAL_BUFFER_SIZE];
    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[ESCSERIAL_BUFFER_SIZE];

#ifdef USE_HAL_DRIVER
    const TIM_HandleTypeDef *txTimerHandle;
    const TIM_HandleTypeDef *rxTimerHandle;
#endif

    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxEdge;

    uint8_t          isTransmittingData;
    uint8_t          isReceivingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint16_t         receiveTimeout;
    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;

    uint8_t          escSerialPortIndex;
    uint8_t          mode;
    uint8_t          outputCount;

    timerCCHandlerRec_t timerCb;
    timerCCHandlerRec_t edgeCb;
} escSerial_t;

typedef struct {
    IO_t io;
    uint8_t inverted;
} escOutputs_t;

escOutputs_t escOutputs[MAX_SUPPORTED_MOTORS];

extern timerHardware_t* serialTimerHardware;

const struct serialPortVTable escSerialVTable[];

escSerial_t escSerialPorts[MAX_ESCSERIAL_PORTS];

PG_REGISTER_WITH_RESET_TEMPLATE(escSerialConfig_t, escSerialConfig, PG_ESCSERIAL_CONFIG, 0);

#ifndef ESCSERIAL_TIMER_TX_PIN
#define ESCSERIAL_TIMER_TX_PIN NONE
#endif

PG_RESET_TEMPLATE(escSerialConfig_t, escSerialConfig,
    .ioTag = IO_TAG(ESCSERIAL_TIMER_TX_PIN),
);

enum {
    TRAILING,
    LEADING
};

#define STOP_BIT_MASK (1 << 0)
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))

// XXX No TIM_DeInit equivalent in HAL driver???
#ifdef USE_HAL_DRIVER
static void TIM_DeInit(TIM_TypeDef *tim)
{
    UNUSED(tim);
}
#endif

static void setTxSignalEsc(escSerial_t *escSerial, uint8_t state)
{
    if (escSerial->mode == PROTOCOL_KISSALL)
    {
        for (volatile uint8_t i = 0; i < escSerial->outputCount; i++) {
            uint8_t state_temp = state;
            if (escOutputs[i].inverted) {
                state_temp ^= ENABLE;
            }

            if (state_temp) {
                IOHi(escOutputs[i].io);
            } else {
                IOLo(escOutputs[i].io);
            }
        }
    }
    else
    {
        if (escSerial->rxTimerHardware->output & TIMER_OUTPUT_INVERTED) {
            state ^= ENABLE;
        }

        if (state) {
            IOHi(escSerial->txIO);
        } else {
            IOLo(escSerial->txIO);
        }
    }
}

static void escSerialGPIOConfig(const timerHardware_t *timhw, ioConfig_t cfg)
{
    ioTag_t tag = timhw->tag;

    if (!tag) {
        return;
    }

    IOInit(IOGetByTag(tag), OWNER_MOTOR, 0);
#ifdef STM32F7
    IOConfigGPIOAF(IOGetByTag(tag), cfg, timhw->alternateFunction);
#else
    IOConfigGPIO(IOGetByTag(tag), cfg);
#endif
}

static void escSerialInputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    escSerialGPIOConfig(timerHardwarePtr, IOCFG_AF_PP_UP);
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr,ENABLE);
}


static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

static bool isEscSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    // start listening
    return instance->txBufferHead == instance->txBufferTail;
}

static void escSerialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    escSerialGPIOConfig(timerHardwarePtr, IOCFG_OUT_PP);
    timerChITConfig(timerHardwarePtr,DISABLE);
}

static void processTxStateBL(escSerial_t *escSerial)
{
    uint8_t mask;
    if (escSerial->isReceivingData) {
        return;
    }

    if (!escSerial->isTransmittingData) {
        char byteToSend;
        if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
            // canreceive
            return;
        }

        // data to send
        byteToSend = escSerial->port.txBuffer[escSerial->port.txBufferTail++];
        if (escSerial->port.txBufferTail >= escSerial->port.txBufferSize) {
            escSerial->port.txBufferTail = 0;
        }

        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        escSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        escSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        escSerial->isTransmittingData = true;


        //set output
        if (escSerial->mode==PROTOCOL_BLHELI || escSerial->mode==PROTOCOL_CASTLE) {
            escSerialOutputPortConfig(escSerial->rxTimerHardware);
        }
        return;
    }

    if (escSerial->bitsLeftToTransmit) {
        mask = escSerial->internalTxBuffer & 1;
        escSerial->internalTxBuffer >>= 1;

        setTxSignalEsc(escSerial, mask);
        escSerial->bitsLeftToTransmit--;
        return;
    }

    escSerial->isTransmittingData = false;
    if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
        if (escSerial->mode==PROTOCOL_BLHELI || escSerial->mode==PROTOCOL_CASTLE)
        {
            escSerialInputPortConfig(escSerial->rxTimerHardware);
        }
    }
}

static void extractAndStoreRxByteBL(escSerial_t *escSerial)
{
    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t haveStartBit = (escSerial->internalRxBuffer & START_BIT_MASK) == 0;
    uint8_t haveStopBit = (escSerial->internalRxBuffer & STOP_BIT_MASK) == 1;

    if (!haveStartBit || !haveStopBit) {
        escSerial->receiveErrors++;
        return;
    }

    uint8_t rxByte = (escSerial->internalRxBuffer >> 1) & 0xFF;

    if (escSerial->port.rxCallback) {
        escSerial->port.rxCallback(rxByte, escSerial->port.rxCallbackData);
    } else {
        escSerial->port.rxBuffer[escSerial->port.rxBufferHead] = rxByte;
        escSerial->port.rxBufferHead = (escSerial->port.rxBufferHead + 1) % escSerial->port.rxBufferSize;
    }
}

static void prepareForNextRxByteBL(escSerial_t *escSerial)
{
    // prepare for next byte
    escSerial->rxBitIndex = 0;
    escSerial->isSearchingForStartBit = true;
    if (escSerial->rxEdge == LEADING) {
        escSerial->rxEdge = TRAILING;
        timerChConfigIC(
            escSerial->rxTimerHardware,
            (escSerial->port.options & SERIAL_INVERTED) ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0
        );
    }
}

static void applyChangedBitsBL(escSerial_t *escSerial)
{
    if (escSerial->rxEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = escSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < escSerial->rxBitIndex; bitToSet++) {
            escSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

static void processRxStateBL(escSerial_t *escSerial)
{
    if (escSerial->isSearchingForStartBit) {
        return;
    }

    escSerial->rxBitIndex++;

    if (escSerial->rxBitIndex == RX_TOTAL_BITS - 1) {
        applyChangedBitsBL(escSerial);
        return;
    }

    if (escSerial->rxBitIndex == RX_TOTAL_BITS) {

        if (escSerial->rxEdge == TRAILING) {
            escSerial->internalRxBuffer |= STOP_BIT_MASK;
        }

        extractAndStoreRxByteBL(escSerial);
        prepareForNextRxByteBL(escSerial);
    }
}

static void onSerialTimerBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    escSerial_t *escSerial = container_of(cbRec, escSerial_t, timerCb);

    processTxStateBL(escSerial);
    processRxStateBL(escSerial);
}

static void serialTimerTxConfigBL(const timerHardware_t *timerHardwarePtr, uint8_t reference, uint32_t baud)
{
    uint32_t clock = SystemCoreClock/2;
    uint32_t timerPeriod;
    TIM_DeInit(timerHardwarePtr->tim);
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

    timerConfigure(timerHardwarePtr, timerPeriod, clock);
    timerChCCHandlerInit(&escSerialPorts[reference].timerCb, onSerialTimerBL);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].timerCb, NULL);
}

static void onSerialRxPinChangeBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);

    escSerial_t *escSerial = container_of(cbRec, escSerial_t, edgeCb);
    bool inverted = escSerial->port.options & SERIAL_INVERTED;

    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    if (escSerial->isSearchingForStartBit) {
        // Adjust the timing so it will interrupt on the middle.
        // This is clobbers transmission, but it is okay because we are
        // always half-duplex.
#ifdef USE_HAL_DRIVER
        __HAL_TIM_SetCounter(escSerial->txTimerHandle, __HAL_TIM_GetAutoreload(escSerial->txTimerHandle) / 2);
#else
        TIM_SetCounter(escSerial->txTimerHardware->tim, escSerial->txTimerHardware->tim->ARR / 2);
#endif
        if (escSerial->isTransmittingData) {
            escSerial->transmissionErrors++;
        }

        timerChConfigIC(escSerial->rxTimerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
        escSerial->rxEdge = LEADING;

        escSerial->rxBitIndex = 0;
        escSerial->rxLastLeadingEdgeAtBitIndex = 0;
        escSerial->internalRxBuffer = 0;
        escSerial->isSearchingForStartBit = false;
        return;
    }

    if (escSerial->rxEdge == LEADING) {
        escSerial->rxLastLeadingEdgeAtBitIndex = escSerial->rxBitIndex;
    }

    applyChangedBitsBL(escSerial);

    if (escSerial->rxEdge == TRAILING) {
        escSerial->rxEdge = LEADING;
        timerChConfigIC(escSerial->rxTimerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
    } else {
        escSerial->rxEdge = TRAILING;
        timerChConfigIC(escSerial->rxTimerHardware, inverted ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
    }
}

static void serialTimerRxConfigBL(const timerHardware_t *timerHardwarePtr, uint8_t reference, portOptions_e options)
{
    // start bit is usually a FALLING signal
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, 0xFFFF, SystemCoreClock / 2);
    timerChConfigIC(timerHardwarePtr, (options & SERIAL_INVERTED) ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
    timerChCCHandlerInit(&escSerialPorts[reference].edgeCb, onSerialRxPinChangeBL);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].edgeCb, NULL);
}

#ifdef USE_ESCSERIAL_SIMONK
static void processTxStateEsc(escSerial_t *escSerial)
{
    uint8_t mask;
    static uint8_t bitq=0, transmitStart=0;
    if (escSerial->isReceivingData) {
        return;
    }

    if (transmitStart==0)
    {
        setTxSignalEsc(escSerial, 1);
    }
    if (!escSerial->isTransmittingData) {
        char byteToSend;
reload:
        if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
            // canreceive
            transmitStart=0;
            return;
        }

        if (transmitStart<3)
        {
            if (transmitStart==0)
                byteToSend = 0xff;
            if (transmitStart==1)
                byteToSend = 0xff;
            if (transmitStart==2)
                byteToSend = 0x7f;
            transmitStart++;
        }
        else{
            // data to send
            byteToSend = escSerial->port.txBuffer[escSerial->port.txBufferTail++];
            if (escSerial->port.txBufferTail >= escSerial->port.txBufferSize) {
                escSerial->port.txBufferTail = 0;
            }
        }


        // build internal buffer, data bits (MSB to LSB)
        escSerial->internalTxBuffer = byteToSend;
        escSerial->bitsLeftToTransmit = 8;
        escSerial->isTransmittingData = true;

        //set output
        escSerialOutputPortConfig(escSerial->rxTimerHardware);
        return;
    }

    if (escSerial->bitsLeftToTransmit) {
        mask = escSerial->internalTxBuffer & 1;
        if (mask)
        {
            if (bitq==0 || bitq==1)
            {
                setTxSignalEsc(escSerial, 1);
            }
            if (bitq==2 || bitq==3)
            {
                setTxSignalEsc(escSerial, 0);
            }
        }
        else
        {
            if (bitq==0 || bitq==2)
            {
                setTxSignalEsc(escSerial, 1);
            }
            if (bitq==1 ||bitq==3)
            {
                setTxSignalEsc(escSerial, 0);
            }
        }
        bitq++;
        if (bitq>3)
        {
            escSerial->internalTxBuffer >>= 1;
            escSerial->bitsLeftToTransmit--;
            bitq=0;
            if (escSerial->bitsLeftToTransmit==0)
            {
                goto reload;
            }
        }
        return;
    }

    if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
        escSerial->isTransmittingData = false;
        escSerialInputPortConfig(escSerial->rxTimerHardware);
    }
}

static void onSerialTimerEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    escSerial_t *escSerial = container_of(cbRec, escSerial_t, timerCb);

    if (escSerial->isReceivingData)
    {
        escSerial->receiveTimeout++;
        if (escSerial->receiveTimeout>8)
        {
            escSerial->isReceivingData=0;
            escSerial->receiveTimeout=0;
            timerChConfigIC(escSerial->rxTimerHardware, ICPOLARITY_FALLING, 0);
        }
    }

    processTxStateEsc(escSerial);
}

static void escSerialTimerTxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference)
{
    uint32_t timerPeriod = 34;
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, timerPeriod, MHZ_TO_HZ(1));
    timerChCCHandlerInit(&escSerialPorts[reference].timerCb, onSerialTimerEsc);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].timerCb, NULL);
}

static void extractAndStoreRxByteEsc(escSerial_t *escSerial)
{
    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t rxByte = (escSerial->internalRxBuffer) & 0xFF;

    if (escSerial->port.rxCallback) {
        escSerial->port.rxCallback(rxByte, escSerial->port.rxCallbackData);
    } else {
        escSerial->port.rxBuffer[escSerial->port.rxBufferHead] = rxByte;
        escSerial->port.rxBufferHead = (escSerial->port.rxBufferHead + 1) % escSerial->port.rxBufferSize;
    }
}

static void onSerialRxPinChangeEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    static uint8_t zerofirst=0;
    static uint8_t bits=0;
    static uint16_t bytes=0;

    escSerial_t *escSerial = container_of(cbRec, escSerial_t, edgeCb);

    //clear timer
#ifdef USE_HAL_DRIVER
    __HAL_TIM_SetCounter(escSerial->rxTimerHandle, 0);
#else
    TIM_SetCounter(escSerial->rxTimerHardware->tim,0);
#endif

    if (capture > 40 && capture < 90)
    {
        zerofirst++;
        if (zerofirst>1)
        {
            zerofirst=0;
            escSerial->internalRxBuffer = escSerial->internalRxBuffer>>1;
            bits++;
        }
    }
    else if (capture>90 && capture < 200)
    {
        zerofirst=0;
        escSerial->internalRxBuffer = escSerial->internalRxBuffer>>1;
        escSerial->internalRxBuffer |= 0x80;
        bits++;
    }
    else
    {
        if (!escSerial->isReceivingData)
        {
            //start
            //lets reset

            escSerial->isReceivingData = 1;
            zerofirst=0;
            bytes=0;
            bits=1;
            escSerial->internalRxBuffer = 0x80;

            timerChConfigIC(escSerial->rxTimerHardware, ICPOLARITY_RISING, 0);
        }
    }
    escSerial->receiveTimeout = 0;

    if (bits==8)
    {
        bits=0;
        bytes++;
        if (bytes>3)
        {
            extractAndStoreRxByteEsc(escSerial);
        }
        escSerial->internalRxBuffer=0;
    }

}

static void escSerialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference)
{
    // start bit is usually a FALLING signal
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, 0xFFFF, MHZ_TO_HZ(1));
    timerChConfigIC(timerHardwarePtr, ICPOLARITY_FALLING, 0);
    timerChCCHandlerInit(&escSerialPorts[reference].edgeCb, onSerialRxPinChangeEsc);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].edgeCb, NULL);
}
#endif

static void resetBuffers(escSerial_t *escSerial)
{
    escSerial->port.rxBufferSize = ESCSERIAL_BUFFER_SIZE;
    escSerial->port.rxBuffer = escSerial->rxBuffer;
    escSerial->port.rxBufferTail = 0;
    escSerial->port.rxBufferHead = 0;

    escSerial->port.txBuffer = escSerial->txBuffer;
    escSerial->port.txBufferSize = ESCSERIAL_BUFFER_SIZE;
    escSerial->port.txBufferTail = 0;
    escSerial->port.txBufferHead = 0;
}

static serialPort_t *openEscSerial(const motorDevConfig_t *motorConfig, escSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, uint16_t output, uint32_t baud, portOptions_e options, uint8_t mode)
{
    escSerial_t *escSerial = &(escSerialPorts[portIndex]);

    if (escSerialConfig()->ioTag == IO_TAG_NONE) {
        return NULL;
    }
    if (mode != PROTOCOL_KISSALL) {
        const ioTag_t tag = motorConfig->ioTags[output];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, 0);

        if (timerHardware == NULL) {
            return NULL;
        }

        escSerial->rxTimerHardware = timerHardware;
        // N-Channels can't be used as RX.
        if (escSerial->rxTimerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
            return NULL;
        }

#ifdef USE_HAL_DRIVER
        escSerial->rxTimerHandle = timerFindTimerHandle(escSerial->rxTimerHardware->tim);
#endif
    }

    escSerial->mode = mode;
    escSerial->txTimerHardware = timerAllocate(escSerialConfig()->ioTag, OWNER_MOTOR, 0);
    if (escSerial->txTimerHardware == NULL) {
        return NULL;
    }

#ifdef USE_HAL_DRIVER
    escSerial->txTimerHandle = timerFindTimerHandle(escSerial->txTimerHardware->tim);
#endif

    escSerial->port.vTable = escSerialVTable;
    escSerial->port.baudRate = baud;
    escSerial->port.mode = MODE_RXTX;
    escSerial->port.options = options;
    escSerial->port.rxCallback = callback;

    resetBuffers(escSerial);

    escSerial->isTransmittingData = false;

    escSerial->isSearchingForStartBit = true;
    escSerial->rxBitIndex = 0;

    escSerial->transmissionErrors = 0;
    escSerial->receiveErrors = 0;
    escSerial->receiveTimeout = 0;

    escSerial->escSerialPortIndex = portIndex;

    if (mode != PROTOCOL_KISSALL)
    {
        escSerial->txIO = IOGetByTag(escSerial->rxTimerHardware->tag);
        escSerialInputPortConfig(escSerial->rxTimerHardware);
        setTxSignalEsc(escSerial, ENABLE);
    }
    delay(50);

#ifdef USE_ESCSERIAL_SIMONK
    if (mode==PROTOCOL_SIMONK) {
        escSerialTimerTxConfig(escSerial->txTimerHardware, portIndex);
        escSerialTimerRxConfig(escSerial->rxTimerHardware, portIndex);
    }
    else
#endif
    if (mode==PROTOCOL_BLHELI) {
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
        serialTimerRxConfigBL(escSerial->rxTimerHardware, portIndex, options);
    }
    else if (mode==PROTOCOL_KISS) {
        escSerialOutputPortConfig(escSerial->rxTimerHardware); // rx is the pin used
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
    }
    else if (mode==PROTOCOL_KISSALL) {
        escSerial->outputCount = 0;
        memset(&escOutputs, 0, sizeof(escOutputs));
        pwmOutputPort_t *pwmMotors = pwmGetMotors();
        for (volatile uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (pwmMotors[i].enabled && pwmMotors[i].io != IO_NONE) {
                const ioTag_t tag = motorConfig->ioTags[i];
                if (tag != IO_TAG_NONE) {
                    const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, 0);
                    if (timerHardware) {
                        escSerialOutputPortConfig(timerHardware);
                        escOutputs[escSerial->outputCount].io = pwmMotors[i].io;
                        if (timerHardware->output & TIMER_OUTPUT_INVERTED) {
                            escOutputs[escSerial->outputCount].inverted = 1;
                        }
                        escSerial->outputCount++;
                    }
                }
            }
        }
        setTxSignalEsc(escSerial, ENABLE);
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
    }
    else if (mode == PROTOCOL_CASTLE) {
        escSerialOutputPortConfig(escSerial->rxTimerHardware);
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
        serialTimerRxConfigBL(escSerial->rxTimerHardware, portIndex, options);
    }
    return &escSerial->port;
}


static void escSerialInputPortDeConfig(const timerHardware_t *timerHardwarePtr)
{
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr,DISABLE);
    escSerialGPIOConfig(timerHardwarePtr, IOCFG_IPU);
}


static void closeEscSerial(escSerialPortIndex_e portIndex, uint8_t mode)
{
    escSerial_t *escSerial = &(escSerialPorts[portIndex]);

    if (mode != PROTOCOL_KISSALL) {
        escSerialInputPortDeConfig(escSerial->rxTimerHardware);
        timerChConfigCallbacks(escSerial->rxTimerHardware,NULL,NULL);
        TIM_DeInit(escSerial->rxTimerHardware->tim);
    }

    timerChConfigCallbacks(escSerial->txTimerHardware,NULL,NULL);
    TIM_DeInit(escSerial->txTimerHardware->tim);
}

static uint32_t escSerialTotalBytesWaiting(const serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    escSerial_t *s = (escSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

static uint8_t escSerialReadByte(serialPort_t *instance)
{
    uint8_t ch;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    if (escSerialTotalBytesWaiting(instance) == 0) {
        return 0;
    }

    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

static void escSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

static void escSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    UNUSED(s);
    UNUSED(baudRate);
}

static void escSerialSetMode(serialPort_t *instance, portMode_e mode)
{
    instance->mode = mode;
}

static uint32_t escSerialTxBytesFree(const serialPort_t *instance)
{
    if ((instance->mode & MODE_TX) == 0) {
        return 0;
    }

    escSerial_t *s = (escSerial_t *)instance;

    uint8_t bytesUsed = (s->port.txBufferHead - s->port.txBufferTail) & (s->port.txBufferSize - 1);

    return (s->port.txBufferSize - 1) - bytesUsed;
}

const struct serialPortVTable escSerialVTable[] = {
    {
        .serialWrite = escSerialWriteByte,
        .serialTotalRxWaiting = escSerialTotalBytesWaiting,
        .serialTotalTxFree = escSerialTxBytesFree,
        .serialRead = escSerialReadByte,
        .serialSetBaudRate = escSerialSetBaudRate,
        .isSerialTransmitBufferEmpty = isEscSerialTransmitBufferEmpty,
        .setMode = escSerialSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL
    }
};

typedef enum {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
    COMMAND_RECEIVED
} mspState_e;

typedef struct mspPort_s {
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[10];
    mspState_e c_state;
    uint8_t cmdMSP;
} mspPort_t;

static mspPort_t currentPort;

static bool processExitCommand(uint8_t c)
{
    if (currentPort.c_state == IDLE) {
        if (c == '$') {
            currentPort.c_state = HEADER_START;
        } else {
            return false;
        }
    } else if (currentPort.c_state == HEADER_START) {
        currentPort.c_state = (c == 'M') ? HEADER_M : IDLE;
    } else if (currentPort.c_state == HEADER_M) {
        currentPort.c_state = (c == '<') ? HEADER_ARROW : IDLE;
    } else if (currentPort.c_state == HEADER_ARROW) {
        if (c > 10) {
            currentPort.c_state = IDLE;

        } else {
            currentPort.dataSize = c;
            currentPort.offset = 0;
            currentPort.checksum = 0;
            currentPort.indRX = 0;
            currentPort.checksum ^= c;
            currentPort.c_state = HEADER_SIZE;
        }
    } else if (currentPort.c_state == HEADER_SIZE) {
        currentPort.cmdMSP = c;
        currentPort.checksum ^= c;
        currentPort.c_state = HEADER_CMD;
    } else if (currentPort.c_state == HEADER_CMD && currentPort.offset < currentPort.dataSize) {
        currentPort.checksum ^= c;
        currentPort.inBuf[currentPort.offset++] = c;
    } else if (currentPort.c_state == HEADER_CMD && currentPort.offset >= currentPort.dataSize) {
        if (currentPort.checksum == c) {
            currentPort.c_state = COMMAND_RECEIVED;

            if ((currentPort.cmdMSP == 0xF4) && (currentPort.dataSize==0))
            {
                currentPort.c_state = IDLE;
                return true;
            }
        } else {
            currentPort.c_state = IDLE;
        }
    }
    return false;
}


bool escEnablePassthrough(serialPort_t *escPassthroughPort, const motorDevConfig_t *motorConfig, uint16_t escIndex, uint8_t mode)
{
    bool exitEsc = false;
    uint8_t motor_output = escIndex;
    LED0_OFF;
    LED1_OFF;
    //StopPwmAllMotors();
    // XXX Review effect of motor refactor
    //pwmDisableMotors();
    motorDisable();
    passPort = escPassthroughPort;

    uint32_t escBaudrate;
    switch (mode) {
        case PROTOCOL_KISS:
            escBaudrate = BAUDRATE_KISS;
            break;
        case PROTOCOL_CASTLE:
            escBaudrate = BAUDRATE_CASTLE;
            break;
        default:
            escBaudrate = BAUDRATE_NORMAL;
            break;
    }

    if ((mode == PROTOCOL_KISS) && (motor_output == 255)) {
        mode = PROTOCOL_KISSALL;
    } else if (motor_output >= MAX_SUPPORTED_MOTORS) {
        return false;
    }

    escPort = openEscSerial(motorConfig, ESCSERIAL1, NULL, motor_output, escBaudrate, 0, mode);

    if (!escPort) {
        return false;
    }

    uint8_t ch;
    while (1) {
        if (mode!=2)
        {
            if (serialRxBytesWaiting(escPort)) {
                LED0_ON;
                while (serialRxBytesWaiting(escPort))
                {
                    ch = serialRead(escPort);
                    serialWrite(escPassthroughPort, ch);
                }
                LED0_OFF;
            }
        }
        if (serialRxBytesWaiting(escPassthroughPort)) {
            LED1_ON;
            while (serialRxBytesWaiting(escPassthroughPort))
            {
                ch = serialRead(escPassthroughPort);
                exitEsc = processExitCommand(ch);
                if (exitEsc)
                {
                    serialWrite(escPassthroughPort, 0x24);
                    serialWrite(escPassthroughPort, 0x4D);
                    serialWrite(escPassthroughPort, 0x3E);
                    serialWrite(escPassthroughPort, 0x00);
                    serialWrite(escPassthroughPort, 0xF4);
                    serialWrite(escPassthroughPort, 0xF4);
                    closeEscSerial(ESCSERIAL1, mode);
                    return true;
                }
                if (mode==PROTOCOL_BLHELI) {
                    serialWrite(escPassthroughPort, ch); // blheli loopback
                }
                serialWrite(escPort, ch);
            }
            LED1_OFF;
        }
        if (mode != PROTOCOL_CASTLE) {
            delay(5);
        }
    }
}

#endif
