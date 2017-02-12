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
#include <string.h>

#include "platform.h"

typedef enum {
    BAUDRATE_NORMAL = 19200,
    BAUDRATE_KISS   = 38400,
    BAUDRATE_CASTLE = 18880
} escBaudRate_e;

#if defined(USE_ESCSERIAL)

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "nvic.h"
#include "system.h"
#include "io.h"
#include "timer.h"

#include "serial.h"
#include "serial_escserial.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"
#include "io/serial.h"
#include "flight/mixer.h"

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

#define MAX_ESCSERIAL_PORTS 1
static serialPort_t *escPort = NULL;
static serialPort_t *passPort = NULL;

typedef struct escSerial_s {
    serialPort_t     port;

    IO_t rxIO;
    IO_t txIO;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[ESCSERIAL_BUFFER_SIZE];
    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[ESCSERIAL_BUFFER_SIZE];

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
extern escSerial_t escSerialPorts[];

extern const struct serialPortVTable escSerialVTable[];


escSerial_t escSerialPorts[MAX_ESCSERIAL_PORTS];

void onSerialTimerEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialRxPinChangeEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialTimerBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialRxPinChangeBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
static void escSerialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

void setTxSignalEsc(escSerial_t *escSerial, uint8_t state)
{
    if(escSerial->mode == PROTOCOL_KISSALL)
    {
        for (volatile uint8_t i = 0; i < escSerial->outputCount; i++) {
            uint8_t state_temp = state;
            if(escOutputs[i].inverted) {
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
        if(escSerial->rxTimerHardware->output & TIMER_OUTPUT_INVERTED) {
            state ^= ENABLE;
        }

        if (state) {
            IOHi(escSerial->txIO);
        } else {
            IOLo(escSerial->txIO);
        }
    }
}

static void escSerialGPIOConfig(ioTag_t tag, ioConfig_t cfg)
{
    if (!tag) {
        return;
    }

    IOInit(IOGetByTag(tag), OWNER_MOTOR, 0);
    IOConfigGPIO(IOGetByTag(tag), cfg);
}

void escSerialInputPortConfig(const timerHardware_t *timerHardwarePtr)
{
#ifdef STM32F10X
    escSerialGPIOConfig(timerHardwarePtr->tag, IOCFG_IPU);
#else
    escSerialGPIOConfig(timerHardwarePtr->tag, IOCFG_AF_PP_UP);
#endif
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr,ENABLE);
}


static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
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

    uint8_t mhz = clock / 1000000;
    timerConfigure(timerHardwarePtr, timerPeriod, mhz);
    timerChCCHandlerInit(&escSerialPorts[reference].timerCb, onSerialTimerBL);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].timerCb, NULL);
}

static void serialTimerRxConfigBL(const timerHardware_t *timerHardwarePtr, uint8_t reference, portOptions_t options)
{
    // start bit is usually a FALLING signal
    uint8_t mhz = SystemCoreClock / 2000000;
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, 0xFFFF, mhz);
    escSerialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, (options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    timerChCCHandlerInit(&escSerialPorts[reference].edgeCb, onSerialRxPinChangeBL);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].edgeCb, NULL);
}

static void escSerialTimerTxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference)
{
    uint32_t timerPeriod=34;
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, timerPeriod, 1);
    timerChCCHandlerInit(&escSerialPorts[reference].timerCb, onSerialTimerEsc);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].timerCb, NULL);
}

static void escSerialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
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

static void escSerialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference)
{
    // start bit is usually a FALLING signal
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, 0xFFFF, 1);
    escSerialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
    timerChCCHandlerInit(&escSerialPorts[reference].edgeCb, onSerialRxPinChangeEsc);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[reference].edgeCb, NULL);
}

static void escSerialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    escSerialGPIOConfig(timerHardwarePtr->tag, IOCFG_OUT_PP);
    timerChITConfig(timerHardwarePtr,DISABLE);
}

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

serialPort_t *openEscSerial(escSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, uint16_t output, uint32_t baud, portOptions_t options, uint8_t mode)
{
    escSerial_t *escSerial = &(escSerialPorts[portIndex]);

    if(mode != PROTOCOL_KISSALL){
        escSerial->rxTimerHardware = &(timerHardware[output]);
    }

    escSerial->mode = mode;
    escSerial->txTimerHardware = &(timerHardware[ESCSERIAL_TIMER_TX_HARDWARE]);

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

    if(mode != PROTOCOL_KISSALL)
    {
        escSerial->txIO = IOGetByTag(escSerial->rxTimerHardware->tag);
        escSerialInputPortConfig(escSerial->rxTimerHardware);
        setTxSignalEsc(escSerial, ENABLE);
    }
    delay(50);

    if(mode==PROTOCOL_SIMONK){
        escSerialTimerTxConfig(escSerial->txTimerHardware, portIndex);
        escSerialTimerRxConfig(escSerial->rxTimerHardware, portIndex);
    }
    else if(mode==PROTOCOL_BLHELI){
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
        serialTimerRxConfigBL(escSerial->rxTimerHardware, portIndex, options);
    }
    else if(mode==PROTOCOL_KISS) {
        escSerialOutputPortConfig(escSerial->rxTimerHardware); // rx is the pin used
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
    }
    else if(mode==PROTOCOL_KISSALL) {
        escSerial->outputCount = 0;
        memset(&escOutputs, 0, sizeof(escOutputs));
        pwmOutputPort_t *pwmMotors = pwmGetMotors();
        for (volatile uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (pwmMotors[i].enabled) {
                if (pwmMotors[i].io != IO_NONE) {
                    for (volatile uint8_t j = 0; j < USABLE_TIMER_CHANNEL_COUNT; j++) {
                        if(pwmMotors[i].io == IOGetByTag(timerHardware[j].tag))
                        {
                            escSerialOutputPortConfig(&timerHardware[j]);
                            if(timerHardware[j].output & TIMER_OUTPUT_INVERTED) {
                                escOutputs[escSerial->outputCount].inverted = 1;
                            }
                            break;
                        }
                    }
                    escOutputs[escSerial->outputCount].io = pwmMotors[i].io;
                    escSerial->outputCount++;
                }
            }
        }
        setTxSignalEsc(escSerial, ENABLE);
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
    }
    else if(mode == PROTOCOL_CASTLE){
        escSerialOutputPortConfig(escSerial->rxTimerHardware);
        serialTimerTxConfigBL(escSerial->txTimerHardware, portIndex, baud);
        serialTimerRxConfigBL(escSerial->rxTimerHardware, portIndex, options);
    }
    return &escSerial->port;
}


void escSerialInputPortDeConfig(const timerHardware_t *timerHardwarePtr)
{
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr,DISABLE);
    escSerialGPIOConfig(timerHardwarePtr->tag, IOCFG_IPU);
}


void closeEscSerial(escSerialPortIndex_e portIndex, uint8_t mode)
{
    escSerial_t *escSerial = &(escSerialPorts[portIndex]);

    if(mode != PROTOCOL_KISSALL){
        escSerialInputPortDeConfig(escSerial->rxTimerHardware);
        timerChConfigCallbacks(escSerial->rxTimerHardware,NULL,NULL);
        TIM_DeInit(escSerial->rxTimerHardware->tim);
    }

    timerChConfigCallbacks(escSerial->txTimerHardware,NULL,NULL);
    TIM_DeInit(escSerial->txTimerHardware->tim);
}

/*********************************************/

void processTxStateEsc(escSerial_t *escSerial)
{
    uint8_t mask;
    static uint8_t bitq=0, transmitStart=0;
    if (escSerial->isReceivingData) {
        return;
    }

    if(transmitStart==0)
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

        if(transmitStart<3)
        {
            if(transmitStart==0)
                byteToSend = 0xff;
            if(transmitStart==1)
                byteToSend = 0xff;
            if(transmitStart==2)
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
        if(mask)
        {
            if(bitq==0 || bitq==1)
            {
                setTxSignalEsc(escSerial, 1);
            }
            if(bitq==2 || bitq==3)
            {
                setTxSignalEsc(escSerial, 0);
            }
        }
        else
        {
            if(bitq==0 || bitq==2)
            {
                setTxSignalEsc(escSerial, 1);
            }
            if(bitq==1 ||bitq==3)
            {
                setTxSignalEsc(escSerial, 0);
            }
        }
        bitq++;
        if(bitq>3)
        {
            escSerial->internalTxBuffer >>= 1;
            escSerial->bitsLeftToTransmit--;
            bitq=0;
            if(escSerial->bitsLeftToTransmit==0)
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

/*-----------------------BL*/
/*********************************************/

void processTxStateBL(escSerial_t *escSerial)
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
        if(escSerial->mode==PROTOCOL_BLHELI || escSerial->mode==PROTOCOL_CASTLE) {
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
        if(escSerial->mode==PROTOCOL_BLHELI || escSerial->mode==PROTOCOL_CASTLE)
        {
            escSerialInputPortConfig(escSerial->rxTimerHardware);
        }
    }
}



enum {
    TRAILING,
    LEADING
};

void applyChangedBitsBL(escSerial_t *escSerial)
{
    if (escSerial->rxEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = escSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < escSerial->rxBitIndex; bitToSet++) {
            escSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

void prepareForNextRxByteBL(escSerial_t *escSerial)
{
    // prepare for next byte
    escSerial->rxBitIndex = 0;
    escSerial->isSearchingForStartBit = true;
    if (escSerial->rxEdge == LEADING) {
        escSerial->rxEdge = TRAILING;
        escSerialICConfig(
            escSerial->rxTimerHardware->tim,
            escSerial->rxTimerHardware->channel,
            (escSerial->port.options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling
        );
    }
}

#define STOP_BIT_MASK (1 << 0)
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))

void extractAndStoreRxByteBL(escSerial_t *escSerial)
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
        escSerial->port.rxCallback(rxByte);
    } else {
        escSerial->port.rxBuffer[escSerial->port.rxBufferHead] = rxByte;
        escSerial->port.rxBufferHead = (escSerial->port.rxBufferHead + 1) % escSerial->port.rxBufferSize;
    }
}

void processRxStateBL(escSerial_t *escSerial)
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

void onSerialTimerBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    escSerial_t *escSerial = container_of(cbRec, escSerial_t, timerCb);

    processTxStateBL(escSerial);
    processRxStateBL(escSerial);
}

void onSerialRxPinChangeBL(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);

    escSerial_t *escSerial = container_of(cbRec, escSerial_t, edgeCb);
    bool inverted = escSerial->port.options & SERIAL_INVERTED;

    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    if (escSerial->isSearchingForStartBit) {
        // synchronise bit counter
        // FIXME this reduces functionality somewhat as receiving breaks concurrent transmission on all ports because
        // the next callback to the onSerialTimer will happen too early causing transmission errors.
        TIM_SetCounter(escSerial->txTimerHardware->tim, escSerial->txTimerHardware->tim->ARR / 2);
        if (escSerial->isTransmittingData) {
            escSerial->transmissionErrors++;
        }

        escSerialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
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
        escSerialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
    } else {
        escSerial->rxEdge = TRAILING;
        escSerialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    }
}
/*-------------------------BL*/

void extractAndStoreRxByteEsc(escSerial_t *escSerial)
{
    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t rxByte = (escSerial->internalRxBuffer) & 0xFF;

    if (escSerial->port.rxCallback) {
        escSerial->port.rxCallback(rxByte);
    } else {
        escSerial->port.rxBuffer[escSerial->port.rxBufferHead] = rxByte;
        escSerial->port.rxBufferHead = (escSerial->port.rxBufferHead + 1) % escSerial->port.rxBufferSize;
    }
}

void onSerialTimerEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    escSerial_t *escSerial = container_of(cbRec, escSerial_t, timerCb);

    if(escSerial->isReceivingData)
    {
        escSerial->receiveTimeout++;
        if(escSerial->receiveTimeout>8)
        {
            escSerial->isReceivingData=0;
            escSerial->receiveTimeout=0;
            escSerialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, TIM_ICPolarity_Falling);
        }
    }


    processTxStateEsc(escSerial);
}

void onSerialRxPinChangeEsc(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    static uint8_t zerofirst=0;
    static uint8_t bits=0;
    static uint16_t bytes=0;

    escSerial_t *escSerial = container_of(cbRec, escSerial_t, edgeCb);

    //clear timer
    TIM_SetCounter(escSerial->rxTimerHardware->tim,0);

    if(capture > 40 && capture < 90)
    {
        zerofirst++;
        if(zerofirst>1)
        {
            zerofirst=0;
            escSerial->internalRxBuffer = escSerial->internalRxBuffer>>1;
            bits++;
        }
    }
    else if(capture>90 && capture < 200)
    {
        zerofirst=0;
        escSerial->internalRxBuffer = escSerial->internalRxBuffer>>1;
        escSerial->internalRxBuffer |= 0x80;
        bits++;
    }
    else
    {
        if(!escSerial->isReceivingData)
        {
            //start
            //lets reset

            escSerial->isReceivingData = 1;
            zerofirst=0;
            bytes=0;
            bits=1;
            escSerial->internalRxBuffer = 0x80;

            escSerialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, TIM_ICPolarity_Rising);
        }
    }
    escSerial->receiveTimeout = 0;

    if(bits==8)
    {
        bits=0;
        bytes++;
        if(bytes>3)
        {
            extractAndStoreRxByteEsc(escSerial);
        }
        escSerial->internalRxBuffer=0;
    }

}

uint32_t escSerialTotalBytesWaiting(const serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    escSerial_t *s = (escSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

uint8_t escSerialReadByte(serialPort_t *instance)
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

void escSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

void escSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    UNUSED(s);
    UNUSED(baudRate);
}

void escSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->mode = mode;
}

bool isEscSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    // start listening
    return instance->txBufferHead == instance->txBufferTail;
}

uint32_t escSerialTxBytesFree(const serialPort_t *instance)
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
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL
    }
};

void escSerialInitialize()
{
   //StopPwmAllMotors();
   pwmDisableMotors();

   for (volatile uint8_t i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
       // set outputs to pullup
       if(timerHardware[i].output & TIMER_OUTPUT_ENABLED)
       {
           escSerialGPIOConfig(timerHardware[i].tag, IOCFG_IPU); //GPIO_Mode_IPU
       }
   }
}

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

static bool ProcessExitCommand(uint8_t c)
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

            if((currentPort.cmdMSP == 0xF4) && (currentPort.dataSize==0))
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


// mode 0=sk, 1=bl, 2=ki   output=timerHardware PWM channel.
void escEnablePassthrough(serialPort_t *escPassthroughPort, uint16_t output, uint8_t mode)
{
    bool exitEsc = false;
    uint8_t motor_output = 0;
    LED0_OFF;
    LED1_OFF;
    //StopPwmAllMotors();
    pwmDisableMotors();
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

    if((mode == PROTOCOL_KISS) && (output == 255)){
        motor_output = 255;
        mode = PROTOCOL_KISSALL;
    }
    else {
        uint8_t first_output = 0;
        for (volatile uint8_t i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
            if(timerHardware[i].output & TIMER_OUTPUT_ENABLED)
            {
                first_output=i;
                break;
            }
        }

        //doesn't work with messy timertable
        motor_output=first_output+output-1;
        if(motor_output >=USABLE_TIMER_CHANNEL_COUNT)
            return;
    }

    escPort = openEscSerial(ESCSERIAL1, NULL, motor_output, escBaudrate, 0, mode);
    uint8_t ch;
    while(1) {
        if(mode!=2)
        {
            if (serialRxBytesWaiting(escPort)) {
                LED0_ON;
                while(serialRxBytesWaiting(escPort))
                {
                    ch = serialRead(escPort);
                    serialWrite(escPassthroughPort, ch);
                }
                LED0_OFF;
            }
        }
        if (serialRxBytesWaiting(escPassthroughPort)) {
            LED1_ON;
            while(serialRxBytesWaiting(escPassthroughPort))
            {
                ch = serialRead(escPassthroughPort);
                exitEsc = ProcessExitCommand(ch);
                if(exitEsc)
                {
                    serialWrite(escPassthroughPort, 0x24);
                    serialWrite(escPassthroughPort, 0x4D);
                    serialWrite(escPassthroughPort, 0x3E);
                    serialWrite(escPassthroughPort, 0x00);
                    serialWrite(escPassthroughPort, 0xF4);
                    serialWrite(escPassthroughPort, 0xF4);
                    closeEscSerial(ESCSERIAL1, mode);
                    return;
                }
                if(mode==PROTOCOL_BLHELI){
                    serialWrite(escPassthroughPort, ch); // blheli loopback
                }
                serialWrite(escPort, ch);
            }
            LED1_OFF;
        }
        if(mode != PROTOCOL_CASTLE){
            delay(5);
        }
    }
}


#endif
