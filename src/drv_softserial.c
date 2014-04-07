#include "board.h"

#define SOFT_SERIAL_TIMER_MHZ 3
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 4
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 5
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 6
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 7

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

#define MAX_SOFTSERIAL_PORTS 2
softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];


void onSerialTimer(uint8_t portIndex, uint16_t capture);
void onSerialRxPinChange(uint8_t portIndex, uint16_t capture);

void setTxSignal(softSerial_t *softSerial, uint8_t state)
{
    if (softSerial->isInverted) {
        state = !state;
    }

    if (state) {
        digitalHi(softSerial->txTimerHardware->gpio, softSerial->txTimerHardware->pin);
    } else {
        digitalLo(softSerial->txTimerHardware->gpio, softSerial->txTimerHardware->pin);
    }
}

softSerial_t* lookupSoftSerial(uint8_t reference)
{
    assert_param(reference >= 0 && reference <= MAX_SOFTSERIAL_PORTS);

    return &(softSerialPorts[reference]);
}

void resetSerialTimer(softSerial_t *softSerial)
{
    //uint16_t counter = TIM_GetCounter(softSerial->rxTimerHardware->tim);
    TIM_SetCounter(softSerial->rxTimerHardware->tim, 0);
    //counter = TIM_GetCounter(softSerial->rxTimerHardware->tim);
}

void stopSerialTimer(softSerial_t *softSerial)
{
    TIM_Cmd(softSerial->rxTimerHardware->tim, DISABLE);
}

void startSerialTimer(softSerial_t *softSerial)
{
    TIM_Cmd(softSerial->rxTimerHardware->tim, ENABLE);
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
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU);
}

bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

void serialTimerTxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference, uint32_t baud)
{
    uint32_t clock = SystemCoreClock;
    uint32_t timerPeriod;
    do {
        timerPeriod = clock / baud;
        if (isTimerPeriodTooLarge(timerPeriod)) {
            if (clock > 1) {
                clock = clock / 2;
            } else {
                // TODO unable to continue, unable to determine clock and timerPeriods for the given baud
            }

        }
    } while (isTimerPeriodTooLarge(timerPeriod));

    uint8_t mhz = SystemCoreClock / 1000000;
    timerConfigure(timerHardwarePtr, timerPeriod, mhz);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, reference, onSerialTimer);
}

void serialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

void serialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference, uint8_t inverted)
{
    // start bit is usually a FALLING signal
    serialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, inverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, reference, onSerialRxPinChange);
}

void serialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_Out_PP);
}

void resetBuffers(softSerial_t *softSerial)
{
    softSerial->port.rxBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;
}

void initialiseSoftSerial(softSerial_t *softSerial, uint8_t portIndex, uint32_t baud, uint8_t inverted)
{
    softSerial->port.vTable = softSerialVTable;
    softSerial->port.mode = MODE_RXTX;

    resetBuffers(softSerial);

    softSerial->isTransmittingData = false;

    softSerial->isSearchingForStartBit = true;
    softSerial->rxBitIndex = 0;
    softSerial->isInverted = inverted;

    softSerial->transmissionErrors = 0;
    softSerial->receiveErrors = 0;

    serialOutputPortConfig(softSerial->txTimerHardware);
    serialInputPortConfig(softSerial->rxTimerHardware);

    setTxSignal(softSerial, ENABLE);
    delay(50);

    serialTimerTxConfig(softSerial->txTimerHardware, portIndex, baud);
    serialTimerRxConfig(softSerial->rxTimerHardware, portIndex, inverted);
}

typedef struct softSerialConfiguration_s {
    uint32_t sharedBaudRate;
    bool primaryPortInitialised;
} softSerialConfiguration_t;

softSerialConfiguration_t softSerialConfiguration = {
    0,
    false
};

void setupSoftSerialPrimary(uint32_t baud, uint8_t inverted)
{
    uint8_t portIndex = 0;
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    softSerial->rxTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_RX_HARDWARE]);
    softSerial->txTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_TX_HARDWARE]);

    initialiseSoftSerial(softSerial, portIndex, baud, inverted);

    softSerialConfiguration.sharedBaudRate = baud;
    softSerialConfiguration.primaryPortInitialised = true;
}

void setupSoftSerialSecondary(uint8_t inverted)
{
    int portIndex = 1;
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    softSerial->rxTimerHardware = &(timerHardware[SOFT_SERIAL_2_TIMER_RX_HARDWARE]);
    softSerial->txTimerHardware = &(timerHardware[SOFT_SERIAL_2_TIMER_TX_HARDWARE]);

    initialiseSoftSerial(softSerial, portIndex, softSerialConfiguration.sharedBaudRate, inverted);
}


void updateBufferIndex(softSerial_t *softSerial)
{
    if (softSerial->port.rxBufferTail >= softSerial->port.rxBufferSize - 1) {
        softSerial->port.rxBufferTail = 0; //cycling the buffer
    } else {
        softSerial->port.rxBufferTail++;
    }
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
        for (bitToSet = softSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < softSerial->rxBitIndex ; bitToSet++) {
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
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, softSerial->isInverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
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
    softSerial->port.rxBuffer[softSerial->port.rxBufferTail] = rxByte;
    updateBufferIndex(softSerial);
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

void onSerialTimer(uint8_t portIndex, uint16_t capture)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    processTxState(softSerial);
    processRxState(softSerial);
}

void onSerialRxPinChange(uint8_t portIndex, uint16_t capture)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    if ((softSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    if (softSerial->isSearchingForStartBit) {
        // synchronise bit counter
        // FIXME this reduces functionality somewhat as receiving breaks concurrent transmission on all ports because
        // the next callback to the onSerialTimer will happen too early causing transmission errors.
        TIM_SetCounter(softSerial->rxTimerHardware->tim, 0);
        if (softSerial->isTransmittingData) {
            softSerial->transmissionErrors++;
        }

        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, softSerial->isInverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
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
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, softSerial->isInverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
    } else {
        softSerial->rxEdge = TRAILING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, softSerial->isInverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    }
}

uint8_t softSerialTotalBytesWaiting(serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    int availableBytes;
    softSerial_t *softSerial = (softSerial_t *)instance;
    if (softSerial->port.rxBufferTail == softSerial->port.rxBufferHead) {
        return 0;
    }

    if (softSerial->port.rxBufferTail > softSerial->port.rxBufferHead) {
        availableBytes = softSerial->port.rxBufferTail - softSerial->port.rxBufferHead;
    } else {
        availableBytes = softSerial->port.rxBufferTail + softSerial->port.rxBufferSize - softSerial->port.rxBufferHead;
    }
    return availableBytes;
}

static void moveHeadToNextByte(softSerial_t *softSerial)
{
    if (softSerial->port.rxBufferHead < softSerial->port.rxBufferSize - 1) {
        softSerial->port.rxBufferHead++;
    } else {
        softSerial->port.rxBufferHead = 0;
    }
}

uint8_t softSerialReadByte(serialPort_t *instance)
{
    char b;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    if (softSerialTotalBytesWaiting(instance) == 0) {
        return 0;
    }

    b = instance->rxBuffer[instance->rxBufferHead];

    moveHeadToNextByte((softSerial_t *)instance);
    return b;
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
    // not implemented.
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
        softSerialTotalBytesWaiting,
        softSerialReadByte,
        softSerialSetBaudRate,
        isSoftSerialTransmitBufferEmpty,
        softSerialSetMode,
    }
};
