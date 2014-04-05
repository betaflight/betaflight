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

uint8_t readRxSignal(softSerial_t *softSerial)
{
    uint8_t invertedSignal = (digitalIn(softSerial->rxTimerHardware->gpio, softSerial->rxTimerHardware->pin) == 0);
    if (softSerial->isInverted) {
          return invertedSignal;
    }
    return !invertedSignal;
}

void setTxSignal(softSerial_t *softSerial, uint8_t state)
{
    if ((state == 1 && softSerial->isInverted == false) || (state == 0 && softSerial->isInverted == true)) {
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

void serialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t reference)
{
    // start bit is a FALLING signal
    serialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, reference, onSerialRxPinChange);
}

void serialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_Out_PP);
}

void initialiseSoftSerial(softSerial_t *softSerial, uint8_t portIndex, uint32_t baud, uint8_t inverted)
{
    softSerial->port.vTable = softSerialVTable;
    softSerial->port.mode = MODE_RXTX;

    softSerial->port.rxBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFT_SERIAL_BUFFER_SIZE,
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;

    softSerial->isTransmittingData = false;

    softSerial->isSearchingForStartBit = true;
    softSerial->rxBitIndex = 0;

    serialOutputPortConfig(softSerial->txTimerHardware);
    serialInputPortConfig(softSerial->rxTimerHardware);

    setTxSignal(softSerial, ENABLE);
    delay(50);

    serialTimerTxConfig(softSerial->txTimerHardware, portIndex, baud);
    serialTimerRxConfig(softSerial->rxTimerHardware, portIndex);
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
    char mask;

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

        // build internal buffer, start bit(1) + data bits + stop bit(0)
        softSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        softSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        softSerial->isTransmittingData = true;
        return;
    }

    mask = softSerial->internalTxBuffer & 1;
    softSerial->internalTxBuffer >>= 1;

    setTxSignal(softSerial, mask);

    if (--softSerial->bitsLeftToTransmit <= 0) {
        softSerial->isTransmittingData = false;
    }
}

enum {
    FALLING,
    RISING
};

void applyChangedBits(softSerial_t *softSerial)
{
    if (softSerial->rxPinMode == FALLING) {
        uint8_t bitToSet;
        for (bitToSet = softSerial->rxLastRiseAtBitIndex; bitToSet < softSerial->rxBitIndex ; bitToSet++) {
            softSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

void prepareForNextRxByte(softSerial_t *softSerial)
{
    // prepare for next byte
    softSerial->rxBitIndex = 0;
    softSerial->isSearchingForStartBit = true;
    if (softSerial->rxPinMode == RISING) {
        softSerial->rxPinMode = FALLING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, TIM_ICPolarity_Falling);
    }
}

void extractAndStoreRxByte(softSerial_t *softSerial)
{
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

    if (softSerial->isSearchingForStartBit) {
        TIM_SetCounter(softSerial->rxTimerHardware->tim, 0); // synchronise bit counter
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, TIM_ICPolarity_Rising);
        softSerial->rxPinMode = RISING;

        softSerial->rxBitIndex = 0;
        softSerial->rxLastRiseAtBitIndex = 0;
        softSerial->internalRxBuffer = 0;
        softSerial->isSearchingForStartBit = false;
        return;
    }

    if (softSerial->rxPinMode == RISING) {
        softSerial->rxLastRiseAtBitIndex = softSerial->rxBitIndex;
    }

    applyChangedBits(softSerial);

    if (softSerial->rxPinMode == FALLING) {
        softSerial->rxPinMode = RISING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, TIM_ICPolarity_Rising);
    } else {
        softSerial->rxPinMode = FALLING;
        serialICConfig(softSerial->rxTimerHardware->tim, softSerial->rxTimerHardware->channel, TIM_ICPolarity_Falling);
    }
}

uint8_t softSerialTotalBytesWaiting(serialPort_t *instance)
{
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
    if (softSerialTotalBytesWaiting(instance) == 0) {
        return 0;
    }

    b = instance->rxBuffer[instance->rxBufferHead];

    moveHeadToNextByte((softSerial_t *)instance);
    return b;
}

void softSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    // not implemented.
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
        isSoftSerialTransmitBufferEmpty
    }
};
