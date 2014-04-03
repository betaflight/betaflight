#include "board.h"

#define SOFT_SERIAL_TIMER_MHZ 3
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 4
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 5

#define RX_TOTAL_BITS 8
#define TX_TOTAL_BITS 10

#define MAX_SOFTSERIAL_PORTS 2
softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

void onSerialTimer(uint8_t portIndex, uint16_t capture);

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

#define TICKS_PER_BIT 3

bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

void serialTimerConfig(const timerHardware_t *timerHardwarePtr, uint32_t baud, uint8_t reference, timerCCCallbackPtr callback)
{
    uint32_t clock = SystemCoreClock;
    uint32_t timerPeriod;
    do {
        timerPeriod = clock / (baud * TICKS_PER_BIT);
        if (isTimerPeriodTooLarge(timerPeriod)) {
            if (clock > 1) {
                clock = clock / 2;
            } else {
                // TODO unable to continue, unable to determine clock and timerPeriods for the given baud
            }

        }
    } while (isTimerPeriodTooLarge(timerPeriod));

    uint8_t mhz = SystemCoreClock / 1000000;
    timerInConfig(timerHardwarePtr, timerPeriod, mhz);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, reference, callback);
}

void serialOutputPortConfig(const timerHardware_t *timerHardwarePtr)
{
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_Out_PP);
}

void setupSoftSerial1(uint32_t baud, uint8_t inverted)
{
    int portIndex = 0;
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    softSerial->port.vTable = softSerialVTable;
    softSerial->port.mode = MODE_RXTX;
    softSerial->port.baudRate = baud;
    softSerial->isInverted = inverted;

    softSerial->rxTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_RX_HARDWARE]);
    softSerial->txTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_TX_HARDWARE]);

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
    softSerial->isSearchingForStopBit = false;

    softSerial->timerRxCounter = 1;

    serialInputPortConfig(softSerial->rxTimerHardware);
    serialOutputPortConfig(softSerial->txTimerHardware);

    setTxSignal(softSerial, 1);
    delay(50);

    serialTimerConfig(softSerial->rxTimerHardware, baud, portIndex, onSerialTimer);
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

void searchForStartBit(softSerial_t *softSerial)
{
    char rxSignal = readRxSignal(softSerial);
    if (rxSignal == 1) {
        // start bit not found
        softSerial->timerRxCounter = 1; // process on next timer event
        return;
    }

    // timer is aligned to falling signal of start bit.
    // three ticks per bit.

    softSerial->isSearchingForStartBit = false;
    softSerial->internalRxBuffer = 0;
    softSerial->timerRxCounter = TICKS_PER_BIT + 1; // align to middle of next bit
    softSerial->bitsLeftToReceive = RX_TOTAL_BITS;
    softSerial->rxBitSelectionMask = 1;
}

void searchForStopBit(softSerial_t *softSerial)
{
    char rxSignal;
    softSerial->timerRxCounter = 1;

    rxSignal = readRxSignal(softSerial);
    if (rxSignal != 1) {
        // not found
        return;
    }

    softSerial->isSearchingForStopBit = false;
    softSerial->isSearchingForStartBit = true;
    softSerial->internalRxBuffer &= 0xFF;

    softSerial->port.rxBuffer[softSerial->port.rxBufferTail] = softSerial->internalRxBuffer;
    updateBufferIndex(softSerial);
}

void readDataBit(softSerial_t *softSerial)
{
    softSerial->timerRxCounter = TICKS_PER_BIT; // keep aligned to middle of bit

    char rxSignal = readRxSignal(softSerial);
    if (rxSignal) {
        softSerial->internalRxBuffer |= softSerial->rxBitSelectionMask;
    }
    softSerial->rxBitSelectionMask <<= 1;
    if (--softSerial->bitsLeftToReceive <= 0) {
        softSerial->isSearchingForStopBit = true;
        softSerial->timerRxCounter = 2;
    }
}

void processRxState(softSerial_t *softSerial)
{
    //digitalToggle(softSerial->txTimerHardware->gpio, softSerial->txTimerHardware->pin);

    if (--softSerial->timerRxCounter > 0) {
        return;
    }

    if (softSerial->isSearchingForStartBit) {
        searchForStartBit(softSerial);
        return;
    }

    if (softSerial->isSearchingForStopBit) {
        searchForStopBit(softSerial);
        return;
    }

    readDataBit(softSerial);
}


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

        // start immediately
        softSerial->timerTxCounter = 1;
        softSerial->isTransmittingData = true;
        return;
    }

    if (--softSerial->timerTxCounter <= 0) {
        mask = softSerial->internalTxBuffer & 1;
        softSerial->internalTxBuffer >>= 1;

        setTxSignal(softSerial, mask);

        softSerial->timerTxCounter = TICKS_PER_BIT;
        if (--softSerial->bitsLeftToTransmit <= 0) {
            softSerial->isTransmittingData = false;
        }
    }
}

void onSerialTimer(uint8_t portIndex, uint16_t capture)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    processTxState(softSerial);
    processRxState(softSerial);
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
