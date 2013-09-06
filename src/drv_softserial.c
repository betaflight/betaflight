#include "board.h"

enum serialBitStatus {
    WAITING_FOR_START_BIT = -1, BIT_0, BIT_1, BIT_2, BIT_3, BIT_4, BIT_5, BIT_6, BIT_7, WAITING_FOR_STOP_BIT
};

#define MAX_SOFTSERIAL_PORTS 2
softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

softSerial_t* lookupSoftSerial(uint8_t reference)
{
    assert_param(reference >= 0 && reference <= MAX_SOFTSERIAL_PORTS);

    return &(softSerialPorts[reference]);
}


void stopSerialTimer(softSerial_t *softSerial)
{
    TIM_Cmd(softSerial->timerHardware->tim, DISABLE);
    TIM_SetCounter(softSerial->timerHardware->tim, 0);
}

void startSerialTimer(softSerial_t *softSerial)
{
    TIM_SetCounter(softSerial->timerHardware->tim, 0);
    TIM_Cmd(softSerial->timerHardware->tim, ENABLE);
}

static void waitForFirstBit(softSerial_t *softSerial)
{
    softSerial->state = BIT_0;
    startSerialTimer(softSerial);
}

static void onSerialPinChange(uint8_t reference, uint16_t capture)
{
    softSerial_t *softSerial = lookupSoftSerial(reference);
    if (softSerial->state == WAITING_FOR_START_BIT) {
        waitForFirstBit(softSerial);
    }
}

uint8_t readSerialSignal(softSerial_t *softSerial)
{
    return GPIO_ReadInputDataBit(softSerial->timerHardware->gpio, softSerial->timerHardware->pin);
}

void mergeSignalWithCurrentByte(softSerial_t *softSerial, uint8_t serialSignal)
{
    softSerial->rxBuffer[softSerial->port.rxBufferTail] |= (serialSignal << softSerial->state);
}

inline void initialiseCurrentByteWithFirstSignal(softSerial_t *softSerial, uint8_t serialSignal)
{
    softSerial->rxBuffer[softSerial->port.rxBufferTail] = serialSignal;
}

inline void prepareForNextSignal(softSerial_t *softSerial) {
    softSerial->state++;
}

void updateBufferIndex(softSerial_t *softSerial)
{
    if (softSerial->port.rxBufferTail >= softSerial->port.rxBufferSize - 1)
    {
        softSerial->port.rxBufferTail = 0; //cycling the buffer
    } else {
        softSerial->port.rxBufferTail++;
    }
}

void prepareForNextByte(softSerial_t *softSerial)
{
    stopSerialTimer(softSerial);
    softSerial->state = WAITING_FOR_START_BIT;
    updateBufferIndex(softSerial);
}

void onSerialTimer(uint8_t portIndex, uint16_t capture)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    uint8_t serialSignal = readSerialSignal(softSerial);

    switch (softSerial->state) {
    case BIT_0:
        initialiseCurrentByteWithFirstSignal(softSerial, serialSignal);
        prepareForNextSignal(softSerial);
        break;

    case BIT_1:
    case BIT_2:
    case BIT_3:
    case BIT_4:
    case BIT_5:
    case BIT_6:
    case BIT_7:
        mergeSignalWithCurrentByte(softSerial, serialSignal);

        prepareForNextSignal(softSerial);
        break;

    case WAITING_FOR_STOP_BIT:
        prepareForNextByte(softSerial);
        break;
    }
}

#define SOFT_SERIAL_TIMER_MHZ 1
#define SOFT_SERIAL_1_TIMER_HARDWARE 4

static void softSerialGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void serialInputPortConfig(const timerHardware_t *timerHardwarePtr, uint16_t baud, uint8_t reference, timerCCCallbackPtr callback)
{
    softSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU);

    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);

    int oneBitPeriod = (SOFT_SERIAL_TIMER_MHZ * 1000000) / baud;

    timerInConfig(timerHardwarePtr, oneBitPeriod, SOFT_SERIAL_TIMER_MHZ);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, reference, callback);
}

void setupSoftSerial1(uint32_t baud)
{
    int portIndex = 0;
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);

    softSerial->timerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_HARDWARE]);
    softSerial->timerIndex = SOFT_SERIAL_1_TIMER_HARDWARE;
    softSerial->state = WAITING_FOR_START_BIT;

    softSerial->port.rxBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = 0;
    softSerial->port.txBufferSize = 0;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;

    softSerial->port.baudRate = baud;
    softSerial->port.mode = MODE_RX;

    // FIXME this uart specific initialisation doesn't belong really here
    softSerial->port.txDMAChannel = 0;
    softSerial->port.txDMAChannel = 0;

    configureTimerChannelCallback(softSerial->timerHardware->tim, TIM_Channel_2, portIndex, onSerialTimer);

    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
    stopSerialTimer(softSerial);

    serialInputPortConfig(softSerial->timerHardware, baud, portIndex, onSerialPinChange);
}

bool serialAvailable(softSerial_t *softSerial)
{
    if (softSerial->port.rxBufferTail == softSerial->port.rxBufferHead) {
        return 0;
    }

    int availableBytes;
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

uint8_t serialReadByte(softSerial_t *softSerial)
{
    if (serialAvailable(softSerial) == 0) {
        return 0;
    }

    char b = softSerial->port.rxBuffer[softSerial->port.rxBufferHead];

    moveHeadToNextByte(softSerial);
    return b;
}
