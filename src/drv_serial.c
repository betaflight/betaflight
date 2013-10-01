#include "board.h"

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++))) {
        serialWrite(instance, ch);
    }
}

inline uint32_t serialGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

inline void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

inline uint8_t serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalBytesWaiting(instance);
}

inline uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

inline void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

inline bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

