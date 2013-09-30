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