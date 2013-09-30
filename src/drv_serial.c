#include "board.h"


void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++))) {
        serialWrite(instance, ch);
    }
}
