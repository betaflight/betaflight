
#include "platform.h"
#include "rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);
    switch(tag) {
    case RCC_AHB:
        RCC_AHBPeriphClockCmd(mask, NewState);
        break;
    case RCC_APB2:
        RCC_APB2PeriphClockCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphClockCmd(mask, NewState);
        break;
    }
}
