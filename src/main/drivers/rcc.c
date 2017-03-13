
#include "platform.h"
#include "rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);
#if defined(USE_HAL_DRIVER)
    (void)tag;
    (void)mask;
    (void)NewState;
#else
    switch (tag) {
#if defined(STM32F3) || defined(STM32F1)
    case RCC_AHB:
        RCC_AHBPeriphClockCmd(mask, NewState);
        break;
#endif
    case RCC_APB2:
        RCC_APB2PeriphClockCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphClockCmd(mask, NewState);
        break;
#if defined(STM32F4)
    case RCC_AHB1:
        RCC_AHB1PeriphClockCmd(mask, NewState);
        break;
#endif
    }
#endif
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);
#if defined(USE_HAL_DRIVER)
    (void)tag;
    (void)mask;
    (void)NewState;
#else
    switch (tag) {
#if defined(STM32F3) || defined(STM32F10X_CL)
    case RCC_AHB:
        RCC_AHBPeriphResetCmd(mask, NewState);
        break;
#endif
    case RCC_APB2:
        RCC_APB2PeriphResetCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphResetCmd(mask, NewState);
        break;
#if defined(STM32F4)
    case RCC_AHB1:
        RCC_AHB1PeriphResetCmd(mask, NewState);
        break;
#endif
    }
#endif
}
