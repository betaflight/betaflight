#pragma once

// TODO - GPIO_TypeDef include
#include "io.h"
#include "platform.h"

typedef struct ioDef_s {
    ioTag_t tag;
} ioDef_t;

typedef struct ioRec_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    resourceOwner_t owner;
    resourceType_t resourcesUsed; // TODO!
} ioRec_t;

extern ioRec_t ioRecs[DEFIO_IO_USED_COUNT];

int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);
#if defined(STM32F10X)
int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);
#elif defined(STM32F303xC)
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
#endif
uint32_t IO_EXTI_Line(IO_t io);
ioRec_t *IO_Rec(IO_t io);
