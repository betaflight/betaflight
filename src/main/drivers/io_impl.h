#pragma once

// TODO - GPIO_TypeDef include
#include "drivers/io.h"
#include "platform.h"

typedef struct ioDef_s {
    ioTag_t tag;
} ioDef_t;

typedef struct ioRec_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    resourceOwner_t owner;
    resourceType_t resource;
    uint8_t index;
} ioRec_t;

extern ioRec_t ioRecs[DEFIO_IO_USED_COUNT];

int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);

int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);

#if defined(STM32F3) || defined(STM32F4)
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
#endif

GPIO_TypeDef* IO_GPIO(IO_t io);
uint16_t IO_Pin(IO_t io);

#define IO_GPIOBYTAG(tag) IO_GPIO(IOGetByTag(tag))
#define IO_PINBYTAG(tag) IO_Pin(IOGetByTag(tag))

uint32_t IO_EXTI_Line(IO_t io);
ioRec_t *IO_Rec(IO_t io);
