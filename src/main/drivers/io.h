#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "resource.h"

#include "drivers/io_types.h"

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

#if defined(STM32F1)

// mode is using only bits 6-2
#define IO_CONFIG(mode, speed) ((mode) | (speed))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_2MHz)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_Out_OD,      GPIO_Speed_2MHz)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_2MHz)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF_OD,       GPIO_Speed_2MHz)
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IPD,         GPIO_Speed_2MHz)
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IPU,         GPIO_Speed_2MHz)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_2MHz)

#elif defined(STM32F7)

//speed is packed inside modebits 5 and 2,
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,      GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

#elif defined(STM32F3) || defined(STM32F4)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_Mode_IN,  GPIO_Speed_25MHz, 0, GPIO_PuPd_UP)

#elif defined(UNIT_TEST)

# define IOCFG_OUT_PP         0
# define IOCFG_OUT_OD         0
# define IOCFG_AF_PP          0
# define IOCFG_AF_OD          0
# define IOCFG_IPD            0
# define IOCFG_IPU            0
# define IOCFG_IN_FLOATING    0

#else
# warning "Unknown TARGET"
#endif

// declare available IO pins. Available pins are specified per target
#include "io_def.h"

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);

void IOInit(IO_t io, resourceOwner_t owner, resourceType_t resource, uint8_t index);
void IORelease(IO_t io);  // unimplemented
resourceOwner_t IOGetOwner(IO_t io);
resourceType_t IOGetResources(IO_t io);
IO_t IOGetByTag(ioTag_t tag);

void IOConfigGPIO(IO_t io, ioConfig_t cfg);
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
#endif

void IOInitGlobal(void);
