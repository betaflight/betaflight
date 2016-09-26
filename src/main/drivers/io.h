#pragma once

#include <stdbool.h>

#include "resource.h"

// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
typedef uint8_t ioTag_t;          // packed tag to specify IO pin
typedef void* IO_t;               // type specifying IO pin. Currently ioRec_t pointer, but this may change

// NONE initializer for IO_t variable
#define IO_NONE ((IO_t)0)

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment (struct can't be used as boolean)
//   IO_t being pointer is only possibility I know of

// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences

typedef uint8_t ioConfig_t;  // packed IO configuration
#if defined(STM32F10X)

// mode is using only bits 6-2
# define IO_CONFIG(mode, speed) ((mode) | (speed))

# define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_2MHz)
# define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_Out_OD,      GPIO_Speed_2MHz)
# define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_2MHz)
# define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF_OD,       GPIO_Speed_2MHz)
# define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IPD,         GPIO_Speed_2MHz)
# define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IPU,         GPIO_Speed_2MHz)
# define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_2MHz)

#elif defined(STM32F303xC)

# define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

# define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
# define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
# define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
# define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
# define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
# define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
# define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)

#elif defined(UNIT_TEST)

# define IOCFG_OUT_PP         0
# define IOCFG_OUT_OD         0
# define IOCFG_AF_PP          0
# define IOCFG_AF_OD          0
# define IOCFG_IPD            0
# define IOCFG_IPU            0
# define IOCFG_IN_FLOATING    0

#else
# warning "Unsupported CPU"
#endif

// declare available IO pins. Available pins are specified per target
#include "io_def.h"

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);

void IOInit(IO_t io, resourceOwner_t owner, resourceType_t resources);
void IORelease(IO_t io);  // unimplemented
resourceOwner_t IOGetOwner(IO_t io);
resourceType_t IOGetResources(IO_t io);
IO_t IOGetByTag(ioTag_t tag);

void IOConfigGPIO(IO_t io, ioConfig_t cfg);
#if defined(STM32F303xC)
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
#endif

void IOInitGlobal(void);
