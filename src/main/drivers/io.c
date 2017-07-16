#include "platform.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"

#include "build/assert.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(STM32F1)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_APB2(IOPA) },
    { RCC_APB2(IOPB) },
    { RCC_APB2(IOPC) },
    { RCC_APB2(IOPD) },
    { RCC_APB2(IOPE) },
{
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
    RCC_APB2(IOPF),
#else
    0,
#endif
},
{
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
    RCC_APB2(IOPG),
#else
    0,
#endif
},
};
#elif defined(STM32F3)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB(GPIOA) },
    { RCC_AHB(GPIOB) },
    { RCC_AHB(GPIOC) },
    { RCC_AHB(GPIOD) },
    { RCC_AHB(GPIOE) },
    { RCC_AHB(GPIOF) },
};
#elif defined(STM32F4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
};
#elif defined(STM32F7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
};
# endif

const char * const ownerNames[OWNER_TOTAL_COUNT] = {
    "FREE", "PWM", "PPM", "MOTOR", "SERVO", "SOFTSERIAL", "ADC", "SERIAL", "DEBUG", "TIMER",
    "RANGEFINDER", "SYSTEM", "SPI", "I2C", "SDCARD", "FLASH", "USB", "BEEPER", "OSD",
    "BARO", "MPU", "INVERTER", "LED STRIP", "LED", "RECEIVER", "TRANSMITTER",
    "SOFTSPI", "NRF24", "VTX"
};

const char * const resourceNames[RESOURCE_TOTAL_COUNT] = {
    "", // NONE
    "IN", "OUT", "IN / OUT",
    "TIMER",
    "UART TX", "UART RX", "UART TX/RX",
    "EXTI",
    "SCL", "SDA",
    "SCK", "MOSI", "MISO", "CS",
    "CH1", "CH2", "CH3", "CH4",
    "CE"
};


ioRec_t* IO_Rec(IO_t io)
{
    ASSERT(io != NULL);
    ASSERT((ioRec_t*)io >= &ioRecs[0]);
    ASSERT((ioRec_t*)io < &ioRecs[DEFIO_IO_USED_COUNT]);

    return io;
}

GPIO_TypeDef* IO_GPIO(IO_t io)
{
    ioRec_t *ioRec = IO_Rec(io);
    return ioRec->gpio;
}

uint16_t IO_Pin(IO_t io)
{
    ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pin;
}

// port index, GPIOA == 0
int IO_GPIOPortIdx(IO_t io)
{
    if (!io)
        return -1;
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io)
        return -1;
    return 31 - __builtin_clz(IO_Pin(io));  // CLZ is a bit faster than FFS
}

int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io)
        return 0;
#if defined(STM32F1)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(STM32F3)
    return IO_GPIOPinIdx(io);
#elif defined(STM32F4)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(STM32F7)
    return 1 << IO_GPIOPinIdx(io);
#else
# error "Unknown target type"
#endif
}

bool IORead(IO_t io)
{
    if (!io)
        return false;
#if defined(USE_HAL_DRIVER)
    return !! HAL_GPIO_ReadPin(IO_GPIO(io),IO_Pin(io));
#else
    return !! (IO_GPIO(io)->IDR & IO_Pin(io));
#endif
}

void IOWrite(IO_t io, bool hi)
{
    if (!io)
        return;
#if defined(USE_HAL_DRIVER)
    if (hi) {
        HAL_GPIO_WritePin(IO_GPIO(io),IO_Pin(io),GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(IO_GPIO(io),IO_Pin(io),GPIO_PIN_RESET);
    }
#elif defined(STM32F4)
    if (hi) {
        IO_GPIO(io)->BSRRL = IO_Pin(io);
    }
    else {
        IO_GPIO(io)->BSRRH = IO_Pin(io);
    }
#else
    IO_GPIO(io)->BSRR = IO_Pin(io) << (hi ? 0 : 16);
#endif
}

void IOHi(IO_t io)
{
    if (!io)
        return;
#if defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io),IO_Pin(io),GPIO_PIN_SET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRL = IO_Pin(io);
#else
    IO_GPIO(io)->BSRR = IO_Pin(io);
#endif
}

void IOLo(IO_t io)
{
    if (!io)
        return;
#if defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io),IO_Pin(io),GPIO_PIN_RESET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRH = IO_Pin(io);
#else
    IO_GPIO(io)->BRR = IO_Pin(io);
#endif
}

void IOToggle(IO_t io)
{
    if (!io)
        return;
    uint32_t mask = IO_Pin(io);
    // Read pin state from ODR but write to BSRR because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.
#if defined(USE_HAL_DRIVER)
    (void)mask;
    HAL_GPIO_TogglePin(IO_GPIO(io),IO_Pin(io));
#elif defined(STM32F4)
    if (IO_GPIO(io)->ODR & mask) {
        IO_GPIO(io)->BSRRH = mask;
    } else {
        IO_GPIO(io)->BSRRL = mask;
    }
#else
    if (IO_GPIO(io)->ODR & mask)
        mask <<= 16;   // bit is set, shift mask to reset half

    IO_GPIO(io)->BSRR = mask;
#endif
}

// claim IO pin, set owner and resources
void IOInit(IO_t io, resourceOwner_t owner, resourceType_t resource, uint8_t index)
{
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = owner;
    ioRec->resource = resource;
    ioRec->index = index;
}

void IORelease(IO_t io)
{
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = OWNER_FREE;
}

resourceOwner_t IOGetOwner(IO_t io)
{
    ioRec_t *ioRec = IO_Rec(io);
    return ioRec->owner;
}

resourceType_t IOGetResource(IO_t io)
{
    ioRec_t *ioRec = IO_Rec(io);
    return ioRec->resource;
}

#if defined(STM32F1)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Speed = cfg & 0x03,
        .GPIO_Mode = cfg & 0x7c,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32F7)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x13,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = (cfg >> 5) & 0x03,
    };
    HAL_GPIO_Init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x13,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };
    HAL_GPIO_Init(IO_GPIO(io), &init);
}
#elif defined(STM32F3) || defined(STM32F4)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io)
        return;

    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}
#endif

static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void) {
    ioRec_t *ioRec = ioRecs;

    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
            if (ioDefUsedMask[port] & (1 << pin)) {
                ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                ioRec->pin = 1 << pin;
                ioRec++;
            }
        }
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    int portIdx = DEFIO_TAG_GPIOID(tag);
    int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx >= DEFIO_PORT_USED_COUNT)
        return NULL;
    // check if pin exists
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx)))
        return NULL;
    // count bits before this pin on single port
    int offset = __builtin_popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // and add port offset
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}
