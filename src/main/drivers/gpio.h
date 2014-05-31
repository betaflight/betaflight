#pragma once

#ifdef STM32F10X_MD
typedef enum
{
    Mode_AIN = 0x0,
    Mode_IN_FLOATING = 0x04,
    Mode_IPD = 0x28,
    Mode_IPU = 0x48,
    Mode_Out_OD = 0x14,
    Mode_Out_PP = 0x10,
    Mode_AF_OD = 0x1C,
    Mode_AF_PP = 0x18
} GPIO_Mode;
#endif

#ifdef STM32F303xC

/*
typedef enum
{
  GPIO_Mode_IN   = 0x00, // GPIO Input Mode
  GPIO_Mode_OUT  = 0x01, // GPIO Output Mode
  GPIO_Mode_AF   = 0x02, // GPIO Alternate function Mode
  GPIO_Mode_AN   = 0x03  // GPIO Analog In/Out Mode
}GPIOMode_TypeDef;

typedef enum
{
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;
*/

typedef enum
{
    Mode_AIN =          (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_AN,
    Mode_IN_FLOATING =  (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_IN,
    Mode_IPD =          (GPIO_PuPd_DOWN   << 2) | GPIO_Mode_IN,
    Mode_IPU =          (GPIO_PuPd_UP     << 2) | GPIO_Mode_IN,
    Mode_Out_OD =       (GPIO_OType_OD << 4) | GPIO_Mode_OUT,
    Mode_Out_PP =       (GPIO_OType_PP << 4) | GPIO_Mode_OUT,
    Mode_AF_OD =        (GPIO_OType_OD << 4) | GPIO_Mode_AF,
    Mode_AF_PP =        (GPIO_OType_PP << 4) | GPIO_Mode_AF,
    Mode_AF_PP_PD =     (GPIO_OType_PP << 4) | (GPIO_PuPd_DOWN  << 2) | GPIO_Mode_AF,
    Mode_AF_PP_PU =     (GPIO_OType_PP << 4) | (GPIO_PuPd_UP    << 2) | GPIO_Mode_AF
} GPIO_Mode;
#endif

typedef enum
{
    Speed_10MHz = 1,
    Speed_2MHz,
    Speed_50MHz
} GPIO_Speed;

typedef enum
{
    Pin_0 = 0x0001,
    Pin_1 = 0x0002,
    Pin_2 = 0x0004,
    Pin_3 = 0x0008,
    Pin_4 = 0x0010,
    Pin_5 = 0x0020,
    Pin_6 = 0x0040,
    Pin_7 = 0x0080,
    Pin_8 = 0x0100,
    Pin_9 = 0x0200,
    Pin_10 = 0x0400,
    Pin_11 = 0x0800,
    Pin_12 = 0x1000,
    Pin_13 = 0x2000,
    Pin_14 = 0x4000,
    Pin_15 = 0x8000,
    Pin_All = 0xFFFF
} GPIO_Pin;

typedef struct
{
    uint16_t pin;
    GPIO_Mode mode;
    GPIO_Speed speed;
} gpio_config_t;

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }
#define digitalIn(p, i)     (p->IDR & i)

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config);
void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc);
void gpioPinRemapConfig(uint32_t remap, bool enable);
