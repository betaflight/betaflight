
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "bus_i2c.h"
#include "bus_spi.h"

#include "system.h"

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

#ifdef STM32F303xC
// from system_stm32f30x.c
void SetSysClock(void);
#endif
#ifdef STM32F10X_MD
// from system_stm32f10x.c
void SetSysClock(bool overclock);
#endif

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(bool overclock)
{

#ifdef STM32F303xC
    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));
#endif

    gpio_config_t gpio;

#ifdef STM32F303xC
    SetSysClock();
#endif
#ifdef STM32F10X_MD
    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock(overclock);
#endif

    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Turn on clocks for stuff we use
#ifdef STM32F10X_MD
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);
#endif
#ifdef STM32F303xC

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_TIM1 |
        RCC_APB2Periph_TIM8 |
#ifdef CHEBUZZF3
        RCC_APB2Periph_TIM15 |
#endif
        RCC_APB2Periph_TIM16 |
        RCC_APB2Periph_TIM17,
        ENABLE
    );
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOC |
        RCC_AHBPeriph_GPIOD |
#ifdef CHEBUZZF3
        RCC_AHBPeriph_GPIOF |
#endif
        RCC_AHBPeriph_GPIOE,
        ENABLE
    );
#endif

    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    gpio.mode = Mode_AIN;
    gpio.pin = Pin_All;
#ifdef STM32F303xC
    gpio.pin = Pin_All & ~(Pin_13|Pin_14|Pin_15);  // Leave JTAG pins alone
    gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_All;
#else
    gpioInit(GPIOA, &gpio);
#endif
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);
#ifdef STM32F303xC
    gpioInit(GPIOD, &gpio);
    gpioInit(GPIOE, &gpio);
#ifdef CHEBUZZF3
    gpioInit(GPIOF, &gpio);
#endif
#endif

#ifdef STM32F10X_MD
    // Turn off JTAG port 'cause we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
#endif

#ifdef STM32F303xC
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_6);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,  GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_4);
#ifdef CHEBUZZF3
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource9,  GPIO_AF_3);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_3);
#endif
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_2);
#endif

#ifdef NAZE32PRO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_6);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,  GPIO_AF_2);
#endif

    ledInit();
    beeperInit();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Configure the rest of the stuff
    i2cInit(I2C2);
    spiInit();

    // sleep for 100ms
    delay(100);
}

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

// FIXME replace mode with an enum so usage can be tracked, currently mode is a magic number
void failureMode(uint8_t mode)
{
    LED1_ON;
    LED0_OFF;
    while (1) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(475 * mode - 2);
        BEEP_ON
        delay(25);
        BEEP_OFF;
    }
}


