#include "board.h"

// Cycle counter stuff - these should be defined by CMSIS, but they aren't
#define DWT_CTRL	(*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT	((volatile uint32_t *)0xE0001004)
#define CYCCNTENA	(1 << 0)

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;

static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000;

    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// enable the CPU cycle counter
	DWT_CTRL |= CYCCNTENA;
}


// SysTick
void SysTick_Handler(void)
{
    sysTickCycleCounter = *DWT_CYCCNT;
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;
    __disable_irq();
    cycle = *DWT_CYCCNT;
    oldCycle = sysTickCycleCounter;
    timeMs = sysTickUptime;
    __enable_irq();
    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Turn on clocks for stuff we use
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // Turn off JTAG port 'cause we're using the GPIO for leds
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // Configure gpio
    
    // PB3, PB4 (LEDs)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    LED0_OFF;
    LED1_OFF;

    // PA12 (Buzzer)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    BEEP_OFF;

    // Init cycle counter
    cycleCounterInit();
    
    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Configure the rest of the stuff
    adcInit();
    i2cInit(I2C2);
    uartInit();
    
    // sleep for 100ms
    delay(100);
}

void delayMicroseconds(uint32_t us)
{
	uint32_t elapsed = 0;
	uint32_t lastCount = *DWT_CYCCNT;

	for (;;) {
		register uint32_t current_count = *DWT_CYCCNT;
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

void delay(uint32_t ms)
{
	while (ms--)
		delayMicroseconds(1000);
}

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

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }        

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}
