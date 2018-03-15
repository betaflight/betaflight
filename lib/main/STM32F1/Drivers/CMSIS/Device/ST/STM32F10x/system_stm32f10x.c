#include <stdbool.h>
#include "stm32f10x.h"

#define SYSCLK_FREQ_72MHz  72000000

uint32_t SystemCoreClock = SYSCLK_FREQ_72MHz;   /*!< System Clock Frequency (Core Clock) */

static const uint8_t AHBPrescTable[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };

uint32_t hse_value = 8000000;

void SystemInit(void)
{
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set HSION bit */
    RCC->CR |= (uint32_t) 0x00000001;

    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    RCC->CFGR &= (uint32_t) 0xF8FF0000;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t) 0xFEF6FFFF;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t) 0xFFFBFFFF;

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= (uint32_t) 0xFF80FFFF;

    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x009F0000;

    SCB->VTOR = FLASH_BASE;     /* Vector Table Relocation in Internal FLASH. */
}

void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0, pllmull = 0, pllsource = 0;

    /* Get SYSCLK source ------------------------------------------------------- */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case 0x00:                 /* HSI used as system clock */
            SystemCoreClock = HSI_VALUE;
            break;
        case 0x04:                 /* HSE used as system clock */
            SystemCoreClock = hse_value;
            break;
        case 0x08:                 /* PLL used as system clock */

            /* Get PLL clock source and multiplication factor ---------------------- */
            pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

            pllmull = (pllmull >> 18) + 2;

            if (pllsource == 0x00) {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
            } else {
                /* HSE selected as PLL clock entry */
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t) RESET) {  /* HSE oscillator clock divided by 2 */
                    SystemCoreClock = (hse_value >> 1) * pllmull;
                } else {
                    SystemCoreClock = hse_value * pllmull;
                }
            }
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Compute HCLK clock frequency ---------------- */
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}

enum {
    SRC_NONE = 0,
    SRC_HSI,
    SRC_HSE
};

// Set system clock to 72 (HSE) or 64 (HSI) MHz
void SetSysClock(bool overclock)
{
    __IO uint32_t StartUpCounter = 0, status = 0, clocksrc = SRC_NONE;
    __IO uint32_t *RCC_CRH = &GPIOC->CRH;
    __IO uint32_t RCC_CFGR_PLLMUL = RCC_CFGR_PLLMULL9;

    // First, try running off HSE
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    RCC->APB2ENR |= RCC_CFGR_HPRE_0;

    // Wait till HSE is ready
    do {
        status = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    } while ((status == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
        // external xtal started up, we're good to go
        clocksrc = SRC_HSE;
    } else {
        // If HSE fails to start-up, try to enable HSI and configure for 64MHz operation
        RCC->CR |= ((uint32_t)RCC_CR_HSION);
        StartUpCounter = 0;
        do {
            status = RCC->CR & RCC_CR_HSIRDY;
            StartUpCounter++;
        } while ((status == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
        if ((RCC->CR & RCC_CR_HSIRDY) != RESET) {
            // we're on internal RC
            clocksrc = SRC_HSI;
        } else {
            while(1); // Unable to continue.
        }
    }

    // Enable Prefetch Buffer
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    // Flash 2 wait state
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
    // HCLK = SYSCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
    // PCLK2 = HCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    // PCLK1 = HCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
    *RCC_CRH &= (uint32_t)~((uint32_t)0xF << (RCC_CFGR_PLLMULL9 >> 16));

    // Configure PLL
    hse_value = 8000000;
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    *RCC_CRH |= (uint32_t)0x8 << (RCC_CFGR_PLLMULL9 >> 16);
    GPIOC->ODR &= (uint32_t)~(CAN_MCR_RESET);
	
#if defined(CJMCU)
    // On CJMCU new revision boards (Late 2014) bit 15 of GPIOC->IDR is '1'.
    RCC_CFGR_PLLMUL = RCC_CFGR_PLLMULL9;
#else
    RCC_CFGR_PLLMUL = GPIOC->IDR & GPIO_IDR_IDR15 ? hse_value = 12000000, RCC_CFGR_PLLMULL6 : RCC_CFGR_PLLMULL9;
#endif
    switch (clocksrc) {
        case SRC_HSE:
            if (overclock) {
                if (RCC_CFGR_PLLMUL == RCC_CFGR_PLLMULL6)
                    RCC_CFGR_PLLMUL = RCC_CFGR_PLLMULL7;
                else if (RCC_CFGR_PLLMUL == RCC_CFGR_PLLMULL9)
                    RCC_CFGR_PLLMUL = RCC_CFGR_PLLMULL10;
            }
            // overclock=false : PLL configuration: PLLCLK = HSE * 9 = 72 MHz || HSE * 6 = 72 MHz
            // overclock=true  : PLL configuration: PLLCLK = HSE * 10 = 80 MHz || HSE * 7 = 84 MHz
            RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL);
            break;
        case SRC_HSI:
            // PLL configuration: PLLCLK = HSI / 2 * 16 = 64 MHz
            RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL16);
            break;
    }

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    // Wait till PLL is ready
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    // Select PLL as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
    // Wait till PLL is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08);

    SystemCoreClockUpdate();
}
