/**
  ******************************************************************************
  * @file    system_stm32g4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include <string.h>

#include "um324xx.h"
#include "drivers/system.h"
#include "platform.h"
#include "drivers/persistent.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE     8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    48000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

#define	SYSCLK_288M
#define VECT_TAB_SRAM

  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 48000000;

const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
const uint8_t APBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

void SystemClock_Config(void); // Forward
void QSPI_QuadEn(QSPI_HandleTypeDef *hqspi);
/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */


/*
 * QSPI pin bring-up, written register-level: the betaflight IO driver is not
 * available this early (SystemInit, before IOInitGlobal), and exFlashInit must
 * not call into code that may be linked above the QSPI window (0x00080000+),
 * because the window only comes alive at the end of this function.
 *
 * Register model (GPIO_TypeDef, um324xF.h), equivalent to HAL_GPIO_Init as
 * invoked by io_um32.c IOConfigGPIOAF():
 *   MODE  2bit/pin   : 00 = input, 10 = alternate function (MODE_AF)
 *   PULL  PE field [15:0] + PS field [31:16], 1bit/pin each; pull-up = PE|PS
 *   SR    1bit/pin   : 0 = high speed (GPIO_SPEED_FREQ_HIGH is 0 on this part)
 *   DS    2bit/pin   : drive strength, 14mA (same as io_um32.c)
 *   AFL/AFH 4bit/pin : alternate function number (AF10)
 */
#define QSPI_PIN_MODE_MASK(pos)    (0x3UL << ((pos) * 2))
#define QSPI_PIN_PULL_MASK(pos)    ((GPIO_PULL_PS_0 | GPIO_PULL_PE_0) << (pos))
#define QSPI_PIN_AFSEL_MASK(pos)   (0xFUL << (((pos) & 0x7U) * 4))

static inline __attribute__((always_inline)) void qspiPinAF(GPIO_TypeDef *port, uint32_t pos)
{
    const uint32_t shift2 = pos * 2;
    const uint32_t shift4 = (pos & 0x7U) * 4;

    port->MODE = (port->MODE & ~QSPI_PIN_MODE_MASK(pos)) | ((uint32_t)MODE_AF << shift2);
    port->PULL = port->PULL | QSPI_PIN_PULL_MASK(pos);                    /* pull-up */
    port->SR   = port->SR & ~(GPIO_SR_SR_0 << pos);                       /* 0 = high speed */
    port->DS   = (port->DS & ~(GPIO_DS_20MA << (pos * 2))) | (GPIO_DS_14MA << (pos * 2));

    if (pos < 8U) {
        port->AFL = (port->AFL & ~QSPI_PIN_AFSEL_MASK(pos)) | ((uint32_t)GPIO_AF10_QSPI << shift4);
    } else {
        port->AFH = (port->AFH & ~QSPI_PIN_AFSEL_MASK(pos)) | ((uint32_t)GPIO_AF10_QSPI << shift4);
    }

    /* input path config, cleared like HAL_GPIO_Init's default (Gpio_Im = 0) */
    port->IM = port->IM & ~(GPIO_IM_IM_0 << pos);
}

static inline __attribute__((always_inline)) void qspiPinInputFloating(GPIO_TypeDef *port, uint32_t pos)
{
    port->MODE = port->MODE & ~QSPI_PIN_MODE_MASK(pos);                   /* 00 = input */
    port->PULL = port->PULL & ~QSPI_PIN_PULL_MASK(pos);                   /* no pull */
}

void exFlashInit(void)
{
    /* Port D/E clocks (macros are plain register writes incl. RCM unlock) */
    __HAL_RCM_GPIOD_CLK_ENABLE();
    __HAL_RCM_GPIOE_CLK_ENABLE();

    /* free pins that used to share the QSPI footprint */
    // qspiPinInputFloating(GPIOE, 5);
    // qspiPinInputFloating(GPIOE, 7);
    // qspiPinInputFloating(GPIOD, 8);

    /* PE10=CLK, PD3=BK1CS, PD4/5/6=BK1 IO0/1/2, PE15=BK1 IO3, all AF10 pull-up */
    qspiPinAF(GPIOE, 10);
    qspiPinAF(GPIOD, 3);
    qspiPinAF(GPIOD, 4);
    qspiPinAF(GPIOD, 5);
    qspiPinAF(GPIOD, 6);
    qspiPinAF(GPIOE, 15);

    __HAL_RCM_QSPI_CLK_ENABLE();
    __HAL_RCM_QSPI_RELEASE_RESET();

    QSPI_HandleTypeDef hqspi = {0};

	hqspi.Instance = QSPI;
	hqspi.DataSize                 = QSPI_DATASIZE_8BIT; 
    hqspi.Init.CSInvalidDelay      = 0x5; 
    hqspi.Init.CSStartDelay        = 0x0; 
    hqspi.Init.CSStopDelay         = 0x5; 
    
    hqspi.Init.AddrSizes           = QSPI_ADDRBYTES_3; 
    hqspi.Init.PageSizes           = 256;
    hqspi.Init.BlockSizes          = 16; // 2^16B = 64KB earch Block
    
    hqspi.Init.WorkMode            = QSPI_WORKMODE_DAC; 
    hqspi.Init.ClockPrescaler      = QSPI_BRDIV_6; 

    hqspi.Init.ClockMode           = QSPI_CLOCK_MODE_0; 

    /* Setting the QSPI_RDCR register  */
    hqspi.Init.ReadDelay           = QSPI_DLYR_4; 
    hqspi.Init.TransDelay          = QSPI_DLYT_0; 
    hqspi.Init.Sampling_Edge       = QSPI_SMES_TRAILING; 
    
    /* Congratulate the read data in the dac mode */
    hqspi.DacMode.ReadCommand      = 0xEB; 
    hqspi.DacMode.ReadAddr_Type    = QSPI_ADMODE_DQ0DQ1DQ2DQ3; 
    hqspi.DacMode.ReadData_Type    = QSPI_DMODE_QUAD;	
    hqspi.DacMode.ReadData_Dummy   = QSPI_DUMMY_CLKS_6;

    /* Congratulate the write data in the dac mode */
    hqspi.DacMode.WriteCommand     = 0x32; 
    hqspi.DacMode.WriteAddr_Type   = QSPI_ADMODE_DQ0; 
    hqspi.DacMode.WriteData_Type   = QSPI_DMODE_QUAD; 
    hqspi.DacMode.WriteData_Dummy  = QSPI_DUMMY_CLKS_0;

    /* HAL_QSPI_Init() waits for the controller IDLE flag using hqspi.Timeout;
     * with Timeout == 0 (from the {0} initialiser) the wait fails on its very
     * first loop iteration and drops into Error_Handler() ¡ª this is what froze
     * the boot LED at checkpoint 2. */
    hqspi.Timeout = 100;

    if (HAL_QSPI_Init(&hqspi) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /* Enable the QE mode */
    QSPI_QuadEn(&hqspi);

    /* Enable qspi cache  */
    (*(volatile uint32_t *)(0x3cfffc00)) |= 0x03;
}

static void initialiseDmaMemorySections(void)
{
    extern uint8_t _sdmaram_bss;
    extern uint8_t _edmaram_bss;
    extern uint8_t _sdmaram_data;
    extern uint8_t _edmaram_data;
    extern uint8_t _sdmaram_idata;
    bzero(&_sdmaram_bss, (size_t) (&_edmaram_bss - &_sdmaram_bss));
    memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t) (&_edmaram_data - &_sdmaram_data));
}

void SystemInit(void)
{
    // initialiseMemorySections();
    
	/* enable Access PMU register*/
	PMU->CPR  = 0xABCD;
	/* Set PDR(rising 1.87V/falling 1.77V),BOR(2.43V)*/
	PMU->VDCR  = 0x0000A983;
	/* not reset BOR configure and pmu state*/
	PMU->SASR = 0x00000025;
	
	/* enable Access RCM register*/
	RCM->RCMPR = 0xA5A55A5A;
	/* enable reset filter*/
	RCM->EXRSTFER |= (1<<0);
	
    /* FPU settings ---------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
#endif

#if(0)
	  /* Query Method Delay Initialize */
	delay_init(HAL_RCM_GetHCLKFreq());
#endif	
	/*boost configer*/
	(*(volatile uint32_t *)(0x400801c0)) = 0x77BBBB77;
	(*(volatile uint32_t *)(0x40080000)) = 0xA5A55A5A;
	(*(volatile uint32_t *)(0x40080038)) = (((*(volatile uint32_t *)(0x40080038)) & 0xFFFFFFF0) | 0xA);
	(*(volatile uint32_t *)(0x400801c0)) = 0x00000000;
  (*(volatile uint32_t *)(0x40080000)) = 0xFFFFFFFF;

#if(0)
	delay(1);
#else
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#endif
	
    /* Reset the RCM clock configuration to the default reset state ------------*/
    /* Unlock protection register */   
    RCM->RCMPR = 0xA5A55A5A;
    /* Set RCH_EN bit */
    RCM->CR0 |= (uint32_t)0x00000001;
    /* Reset RCM_CFGR0 register */
    RCM->CFGR0 = 0x09008040;
    /* Reset RCM_CFGR1 register */
    RCM->CFGR1 = 0xFF062002;

    /* Reset XTH_MEN, XTH_EN and PLL0EN bits */
    RCM->CR0 &= (uint32_t)0xFEFEFFFD;

    /* Reset RCM_PLL0CFGR0,RCM_PLL0CFGR1,RCM_PLL0CFGR2 register */
    RCM->PLL0CFGR0 = 0x00010388;
    RCM->PLL0CFGR1 = 0x00000000;
    RCM->PLL0CFGR2 = 0x08000000;

    /* Reset XTH_BYP bit */
    RCM->CR0 &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCM->CIER = 0x00000000;
    /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  // Copy vector table from isr_vector_table_flash_base to isr_vector_table_base.
  // If these two regions are the same, the copy will have no effect
  // (Happens when linker script aliases VECTAB to FLASH).

    extern uint8_t isr_vector_table_flash_base;
    extern uint8_t isr_vector_table_base;
    extern uint8_t isr_vector_table_end;

    memcpy(&isr_vector_table_base, &isr_vector_table_flash_base, &isr_vector_table_end - &isr_vector_table_base);
    SCB->VTOR = (uint32_t)&isr_vector_table_base;
#else
    extern uint8_t isr_vector_table_flash_base;
    SCB->VTOR = (uint32_t)&isr_vector_table_flash_base;
#endif

#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    SystemClock_Config();
    
    SystemCoreClockUpdate();

    exFlashInit();

    initialiseMemorySections();

    // Copy .dmaram_data from flash and zero .dmaram_bss before anything
    // that might touch DMA_DATA-placed variables (the standard Reset_Handler
    // .data/.bss copy/clear loop doesn't cover these sections).
    initialiseDmaMemorySections();
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (**) HSI_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32g4xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
    uint32_t cfgr0 = 0U;

	/* Get SYSCLK source -------------------------------------------------------*/
	tmp = ((RCM->CFGR0 & RCM_CFGR0_SYS_SWS)>>2);

	switch (tmp)
	{
		case 0x00:  /* RCH used as system clock source */
			SystemCoreClock = (RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV) >> RCM_CFGR0_RCH_DIV_Pos) + 1));
			break;
		case 0x01:  /* XTH used as system clock source */
			SystemCoreClock = XTH_VALUE;
			break;
		case 0x02:  /* PLL0 used as system clock source */
			
			/* 
				PLL_VCO = (RCH_VALUE or XTH_VALUE / PLL_M) * PLL_N
				SYSCLK = PLL_VCO / PLL_P
			*/   		
			pllsource = (RCM->CR0 & RCM_CR0_PLLSRC) >> RCM_CR0_PLLSRC_Pos;		
			pllm = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DM) >> RCM_PLL0CFGR0_PLL0_DM_Pos);
		
			if (pllsource != 0)
			{
				/* XTH used as PLL0 clock source */
				pllvco = (XTH_VALUE / pllm) * ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos);
			}
			else
			{
                cfgr0 = RCM->CFGR0;
				/* RCH used as PLL0 clock source */
				pllvco = (((RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV) >> RCM_CFGR0_RCH_DIV_Pos) + 1)) / pllm) * ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos))>>(AHBPrescTable[(cfgr0 & RCM_CFGR0_AHB_DIV)>> RCM_CFGR0_AHB_DIV_Pos]);
			}		
		
			pllp = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DP) >> RCM_PLL0CFGR0_PLL0_DP_Pos);
			
			SystemCoreClock = (pllvco/pllp);
			break;
		case 0x03:  /* RCL or XTL used as system clock source */
			if(READ_BIT(PMU->XTLCR, PMU_XTLCR_LSCLK_SEL))
			{
				/* XTL used as Low Speed clock source */
				SystemCoreClock = XTL_VALUE;
			}		
			else
			{
				/* RCL used as Low Speed clock source */
				SystemCoreClock = RCL_VALUE;
			}
			break;
		default:
			SystemCoreClock = RCH_VALUE;
		break;
	}
	/* Compute HCLK frequency --------------------------------------------------*/
	/* Get HCLK prescaler */
	tmp = AHBPrescTable[((RCM->CFGR0 & RCM_CFGR0_AHB_DIV) >> RCM_CFGR0_AHB_DIV_Pos)];
	
    /* HCLK frequency */
	SystemCoreClock >>= tmp;	
}

void Error_Handler(void)
{
    while (1) {
    }
}


// SystemSYSCLKSource
// 0: HSI
// 1; HSE
// 2: PLL0

int SystemSYSCLKSource(void)
{
    switch (__HAL_RCM_GET_SYSCLK_SOURCE()) {
    case RCM_CFGR0_SYS_SWS_XTH:
        return 1;   // HSE
    case RCM_CFGR0_SYS_SWS_PLL0:
        return 2;   // PLL
    case RCM_CFGR0_SYS_SWS_RCH:
    default:
        return 0;   // HSI
    }
}

// SystemPLLSource
//   0: HSI
//   1: HSE

int SystemPLLSource(void)
{
    switch (__HAL_RCM_GET_PLL_OSCSOURCE()) {
    case RCM_CR0_PLLSRC_XTH:
        return 1;   // HSE
    case RCM_CR0_PLLSRC_RCH:
    default:
        return 0;   // HSI
    }
}

typedef struct pllConfig_s {
  uint16_t mhz; // target SYSCLK
  uint16_t n;
  uint16_t p;
  uint16_t q;   //USB_SDIO_DIV
  uint16_t r;   //AHB_DIV
} pllConfig_t;

// PLL parameters for PLL input = 1MHz.
// For PLL input = 2MHz, divide n by 2; see SystemInitPLLParameters below.

static const pllConfig_t overclockLevels[] = {
  { 288, 576, 2,  USB_SDIO_Div6, AHB_Div1 },  // 288 MHz
  { 336, 336, 1,  USB_SDIO_Div7, AHB_Div1 },  // 336 MHz
};

void OverclockRebootIfNecessary(uint32_t targetMhz)
{
  // targetMhz == 0 means "OFF" / use default (first entry)
  if (targetMhz == 0) {
    targetMhz = overclockLevels[0].mhz;
  }

  for (unsigned i = 0; i < ARRAYLEN(overclockLevels); i++) {
    if (overclockLevels[i].mhz == targetMhz) {
      // Reboot to adjust overclock frequency
      if (SystemCoreClock != targetMhz * 1000000U) {
        persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, i);
        __disable_irq();
        NVIC_SystemReset();
      }
      return;
    }
  }
}

void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);
        __disable_irq();
        NVIC_SystemReset();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// Extracted from MX generated main.c 

void SystemClock_Config(void)
{
    RCM_ClkInitTypeDef RCM_ClkInitStruct;
    RCM_OscInitTypeDef RCM_OscInitStruct;

#ifdef USE_OVERCLOCK
    uint32_t currentOverclockLevel = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);
    if (currentOverclockLevel >= ARRAYLEN(overclockLevels)) {
        currentOverclockLevel = 0;
    }
#endif
    if(SYSCLK_SRC == SYSCLK_USE_RCH)
    {
        /* Enable RCH Oscillator as source */
        RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_RCH;
        RCM_OscInitStruct.RCHState = RCM_RCH_ON;

        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
        /* Select RCH as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_RCH | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3);
        RCM_ClkInitStruct.RCHDivider = RCM_RCH_DIV1;  
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_RCH;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = RCH/RCM_SYSCLK_DIV1 = 48Mhz/1 = 48MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV1;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV1 = 48MHz/1 = 48MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 48MHz/1 = 48MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 48MHz/1 = 48MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV1;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV1 = 48MHz/1 = 48MHz
        
        /*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_0);      
    }
    else if(SYSCLK_SRC == SYSCLK_USE_RCH_PLL)
    {
     //   note:  1.Fxxx=Ref ck*DN/DM >300MHz   2.DN>=16
        
        /* Enable XTH Oscillator and activate PLL with HSE as source */
        RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_RCH;
        RCM_OscInitStruct.RCHState = RCM_RCH_ON;
        RCM_OscInitStruct.PLL.PLLState = RCM_PLL_ON;
        RCM_OscInitStruct.PLL.PLLSource = RCM_PLLSOURCE_RCH;

#ifdef SYSCLK_168M
        RCM_OscInitStruct.PLL.PLLM = 2;
        RCM_OscInitStruct.PLL.PLLN = 28;
        RCM_OscInitStruct.PLL.PLLP = 1;                          //RCH=24M  ,PLL CLKOUT(Fclk)=336Mhz, Fref= RCH = 24M , Fclk=(Fref*DN)/(DP*DM)
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_RCH | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3);
        RCM_ClkInitStruct.RCHDivider = RCM_RCH_DIV2;     		
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV2;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV2 = 336Mhz/2 = 168MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV4;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV4 = 168MHz/4 = 42MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 168MHz/1 = 168MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 168MHz/1 = 168MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV4;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV4 = 168MHz/4 = 42MHz
		
		/*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_3); 
#endif

#ifdef SYSCLK_204M
        RCM_OscInitStruct.PLL.PLLM = 2;
        RCM_OscInitStruct.PLL.PLLN = 17;
        RCM_OscInitStruct.PLL.PLLP = 1;                          //RCH=24M  ,PLL CLKOUT(Fclk)=204Mhz, Fref= RCH = 24M , Fclk=(Fref*DN)/(DP*DM)
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_RCH | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3);
        RCM_ClkInitStruct.RCHDivider = RCM_RCH_DIV2;     		
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV1 = 204Mhz/1 = 204MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV8;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV8 = 204MHz/8 = 25.5MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 204MHz/1 = 204MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 204MHz/1 = 204MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV8;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV8 = 204MHz/8 = 25.5MHz
		
		/*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_4); 
#endif

#ifdef SYSCLK_240M
        RCM_OscInitStruct.PLL.PLLM = 2;
        RCM_OscInitStruct.PLL.PLLN = 40;
        RCM_OscInitStruct.PLL.PLLP = 2;                          //RCH=24M  ,PLL CLKOUT(Fclk)=240Mhz, Fref= RCH = 24M , Fclk=(Fref*DN)/(DP*DM)
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_RCH | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3);
        RCM_ClkInitStruct.RCHDivider = RCM_RCH_DIV2;     		
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV1 = 240Mhz/1 = 240MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV8;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV8;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV8 = 240MHz/8 = 30MHz
		
		/*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_4); 
#endif
 
    }
    else if(SYSCLK_SRC == SYSCLK_USE_XTH)
    {
        /* Enable XTH Oscillator and activate PLL with HSE as source */
        RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_XTH;
        RCM_OscInitStruct.XTHState = RCM_XTH_ON;

        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
        /* Select XTH as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3);
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_XTH;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = XTH/RCM_SYSCLK_DIV1 = 12Mhz/1 = 12MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV1;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV1 = 12MHz/1 = 12MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 12MHz/1 = 12MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 12MHz/1 = 12MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV1;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV1 = 12MHz/1 = 12MHz
        
        /*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_0);      
    }
    else if(SYSCLK_SRC == SYSCLK_USE_XTH_PLL)
    {
        /* Enable XTH Oscillator and activate PLL with HSE as source */
        RCM_OscInitStruct.OscillatorType = RCM_OSCILLATORTYPE_XTH;
        RCM_OscInitStruct.XTHState = RCM_XTH_ON;
        RCM_OscInitStruct.PLL.PLLState = RCM_PLL_ON;
        RCM_OscInitStruct.PLL.PLLSource = RCM_PLLSOURCE_XTH;
		
#ifdef USE_OVERCLOCK
        if(currentOverclockLevel == 0){ //XTH=8M  ,PLL CLKOUT(Fclk)=240Mhz, Fref= XTH = 12M , Fclk=(Fref*DN)/(DP*DM)
            RCM_OscInitStruct.PLL.PLLM = 2;
            RCM_OscInitStruct.PLL.PLLN = 72;
            RCM_OscInitStruct.PLL.PLLP = 1;	
        }else{
            RCM_OscInitStruct.PLL.PLLM = 2;
            RCM_OscInitStruct.PLL.PLLN = 84;
            RCM_OscInitStruct.PLL.PLLP = 1;	//XTH=8M  ,PLL CLKOUT(Fclk)=240Mhz, Fref= XTH = 12M , Fclk=(Fref*DN)/(DP*DM)
        }
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3 | RCM_CLOCKTYPE_USBSDIO);
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV1 = 240Mhz/1 = 240MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV8;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV8;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV8 = 240MHz/8 = 30MHz
        
        if(currentOverclockLevel == 0){
            RCM_ClkInitStruct.USBSDIOCLKDivider = RCM_USB_SDIO_DIV6;//240MHz/5 = 48MHz	
            /*Initializes the CPU, AHB and APB busses clocks */
            HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_5); 
        }else{
            RCM_ClkInitStruct.USBSDIOCLKDivider = RCM_USB_SDIO_DIV7;//240MHz/5 = 48MHz	
            /*Initializes the CPU, AHB and APB busses clocks */
            HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_6); 
        }
#else
#ifdef SYSCLK_288M
		RCM_OscInitStruct.PLL.PLLM = 2;
        RCM_OscInitStruct.PLL.PLLN = 72;
        RCM_OscInitStruct.PLL.PLLP = 1;							//XTH=12M  ,PLL CLKOUT(Fclk)=240Mhz, Fref= XTH = 12M , Fclk=(Fref*DN)/(DP*DM)
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3 | RCM_CLOCKTYPE_USBSDIO);
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV1 = 240Mhz/1 = 240MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV8;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV8;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.USBSDIOCLKDivider = RCM_USB_SDIO_DIV6;//240MHz/5 = 48MHz	
		
		/*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_5); 
#endif
  
#ifdef SYSCLK_336M
		RCM_OscInitStruct.PLL.PLLM = 2;
        RCM_OscInitStruct.PLL.PLLN = 84;
        RCM_OscInitStruct.PLL.PLLP = 1;							//XTH=12M  ,PLL CLKOUT(Fclk)=240Mhz, Fref= XTH = 12M , Fclk=(Fref*DN)/(DP*DM)
         
        /*Initializes the RCM Oscillators*/
        HAL_RCM_OscConfig(&RCM_OscInitStruct);   
        
         /* Select PLL as system clock source and configure the HCLK, PCLK0, PCLK1 ,PCLK2, and PCLK3
         clocks dividers */
        RCM_ClkInitStruct.ClockType = (RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK0 |RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2| RCM_CLOCKTYPE_PCLK3 | RCM_CLOCKTYPE_USBSDIO);
        RCM_ClkInitStruct.SYSCLKSource = RCM_SYSCLKSOURCE_PLL0CLK;
        RCM_ClkInitStruct.AHBCLKDivider = RCM_SYSCLK_DIV1;		//SystemCoreClock = Fclk/RCM_SYSCLK_DIV1 = 240Mhz/1 = 240MHz
        RCM_ClkInitStruct.APB0CLKDivider = RCM_PCLK0_DIV8;		//APB0Clock = SystemCoreClock/RCM_PCLK0_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.APB1CLKDivider = RCM_PCLK1_DIV1;		//APB1Clock = SystemCoreClock/RCM_PCLK1_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB2CLKDivider = RCM_PCLK2_DIV1;		//APB2Clock = SystemCoreClock/RCM_PCLK2_DIV1 = 240MHz/1 = 240MHz
        RCM_ClkInitStruct.APB3CLKDivider = RCM_PCLK3_DIV8;		//APB3Clock = SystemCoreClock/RCM_PCLK3_DIV8 = 240MHz/8 = 30MHz
        RCM_ClkInitStruct.USBSDIOCLKDivider = RCM_USB_SDIO_DIV7;//240MHz/5 = 48MHz	
		
		/*Initializes the CPU, AHB and APB busses clocks */
        HAL_RCM_ClockConfig(&RCM_ClkInitStruct, FLASH_RWAITCYC_6); 
#endif
#endif
    }  
}
