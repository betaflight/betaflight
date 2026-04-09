/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_rcc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_rcc.h"

/* RCC registers bit mask */

/* CTLR register bit mask */
#define CTLR_HSEBYP_Reset          ((uint32_t)0xFFFBFFFF)
#define CTLR_HSEBYP_Set            ((uint32_t)0x00040000)
#define CTLR_HSEON_Reset           ((uint32_t)0xFFFEFFFF)
#define CTLR_HSEON_Set             ((uint32_t)0x00010000)
#define CTLR_HSITRIM_Mask          ((uint32_t)0xFFFFFF07)

/* CFGR0 register bit mask */
#define CFGR0_SWS_Mask             ((uint32_t)0x0000000C)
#define CFGR0_SW_Mask              ((uint32_t)0xFFFFFFFC)
#define CFGR0_HPRE_Set_Mask        ((uint32_t)0x000000F0)

/* RSTSCKR register bit mask */
#define RSTSCKR_RMVF_Set           ((uint32_t)0x01000000)

/* RCC Flag Mask */
#define FLAG_Mask                  ((uint8_t)0x1F)

static __I uint8_t PLLMULTable[32] = {4,6,7,8,17,9,19,10,21,11,23,12,25,13,14,15,16,17,18,19,20,22,24,26,28,30,32,34,36,38,40,59};
static __I uint8_t HBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t FPRETable[4] = {0, 1, 2, 2};
static __I uint8_t PPRE2Table[8] = {0, 0, 0, 0, 1, 2, 3, 4};
static __I uint8_t ADCPRETable[4] = {2, 4, 6, 8};

/*********************************************************************
 * @fn      RCC_DeInit
 *
 * @brief   Resets the RCC clock configuration to the default reset state.
 *        Note-
 *          HSE can not be stopped if it is used directly or through the PLL as system clock.
 *
 * @return  none
 */
void RCC_DeInit(void)
{
    RCC->CTLR |= (uint32_t)0x00000001;
    RCC->CFGR0 &= (uint32_t)0x305C0000;
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
    {
    }
    RCC->CFGR0 &= (uint32_t)0xFFBFFFFF;

    RCC->PLLCFGR &= (uint32_t)0x7FFFFFFF;

    RCC->CTLR &= (uint32_t)0x6AA6FFFF;
    RCC->CTLR &= (uint32_t)0xFFFBFFFF;

    
    RCC->PLLCFGR &= (uint32_t)0x0FFFC000;
    RCC->PLLCFGR |= (uint32_t)0x00000004;

    RCC->INTR = 0x00FF0000;
    RCC->CFGR2 &= 0x0C600000;
    RCC->PLLCFGR2 &= 0xFFF0E080;
    RCC->PLLCFGR2 |= 0x00080020;
}

/*********************************************************************
 * @fn      RCC_HSEConfig
 *
 * @brief   Configures the External High Speed oscillator (HSE).
 *
 * @param   RCC_HSE -
 *            RCC_HSE_OFF - HSE oscillator OFF.
 *            RCC_HSE_ON - HSE oscillator ON.
 *            RCC_HSE_Bypass - HSE oscillator bypassed with external clock.
 *          Note-
 *            HSE can not be stopped if it is used directly or through the PLL as system clock.
 * @return  none
 */
void RCC_HSEConfig(uint32_t RCC_HSE)
{
    RCC->CTLR &= CTLR_HSEON_Reset;
    RCC->CTLR &= CTLR_HSEBYP_Reset;

    switch(RCC_HSE)
    {
        case RCC_HSE_ON:
            RCC->CTLR |= CTLR_HSEON_Set;
            break;

        case RCC_HSE_Bypass:
            RCC->CTLR |= CTLR_HSEBYP_Set | CTLR_HSEON_Set;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      RCC_WaitForHSEStartUp
 *
 * @brief   Waits for HSE start-up.
 *
 * @return  READY - HSE oscillator is stable and ready to use.
 *          NoREADY - HSE oscillator not yet ready.
 */
ErrorStatus RCC_WaitForHSEStartUp(void)
{
    __IO uint32_t StartUpCounter = 0;

    ErrorStatus status = NoREADY;
    FlagStatus  HSEStatus = RESET;

    do
    {
        HSEStatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
        StartUpCounter++;
    } while((StartUpCounter != HSE_STARTUP_TIMEOUT) && (HSEStatus == RESET));

    if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    {
        status = READY;
    }
    else
    {
        status = NoREADY;
    }

    return (status);
}

/*********************************************************************
 * @fn      RCC_AdjustHSICalibrationValue
 *
 * @brief   Adjusts the Internal High Speed oscillator (HSI) calibration value.
 *
 * @param   HSICalibrationValue - specifies the calibration trimming value.
 *                    This parameter must be a number between 0 and 0x1F.
 *
 * @return  none
 */
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CTLR;
    tmpreg &= CTLR_HSITRIM_Mask;
    tmpreg |= (uint32_t)HSICalibrationValue << 3;
    RCC->CTLR = tmpreg;
}

/*********************************************************************
 * @fn      RCC_HSICmd
 *
 * @brief   Enables or disables the Internal High Speed oscillator (HSI).
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HSICmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= (1 << 0);
    }
    else
    {
        RCC->CTLR &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      RCC_PLLConfig
 *
 * @brief   Configures the PLL clock source and multiplication and divider factor.
 *
 * @param   RCC_PLLSource - specifies the PLL entry clock source.
 *            RCC_PLLSource_HSI - HSI clock selected as PLL clock entry.
 *            RCC_PLLSource_HSE - HSE clock selected as PLL clock entry.
 *            RCC_PLLSource_USBHS(480M) - USBHS clock selected as PLL clock entry.
 *            RCC_PLLSource_ETH(500MHz) - ETH clock selected as PLL clock entry.
 *            RCC_PLLSource_USBSS(125MHz) - USBSS clock selected as PLL clock entry.
 *            RCC_PLLSource_SERDES(500MHz) - SERDES clock selected as PLL clock entry.
 *
 *          RCC_PLLMul - specifies the PLL multiplication factor.
 *            This parameter can be RCC_PLLMul_x where x:[4,59].
 *          RCC_PLLDiv - specifies the PLL Division factor.
 *            This parameter can be RCC_PLLDiv_x where x:[1,64].
 *          Should close PLLON before use this function
 *
 * @return  none
 */
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLDiv, uint32_t RCC_PLLMul)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->PLLCFGR;
    tmpreg &= ~0x3FFF;
    tmpreg |= RCC_PLLSource | RCC_PLLDiv | RCC_PLLMul;

    RCC->PLLCFGR = tmpreg;
}

/*********************************************************************
 * @fn      RCC_PLLCmd
 *
 * @brief   Enables or disables the PLL.
 *          Note-The PLL can not be disabled if it is used as system clock.
 *          
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= (1 << 24);
    }
    else
    {
        RCC->CTLR &= ~(1 << 24);
    }
}

/*********************************************************************
 * @fn      RCC_SYSCLKConfig
 *
 * @brief   Configures the system clock (SYSCLK).
 *
 * @param   RCC_SYSCLKSource - specifies the clock source used as system clock.
 *            RCC_SYSCLKSource_HSI - HSI selected as system clock.
 *            RCC_SYSCLKSource_HSE - HSE selected as system clock.
 *            RCC_SYSCLKSource_PLLCLK - PLL selected as system clock.
 *
 * @return  none
 */
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_SW_Mask;
    tmpreg |= RCC_SYSCLKSource;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_GetSYSCLKSource
 *
 * @brief   Returns the clock source used as system clock.
 *
 * @return  0x00 - HSI used as system clock.
 *          0x04 - HSE used as system clock.
 *          0x08 - PLL used as system clock.
 */
uint8_t RCC_GetSYSCLKSource(void)
{
    return ((uint8_t)(RCC->CFGR0 & CFGR0_SWS_Mask));
}

/*********************************************************************
 * @fn      RCC_HCLKSourceDivConfig
 *
 * @brief   Configures the HCLK clock .
 *
 * @param   RCC_SYSCLK - defines the HB clock divider. This clock is derived from
 *        the system clock (SYSCLK).
 *            RCC_SYSCLK_Div1 - Core1 clock = SYSCLK.
 *            RCC_SYSCLK_Div2 - Core1 clock = SYSCLK/2.
 *            RCC_SYSCLK_Div4 - Core1 clock = SYSCLK/4.
 *            RCC_SYSCLK_Div8 - Core1 clock = SYSCLK/8.
 *            RCC_SYSCLK_Div16 - Core1 clock = SYSCLK/16.
 *            RCC_SYSCLK_Div64 - Core1  clock = SYSCLK/64.
 *            RCC_SYSCLK_Div128 - Core1  clock = SYSCLK/128.
 *            RCC_SYSCLK_Div256 - Core1  clock = SYSCLK/256.
 *            RCC_SYSCLK_Div512 - Core1  clock = SYSCLK/512.
 *          RCC_SYSCLKFPRE - defines the FPRE clock divider. This clock is derived from
 *        the Core1 clock.
 *            RCC_SYSCLKFPRE_Div1 - HCLK clock = Core1 clock/1.
 *            RCC_SYSCLKFPRE_Div2 - HCLK clock = Core1 clock/2.
 *            RCC_SYSCLKFPRE_Div4 - HCLK clock = Core1 clock/4.
 *
 * @return  none
 */
void RCC_HCLKSourceDivConfig(uint32_t RCC_SYSCLK, uint32_t RCC_SYSCLKFPRE)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= ~(RCC_HPRE | RCC_FPRE);
    tmpreg |= RCC_SYSCLK | RCC_SYSCLKFPRE;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_TIMClockSourDivConfig
 *
 * @brief   TIMx - where x can be 1 to 10 to select the TIM peripheral.
 *          TIM_Clock_Divx -  defines the TIM clock divider
 *           TIM_Clock_Div2 - TIM_Clock = HCLK/2
 *           TIM_Clock_Div4 - TIM_Clock = HCLK/4
 *           TIM_Clock_Div8 - TIM_Clock = HCLK/8
 *
 * @return  none
 */
void RCC_TIMClockSourDivConfig(TIM_TypeDef * TIMx, uint32_t TIM_Clock_Divx)
{
   if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10))
   {
       RCC->CFGR0 &= ~RCC_PPRE2;
       RCC->CFGR0 |= TIM_Clock_Divx << 11;
   }
   else
   {
       RCC->CFGR0 &= ~RCC_PPRE1;
       RCC->CFGR0 |= TIM_Clock_Divx << 8;
   }
}

/*********************************************************************
 * @fn      RCC_LPTIMClockSourDivConfig
 *
 * @brief   LPTIM_Clock_Divx -  defines the LPTIM clock divider
 *           LPTIM_Clock_Div2 - LPTIM_Clock = HCLK/2
 *           LPTIM_Clock_Div4 - LPTIM_Clock = HCLK/4
 *           LPTIM_Clock_Div8 - LPTIM_Clock = HCLK/8
 *
 * @return  none
 */
void RCC_LPTIMClockSourDivConfig(uint32_t LPTIM_Clock_Divx)
{
    RCC->CFGR0 &= ~RCC_PPRE1;
    RCC->CFGR0 |= LPTIM_Clock_Divx << 8;
}

/*********************************************************************
 * @fn      RCC_ITConfig
 *
 * @brief   Enables or disables the specified RCC interrupts.
 *
 * @param   RCC_IT - specifies the RCC interrupt sources to be enabled or disabled.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *            RCC_IT_ETHPLLRDY - ETH PLL ready interrupt.
 *            RCC_IT_SERDESPLLRDY - SERDES PLL ready interrupt.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        *(__IO uint8_t *)0x4002100D |= RCC_IT;
    }
    else
    {
        *(__IO uint8_t *)0x4002100D &= (uint8_t)~RCC_IT;
    }
}

/*********************************************************************
 * @fn      RCC_ADCUSBHSPLLCLKAsSourceConfig
 *
 * @brief   Configures the ADC clock (ADCCLK), the PLL clock source is USBHSPLL(480MHz).
 *
 * @param   RCC_PPRE - defines the ADC clock divider. This clock is derived from
 *        the USBHS_PLL_CLK clock .
 *          RCC_USBHS_5 - ADC clock = USBHS_PLL_CLK/5.
 *          RCC_USBHS_6 - ADC clock = USBHS_PLL_CLK/6.
 *          RCC_USBHS_7 - ADC clock = USBHS_PLL_CLK/7.
 *          RCC_USBHS_8 - ADC clock = USBHS_PLL_CLK/8.
 *          RCC_USBHS_9 - ADC clock = USBHS_PLL_CLK/9.
 *          RCC_USBHS_10 - ADC clock = USBHS_PLL_CLK/10.
 *          RCC_USBHS_11 - ADC clock = USBHS_PLL_CLK/11.
 *          RCC_USBHS_12 - ADC clock = USBHS_PLL_CLK/12.
 *          RCC_USBHS_13 - ADC clock = USBHS_PLL_CLK/13.
 *          RCC_USBHS_14 - ADC clock = USBHS_PLL_CLK/14.
 *          RCC_USBHS_15 - ADC clock = USBHS_PLL_CLK/15.
 *          RCC_USBHS_16 - ADC clock = USBHS_PLL_CLK/16.
 *          RCC_USBHS_17 - ADC clock = USBHS_PLL_CLK/17.
 *          RCC_USBHS_18 - ADC clock = USBHS_PLL_CLK/18.
 *          RCC_USBHS_19 - ADC clock = USBHS_PLL_CLK/19.
 *          RCC_USBHS_20 - ADC clock = USBHS_PLL_CLK/20.
 *          RCC_USBHS_21 - ADC clock = USBHS_PLL_CLK/21.
 *          RCC_USBHS_22 - ADC clock = USBHS_PLL_CLK/22.
 *          RCC_USBHS_23 - ADC clock = USBHS_PLL_CLK/23.
 *          RCC_USBHS_24 - ADC clock = USBHS_PLL_CLK/24.
 *          RCC_USBHS_25 - ADC clock = USBHS_PLL_CLK/25.
 *          RCC_USBHS_26 - ADC clock = USBHS_PLL_CLK/26.
 *          RCC_USBHS_27 - ADC clock = USBHS_PLL_CLK/27.
 *          RCC_USBHS_28 - ADC clock = USBHS_PLL_CLK/28.
 *          RCC_USBHS_29 - ADC clock = USBHS_PLL_CLK/29.
 *          RCC_USBHS_30 - ADC clock = USBHS_PLL_CLK/30.
 *          RCC_USBHS_31 - ADC clock = USBHS_PLL_CLK/31.
 *          RCC_USBHS_32 - ADC clock = USBHS_PLL_CLK/32.
 *          RCC_USBHS_33 - ADC clock = USBHS_PLL_CLK/33.
 *          RCC_USBHS_34 - ADC clock = USBHS_PLL_CLK/34.
 *          RCC_USBHS_35 - ADC clock = USBHS_PLL_CLK/35.
 *          RCC_USBHS_36 - ADC clock = USBHS_PLL_CLK/36.
 *
 * @return  none
 */
void RCC_ADCUSBHSPLLCLKAsSourceConfig(uint32_t RCC_PPRE)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= ~(RCC_PPRE2 | RCC_ADCPRE);
    tmpreg |= (RCC_PPRE << 11);
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_ADCHCLKCLKAsSourceConfig
 *
 * @brief   Configures the ADC_clock (ADCCLK).
 *
 * @param   RCC_PPRE2_DIV - the PPRE2 clock divider. This clock is derived from
 *        the HCLK clock .
 *            RCC_PPRE2_DIV0 - HCLK.
 *            RCC_PPRE2_DIV2 - HCLK/2.
 *            RCC_PPRE2_DIV4 - HCLK/4.
 *            RCC_PPRE2_DIV8 - HCLK/8.
 *            RCC_PPRE2_DIV16 - HCLK/16.
 *          RCC_ADC_DIV - the ADC clock divider.
 *            RCC_HCLK_ADCPRE_DIV2 - ADC clock = HCLK/PPRE2_DIV/2.
 *            RCC_HCLK_ADCPRE_DIV4 - ADC clock = HCLK/PPRE2_DIV/4.
 *            RCC_HCLK_ADCPRE_DIV6 - ADC clock = HCLK/PPRE2_DIV/6.
 *            RCC_HCLK_ADCPRE_DIV8 - ADC clock = HCLK/PPRE2_DIV/8.
 *
 *          (ADC_Clock=HCLK/(RCC_PPRE2*RCC_ADCPRE))
 *
 * @return  none
 */
void RCC_ADCHCLKCLKAsSourceConfig(uint32_t RCC_PPRE2_DIV, uint32_t RCC_ADC_DIV)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= ~(RCC_PPRE2 | RCC_ADCPRE);
    tmpreg |= (RCC_PPRE2_DIV << 11)|(RCC_ADC_DIV << 14);
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_LSEConfig
 *
 * @brief   Configures the External Low Speed oscillator (LSE).
 *
 * @param   RCC_LSE - specifies the new state of the LSE.
 *            RCC_LSE_OFF - LSE oscillator OFF.
 *            RCC_LSE_ON - LSE oscillator ON.
 *            RCC_LSE_Bypass - LSE oscillator bypassed with external clock.
 *
 * @return  none
 */
void RCC_LSEConfig(uint8_t RCC_LSE)
{
    *(__IO uint8_t *)0x40021024 = RCC_LSE_OFF;
    *(__IO uint8_t *)0x40021024 = RCC_LSE_OFF;

    switch(RCC_LSE)
    {
        case RCC_LSE_ON:
            *(__IO uint8_t *)0x40021024 = RCC_LSE_ON;
            break;

        case RCC_LSE_Bypass:
            *(__IO uint8_t *)0x40021024 = RCC_LSE_Bypass | RCC_LSE_ON;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      RCC_LSICmd
 *
 * @brief   Enables or disables the Internal Low Speed oscillator (LSI).
 *          Note-
 *          LSI can not be disabled if the IWDG is running.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_LSICmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->RSTSCKR |= (1 << 0);
    }
    else
    {
        RCC->RSTSCKR &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      RCC_RTCCLKConfig
 *
 * @brief   Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
 *
 * @param   RCC_RTCCLKSource - specifies the RTC clock source.
 *            RCC_RTCCLKSource_LSE - LSE selected as RTC clock.
 *            RCC_RTCCLKSource_LSI - LSI selected as RTC clock.
 *            RCC_RTCCLKSource_HSE_Div512 - HSE clock divided by 128 selected as RTC clock.
 *         Note-   
 *           Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
 *
 * @return  none
 */
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource)
{
    RCC->BDCTLR &= (~RCC_RTCCLKSource_HSE_Div512);
    RCC->BDCTLR |= RCC_RTCCLKSource;
}

/*********************************************************************
 * @fn      RCC_RTCCLKCmd
 *
 * @brief   This function must be used only after the RTC clock was selected
 *        using the RCC_RTCCLKConfig function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_RTCCLKCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->BDCTLR |= (1 << 8);
    }
    else
    {
        RCC->BDCTLR &= ~(1 << 8);
    }
}

/*********************************************************************
 * @fn      RCC_GetClocksFreq
 *
 * @brief   The result of this function could be not correct when using
 *        fractional value for HSE crystal.
 *
 * @param   RCC_Clocks - pointer to a RCC_ClocksTypeDef structure which will hold
 *        the clocks frequencies.
 *
 * @return  none
 */
void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks)
{
    uint32_t tmp = 0,tmp1 = 0, tmp2 = 0, tmp3 = 0, pllmull = 0, pllsource = 0, presc = 0, presc1 = 0;

    tmp = RCC->CFGR0 & CFGR0_SWS_Mask;
    tmp2 = RCC->PLLCFGR & RCC_SYSPLL_SEL;

    switch(tmp)
    {
        case 0x00:
            RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
            break;

        case 0x04:
            RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
            break;

        case 0x08:
            switch(tmp2)
            { 
                case RCC_SYSPLL_PLL:
                    pllmull = RCC->PLLCFGR & RCC_PLLMUL;
                    pllsource = RCC->PLLCFGR & RCC_PLLSRC;
                    presc = (((RCC->PLLCFGR & RCC_PLL_SRC_DIV) >> 8) + 1);

                    if((pllsource == 0xE0) || (pllsource == 0xA0))
                    {
                        tmp1 = 500000000 / presc;
                    }
                    else if(pllsource == 0x80)
                    {
                        tmp1 = 480000000 / presc;
                    }
                    else if(pllsource == 0xC0)
                    {
                        tmp1 = 125000000 / presc;
                    }
                    else if(pllsource == 0x20)
                    {
                        tmp1 = HSE_VALUE / presc;
                    }
                    else
                    {
                        tmp1 = HSI_VALUE / presc;
                    }

                    if((pllmull == 4) || (pllmull == 6) || (pllmull == 8) || (pllmull == 10) || (pllmull == 12))
                    {
                        RCC_Clocks->SYSCLK_Frequency = (tmp1 * PLLMULTable[pllmull]) >> 1;
                    }
                    else
                    {
                        RCC_Clocks->SYSCLK_Frequency = tmp1 * PLLMULTable[pllmull];
                    }
                    break;

                case RCC_SYSPLL_USBHS:
                    RCC_Clocks->SYSCLK_Frequency = 480000000;
                    break;

                case RCC_SYSPLL_ETH:
                    RCC_Clocks->SYSCLK_Frequency = 500000000;
                    break;

                case RCC_SYSPLL_SERDES:
                    RCC_Clocks->SYSCLK_Frequency = 500000000;
                    break;

                case RCC_SYSPLL_USBSS:
                    RCC_Clocks->SYSCLK_Frequency = 125000000;
                    break;

                default:
                    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
                    break;
            }  
            break;

        default:
            RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
            break;
    }

    tmp = (RCC->CFGR0 & CFGR0_HPRE_Set_Mask) >> 4;
    presc1 = HBPrescTable[tmp];

       tmp3 = RCC_Clocks->SYSCLK_Frequency >> presc1;
  
    
    tmp = (RCC->CFGR0 & RCC_FPRE) >> 16;
    presc1 = FPRETable[tmp];
    RCC_Clocks->HCLK_Frequency = tmp3 >> presc1;

    if(NVIC_GetCurrentCoreID() == 0)//V3F
    {
        RCC_Clocks->Core_Frequency = RCC_Clocks->HCLK_Frequency;
    }
    else 
    {
        RCC_Clocks->Core_Frequency = tmp3;
    }

    if((RCC->CFGR0 & RCC_ADCSRC) == RCC_ADCSRC)
    {
        RCC_Clocks->ADCCLK_Frequency = 480000000 / (((RCC->CFGR0 & 0xF800) >> 11) + 5);
    }
    else
    {
        tmp = (RCC->CFGR0 & RCC_PPRE2) >> 11;
        presc = PPRE2Table[tmp];
        RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
        tmp = (RCC->CFGR0 & RCC_ADCPRE) >> 14;
        presc = ADCPRETable[tmp];
        RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->ADCCLK_Frequency / presc;  
    }
}

/*********************************************************************
 * @fn      RCC_HBPeriphClockCmd
 *
 * @brief   Enables or disables the HB peripheral clock.
 *
 * @param   RCC_HBPeriph - specifies the HB peripheral to gates its clock.
 *            RCC_HBPeriph_DMA1
 *            RCC_HBPeriph_DMA2
 *            RCC_HBPeriph_CRC
 *            RCC_HBPeriph_FMC
 *            RCC_HBPeriph_RNG
 *            RCC_HBPeriph_SDMMC
 *            RCC_HBPeriph_USBHS
 *            RCC_HBPeriph_USBSS
 *            RCC_HBPeriph_DVP
 *            RCC_HBPeriph_ETH
 *            RCC_HBPeriph_OTG_FS
 *            RCC_HBPeriph_UHSIF
 *            RCC_HBPeriph_USBPD
 *            RCC_HBPeriph_SERDES
 *            RCC_HBPeriph_POIC
 *         
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HBPeriphClockCmd(uint32_t RCC_HBPeriph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HBPCENR |= RCC_HBPeriph;
    }
    else
    {
        RCC->HBPCENR &= ~RCC_HBPeriph;
    }
}

/*********************************************************************
 * @fn      RCC_HB2PeriphClockCmd
 *
 * @brief   Enables or disables the High Speed HB2 peripheral clock.
 *
 * @param   RCC_HB2Periph - specifies the HB2 peripheral to gates its clock.
 *            RCC_HB2Periph_AFIO
 *            RCC_HB2Periph_HSADC
 *            RCC_HB2Periph_GPIOA
 *            RCC_HB2Periph_GPIOB
 *            RCC_HB2Periph_GPIOC
 *            RCC_HB2Periph_GPIOD
 *            RCC_HB2Periph_GPIOE
 *            RCC_HB2Periph_GPIOF
 *            RCC_HB2Periph_ADC1
 *            RCC_HB2Periph_ADC2
 *            RCC_HB2Periph_TIM1
 *            RCC_HB2Periph_SPI1
 *            RCC_HB2Periph_TIM8
 *            RCC_HB2Periph_USART1
 *            RCC_HB2Periph_I2C4
 *            RCC_HB2Periph_SAI
 *            RCC_HB2Periph_SDIO
 *            RCC_HB2Periph_TIM9
 *            RCC_HB2Periph_TIM10
 *            RCC_HB2Periph_TIM11
 *            RCC_HB2Periph_TIM12
 *            RCC_HB2Periph_OPCM
 *            RCC_HB2Periph_DFSDM
 *            RCC_HB2Periph_ECDC
 *            RCC_HB2Periph_GPHA
 *            RCC_HB2Periph_LTDC
 *            RCC_HB2Periph_I3C
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_HB2PeriphClockCmd(uint32_t RCC_HB2PCENR, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HB2PCENR |= RCC_HB2PCENR;
    }
    else
    {
        RCC->HB2PCENR &= ~ RCC_HB2PCENR;
    }
}

/*********************************************************************
 * @fn      RCC_HB1PeriphClockCmd
 *
 * @brief   Enables or disables the Low Speed HB1 peripheral clock.
 *
 * @param   RCC_HB1Periph - specifies the HB1 peripheral to gates its clock.
 *            RCC_HB1Periph_TIM2
 *            RCC_HB1Periph_TIM3
 *            RCC_HB1Periph_TIM4
 *            RCC_HB1Periph_TIM5
 *            RCC_HB1Periph_TIM6
 *            RCC_HB1Periph_TIM7
 *            RCC_HB1Periph_USART6
 *            RCC_HB1Periph_USART7
 *            RCC_HB1Periph_USART8
 *            RCC_HB1Periph_LPTIM1
 *            RCC_HB1Periph_LPTIM2
 *            RCC_HB1Periph_WWDG
 *            RCC_HB1Periph_QSPI1
 *            RCC_HB1Periph_QSPI2
 *            RCC_HB1Periph_SPI2
 *            RCC_HB1Periph_SPI3
 *            RCC_HB1Periph_SPI4
 *            RCC_HB1Periph_USART2
 *            RCC_HB1Periph_USART3
 *            RCC_HB1Periph_USART4
 *            RCC_HB1Periph_USART5
 *            RCC_HB1Periph_I2C1
 *            RCC_HB1Periph_I2C2
 *            RCC_HB1Periph_CAN3
 *            RCC_HB1Periph_CAN1
 *            RCC_HB1Periph_CAN2
 *            RCC_HB1Periph_BKP
 *            RCC_HB1Periph_PWR
 *            RCC_HB1Periph_DAC
 *            RCC_HB1Periph_I2C3
 *            RCC_HB1Periph_SWPMI
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HB1PeriphClockCmd(uint32_t RCC_HB1PCENR, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HB1PCENR |= RCC_HB1PCENR;
    }
    else
    {
        RCC->HB1PCENR &= ~RCC_HB1PCENR;
    }
}

/*********************************************************************
 * @fn      RCC_HBPeriphResetCmd
 *
 * @brief   Forces or releases HB peripheral reset.
 *
 * @param   RCC_HBPeriph - specifies the HB peripheral to reset.
 *            RCC_HBPeriph_DMA1
 *            RCC_HBPeriph_DMA2
 *            RCC_HBPeriph_FMC
 *            RCC_HBPeriph_RNG
 *            RCC_HBPeriph_SDMMC
 *            RCC_HBPeriph_USBHS
 *            RCC_HBPeriph_USBSS
 *            RCC_HBPeriph_DVP
 *            RCC_HBPeriph_ETH
 *            RCC_HBPeriph_OTG_FS
 *            RCC_HBPeriph_UHSIF
 *            RCC_HBPeriph_USBPD
 *            RCC_HBPeriph_SERDES
 *            RCC_HBPeriph_POIC
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HBPeriphResetCmd(uint32_t RCC_HBPeriph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HBPRSTR |= RCC_HBPeriph;
    }
    else
    {
        RCC->HBPRSTR &= ~RCC_HBPeriph;
    }
}

/*********************************************************************
 * @fn      RCC_HB2PeriphResetCmd
 *
 * @brief   Forces or releases High Speed HB2 peripheral reset.
 *
 * @param   RCC_HB2Periph - specifies the HB2 peripheral to reset.
 *            RCC_HB2Periph_AFIO
 *            RCC_HB2Periph_HSADC
 *            RCC_HB2Periph_IOPA
 *            RCC_HB2Periph_IOPB
 *            RCC_HB2Periph_IOPC
 *            RCC_HB2Periph_IOPD
 *            RCC_HB2Periph_IOPE
 *            RCC_HB2Periph_IOPF
 *            RCC_HB2Periph_ADC1
 *            RCC_HB2Periph_ADC2
 *            RCC_HB2Periph_TIM1
 *            RCC_HB2Periph_SPI1
 *            RCC_HB2Periph_TIM8
 *            RCC_HB2Periph_USART1
 *            RCC_HB2Periph_I2C4
 *            RCC_HB2Periph_SAI
 *            RCC_HB2Periph_SDIO
 *            RCC_HB2Periph_TIM9
 *            RCC_HB2Periph_TIM10
 *            RCC_HB2Periph_TIM11
 *            RCC_HB2Periph_TIM12
 *            RCC_HB2Periph_OPCM
 *            RCC_HB2Periph_DFSDM
 *            RCC_HB2Periph_ECDC
 *            RCC_HB2Periph_GPHA
 *            RCC_HB2Periph_LTDC
 *            RCC_HB2Periph_I3C
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_HB2PeriphResetCmd(uint32_t RCC_HB2Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HB2PRSTR |= RCC_HB2Periph;
    }
    else
    {
        RCC->HB2PRSTR &= ~RCC_HB2Periph;
    }
}

/*********************************************************************
 * @fn      RCC_HB1PeriphResetCmd
 *
 * @brief   Forces or releases Low Speed HB1 peripheral reset.
 *
 * @param   RCC_HB1Periph - specifies the HB1 peripheral to reset.
 *            RCC_HB1Periph_TIM2
 *            RCC_HB1Periph_TIM3
 *            RCC_HB1Periph_TIM4
 *            RCC_HB1Periph_TIM5
 *            RCC_HB1Periph_TIM6
 *            RCC_HB1Periph_TIM7
 *            RCC_HB1Periph_USART6
 *            RCC_HB1Periph_USART7
 *            RCC_HB1Periph_USART8
 *            RCC_HB1Periph_LPTIM1
 *            RCC_HB1Periph_LPTIM2
 *            RCC_HB1Periph_WWDG
 *            RCC_HB1Periph_QSPI1
 *            RCC_HB1Periph_QSPI2
 *            RCC_HB1Periph_SPI2
 *            RCC_HB1Periph_SPI3
 *            RCC_HB1Periph_SPI4
 *            RCC_HB1Periph_USART2
 *            RCC_HB1Periph_USART3
 *            RCC_HB1Periph_USART4
 *            RCC_HB1Periph_USART5
 *            RCC_HB1Periph_I2C1
 *            RCC_HB1Periph_I2C2
 *            RCC_HB1Periph_CAN3
 *            RCC_HB1Periph_CAN1
 *            RCC_HB1Periph_CAN2
 *            RCC_HB1Periph_BKP
 *            RCC_HB1Periph_PWR
 *            RCC_HB1Periph_DAC
 *            RCC_HB1Periph_I2C3
 *            RCC_HB1Periph_SWPMI
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HB1PeriphResetCmd(uint32_t RCC_HB1Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HB1PRSTR |= RCC_HB1Periph;
    }
    else
    {
        RCC->HB1PRSTR &= ~RCC_HB1Periph;
    }
}

/*********************************************************************
 * @fn      RCC_BackupResetCmd
 *
 * @brief   Forces or releases the Backup domain reset.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_BackupResetCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->BDCTLR |= (1 << 16);
    }
    else
    {
        RCC->BDCTLR &= ~(1 << 16);
    }
}

/*********************************************************************
 * @fn      RCC_ClockSecuritySystemCmd
 *
 * @brief   Enables or disables the Clock Security System.
 *
 * @param   RCC_CSSHSEDIS - Enables or disables the Hardware shutdown of HSE and PLL.
 *            RCC_CSSHSEDIS_ENABLE - PLL will close when set CSSON (HSE have problem).
 *            RCC_CSSHSEDIS_DISABLE - Close CSSHSEDIS function
 *          RCC_CSSHSEDIS - Enables or disables the Clock Security System.
 *            RCC_CSSON_ENABLE - Open CSSON function
 *            RCC_CSSON_DISABLE -  Close CSSON function
 *
 * @return  none
 */
void RCC_ClockSecuritySystemCmd(uint8_t RCC_CSSHSEDIS, uint8_t RCC_CSSON_State)
{
    if(RCC_CSSHSEDIS)
    {
        RCC->CTLR |= (1 << 31);
    }
    else
    {
        RCC->CTLR &= ~(1 << 31);
    }

    if(RCC_CSSON_State)
    {
        RCC->CTLR |= (1 << 19);
    }
    else
    {
        RCC->CTLR &= ~(1 << 19);
    }
}

/*********************************************************************
 * @fn      RCC_MCOConfig
 *
 * @brief   Selects the clock source to output on MCO pin.
 *
 * @param   RCC_MCO - specifies the clock source to output.
 *            RCC_MCO_NoClock - No clock selected.
 *            RCC_MCO_SYSCLK - System clock selected.
 *            RCC_MCO_HSI - HSI oscillator clock selected.
 *            RCC_MCO_HSE - HSE oscillator clock selected.
 *            RCC_MCO_PLLCLK_Div2 - PLL clock divided by 2 selected
 *            RCC_MCO_UTMI - UTMI clock selected.
 *            RCC_MCO_USBSSPLL_Div2 -  USBSS PLL clock divided by 2 selected
 *            RCC_MCO_ETHPLL_Div8 - ETH PLL clock divided by 8 selected
 *            RCC_MCO_SERDESPLL_Div16 - SERDES PLL clock divided by 16 selected
 *
 * @return  none
 */
void RCC_MCOConfig(uint8_t RCC_MCO)
{
    *(__IO uint8_t *)0x40021007 = RCC_MCO;
}

/*********************************************************************
 * @fn      RCC_GetFlagStatus
 *
 * @brief   Checks whether the specified RCC flag is set or not.
 *
 * @param   RCC_FLAG - specifies the flag to check.
 *            RCC_FLAG_HSIRDY - HSI oscillator clock ready.
 *            RCC_FLAG_HSERDY - HSE oscillator clock ready.
 *            RCC_FLAG_PLLRDY - PLL clock ready.
 *            RCC_FLAG_SERDESPLLRDY - SERDES PLL clock ready.
 *            RCC_FLAG_ETHPLLRDY - ETH PLL clock ready.
 *            RCC_FLAG_USBSSPLLRDY - USBSS PLL clock ready.
 *            RCC_FLAG_USBHSPLLRDY - USBHS PLL clock ready.
 *            RCC_FLAG_LSERDY - LSE oscillator clock ready.
 *            RCC_FLAG_LSIRDY - LSI oscillator clock ready.
 *            RCC_FLAG_PINRST - Pin reset.
 *            RCC_FLAG_PORRST - POR/PDR reset.
 *            RCC_FLAG_SFTRST - Software reset.
 *            RCC_FLAG_IWDGRST - Independent Watchdog reset.
 *            RCC_FLAG_WWDGRST - Window Watchdog reset.
 *            RCC_FLAG_LKUPRSTF - Lockup reset.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
    uint32_t tmp = 0;
    uint32_t statusreg = 0;

    FlagStatus bitstatus = RESET;
    tmp = RCC_FLAG >> 5;

    if(tmp == 1)
    {
        statusreg = RCC->CTLR;
    }
    else if(tmp == 2)
    {
        statusreg = RCC->BDCTLR;
    }
    else
    {
        statusreg = RCC->RSTSCKR;
    }

    tmp = RCC_FLAG & FLAG_Mask;

    if((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      RCC_ClearFlag
 *
 * @brief   Clears the RCC reset flags.
 *          Note-   
 *            The reset flags are: RCC_FLAG_PINRST, RCC_FLAG_PORRST, RCC_FLAG_SFTRST,
 *          RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LKUPRSTF
 *
 * @return  none
 */
void RCC_ClearFlag(void)
{
    RCC->RSTSCKR |= RSTSCKR_RMVF_Set;
}

/*********************************************************************
 * @fn      RCC_GetITStatus
 *
 * @brief   Checks whether the specified RCC interrupt has occurred or not.
 *
 * @param   RCC_IT - specifies the RCC interrupt source to check.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *            RCC_IT_SERDESPLLRDY - SERDES PLL ready interrupt.
 *            RCC_IT_ETHPLLRDY - ETH PLL ready interrupt.
 *            RCC_IT_CSSF - Clock Security System interrupt.
 *
 * @return  ITStatus - SET or RESET.
 */

ITStatus RCC_GetITStatus(uint8_t RCC_IT)
{
    ITStatus bitstatus = RESET;

    if((RCC->INTR & RCC_IT) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      RCC_ClearITPendingBit
 *
 * @brief   Clears the RCC's interrupt pending bits.
 *
 * @param   RCC_IT - specifies the interrupt pending bit to clear.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *            RCC_IT_SERDESPLLRDY - SERDES PLL ready interrupt.
 *            RCC_IT_ETHPLLRDY - ETH PLL ready interrupt.
 *            RCC_IT_CSSF - Clock Security System interrupt.
 *
 * @return  none
 */
void RCC_ClearITPendingBit(uint8_t RCC_IT)
{
    *(__IO uint8_t *)0x4002100E = RCC_IT;
}

/*********************************************************************
 * @fn      RCC_USBHS_PLLCmd
 *
 * @brief   ENABLE or DISABLE USBHS PLL Clock.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_USBHS_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= RCC_USBHS_PLLON;
    }
    else
    {
        RCC->CTLR &= ~RCC_USBHS_PLLON;
    }
}

/*********************************************************************
 * @fn      RCC_USBSS_PLLCmd
 *
 * @brief   ENABLE or DISABLE USBSS PLL Clock.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_USBSS_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= RCC_USBSS_PLLON;
    }
    else
    {
        RCC->CTLR &= ~RCC_USBSS_PLLON;
    }
}

/*********************************************************************
 * @fn      RCC_ETH_PLLCmd
 *
 * @brief   ENABLE or DISABLE ETH PLL Clock.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ETH_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= RCC_ETH_PLLON;
    }
    else
    {
        RCC->CTLR &= ~RCC_ETH_PLLON;
    }
}

/*********************************************************************
 * @fn      RCC_SERDES_PLLCmd
 *
 * @brief   ENABLE or DISABLE SERDES PLL Clock.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_SERDES_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= RCC_SERDES_PLLON;
    }
    else
    {
        RCC->CTLR &= ~RCC_SERDES_PLLON;
    }
}

/*********************************************************************
 * @fn      RCC_PIPEcmd
 *
 * @brief   ENABLE or DISABLE PIPE.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PIPECmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CFGR0 |= RCC_PIPEON;
    }
    else
    {
        RCC->CFGR0 &= ~RCC_PIPEON;
    }
}

/*********************************************************************
 * @fn      RCC_UTMIcmd
 *
 * @brief   ENABLE or DISABLE UTMI.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_UTMIcmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CFGR0 |= RCC_UTMION;
    }
    else
    {
        RCC->CFGR0 &= ~RCC_UTMION;
    }
}

/*********************************************************************
 * @fn      RCC_RGMIIcmd
 *
 * @brief   ENABLE or DISABLE RGMII.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_RGMIIcmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CFGR0 |= RCC_RGMIION;
    }
    else
    {
        RCC->CFGR0 &= ~RCC_RGMIION;
    }
}

/*********************************************************************
 * @fn      RCC_ETH125MCLKConfig
 *
 * @brief   Configures the ETH125M clock source.
 *
 * @param   RCC_ETH125MMSource - !000M ETH's 125M clock source. 
 *            RCC_ETH125MSource_PLLCLK - PLLCLK clock selected as ETH clock 125MHz  clock entry
 *            RCC_ETH125MSource_USBSS - USBSS_PLL clock selected as ETH clock 125MHz  clock entry
 *            RCC_ETH125MSource_ETH_Div4 - ETH_PLL_Div4 clock selected as ETH clock 125MHz  clock entry
 *            RCC_ETH125MSource_SERDES_Div8 - SERDES_PLL_Div8 clock selected as ETH clock 125MHz  clock entry
 *
 * @return  none
 */
void RCC_ETH125MCLKConfig(uint32_t RCC_ETH125MMSource)
{
    RCC->CFGR2 &= ~RCC_ETH1GSRC;
    RCC->CFGR2 |= (RCC_ETH125MMSource << 30);
}

/*********************************************************************
 * @fn      RCC_HSADCCLKConfig
 *
 * @brief   Configures the HSADC Clock source.
 *
 * @param   RCC_HSADCSource - specifies the HSADC source.
 *            RCC_HSADCSource_PLLCLK - PLLCLK clock selected as HSADC clock entry
 *            RCC_HSADCSource_SYSCLK - SYSCLK clock selected as HSADC clock entry
 *            RCC_HSADCSource_USBHS - USBHS clock selected as HSADC clock entry
 *            RCC_HSADCSource_ETH - TTH clock selected as HSADC clock entry
 *
 * @return  none
 */
void RCC_HSADCCLKConfig(uint32_t RCC_HSADCSource)
{
    RCC->CFGR2 &= ~RCC_HSADCSRC;
    RCC->CFGR2 |= (RCC_HSADCSource << 28);
}

/*********************************************************************
 * @fn      RCC_I2S3CLKConfig
 *
 * @brief   Configures the I2S3 clock source(I2S23CLK).
 *
 * @param   RCC_I2S3CLKSource - specifies the I2S3 clock source.
 *            RCC_I2S3CLKSource_SYSCLK - system clock selected as I2S3 clock entry
 *            RCC_I2S3CLKSource_PLLCLK - PLL clock selected as I2S3 clock entry
 *          
 * @return  none
 */
void RCC_I2S3CLKConfig(uint32_t RCC_I2S3CLKSource)
{
    RCC->CFGR2 &= ~RCC_I2S3SRC;
    RCC->CFGR2 |= (RCC_I2S3CLKSource << 25);
}

/*********************************************************************
 * @fn      RCC_I2S2CLKConfig
 *
 * @brief   Configures the I2S2 clock source(I2S2CLK).
 *
 * @param   RCC_I2S2CLKSource - specifies the I2S2 clock source.
 *            RCC_I2S2CLKSource_SYSCLK - system clock selected as I2S2 clock entry
 *            RCC_I2S2CLKSource_PLLCLK - PLLclock selected as I2S2 clock entry
 *          
 * @return  none
 */
void RCC_I2S2CLKConfig(uint32_t RCC_I2S2CLKSource)
{
    RCC->CFGR2 &= ~RCC_I2S2SRC;
    RCC->CFGR2 |= (RCC_I2S2CLKSource << 24);
}

/*********************************************************************
 * @fn      RCC_RNGCLKConfig
 *
 * @brief   Configures the RNG clock source.
 *
 * @param   RCC_RNGCLKSource - specifies the RNG clock source.
 *            RCC_RNGCLKSource_SYSCLK - system clock selected as RNG clock entry
 *            RCC_RNGCLKSource_PLLCLK - PLL clock selected as RNG clock entry
 *
 * @return  none
 */
void RCC_RNGCLKConfig(uint32_t RCC_RNGCLKSource)
{
    RCC->CFGR2 &= ~RCC_RNGSRC;
    RCC->CFGR2 |= (RCC_RNGCLKSource << 23);
}

/*********************************************************************
 * @fn      RCC_USBFSCLKConfig
 *
 * @brief   Configures the USBFS clock source.
 *
 * @param   RCC_USBFSCLKSource - specifies the RNG clock source.
 *            RCC_USBFSCLKSource_USBHSPLL - USBHS PLL clock selected as USBFS_48M clock entry
 *            RCC_USBFSCLKSource_PLL - PLL clock selected as USBFS_48M clock entry
 *
 * @return  none
 */
void RCC_USBFSCLKConfig(uint32_t RCC_USBFSCLKSource)
{
    RCC->CFGR2 &= ~RCC_USBFSSRC;
    RCC->CFGR2 |= (RCC_USBFSCLKSource << 20);
}

/*********************************************************************
 * @fn      RCC_USBFS48ClockSourceDivConfig
 *
 * @brief   Configures the USBFS clock Source Division.
 *
 * @param   RCC_USBFS_DIV - USBFS clock Source Division.
 *            RCC_USBFS_Divx - x can select(1,2,3,4,5,6,8,10,1.5,2.5,3.5,4.5,
 *               5.5,6.5,7.5,9.5)
 *
 * @return  none
 */
void RCC_USBFS48ClockSourceDivConfig(uint32_t RCC_USBFS_DIV)
{
    RCC->CFGR2 &= ~RCC_USBFSDIV;
    RCC->CFGR2 |= (RCC_USBFS_DIV << 16);
}

/*********************************************************************
 * @fn      RCC_LTDCCLKConfig
 *
 * @brief   Configures the LTDC clock source.
 *
 * @param   RCC_LTDCClockSource - the LTDC clock source.
 *            RCC_LTDCClockSource_PLL - HCLK clock selected as LTDC clock entry
 *            RCC_LTDCClockSource_SERDESPLL - SERDES PLL clock selected as LTDC clock entry
 *            RCC_LTDCClockSource_ETHPLL - ETH PLL clock selected as LTDC clock entry
 *            RCC_LTDCClockSource_USBHSPLL - USBHS PLL clock selected as LTDC clock entry
 *
 * @return  none
 */
void RCC_LTDCCLKConfig(uint32_t RCC_LTDCClockSource)
{
    RCC->CFGR2 &= ~RCC_LTDCSRC;
    RCC->CFGR2 |= (RCC_LTDCClockSource << 14);
}

/*********************************************************************
 * @fn      RCC_LTDCClockSourceDivConfig
 *
 * @brief   Configures the LTDC clock Source Division.
 *
 * @param   RCC_LTDCClockSource_Div - the LTDC clock Source Division
 *            RCC_LTDCClockSource_Divx - x can select(1~64)
 *
 * @return  none
 */
void RCC_LTDCClockSourceDivConfig(uint32_t RCC_LTDCClockSource_Div)
{
    RCC->CFGR2 &= ~RCC_LTDCDIV;
    RCC->CFGR2 |= RCC_LTDCClockSource_Div << 8;
}

/*********************************************************************
 * @fn      RCC_UHSIFCLKConfig
 *
 * @brief   Configures the UHSIF clock source.
 *
 * @param   RCC_UHSIFClockSource - the UHSIF clock source.
 *            RCC_UHSIFClockSource_SYSCLK - system clock as the UHSIF clock source 
 *            RCC_UHSIFClockSource_PLL - PLL as the UHSIF clock source
 *            RCC_UHSIFClockSource_USBHSPLL - USBHS PLL as  the UHSIF clock source
 *            RCC_UHSIFClockSource_ETHPLL - ETH PLL as  the UHSIF clock source
 *
 * @return  none
 */
void RCC_UHSIFCLKConfig(uint32_t RCC_UHSIFClockSource)
{
    RCC->CFGR2 &= ~RCC_UHSIFSRC;
    RCC->CFGR2 |= (RCC_UHSIFClockSource << 6);
}

/*********************************************************************
 * @fn      RCC_UHSIFClockSourceDivConfig
 *
 * @brief   Configures the UHSIF clock Source Division.
 *
 * @param   RCC_UHSIFClockSource_Div - the UHSIF clock Source Division.
 *            RCC_UHSIFClockSource_Divx - x can select(1~64)
 *
 * @return  none
 */
void RCC_UHSIFClockSourceDivConfig(uint32_t RCC_UHSIFClockSource_Div)
{
    RCC->CFGR2 &= ~RCC_UHSIFDIV ;
    RCC->CFGR2 |= RCC_UHSIFClockSource_Div;
}

/*********************************************************************
 * @fn      RCC_USBHSPLLCLKConfig
 *
 * @brief   Configures the USBHS PLL clock source.
 *
 * @param   RCC_USBHSPLLSource - the USBHS clock source.
 *            RCC_USBHSPLLSource_HSE - HSE clock selected as USBHS PLL clock entry
 *            RCC_USBHSPLLSource_HSI - HSI clock selected as USBHS PLL clock entry
 *            RCC_USBHSPLLSource_20METH  - ETH 20M clock selected as USBHS PLL clock entry
 *            RCC_USBHSPLLSource_PLL_CLK_DIV - PLL_CLK division clock selected as USBHS PLL clock entry
 *
 * @return  none
 */
void RCC_USBHSPLLCLKConfig(uint32_t RCC_USBHSPLLSource)
{
    RCC->PLLCFGR2 &= ~RCC_USBHSPLLSRC;
    RCC->PLLCFGR2 |= RCC_USBHSPLLSource;
}

/*********************************************************************
 * @fn      RCC_USBHSPLLReferConfig
 *
 * @brief   Configures the USBHS PLL refer clock.
 *
 * @param   RCC_USBHSPLLRefer - the USBHS PLL refer clock.
 *            RCC_USBHSPLLRefer_25M - USBHS PLL Refer clock 25M
 *            RCC_USBHSPLLRefer_20M - USBHS PLL Refer clock 20M
 *            RCC_USBHSPLLRefer_24M  - USBHS PLL Refer clock 24M
 *            RCC_USBHSPLLRefer_32M - USBHS PLL Refer clock 32M
 *
 * @return  none
 */
void RCC_USBHSPLLReferConfig(uint32_t RCC_USBHSPLLRefer)
{
    RCC->PLLCFGR2 &= ~RCC_USBHSPLL_REFSEL;
    RCC->PLLCFGR2 |= RCC_USBHSPLLRefer;
}

/*********************************************************************
 * @fn      RCC_USBSSPLLReferConfig
 *
 * @brief   Configures the USBSS PLL refer clock.
 *
 * @param   RCC_USBSSPLLRefer - the USBSS PLL refer clock.
 *            RCC_USBSSPLLRefer_20M - USBSS PLL Refer clock 20M
 *            RCC_USBSSPLLRefer_24M - USBSS PLL Refer clock 24M
 *            RCC_USBSSPLLRefer_25M - USBSS PLL Refer clock 25M
 *            RCC_USBSSPLLRefer_30M - USBSS PLL Refer clock 30M
 *            RCC_USBSSPLLRefer_32M - USBSS PLL Refer clock 32M
 *            RCC_USBSSPLLRefer_40M - USBSS PLL Refer clock 40M
 *            RCC_USBSSPLLRefer_60M - USBSS PLL Refer clock 60M
 *            RCC_USBSSPLLRefer_80M - USBSS PLL Refer clock 80M
 *
 * @return  none
 */
void RCC_USBSSPLLReferConfig(uint32_t RCC_USBSSPLLRefer)
{
    RCC->PLLCFGR2 &= ~RCC_USBSSPLL_REFSEL;
    RCC->PLLCFGR2 |= RCC_USBSSPLLRefer;
}

/*********************************************************************
 * @fn      RCC_USBHSPLLClockSourceDivConfig
 *
 * @brief   Configures the USBHS PLL source clock division.
 *
 * @param   RCC_USBHSPLL_IN_Div - the USBHS PLL source clock division.
 *            RCC_USBHSPLL_IN_Divx - x can select(1~32)
 *
 * @return  none
 */
void RCC_USBHSPLLClockSourceDivConfig(uint32_t RCC_USBHSPLL_IN_Div)
{
    RCC->PLLCFGR2 &= ~RCC_USBHSPLL_IN_DIV;
    RCC->PLLCFGR2 |= RCC_USBHSPLL_IN_Div;
}

/*********************************************************************
 * @fn      RCC_SERDESPLLMulConfig
 *
 * @brief   Configures the SERDES PLL Mul.
 *
 * @param   RCC_SERDESMul_25 - the SERDES PLL Mul.
 *            RCC_SERDESPLLMul_x - x can select(25,28,30,32,35,38,40,45,50,
 *       56,60,64,70,76,80,90)
 *          
 * @return  none
 */
void RCC_SERDESPLLMulConfig(uint32_t RCC_SERDESPLLMul)
{
    RCC->PLLCFGR2 &= ~RCC_SERDESPLL_MUL;
    RCC->PLLCFGR2 |= (RCC_SERDESPLLMul << 16);
}

/*********************************************************************
 * @fn      RCC_ADCCLKDutyCycleConfig
 *
 * @brief   Configures the ADC clock high level duty cycle.
 *
 * @param   RCC_DutyCycle - high level duty cycle.
 *            RCC_ADC_H_Level_Mode0 - ADC clock high-level duty cycle is mode 0.
 *            RCC_ADC_H_Level_Mode1 - ADC clock high-level duty cycle is mode 1.
 *
 * @return  none
 */
void RCC_ADCCLKDutyCycleConfig(uint32_t RCC_DutyCycle)
{
    RCC->CFGR0 &= ~RCC_ADC_DUTY_SEL;
    RCC->CFGR0 |= RCC_DutyCycle;
}

/*********************************************************************
 * @fn      RCC_ADCCLKConfig
 *
 * @brief   Configures the ADC clock source.
 *
 * @param   RCC_ADCCLKSource - specifies the RNG clock source.
 *            RCC_ADCCLKSource_HCLK -  HCLK clock selected as ADC clock entry
 *            RCC_ADCCLKSource_USBHSPLL - USBHS PLL clock selected as ADC clock entry
 *
 * @return  none
 */
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLKSource)
{
    RCC->CFGR2 &= ~RCC_ADCSRC;
    RCC->CFGR2 |= (RCC_ADCCLKSource << 31);
}

/*********************************************************************
 * @fn      RCC_SYSPLLGATEcmd
 *
 * @brief   ENABLE or DISABLE System clock  gate.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_SYSPLLGATEcmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->PLLCFGR |= RCC_SYSPLL_GATE;
    }
    else
    {
        RCC->PLLCFGR &= ~RCC_SYSPLL_GATE;
    }
}

/*********************************************************************
 * @fn      RCC_SYSPLLConfig
 *
 * @brief   Configures the System PLL clock source.
 *
 * @param   RCC_SYSPLLClockSource - the System PLL clock source.
 *            RCC_SYSPLLClockSource_PLL - PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_USBHSPLL - USBHS PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_ETHPLL - ETH PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_SERDESPLL - SERDES PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_USBSSPLL - USBSS PLL clock selected as System PLL clock entry
 *
 * @return  none
 */
void RCC_SYSPLLConfig(uint32_t RCC_SYSPLLClockSource)
{
    RCC->PLLCFGR &= ~RCC_SYSPLL_SEL;
    RCC->PLLCFGR |= (RCC_SYSPLLClockSource << 28);
}
