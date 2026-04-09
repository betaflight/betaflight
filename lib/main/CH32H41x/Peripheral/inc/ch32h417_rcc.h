/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_rcc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_RCC_H
#define __CH32H417_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* RCC_Exported_Types */
typedef struct
{
  uint32_t SYSCLK_Frequency;  /* returns SYSCLK clock frequency expressed in Hz */
  uint32_t HCLK_Frequency;    /* returns HCLK/Core0 clock frequency expressed in Hz */
  uint32_t Core_Frequency;   /* returns Core clock frequency expressed in Hz */
  uint32_t ADCCLK_Frequency;  /* returns ADCCLK clock frequency expressed in Hz */
}RCC_ClocksTypeDef;

/* HSE_configuration */
#define RCC_HSE_OFF                      ((uint32_t)0x00000000)
#define RCC_HSE_ON                       ((uint32_t)0x00010000)
#define RCC_HSE_Bypass                   ((uint32_t)0x00040000)

/* PLL_entry_clock_source */
#define RCC_PLLSource_HSI                ((uint32_t)0x00000000)
#define RCC_PLLSource_HSE                ((uint32_t)0x00000020)
#define RCC_PLLSource_USBHS              ((uint32_t)0x00000080)
#define RCC_PLLSource_ETH                ((uint32_t)0x000000A0)
#define RCC_PLLSource_USBSS              ((uint32_t)0x000000C0)
#define RCC_PLLSource_SERDES             ((uint32_t)0x000000E0)

/* PLL_multiplication_factor */
#define RCC_PLLMul_4                     ((uint32_t)0x00000000)
#define RCC_PLLMul_6                     ((uint32_t)0x00000001)
#define RCC_PLLMul_7                     ((uint32_t)0x00000002)
#define RCC_PLLMul_8                     ((uint32_t)0x00000003)
#define RCC_PLLMul_8_5                   ((uint32_t)0x00000004)
#define RCC_PLLMul_9                     ((uint32_t)0x00000005)
#define RCC_PLLMul_9_5                   ((uint32_t)0x00000006)
#define RCC_PLLMul_10                    ((uint32_t)0x00000007)
#define RCC_PLLMul_10_5                  ((uint32_t)0x00000008)
#define RCC_PLLMul_11                    ((uint32_t)0x00000009)
#define RCC_PLLMul_11_5                  ((uint32_t)0x0000000A)
#define RCC_PLLMul_12                    ((uint32_t)0x0000000B)
#define RCC_PLLMul_12_5                  ((uint32_t)0x0000000C)
#define RCC_PLLMul_13                    ((uint32_t)0x0000000D)
#define RCC_PLLMul_14                    ((uint32_t)0x0000000E)
#define RCC_PLLMul_15                    ((uint32_t)0x0000000F)
#define RCC_PLLMul_16                    ((uint32_t)0x00000010)
#define RCC_PLLMul_17                    ((uint32_t)0x00000011)
#define RCC_PLLMul_18                    ((uint32_t)0x00000012)
#define RCC_PLLMul_19                    ((uint32_t)0x00000013)
#define RCC_PLLMul_20                    ((uint32_t)0x00000014)
#define RCC_PLLMul_22                    ((uint32_t)0x00000015)
#define RCC_PLLMul_24                    ((uint32_t)0x00000016)
#define RCC_PLLMul_26                    ((uint32_t)0x00000017)
#define RCC_PLLMul_28                    ((uint32_t)0x00000018)
#define RCC_PLLMul_30                    ((uint32_t)0x00000019)
#define RCC_PLLMul_32                    ((uint32_t)0x0000001A)
#define RCC_PLLMul_34                    ((uint32_t)0x0000001B)
#define RCC_PLLMul_36                    ((uint32_t)0x0000001C)
#define RCC_PLLMul_38                    ((uint32_t)0x0000001D)
#define RCC_PLLMul_40                    ((uint32_t)0x0000001E)
#define RCC_PLLMul_59                    ((uint32_t)0x0000001F)

/* PLL_division_factor */
#define RCC_PLLDiv_1                     ((uint32_t)0x00000000)
#define RCC_PLLDiv_2                     ((uint32_t)0x00000001)
#define RCC_PLLDiv_3                     ((uint32_t)0x00000002)
#define RCC_PLLDiv_4                     ((uint32_t)0x00000003)
#define RCC_PLLDiv_5                     ((uint32_t)0x00000004)
#define RCC_PLLDiv_6                     ((uint32_t)0x00000005)
#define RCC_PLLDiv_7                     ((uint32_t)0x00000006)
#define RCC_PLLDiv_8                     ((uint32_t)0x00000007)
#define RCC_PLLDiv_9                     ((uint32_t)0x00000008)
#define RCC_PLLDiv_10                    ((uint32_t)0x00000009)
#define RCC_PLLDiv_11                    ((uint32_t)0x0000000A)
#define RCC_PLLDiv_12                    ((uint32_t)0x0000000B)
#define RCC_PLLDiv_13                    ((uint32_t)0x0000000C)
#define RCC_PLLDiv_14                    ((uint32_t)0x0000000D)
#define RCC_PLLDiv_15                    ((uint32_t)0x0000000E)
#define RCC_PLLDiv_16                    ((uint32_t)0x0000000F)
#define RCC_PLLDiv_17                    ((uint32_t)0x00000010)
#define RCC_PLLDiv_18                    ((uint32_t)0x00000011)
#define RCC_PLLDiv_19                    ((uint32_t)0x00000012)
#define RCC_PLLDiv_20                    ((uint32_t)0x00000013)
#define RCC_PLLDiv_21                    ((uint32_t)0x00000014)
#define RCC_PLLDiv_22                    ((uint32_t)0x00000015)
#define RCC_PLLDiv_23                    ((uint32_t)0x00000016)
#define RCC_PLLDiv_24                    ((uint32_t)0x00000017)
#define RCC_PLLDiv_25                    ((uint32_t)0x00000018)
#define RCC_PLLDiv_26                    ((uint32_t)0x00000019)
#define RCC_PLLDiv_27                    ((uint32_t)0x0000001A)
#define RCC_PLLDiv_28                    ((uint32_t)0x0000001B)
#define RCC_PLLDiv_29                    ((uint32_t)0x0000001C)
#define RCC_PLLDiv_30                    ((uint32_t)0x0000001D)
#define RCC_PLLDiv_31                    ((uint32_t)0x0000001E)
#define RCC_PLLDiv_32                    ((uint32_t)0x0000001F)
#define RCC_PLLDiv_33                    ((uint32_t)0x00000020)
#define RCC_PLLDiv_34                    ((uint32_t)0x00000021)
#define RCC_PLLDiv_35                    ((uint32_t)0x00000022)
#define RCC_PLLDiv_36                    ((uint32_t)0x00000023)
#define RCC_PLLDiv_37                    ((uint32_t)0x00000024)
#define RCC_PLLDiv_38                    ((uint32_t)0x00000025)
#define RCC_PLLDiv_39                    ((uint32_t)0x00000026)
#define RCC_PLLDiv_40                    ((uint32_t)0x00000027)
#define RCC_PLLDiv_41                    ((uint32_t)0x00000028)
#define RCC_PLLDiv_42                    ((uint32_t)0x00000029)
#define RCC_PLLDiv_43                    ((uint32_t)0x0000002A)
#define RCC_PLLDiv_44                    ((uint32_t)0x0000002B)
#define RCC_PLLDiv_45                    ((uint32_t)0x0000002C)
#define RCC_PLLDiv_46                    ((uint32_t)0x0000002D)
#define RCC_PLLDiv_47                    ((uint32_t)0x0000002E)
#define RCC_PLLDiv_48                    ((uint32_t)0x0000002F)
#define RCC_PLLDiv_49                    ((uint32_t)0x00000030)
#define RCC_PLLDiv_50                    ((uint32_t)0x00000031)
#define RCC_PLLDiv_51                    ((uint32_t)0x00000032)
#define RCC_PLLDiv_52                    ((uint32_t)0x00000033)
#define RCC_PLLDiv_53                    ((uint32_t)0x00000034)
#define RCC_PLLDiv_54                    ((uint32_t)0x00000035)
#define RCC_PLLDiv_55                    ((uint32_t)0x00000036)
#define RCC_PLLDiv_56                    ((uint32_t)0x00000037)
#define RCC_PLLDiv_57                    ((uint32_t)0x00000038)
#define RCC_PLLDiv_58                    ((uint32_t)0x00000039)
#define RCC_PLLDiv_59                    ((uint32_t)0x0000003A)
#define RCC_PLLDiv_60                    ((uint32_t)0x0000003B)
#define RCC_PLLDiv_61                    ((uint32_t)0x0000003C)
#define RCC_PLLDiv_62                    ((uint32_t)0x0000003D)
#define RCC_PLLDiv_63                    ((uint32_t)0x0000003E)
#define RCC_PLLDiv_64                    ((uint32_t)0x0000003F)

/* System_clock_source */
#define RCC_SYSCLKSource_HSI             ((uint32_t)0x00000000)
#define RCC_SYSCLKSource_HSE             ((uint32_t)0x00000001)
#define RCC_SYSCLKSource_PLLCLK          ((uint32_t)0x00000002)

/* Core1_division_factor */
#define RCC_SYSCLK_Div1                  ((uint32_t)0x00000000)
#define RCC_SYSCLK_Div2                  ((uint32_t)0x00000008)
#define RCC_SYSCLK_Div4                  ((uint32_t)0x00000009)
#define RCC_SYSCLK_Div8                  ((uint32_t)0x0000000A)
#define RCC_SYSCLK_Div16                 ((uint32_t)0x0000000B)
#define RCC_SYSCLK_Div64                 ((uint32_t)0x0000000C)
#define RCC_SYSCLK_Div128                ((uint32_t)0x0000000D)
#define RCC_SYSCLK_Div256                ((uint32_t)0x0000000E)
#define RCC_SYSCLK_Div512                ((uint32_t)0x0000000F)

/* HCLK_division_factor */
#define RCC_SYSCLKFPRE_Div1              ((uint32_t)0x00000000)
#define RCC_SYSCLKFPRE_Div2              ((uint32_t)0x00010000)
#define RCC_SYSCLKFPRE_Div4              ((uint32_t)0x00020000)

/* TIM_Clock_division_factor */
#define TIM_Clock_Div2                   ((uint32_t)0x00000005)
#define TIM_Clock_Div4                   ((uint32_t)0x00000006)
#define TIM_Clock_Div8                   ((uint32_t)0x00000007)

/* LPTIM_Clock_division_factor */
#define LPTIM_Clock_Div2                 ((uint32_t)0x00000005)
#define LPTIM_Clock_Div4                 ((uint32_t)0x00000006)
#define LPTIM_Clock_Div8                 ((uint32_t)0x00000007)

/* RCC_Interrupt_source */
#define RCC_IT_LSIRDY                    ((uint8_t)0x01)
#define RCC_IT_LSERDY                    ((uint8_t)0x02)
#define RCC_IT_HSIRDY                    ((uint8_t)0x04)
#define RCC_IT_HSERDY                    ((uint8_t)0x08)
#define RCC_IT_PLLRDY                    ((uint8_t)0x10)
#define RCC_IT_ETHPLLRDY                 ((uint8_t)0x20)
#define RCC_IT_SERDESPLLRDY              ((uint8_t)0x40)
#define RCC_IT_CSSF                      ((uint8_t)0x80)

/* ADC_Clock_source= USBHS/Divx */
#define RCC_USBHS_Div5                   ((uint8_t)0x00)
#define RCC_USBHS_Div6                   ((uint8_t)0x01)
#define RCC_USBHS_Div7                   ((uint8_t)0x02)
#define RCC_USBHS_Div8                   ((uint8_t)0x03)
#define RCC_USBHS_Div9                   ((uint8_t)0x04)
#define RCC_USBHS_Div10                  ((uint8_t)0x05)
#define RCC_USBHS_Div11                  ((uint8_t)0x06)
#define RCC_USBHS_Div12                  ((uint8_t)0x07)
#define RCC_USBHS_Div13                  ((uint8_t)0x08)
#define RCC_USBHS_Div14                  ((uint8_t)0x09)
#define RCC_USBHS_Div15                  ((uint8_t)0x0A)
#define RCC_USBHS_Div16                  ((uint8_t)0x0B)
#define RCC_USBHS_Div17                  ((uint8_t)0x0C)
#define RCC_USBHS_Div18                  ((uint8_t)0x0D)
#define RCC_USBHS_Div19                  ((uint8_t)0x0E)
#define RCC_USBHS_Div20                  ((uint8_t)0x0F)
#define RCC_USBHS_Div21                  ((uint8_t)0x10)
#define RCC_USBHS_Div22                  ((uint8_t)0x11)
#define RCC_USBHS_Div23                  ((uint8_t)0x12)
#define RCC_USBHS_Div24                  ((uint8_t)0x13)
#define RCC_USBHS_Div25                  ((uint8_t)0x14)
#define RCC_USBHS_Div26                  ((uint8_t)0x15)
#define RCC_USBHS_Div27                  ((uint8_t)0x16)
#define RCC_USBHS_Div28                  ((uint8_t)0x17)
#define RCC_USBHS_Div29                  ((uint8_t)0x18)
#define RCC_USBHS_Div30                  ((uint8_t)0x19)
#define RCC_USBHS_Div31                  ((uint8_t)0x1A)
#define RCC_USBHS_Div32                  ((uint8_t)0x1B)
#define RCC_USBHS_Div33                  ((uint8_t)0x1C)
#define RCC_USBHS_Div34                  ((uint8_t)0x1D)
#define RCC_USBHS_Div35                  ((uint8_t)0x1E)
#define RCC_USBHS_Div36                  ((uint8_t)0x1F)

/* ADC_Clock_source = HCLK/(RCC_PPRE2*RCC_ADCPRE) */
#define RCC_PPRE2_DIV0                   ((uint8_t)0x00)
#define RCC_PPRE2_DIV2                   ((uint8_t)0x04)
#define RCC_PPRE2_DIV4                   ((uint8_t)0x05)
#define RCC_PPRE2_DIV8                   ((uint8_t)0x06)
#define RCC_PPRE2_DIV16                  ((uint8_t)0x07)

#define RCC_HCLK_ADCPRE_DIV2             ((uint8_t)0x00)
#define RCC_HCLK_ADCPRE_DIV4             ((uint8_t)0x01)
#define RCC_HCLK_ADCPRE_DIV6             ((uint8_t)0x02)
#define RCC_HCLK_ADCPRE_DIV8             ((uint8_t)0x03)

/* LSE_configuration */
#define RCC_LSE_OFF                      ((uint8_t)0x00)
#define RCC_LSE_ON                       ((uint8_t)0x01)
#define RCC_LSE_Bypass                   ((uint8_t)0x04)

/* RTC_clock_source */
#define RCC_RTCCLKSource_LSE             ((uint32_t)0x00000040)
#define RCC_RTCCLKSource_LSI             ((uint32_t)0x00000080)
#define RCC_RTCCLKSource_HSE_Div512      ((uint32_t)0x000000C0)

/* HB_peripheral */
#define RCC_HBPeriph_DMA1                ((uint32_t)0x00000001)
#define RCC_HBPeriph_DMA2                ((uint32_t)0x00000002)
#define RCC_HBPeriph_CRC                 ((uint32_t)0x00000040)
#define RCC_HBPeriph_FMC                 ((uint32_t)0x00000100)
#define RCC_HBPeriph_RNG                 ((uint32_t)0x00000200)
#define RCC_HBPeriph_SDMMC               ((uint32_t)0x00000400)
#define RCC_HBPeriph_USBHS               ((uint32_t)0x00000800)
#define RCC_HBPeriph_USBSS               ((uint32_t)0x00001000)
#define RCC_HBPeriph_DVP                 ((uint32_t)0x00002000)
#define RCC_HBPeriph_ETH                 ((uint32_t)0x00004000)
#define RCC_HBPeriph_OTG_FS              ((uint32_t)0x00020000)
#define RCC_HBPeriph_UHSIF               ((uint32_t)0x00040000)
#define RCC_HBPeriph_USBPD               ((uint32_t)0x00080000)
#define RCC_HBPeriph_SERDES              ((uint32_t)0x00100000)
#define RCC_HBPeriph_PIOC                ((uint32_t)0x00400000)

/* HB2_peripheral */
#define RCC_HB2Periph_AFIO               ((uint32_t)0x00000001)
#define RCC_HB2Periph_HSADC              ((uint32_t)0x00000002)
#define RCC_HB2Periph_GPIOA              ((uint32_t)0x00000004)
#define RCC_HB2Periph_GPIOB              ((uint32_t)0x00000008)
#define RCC_HB2Periph_GPIOC              ((uint32_t)0x00000010)
#define RCC_HB2Periph_GPIOD              ((uint32_t)0x00000020)
#define RCC_HB2Periph_GPIOE              ((uint32_t)0x00000040)
#define RCC_HB2Periph_GPIOF              ((uint32_t)0x00000080)
#define RCC_HB2Periph_ADC1               ((uint32_t)0x00000200)
#define RCC_HB2Periph_ADC2               ((uint32_t)0x00000400)
#define RCC_HB2Periph_TIM1               ((uint32_t)0x00000800)
#define RCC_HB2Periph_SPI1               ((uint32_t)0x00001000)
#define RCC_HB2Periph_TIM8               ((uint32_t)0x00002000)
#define RCC_HB2Periph_USART1             ((uint32_t)0x00004000)
#define RCC_HB2Periph_I2C4               ((uint32_t)0x00008000)
#define RCC_HB2Periph_SAI                ((uint32_t)0x00010000)
#define RCC_HB2Periph_SDIO               ((uint32_t)0x00040000)
#define RCC_HB2Periph_TIM9               ((uint32_t)0x00080000)
#define RCC_HB2Periph_TIM10              ((uint32_t)0x00100000)
#define RCC_HB2Periph_TIM11              ((uint32_t)0x00200000)
#define RCC_HB2Periph_TIM12              ((uint32_t)0x00400000)
#define RCC_HB2Periph_OPCM               ((uint32_t)0x00800000)
#define RCC_HB2Periph_DFSDM              ((uint32_t)0x02000000)
#define RCC_HB2Periph_ECDC               ((uint32_t)0x04000000)
#define RCC_HB2Periph_GPHA               ((uint32_t)0x08000000)
#define RCC_HB2Periph_LTDC               ((uint32_t)0x40000000)
#define RCC_HB2Periph_I3C                ((uint32_t)0x80000000)

/* HB1_peripheral */
#define RCC_HB1Periph_TIM2               ((uint32_t)0x00000001)
#define RCC_HB1Periph_TIM3               ((uint32_t)0x00000002)
#define RCC_HB1Periph_TIM4               ((uint32_t)0x00000004)
#define RCC_HB1Periph_TIM5               ((uint32_t)0x00000008)
#define RCC_HB1Periph_TIM6               ((uint32_t)0x00000010)
#define RCC_HB1Periph_TIM7               ((uint32_t)0x00000020)
#define RCC_HB1Periph_USART6             ((uint32_t)0x00000040)
#define RCC_HB1Periph_USART7             ((uint32_t)0x00000080)
#define RCC_HB1Periph_USART8             ((uint32_t)0x00000100)
#define RCC_HB1Periph_LPTIM1             ((uint32_t)0x00000200)
#define RCC_HB1Periph_LPTIM2             ((uint32_t)0x00000400)
#define RCC_HB1Periph_WWDG               ((uint32_t)0x00000800)
#define RCC_HB1Periph_QSPI1              ((uint32_t)0x00001000)
#define RCC_HB1Periph_QSPI2              ((uint32_t)0x00002000)
#define RCC_HB1Periph_SPI2               ((uint32_t)0x00004000)
#define RCC_HB1Periph_SPI3               ((uint32_t)0x00008000)
#define RCC_HB1Periph_SPI4               ((uint32_t)0x00010000)
#define RCC_HB1Periph_USART2             ((uint32_t)0x00020000)
#define RCC_HB1Periph_USART3             ((uint32_t)0x00040000)
#define RCC_HB1Periph_USART4             ((uint32_t)0x00080000)
#define RCC_HB1Periph_USART5             ((uint32_t)0x00100000)
#define RCC_HB1Periph_I2C1               ((uint32_t)0x00200000)
#define RCC_HB1Periph_I2C2               ((uint32_t)0x00400000)
#define RCC_HB1Periph_CAN3               ((uint32_t)0x01000000)
#define RCC_HB1Periph_CAN1               ((uint32_t)0x02000000)
#define RCC_HB1Periph_CAN2               ((uint32_t)0x04000000)
#define RCC_HB1Periph_BKP                ((uint32_t)0x08000000)
#define RCC_HB1Periph_PWR                ((uint32_t)0x10000000)
#define RCC_HB1Periph_DAC                ((uint32_t)0x20000000)
#define RCC_HB1Periph_I2C3               ((uint32_t)0x40000000)
#define RCC_HB1Periph_SWPMI              ((uint32_t)0x80000000)

/* CSSON */
#define RCC_CSSON_DISABLE                (uint8_t)0x00
#define RCC_CSSON_ENABLE                 (uint8_t)0x01

/* CSSHSEDIS */
#define RCC_CSSHSEDIS_DISABLE            (uint8_t)0x00
#define RCC_CSSHSEDIS_ENABLE             (uint8_t)0x01

/* Clock_source_to_output_on_MCO_pin */
#define RCC_MCO_NoClock                  ((uint8_t)0x00)
#define RCC_MCO_SYSCLK                   ((uint8_t)0x04)
#define RCC_MCO_HSI                      ((uint8_t)0x05)
#define RCC_MCO_HSE                      ((uint8_t)0x06)
#define RCC_MCO_PLLCLK_Div2              ((uint8_t)0x07)
#define RCC_MCO_UTMI                     ((uint8_t)0x08)
#define RCC_MCO_USBSSPLL_Div2            ((uint8_t)0x09)
#define RCC_MCO_ETHPLL_Div8              ((uint8_t)0x0A)
#define RCC_MCO_SERDESPLL_Div16          ((uint8_t)0x0B)

/* RCC_Flag */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_USBHSPLLRDY             ((uint8_t)0x35)
#define RCC_FLAG_USBSSPLLRDY             ((uint8_t)0x37)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_ETHPLLRDY               ((uint8_t)0x3B)
#define RCC_FLAG_SERDESPLLRDY            ((uint8_t)0x3D)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LKUPRSTF                ((uint8_t)0x7F)

/*ETH 125M clock source*/
#define RCC_ETH125MSource_PLLCLK         ((uint8_t)0x00)
#define RCC_ETH125MSource_USBSS          ((uint8_t)0x01)
#define RCC_ETH125MSource_ETH_Div4       ((uint8_t)0x02)
#define RCC_ETH125MSource_SERDES_Div8    ((uint8_t)0x03)

/*HSADC clock source*/
#define RCC_HSADCSource_PLLCLK           ((uint8_t)0x00)
#define RCC_HSADCSource_SYSCLK           ((uint8_t)0x01)
#define RCC_HSADCSource_USBHS            ((uint8_t)0x02)
#define RCC_HSADCSource_ETH              ((uint8_t)0x03)

/* I2S2_clock_source */
#define RCC_I2S2CLKSource_SYSCLK         ((uint8_t)0x00)
#define RCC_I2S2CLKSource_PLLCLK         ((uint8_t)0x01)

/* I2S3_clock_source */
#define RCC_I2S3CLKSource_SYSCLK         ((uint8_t)0x00)
#define RCC_I2S3CLKSource_PLLCLK         ((uint8_t)0x01)

/* RNG_clock_source */
#define RCC_RNGCLKSource_SYSCLK          ((uint8_t)0x00)
#define RCC_RNGCLKSource_PLLCLK          ((uint8_t)0x01)

/* USBFS_clock_source */
#define RCC_USBFSCLKSource_PLL           ((uint8_t)0x00)
#define RCC_USBFSCLKSource_USBHSPLL      ((uint8_t)0x01)

/* USBFS_division_factor */
#define RCC_USBFS_Div1                   ((uint8_t)0x00)
#define RCC_USBFS_Div2                   ((uint8_t)0x01)
#define RCC_USBFS_Div3                   ((uint8_t)0x02)
#define RCC_USBFS_Div4                   ((uint8_t)0x03)
#define RCC_USBFS_Div5                   ((uint8_t)0x04)
#define RCC_USBFS_Div6                   ((uint8_t)0x05)
#define RCC_USBFS_Div8                   ((uint8_t)0x06)
#define RCC_USBFS_Div10                  ((uint8_t)0x07)
#define RCC_USBFS_1Div5                  ((uint8_t)0x08)
#define RCC_USBFS_2Div5                  ((uint8_t)0x09)
#define RCC_USBFS_3Div5                  ((uint8_t)0x0A)
#define RCC_USBFS_4Div5                  ((uint8_t)0x0B)
#define RCC_USBFS_5Div5                  ((uint8_t)0x0C)
#define RCC_USBFS_6Div5                  ((uint8_t)0x0D)
#define RCC_USBFS_7Div5                  ((uint8_t)0x0E)
#define RCC_USBFS_9Div5                  ((uint8_t)0x0F)

/* LTDC_clock_source */
#define RCC_LTDCClockSource_PLL          ((uint8_t)0x00)
#define RCC_LTDCClockSource_SERDESPLL    ((uint8_t)0x01)
#define RCC_LTDCClockSource_ETHPLL       ((uint8_t)0x02)
#define RCC_LTDCClockSource_USBHSPLL     ((uint8_t)0x03)

/* LTDC_division_factor */
#define RCC_LTDCClockSource_Div1         ((uint8_t)0x00)
#define RCC_LTDCClockSource_Div2         ((uint8_t)0x01)
#define RCC_LTDCClockSource_Div3         ((uint8_t)0x02)
#define RCC_LTDCClockSource_Div4         ((uint8_t)0x03)
#define RCC_LTDCClockSource_Div5         ((uint8_t)0x04)
#define RCC_LTDCClockSource_Div6         ((uint8_t)0x05)
#define RCC_LTDCClockSource_Div7         ((uint8_t)0x06)
#define RCC_LTDCClockSource_Div8         ((uint8_t)0x07)
#define RCC_LTDCClockSource_Div9         ((uint8_t)0x08)
#define RCC_LTDCClockSource_Div10        ((uint8_t)0x09)
#define RCC_LTDCClockSource_Div11        ((uint8_t)0x0A)
#define RCC_LTDCClockSource_Div12        ((uint8_t)0x0B)
#define RCC_LTDCClockSource_Div13        ((uint8_t)0x0C)
#define RCC_LTDCClockSource_Div14        ((uint8_t)0x0D)
#define RCC_LTDCClockSource_Div15        ((uint8_t)0x0E)
#define RCC_LTDCClockSource_Div16        ((uint8_t)0x0F)
#define RCC_LTDCClockSource_Div17        ((uint8_t)0x10)
#define RCC_LTDCClockSource_Div18        ((uint8_t)0x11)
#define RCC_LTDCClockSource_Div19        ((uint8_t)0x12)
#define RCC_LTDCClockSource_Div20        ((uint8_t)0x13)
#define RCC_LTDCClockSource_Div21        ((uint8_t)0x14)
#define RCC_LTDCClockSource_Div22        ((uint8_t)0x15)
#define RCC_LTDCClockSource_Div23        ((uint8_t)0x16)
#define RCC_LTDCClockSource_Div24        ((uint8_t)0x17)
#define RCC_LTDCClockSource_Div25        ((uint8_t)0x18)
#define RCC_LTDCClockSource_Div26        ((uint8_t)0x19)
#define RCC_LTDCClockSource_Div27        ((uint8_t)0x1A)
#define RCC_LTDCClockSource_Div28        ((uint8_t)0x1B)
#define RCC_LTDCClockSource_Div29        ((uint8_t)0x1C)
#define RCC_LTDCClockSource_Div30        ((uint8_t)0x1D)
#define RCC_LTDCClockSource_Div31        ((uint8_t)0x1E)
#define RCC_LTDCClockSource_Div32        ((uint8_t)0x1F)
#define RCC_LTDCClockSource_Div33        ((uint8_t)0x20)
#define RCC_LTDCClockSource_Div34        ((uint8_t)0x21)
#define RCC_LTDCClockSource_Div35        ((uint8_t)0x22)
#define RCC_LTDCClockSource_Div36        ((uint8_t)0x23)
#define RCC_LTDCClockSource_Div37        ((uint8_t)0x24)
#define RCC_LTDCClockSource_Div38        ((uint8_t)0x25)
#define RCC_LTDCClockSource_Div39        ((uint8_t)0x26)
#define RCC_LTDCClockSource_Div40        ((uint8_t)0x27)
#define RCC_LTDCClockSource_Div41        ((uint8_t)0x28)
#define RCC_LTDCClockSource_Div42        ((uint8_t)0x29)
#define RCC_LTDCClockSource_Div43        ((uint8_t)0x2A)
#define RCC_LTDCClockSource_Div44        ((uint8_t)0x2B)
#define RCC_LTDCClockSource_Div45        ((uint8_t)0x2C)
#define RCC_LTDCClockSource_Div46        ((uint8_t)0x2D)
#define RCC_LTDCClockSource_Div47        ((uint8_t)0x2E)
#define RCC_LTDCClockSource_Div48        ((uint8_t)0x2F)
#define RCC_LTDCClockSource_Div49        ((uint8_t)0x30)
#define RCC_LTDCClockSource_Div50        ((uint8_t)0x31)
#define RCC_LTDCClockSource_Div51        ((uint8_t)0x32)
#define RCC_LTDCClockSource_Div52        ((uint8_t)0x33)
#define RCC_LTDCClockSource_Div53        ((uint8_t)0x34)
#define RCC_LTDCClockSource_Div54        ((uint8_t)0x35)
#define RCC_LTDCClockSource_Div55        ((uint8_t)0x36)
#define RCC_LTDCClockSource_Div56        ((uint8_t)0x37)
#define RCC_LTDCClockSource_Div57        ((uint8_t)0x38)
#define RCC_LTDCClockSource_Div58        ((uint8_t)0x39)
#define RCC_LTDCClockSource_Div59        ((uint8_t)0x3A)
#define RCC_LTDCClockSource_Div60        ((uint8_t)0x3B)
#define RCC_LTDCClockSource_Div61        ((uint8_t)0x3C)
#define RCC_LTDCClockSource_Div62        ((uint8_t)0x3D)
#define RCC_LTDCClockSource_Div63        ((uint8_t)0x3E)
#define RCC_LTDCClockSource_Div64        ((uint8_t)0x3F)

/* UHSIF_clock_source */
#define RCC_UHSIFClockSource_SYSCLK      ((uint8_t)0x00)
#define RCC_UHSIFClockSource_PLL         ((uint8_t)0x01)
#define RCC_UHSIFClockSource_USBHSPLL    ((uint8_t)0x02)
#define RCC_UHSIFClockSource_ETHPLL      ((uint8_t)0x03)

/* UHSIF_division_factor */
#define RCC_UHSIFClockSource_Div1        ((uint8_t)0x00)
#define RCC_UHSIFClockSource_Div2        ((uint8_t)0x01)
#define RCC_UHSIFClockSource_Div3        ((uint8_t)0x02)
#define RCC_UHSIFClockSource_Div4        ((uint8_t)0x03)
#define RCC_UHSIFClockSource_Div5        ((uint8_t)0x04)
#define RCC_UHSIFClockSource_Div6        ((uint8_t)0x05)
#define RCC_UHSIFClockSource_Div7        ((uint8_t)0x06)
#define RCC_UHSIFClockSource_Div8        ((uint8_t)0x07)
#define RCC_UHSIFClockSource_Div9        ((uint8_t)0x08)
#define RCC_UHSIFClockSource_Div10       ((uint8_t)0x09)
#define RCC_UHSIFClockSource_Div11       ((uint8_t)0x0A)
#define RCC_UHSIFClockSource_Div12       ((uint8_t)0x0B)
#define RCC_UHSIFClockSource_Div13       ((uint8_t)0x0C)
#define RCC_UHSIFClockSource_Div14       ((uint8_t)0x0D)
#define RCC_UHSIFClockSource_Div15       ((uint8_t)0x0E)
#define RCC_UHSIFClockSource_Div16       ((uint8_t)0x0F)
#define RCC_UHSIFClockSource_Div17       ((uint8_t)0x10)
#define RCC_UHSIFClockSource_Div18       ((uint8_t)0x11)
#define RCC_UHSIFClockSource_Div19       ((uint8_t)0x12)
#define RCC_UHSIFClockSource_Div20       ((uint8_t)0x13)
#define RCC_UHSIFClockSource_Div21       ((uint8_t)0x14)
#define RCC_UHSIFClockSource_Div22       ((uint8_t)0x15)
#define RCC_UHSIFClockSource_Div23       ((uint8_t)0x16)
#define RCC_UHSIFClockSource_Div24       ((uint8_t)0x17)
#define RCC_UHSIFClockSource_Div25       ((uint8_t)0x18)
#define RCC_UHSIFClockSource_Div26       ((uint8_t)0x19)
#define RCC_UHSIFClockSource_Div27       ((uint8_t)0x1A)
#define RCC_UHSIFClockSource_Div28       ((uint8_t)0x1B)
#define RCC_UHSIFClockSource_Div29       ((uint8_t)0x1C)
#define RCC_UHSIFClockSource_Div30       ((uint8_t)0x1D)
#define RCC_UHSIFClockSource_Div31       ((uint8_t)0x1E)
#define RCC_UHSIFClockSource_Div32       ((uint8_t)0x1F)
#define RCC_UHSIFClockSource_Div33       ((uint8_t)0x20)
#define RCC_UHSIFClockSource_Div34       ((uint8_t)0x21)
#define RCC_UHSIFClockSource_Div35       ((uint8_t)0x22)
#define RCC_UHSIFClockSource_Div36       ((uint8_t)0x23)
#define RCC_UHSIFClockSource_Div37       ((uint8_t)0x24)
#define RCC_UHSIFClockSource_Div38       ((uint8_t)0x25)
#define RCC_UHSIFClockSource_Div39       ((uint8_t)0x26)
#define RCC_UHSIFClockSource_Div40       ((uint8_t)0x27)
#define RCC_UHSIFClockSource_Div41       ((uint8_t)0x28)
#define RCC_UHSIFClockSource_Div42       ((uint8_t)0x29)
#define RCC_UHSIFClockSource_Div43       ((uint8_t)0x2A)
#define RCC_UHSIFClockSource_Div44       ((uint8_t)0x2B)
#define RCC_UHSIFClockSource_Div45       ((uint8_t)0x2C)
#define RCC_UHSIFClockSource_Div46       ((uint8_t)0x2D)
#define RCC_UHSIFClockSource_Div47       ((uint8_t)0x2E)
#define RCC_UHSIFClockSource_Div48       ((uint8_t)0x2F)
#define RCC_UHSIFClockSource_Div49       ((uint8_t)0x30)
#define RCC_UHSIFClockSource_Div50       ((uint8_t)0x31)
#define RCC_UHSIFClockSource_Div51       ((uint8_t)0x32)
#define RCC_UHSIFClockSource_Div52       ((uint8_t)0x33)
#define RCC_UHSIFClockSource_Div53       ((uint8_t)0x34)
#define RCC_UHSIFClockSource_Div54       ((uint8_t)0x35)
#define RCC_UHSIFClockSource_Div55       ((uint8_t)0x36)
#define RCC_UHSIFClockSource_Div56       ((uint8_t)0x37)
#define RCC_UHSIFClockSource_Div57       ((uint8_t)0x38)
#define RCC_UHSIFClockSource_Div58       ((uint8_t)0x39)
#define RCC_UHSIFClockSource_Div59       ((uint8_t)0x3A)
#define RCC_UHSIFClockSource_Div60       ((uint8_t)0x3B)
#define RCC_UHSIFClockSource_Div61       ((uint8_t)0x3C)
#define RCC_UHSIFClockSource_Div62       ((uint8_t)0x3D)
#define RCC_UHSIFClockSource_Div63       ((uint8_t)0x3E)
#define RCC_UHSIFClockSource_Div64       ((uint8_t)0x3F)

/* USBHSPLL_clock_source */
#define RCC_USBHSPLLSource_HSE           ((uint8_t)0x00)
#define RCC_USBHSPLLSource_HSI           ((uint8_t)0x01)
#define RCC_USBHSPLLSource_20METH        ((uint8_t)0x02)
#define RCC_USBHSPLLSource_PLL_CLK_DIV   ((uint8_t)0x03)

/* USBHS PLL Refer clock */
#define RCC_USBHSPLLRefer_25M            ((uint8_t)0x00)
#define RCC_USBHSPLLRefer_20M            ((uint8_t)0x01)
#define RCC_USBHSPLLRefer_24M            ((uint8_t)0x02)
#define RCC_USBHSPLLRefer_32M            ((uint8_t)0x03)

/* USBSS PLL Refer clock */
#define RCC_USBSSPLLRefer_20M            ((uint8_t)0x00)
#define RCC_USBSSPLLRefer_24M            ((uint8_t)0x10)
#define RCC_USBSSPLLRefer_25M            ((uint8_t)0x20)
#define RCC_USBSSPLLRefer_30M            ((uint8_t)0x30)
#define RCC_USBSSPLLRefer_32M            ((uint8_t)0x40)
#define RCC_USBSSPLLRefer_40M            ((uint8_t)0x50)
#define RCC_USBSSPLLRefer_60M            ((uint8_t)0x60)
#define RCC_USBSSPLLRefer_80M            ((uint8_t)0x70)

/* USBHS PLL Source clock Division*/
#define RCC_USBHSPLL_IN_Div1             ((uint32_t)0x0000)
#define RCC_USBHSPLL_IN_Div2             ((uint32_t)0x0100)
#define RCC_USBHSPLL_IN_Div3             ((uint32_t)0x0200)
#define RCC_USBHSPLL_IN_Div4             ((uint32_t)0x0300)
#define RCC_USBHSPLL_IN_Div5             ((uint32_t)0x0400)
#define RCC_USBHSPLL_IN_Div6             ((uint32_t)0x0500)
#define RCC_USBHSPLL_IN_Div7             ((uint32_t)0x0600)
#define RCC_USBHSPLL_IN_Div8             ((uint32_t)0x0700)
#define RCC_USBHSPLL_IN_Div9             ((uint32_t)0x0800)
#define RCC_USBHSPLL_IN_Div10            ((uint32_t)0x0900)
#define RCC_USBHSPLL_IN_Div11            ((uint32_t)0x0A00)
#define RCC_USBHSPLL_IN_Div12            ((uint32_t)0x0B00)
#define RCC_USBHSPLL_IN_Div13            ((uint32_t)0x0C00)
#define RCC_USBHSPLL_IN_Div14            ((uint32_t)0x0D00)
#define RCC_USBHSPLL_IN_Div15            ((uint32_t)0x0E00)
#define RCC_USBHSPLL_IN_Div16            ((uint32_t)0x0F00)
#define RCC_USBHSPLL_IN_Div17            ((uint32_t)0x1000)
#define RCC_USBHSPLL_IN_Div18            ((uint32_t)0x1100)
#define RCC_USBHSPLL_IN_Div19            ((uint32_t)0x1200)
#define RCC_USBHSPLL_IN_Div20            ((uint32_t)0x1300)
#define RCC_USBHSPLL_IN_Div21            ((uint32_t)0x1400)
#define RCC_USBHSPLL_IN_Div22            ((uint32_t)0x1500)
#define RCC_USBHSPLL_IN_Div23            ((uint32_t)0x1600)
#define RCC_USBHSPLL_IN_Div24            ((uint32_t)0x1700)
#define RCC_USBHSPLL_IN_Div25            ((uint32_t)0x1800)
#define RCC_USBHSPLL_IN_Div26            ((uint32_t)0x1900)
#define RCC_USBHSPLL_IN_Div27            ((uint32_t)0x1A00)
#define RCC_USBHSPLL_IN_Div28            ((uint32_t)0x1B00)
#define RCC_USBHSPLL_IN_Div29            ((uint32_t)0x1C00)
#define RCC_USBHSPLL_IN_Div30            ((uint32_t)0x1D00)
#define RCC_USBHSPLL_IN_Div31            ((uint32_t)0x1E00)
#define RCC_USBHSPLL_IN_Div32            ((uint32_t)0x1F00)

/* SERDES PLL clock Mul*/
#define RCC_SERDESPLLMul_25              ((uint32_t)0x0000)
#define RCC_SERDESPLLMul_28              ((uint32_t)0x0001)
#define RCC_SERDESPLLMul_30              ((uint32_t)0x0002)
#define RCC_SERDESPLLMul_32              ((uint32_t)0x0003)
#define RCC_SERDESPLLMul_35              ((uint32_t)0x0004)
#define RCC_SERDESPLLMul_38              ((uint32_t)0x0005)
#define RCC_SERDESPLLMul_40              ((uint32_t)0x0006)
#define RCC_SERDESPLLMul_45              ((uint32_t)0x0007)
#define RCC_SERDESPLLMul_50              ((uint32_t)0x0008)
#define RCC_SERDESPLLMul_56              ((uint32_t)0x0009)
#define RCC_SERDESPLLMul_60              ((uint32_t)0x000A)
#define RCC_SERDESPLLMul_64              ((uint32_t)0x000B)
#define RCC_SERDESPLLMul_70              ((uint32_t)0x000C)
#define RCC_SERDESPLLMul_76              ((uint32_t)0x000D)
#define RCC_SERDESPLLMul_80              ((uint32_t)0x000E)
#define RCC_SERDESPLLMul_90              ((uint32_t)0x000F)

/* ADC_clock_H_Level_Duty_Cycle */
#define RCC_ADC_H_Level_Mode0            ((uint32_t)0x00000000)
#define RCC_ADC_H_Level_Mode1            ((uint32_t)0x40000000)

/* ADC_clock_source */
#define RCC_ADCCLKSource_HCLK            ((uint8_t)0x00)
#define RCC_ADCCLKSource_USBHSPLL        ((uint8_t)0x01)

/* SYSPLL_clock_source */
#define RCC_SYSPLLClockSource_PLL        ((uint8_t)0x00)
#define RCC_SYSPLLClockSource_USBHSPLL   ((uint8_t)0x04)
#define RCC_SYSPLLClockSource_ETHPLL     ((uint8_t)0x05)
#define RCC_SYSPLLClockSource_SERDESPLL  ((uint8_t)0x06)
#define RCC_SYSPLLClockSource_USBSSPLL   ((uint8_t)0x07)

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLDiv, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKSourceDivConfig(uint32_t RCC_SYSCLK, uint32_t RCC_SYSCLKFPRE);
void RCC_TIMClockSourDivConfig(TIM_TypeDef * Tim, uint32_t TIM_Clock_Divx);
void RCC_LPTIMClockSourDivConfig(uint32_t LPTIM_Clock_Divx);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
void RCC_ADCUSBHSPLLCLKAsSourceConfig(uint32_t RCC_PPRE);
void RCC_ADCHCLKCLKAsSourceConfig(uint32_t RCC_PPRE2_DIV, uint32_t RCC_ADC_DIV);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks);
void RCC_HBPeriphClockCmd(uint32_t RCC_HBPeriph, FunctionalState NewState);
void RCC_HB2PeriphClockCmd(uint32_t RCC_HB2Periph, FunctionalState NewState);
void RCC_HB1PeriphClockCmd(uint32_t RCC_HB1Periph, FunctionalState NewState);
void RCC_HBPeriphResetCmd(uint32_t RCC_HBPeriph, FunctionalState NewState);
void RCC_HB2PeriphResetCmd(uint32_t RCC_HB2Periph, FunctionalState NewState);
void RCC_HB1PeriphResetCmd(uint32_t RCC_HB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(uint8_t RCC_CSSHSEDIS, uint8_t RCC_CSSON_State);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);
void RCC_USBHS_PLLCmd(FunctionalState NewState);
void RCC_USBSS_PLLCmd(FunctionalState NewState);
void RCC_ETH_PLLCmd(FunctionalState NewState);
void RCC_SERDES_PLLCmd(FunctionalState NewState);
void RCC_PIPECmd(FunctionalState NewState);
void RCC_UTMIcmd(FunctionalState NewState);
void RCC_RGMIIcmd(FunctionalState NewState);
void RCC_ETH125MCLKConfig(uint32_t RCC_ETH125MMSource);
void RCC_HSADCCLKConfig(uint32_t RCC_HSADCSource);
void RCC_I2S3CLKConfig(uint32_t RCC_I2S3CLKSource);
void RCC_I2S2CLKConfig(uint32_t RCC_I2S2CLKSource);
void RCC_RNGCLKConfig(uint32_t RCC_RNGCLKSource);
void RCC_USBFSCLKConfig(uint32_t RCC_USBFSCLKSource);
void RCC_USBFS48ClockSourceDivConfig(uint32_t RCC_USBFS_DIV);
void RCC_LTDCCLKConfig(uint32_t RCC_LTDCClockSource);
void RCC_LTDCClockSourceDivConfig(uint32_t RCC_LTDCClockSource);
void RCC_UHSIFCLKConfig(uint32_t RCC_UHSIFClockSource);
void RCC_UHSIFClockSourceDivConfig(uint32_t RCC_UHSIFClockSource);
void RCC_USBHSPLLCLKConfig(uint32_t RCC_USBHSPLLSource);
void RCC_USBHSPLLReferConfig(uint32_t RCC_USBHSPLLRefer);
void RCC_USBSSPLLReferConfig(uint32_t RCC_USBSSPLLRefer);
void RCC_USBHSPLLClockSourceDivConfig(uint32_t RCC_USBHSPLL_IN_Div);
void RCC_SERDESPLLMulConfig(uint32_t RCC_SERDESPLLMul);
void RCC_ADCCLKDutyCycleConfig(uint32_t RCC_DutyCycle);
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLKSource);
void RCC_SYSPLLGATEcmd(FunctionalState NewState);
void RCC_SYSPLLConfig(uint32_t RCC_SYSPLLClockSource);


 #ifdef __cplusplus
 }
 #endif

 #endif

