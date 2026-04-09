/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_gpio.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      GPIO firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#ifndef __CH32H417_GPIO_H
#define __CH32H417_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"
                                 
/* Output Maximum frequency selection */
typedef enum
{ 
  GPIO_Speed_Low = 0,
  GPIO_Speed_Medium, 
  GPIO_Speed_High,
  GPIO_Speed_Very_High
}GPIOSpeed_TypeDef;

/* Configuration Mode enumeration */
typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;

/* GPIO Init structure definition */
typedef struct
{
  uint16_t GPIO_Pin;             /* Specifies the GPIO pins to be configured.
                                    This parameter can be any value of @ref GPIO_pins_define */

  GPIOSpeed_TypeDef GPIO_Speed;  /* Specifies the speed for the selected pins.
                                    This parameter can be a value of @ref GPIOSpeed_TypeDef */

  GPIOMode_TypeDef GPIO_Mode;    /* Specifies the operating mode for the selected pins.
                                    This parameter can be a value of @ref GPIOMode_TypeDef */
}GPIO_InitTypeDef;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
	Bit_RESET = 0,
  Bit_SET
}BitAction;

/* GPIO_pins_define */
#define GPIO_Pin_0                         ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                         ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                         ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                         ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                         ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                         ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                         ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                         ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_Pin_8                         ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                         ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_Pin_10                        ((uint16_t)0x0400)  /* Pin 10 selected */
#define GPIO_Pin_11                        ((uint16_t)0x0800)  /* Pin 11 selected */
#define GPIO_Pin_12                        ((uint16_t)0x1000)  /* Pin 12 selected */
#define GPIO_Pin_13                        ((uint16_t)0x2000)  /* Pin 13 selected */
#define GPIO_Pin_14                        ((uint16_t)0x4000)  /* Pin 14 selected */
#define GPIO_Pin_15                        ((uint16_t)0x8000)  /* Pin 15 selected */
#define GPIO_Pin_All                       ((uint16_t)0xFFFF)  /* All pins selected */

/* GPIO_Remap_define */
#define GPIO_Remap_PD0PD1                  ((uint32_t)0x00000001)  /* PD0 and PD1 Alternate Function mapping */
#define GPIO_Remap_ADC1_ETRGINJ            ((uint32_t)0x00000002)  /* ADC1 external trigger injection conversion mapping */
#define GPIO_Remap_ADC1_ETRGREG            ((uint32_t)0x00000004)  /* ADC1 external trigger regular conversion mapping */
#define GPIO_Remap_ADC2_ETRGINJ            ((uint32_t)0x00000008)  /* ADC1 external trigger injection conversion mapping */
#define GPIO_Remap_ADC2_ETRGREG            ((uint32_t)0x00000010)  /* ADC1 external trigger regular conversion mapping */
#define GPIO_PartialRemap_UHSIF_CLK        ((uint32_t)0x00160040)  /* UHSIF CLK Partial Alternate Function mapping */
#define GPIO_PartialRemap1_UHSIF_CLK       ((uint32_t)0x00160080)  /* UHSIF CLK Partial1 Alternate Function mapping */
#define GPIO_FullRemap_UHSIF_CLK           ((uint32_t)0x001600C0)  /* UHSIF CLK Full Alternate Function mapping */
#define GPIO_PartialRemap_UHSIF_PORT       ((uint32_t)0x00180100)  /* UHSIF Port Partial Alternate Function mapping */
#define GPIO_PartialFullRemap_UHSIF_PORT   ((uint32_t)0x00180300)  /* UHSIF Port Full Alternate Function mapping */
#define GPIO_PartialRemap_SDMMC            ((uint32_t)0x001A0400)  /* SDMMC Partial Alternate Function mapping */
#define GPIO_PartialFullRemap_SDMMC        ((uint32_t)0x001A0C00)  /* SDMMC Full Alternate Function mapping */
#define GPIO_Remap_TIM2ITR1                ((uint32_t)0x00001000)  /* TIM2 TRIG Alternate Function mapping */
#define GPIO_Remap_VIO1V8_IO_HSLV          ((uint32_t)0x00200002)  /* VIO(1.8V) GPIO speed configration Alternate Function mapping */
#define GPIO_Remap_VIO3V3_IO_HSLV          ((uint32_t)0x00200004)  /* VIO(3.3V) GPIO speed configration Alternate Function mapping */
#define GPIO_Remap_VDD3V3_IO_HSLV          ((uint32_t)0x00200008)  /* VDD(3.3V) GPIO speed configration Alternate Function mapping */
#define GPIO_Remap_SWJ_Disable             ((uint32_t)0x00300400)  /* SWJ Disable Function mapping */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA               ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB               ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC               ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD               ((uint8_t)0x03)
#define GPIO_PortSourceGPIOE               ((uint8_t)0x04)
#define GPIO_PortSourceGPIOF               ((uint8_t)0x05)
#define GPIO_PortSourceCMP                 ((uint8_t)0x06)

/* GPIO_Pin_sources */
#define GPIO_PinSource0                    ((uint8_t)0x00)
#define GPIO_PinSource1                    ((uint8_t)0x01)
#define GPIO_PinSource2                    ((uint8_t)0x02)
#define GPIO_PinSource3                    ((uint8_t)0x03)
#define GPIO_PinSource4                    ((uint8_t)0x04)
#define GPIO_PinSource5                    ((uint8_t)0x05)
#define GPIO_PinSource6                    ((uint8_t)0x06)
#define GPIO_PinSource7                    ((uint8_t)0x07)
#define GPIO_PinSource8                    ((uint8_t)0x08)
#define GPIO_PinSource9                    ((uint8_t)0x09)
#define GPIO_PinSource10                   ((uint8_t)0x0A)
#define GPIO_PinSource11                   ((uint8_t)0x0B)
#define GPIO_PinSource12                   ((uint8_t)0x0C)
#define GPIO_PinSource13                   ((uint8_t)0x0D)
#define GPIO_PinSource14                   ((uint8_t)0x0E)
#define GPIO_PinSource15                   ((uint8_t)0x0F)

/* GPIO_AF_Define */
#define GPIO_AF0                           ((uint8_t)0x00)
#define GPIO_AF1                           ((uint8_t)0x01)
#define GPIO_AF2                           ((uint8_t)0x02)
#define GPIO_AF3                           ((uint8_t)0x03)
#define GPIO_AF4                           ((uint8_t)0x04)
#define GPIO_AF5                           ((uint8_t)0x05)
#define GPIO_AF6                           ((uint8_t)0x06)
#define GPIO_AF7                           ((uint8_t)0x07)
#define GPIO_AF8                           ((uint8_t)0x08)
#define GPIO_AF9                           ((uint8_t)0x09)
#define GPIO_AF10                          ((uint8_t)0x0A)
#define GPIO_AF11                          ((uint8_t)0x0B)
#define GPIO_AF12                          ((uint8_t)0x0C)
#define GPIO_AF13                          ((uint8_t)0x0D)
#define GPIO_AF14                          ((uint8_t)0x0E)
#define GPIO_AF15                          ((uint8_t)0x0F)

void GPIO_DeInit(GPIO_TypeDef *GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint8_t GPIO_PinSource, uint8_t GPIO_AF);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_IPD_Unused(void);


#ifdef __cplusplus
}
#endif

#endif 







