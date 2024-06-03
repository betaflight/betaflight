/**
  ******************************************************************************
  * @file    usbpd_hw.h
  * @author  MCD Application Team
  * @brief   This file contains interface hw control.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef USBPD_HW_H
#define USBPD_HW_H
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Variable containing ADC conversions results */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
UCPD_TypeDef *USBPD_HW_GetUSPDInstance(uint8_t PortNum);
DMA_Channel_TypeDef *USBPD_HW_Init_DMARxInstance(uint8_t PortNum);
void USBPD_HW_DeInit_DMARxInstance(uint8_t PortNum);
DMA_Channel_TypeDef *USBPD_HW_Init_DMATxInstance(uint8_t PortNum);
void USBPD_HW_DeInit_DMATxInstance(uint8_t PortNum);
uint32_t USBPD_HW_GetRpResistorValue(uint8_t Portnum);
void USBPD_HW_SetFRSSignalling(uint8_t Portnum, uint8_t cc);
#endif /* USBPD_BSP_HW_H */

