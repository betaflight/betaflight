/**
  ******************************************************************************
  * @file    usbpd_phy_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains phy interface control functions.
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

#define USBPD_HW_C
/* Includes ------------------------------------------------------------------*/
#include "usbpd_devices_conf.h"
#include "usbpd_hw.h"

/* Private typedef -----------------------------------------------------------*/
/* Variable containing ADC conversions results */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

UCPD_TypeDef *USBPD_HW_GetUSPDInstance(uint8_t PortNum)
{
  return UCPD_INSTANCE0;
}

DMA_Channel_TypeDef *USBPD_HW_Init_DMARxInstance(uint8_t PortNum)
{
  /* Enable the clock */
  UCPDDMA_INSTANCE0_CLOCKENABLE_RX;

  LL_DMA_ConfigTransfer(UCPDDMA_INSTANCE0_DMA_RX, UCPDDMA_INSTANCE0_LL_CHANNEL_RX,
                        LL_DMA_SRC_FIXED            |
                        LL_DMA_DEST_INCREMENT       |
                        LL_DMA_SRC_DATAWIDTH_BYTE   |
                        LL_DMA_DEST_DATAWIDTH_BYTE);

  LL_DMA_SetPeriphRequest(UCPDDMA_INSTANCE0_DMA_RX, UCPDDMA_INSTANCE0_LL_CHANNEL_RX, UCPDDMA_INSTANCE0_REQUEST_RX);

  return UCPDDMA_INSTANCE0_CHANNEL_RX;
}

void USBPD_HW_DeInit_DMARxInstance(uint8_t PortNum)
{
  (void)PortNum;
}

DMA_Channel_TypeDef *USBPD_HW_Init_DMATxInstance(uint8_t PortNum)
{
  /* Enable the clock */
  UCPDDMA_INSTANCE0_CLOCKENABLE_TX;

  LL_DMA_ConfigTransfer(UCPDDMA_INSTANCE0_DMA_TX, (uint32_t)UCPDDMA_INSTANCE0_LL_CHANNEL_TX,
                        LL_DMA_SRC_INCREMENT        |
                        LL_DMA_DEST_FIXED           |
                        LL_DMA_SRC_DATAWIDTH_BYTE   |
                        LL_DMA_DEST_DATAWIDTH_BYTE);

  LL_DMA_SetPeriphRequest(UCPDDMA_INSTANCE0_DMA_TX, UCPDDMA_INSTANCE0_LL_CHANNEL_TX, UCPDDMA_INSTANCE0_REQUEST_TX);
  UCPDDMA_INSTANCE0_CHANNEL_TX->CTR2 |= DMA_CTR2_DREQ;

  return UCPDDMA_INSTANCE0_CHANNEL_TX;
}

void USBPD_HW_DeInit_DMATxInstance(uint8_t PortNum)
{
  (void)PortNum;
}

uint32_t USBPD_HW_GetRpResistorValue(uint8_t PortNum)
{
  (void)PortNum;
  return LL_UCPD_RESISTOR_3_0A;
}

void USBPD_HW_SetFRSSignalling(uint8_t PortNum, uint8_t cc)
{
  (void)PortNum;

  /* Configure FRSTX GPIO */
  if (1u == cc)
  {
    /* FRS_TX common */
    UCPDFRS_INSTANCE0_FRSCC1;
  }
  else
  {
    /* FRS_TX common */
    UCPDFRS_INSTANCE0_FRSCC2;
  }
}

