/**
  ******************************************************************************
  * @file    usbpd_devices_conf.h
  * @author  MCD Application Team
  * @brief   This file contains the device define.
  ******************************************************************************
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

#ifndef USBPD_DEVICE_CONF_H
#define USBPD_DEVICE_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_ll_adc.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_dma.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_ucpd.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_tim.h"

/* Following include file may be replaced with the BSP UBSPD PWR header file */
#warning "Update for the series"

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
      usbpd_hw.c
-------------------------------------------------------------------------------*/

/* defined used to configure function : USBPD_HW_GetUSPDInstance */
#define UCPD_INSTANCE0 UCPD1

/* defined used to configure function : USBPD_HW_Init_DMARxInstance,USBPD_HW_DeInit_DMARxInstance */
#define UCPDDMA_INSTANCE0_CLOCKENABLE_RX  \
  do                                      \
  {                                       \
    __HAL_RCC_GPDMA1_CLK_ENABLE();        \
  } while(0)

#define UCPDDMA_INSTANCE0_DMA_RX  GPDMA1

#define UCPDDMA_INSTANCE0_REQUEST_RX   LL_GPDMA1_REQUEST_USBPD_RX

#define UCPDDMA_INSTANCE0_LL_CHANNEL_RX   LL_DMA_CHANNEL_5

#define UCPDDMA_INSTANCE0_CHANNEL_RX   GPDMA1_Channel5

/* defined used to configure function : USBPD_HW_Init_DMATxInstance, USBPD_HW_DeInit_DMATxInstance */
#define UCPDDMA_INSTANCE0_CLOCKENABLE_TX  \
  do                                      \
  {                                       \
    __HAL_RCC_GPDMA1_CLK_ENABLE();        \
  } while(0)

#define UCPDDMA_INSTANCE0_DMA_TX  GPDMA1

#define UCPDDMA_INSTANCE0_REQUEST_TX   LL_GPDMA1_REQUEST_USBPD_TX

#define UCPDDMA_INSTANCE0_LL_CHANNEL_TX   LL_DMA_CHANNEL_3

#define UCPDDMA_INSTANCE0_CHANNEL_TX   GPDMA1_Channel3

/* Defines used to configure  USBPD_HW_SetFRSSignalling */
#define UCPDFRS_INSTANCE0_FRSCC1                                                      \
  do                                                                                  \
  {                                                                                   \
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);                              \
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);                \
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_6);                       \
  } while(0)

#define UCPDFRS_INSTANCE0_FRSCC2    UCPDFRS_INSTANCE0_FRSCC1 /* Only one common FRSCC pin is available on this series */

#define UCPD_INSTANCE0_ENABLEIRQ               \
  do                                           \
  {                                            \
    NVIC_SetPriority(UCPD1_IRQn,2);           \
    NVIC_EnableIRQ(UCPD1_IRQn);               \
  } while(0)

/* -----------------------------------------------------------------------------
      Definitions for timer service feature
-------------------------------------------------------------------------------*/
#define TIMX                           TIM2
#define TIMX_CLK_ENABLE                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2)
#define TIMX_CLK_DISABLE               LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2)
#define TIMX_IRQ                       TIM2_IRQn
#define TIMX_CHANNEL_CH1               LL_TIM_CHANNEL_CH1
#define TIMX_CHANNEL_CH2               LL_TIM_CHANNEL_CH2
#define TIMX_CHANNEL_CH3               LL_TIM_CHANNEL_CH3
#define TIMX_CHANNEL_CH4               LL_TIM_CHANNEL_CH4
#define TIMX_CHANNEL1_SETEVENT                                         \
  do                                                                   \
  {                                                                    \
    LL_TIM_OC_SetCompareCH1(TIMX, (TimeUs + TIMX->CNT) % TIM_MAX_TIME);\
    LL_TIM_ClearFlag_CC1(TIMX);                                        \
  } while(0)
#define TIMX_CHANNEL2_SETEVENT                                         \
  do                                                                   \
  {                                                                    \
    LL_TIM_OC_SetCompareCH2(TIMX, (TimeUs + TIMX->CNT) % TIM_MAX_TIME);\
    LL_TIM_ClearFlag_CC2(TIMX);                                        \
  }while(0)
#define TIMX_CHANNEL3_SETEVENT                                         \
  do                                                                   \
  {                                                                    \
    LL_TIM_OC_SetCompareCH3(TIMX, (TimeUs + TIMX->CNT) % TIM_MAX_TIME);\
    LL_TIM_ClearFlag_CC3(TIMX);                                        \
  } while(0)
#define TIMX_CHANNEL4_SETEVENT                                         \
  do                                                                   \
  {                                                                    \
    LL_TIM_OC_SetCompareCH4(TIMX, (TimeUs + TIMX->CNT) % TIM_MAX_TIME);\
    LL_TIM_ClearFlag_CC4(TIMX);                                        \
  } while(0)
#define TIMX_CHANNEL1_GETFLAG          LL_TIM_IsActiveFlag_CC1
#define TIMX_CHANNEL2_GETFLAG          LL_TIM_IsActiveFlag_CC2
#define TIMX_CHANNEL3_GETFLAG          LL_TIM_IsActiveFlag_CC3
#define TIMX_CHANNEL4_GETFLAG          LL_TIM_IsActiveFlag_CC4

#ifdef __cplusplus
}
#endif

#endif /* USBPD_DEVICE_CONF_H */

