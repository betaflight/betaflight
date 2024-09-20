/**
  *
  * @file    apm32f4xx_ddl_usart.c
  * @brief   USART DDL module driver.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */

#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_usart.h"
#include "apm32f4xx_ddl_rcm.h"
#include "apm32f4xx_ddl_bus.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (USART1) || defined (USART2) || defined (USART3) || defined (USART6) || defined (UART4) || defined (UART5) || defined (UART7) || defined (UART8) || defined (UART9) || defined (UART10)

/** @addtogroup USART_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup USART_DDL_Private_Constants
  * @{
  */

/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/** @addtogroup USART_DDL_Private_Macros
  * @{
  */

/* __BAUDRATE__ The maximum Baud Rate is derived from the maximum clock available
 *              divided by the smallest oversampling used on the USART (i.e. 8)    */
#define IS_DDL_USART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) <= 12500000U)

/* __VALUE__ In case of oversampling by 16 and 8, BRR content must be greater than or equal to 16d. */
#define IS_DDL_USART_BR_MIN(__VALUE__) ((__VALUE__) >= 16U)

#define IS_DDL_USART_DIRECTION(__VALUE__) (((__VALUE__) == DDL_USART_DIRECTION_NONE) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_RX) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_TX) \
                                          || ((__VALUE__) == DDL_USART_DIRECTION_TX_RX))

#define IS_DDL_USART_PARITY(__VALUE__) (((__VALUE__) == DDL_USART_PARITY_NONE) \
                                       || ((__VALUE__) == DDL_USART_PARITY_EVEN) \
                                       || ((__VALUE__) == DDL_USART_PARITY_ODD))

#define IS_DDL_USART_DATAWIDTH(__VALUE__) (((__VALUE__) == DDL_USART_DATAWIDTH_8B) \
                                          || ((__VALUE__) == DDL_USART_DATAWIDTH_9B))

#define IS_DDL_USART_OVERSAMPLING(__VALUE__) (((__VALUE__) == DDL_USART_OVERSAMPLING_16) \
                                             || ((__VALUE__) == DDL_USART_OVERSAMPLING_8))

#define IS_DDL_USART_LASTBITCLKOUTPUT(__VALUE__) (((__VALUE__) == DDL_USART_LASTCLKPULSE_NO_OUTPUT) \
                                                 || ((__VALUE__) == DDL_USART_LASTCLKPULSE_OUTPUT))

#define IS_DDL_USART_CLOCKPHASE(__VALUE__) (((__VALUE__) == DDL_USART_PHASE_1EDGE) \
                                           || ((__VALUE__) == DDL_USART_PHASE_2EDGE))

#define IS_DDL_USART_CLOCKPOLARITY(__VALUE__) (((__VALUE__) == DDL_USART_POLARITY_LOW) \
                                              || ((__VALUE__) == DDL_USART_POLARITY_HIGH))

#define IS_DDL_USART_CLOCKOUTPUT(__VALUE__) (((__VALUE__) == DDL_USART_CLOCK_DISABLE) \
                                            || ((__VALUE__) == DDL_USART_CLOCK_ENABLE))

#define IS_DDL_USART_STOPBITS(__VALUE__) (((__VALUE__) == DDL_USART_STOPBITS_0_5) \
                                         || ((__VALUE__) == DDL_USART_STOPBITS_1) \
                                         || ((__VALUE__) == DDL_USART_STOPBITS_1_5) \
                                         || ((__VALUE__) == DDL_USART_STOPBITS_2))

#define IS_DDL_USART_HWCONTROL(__VALUE__) (((__VALUE__) == DDL_USART_HWCONTROL_NONE) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_RTS) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_CTS) \
                                          || ((__VALUE__) == DDL_USART_HWCONTROL_RTS_CTS))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USART_DDL_Exported_Functions
  * @{
  */

/** @addtogroup USART_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize USART registers (Registers restored to their default values).
  * @param  USARTx USART Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers are de-initialized
  *          - ERROR: USART registers are not de-initialized
  */
ErrorStatus DDL_USART_DeInit(USART_TypeDef *USARTx)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(USARTx));

  if (USARTx == USART1)
  {
    /* Force reset of USART clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_USART1);

    /* Release reset of USART clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_USART1);
  }
  else if (USARTx == USART2)
  {
    /* Force reset of USART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_USART2);

    /* Release reset of USART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_USART2);
  }
#if defined(USART3)
  else if (USARTx == USART3)
  {
    /* Force reset of USART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_USART3);

    /* Release reset of USART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_USART3);
  }
#endif /* USART3 */
#if defined(USART6)
  else if (USARTx == USART6)
  {
    /* Force reset of USART clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_USART6);

    /* Release reset of USART clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_USART6);
  }
#endif /* USART6 */
#if defined(UART4)
  else if (USARTx == UART4)
  {
    /* Force reset of UART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_UART4);

    /* Release reset of UART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_UART4);
  }
#endif /* UART4 */
#if defined(UART5)
  else if (USARTx == UART5)
  {
    /* Force reset of UART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_UART5);

    /* Release reset of UART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_UART5);
  }
#endif /* UART5 */
#if defined(UART7)
  else if (USARTx == UART7)
  {
    /* Force reset of UART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_UART7);

    /* Release reset of UART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_UART7);
  }
#endif /* UART7 */
#if defined(UART8)
  else if (USARTx == UART8)
  {
    /* Force reset of UART clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_UART8);

    /* Release reset of UART clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_UART8);
  }
#endif /* UART8 */
#if defined(UART9)
  else if (USARTx == UART9)
  {
    /* Force reset of UART clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_UART9);

    /* Release reset of UART clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_UART9);
  }
#endif /* UART9 */
#if defined(UART10)
  else if (USARTx == UART10)
  {
    /* Force reset of UART clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_UART10);

    /* Release reset of UART clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_UART10);
  }
#endif /* UART10 */
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief  Initialize USART registers according to the specified
  *         parameters in USART_InitStruct.
  * @note   As some bits in USART configuration registers can only be written when the USART is disabled (USART_CTRL1_UEN bit =0),
  *         USART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in USART_InitStruct BaudRate field, should be valid (different from 0).
  * @param  USARTx USART Instance
  * @param  USART_InitStruct pointer to a DDL_USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers are initialized according to USART_InitStruct content
  *          - ERROR: Problem occurred during USART Registers initialization
  */
ErrorStatus DDL_USART_Init(USART_TypeDef *USARTx, DDL_USART_InitTypeDef *USART_InitStruct)
{
  ErrorStatus status = ERROR;
  uint32_t periphclk = DDL_RCM_PERIPH_FREQUENCY_NO;
  DDL_RCM_ClocksTypeDef rcc_clocks;

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(USARTx));
  ASSERT_PARAM(IS_DDL_USART_BAUDRATE(USART_InitStruct->BaudRate));
  ASSERT_PARAM(IS_DDL_USART_DATAWIDTH(USART_InitStruct->DataWidth));
  ASSERT_PARAM(IS_DDL_USART_STOPBITS(USART_InitStruct->StopBits));
  ASSERT_PARAM(IS_DDL_USART_PARITY(USART_InitStruct->Parity));
  ASSERT_PARAM(IS_DDL_USART_DIRECTION(USART_InitStruct->TransferDirection));
  ASSERT_PARAM(IS_DDL_USART_HWCONTROL(USART_InitStruct->HardwareFlowControl));
  ASSERT_PARAM(IS_DDL_USART_OVERSAMPLING(USART_InitStruct->OverSampling));

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CTRLx registers */
  if (DDL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CTRL1 Configuration -----------------------
     * Configure USARTx CTRL1 (USART Word Length, Parity, Mode and Oversampling bits) with parameters:
     * - DataWidth:          USART_CTRL1_DBLCFG bits according to USART_InitStruct->DataWidth value
     * - Parity:             USART_CTRL1_PCEN, USART_CTRL1_PCFG bits according to USART_InitStruct->Parity value
     * - TransferDirection:  USART_CTRL1_TXEN, USART_CTRL1_RXEN bits according to USART_InitStruct->TransferDirection value
     * - Oversampling:       USART_CTRL1_OSMCFG bit according to USART_InitStruct->OverSampling value.
     */
    MODIFY_REG(USARTx->CTRL1,
               (USART_CTRL1_DBLCFG | USART_CTRL1_PCEN | USART_CTRL1_PCFG |
                USART_CTRL1_TXEN | USART_CTRL1_RXEN | USART_CTRL1_OSMCFG),
               (USART_InitStruct->DataWidth | USART_InitStruct->Parity |
                USART_InitStruct->TransferDirection | USART_InitStruct->OverSampling));

    /*---------------------------- USART CTRL2 Configuration -----------------------
     * Configure USARTx CTRL2 (Stop bits) with parameters:
     * - Stop Bits:          USART_CTRL2_STOPCFG bits according to USART_InitStruct->StopBits value.
     * - CLKEN, CPOL, CPHA and LBCL bits are to be configured using DDL_USART_ClockInit().
     */
    DDL_USART_SetStopBitsLength(USARTx, USART_InitStruct->StopBits);

    /*---------------------------- USART CTRL3 Configuration -----------------------
     * Configure USARTx CTRL3 (Hardware Flow Control) with parameters:
     * - HardwareFlowControl: USART_CTRL3_RTSEN, USART_CTRL3_CTSEN bits according to USART_InitStruct->HardwareFlowControl value.
     */
    DDL_USART_SetHWFlowCtrl(USARTx, USART_InitStruct->HardwareFlowControl);

    /*---------------------------- USART BRR Configuration -----------------------
     * Retrieve Clock frequency used for USART Peripheral
     */
    DDL_RCM_GetSystemClocksFreq(&rcc_clocks);
    if (USARTx == USART1)
    {
      periphclk = rcc_clocks.PCLK2_Frequency;
    }
    else if (USARTx == USART2)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#if defined(USART3)
    else if (USARTx == USART3)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#endif /* USART3 */
#if defined(USART6)
    else if (USARTx == USART6)
    {
      periphclk = rcc_clocks.PCLK2_Frequency;
    }
#endif /* USART6 */
#if defined(UART4)
    else if (USARTx == UART4)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#endif /* UART4 */
#if defined(UART5)
    else if (USARTx == UART5)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#endif /* UART5 */
#if defined(UART7)
    else if (USARTx == UART7)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#endif /* UART7 */
#if defined(UART8)
    else if (USARTx == UART8)
    {
      periphclk = rcc_clocks.PCLK1_Frequency;
    }
#endif /* UART8 */
#if defined(UART9)
    else if (USARTx == UART9)
    {
      periphclk = rcc_clocks.PCLK2_Frequency;
    }
#endif /* UART9 */
#if defined(UART10)
    else if (USARTx == UART10)
    {
      periphclk = rcc_clocks.PCLK2_Frequency;
    }
#endif /* UART10 */
    else
    {
      /* Nothing to do, as error code is already assigned to ERROR value */
    }

    /* Configure the USART Baud Rate :
       - valid baud rate value (different from 0) is required
       - Peripheral clock as returned by RCM service, should be valid (different from 0).
    */
    if ((periphclk != DDL_RCM_PERIPH_FREQUENCY_NO)
        && (USART_InitStruct->BaudRate != 0U))
    {
      status = SUCCESS;
      DDL_USART_SetBaudRate(USARTx,
                           periphclk,
                           USART_InitStruct->OverSampling,
                           USART_InitStruct->BaudRate);

      /* Check BR is greater than or equal to 16d */
      ASSERT_PARAM(IS_DDL_USART_BR_MIN(USARTx->BR));
    }
  }
  /* Endif (=> USART not in Disabled state => return ERROR) */

  return (status);
}

/**
  * @brief Set each @ref DDL_USART_InitTypeDef field to default value.
  * @param USART_InitStruct Pointer to a @ref DDL_USART_InitTypeDef structure
  *                         whose fields will be set to default values.
  * @retval None
  */

void DDL_USART_StructInit(DDL_USART_InitTypeDef *USART_InitStruct)
{
  /* Set USART_InitStruct fields to default values */
  USART_InitStruct->BaudRate            = 9600U;
  USART_InitStruct->DataWidth           = DDL_USART_DATAWIDTH_8B;
  USART_InitStruct->StopBits            = DDL_USART_STOPBITS_1;
  USART_InitStruct->Parity              = DDL_USART_PARITY_NONE ;
  USART_InitStruct->TransferDirection   = DDL_USART_DIRECTION_TX_RX;
  USART_InitStruct->HardwareFlowControl = DDL_USART_HWCONTROL_NONE;
  USART_InitStruct->OverSampling        = DDL_USART_OVERSAMPLING_16;
}

/**
  * @brief  Initialize USART Clock related settings according to the
  *         specified parameters in the USART_ClockInitStruct.
  * @note   As some bits in USART configuration registers can only be written when the USART is disabled (USART_CTRL1_UEN bit =0),
  *         USART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  USARTx USART Instance
  * @param  USART_ClockInitStruct Pointer to a @ref DDL_USART_ClockInitTypeDef structure
  *         that contains the Clock configuration information for the specified USART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: USART registers related to Clock settings are initialized according to USART_ClockInitStruct content
  *          - ERROR: Problem occurred during USART Registers initialization
  */
ErrorStatus DDL_USART_ClockInit(USART_TypeDef *USARTx, DDL_USART_ClockInitTypeDef *USART_ClockInitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check USART Instance and Clock signal output parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(USARTx));
  ASSERT_PARAM(IS_DDL_USART_CLOCKOUTPUT(USART_ClockInitStruct->ClockOutput));

  /* USART needs to be in disabled state, in order to be able to configure some bits in
     CTRLx registers */
  if (DDL_USART_IsEnabled(USARTx) == 0U)
  {
    /*---------------------------- USART CTRL2 Configuration -----------------------*/
    /* If Clock signal has to be output */
    if (USART_ClockInitStruct->ClockOutput == DDL_USART_CLOCK_DISABLE)
    {
      /* Deactivate Clock signal delivery :
       * - Disable Clock Output:        USART_CTRL2_CLKEN cleared
       */
      DDL_USART_DisableSCLKOutput(USARTx);
    }
    else
    {
      /* Ensure USART instance is USART capable */
      ASSERT_PARAM(IS_USART_INSTANCE(USARTx));

      /* Check clock related parameters */
      ASSERT_PARAM(IS_DDL_USART_CLOCKPOLARITY(USART_ClockInitStruct->ClockPolarity));
      ASSERT_PARAM(IS_DDL_USART_CLOCKPHASE(USART_ClockInitStruct->ClockPhase));
      ASSERT_PARAM(IS_DDL_USART_LASTBITCLKOUTPUT(USART_ClockInitStruct->LastBitClockPulse));

      /*---------------------------- USART CTRL2 Configuration -----------------------
       * Configure USARTx CTRL2 (Clock signal related bits) with parameters:
       * - Enable Clock Output:         USART_CTRL2_CLKEN set
       * - Clock Polarity:              USART_CTRL2_CPOL bit according to USART_ClockInitStruct->ClockPolarity value
       * - Clock Phase:                 USART_CTRL2_CPHA bit according to USART_ClockInitStruct->ClockPhase value
       * - Last Bit Clock Pulse Output: USART_CTRL2_LBCPOEN bit according to USART_ClockInitStruct->LastBitClockPulse value.
       */
      MODIFY_REG(USARTx->CTRL2,
                 USART_CTRL2_CLKEN | USART_CTRL2_CPHA | USART_CTRL2_CPOL | USART_CTRL2_LBCPOEN,
                 USART_CTRL2_CLKEN | USART_ClockInitStruct->ClockPolarity |
                 USART_ClockInitStruct->ClockPhase | USART_ClockInitStruct->LastBitClockPulse);
    }
  }
  /* Else (USART not in Disabled state => return ERROR */
  else
  {
    status = ERROR;
  }

  return (status);
}

/**
  * @brief Set each field of a @ref DDL_USART_ClockInitTypeDef type structure to default value.
  * @param USART_ClockInitStruct Pointer to a @ref DDL_USART_ClockInitTypeDef structure
  *                              whose fields will be set to default values.
  * @retval None
  */
void DDL_USART_ClockStructInit(DDL_USART_ClockInitTypeDef *USART_ClockInitStruct)
{
  /* Set DDL_USART_ClockInitStruct fields with default values */
  USART_ClockInitStruct->ClockOutput       = DDL_USART_CLOCK_DISABLE;
  USART_ClockInitStruct->ClockPolarity     = DDL_USART_POLARITY_LOW;            /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
  USART_ClockInitStruct->ClockPhase        = DDL_USART_PHASE_1EDGE;             /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
  USART_ClockInitStruct->LastBitClockPulse = DDL_USART_LASTCLKPULSE_NO_OUTPUT;  /* Not relevant when ClockOutput = DDL_USART_CLOCK_DISABLE */
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* USART1 || USART2 || USART3 || USART6 || UART4 || UART5 || UART7 || UART8 || UART9 || UART10 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */


