/**
  *
  * @file    apm32f4xx_dal_usart.h
  * @brief   Header file of USART DAL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_USART_H
#define APM32F4xx_DAL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup USART
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup USART_Exported_Types USART Exported Types
  * @{
  */

/**
  * @brief USART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the Usart communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (8 * (husart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8) + 0.5 */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */

  uint32_t CLKPolarity;               /*!< Specifies the steady state of the serial clock.
                                           This parameter can be a value of @ref USART_Clock_Polarity */

  uint32_t CLKPhase;                  /*!< Specifies the clock transition on which the bit capture is made.
                                           This parameter can be a value of @ref USART_Clock_Phase */

  uint32_t CLKLastBit;                /*!< Specifies whether the clock pulse corresponding to the last transmitted
                                           data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                           This parameter can be a value of @ref USART_Last_Bit */
} USART_InitTypeDef;

/**
  * @brief DAL State structures definition
  */
typedef enum
{
  DAL_USART_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized   */
  DAL_USART_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use */
  DAL_USART_STATE_BUSY              = 0x02U,    /*!< an internal process is ongoing */
  DAL_USART_STATE_BUSY_TX           = 0x12U,    /*!< Data Transmission process is ongoing */
  DAL_USART_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing */
  DAL_USART_STATE_BUSY_TX_RX        = 0x32U,    /*!< Data Transmission Reception process is ongoing */
  DAL_USART_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state */
  DAL_USART_STATE_ERROR             = 0x04U     /*!< Error */
} DAL_USART_StateTypeDef;

/**
  * @brief  USART handle Structure definition
  */
typedef struct __USART_HandleTypeDef
{
  USART_TypeDef                 *Instance;        /*!< USART registers base address        */

  USART_InitTypeDef             Init;             /*!< Usart communication parameters      */

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to Usart Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< Usart Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< Usart Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to Usart Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< Usart Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< Usart Rx Transfer Counter           */

  DMA_HandleTypeDef             *hdmatx;          /*!< Usart Tx DMA Handle parameters      */

  DMA_HandleTypeDef             *hdmarx;          /*!< Usart Rx DMA Handle parameters      */

  DAL_LockTypeDef                Lock;            /*!< Locking object                      */

  __IO DAL_USART_StateTypeDef    State;           /*!< Usart communication state           */

  __IO uint32_t                  ErrorCode;       /*!< USART Error code                    */

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  void (* TxHalfCpltCallback)(struct __USART_HandleTypeDef *husart);        /*!< USART Tx Half Complete Callback        */
  void (* TxCpltCallback)(struct __USART_HandleTypeDef *husart);            /*!< USART Tx Complete Callback             */
  void (* RxHalfCpltCallback)(struct __USART_HandleTypeDef *husart);        /*!< USART Rx Half Complete Callback        */
  void (* RxCpltCallback)(struct __USART_HandleTypeDef *husart);            /*!< USART Rx Complete Callback             */
  void (* TxRxCpltCallback)(struct __USART_HandleTypeDef *husart);          /*!< USART Tx Rx Complete Callback          */
  void (* ErrorCallback)(struct __USART_HandleTypeDef *husart);             /*!< USART Error Callback                   */
  void (* AbortCpltCallback)(struct __USART_HandleTypeDef *husart);         /*!< USART Abort Complete Callback          */

  void (* MspInitCallback)(struct __USART_HandleTypeDef *husart);           /*!< USART Msp Init callback                */
  void (* MspDeInitCallback)(struct __USART_HandleTypeDef *husart);         /*!< USART Msp DeInit callback              */
#endif  /* USE_DAL_USART_REGISTER_CALLBACKS */

} USART_HandleTypeDef;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL USART Callback ID enumeration definition
  */
typedef enum
{
  DAL_USART_TX_HALFCOMPLETE_CB_ID         = 0x00U,    /*!< USART Tx Half Complete Callback ID        */
  DAL_USART_TX_COMPLETE_CB_ID             = 0x01U,    /*!< USART Tx Complete Callback ID             */
  DAL_USART_RX_HALFCOMPLETE_CB_ID         = 0x02U,    /*!< USART Rx Half Complete Callback ID        */
  DAL_USART_RX_COMPLETE_CB_ID             = 0x03U,    /*!< USART Rx Complete Callback ID             */
  DAL_USART_TX_RX_COMPLETE_CB_ID          = 0x04U,    /*!< USART Tx Rx Complete Callback ID          */
  DAL_USART_ERROR_CB_ID                   = 0x05U,    /*!< USART Error Callback ID                   */
  DAL_USART_ABORT_COMPLETE_CB_ID          = 0x06U,    /*!< USART Abort Complete Callback ID          */

  DAL_USART_MSPINIT_CB_ID                 = 0x07U,    /*!< USART MspInit callback ID                 */
  DAL_USART_MSPDEINIT_CB_ID               = 0x08U     /*!< USART MspDeInit callback ID               */

} DAL_USART_CallbackIDTypeDef;

/**
  * @brief  DAL USART Callback pointer definition
  */
typedef  void (*pUSART_CallbackTypeDef)(USART_HandleTypeDef *husart);  /*!< pointer to an USART callback function */

#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USART_Exported_Constants USART Exported Constants
  * @{
  */

/** @defgroup USART_Error_Code USART Error Code
  * @brief    USART Error Code
  * @{
  */
#define DAL_USART_ERROR_NONE             0x00000000U   /*!< No error                */
#define DAL_USART_ERROR_PE               0x00000001U   /*!< Parity error            */
#define DAL_USART_ERROR_NE               0x00000002U   /*!< Noise error             */
#define DAL_USART_ERROR_FE               0x00000004U   /*!< Frame error             */
#define DAL_USART_ERROR_ORE              0x00000008U   /*!< Overrun error           */
#define DAL_USART_ERROR_DMA              0x00000010U   /*!< DMA transfer error      */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
#define DAL_USART_ERROR_INVALID_CALLBACK 0x00000020U    /*!< Invalid Callback error */
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup USART_Word_Length USART Word Length
  * @{
  */
#define USART_WORDLENGTH_8B          0x00000000U
#define USART_WORDLENGTH_9B          ((uint32_t)USART_CTRL1_DBLCFG)
/**
  * @}
  */

/** @defgroup USART_Stop_Bits USART Number of Stop Bits
  * @{
  */
#define USART_STOPBITS_1             0x00000000U
#define USART_STOPBITS_0_5           ((uint32_t)USART_CTRL2_STOPCFG_0)
#define USART_STOPBITS_2             ((uint32_t)USART_CTRL2_STOPCFG_1)
#define USART_STOPBITS_1_5           ((uint32_t)(USART_CTRL2_STOPCFG_0 | USART_CTRL2_STOPCFG_1))
/**
  * @}
  */

/** @defgroup USART_Parity USART Parity
  * @{
  */
#define USART_PARITY_NONE            0x00000000U
#define USART_PARITY_EVEN            ((uint32_t)USART_CTRL1_PCEN)
#define USART_PARITY_ODD             ((uint32_t)(USART_CTRL1_PCEN | USART_CTRL1_PCFG))
/**
  * @}
  */

/** @defgroup USART_Mode USART Mode
  * @{
  */
#define USART_MODE_RX                ((uint32_t)USART_CTRL1_RXEN)
#define USART_MODE_TX                ((uint32_t)USART_CTRL1_TXEN)
#define USART_MODE_TX_RX             ((uint32_t)(USART_CTRL1_TXEN | USART_CTRL1_RXEN))
/**
  * @}
  */

/** @defgroup USART_Clock USART Clock
  * @{
  */
#define USART_CLOCK_DISABLE          0x00000000U
#define USART_CLOCK_ENABLE           ((uint32_t)USART_CTRL2_CLKEN)
/**
  * @}
  */

/** @defgroup USART_Clock_Polarity USART Clock Polarity
  * @{
  */
#define USART_POLARITY_LOW           0x00000000U
#define USART_POLARITY_HIGH          ((uint32_t)USART_CTRL2_CPOL)
/**
  * @}
  */

/** @defgroup USART_Clock_Phase USART Clock Phase
  * @{
  */
#define USART_PHASE_1EDGE            0x00000000U
#define USART_PHASE_2EDGE            ((uint32_t)USART_CTRL2_CPHA)
/**
  * @}
  */

/** @defgroup USART_Last_Bit USART Last Bit
  * @{
  */
#define USART_LASTBIT_DISABLE        0x00000000U
#define USART_LASTBIT_ENABLE         ((uint32_t)USART_CTRL2_LBCPOEN)
/**
  * @}
  */

/** @defgroup USART_NACK_State USART NACK State
  * @{
  */
#define USART_NACK_ENABLE            ((uint32_t)USART_CTRL3_SCNACKEN)
#define USART_NACK_DISABLE           0x00000000U
/**
  * @}
  */

/** @defgroup USART_Flags USART Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the SR register
  * @{
  */
#define USART_FLAG_TXE               ((uint32_t)USART_STS_TXBEFLG)
#define USART_FLAG_TC                ((uint32_t)USART_STS_TXCFLG)
#define USART_FLAG_RXNE              ((uint32_t)USART_STS_RXBNEFLG)
#define USART_FLAG_IDLE              ((uint32_t)USART_STS_IDLEFLG)
#define USART_FLAG_ORE               ((uint32_t)USART_STS_OVREFLG)
#define USART_FLAG_NE                ((uint32_t)USART_STS_NEFLG)
#define USART_FLAG_FE                ((uint32_t)USART_STS_FEFLG)
#define USART_FLAG_PE                ((uint32_t)USART_STS_PEFLG)
/**
  * @}
  */

/** @defgroup USART_Interrupt_definition USART Interrupts Definition
  *        Elements values convention: 0xY000XXXX
  *           - XXXX  : Interrupt mask in the XX register
  *           - Y  : Interrupt source register (2bits)
  *                 - 01: CTRL1 register
  *                 - 10: CTRL2 register
  *                 - 11: CTRL3 register
  * @{
  */
#define USART_IT_PE                  ((uint32_t)(USART_CTRL1_RXENG_INDEX << 28U | USART_CTRL1_PEIEN))
#define USART_IT_TXE                 ((uint32_t)(USART_CTRL1_RXENG_INDEX << 28U | USART_CTRL1_TXBEIEN))
#define USART_IT_TC                  ((uint32_t)(USART_CTRL1_RXENG_INDEX << 28U | USART_CTRL1_TXCIEN))
#define USART_IT_RXNE                ((uint32_t)(USART_CTRL1_RXENG_INDEX << 28U | USART_CTRL1_RXBNEIEN))
#define USART_IT_IDLE                ((uint32_t)(USART_CTRL1_RXENG_INDEX << 28U | USART_CTRL1_IDLEIEN))
#define USART_IT_ERR                 ((uint32_t)(USART_CTRL3_REG_INDEX << 28U | USART_CTRL3_ERRIEN))
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USART_Exported_Macros USART Exported Macros
  * @{
  */

/** @brief Reset USART handle state
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
#define __DAL_USART_RESET_HANDLE_STATE(__HANDLE__)  do{                                            \
                                                      (__HANDLE__)->State = DAL_USART_STATE_RESET; \
                                                      (__HANDLE__)->MspInitCallback = NULL;        \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;      \
                                                    } while(0U)
#else
#define __DAL_USART_RESET_HANDLE_STATE(__HANDLE__)  ((__HANDLE__)->State = DAL_USART_STATE_RESET)
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

/** @brief  Check whether the specified USART flag is set or not.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg USART_FLAG_TXE:  Transmit data register empty flag
  *            @arg USART_FLAG_TC:   Transmission Complete flag
  *            @arg USART_FLAG_RXNE: Receive data register not empty flag
  *            @arg USART_FLAG_IDLE: Idle Line detection flag
  *            @arg USART_FLAG_ORE:  Overrun Error flag
  *            @arg USART_FLAG_NE:   Noise Error flag
  *            @arg USART_FLAG_FE:   Framing Error flag
  *            @arg USART_FLAG_PE:   Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_USART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified USART pending flags.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg USART_FLAG_TC:   Transmission Complete flag.
  *            @arg USART_FLAG_RXNE: Receive data register not empty flag.
  *
  * @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
  *          error) and IDLE (Idle line detected) flags are cleared by software
  *          sequence: a read operation to USART_STS register followed by a read
  *          operation to USART_DATA register.
  * @note   RXNE flag can be also cleared by a read to the USART_DATA register.
  * @note   TC flag can be also cleared by software sequence: a read operation to
  *          USART_STS register followed by a write operation to USART_DATA register.
  * @note   TXE flag is cleared only by a write to the USART_DATA register.
  *
  * @retval None
  */
#define __DAL_USART_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->STS = ~(__FLAG__))

/** @brief  Clear the USART PE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_CLEAR_PEFLAG(__HANDLE__)    \
  do{                                           \
    __IO uint32_t tmpreg = 0x00U;               \
    tmpreg = (__HANDLE__)->Instance->STS;        \
    tmpreg = (__HANDLE__)->Instance->DATA;        \
    UNUSED(tmpreg);                             \
  } while(0U)

/** @brief  Clear the USART FE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_CLEAR_FEFLAG(__HANDLE__) __DAL_USART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the USART NE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_CLEAR_NEFLAG(__HANDLE__) __DAL_USART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the USART ORE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_CLEAR_OREFLAG(__HANDLE__) __DAL_USART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the USART IDLE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_CLEAR_IDLEFLAG(__HANDLE__) __DAL_USART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Enables or disables the specified USART interrupts.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __INTERRUPT__ specifies the USART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:   Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_PE:   Parity Error interrupt
  *            @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
#define __DAL_USART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((((__INTERRUPT__) >> 28U) == USART_CTRL1_RXENG_INDEX)? ((__HANDLE__)->Instance->CTRL1 |= ((__INTERRUPT__) & USART_IT_MASK)): \
                                                            (((__INTERRUPT__) >> 28U) == USART_CTRL2_REG_INDEX)? ((__HANDLE__)->Instance->CTRL2 |= ((__INTERRUPT__) & USART_IT_MASK)): \
                                                            ((__HANDLE__)->Instance->CTRL3 |= ((__INTERRUPT__) & USART_IT_MASK)))
#define __DAL_USART_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((((__INTERRUPT__) >> 28U) == USART_CTRL1_RXENG_INDEX)? ((__HANDLE__)->Instance->CTRL1 &= ~((__INTERRUPT__) & USART_IT_MASK)): \
                                                            (((__INTERRUPT__) >> 28U) == USART_CTRL2_REG_INDEX)? ((__HANDLE__)->Instance->CTRL2 &= ~((__INTERRUPT__) & USART_IT_MASK)): \
                                                            ((__HANDLE__)->Instance->CTRL3 &= ~ ((__INTERRUPT__) & USART_IT_MASK)))

/** @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __IT__ specifies the USART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:  Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_ERR: Error interrupt
  *            @arg USART_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_USART_GET_IT_SOURCE(__HANDLE__, __IT__) (((((__IT__) >> 28U) == USART_CTRL1_RXENG_INDEX)? (__HANDLE__)->Instance->CTRL1:(((((uint32_t)(__IT__)) >> 28U) == USART_CTRL2_REG_INDEX)? \
                                                        (__HANDLE__)->Instance->CTRL2 : (__HANDLE__)->Instance->CTRL3)) & (((uint32_t)(__IT__)) & USART_IT_MASK))

/** @brief  Macro to enable the USART's one bit sample method
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define __DAL_USART_ONE_BIT_SAMPLE_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3 |= USART_CTRL3_SAMCFG)

/** @brief  Macro to disable the USART's one bit sample method
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define __DAL_USART_ONE_BIT_SAMPLE_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3\
                                                        &= (uint16_t)~((uint16_t)USART_CTRL3_SAMCFG))

/** @brief  Enable USART
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_ENABLE(__HANDLE__)               ((__HANDLE__)->Instance->CTRL1 |= USART_CTRL1_UEN)

/** @brief  Disable USART
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_USART_DISABLE(__HANDLE__)              ((__HANDLE__)->Instance->CTRL1 &= ~USART_CTRL1_UEN)

/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @addtogroup USART_Exported_Functions
  * @{
  */

/** @addtogroup USART_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
DAL_StatusTypeDef DAL_USART_Init(USART_HandleTypeDef *husart);
DAL_StatusTypeDef DAL_USART_DeInit(USART_HandleTypeDef *husart);
void DAL_USART_MspInit(USART_HandleTypeDef *husart);
void DAL_USART_MspDeInit(USART_HandleTypeDef *husart);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_USART_RegisterCallback(USART_HandleTypeDef *husart, DAL_USART_CallbackIDTypeDef CallbackID,
                                             pUSART_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_USART_UnRegisterCallback(USART_HandleTypeDef *husart, DAL_USART_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *******************************************************/
DAL_StatusTypeDef DAL_USART_Transmit(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_USART_TransmitReceive(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_USART_Transmit_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size);
DAL_StatusTypeDef DAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
DAL_StatusTypeDef DAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                               uint16_t Size);
DAL_StatusTypeDef DAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size);
DAL_StatusTypeDef DAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
DAL_StatusTypeDef DAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                                uint16_t Size);
DAL_StatusTypeDef DAL_USART_DMAPause(USART_HandleTypeDef *husart);
DAL_StatusTypeDef DAL_USART_DMAResume(USART_HandleTypeDef *husart);
DAL_StatusTypeDef DAL_USART_DMAStop(USART_HandleTypeDef *husart);
/* Transfer Abort functions */
DAL_StatusTypeDef DAL_USART_Abort(USART_HandleTypeDef *husart);
DAL_StatusTypeDef DAL_USART_Abort_IT(USART_HandleTypeDef *husart);

void DAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void DAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void DAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void DAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void DAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void DAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void DAL_USART_ErrorCallback(USART_HandleTypeDef *husart);
void DAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart);
/**
  * @}
  */

/** @addtogroup USART_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  ************************************************/
DAL_USART_StateTypeDef DAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               DAL_USART_GetError(USART_HandleTypeDef *husart);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup USART_Private_Constants USART Private Constants
  * @{
  */
/** @brief USART interruptions flag mask
  *
  */
#define USART_IT_MASK  ((uint32_t) USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN | USART_CTRL1_RXBNEIEN | \
                        USART_CTRL1_IDLEIEN | USART_CTRL2_LBDIEN | USART_CTRL3_CTSIEN | USART_CTRL3_ERRIEN )

#define USART_CTRL1_RXENG_INDEX        1U
#define USART_CTRL2_REG_INDEX          2U
#define USART_CTRL3_REG_INDEX          3U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup USART_Private_Macros USART Private Macros
  * @{
  */
#define IS_USART_NACK_STATE(NACK)    (((NACK) == USART_NACK_ENABLE) || \
                                      ((NACK) == USART_NACK_DISABLE))

#define IS_USART_LASTBIT(LASTBIT)    (((LASTBIT) == USART_LASTBIT_DISABLE) || \
                                      ((LASTBIT) == USART_LASTBIT_ENABLE))

#define IS_USART_PHASE(CPHA)         (((CPHA) == USART_PHASE_1EDGE) || \
                                      ((CPHA) == USART_PHASE_2EDGE))

#define IS_USART_POLARITY(CPOL)      (((CPOL) == USART_POLARITY_LOW) || \
                                      ((CPOL) == USART_POLARITY_HIGH))

#define IS_USART_CLOCK(CLOCK)        (((CLOCK) == USART_CLOCK_DISABLE) || \
                                      ((CLOCK) == USART_CLOCK_ENABLE))

#define IS_USART_WORD_LENGTH(LENGTH) (((LENGTH) == USART_WORDLENGTH_8B) || \
                                      ((LENGTH) == USART_WORDLENGTH_9B))

#define IS_USART_STOPBITS(STOPBITS)  (((STOPBITS) == USART_STOPBITS_1) || \
                                      ((STOPBITS) == USART_STOPBITS_0_5) || \
                                      ((STOPBITS) == USART_STOPBITS_1_5) || \
                                      ((STOPBITS) == USART_STOPBITS_2))

#define IS_USART_PARITY(PARITY)      (((PARITY) == USART_PARITY_NONE) || \
                                      ((PARITY) == USART_PARITY_EVEN) || \
                                      ((PARITY) == USART_PARITY_ODD))

#define IS_USART_MODE(MODE)          ((((MODE) & (~((uint32_t)USART_MODE_TX_RX))) == 0x00U) && ((MODE) != 0x00U))

#define IS_USART_BAUDRATE(BAUDRATE)  ((BAUDRATE) <= 12500000U)

#define USART_DIV(_PCLK_, _BAUD_)      ((uint32_t)((((uint64_t)(_PCLK_))*25U)/(2U*((uint64_t)(_BAUD_)))))

#define USART_DIVMANT(_PCLK_, _BAUD_)  (USART_DIV((_PCLK_), (_BAUD_))/100U)

#define USART_DIVFRAQ(_PCLK_, _BAUD_)  ((((USART_DIV((_PCLK_), (_BAUD_)) - (USART_DIVMANT((_PCLK_), (_BAUD_)) * 100U)) * 8U) + 50U) / 100U)

  /* UART BRR = mantissa + overflow + fraction
              = (UART DIVMANT << 4) + ((UART DIVFRAQ & 0xF8) << 1) + (UART DIVFRAQ & 0x07U) */
              
#define USART_BR(_PCLK_, _BAUD_)      (((USART_DIVMANT((_PCLK_), (_BAUD_)) << 4U) + \
                                         ((USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0xF8U) << 1U)) + \
                                        (USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0x07U))
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup USART_Private_Functions USART Private Functions
  * @{
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_USART_H */

