/**
  *
  * @file    apm32f4xx_dal_smartcard.h
  * @brief   Header file of SMARTCARD DAL module.
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
#ifndef APM32F4xx_DAL_SMARTCARD_H
#define APM32F4xx_DAL_SMARTCARD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup SMARTCARD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Types SMARTCARD Exported Types
  * @{
  */

/**
  * @brief SMARTCARD Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the SmartCard communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (16 * (hsc->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 16) + 0.5 */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref SMARTCARD_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref SMARTCARD_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref SMARTCARD_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits).*/

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref SMARTCARD_Mode */

  uint32_t CLKPolarity;               /*!< Specifies the steady state of the serial clock.
                                           This parameter can be a value of @ref SMARTCARD_Clock_Polarity */

  uint32_t CLKPhase;                  /*!< Specifies the clock transition on which the bit capture is made.
                                           This parameter can be a value of @ref SMARTCARD_Clock_Phase */

  uint32_t CLKLastBit;                /*!< Specifies whether the clock pulse corresponding to the last transmitted
                                           data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                           This parameter can be a value of @ref SMARTCARD_Last_Bit */

  uint32_t Prescaler;                 /*!< Specifies the SmartCard Prescaler value used for dividing the system clock
                                           to provide the smartcard clock. The value given in the register (5 significant bits)
                                           is multiplied by 2 to give the division factor of the source clock frequency.
                                           This parameter can be a value of @ref SMARTCARD_Prescaler */

  uint32_t GuardTime;                 /*!< Specifies the SmartCard Guard Time value in terms of number of baud clocks */

  uint32_t NACKState;                 /*!< Specifies the SmartCard NACK Transmission state.
                                           This parameter can be a value of @ref SMARTCARD_NACK_State */
}SMARTCARD_InitTypeDef;

/**
  * @brief DAL SMARTCARD State structures definition
  * @note  DAL SMARTCARD State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains SMARTCARD state information related to global Handle management
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7-b6  Error information
  *             00 : No Error
  *             01 : (Not Used)
  *             10 : Timeout
  *             11 : Error
  *          b5     IP initialization status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP initialized. DAL SMARTCARD Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (IP busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     IP initialization status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP initialized)
  *          b4-b2  (not used)
  *            xxx : Should be set to 000
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     (not used)
  *             x  : Should be set to 0.
  */
typedef enum
{
  DAL_SMARTCARD_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                        Value is allowed for gState and RxState */
  DAL_SMARTCARD_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                        Value is allowed for gState and RxState */
  DAL_SMARTCARD_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                        Value is allowed for gState only */
  DAL_SMARTCARD_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                        Value is allowed for gState only */
  DAL_SMARTCARD_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                        Value is allowed for RxState only */
  DAL_SMARTCARD_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                        Not to be used for neither gState nor RxState.
                                                        Value is result of combination (Or) between gState and RxState values */
  DAL_SMARTCARD_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                        Value is allowed for gState only */
  DAL_SMARTCARD_STATE_ERROR             = 0xE0U     /*!< Error
                                                        Value is allowed for gState only */
}DAL_SMARTCARD_StateTypeDef;

/**
  * @brief  SMARTCARD handle Structure definition
  */
typedef struct __SMARTCARD_HandleTypeDef
{
  USART_TypeDef                    *Instance;        /*!< USART registers base address */

  SMARTCARD_InitTypeDef            Init;             /*!< SmartCard communication parameters */

  const uint8_t                    *pTxBuffPtr;      /*!< Pointer to SmartCard Tx transfer Buffer */

  uint16_t                         TxXferSize;       /*!< SmartCard Tx Transfer size */

  __IO uint16_t                    TxXferCount;      /*!< SmartCard Tx Transfer Counter */

  uint8_t                          *pRxBuffPtr;      /*!< Pointer to SmartCard Rx transfer Buffer */

  uint16_t                         RxXferSize;       /*!< SmartCard Rx Transfer size */

  __IO uint16_t                    RxXferCount;      /*!< SmartCard Rx Transfer Counter */

  DMA_HandleTypeDef                *hdmatx;          /*!< SmartCard Tx DMA Handle parameters */

  DMA_HandleTypeDef                *hdmarx;          /*!< SmartCard Rx DMA Handle parameters */

  DAL_LockTypeDef                  Lock;             /*!< Locking object */

  __IO DAL_SMARTCARD_StateTypeDef  gState;           /*!< SmartCard state information related to global Handle management
                                                        and also related to Tx operations.
                                                        This parameter can be a value of @ref DAL_SMARTCARD_StateTypeDef */

  __IO DAL_SMARTCARD_StateTypeDef  RxState;          /*!< SmartCard state information related to Rx operations.
                                                        This parameter can be a value of @ref DAL_SMARTCARD_StateTypeDef */

  __IO uint32_t                    ErrorCode;        /*!< SmartCard Error code */

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  void (* TxCpltCallback)(struct __SMARTCARD_HandleTypeDef *hsc);            /*!< SMARTCARD Tx Complete Callback             */

  void (* RxCpltCallback)(struct __SMARTCARD_HandleTypeDef *hsc);            /*!< SMARTCARD Rx Complete Callback             */

  void (* ErrorCallback)(struct __SMARTCARD_HandleTypeDef *hsc);             /*!< SMARTCARD Error Callback                   */

  void (* AbortCpltCallback)(struct __SMARTCARD_HandleTypeDef *hsc);         /*!< SMARTCARD Abort Complete Callback          */

  void (* AbortTransmitCpltCallback)(struct __SMARTCARD_HandleTypeDef *hsc); /*!< SMARTCARD Abort Transmit Complete Callback */

  void (* AbortReceiveCpltCallback)(struct __SMARTCARD_HandleTypeDef *hsc);  /*!< SMARTCARD Abort Receive Complete Callback  */

  void (* MspInitCallback)(struct __SMARTCARD_HandleTypeDef *hsc);           /*!< SMARTCARD Msp Init callback                */

  void (* MspDeInitCallback)(struct __SMARTCARD_HandleTypeDef *hsc);         /*!< SMARTCARD Msp DeInit callback              */
#endif  /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */

} SMARTCARD_HandleTypeDef;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL SMARTCARD Callback ID enumeration definition
  */
typedef enum
{
  DAL_SMARTCARD_TX_COMPLETE_CB_ID             = 0x00U,    /*!< SMARTCARD Tx Complete Callback ID             */
  DAL_SMARTCARD_RX_COMPLETE_CB_ID             = 0x01U,    /*!< SMARTCARD Rx Complete Callback ID             */
  DAL_SMARTCARD_ERROR_CB_ID                   = 0x02U,    /*!< SMARTCARD Error Callback ID                   */
  DAL_SMARTCARD_ABORT_COMPLETE_CB_ID          = 0x03U,    /*!< SMARTCARD Abort Complete Callback ID          */
  DAL_SMARTCARD_ABORT_TRANSMIT_COMPLETE_CB_ID = 0x04U,    /*!< SMARTCARD Abort Transmit Complete Callback ID */
  DAL_SMARTCARD_ABORT_RECEIVE_COMPLETE_CB_ID  = 0x05U,    /*!< SMARTCARD Abort Receive Complete Callback ID  */

  DAL_SMARTCARD_MSPINIT_CB_ID                 = 0x08U,    /*!< SMARTCARD MspInit callback ID                 */
  DAL_SMARTCARD_MSPDEINIT_CB_ID               = 0x09U     /*!< SMARTCARD MspDeInit callback ID               */

} DAL_SMARTCARD_CallbackIDTypeDef;

/**
  * @brief  DAL SMARTCARD Callback pointer definition
  */
typedef  void (*pSMARTCARD_CallbackTypeDef)(SMARTCARD_HandleTypeDef *hsc);  /*!< pointer to an SMARTCARD callback function */

#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Constants  SMARTCARD Exported constants
  * @{
  */

/** @defgroup SMARTCARD_Error_Code SMARTCARD Error Code
  * @{
  */
#define DAL_SMARTCARD_ERROR_NONE             0x00000000U   /*!< No error            */
#define DAL_SMARTCARD_ERROR_PE               0x00000001U   /*!< Parity error        */
#define DAL_SMARTCARD_ERROR_NE               0x00000002U   /*!< Noise error         */
#define DAL_SMARTCARD_ERROR_FE               0x00000004U   /*!< Frame error         */
#define DAL_SMARTCARD_ERROR_ORE              0x00000008U   /*!< Overrun error       */
#define DAL_SMARTCARD_ERROR_DMA              0x00000010U   /*!< DMA transfer error  */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
#define DAL_SMARTCARD_ERROR_INVALID_CALLBACK 0x00000020U   /*!< Invalid Callback error  */
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup SMARTCARD_Word_Length SMARTCARD Word Length
  * @{
  */
#define SMARTCARD_WORDLENGTH_9B             ((uint32_t)USART_CTRL1_DBLCFG)
/**
  * @}
  */

/** @defgroup SMARTCARD_Stop_Bits SMARTCARD Number of Stop Bits
  * @{
  */
#define SMARTCARD_STOPBITS_0_5              ((uint32_t)USART_CTRL2_STOPCFG_0)
#define SMARTCARD_STOPBITS_1_5              ((uint32_t)(USART_CTRL2_STOPCFG_0 | USART_CTRL2_STOPCFG_1))
/**
  * @}
  */

/** @defgroup SMARTCARD_Parity SMARTCARD Parity
  * @{
  */
#define SMARTCARD_PARITY_EVEN               ((uint32_t)USART_CTRL1_PCEN)
#define SMARTCARD_PARITY_ODD                ((uint32_t)(USART_CTRL1_PCEN | USART_CTRL1_PCFG))
/**
  * @}
  */

/** @defgroup SMARTCARD_Mode SMARTCARD Mode
  * @{
  */
#define SMARTCARD_MODE_RX                   ((uint32_t)USART_CTRL1_RXEN)
#define SMARTCARD_MODE_TX                   ((uint32_t)USART_CTRL1_TXEN)
#define SMARTCARD_MODE_TX_RX                ((uint32_t)(USART_CTRL1_TXEN |USART_CTRL1_RXEN))
/**
  * @}
  */

/** @defgroup SMARTCARD_Clock_Polarity SMARTCARD Clock Polarity
  * @{
  */
#define SMARTCARD_POLARITY_LOW              0x00000000U
#define SMARTCARD_POLARITY_HIGH             ((uint32_t)USART_CTRL2_CPOL)
/**
  * @}
  */

/** @defgroup SMARTCARD_Clock_Phase  SMARTCARD Clock Phase
  * @{
  */
#define SMARTCARD_PHASE_1EDGE               0x00000000U
#define SMARTCARD_PHASE_2EDGE               ((uint32_t)USART_CTRL2_CPHA)
/**
  * @}
  */

/** @defgroup SMARTCARD_Last_Bit  SMARTCARD Last Bit
  * @{
  */
#define SMARTCARD_LASTBIT_DISABLE           0x00000000U
#define SMARTCARD_LASTBIT_ENABLE            ((uint32_t)USART_CTRL2_LBCPOEN)
/**
  * @}
  */

/** @defgroup SMARTCARD_NACK_State  SMARTCARD NACK State
  * @{
  */
#define SMARTCARD_NACK_ENABLE               ((uint32_t)USART_CTRL3_SCNACKEN)
#define SMARTCARD_NACK_DISABLE              0x00000000U
/**
  * @}
  */

/** @defgroup SMARTCARD_DMA_Requests   SMARTCARD DMA requests
  * @{
  */
#define SMARTCARD_DMAREQ_TX                 ((uint32_t)USART_CTRL3_DMATXEN)
#define SMARTCARD_DMAREQ_RX                 ((uint32_t)USART_CTRL3_DMARXEN)
/**
  * @}
  */

/** @defgroup SMARTCARD_Prescaler SMARTCARD Prescaler
  * @{
  */
#define SMARTCARD_PRESCALER_SYSCLK_DIV2     0x00000001U          /*!< SYSCLK divided by 2 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV4     0x00000002U          /*!< SYSCLK divided by 4 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV6     0x00000003U          /*!< SYSCLK divided by 6 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV8     0x00000004U          /*!< SYSCLK divided by 8 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV10    0x00000005U          /*!< SYSCLK divided by 10 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV12    0x00000006U          /*!< SYSCLK divided by 12 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV14    0x00000007U          /*!< SYSCLK divided by 14 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV16    0x00000008U          /*!< SYSCLK divided by 16 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV18    0x00000009U          /*!< SYSCLK divided by 18 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV20    0x0000000AU          /*!< SYSCLK divided by 20 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV22    0x0000000BU          /*!< SYSCLK divided by 22 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV24    0x0000000CU          /*!< SYSCLK divided by 24 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV26    0x0000000DU          /*!< SYSCLK divided by 26 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV28    0x0000000EU          /*!< SYSCLK divided by 28 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV30    0x0000000FU          /*!< SYSCLK divided by 30 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV32    0x00000010U          /*!< SYSCLK divided by 32 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV34    0x00000011U          /*!< SYSCLK divided by 34 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV36    0x00000012U          /*!< SYSCLK divided by 36 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV38    0x00000013U          /*!< SYSCLK divided by 38 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV40    0x00000014U          /*!< SYSCLK divided by 40 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV42    0x00000015U          /*!< SYSCLK divided by 42 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV44    0x00000016U          /*!< SYSCLK divided by 44 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV46    0x00000017U          /*!< SYSCLK divided by 46 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV48    0x00000018U          /*!< SYSCLK divided by 48 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV50    0x00000019U          /*!< SYSCLK divided by 50 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV52    0x0000001AU          /*!< SYSCLK divided by 52 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV54    0x0000001BU          /*!< SYSCLK divided by 54 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV56    0x0000001CU          /*!< SYSCLK divided by 56 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV58    0x0000001DU          /*!< SYSCLK divided by 58 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV60    0x0000001EU          /*!< SYSCLK divided by 60 */
#define SMARTCARD_PRESCALER_SYSCLK_DIV62    0x0000001FU          /*!< SYSCLK divided by 62 */
/**
  * @}
  */

/** @defgroup SmartCard_Flags SMARTCARD Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the SR register
  * @{
  */
#define SMARTCARD_FLAG_TXE                  ((uint32_t)USART_STS_TXBEFLG)
#define SMARTCARD_FLAG_TC                   ((uint32_t)USART_STS_TXCFLG)
#define SMARTCARD_FLAG_RXNE                 ((uint32_t)USART_STS_RXBNEFLG)
#define SMARTCARD_FLAG_IDLE                 ((uint32_t)USART_STS_IDLEFLG)
#define SMARTCARD_FLAG_ORE                  ((uint32_t)USART_STS_OVREFLG)
#define SMARTCARD_FLAG_NE                   ((uint32_t)USART_STS_NEFLG)
#define SMARTCARD_FLAG_FE                   ((uint32_t)USART_STS_FEFLG)
#define SMARTCARD_FLAG_PE                   ((uint32_t)USART_STS_PEFLG)
/**
  * @}
  */

/** @defgroup SmartCard_Interrupt_definition SMARTCARD Interrupts Definition
  *        Elements values convention: 0xY000XXXX
  *           - XXXX  : Interrupt mask in the Y register
  *           - Y  : Interrupt source register (2bits)
  *                 - 01: CTRL1 register
  *                 - 11: CTRL3 register
  * @{
  */
#define SMARTCARD_IT_PE                     ((uint32_t)(SMARTCARD_CTRL1_REG_INDEX << 28U | USART_CTRL1_PEIEN))
#define SMARTCARD_IT_TXE                    ((uint32_t)(SMARTCARD_CTRL1_REG_INDEX << 28U | USART_CTRL1_TXBEIEN))
#define SMARTCARD_IT_TC                     ((uint32_t)(SMARTCARD_CTRL1_REG_INDEX << 28U | USART_CTRL1_TXCIEN))
#define SMARTCARD_IT_RXNE                   ((uint32_t)(SMARTCARD_CTRL1_REG_INDEX << 28U | USART_CTRL1_RXBNEIEN))
#define SMARTCARD_IT_IDLE                   ((uint32_t)(SMARTCARD_CTRL1_REG_INDEX << 28U | USART_CTRL1_IDLEIEN))
#define SMARTCARD_IT_ERR                    ((uint32_t)(SMARTCARD_CTRL3_REG_INDEX << 28U | USART_CTRL3_ERRIEN))
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Macros SMARTCARD Exported Macros
  * @{
  */

/** @brief Reset SMARTCARD handle gstate & RxState
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#if USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1
#define __DAL_SMARTCARD_RESET_HANDLE_STATE(__HANDLE__)  do{                                                       \
                                                           (__HANDLE__)->gState = DAL_SMARTCARD_STATE_RESET;      \
                                                           (__HANDLE__)->RxState = DAL_SMARTCARD_STATE_RESET;     \
                                                           (__HANDLE__)->MspInitCallback = NULL;                  \
                                                           (__HANDLE__)->MspDeInitCallback = NULL;                \
                                                          } while(0U)
#else
#define __DAL_SMARTCARD_RESET_HANDLE_STATE(__HANDLE__)  do{                                                       \
                                                           (__HANDLE__)->gState = DAL_SMARTCARD_STATE_RESET;      \
                                                           (__HANDLE__)->RxState = DAL_SMARTCARD_STATE_RESET;     \
                                                          } while(0U)
#endif /*USE_DAL_SMARTCARD_REGISTER_CALLBACKS  */

/** @brief  Flush the Smartcard DATA register
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_FLUSH_DRREGISTER(__HANDLE__) ((__HANDLE__)->Instance->DATA)

/** @brief  Check whether the specified Smartcard flag is set or not.
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg SMARTCARD_FLAG_TXE:  Transmit data register empty flag
  *            @arg SMARTCARD_FLAG_TC:   Transmission Complete flag
  *            @arg SMARTCARD_FLAG_RXNE: Receive data register not empty flag
  *            @arg SMARTCARD_FLAG_IDLE: Idle Line detection flag
  *            @arg SMARTCARD_FLAG_ORE:  Overrun Error flag
  *            @arg SMARTCARD_FLAG_NE:   Noise Error flag
  *            @arg SMARTCARD_FLAG_FE:   Framing Error flag
  *            @arg SMARTCARD_FLAG_PE:   Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_SMARTCARD_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified Smartcard pending flags.
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg SMARTCARD_FLAG_TC:   Transmission Complete flag.
  *            @arg SMARTCARD_FLAG_RXNE: Receive data register not empty flag.
  *
  * @note   PE (Parity error), FE (Framing error), NE (Noise error) and ORE (Overrun
  *          error) flags are cleared by software sequence: a read operation to
  *          USART_STS register followed by a read operation to USART_DATA register.
  * @note   RXNE flag can be also cleared by a read to the USART_DATA register.
  * @note   TC flag can be also cleared by software sequence: a read operation to
  *          USART_STS register followed by a write operation to USART_DATA register.
  * @note   TXE flag is cleared only by a write to the USART_DATA register.
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->STS = ~(__FLAG__))

/** @brief  Clear the SMARTCARD PE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_PEFLAG(__HANDLE__)  \
    do{                                           \
      __IO uint32_t tmpreg = 0x00U;               \
      tmpreg = (__HANDLE__)->Instance->STS;        \
      tmpreg = (__HANDLE__)->Instance->DATA;        \
      UNUSED(tmpreg);                             \
    } while(0U)

/** @brief  Clear the SMARTCARD FE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_FEFLAG(__HANDLE__) __DAL_SMARTCARD_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the SMARTCARD NE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_NEFLAG(__HANDLE__) __DAL_SMARTCARD_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the SMARTCARD ORE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_OREFLAG(__HANDLE__) __DAL_SMARTCARD_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clear the SMARTCARD IDLE pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_CLEAR_IDLEFLAG(__HANDLE__) __DAL_SMARTCARD_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Enable the specified SmartCard interrupt.
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __INTERRUPT__ specifies the SMARTCARD interrupt to enable.
  *          This parameter can be one of the following values:
  *            @arg SMARTCARD_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg SMARTCARD_IT_TC:   Transmission complete interrupt
  *            @arg SMARTCARD_IT_RXNE: Receive Data register not empty interrupt
  *            @arg SMARTCARD_IT_IDLE: Idle line detection interrupt
  *            @arg SMARTCARD_IT_PE:   Parity Error interrupt
  *            @arg SMARTCARD_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
#define __DAL_SMARTCARD_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((((__INTERRUPT__) >> 28U) == SMARTCARD_CTRL1_REG_INDEX)? ((__HANDLE__)->Instance->CTRL1 |= ((__INTERRUPT__) & SMARTCARD_IT_MASK)): \
                                                                 ((__HANDLE__)->Instance->CTRL3 |= ((__INTERRUPT__) & SMARTCARD_IT_MASK)))

/** @brief  Disable the specified SmartCard interrupt.
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @param  __INTERRUPT__ specifies the SMARTCARD interrupt to disable.
  *          This parameter can be one of the following values:
  *            @arg SMARTCARD_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg SMARTCARD_IT_TC:   Transmission complete interrupt
  *            @arg SMARTCARD_IT_RXNE: Receive Data register not empty interrupt
  *            @arg SMARTCARD_IT_IDLE: Idle line detection interrupt
  *            @arg SMARTCARD_IT_PE:   Parity Error interrupt
  *            @arg SMARTCARD_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
#define __DAL_SMARTCARD_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((((__INTERRUPT__) >> 28U) == SMARTCARD_CTRL1_REG_INDEX)? ((__HANDLE__)->Instance->CTRL1 &= ~((__INTERRUPT__) & SMARTCARD_IT_MASK)): \
                                                                 ((__HANDLE__)->Instance->CTRL3 &= ~ ((__INTERRUPT__) & SMARTCARD_IT_MASK)))

/** @brief  Checks whether the specified SmartCard interrupt has occurred or not.
  * @param  __HANDLE__ specifies the SmartCard Handle.
  * @param  __IT__ specifies the SMARTCARD interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SMARTCARD_IT_TXE: Transmit Data Register empty interrupt
  *            @arg SMARTCARD_IT_TC:  Transmission complete interrupt
  *            @arg SMARTCARD_IT_RXNE: Receive Data register not empty interrupt
  *            @arg SMARTCARD_IT_IDLE: Idle line detection interrupt
  *            @arg SMARTCARD_IT_ERR: Error interrupt
  *            @arg SMARTCARD_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_SMARTCARD_GET_IT_SOURCE(__HANDLE__, __IT__) (((((__IT__) >> 28U) == SMARTCARD_CTRL1_REG_INDEX)? (__HANDLE__)->Instance->CTRL1: (__HANDLE__)->Instance->CTRL3) & (((uint32_t)(__IT__)) & SMARTCARD_IT_MASK))

/** @brief  Macro to enable the SMARTCARD's one bit sample method
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  * @retval None
  */
#define __DAL_SMARTCARD_ONE_BIT_SAMPLE_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3|= USART_CTRL3_SAMCFG)

/** @brief  Macro to disable the SMARTCARD's one bit sample method
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  * @retval None
  */
#define __DAL_SMARTCARD_ONE_BIT_SAMPLE_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3 &= (uint16_t)~((uint16_t)USART_CTRL3_SAMCFG))

/** @brief  Enable the USART associated to the SMARTCARD Handle
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL1 |=  USART_CTRL1_UEN)

/** @brief  Disable the USART associated to the SMARTCARD Handle
  * @param  __HANDLE__ specifies the SMARTCARD Handle.
  *         SMARTCARD Handle selects the USARTx peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_SMARTCARD_DISABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL1 &=  ~USART_CTRL1_UEN)

/** @brief  Macros to enable the SmartCard DMA request.
  * @param  __HANDLE__ specifies the SmartCard Handle.
  * @param  __REQUEST__ specifies the SmartCard DMA request.
  *          This parameter can be one of the following values:
  *            @arg SMARTCARD_DMAREQ_TX: SmartCard DMA transmit request
  *            @arg SMARTCARD_DMAREQ_RX: SmartCard DMA receive request
  * @retval None
  */
#define __DAL_SMARTCARD_DMA_REQUEST_ENABLE(__HANDLE__, __REQUEST__)    ((__HANDLE__)->Instance->CTRL3 |=  (__REQUEST__))

/** @brief  Macros to disable the SmartCard DMA request.
  * @param  __HANDLE__ specifies the SmartCard Handle.
  * @param  __REQUEST__ specifies the SmartCard DMA request.
  *          This parameter can be one of the following values:
  *            @arg SMARTCARD_DMAREQ_TX: SmartCard DMA transmit request
  *            @arg SMARTCARD_DMAREQ_RX: SmartCard DMA receive request
  * @retval None
  */
#define __DAL_SMARTCARD_DMA_REQUEST_DISABLE(__HANDLE__, __REQUEST__)   ((__HANDLE__)->Instance->CTRL3 &=  ~(__REQUEST__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SMARTCARD_Exported_Functions
  * @{
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
DAL_StatusTypeDef DAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_ReInit(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsc);
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
DAL_StatusTypeDef DAL_SMARTCARD_RegisterCallback(SMARTCARD_HandleTypeDef *hsc, DAL_SMARTCARD_CallbackIDTypeDef CallbackID, pSMARTCARD_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_SMARTCARD_UnRegisterCallback(SMARTCARD_HandleTypeDef *hsc, DAL_SMARTCARD_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *******************************************************/
DAL_StatusTypeDef DAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
/* Transfer Abort functions */
DAL_StatusTypeDef DAL_SMARTCARD_Abort(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_AbortTransmit(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_AbortReceive(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_Abort_IT(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_AbortTransmit_IT(SMARTCARD_HandleTypeDef *hsc);
DAL_StatusTypeDef DAL_SMARTCARD_AbortReceive_IT(SMARTCARD_HandleTypeDef *hsc);

void DAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_AbortCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_AbortTransmitCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void DAL_SMARTCARD_AbortReceiveCpltCallback(SMARTCARD_HandleTypeDef *hsc);
/**
  * @}
  */

/** @addtogroup SMARTCARD_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  **************************************************/
DAL_SMARTCARD_StateTypeDef DAL_SMARTCARD_GetState(SMARTCARD_HandleTypeDef *hsc);
uint32_t DAL_SMARTCARD_GetError(SMARTCARD_HandleTypeDef *hsc);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup SMARTCARD_Private_Constants SMARTCARD Private Constants
  * @{
  */

/** @brief SMARTCARD interruptions flag mask
  *
  */
#define SMARTCARD_IT_MASK                   ((uint32_t) USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN | USART_CTRL1_RXBNEIEN | \
                                                        USART_CTRL1_IDLEIEN | USART_CTRL3_ERRIEN )

#define SMARTCARD_CTRL1_REG_INDEX             1U
#define SMARTCARD_CTRL3_REG_INDEX             3U
/**
  * @}
  */

/* Private macros --------------------------------------------------------*/
/** @defgroup SMARTCARD_Private_Macros   SMARTCARD Private Macros
  * @{
  */
#define IS_SMARTCARD_WORD_LENGTH(LENGTH)    ((LENGTH) == SMARTCARD_WORDLENGTH_9B)
#define IS_SMARTCARD_STOPBITS(STOPBITS)     (((STOPBITS) == SMARTCARD_STOPBITS_0_5) || \
                                             ((STOPBITS) == SMARTCARD_STOPBITS_1_5))
#define IS_SMARTCARD_PARITY(PARITY)         (((PARITY) == SMARTCARD_PARITY_EVEN) || \
                                             ((PARITY) == SMARTCARD_PARITY_ODD))
#define IS_SMARTCARD_MODE(MODE)             ((((MODE) & 0x0000FFF3U) == 0x00U) && ((MODE) != 0x000000U))
#define IS_SMARTCARD_POLARITY(CPOL)         (((CPOL) == SMARTCARD_POLARITY_LOW) || ((CPOL) == SMARTCARD_POLARITY_HIGH))
#define IS_SMARTCARD_PHASE(CPHA)            (((CPHA) == SMARTCARD_PHASE_1EDGE) || ((CPHA) == SMARTCARD_PHASE_2EDGE))
#define IS_SMARTCARD_LASTBIT(LASTBIT)       (((LASTBIT) == SMARTCARD_LASTBIT_DISABLE) || \
                                             ((LASTBIT) == SMARTCARD_LASTBIT_ENABLE))
#define IS_SMARTCARD_NACK_STATE(NACK)       (((NACK) == SMARTCARD_NACK_ENABLE) || \
                                             ((NACK) == SMARTCARD_NACK_DISABLE))
#define IS_SMARTCARD_BAUDRATE(BAUDRATE)     ((BAUDRATE) < 10500001U)

#define SMARTCARD_DIV(__PCLK__, __BAUD__)                ((uint32_t)((((uint64_t)(__PCLK__))*25U)/(4U*((uint64_t)(__BAUD__)))))
#define SMARTCARD_DIVMANT(__PCLK__, __BAUD__)            (SMARTCARD_DIV((__PCLK__), (__BAUD__))/100U)
#define SMARTCARD_DIVFRAQ(__PCLK__, __BAUD__)            ((((SMARTCARD_DIV((__PCLK__), (__BAUD__)) - (SMARTCARD_DIVMANT((__PCLK__), (__BAUD__)) * 100U)) * 16U) + 50U) / 100U)
/* SMARTCARD BR = mantissa + overflow + fraction
            = (SMARTCARD DIVMANT << 4) + (SMARTCARD DIVFRAQ & 0xF0) + (SMARTCARD DIVFRAQ & 0x0FU) */
#define SMARTCARD_BR(__PCLK__, __BAUD__)       (((SMARTCARD_DIVMANT((__PCLK__), (__BAUD__)) << 4U) + \
                                                 (SMARTCARD_DIVFRAQ((__PCLK__), (__BAUD__)) & 0xF0U)) + \
                                                 (SMARTCARD_DIVFRAQ((__PCLK__), (__BAUD__)) & 0x0FU))

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup SMARTCARD_Private_Functions SMARTCARD Private Functions
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

#endif /* APM32F4xx_DAL_SMARTCARD_H */

