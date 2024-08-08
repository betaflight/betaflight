/**
  *
  * @file    apm32f4xx_dal_uart.h
  * @brief   Header file of UART DAL module.
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
#ifndef APM32F4xx_DAL_UART_H
#define APM32F4xx_DAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup UART
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup UART_Exported_Types UART Exported Types
  * @{
  */

/**
  * @brief UART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CTRL1 register. */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */

  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling */
} UART_InitTypeDef;

/**
  * @brief DAL UART State structures definition
  * @note  DAL UART State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains UART state information related to global Handle management
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7-b6  Error information
  *             00 : No Error
  *             01 : (Not Used)
  *             10 : Timeout
  *             11 : Error
  *          b5     Peripheral initialization status
  *             0  : Reset (Peripheral not initialized)
  *             1  : Init done (Peripheral initialized. DAL UART Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (Peripheral busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     Peripheral initialization status
  *             0  : Reset (Peripheral not initialized)
  *             1  : Init done (Peripheral initialized)
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
  DAL_UART_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  DAL_UART_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  DAL_UART_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  DAL_UART_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  DAL_UART_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  DAL_UART_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  DAL_UART_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  DAL_UART_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} DAL_UART_StateTypeDef;

/**
  * @brief DAL UART Reception type definition
  * @note  DAL UART Reception type value aims to identify which type of Reception is ongoing.
  *        It is expected to admit following values :
  *           DAL_UART_RECEPTION_STANDARD         = 0x00U,
  *           DAL_UART_RECEPTION_TOIDLE           = 0x01U,
  */
typedef uint32_t DAL_UART_RxTypeTypeDef;

/**
  * @brief  UART handle Structure definition
  */
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;        /*!< UART registers base address        */

  UART_InitTypeDef              Init;             /*!< UART communication parameters      */

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< UART Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< UART Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< UART Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< UART Rx Transfer Counter           */

  __IO DAL_UART_RxTypeTypeDef ReceptionType;      /*!< Type of ongoing reception          */

  DMA_HandleTypeDef             *hdmatx;          /*!< UART Tx DMA Handle parameters      */

  DMA_HandleTypeDef             *hdmarx;          /*!< UART Rx DMA Handle parameters      */

  DAL_LockTypeDef               Lock;             /*!< Locking object                     */

  __IO DAL_UART_StateTypeDef    gState;           /*!< UART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref DAL_UART_StateTypeDef */

  __IO DAL_UART_StateTypeDef    RxState;          /*!< UART state information related to Rx operations.
                                                       This parameter can be a value of @ref DAL_UART_StateTypeDef */

  __IO uint32_t                 ErrorCode;        /*!< UART Error code                    */

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Tx Half Complete Callback        */
  void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
  void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Half Complete Callback        */
  void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */
  void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Abort Complete Callback          */
  void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart); /*!< UART Abort Transmit Complete Callback */
  void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);  /*!< UART Abort Receive Complete Callback  */
  void (* WakeupCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Wakeup Callback                  */
  void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos); /*!< UART Reception Event Callback     */

  void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
#endif  /* USE_DAL_UART_REGISTER_CALLBACKS */

} UART_HandleTypeDef;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL UART Callback ID enumeration definition
  */
typedef enum
{
  DAL_UART_TX_HALFCOMPLETE_CB_ID         = 0x00U,    /*!< UART Tx Half Complete Callback ID        */
  DAL_UART_TX_COMPLETE_CB_ID             = 0x01U,    /*!< UART Tx Complete Callback ID             */
  DAL_UART_RX_HALFCOMPLETE_CB_ID         = 0x02U,    /*!< UART Rx Half Complete Callback ID        */
  DAL_UART_RX_COMPLETE_CB_ID             = 0x03U,    /*!< UART Rx Complete Callback ID             */
  DAL_UART_ERROR_CB_ID                   = 0x04U,    /*!< UART Error Callback ID                   */
  DAL_UART_ABORT_COMPLETE_CB_ID          = 0x05U,    /*!< UART Abort Complete Callback ID          */
  DAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID = 0x06U,    /*!< UART Abort Transmit Complete Callback ID */
  DAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID  = 0x07U,    /*!< UART Abort Receive Complete Callback ID  */
  DAL_UART_WAKEUP_CB_ID                  = 0x08U,    /*!< UART Wakeup Callback ID                  */

  DAL_UART_MSPINIT_CB_ID                 = 0x0BU,    /*!< UART MspInit callback ID                 */
  DAL_UART_MSPDEINIT_CB_ID               = 0x0CU     /*!< UART MspDeInit callback ID               */

} DAL_UART_CallbackIDTypeDef;

/**
  * @brief  DAL UART Callback pointer definition
  */
typedef  void (*pUART_CallbackTypeDef)(UART_HandleTypeDef *huart);  /*!< pointer to an UART callback function */
typedef  void (*pUART_RxEventCallbackTypeDef)(struct __UART_HandleTypeDef *huart, uint16_t Pos);   /*!< pointer to a UART Rx Event specific callback function */

#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_Exported_Constants UART Exported Constants
  * @{
  */

/** @defgroup UART_Error_Code UART Error Code
  * @{
  */
#define DAL_UART_ERROR_NONE              0x00000000U   /*!< No error            */
#define DAL_UART_ERROR_PE                0x00000001U   /*!< Parity error        */
#define DAL_UART_ERROR_NE                0x00000002U   /*!< Noise error         */
#define DAL_UART_ERROR_FE                0x00000004U   /*!< Frame error         */
#define DAL_UART_ERROR_ORE               0x00000008U   /*!< Overrun error       */
#define DAL_UART_ERROR_DMA               0x00000010U   /*!< DMA transfer error  */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
#define  DAL_UART_ERROR_INVALID_CALLBACK 0x00000020U   /*!< Invalid Callback error  */
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup UART_Word_Length UART Word Length
  * @{
  */
#define UART_WORDLENGTH_8B                  0x00000000U
#define UART_WORDLENGTH_9B                  ((uint32_t)USART_CTRL1_DBLCFG)
/**
  * @}
  */

/** @defgroup UART_Stop_Bits UART Number of Stop Bits
  * @{
  */
#define UART_STOPBITS_1                     0x00000000U
#define UART_STOPBITS_2                     ((uint32_t)USART_CTRL2_STOPCFG_1)
/**
  * @}
  */

/** @defgroup UART_Parity UART Parity
  * @{
  */
#define UART_PARITY_NONE                    0x00000000U
#define UART_PARITY_EVEN                    ((uint32_t)USART_CTRL1_PCEN)
#define UART_PARITY_ODD                     ((uint32_t)(USART_CTRL1_PCEN | USART_CTRL1_PCFG))
/**
  * @}
  */

/** @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
  * @{
  */
#define UART_HWCONTROL_NONE                  0x00000000U
#define UART_HWCONTROL_RTS                   ((uint32_t)USART_CTRL3_RTSEN)
#define UART_HWCONTROL_CTS                   ((uint32_t)USART_CTRL3_CTSEN)
#define UART_HWCONTROL_RTS_CTS               ((uint32_t)(USART_CTRL3_RTSEN | USART_CTRL3_CTSEN))
/**
  * @}
  */

/** @defgroup UART_Mode UART Transfer Mode
  * @{
  */
#define UART_MODE_RX                        ((uint32_t)USART_CTRL1_RXEN)
#define UART_MODE_TX                        ((uint32_t)USART_CTRL1_TXEN)
#define UART_MODE_TX_RX                     ((uint32_t)(USART_CTRL1_TXEN | USART_CTRL1_RXEN))
/**
  * @}
  */

/** @defgroup UART_State UART State
  * @{
  */
#define UART_STATE_DISABLE                  0x00000000U
#define UART_STATE_ENABLE                   ((uint32_t)USART_CTRL1_UEN)
/**
  * @}
  */

/** @defgroup UART_Over_Sampling UART Over Sampling
  * @{
  */
#define UART_OVERSAMPLING_16                    0x00000000U
#define UART_OVERSAMPLING_8                     ((uint32_t)USART_CTRL1_OSMCFG)
/**
  * @}
  */

/** @defgroup UART_LIN_Break_Detection_Length  UART LIN Break Detection Length
  * @{
  */
#define UART_LINBREAKDETECTLENGTH_10B      0x00000000U
#define UART_LINBREAKDETECTLENGTH_11B      ((uint32_t)USART_CTRL2_LBDLCFG)
/**
  * @}
  */

/** @defgroup UART_WakeUp_functions  UART Wakeup Functions
  * @{
  */
#define UART_WAKEUPMETHOD_IDLELINE                0x00000000U
#define UART_WAKEUPMETHOD_ADDRESSMARK             ((uint32_t)USART_CTRL1_WUPMCFG)
/**
  * @}
  */

/** @defgroup UART_Flags   UART FLags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the SR register
  * @{
  */
#define UART_FLAG_CTS                       ((uint32_t)USART_STS_CTSFLG)
#define UART_FLAG_LBD                       ((uint32_t)USART_STS_LBDFLG)
#define UART_FLAG_TXE                       ((uint32_t)USART_STS_TXBEFLG)
#define UART_FLAG_TC                        ((uint32_t)USART_STS_TXCFLG)
#define UART_FLAG_RXNE                      ((uint32_t)USART_STS_RXBNEFLG)
#define UART_FLAG_IDLE                      ((uint32_t)USART_STS_IDLEFLG)
#define UART_FLAG_ORE                       ((uint32_t)USART_STS_OVREFLG)
#define UART_FLAG_NE                        ((uint32_t)USART_STS_NEFLG)
#define UART_FLAG_FE                        ((uint32_t)USART_STS_FEFLG)
#define UART_FLAG_PE                        ((uint32_t)USART_STS_PEFLG)
/**
  * @}
  */

/** @defgroup UART_Interrupt_definition  UART Interrupt Definitions
  *        Elements values convention: 0xY000XXXX
  *           - XXXX  : Interrupt mask (16 bits) in the Y register
  *           - Y  : Interrupt source register (2bits)
  *                   - 0001: CTRL1 register
  *                   - 0010: CTRL2 register
  *                   - 0011: CTRL3 register
  * @{
  */

#define UART_IT_PE                       ((uint32_t)(UART_CTRL1_REG_INDEX << 28U | USART_CTRL1_PEIEN))
#define UART_IT_TXE                      ((uint32_t)(UART_CTRL1_REG_INDEX << 28U | USART_CTRL1_TXBEIEN))
#define UART_IT_TC                       ((uint32_t)(UART_CTRL1_REG_INDEX << 28U | USART_CTRL1_TXCIEN))
#define UART_IT_RXNE                     ((uint32_t)(UART_CTRL1_REG_INDEX << 28U | USART_CTRL1_RXBNEIEN))
#define UART_IT_IDLE                     ((uint32_t)(UART_CTRL1_REG_INDEX << 28U | USART_CTRL1_IDLEIEN))

#define UART_IT_LBD                      ((uint32_t)(UART_CTRL2_REG_INDEX << 28U | USART_CTRL2_LBDIEN))

#define UART_IT_CTS                      ((uint32_t)(UART_CTRL3_REG_INDEX << 28U | USART_CTRL3_CTSIEN))
#define UART_IT_ERR                      ((uint32_t)(UART_CTRL3_REG_INDEX << 28U | USART_CTRL3_ERRIEN))
/**
  * @}
  */

/** @defgroup UART_RECEPTION_TYPE_Values  UART Reception type values
  * @{
  */
#define DAL_UART_RECEPTION_STANDARD          (0x00000000U)             /*!< Standard reception                       */
#define DAL_UART_RECEPTION_TOIDLE            (0x00000001U)             /*!< Reception till completion or IDLE event  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup UART_Exported_Macros UART Exported Macros
  * @{
  */

/** @brief Reset UART handle gstate & RxState
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
#define __DAL_UART_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                       (__HANDLE__)->gState = DAL_UART_STATE_RESET;      \
                                                       (__HANDLE__)->RxState = DAL_UART_STATE_RESET;     \
                                                       (__HANDLE__)->MspInitCallback = NULL;             \
                                                       (__HANDLE__)->MspDeInitCallback = NULL;           \
                                                     } while(0U)
#else
#define __DAL_UART_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                       (__HANDLE__)->gState = DAL_UART_STATE_RESET;      \
                                                       (__HANDLE__)->RxState = DAL_UART_STATE_RESET;     \
                                                     } while(0U)
#endif /*USE_DAL_UART_REGISTER_CALLBACKS */

/** @brief  Flushes the UART DATA register
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  */
#define __DAL_UART_FLUSH_DRREGISTER(__HANDLE__) ((__HANDLE__)->Instance->DATA)

/** @brief  Checks whether the specified UART flag is set or not.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
  *            @arg UART_FLAG_LBD:  LIN Break detection flag
  *            @arg UART_FLAG_TXE:  Transmit data register empty flag
  *            @arg UART_FLAG_TC:   Transmission Complete flag
  *            @arg UART_FLAG_RXNE: Receive data register not empty flag
  *            @arg UART_FLAG_IDLE: Idle Line detection flag
  *            @arg UART_FLAG_ORE:  Overrun Error flag
  *            @arg UART_FLAG_NE:   Noise Error flag
  *            @arg UART_FLAG_FE:   Framing Error flag
  *            @arg UART_FLAG_PE:   Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))

/** @brief  Clears the specified UART pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @param  __FLAG__ specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg UART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5).
  *            @arg UART_FLAG_LBD:  LIN Break detection flag.
  *            @arg UART_FLAG_TC:   Transmission Complete flag.
  *            @arg UART_FLAG_RXNE: Receive data register not empty flag.
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
#define __DAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->STS = ~(__FLAG__))

/** @brief  Clears the UART PE pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#define __DAL_UART_CLEAR_PEFLAG(__HANDLE__)     \
  do{                                           \
    __IO uint32_t tmpreg = 0x00U;               \
    tmpreg = (__HANDLE__)->Instance->STS;        \
    tmpreg = (__HANDLE__)->Instance->DATA;        \
    UNUSED(tmpreg);                             \
  } while(0U)

/** @brief  Clears the UART FE pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#define __DAL_UART_CLEAR_FEFLAG(__HANDLE__) __DAL_UART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clears the UART NE pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#define __DAL_UART_CLEAR_NEFLAG(__HANDLE__) __DAL_UART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clears the UART ORE pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#define __DAL_UART_CLEAR_OREFLAG(__HANDLE__) __DAL_UART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Clears the UART IDLE pending flag.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @retval None
  */
#define __DAL_UART_CLEAR_IDLEFLAG(__HANDLE__) __DAL_UART_CLEAR_PEFLAG(__HANDLE__)

/** @brief  Enable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @param  __INTERRUPT__ specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
#define __DAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((((__INTERRUPT__) >> 28U) == UART_CTRL1_REG_INDEX)? ((__HANDLE__)->Instance->CTRL1 |= ((__INTERRUPT__) & UART_IT_MASK)): \
                                                           (((__INTERRUPT__) >> 28U) == UART_CTRL2_REG_INDEX)? ((__HANDLE__)->Instance->CTRL2 |= ((__INTERRUPT__) & UART_IT_MASK)): \
                                                           ((__HANDLE__)->Instance->CTRL3 |= ((__INTERRUPT__) & UART_IT_MASK)))

/** @brief  Disable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @param  __INTERRUPT__ specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
#define __DAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((((__INTERRUPT__) >> 28U) == UART_CTRL1_REG_INDEX)? ((__HANDLE__)->Instance->CTRL1 &= ~((__INTERRUPT__) & UART_IT_MASK)): \
                                                           (((__INTERRUPT__) >> 28U) == UART_CTRL2_REG_INDEX)? ((__HANDLE__)->Instance->CTRL2 &= ~((__INTERRUPT__) & UART_IT_MASK)): \
                                                           ((__HANDLE__)->Instance->CTRL3 &= ~ ((__INTERRUPT__) & UART_IT_MASK)))

/** @brief  Checks whether the specified UART interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the USARTx or UARTy peripheral
  *         (USART,UART availability and x,y values depending on device).
  * @param  __IT__ specifies the UART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS: CTS change interrupt (not available for UART4 and UART5)
  *            @arg UART_IT_LBD: LIN Break detection interrupt
  *            @arg UART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:  Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_ERR: Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_UART_GET_IT_SOURCE(__HANDLE__, __IT__) (((((__IT__) >> 28U) == UART_CTRL1_REG_INDEX)? (__HANDLE__)->Instance->CTRL1:(((((uint32_t)(__IT__)) >> 28U) == UART_CTRL2_REG_INDEX)? \
                                                       (__HANDLE__)->Instance->CTRL2 : (__HANDLE__)->Instance->CTRL3)) & (((uint32_t)(__IT__)) & UART_IT_MASK))

/** @brief  Enable CTS flow control
  * @note   This macro allows to enable CTS hardware flow control for a given UART instance,
  *         without need to call DAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of DAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __DAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __DAL_UART_ENABLE(__HANDLE__)).
  * @param  __HANDLE__ specifies the UART Handle.
  *         The Handle Instance can be any USARTx (supporting the HW Flow control feature).
  *         It is used to select the USART peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_UART_HWCONTROL_CTS_ENABLE(__HANDLE__)        \
  do{                                                      \
    ATOMIC_SET_BIT((__HANDLE__)->Instance->CTRL3, USART_CTRL3_CTSEN);  \
    (__HANDLE__)->Init.HwFlowCtl |= USART_CTRL3_CTSEN;        \
  } while(0U)

/** @brief  Disable CTS flow control
  * @note   This macro allows to disable CTS hardware flow control for a given UART instance,
  *         without need to call DAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of DAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __DAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __DAL_UART_ENABLE(__HANDLE__)).
  * @param  __HANDLE__ specifies the UART Handle.
  *         The Handle Instance can be any USARTx (supporting the HW Flow control feature).
  *         It is used to select the USART peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_UART_HWCONTROL_CTS_DISABLE(__HANDLE__)        \
  do{                                                       \
    ATOMIC_CLEAR_BIT((__HANDLE__)->Instance->CTRL3, USART_CTRL3_CTSEN); \
    (__HANDLE__)->Init.HwFlowCtl &= ~(USART_CTRL3_CTSEN);      \
  } while(0U)

/** @brief  Enable RTS flow control
  *         This macro allows to enable RTS hardware flow control for a given UART instance,
  *         without need to call DAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of DAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __DAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __DAL_UART_ENABLE(__HANDLE__)).
  * @param  __HANDLE__ specifies the UART Handle.
  *         The Handle Instance can be any USARTx (supporting the HW Flow control feature).
  *         It is used to select the USART peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_UART_HWCONTROL_RTS_ENABLE(__HANDLE__)       \
  do{                                                     \
    ATOMIC_SET_BIT((__HANDLE__)->Instance->CTRL3, USART_CTRL3_RTSEN); \
    (__HANDLE__)->Init.HwFlowCtl |= USART_CTRL3_RTSEN;       \
  } while(0U)

/** @brief  Disable RTS flow control
  *         This macro allows to disable RTS hardware flow control for a given UART instance,
  *         without need to call DAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of DAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __DAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __DAL_UART_ENABLE(__HANDLE__)).
  * @param  __HANDLE__ specifies the UART Handle.
  *         The Handle Instance can be any USARTx (supporting the HW Flow control feature).
  *         It is used to select the USART peripheral (USART availability and x value depending on device).
  * @retval None
  */
#define __DAL_UART_HWCONTROL_RTS_DISABLE(__HANDLE__)       \
  do{                                                      \
    ATOMIC_CLEAR_BIT((__HANDLE__)->Instance->CTRL3, USART_CTRL3_RTSEN);\
    (__HANDLE__)->Init.HwFlowCtl &= ~(USART_CTRL3_RTSEN);     \
  } while(0U)

/** @brief  Macro to enable the UART's one bit sample method
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __DAL_UART_ONE_BIT_SAMPLE_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3|= USART_CTRL3_SAMCFG)

/** @brief  Macro to disable the UART's one bit sample method
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __DAL_UART_ONE_BIT_SAMPLE_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL3\
                                                       &= (uint16_t)~((uint16_t)USART_CTRL3_SAMCFG))

/** @brief  Enable UART
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __DAL_UART_ENABLE(__HANDLE__)               ((__HANDLE__)->Instance->CTRL1 |=  USART_CTRL1_UEN)

/** @brief  Disable UART
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __DAL_UART_DISABLE(__HANDLE__)              ((__HANDLE__)->Instance->CTRL1 &=  ~USART_CTRL1_UEN)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_Exported_Functions
  * @{
  */

/** @addtogroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  **********************************/
DAL_StatusTypeDef DAL_UART_Init(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
DAL_StatusTypeDef DAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
DAL_StatusTypeDef DAL_UART_DeInit(UART_HandleTypeDef *huart);
void DAL_UART_MspInit(UART_HandleTypeDef *huart);
void DAL_UART_MspDeInit(UART_HandleTypeDef *huart);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_UART_RegisterCallback(UART_HandleTypeDef *huart, DAL_UART_CallbackIDTypeDef CallbackID,
                                            pUART_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_UART_UnRegisterCallback(UART_HandleTypeDef *huart, DAL_UART_CallbackIDTypeDef CallbackID);

DAL_StatusTypeDef DAL_UART_RegisterRxEventCallback(UART_HandleTypeDef *huart, pUART_RxEventCallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_UART_UnRegisterRxEventCallback(UART_HandleTypeDef *huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group2 IO operation functions
  * @{
  */

/* IO operation functions *******************************************************/
DAL_StatusTypeDef DAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_UART_DMAPause(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_DMAResume(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_DMAStop(UART_HandleTypeDef *huart);

DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen,
                                           uint32_t Timeout);
DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/* Transfer Abort functions */
DAL_StatusTypeDef DAL_UART_Abort(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_AbortReceive(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_Abort_IT(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void DAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void DAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void DAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void DAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);

void DAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions  ************************************************/
DAL_StatusTypeDef DAL_LIN_SendBreak(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
DAL_StatusTypeDef DAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);
/**
  * @}
  */

/** @addtogroup UART_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions  **************************************************/
DAL_UART_StateTypeDef DAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              DAL_UART_GetError(UART_HandleTypeDef *huart);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup UART_Private_Constants UART Private Constants
  * @{
  */
/** @brief UART interruptions flag mask
  *
  */
#define UART_IT_MASK                     0x0000FFFFU

#define UART_CTRL1_REG_INDEX               1U
#define UART_CTRL2_REG_INDEX               2U
#define UART_CTRL3_REG_INDEX               3U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup UART_Private_Macros UART Private Macros
  * @{
  */
#define IS_UART_WORD_LENGTH(LENGTH) (((LENGTH) == UART_WORDLENGTH_8B) || \
                                     ((LENGTH) == UART_WORDLENGTH_9B))
#define IS_UART_LIN_WORD_LENGTH(LENGTH) (((LENGTH) == UART_WORDLENGTH_8B))
#define IS_UART_STOPBITS(STOPBITS) (((STOPBITS) == UART_STOPBITS_1) || \
                                    ((STOPBITS) == UART_STOPBITS_2))
#define IS_UART_PARITY(PARITY) (((PARITY) == UART_PARITY_NONE) || \
                                ((PARITY) == UART_PARITY_EVEN) || \
                                ((PARITY) == UART_PARITY_ODD))
#define IS_UART_HARDWARE_FLOW_CONTROL(CONTROL)\
                              (((CONTROL) == UART_HWCONTROL_NONE) || \
                               ((CONTROL) == UART_HWCONTROL_RTS) || \
                               ((CONTROL) == UART_HWCONTROL_CTS) || \
                               ((CONTROL) == UART_HWCONTROL_RTS_CTS))
#define IS_UART_MODE(MODE) ((((MODE) & 0x0000FFF3U) == 0x00U) && ((MODE) != 0x00U))
#define IS_UART_STATE(STATE) (((STATE) == UART_STATE_DISABLE) || \
                              ((STATE) == UART_STATE_ENABLE))
#define IS_UART_OVERSAMPLING(SAMPLING) (((SAMPLING) == UART_OVERSAMPLING_16) || \
                                        ((SAMPLING) == UART_OVERSAMPLING_8))
#define IS_UART_LIN_OVERSAMPLING(SAMPLING) (((SAMPLING) == UART_OVERSAMPLING_16))
#define IS_UART_LIN_BREAK_DETECT_LENGTH(LENGTH) (((LENGTH) == UART_LINBREAKDETECTLENGTH_10B) || \
                                                 ((LENGTH) == UART_LINBREAKDETECTLENGTH_11B))
#define IS_UART_WAKEUPMETHOD(WAKEUP) (((WAKEUP) == UART_WAKEUPMETHOD_IDLELINE) || \
                                      ((WAKEUP) == UART_WAKEUPMETHOD_ADDRESSMARK))
#define IS_UART_BAUDRATE(BAUDRATE) ((BAUDRATE) <= 10500000U)
#define IS_UART_ADDRESS(ADDRESS) ((ADDRESS) <= 0x0FU)

#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)            ((uint32_t)((((uint64_t)(_PCLK_))*25U)/(4U*((uint64_t)(_BAUD_)))))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)        (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100U)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)        ((((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100U)) * 16U)\
                                                         + 50U) / 100U)
/* UART BR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + (UART DIVFRAQ & 0xF0) + (UART DIVFRAQ & 0x0FU) */
#define UART_BR_SAMPLING16(_PCLK_, _BAUD_)            ((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4U) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0U) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0FU))

#define UART_DIV_SAMPLING8(_PCLK_, _BAUD_)             ((uint32_t)((((uint64_t)(_PCLK_))*25U)/(2U*((uint64_t)(_BAUD_)))))
#define UART_DIVMANT_SAMPLING8(_PCLK_, _BAUD_)         (UART_DIV_SAMPLING8((_PCLK_), (_BAUD_))/100U)
#define UART_DIVFRAQ_SAMPLING8(_PCLK_, _BAUD_)         ((((UART_DIV_SAMPLING8((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING8((_PCLK_), (_BAUD_)) * 100U)) * 8U)\
                                                         + 50U) / 100U)
/* UART BR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + ((UART DIVFRAQ & 0xF8) << 1) + (UART DIVFRAQ & 0x07U) */
#define UART_BR_SAMPLING8(_PCLK_, _BAUD_)             ((UART_DIVMANT_SAMPLING8((_PCLK_), (_BAUD_)) << 4U) + \
                                                        ((UART_DIVFRAQ_SAMPLING8((_PCLK_), (_BAUD_)) & 0xF8U) << 1U) + \
                                                        (UART_DIVFRAQ_SAMPLING8((_PCLK_), (_BAUD_)) & 0x07U))

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */

DAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

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

#endif /* APM32F4xx_DAL_UART_H */

