 /**
  ******************************************************************************
  * @file     um324xx_hal_uart.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-11
  * @brief   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_UART_H__
#define __UM324XX_HAL_UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
 * @{
 */

/** @addtogroup UART
 * @{
 */

/**
  * @brief  UART0 protocol frame definition
  */
typedef struct
{                         
  uint32_t BaudRate;            /*!< This member configures the UART communication baud rate.
                                     The baud rate is computed using the following formula:
                                     (PCLKx + (huart->Init.BaudRate/2)) / huart->Init.BaudRate */

  uint32_t Parity;              /*!< Specifies the parity mode.
                                     This parameter can be a value of @ref UART_Parity
                                     @note When parity is enabled, the computed parity is inserted
                                     at the MSB position of the transmitted data*/

  uint32_t Mode;                /*!< Select three uart modes: TX or TEST */

} UART_InitTypeDef;

/**
  * @brief  UART0 State definition
  */
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  HAL_UART_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  HAL_UART_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  HAL_UART_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} HAL_UART_StateTypeDef;

/**
  * @brief  UART0 handle Structure definition
  */
typedef struct __UART_HandleTypeDef
{
  UART_TypeDef                 *Instance;         /*!< UART registers base address        */

  UART_InitTypeDef              Init;             /*!< UART communication parameters      */

  const uint8_t                *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< UART Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< UART Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< UART Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< UART Rx Transfer Counter           */

  HAL_LockTypeDef               Lock;             /*!< Locking object                     */

  __IO HAL_UART_StateTypeDef    gState;           /*!< UART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO HAL_UART_StateTypeDef    RxState;          /*!< UART state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t                 ErrorCode;        /*!< UART Error code                    */

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
  void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */

  void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
#endif  /* USE_HAL_UART_REGISTER_CALLBACKS */
} UART_HandleTypeDef;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL UART Callback ID enumeration definition
  */
typedef enum
{
  HAL_UART_TX_COMPLETE_CB_ID             = 0x01U,    /*!< UART Tx Complete Callback ID             */
  HAL_UART_RX_COMPLETE_CB_ID             = 0x02U,    /*!< UART Rx Complete Callback ID             */
  HAL_UART_RX_HALF_COMPLETE_CB_ID        = 0x03U,    /*!< UART Rx Half Complete Callback ID        */
  HAL_UART_RX_FULL_COMPLETE_CB_ID        = 0x04U,    /*!< UART Rx Full Complete Callback ID        */
  HAL_UART_ERROR_CB_ID                   = 0x05U,    /*!< UART Error Callback ID                   */

  HAL_UART_MSPINIT_CB_ID                 = 0x0BU,    /*!< UART MspInit callback ID                 */
  HAL_UART_MSPDEINIT_CB_ID               = 0x0CU     /*!< UART MspDeInit callback ID               */

} HAL_UART_CallbackIDTypeDef;

/**
  * @brief  HAL UART Callback pointer definition
  */
typedef  void (*pUART_CallbackTypeDef)(UART_HandleTypeDef *huart);  /*!< pointer to an UART callback function */

#endif /* USE_HAL_UART_REGISTER_CALLBACKS */



/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_Exported_Constants UART Exported Constants
  * @{
  */

/** @defgroup UART_Error_Code UART Error Code
  * @{
  */
#define HAL_UART_ERROR_NONE              0x00000000U   /*!< No error            */
#define HAL_UART_ERROR_PE                0x00000001U   /*!< Parity error        */
#define HAL_UART_ERROR_ORF               0x00000002U   /*!< Overflow error       */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
#define  HAL_UART_ERROR_INVALID_CALLBACK 0x00000020U   /*!< Invalid Callback error  */
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup UART_Flags   UART FLags
  * @{
  */
#define UART_FLAG_FIFO_NO_EMPTY             UART_ISR_FIFO_NE    /*!< FIFO is not empty flag    */
#define UART_FLAG_FIFO_HALF_FULL            UART_ISR_FIFO_HF    /*!< FIFO half full flag       */
#define UART_FLAG_FIFO_FULL                 UART_ISR_FIFO_FU    /*!< FIFO full flag            */
#define UART_FLAG_RXFIFO_ORF_ERR            UART_ISR_FIFO_OV    /*!< Rx-FIFO overflow error    */
#define UART_FLAG_TXEND                     UART_ISR_TXEND      /*!< UART tx Completion        */
#define UART_FLAG_PARITY_ERR                UART_ISR_TRE        /*!< The parity check error    */

/**
  * @}
  */

/** @defgroup UART_Interrupt_definition  UART Interrupt Definitions
  * @{
  */
#define UART_IT_FIFO_NO_EMPTY              UART_IER_FIFO_EN     /*!< FIFO is not empty Interrupt        */
#define UART_IT_FIFO_HALF_FULL             UART_IER_FIFO_HFEN   /*!< FIFO half full Interrupt           */
#define UART_IT_FIFO_FULL                  UART_IER_FIFO_FUEN   /*!< FIFO full Interrupt                */
#define UART_IT_FIFO_ORF                   UART_IER_FIFO_OVEN   /*!< Rx-FIFO overflow error Interrupt   */
#define UART_IT_TXEND                      UART_IER_TXEND_EN    /*!< UART tx Completion Interrupt       */
#define UART_IT_PARITY_ERR                 UART_IER_TRE_EN      /*!< The parity check erro Interrupt    */
#define UART_IT_NONE                       0x00000000U          /*!< NONE Interrupt                     */

/**
  * @}
  */

/** @defgroup UART_Parity UART Parity
  * @{
  */
#define UART_PARITY_NONE                    UART_CR_UART_PD
#define UART_PARITY_EVEN                    0x00000000U
#define UART_PARITY_ODD                     ((uint32_t)(UART_CR_ODD_EN | 0x00000000U))
/**
  * @}
  */
  
/** @defgroup UART_Mode UART Mode
  * @{
  */
#define UART_MODE_LB                        ((uint32_t)UART_CR_UART_LB)
#define UART_MODE_NOMAL                     ((uint32_t)UART_CR_TRS)

/**
  * @}
  */
  
/** @defgroup UART_Flush UART Flush
  * @{
  */
#define UART_FIFO_NOTCLEAR                  0x00000000U
#define UART_FIFO_CLEAR                     ((uint32_t)UART_CR_FLUSH)

/**
  * @}
  */

/**
  * @}
  */

/*==============Instance Operation Macro==================*/

/** @brief  Checks whether the specified UART flag is set or not.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the  UART peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_FLAG_FIFO_NO_EMPTY:  Receiving FIFO is not empty
  *            @arg UART_FLAG_FIFO_HALF_FULL: Receive FIFO half full
  *            @arg UART_FLAG_FIFO_FULL:      Receive FIFO full
  *            @arg UART_FLAG_RXFIFO_OV_ERR:  Receive FIFO overflow
  *            @arg UART_FLAG_TXEND:          Completion of sending
  *            @arg UART_FLAG_PARITY_ERR:     The parity check error
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__)       (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))

/** @brief  Checks whether the specified UART flag is set or not.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the  UART peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_FLAG_FIFO_NO_EMPTY:  Receiving FIFO is not empty
  *            @arg UART_FLAG_FIFO_HALF_FULL: Receive FIFO half full
  *            @arg UART_FLAG_FIFO_FULL:      Receive FIFO full
  *            @arg UART_FLAG_RXFIFO_OV_ERR:  Receive FIFO overflow
  *            @arg UART_FLAG_TXEND:          Completion of sending
  *            @arg UART_FLAG_PARITY_ERR:     The parity check error
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__)       CLEAR_BIT((__HANDLE__)->Instance->ISR ,(__FLAG__))

/** @brief  Enable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the UART0/2/3 peripheral
  * @param  __INTERRUPT__ specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_FIFO_NO_EMPTY:  Receiving FIFO is not empty interrupt
  *            @arg UART_IT_FIFO_HALF_FULL: Receive FIFO half full interrupt
  *            @arg UART_IT_FIFO_FULL:      Receive FIFO full interrupt
  *            @arg UART_IT_FIFO_OV:        Receive FIFO overflow interrupt
  *            @arg UART_IT_TXEND:          Completion of sending interrupt
  *            @arg UART_IT_PARITY_ERR:     The parity check error interrupt
  * @retval None
  */
#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)     SET_BIT((__HANDLE__)->Instance->IER,(__INTERRUPT__))


/** @brief  Disable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         UART Handle selects the UART0/2/3 peripheral
  * @param  __INTERRUPT__ specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_FIFO_NO_EMPTY:  Receiving FIFO is not empty interrupt
  *            @arg UART_IT_FIFO_HALF_FULL: Receive FIFO half full interrupt
  *            @arg UART_IT_FIFO_FULL:      Receive FIFO full interrupt
  *            @arg UART_IT_FIFO_OV:        Receive FIFO overflow interrupt
  *            @arg UART_IT_TXEND:          Completion of sending interrupt
  *            @arg UART_IT_PARITY_ERR:     The parity check error interrupt
  * @retval None
  */
#define __HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)     CLEAR_BIT((__HANDLE__)->Instance->IER,(__INTERRUPT__))

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#define __HAL_UART_GET_IER_FLAG(__HANDLE__, __FLAG__)   (((__HANDLE__)->Instance->IER & (__FLAG__)) == (__FLAG__))

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);

/**
  * @brief  Receives an amount of data in blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
  * @brief  Returns the UART state.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL state
  */
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);


/**
  * @brief  Return the UART error code
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval UART Error Code
  */
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);

void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __UM324xx_HAL_UART_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/





