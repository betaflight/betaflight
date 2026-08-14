 /**
  ******************************************************************************
  * @file     um324xx_hal_lpuart.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-03-21  
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
#ifndef __UM324XX_HAL_LPUART_H__
#define __UM324XX_HAL_LPUART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup LPUART
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup LPUART_Exported_typedefs xxx Exported Typedefs
  * @{
  */ 

/**
  * @brief LPUART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the LPUART communication baud rate. */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref LPUART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref LPUART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref LPUART_Parity */
	
  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref LPUART_Mode */
	
  uint32_t ReceiveITType;             /*!< Specifies whether the Receive IT Type .
                                           This parameter can be a value of @ref LPUART_Receive_IT_Type */

} LPUART_InitTypeDef;

/**
  * @brief HAL LPUART State structures definition
  */
typedef enum
{
  HAL_LPUART_STATE_RESET           = 0x00U,    /*!< Peripheral is not yet Initialized
                                                 Value is allowed for gState and RxState */
  HAL_LPUART_STATE_READY           = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                 Value is allowed for gState and RxState */
  HAL_LPUART_STATE_BUSY            = 0x24U,    /*!< an internal process is ongoing
                                                 Value is allowed for gState only */
  HAL_LPUART_STATE_BUSY_TX         = 0x21U,    /*!< Data Transmission process is ongoing
                                                 Value is allowed for gState only */
  HAL_LPUART_STATE_BUSY_RX         = 0x22U,    /*!< Data Reception process is ongoing
                                                 Value is allowed for RxState only */
  HAL_LPUART_STATE_BUSY_TX_RX      = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                 Not to be used for neither gState nor RxState.
                                                 Value is result of combination (Or) between gState and RxState values */
  HAL_LPUART_STATE_TIMEOUT         = 0xA0U,    /*!< Timeout state
                                                 Value is allowed for gState only */
  HAL_LPUART_STATE_ERROR           = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} HAL_LPUART_StateTypeDef;

/**
  * @brief  LPUART handle Structure definition
  */
typedef struct __LPUART_HandleTypeDef
{
  LPUART_TypeDef                *Instance;        /*!< LPUART registers base address        */

  LPUART_InitTypeDef            Init;             /*!< LPUART communication parameters      */

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to LPUART Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< LPUART Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< LPUART Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to LPUART Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< LPUART Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< LPUART Rx Transfer Counter           */

  HAL_LockTypeDef               Lock;             /*!< Locking object                     	*/

  __IO HAL_LPUART_StateTypeDef    gState;         /*!< LPUART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_LPUART_StateTypeDef */

  __IO HAL_LPUART_StateTypeDef    RxState;        /*!< LPUART state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_LPUART_StateTypeDef */

  __IO uint32_t                 ErrorCode;        /*!< LPUART Error code                    */

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
  void (* TxCpltCallback)(struct __LPUART_HandleTypeDef *hlpuart);            /*!< LPUART Tx Complete Callback             */
  void (* RxCpltCallback)(struct __LPUART_HandleTypeDef *hlpuart);            /*!< LPUART Rx Complete Callback             */
  void (* ErrorCallback)(struct __LPUART_HandleTypeDef *hlpuart);             /*!< LPUART Error Callback                   */
  void (* MatchCallback)(struct __LPUART_HandleTypeDef *hlpuart);             /*!< LPUART Match Callback                   */

  void (* MspInitCallback)(struct __LPUART_HandleTypeDef *hlpuart);           /*!< LPUART Msp Init callback                */
  void (* MspDeInitCallback)(struct __LPUART_HandleTypeDef *hlpuart);         /*!< LPUART Msp DeInit callback              */
#endif  /* USE_HAL_LPUART_REGISTER_CALLBACKS */

} LPUART_HandleTypeDef;

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL LPUART Callback ID enumeration definition
  */
typedef enum
{
  HAL_LPUART_TX_COMPLETE_CB_ID             = 0x01U,    /*!< LPUART Tx Complete Callback ID             */
  HAL_LPUART_RX_COMPLETE_CB_ID             = 0x03U,    /*!< LPUART Rx Complete Callback ID             */
  HAL_LPUART_ERROR_CB_ID                   = 0x04U,    /*!< LPUART Error Callback ID                   */
  HAL_LPUART_MATCH_CB_ID                   = 0x08U,    /*!< LPUART Match Callback ID                   */

  HAL_LPUART_MSPINIT_CB_ID                 = 0x0BU,    /*!< LPUART MspInit callback ID                 */
  HAL_LPUART_MSPDEINIT_CB_ID               = 0x0CU     /*!< LPUART MspDeInit callback ID               */
	  
} HAL_LPUART_CallbackIDTypeDef;

/**
  * @brief  HAL LPUART Callback pointer definition
  */
typedef  void (*pLPUART_CallbackTypeDef)(LPUART_HandleTypeDef *hlpuart);  /*!< pointer to an LPUART callback function */

#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup LPUART_Exported_constants LPUART Exported Constants
  * @{
  */ 

/** @defgroup LPUART_Baud_Rate LPUART Baud Rate
  * @{
  */
#define LPUART_BAUDRATE_9600                  0x00000000U
#define LPUART_BAUDRATE_4800                  ((uint32_t)LPUART_BAUD_BAUD_0)
#define LPUART_BAUDRATE_2400                  ((uint32_t)LPUART_BAUD_BAUD_1)
#define LPUART_BAUDRATE_1200                  ((uint32_t)(LPUART_BAUD_BAUD_0 | LPUART_BAUD_BAUD_1))
#define LPUART_BAUDRATE_600                   ((uint32_t)LPUART_BAUD_BAUD_2)
#define LPUART_BAUDRATE_300                   ((uint32_t)(LPUART_BAUD_BAUD_0 | LPUART_BAUD_BAUD_2))
/**
  * @}
  */

/** @defgroup LPUART_Word_Length LPUART Word Length
  * @{
  */
#define LPUART_WORDLENGTH_8B                  0x00000000U
#define LPUART_WORDLENGTH_7B                  ((uint32_t)LPUART_CON_DL)
/**
  * @}
  */

/** @defgroup LPUART_Stop_Bits LPUART Number of Stop Bits
  * @{
  */
#define LPUART_STOPBITS_1                     0x00000000U
#define LPUART_STOPBITS_2                     ((uint32_t)LPUART_CON_SL)
/**
  * @}
  */

/** @defgroup LPUART_Parity LPUART Parity
  * @{
  */
#define LPUART_PARITY_NONE                    0x00000000U
#define LPUART_PARITY_ODD                     ((uint32_t)(LPUART_CON_PAREN | LPUART_CON_PTYP))
#define LPUART_PARITY_EVEN                    ((uint32_t)LPUART_CON_PAREN)
/**
  * @}
  */

/** @defgroup LPUART_Mode LPUART Transfer Mode
  * @{
  */
#define LPUART_MODE_RX                        ((uint32_t)LPUART_EN_RXEN)
#define LPUART_MODE_TX                        ((uint32_t)LPUART_EN_TXEN)
#define LPUART_MODE_TX_RX                     ((uint32_t)(LPUART_EN_RXEN | LPUART_EN_TXEN))
/**
  * @}
  */

/** @defgroup LPUART_Flags   LPUART FLags
  * @{
  */
#define LPUART_FLAG_TC                        ((uint32_t)LPUART_STA_TC)
#define LPUART_FLAG_TXE                       ((uint32_t)LPUART_STA_TXE)
#define LPUART_FLAG_START                     ((uint32_t)LPUART_STA_START)
#define LPUART_FLAG_PERR                      ((uint32_t)LPUART_STA_PERR)
#define LPUART_FLAG_FERR                      ((uint32_t)LPUART_STA_FERR)
#define LPUART_FLAG_RXOV                      ((uint32_t)LPUART_STA_RXOV)
#define LPUART_FLAG_RXF                       ((uint32_t)LPUART_STA_RXF)
#define LPUART_FLAG_MATCH                     ((uint32_t)LPUART_STA_MATCH)

/**
  * @}
  */
  
/** @defgroup LPUART_Interrupt_FLAG  LPUART Interrupt Flag
  * @{
  */
#define LPUART_IT_TC                          ((uint32_t)LPUART_CON_TCIE) 
#define LPUART_IT_TX                          ((uint32_t)LPUART_CON_TXIE)
#define LPUART_IT_ERR                         ((uint32_t)LPUART_CON_ERRIE)
#define LPUART_IT_RX                          ((uint32_t)LPUART_CON_RXIE)

/**
  * @}
  */
  
/** @defgroup LPUART_Receive_IT_Type  LPUART Receive IT Type
  * @{
  */
#define LPUART_RECEIVE_IT_START_WAKEUP                0x00000000U
#define LPUART_RECEIVE_IT_1BYTE                       ((uint32_t)LPUART_CON_RXEV_0)
#define LPUART_RECEIVE_IT_MATCH                       ((uint32_t)LPUART_CON_RXEV_1)
#define LPUART_RECEIVE_IT_FALLING_EDGE                ((uint32_t)(LPUART_CON_RXEV_0 | LPUART_CON_RXEV_1))

/**
  * @}
  */

/** @defgroup LPUART_Error_Code LPUART Error Code
  * @{
  */
#define HAL_LPUART_ERROR_NONE              0x00000000U   /*!< No error            */
#define HAL_LPUART_ERROR_PERR              0x00000001U   /*!< Parity error        */
#define HAL_LPUART_ERROR_FERR              0x00000002U   /*!< Frame error         */
#define HAL_LPUART_ERROR_RXOV              0x00000004U   /*!< Overrun error       */
#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
#define  HAL_LPUART_ERROR_INVALID_CALLBACK 0x00000008U   /*!< Invalid Callback error  */
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LPUART_Exported_macro LPUART Exported Macro
  * @{
  */ 

/** @brief  Checks whether the specified LPUART flag is set or not.
  * @param  __HANDLE__ specifies the LPUART Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg LPUART_FLAG_TC:			Transmission Completed Flag
  *            @arg LPUART_FLAG_TXE:		Transmission Buffer Empty Flag
  *            @arg LPUART_FLAG_START:		Start Bit Detection Flag
  *            @arg LPUART_FLAG_PERR: 		Parity Error flag
  *            @arg LPUART_FLAG_FERR: 		Framing Error flag
  *            @arg LPUART_FLAG_RXOV: 		Receive Buffer Overrun Flag
  *            @arg LPUART_FLAG_RXF: 		Receive Buffer Full Flag
  *            @arg LPUART_FLAG_MATCH:
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_LPUART_GET_FLAG(__HANDLE__, __FLAG__) (((__FLAG__) & (__HANDLE__)->Instance->STA) == (__FLAG__))                             


/** @brief  Enable the specified LPUART interrupt.
  * @param  __HANDLE__ specifies the LPUART Handle.
  * @param  __INTERRUPT__ specifies the LPUART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg LPUART_IT_TC:  Transmit Complete Interrupt
  *            @arg LPUART_IT_TX:  Transmit Buffer EmptyInterrupt
  *            @arg LPUART_IT_ERR: Error Interrupt
  *            @arg LPUART_IT_RX:  Received Interrupt
  * @retval None
  */
#define __HAL_LPUART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CON |= (__INTERRUPT__))

/** @brief  Disable the specified LPUART interrupt.
  * @param  __HANDLE__ specifies the LPUART Handle.
  * @param  __INTERRUPT__ specifies the LPUART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg LPUART_IT_TC:  Transmit Complete Interrupt
  *            @arg LPUART_IT_TX:  Transmit Buffer EmptyInterrupt
  *            @arg LPUART_IT_ERR: Error Interrupt
  *            @arg LPUART_IT_RX:  Received Interrupt
  * @retval None
  */
#define __HAL_LPUART_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CON &= ~(__INTERRUPT__))

/**
  * @}
  */
  
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup LPUART_Private_Functions LPUART Private Functions
  * @{
  */  

/** @addtogroup LPUART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef HAL_LPUART_Init(LPUART_HandleTypeDef *hlpuart);
HAL_StatusTypeDef HAL_LPUART_DeInit(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_MspInit(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_MspDeInit(LPUART_HandleTypeDef *hlpuart);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_LPUART_RegisterCallback(LPUART_HandleTypeDef *hlpuart, HAL_LPUART_CallbackIDTypeDef CallbackID,
                                            pLPUART_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_LPUART_UnRegisterCallback(LPUART_HandleTypeDef *hlpuart, HAL_LPUART_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup LPUART_Exported_Functions_Group2 IO operation functions
  * @{
  */

/* IO operation functions *******************************************************/
HAL_StatusTypeDef HAL_LPUART_Transmit(LPUART_HandleTypeDef *hlpuart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPUART_Receive(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPUART_Transmit_IT(LPUART_HandleTypeDef *hlpuart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_LPUART_Receive_IT(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size);

void HAL_LPUART_IRQHandler(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_TxCpltCallback(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_RxCpltCallback(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_ErrorCallback(LPUART_HandleTypeDef *hlpuart);
void HAL_LPUART_MatchCallback(LPUART_HandleTypeDef *hlpuart);

/**
  * @}
  */

/** @addtogroup LPUART_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  **************************************************/
HAL_LPUART_StateTypeDef HAL_LPUART_GetState(LPUART_HandleTypeDef *hlpuart);
uint32_t                HAL_LPUART_GetError(LPUART_HandleTypeDef *hlpuart);
/**
  * @}
  */

/** @defgroup LPUART_Private_Functions LPUART Private Functions
  * @{
  */  
HAL_StatusTypeDef LPUART_Start_Receive_IT(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
