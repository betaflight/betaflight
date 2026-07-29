/**
  ******************************************************************************
  * @file     um324xx_hal_uart_ex.h
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
#ifndef __UM324XX_HAL_UART_EX_H__
#define __UM324XX_HAL_UART_EX_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"
#include "um324xx_hal_dma.h"

/** @addtogroup UM324xx_HAL_Driver
 * @{
 */

/** @addtogroup UART_EX UART1 
 * @{
 */

/**
  * @brief  UART1 protocol frame definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART1 communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_EX_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_EX_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_EX_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled. */
                                          

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_EX_Hardware_Flow_Control */

} UART_EX_InitTypeDef;
  
/**
  * @brief  UART1 State definition
  */
typedef enum
{
  HAL_UART_EX_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  HAL_UART_EX_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  HAL_UART_EX_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_EX_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  HAL_UART_EX_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  HAL_UART_EX_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  HAL_UART_EX_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  HAL_UART_EX_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} HAL_UART_EX_StateTypeDef;
   
/**
  * @brief HAL UART1 Reception type definition
  * @note  HAL UART1 Reception type value aims to identify which type of Reception is ongoing.
  *        It is expected to admit following values :
  *           HAL_UART_EX_RECEPTION_STANDARD         = 0x00U,
  *           HAL_UART_EX_RECEPTION_TOIDLE           = 0x01U,
  */
typedef uint32_t HAL_UART_EX_RxTypeTypeDef;

/**
  * @brief  UART1 handle Structure definition
  */
typedef struct __UART_EX_HandleTypeDef
{
  UART_EX_TypeDef                 *Instance;        /*!< UART1 registers base address        */

  UART_EX_InitTypeDef              Init;            /*!< UART1 communication parameters      */

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to UART1 Tx transfer Buffer  */

  uint16_t                      TxXferSize;       /*!< UART1 Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< UART1 Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to UART1 Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< UART1 Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< UART1 Rx Transfer Counter           */

  DMA_HandleTypeDef             *hdmatx;          /*!< UART1 Tx DMA Handle parameters      */

  DMA_HandleTypeDef             *hdmarx;          /*!< UART1 Rx DMA Handle parameters      */    
    
  __IO HAL_UART_EX_RxTypeTypeDef  ReceptionType;      /*!< Type of ongoing reception          */

  HAL_LockTypeDef               Lock;             /*!< Locking object                     */

  __IO HAL_UART_EX_StateTypeDef    gState;           /*!< UART1 state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_UART_EX_StateTypeDef */

  __IO HAL_UART_EX_StateTypeDef    RxState;          /*!< UART1 state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_UART_EX_StateTypeDef */

  __IO uint32_t                 ErrorCode;        /*!< UART1 Error code                    */

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
  void (* TxCpltCallback)(struct __UART_EX_HandleTypeDef *huart);            /*!< UART1 Tx Complete Callback             */
  void (* RxCpltCallback)(struct __UART_EX_HandleTypeDef *huart);            /*!< UART1 Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_EX_HandleTypeDef *huart);             /*!< UART1 Error Callback                   */


  void (* MspInitCallback)(struct __UART_EX_HandleTypeDef *huart);           /*!< UART1 Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_EX_HandleTypeDef *huart);         /*!< UART1 Msp DeInit callback              */
#endif  /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

} UART_EX_HandleTypeDef;

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL UART1 Callback ID enumeration definition
  */
typedef enum
{
  HAL_UART_EX_TX_COMPLETE_CB_ID             = 0x01U,    /*!< UART1 Tx Complete Callback ID             */
  HAL_UART_EX_RX_COMPLETE_CB_ID             = 0x03U,    /*!< UART1 Rx Complete Callback ID             */
  HAL_UART_EX_ERROR_CB_ID                   = 0x04U,    /*!< UART1 Error Callback ID                   */

  HAL_UART_EX_MSPINIT_CB_ID                 = 0x0BU,    /*!< UART1 MspInit callback ID                 */
  HAL_UART_EX_MSPDEINIT_CB_ID               = 0x0CU     /*!< UART1 MspDeInit callback ID               */

} HAL_UART_EX_CallbackIDTypeDef;

/**
  * @brief  HAL UART1 Callback pointer definition
  */
typedef  void (*pUART_EX_CallbackTypeDef)(UART_EX_HandleTypeDef *huart1);  /*!< pointer to an UART1 callback function */

#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup UART_EX_Exported_Constants UART1 Exported Constants
  * @{
  */
#define HAL_UART_EX_ERROR_NONE              0x00000000U   /*!< No error            */
#define HAL_UART_EX_ERROR_PE                0x00000001U   /*!< Parity error        */
#define HAL_UART_EX_ERROR_FE                0x00000002U   /*!< Frame error         */
#define HAL_UART_EX_ERROR_ORE               0x00000004U   /*!< Overrun error       */
#define HAL_UART_EX_ERROR_DMA               0x00000010U   /*!< DMA transfer error  */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
#define  HAL_UART_EX_ERROR_INVALID_CALLBACK 0x00000008U   /*!< Invalid Callback error  */
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */



/** @defgroup UART_EX_Word_Length UART1 Word Length
  * @{
  */
#define UART_EX_WORDLENGTH_5B                  0x00000000U
#define UART_EX_WORDLENGTH_6B                  ((uint32_t)UART_EX_LCR_DLS_0)
#define UART_EX_WORDLENGTH_7B                  ((uint32_t)UART_EX_LCR_DLS_1)
#define UART_EX_WORDLENGTH_8B                  ((uint32_t)(UART_EX_LCR_DLS_0 | UART_EX_LCR_DLS_1))
#define UART_EX_WORDLENGTH_9B                  ((uint32_t)UART_EX_LCRE_TRANSMIT_MODE | UART_EX_LCRE_DLS_E)
/**
  * @}
  */

/** @defgroup UART_EX_Stop_Bits UART1 Number of Stop Bits
  * @{
  */
#define UART_EX_STOPBITS_1                     0x00000000U
#define UART_EX_STOPBITS_2                     ((uint32_t)UART_EX_LCR_STOP)
/**
  * @}
  */


/** @defgroup UART_EX_Parity UART1 Parity
  * @{
  */
#define UART_EX_PARITY_NONE                    0x00000000U
#define UART_EX_PARITY_ODD                     ((uint32_t)UART_EX_LCR_PEN)
#define UART_EX_PARITY_EVEN                    ((uint32_t)(UART_EX_LCR_PEN | UART_EX_LCR_EPS))
/**
  * @}
  */

/** @defgroup UART_EX_Hardware_Flow_Control UART1 Hardware Flow Control
  * @{
  */
#define UART_EX_HWCONTROL_NONE                  0x00000000U
#define UART_EX_HWCONTROL_RTS                   ((uint32_t)UART_EX_MCR_RTS)
/**
  * @}
  */

/** @defgroup UART_EX_Flags   UART1 FLags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: LSR register
  *                 - 10: MSR register
  *                 - 11: USR register
  * @{
  */
/* Flags in the LSR register */
#define UART_EX_FLAG_ADDR_RCVD                    ((uint8_t)0x28)
#define UART_EX_FLAG_RFE                          ((uint8_t)0x27)
#define UART_EX_FLAG_TEMT                         ((uint8_t)0x26)
#define UART_EX_FLAG_THRE                         ((uint8_t)0x25)
#define UART_EX_FLAG_BI                           ((uint8_t)0x24)
#define UART_EX_FLAG_FE                           ((uint8_t)0x23)
#define UART_EX_FLAG_PE                           ((uint8_t)0x22)
#define UART_EX_FLAG_OE                           ((uint8_t)0x21)
#define UART_EX_FLAG_DR                           ((uint8_t)0x20)

/* Flags in the MSR register */
#define UART_EX_FLAG_CTS                          ((uint8_t)0x44)

/* Flags in the USR register */
#define UART_EX_FLAG_BUSY                         ((uint8_t)0x60)
#define UART_EX_FLAG_TFNF                         ((uint8_t)0x61)
#define UART_EX_FLAG_TFE                          ((uint8_t)0x62)
#define UART_EX_FLAG_RFNE                         ((uint8_t)0x63)
#define UART_EX_FLAG_RFF                          ((uint8_t)0x64)

/**
  * @}
  */
  
/** @defgroup UART_EX_Interrupt_definition  UART1 Interrupt Definitions
  *           IT mask in the IER register
  * @{
  */
#define UART_EX_IT_PTIME                        ((uint32_t)UART_EX_IER_PTIME) 
#define UART_EX_IT_ELSI                         ((uint32_t)UART_EX_IER_ELSI)
#define UART_EX_IT_ETBEI                        ((uint32_t)UART_EX_IER_ETBEI)
#define UART_EX_IT_ERBFI                        ((uint32_t)UART_EX_IER_ERBFI)

/**
  * @}
  */

/** @defgroup Enable DMA function define
  * @{
  */
#define UART_EX_MCR_DMA_ENABLE                        UART_EX_MCR_DMAE
#define UART_EX_MCR_DMA_DISABLE                       0x00000000U

/**
  * @}
  */

/** @defgroup UART1 interruptions flag mask
  * @{
  */
#define UART_EX_IT_MASK                     0x00000087U

/**
  * @}
  */

/** @brief  Checks whether the specified UART1 flag is set or not.
  * @param  __HANDLE__ specifies the UART1 Handle.
  *         UART1 Handle selects the UART1 peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_EX_FLAG_ADDR_RCVD:   Address Received flag
  *            @arg UART_EX_FLAG_RFE:         Receiver FIFO Error flag
  *            @arg UART_EX_FLAG_TEMT:        Transmitter Empty flag
  *            @arg UART_EX_FLAG_THRE:        Transmit Holding Register Empty flag
  *            @arg UART_EX_FLAG_BI:          Break Interrupt flag
  *            @arg UART_EX_FLAG_FE:          Framing error flag
  *            @arg UART_EX_FLAG_PE:          Parity error flag
  *            @arg UART_EX_FLAG_OE:          Overrun error flag
  *            @arg UART_EX_FLAG_CTS:         Clear to Send flag
  *            @arg UART_EX_FLAG_BUSY:        UART1 Busy flag
  *            @arg UART_EX_FLAG_TFNF:        Transmit FIFO Not Full flag
  *            @arg UART_EX_FLAG_TFE :        Transmit FIFO Empty flag
  *            @arg UART_EX_FLAG_RFNE:        Receive FIFO Not Empty flag
  *            @arg UART_EX_FLAG_RFF :        Receive FIFO Full flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define UART_EX_FLAG_MASK   0x1F
#define __HAL_UART_EX_GET_FLAG(__HANDLE__, __FLAG__) (((((((__FLAG__) >> 5) == 1U)? \
                            (__HANDLE__)->Instance->LSR : (((__FLAG__) >> 5) == 2U)? \
                            (__HANDLE__)->Instance->MSR : (__HANDLE__)->Instance->USR) & \
                            (1U << ((__FLAG__) & UART_EX_FLAG_MASK))) != 0U)? 1U:0U)

/** @brief  Enable the specified UART1 interrupt.
  * @param  __HANDLE__ specifies the UART1 Handle.
  *         UART1 Handle selects the UART1 peripheral
  * @param  __INTERRUPT__ specifies the UART1 interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_EX_IT_PTIME:  Programmable THRE Interrupt
  *            @arg UART_EX_IT_ELSI:   Receiver Line Status Interrupt
  *            @arg UART_EX_IT_ETBEI:  Transmit Holding Register Empty Interrupt
  *            @arg UART_EX_IT_ERBFI:  Received Data Available Interrupt
  * @retval None
  */
#define __HAL_UART_EX_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->IER |= ((__INTERRUPT__) & UART_EX_IT_MASK))


/** @brief  Disable the specified UART1 interrupt.
  * @param  __HANDLE__ specifies the UART1 Handle.
  *         UART1 Handle selects the UART1 peripheral
  * @param  __INTERRUPT__ specifies the UART1 interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg UART_EX_IT_PTIME:  Programmable THRE Interrupt
  *            @arg UART_EX_IT_ELSI:   Receiver Line Status Interrupt
  *            @arg UART_EX_IT_ETBEI:  Transmit Holding Register Empty Interrupt
  *            @arg UART_EX_IT_ERBFI:  Received Data Available Interrupt
  * @retval None
  */
#define __HAL_UART_EX_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->IER &= ~((__INTERRUPT__) & UART_EX_IT_MASK))


/** @brief  Enable DMA function.
  * @retval None
  */
#define __HAL_UART_EX_DMA_ENABLE()                    SET_BIT(UART1->MCR , UART_EX_MCR_DMA_ENABLE)

/** @brief  Disable DMA function.
  * @retval None
  */
#define __HAL_UART_EX_DMA_DISABLE()                   CLEAR_BIT(UART1->MCR , UART_EX_MCR_DMA_ENABLE)


/* Private macros ------------------------------------------------------------*/
/** @defgroup UART_EX_Private_Macros 
  * @{
  */
/* UART1 DLF = (((uint32_t)((_PCLK_)%(_BAUD_ * 16)) / baud_rate)*4) & 0x3F */
#define UART_EX_DLF_DIV(_PCLK_, _BAUD_)            ((((uint32_t)((_PCLK_) % ((_BAUD_) * 16U)) / _BAUD_) * 4) & 0x3F)
/* UART1 DLH = (( _PCLK_ / ( _BAUD_ * 16)) >> 8) & 0xFF */
#define UART_EX_DLH_DIV(_PCLK_, _BAUD_)            (((((uint32_t)_PCLK_) / (((uint32_t)_BAUD_) * 16U)) >> 8U) & 0xff)

/* UART1 DLL = ( _PCLK_ / (_BAUD_ * 16)) & 0xFF */
#define UART_EX_DLL_DIV(_PCLK_, _BAUD_)            ((((uint32_t)_PCLK_) / (((uint32_t)_BAUD_) * 16U)) & 0xff)

/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_EX_Private_Functions UART1 Private Functions
  * @{
  */  
  
/** @addtogroup UART_EX_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef HAL_UART_EX_Init(UART_EX_HandleTypeDef *huart1);
HAL_StatusTypeDef HAL_UART_EX_DeInit(UART_EX_HandleTypeDef *huart1);
void HAL_UART_EX_MspInit(UART_EX_HandleTypeDef *huart1);
void HAL_UART_EX_MspDeInit(UART_EX_HandleTypeDef *huart1);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_UART_EX_RegisterCallback(UART_EX_HandleTypeDef *huart1, HAL_UART_EX_CallbackIDTypeDef CallbackID,
                                            pUART_EX_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_UART_EX_UnRegisterCallback(UART_EX_HandleTypeDef *huart1, HAL_UART_EX_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

/**
  * @}
  */


/** @addtogroup UART_EX_Exported_Functions_Group2 IO operation functions
  * @{
  */

/* IO operation functions *******************************************************/
HAL_StatusTypeDef HAL_UART_EX_Transmit(UART_EX_HandleTypeDef *huart1, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_EX_Receive(UART_EX_HandleTypeDef *huart1, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_EX_Transmit_IT(UART_EX_HandleTypeDef *huart1, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_EX_Receive_IT(UART_EX_HandleTypeDef *huart1, uint8_t *pData, uint16_t Size);

#ifdef HAL_DMA_MODULE_ENABLED
/**
  * @brief  Sends an amount of data in DMA mode.
  * @note   When UART1 parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART1 module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Transmit_DMA(UART_EX_HandleTypeDef *huart1, const uint8_t *pData, uint16_t Size);

/**
  * @brief  Receives an amount of data in DMA mode.
  * @note   When UART1 parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART1 module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Receive_DMA(UART_EX_HandleTypeDef *huart1, uint8_t *pData, uint16_t Size);

/**
  * @brief  Start Receive operation in DMA mode.
  * @note   This function could be called by all HAL UART1 API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART1 Handle is assumed as Locked.
  * @param  huart1 UART1 handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_EX_Start_Receive_DMA(UART_EX_HandleTypeDef *huart1, uint8_t *pData, uint16_t Size);

#endif

/* Interrupt handle and callback operation functions *******************************************************/
void HAL_UART_EX_IRQHandler(UART_EX_HandleTypeDef *huart1);
void HAL_UART_EX_TxCpltCallback(UART_EX_HandleTypeDef *huart1);
void HAL_UART_EX_RxCpltCallback(UART_EX_HandleTypeDef *huart1);
void HAL_UART_EX_ErrorCallback(UART_EX_HandleTypeDef *huart1);

/**
  * @}
  */

/** @addtogroup UART_EX_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  **************************************************/
HAL_UART_EX_StateTypeDef HAL_UART_EX_GetState(UART_EX_HandleTypeDef *huart1);

/**
  * @}
  */
 
/* Peripheral Error states functions  **************************************************/
uint32_t HAL_UART_EX_GetError(UART_EX_HandleTypeDef *huart1);

/**
  * @}
  */

/** @defgroup UART_EX_Private_Functions UART1 Private Functions
  * @{
  */  
HAL_StatusTypeDef UART_EX_Start_Receive_IT(UART_EX_HandleTypeDef *huart1, uint8_t *pData, uint16_t Size);

/**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __UM324xx_HAL_UART_EX_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


