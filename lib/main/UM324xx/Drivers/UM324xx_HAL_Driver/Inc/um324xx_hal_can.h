 /**
  ******************************************************************************
  * @file     um324xF_hal_can.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-08  
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
#ifndef __UM324XX_HAL_CAN_H__
#define __UM324XX_HAL_CAN_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup CAN
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup CAN_Exported_typedefs CAN Exported Typedefs
  * @{
  */ 
  
 /** @defgroup CAN_Buffer_size CAN Buffer size
  * @{
  */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00U,  /*!< CAN not yet initialized or disabled */
  HAL_CAN_STATE_READY             = 0x01U,  /*!< CAN initialized and ready for use   */
  HAL_CAN_STATE_LISTENING         = 0x02U,  /*!< CAN receive process is ongoing      */
  HAL_CAN_STATE_ERROR             = 0x03U   /*!< CAN error state                     */

} HAL_CAN_StateTypeDef;


/**
  * @brief  CAN init structure definition
  */
typedef struct
{
  uint32_t Prescaler;                  /*!< Specifies the length of a time quantum.
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 1024. */

  uint32_t Mode;                       /*!< Specifies the CAN operating mode.
                                            This parameter can be a value of @ref CAN_operating_mode */

  uint32_t SyncJumpWidth;              /*!< Specifies the maximum number of time quanta the CAN hardware
                                            is allowed to lengthen or shorten a bit to perform resynchronization.
                                            This parameter can be a value of @ref CAN_synchronisation_jump_width */

  uint32_t TimeSeg1;                   /*!< Specifies the number of time quanta in Bit Segment 1.
                                            This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_1 */

  uint32_t TimeSeg2;                   /*!< Specifies the number of time quanta in Bit Segment 2.
                                            This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_2 */
} CAN_InitTypeDef;
  
  
  
  
/**
  * @brief  CAN filter configuration structure definition
  */
typedef struct
{
  uint32_t FilterMode;              /*!< Specifies the filter mode to be initialized.
                                       This parameter can be a value of @ref CAN_Filter_Mode */

  uint32_t FilterStdId1;            /*!< Specifies the filter standard identifier1.
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */
  
  uint32_t FilterStdId2;            /*!< Specifies the filter standard identifier2.
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

    
  uint32_t FilterExtId1;            /*!< Specifies the filter extend identifier1.
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */
  
  uint32_t FilterExtId2;            /*!< Specifies the filter identification number (LSBs for a 32-bit
                                       configuration, second one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t FilterRtr;               /*!< Specifies the filter scale.
                                       This parameter can be a value of @ref CAN_identifier_type */
                                       
  uint32_t FilterIde;               /*!< Specifies the filter scale.
                                       This parameter can be a value of @ref CAN_identifier_type */
                                       
  uint32_t FilterMaskType;          /*!< Specifies the filter scale.
                                    This parameter can be a value of @ref CAN_identifier_type */
                                       
                                       
  uint32_t FilterData1;            /*!< Specifies the filter data (MSBs for a 8-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xF. */
                                       
  uint32_t FilterData2;            /*!< Specifies the filter data (MSBs for a 8-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xF. */

} CAN_FilterTypeDef;

/**
  * @brief  CAN Tx message header structure definition
  */
typedef struct
{
  uint32_t StdId;            /*!< Specifies the standard identifier.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */
        
  uint32_t ExtId;            /*!< Specifies the extended identifier.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */
        
  uint32_t IDE;              /*!< Specifies the type of identifier for the message that will be transmitted.
                                This parameter can be a value of @ref CAN_identifier_type */
        
  uint32_t RTR;              /*!< Specifies the type of frame for the message that will be transmitted.
                                This parameter can be a value of @ref CAN_remote_transmission_request */
        
  uint32_t DLC;              /*!< Specifies the length of the frame that will be transmitted.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 8. */
  uint32_t pTxBuffer[4];
    
} CAN_TxHeaderTypeDef;

/**
  * @brief  CAN Rx message header structure definition
  */
typedef struct
{
    uint32_t StdId;               /*!< Specifies the standard identifier.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */
                                
    uint32_t ExtId;               /*!< Specifies the extended identifier.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */
                                
    uint32_t IDE;                 /*!< Specifies the type of identifier for the message that will be transmitted.
                                This parameter can be a value of @ref CAN_identifier_type */
                                
    uint32_t RTR;                 /*!< Specifies the type of frame for the message that will be transmitted.
                                This parameter can be a value of @ref CAN_remote_transmission_request */
                                
    uint32_t DLC;                 /*!< Specifies the length of the frame that will be transmitted.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

    uint32_t pRxBuffer[16];
    
} CAN_RxHeaderTypeDef;

/**
  * @brief  CAN handle Structure definition
  */
typedef struct __CAN_HandleTypeDef
{
  CAN_TypeDef                 *Instance;                 /*!< Register base address */

  CAN_InitTypeDef             Init;                      /*!< CAN required parameters */

  __IO HAL_CAN_StateTypeDef   State;                     /*!< CAN communication state */

  __IO uint32_t               ErrorCode;                 /*!< CAN Error code.
                                                              This parameter can be a value of @ref CAN_Error_Code */
} CAN_HandleTypeDef;
/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup CAN_Exported_constants CAN Exported Constants
  * @{
  */ 
/** @defgroup CAN_Error_Code CAN Error Code
  * @{
  */
#define HAL_CAN_ERROR_NONE              (0x00000000U)     /*!< No error                                                          */
#define HAL_CAN_ERROR_BER               (0x00000001U)     /*!< Bit error                                                         */
#define HAL_CAN_ERROR_STFER             (0x00000002U)     /*!< Stuff error                                                       */
#define HAL_CAN_ERROR_CRCER             (0x00000004U)     /*!< CRC error                                                         */
#define HAL_CAN_ERROR_FRMER             (0x00000008U)     /*!< Form error                                                        */
#define HAL_CAN_ERROR_ACKER             (0x00000010U)     /*!< ACK error                                                         */
#define HAL_CAN_ERROR_EDIR              (0x00000020U)     /*!< Indicates Receive while error occurred.                           */
#define HAL_CAN_ERROR_TXWRN             (0x00000040U)     /*!< Set when TXERR counter is greater than or equal to 96             */
#define HAL_CAN_ERROR_RXWRN             (0x00000080U)     /*!< Set when RXERR counter is greater than or equal to 96             */
#define HAL_CAN_ERROR_RX_DO             (0x00000100U)     /*!< Receive data overflow error                                       */
#define HAL_CAN_ERROR_BE                (0x00000200U)     /*!< Bus error                                                         */
#define HAL_CAN_ERROR_EP                (0x00000400U)     /*!< Passive error                                                     */
#define HAL_CAN_ERROR_EW                (0x00000800U)     /*!< Protocol Error Warning                                            */
#define HAL_CAN_ERROR_AL                (0x00000C00U)     /*!< Arbitration loss error                                            */
#define HAL_CAN_ERROR_NOT_INITIALIZED   (0x00001000U)     /*!< Peripheral not initialized                                        */

/** @defgroup CAN_Filter_MASK CAN Filter MASK 
  * @{
  */
  
#define CAN_Filter_MASK_ALL                 (0xFFFFFFFFU)           /*!< Mask all, do not compare       */
#define CAN_Filter_MASK_NONE                (0x00000000U)           /*!< Mask none, compare all bits    */
 
/*CAN_Filter Single*/
#define CAN_Filter_MASK_ID_SINGLE_STD       (0x0000E0FFU)           /*!< Filter single mode, STD mode ,mask ID, compare DATA and RTR    */
#define CAN_Filter_MASK_DATA_SINGLE_STD     (0xFFFF0000U)           /*!< Filter single mode, STD mode ,mask DATA, compare ID and RTR    */
#define CAN_Filter_MASK_ID_SINGLE_EXT       (0xF8FFFFFFU)           /*!< Filter single mode, EXT mode ,mask ID, compare RTR             */
/*CAN_Filter Double*/
#define CAN_Filter_MASK_ID_DOUBLE_STD       (0x0000E0FFU)           /*!< Filter double mode, STD mode ,mask ID1, compare DATA1 or ID2   */
#define CAN_Filter_MASK_DATA_DOUBLE_STD     (0x0F000F00U)           /*!< Filter double mode, STD mode ,mask DATA1, compare ID1 or ID2   */
#define CAN_Filter_MASK_ID_DOUBLE_EXT       (0xFFFFFFFFU)           /*!< Filter double mode, STD mode ,mask ID, do not to compare       */
#define CAN_Filter_MASK_NONE_DOUBLE_EXT     (0x00000000U)           /*!< Filter double mode, EXT mode ,mask none, compare ID1 or ID2    */

/** @defgroup CAN_Filter_Mode CAN Filter Mode
  * @{
  */
#define HAL_CAN_FilterMode_DOUBLE       (0x00000000U)                          /*!< Use double filter */
#define HAL_CAN_FilterMode_SINGLE       (0x00000001U)                          /*!< Use single filter */                                                                              

/** @defgroup CAN_operating_mode CAN Operating Mode
  * @{
  */
#define CAN_MODE_NORMAL             (0x00000000U)                              /*!< Normal mode     */
#define CAN_MODE_RESET              ((uint32_t)CAN_CONFIG0_RM)                 /*!< Reset mode      */
#define CAN_MODE_LISTEN             ((uint32_t)CAN_CONFIG0_LOM)                /*!< Listen  mode    */

/** @defgroup CAN_synchronisation_jump_width CAN Synchronization Jump Width
  * @{
  */
#define CAN_SJW_1TQ                 (0x00000000U)                              /*!< 1 time quantum */
#define CAN_SJW_2TQ                 ((uint32_t)CAN_CONFIG1_SJW_0)              /*!< 2 time quantum */
#define CAN_SJW_3TQ                 ((uint32_t)CAN_CONFIG1_SJW_1)              /*!< 3 time quantum */
#define CAN_SJW_4TQ                 ((uint32_t)CAN_CONFIG1_SJW)                /*!< 4 time quantum */

/** @defgroup CAN_time_quantum_in_bit_segment_1 CAN Time Quantum in Bit Segment 1
  * @{
  */
#define CAN_BS1_1TQ                 (0x00000000U)                                                                   /*!< 1 time quantum  */
#define CAN_BS1_2TQ                 ((uint32_t)CAN_CONFIG1_TSEG1_0)                                                 /*!< 2 time quantum  */
#define CAN_BS1_3TQ                 ((uint32_t)CAN_CONFIG1_TSEG1_1)                                                 /*!< 3 time quantum  */
#define CAN_BS1_4TQ                 ((uint32_t)(CAN_CONFIG1_TSEG1_1 | CAN_CONFIG1_TSEG1_0))                         /*!< 4 time quantum  */
#define CAN_BS1_5TQ                 ((uint32_t)CAN_CONFIG1_TSEG1_2)                                                 /*!< 5 time quantum  */
#define CAN_BS1_6TQ                 ((uint32_t)(CAN_CONFIG1_TSEG1_2 | CAN_CONFIG1_TSEG1_0))                         /*!< 6 time quantum  */
#define CAN_BS1_7TQ                 ((uint32_t)(CAN_CONFIG1_TSEG1_2 | CAN_CONFIG1_TSEG1_1))                         /*!< 7 time quantum  */
#define CAN_BS1_8TQ                 ((uint32_t)(CAN_CONFIG1_TSEG1_2 | CAN_CONFIG1_TSEG1_1 | CAN_CONFIG1_TSEG1_0))   /*!< 8 time quantum  */
#define CAN_BS1_9TQ                 ((uint32_t)CAN_CONFIG1_TSEG1_3)                                                 /*!< 9 time quantum  */
#define CAN_BS1_10TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_0))                         /*!< 10 time quantum */
#define CAN_BS1_11TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_1))                         /*!< 11 time quantum */
#define CAN_BS1_12TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_1 | CAN_CONFIG1_TSEG1_0))   /*!< 12 time quantum */
#define CAN_BS1_13TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_2))                         /*!< 13 time quantum */
#define CAN_BS1_14TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_2 | CAN_CONFIG1_TSEG1_0))   /*!< 14 time quantum */
#define CAN_BS1_15TQ                ((uint32_t)(CAN_CONFIG1_TSEG1_3 | CAN_CONFIG1_TSEG1_2 | CAN_CONFIG1_TSEG1_1))   /*!< 15 time quantum */
#define CAN_BS1_16TQ                ((uint32_t)CAN_CONFIG1_TSEG1)                                                   /*!< 16 time quantum */


/** @defgroup CAN_time_quantum_in_bit_segment_2 CAN Time Quantum in Bit Segment 2
  * @{
  */
#define CAN_BS2_1TQ                 (0x00000000U)                                                                   /*!< 1 time quantum */
#define CAN_BS2_2TQ                 ((uint32_t)CAN_CONFIG1_TSEG2_0)                                                 /*!< 2 time quantum */
#define CAN_BS2_3TQ                 ((uint32_t)CAN_CONFIG1_TSEG2_1)                                                 /*!< 3 time quantum */
#define CAN_BS2_4TQ                 ((uint32_t)(CAN_CONFIG1_TSEG2_1 | CAN_CONFIG1_TSEG2_0))                         /*!< 4 time quantum */
#define CAN_BS2_5TQ                 ((uint32_t)CAN_CONFIG1_TSEG2_2)                                                 /*!< 5 time quantum */
#define CAN_BS2_6TQ                 ((uint32_t)(CAN_CONFIG1_TSEG2_2 | CAN_CONFIG1_TSEG2_0))                         /*!< 6 time quantum */
#define CAN_BS2_7TQ                 ((uint32_t)(CAN_CONFIG1_TSEG2_2 | CAN_CONFIG1_TSEG2_1))                         /*!< 7 time quantum */
#define CAN_BS2_8TQ                 ((uint32_t)CAN_CONFIG1_TSEG2)                                                   /*!< 8 time quantum */

/** @defgroup CAN_filter_activation CAN Filter Activation
  * @{
  */
#define CAN_FILTER_DISABLE          (0x00000000U)  /*!< Disable filter */
#define CAN_FILTER_ENABLE           (0x00000001U)  /*!< Enable filter  */

/** @defgroup CAN_identifier_type CAN Identifier Type
  * @{
  */
#define CAN_ID_STD                  (0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT                  (0x00000001U)  /*!< Extended Id */

/** @defgroup CAN_remote_transmission_request CAN Remote Transmission Request
  * @{
  */
#define CAN_RTR_DATA                (0x00000000U)  /*!< Data frame   */
#define CAN_RTR_REMOTE              (0x00000001U)  /*!< Remote frame */

/** @defgroup CAN_Buffer_size CAN Buffer size
  * @{
  */
#define CAN_RX_BUFFER_SIZE          4U
#define CAN_TX_BUFFER_SIZE          4U

/** @defgroup CAN_Interrupts CAN Interrupts
  * @{
  */
#define CAN_IT_ARBITRATION_LOSS     ((uint32_t)CAN_CONFIG0_ALI)               /*!< Arbitration loss interrupt         */
#define CAN_IT_ERROR_WARNING        ((uint32_t)CAN_CONFIG0_EWI)               /*!< Error warning Interrupt            */
#define CAN_IT_ERROR_PASSIVE        ((uint32_t)CAN_CONFIG0_EPI)               /*!< Error passive interrupt            */
#define CAN_IT_RECEIVE              ((uint32_t)CAN_CONFIG0_RI)                /*!< Receive interrupt                  */
#define CAN_IT_TRANSMIT             ((uint32_t)CAN_CONFIG0_TI)                /*!< Transmit interrupt                 */
#define CAN_IT_BUSS_ERROR           ((uint32_t)CAN_CONFIG0_BEI)               /*!< Bus error                          */
#define CAN_IT_RECEIVE_OVERFLOW     ((uint32_t)CAN_CONFIG0_DOI)               /*!< Received data overflow interrupt    */

/** @defgroup CAN_Interrupts_Mask CAN Interrupts Mask
  * @{
  */
#define CAN_IT_MASK_ARBITRATION_LOSS     ((uint32_t)CAN_CONFIG1_ALIM)         /*!< Enable Arbitration loss interrupt         */
#define CAN_IT_MASK_ERROR_WARNING        ((uint32_t)CAN_CONFIG1_EWIM)         /*!< Enable Error warning Interrupt            */
#define CAN_IT_MASK_ERROR_PASSIVE        ((uint32_t)CAN_CONFIG1_EPIM)         /*!< Enable Error passive interrupt            */
#define CAN_IT_MASK_RECEIVE              ((uint32_t)CAN_CONFIG1_RIM)          /*!< Enable Receive interrupt                  */
#define CAN_IT_MASK_TRANSMIT             ((uint32_t)CAN_CONFIG1_TIM)          /*!< Enable Transmit interrupt                 */
#define CAN_IT_MASK_BUSS_ERROR           ((uint32_t)CAN_CONFIG1_BEIM)         /*!< Enable Bus error interrupt                */
#define CAN_IT_MASK_RECEIVE_OVERFLOW     ((uint32_t)CAN_CONFIG1_DOIM)         /*!< Enable Received data overflow interrupt    */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CAN_Exported_Macros CAN Exported Macro
  * @{
  */ 
  
/**
  * @brief  CAN module operation reset.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_RESET_ENABLE(__HANDLE__)               (((__HANDLE__)->Instance->CONFIG0) |= CAN_CONFIG0_RM)

/**
  * @brief  CAN module operation disable reset.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_RESET_DISABLE(__HANDLE__)               (((__HANDLE__)->Instance->CONFIG0) &= (~(CAN_CONFIG0_RM)))

/**
  * @brief  CAN Filter setting single filter.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_SINGLE_FILTER(__HANDLE__)              (((__HANDLE__)->Instance->CONFIG0) |= CAN_CONFIG0_AFM)

/**
  * @brief  CAN Filter setting single filter.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_DOUBLE_FILTER(__HANDLE__)              (((__HANDLE__)->Instance->CONFIG0) &= (~(CAN_CONFIG0_AFM)))

/**
  * @brief  CAN set normal module.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_NORMAL_MODE(__HANDLE__)                (((__HANDLE__)->Instance->CONFIG0) &= (~(CAN_CONFIG0_LOM)))

/**
  * @brief  CAN set normal module.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_LISTEN_MODE(__HANDLE__)                (((__HANDLE__)->Instance->CONFIG0) |= CAN_CONFIG0_LOM)

/**
  * @brief  Enable the specified CAN interrupts.
  * @param  __HANDLE__ CAN handle.
  * @param  __INTERRUPT__ CAN Interrupt sources to enable.
  *           This parameter can be any combination of @arg CAN_Interrupts
  *            @arg CAN_IT_ARBITRATION_LOSS     Arbitration loss interrupt   
  *            @arg CAN_IT_ERROR_WARNING        Error warning Interrupt          
  *            @arg CAN_IT_ERROR_PASSIVE        Error passive interrupt            
  *            @arg CAN_IT_RECEIVE              Receive interrupt                 
  *            @arg CAN_IT_TRANSMIT             Transmit interrupt                
  *            @arg CAN_IT_BUSS_ERROR           Bus error                          
  *            @arg CAN_IT_RECEIVE_OVERFLOW     Received data overflow interrupt    
  * @retval None
  */
#define __HAL_CAN_ENABLE_IT(__HANDLE__, __INTERRUPT__)  (((__HANDLE__)->Instance->CONFIG1) |= (__INTERRUPT__))

/**
  * @brief  Disable the specified CAN interrupts.
  * @param  __HANDLE__ CAN handle.
  * @param  __INTERRUPT__ CAN Interrupt sources to disable.
  *           This parameter can be any combination of @arg CAN_Interrupts
  *            @arg CAN_IT_ARBITRATION_LOSS     Arbitration loss interrupt   
  *            @arg CAN_IT_ERROR_WARNING        Error warning Interrupt          
  *            @arg CAN_IT_ERROR_PASSIVE        Error passive interrupt            
  *            @arg CAN_IT_RECEIVE              Receive interrupt                 
  *            @arg CAN_IT_TRANSMIT             Transmit interrupt                
  *            @arg CAN_IT_BUSS_ERROR           Bus error                          
  *            @arg CAN_IT_RECEIVE_OVERFLOW     Received data overflow interrupt  
  * @retval None
  */
#define __HAL_CAN_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CONFIG1) &= ~(__INTERRUPT__))
/**
  * @brief  Get the specified CAN interrupts.
  * @param  __HANDLE__ CAN handle.
  * @param  __FLAG__ CAN Interrupts.
  *           This parameter can be any combination of @arg CAN_Interrupts
  * @retval None
  */
#define __HAL_CAN_GET_ITFLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->CONFIG0 & 0xFF000000) & (__FLAG__))

/**
  * @brief  Disable the specified CAN interrupts.
  * @param  __HANDLE__ CAN handle.
  * @param  __FLAG__ CAN Interrupts.
  *           This parameter can be any combination of @arg CAN_Interrupts
  * @retval None
  */
#define __HAL_CAN_CLEAR_ITFLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->CONFIG0) = (__FLAG__))

/**
  * @brief  CAN module operation reset.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_TRANSMIT_ENABLE(__HANDLE__)               (((__HANDLE__)->Instance->CONFIG0) |= CAN_CONFIG0_TR)

/**
  * @brief  CAN module operation disable reset.
  * @param  __HANDLE__ CAN handle.
  * @retval None
  */
#define __HAL_CAN_TRANSMIT_DISABLE(__HANDLE__)               (((__HANDLE__)->Instance->CONFIG0) &= (~(CAN_CONFIG0_TR)))
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CAN_Exported_Functions
  * @{
  */ 
  
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxCallback(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);
HAL_StatusTypeDef HAL_CAN_Tx_Start(CAN_HandleTypeDef *hcan);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);

/* Private macros ------------------------------------------------------------*/
/** @defgroup xxx_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_Private_Functions UART Private Functions
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

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
