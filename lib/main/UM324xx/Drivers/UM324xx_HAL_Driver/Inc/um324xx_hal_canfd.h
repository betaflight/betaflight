 /**
  ******************************************************************************
  * @file     um324xx_hal_can.h
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
#ifndef __UM324XX_HAL_CANFD_H__
#define __UM324XX_HAL_CANFD_H__



#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"
#include "um324xx_hal_dma.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup CANFD
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CANFD_Exported_Types CANFD Exported Types
  * @{
  */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_CANFD_STATE_RESET             = 0x00U,  /*!< CANFD not yet initialized or disabled */
    HAL_CANFD_STATE_READY             = 0x01U,  /*!< CANFD initialized and ready for use   */
    HAL_CANFD_STATE_LISTENING         = 0x02U,  /*!< CANFD receive process is ongoing      */
    HAL_CANFD_STATE_SLEEP_PENDING     = 0x03U,  /*!< CANFD sleep request is pending        */
    HAL_CANFD_STATE_SLEEP_ACTIVE      = 0x04U,  /*!< CANFD sleep mode is active            */
    HAL_CANFD_STATE_ERROR             = 0x05U   /*!< CANFD error state                     */

} HAL_CANFD_StateTypeDef;

typedef enum
{
    HAL_CANFD_FilterMode_MASK         = 0x00U,  /*!< CANFD not yet initialized or disabled */
    HAL_CANFD_FilterMode_SINGLE       = 0x01U,  /*!< CANFD initialized and ready for use   */
    HAL_CANFD_FilterMode_DOUBLE       = 0x02U,  /*!< CANFD receive process is ongoing      */
    
} HAL_CANFD_FilterModeTypeDef;

/**
  * @brief  CANFD init structure definition
  */
typedef struct
{
    uint32_t Prescaler;                     /*!< Specifies the length of a time quantum.
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 63. */

    uint32_t Mode;                          /*!< Specifies the CANFD operating mode.
                                            This parameter can be a value of @ref CANFD_MODE_define */

    uint32_t SyncJumpWidth;                 /*!< Specifies the maximum number of time quanta the CANFD hardware
                                            is allowed to lengthen or shorten a bit to perform resynchronization.
                                            This parameter can be a value of @ref CANFD_synchronisation_jump_width */

    uint32_t TimeSeg1;                      /*!< Specifies the number of time quanta in Bit Segment 1.
                                            This parameter can be a value of @ref CANFD_time_quantum_in_bit_segment_1 */

    uint32_t TimeSeg2;                      /*!< Specifies the number of time quanta in Bit Segment 2.
                                            This parameter can be a value of @ref CANFD_time_quantum_in_bit_segment_2 */

    uint32_t CanFrameFormat;                /*!< Classic CANFD frame format or CANFD frame format
                                            This parameter can be a value of @ref CANFD_frame_format_define*/

    uint32_t CanLoopBack;                   /*!< CANFD loop back mode config:ENABLE or DISABLE*/

    uint32_t TxdState;                      /*!< CANFD in loop back mode, TXD terminal status selection£º
                                            This parameter can be a value of @ref CANFD_Txd_Status_config */
    
} CANFD_InitTypeDef;






/**
  * @brief  CANFD FD init structure definition
  */
typedef struct
{
    uint32_t CanfdBaudRateSwitch;         /*!< CANFD data baud rate switching
                                            This parameter can be a value of @ref CANFD_BaudRate_Switch_define*/    

    uint32_t CanfdExtbtSelect;            /*!< Arbitration phase bit time selection       
                                            This parameter can be a value of @ref  CANFD_EXTBT_define */
                                            
    uint32_t CanfdFormatSelect;           /*!< The ISO CANFD format is selected         
                                            This parameter can be a value of @ref  CANFD_FormatSelect_define */

    uint32_t CanfdNbrp;                   /*!< Baud rate predivision in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 1023. */

    uint32_t CanfdNseg1;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 255. */

    uint32_t CanfdNseg2;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 127. */

    uint32_t CanfdNsjm;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 127. */
                                            
    uint32_t CanfdDbrp;                   /*!< Baud rate predivision in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 1023. */

    uint32_t CanfdDseg1;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 63. */

    uint32_t CanfdDseg2;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 15. */

    uint32_t CanfdDsjm;                   /*!< The period before the sampling point in the arbitration phase
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 15. */                                            

} CANFD_FdTypeDef;



/**
  * @brief  CANFD filter configuration structure definition
  */
typedef struct
{
    uint32_t FilterMode;              /*!< Specifies the filter mode to be initialized.
                                       This parameter can be a value of @ref CANFD_filter_mode */

    uint32_t FilterStdId1;            /*!< Specifies the filter identification number (MSBs for a 32-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */
  
    uint32_t FilterStdId2;            /*!< Specifies the filter identification number (MSBs for a 32-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

    
    uint32_t FilterExtId1;            /*!< Specifies the filter identification number (LSBs for a 32-bit
                                       configuration, second one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */
  
    uint32_t FilterExtId2;            /*!< Specifies the filter identification number (LSBs for a 32-bit
                                       configuration, second one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

    uint32_t FilterRtr;               /*!< Specifies the filter scale.
                                       This parameter can be a value of @ref CANFD_identifier_type */
                                       
    uint32_t FilterIde;               /*!< Specifies the filter scale.
                                       This parameter can be a value of @ref CANFD_identifier_type */
                                       
    uint32_t FilterMaskType;          /*!< Specifies the filter scale.
                                    This parameter can be a value of @ref CANFD_Filter_MASK_define */
                                       
                                       
    uint32_t FilterData1;            /*!< Specifies the filter data (MSBs for a 8-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xF. */
                                       
    uint32_t FilterData2;            /*!< Specifies the filter data (MSBs for a 8-bit
                                       configuration, first one for a 16-bit configuration).
                                       This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xF. */
  
    uint32_t  CHANNEL;
  
} CANFD_FilterTypeDef;
/**
  * @brief  CANFD Tx message header structure definition
  */
typedef struct
{
    uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

    uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CANFD_identifier_type */

    uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CANFD_remote_transmission_request */

    uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 64. */
    
    uint32_t FDF;      /*!< CANFD frame format selection.
                          This parameter can be a value of @ref CANFD_frame_format_define */    
    
    uint32_t BRS;      /*!< Baud rate switching selection bit.
                          This parameter can be a value of @ref CANFD_BaudRate_Switch_define */    
    
    uint32_t SELTX;    /*!< Select the buffer to send.
                          This parameter can be a value of @ref CANFD_SETTX_BUFFER_define. */ 
  
    uint32_t DATALEN;  /*!< Number of data in TXBUFF.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 64. */
  
    uint32_t TxBufferLen;
  
    uint32_t TxXferCount;
  
    uint8_t* TxData;    

    uint32_t pTxBuffer[18];
  
} CANFD_TxHeaderTypeDef;

/**
  * @brief  CANFD Rx message header structure definition
  */
typedef struct
{
    uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

    uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CANFD_identifier_type */

    uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CANFD_remote_transmission_request */
    
    uint32_t FDF;      /*!< CANFD frame format selection.*/   
    
    uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8. */
    
    uint32_t FIDX;     /*!< The index of the matching filter (FIDX). When FILTER_ NUM_ When the C parameter 
                          is greater than 1, the filter index is always added after the received data. When the 
                          message matches filter number 0, FIXX=1, and when the message matches filter number 1, FIXD=2*/
    
    uint32_t DATALEN;         /*!< Number of data in RXBUFF.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 64. */
                          
    uint8_t* RxData;


    uint32_t pRxBuffer[18];
  
} CANFD_RxHeaderTypeDef;

/**
  * @brief  CANFD handle Structure definition
  */
typedef struct __CANFD_HandleTypeDef
{
    CANFD_TypeDef                 *Instance;                 /*!< Register base address */

    CANFD_InitTypeDef             Init;                      /*!< CANFD required parameters */

    CANFD_FdTypeDef               Fd;                        /*!< CANFD required parameters */

    CANFD_RxHeaderTypeDef         RxHeader;                  /*!< CANFD RxHeader required parameters */

    CANFD_TxHeaderTypeDef         TxHeader;                  /*!< CANFD TxHeader required parameters */

    DMA_HandleTypeDef           *hdmatx;                   /*!< CANFD Tx DMA Handle parameters */

    DMA_HandleTypeDef           *hdmarx;                   /*!< CANFD Rx DMA Handle parameters */    
    
    HAL_LockTypeDef               Lock;                    /*!< CANFD locking object */    
    
    __IO HAL_CANFD_StateTypeDef   State;                     /*!< CANFD communication state */

    __IO uint32_t               ErrorCode;                 /*!< CANFD Error code.
                                                              This parameter can be a value of @ref CANFD_Error_Code */
#if USE_HAL_CANFD_REGISTER_CALLBACKS == 1
  void (* TxCpltCallback)(struct __CANFD_HandleTypeDef *hcan);            /*!< CANFD Tx Complete Callback             */
  void (* RxCpltCallback)(struct __CANFD_HandleTypeDef *hcan);            /*!< CANFD Rx Complete Callback             */
  void (* ErrorCallback)(struct __CANFD_HandleTypeDef *hcan);             /*!< CANFD Error callback                    */

  void (* MspInitCallback)(struct __CANFD_HandleTypeDef *hcan);           /*!< CANFD Msp Init callback                 */
  void (* MspDeInitCallback)(struct __CANFD_HandleTypeDef *hcan);         /*!< CANFD Msp DeInit callback               */

#endif /* (USE_HAL_CANFD_REGISTER_CALLBACKS) */
} CANFD_HandleTypeDef;

#if USE_HAL_CANFD_REGISTER_CALLBACKS == 1
/**
  * @brief  HAL CANFD common Callback ID enumeration definition
  */
typedef enum
{
  HAL_CANFD_TX_COMPLETE_CB_ID             = 0x01U,    /*!< CANFD Tx Complete Callback ID             */
  HAL_CANFD_RX_COMPLETE_CB_ID             = 0x02U,    /*!< CANFD Rx Complete Callback ID             */
  HAL_CANFD_ERROR_CB_ID                   = 0x05U,    /*!< CANFD Error Callback ID                   */

  HAL_CANFD_MSPINIT_CB_ID                 = 0x0BU,    /*!< CANFD MspInit callback ID                 */
  HAL_CANFD_MSPDEINIT_CB_ID               = 0x0CU     /*!< CANFD MspDeInit callback ID               */

} HAL_CANFD_CallbackIDTypeDef;

/**
  * @brief  HAL CANFD Callback pointer definition
  */
typedef  void (*pCANFD_CallbackTypeDef)(CANFD_HandleTypeDef *hcan); /*!< pointer to a CANFD callback function   */

#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */
/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/


/** @defgroup UART_Error_Code CANFD Error Code
  * @{
  */
#define HAL_CANFD_ERROR_NONE              0x00000000U   /*!< ERROR NONE               */
#define HAL_CANFD_ERROR_ALI               0x00000001U   /*!< Arbitration lost         */
#define HAL_CANFD_ERROR_EWI               0x00000002U   /*!< Error warning            */
#define HAL_CANFD_ERROR_EPI               0x00000003U   /*!< Error passive            */
#define HAL_CANFD_ERROR_BEI               0x00000004U   /*!< Bus Error                */
#define HAL_CANFD_ERROR_DOI               0x00000005U   /*!< Receive data overflow    */




#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
#define  HAL_CANFD_ERROR_INVALID_CALLBACK 0x00000020U   /*!< Invalid Callback error  */
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
/**
  * @}
  */




/** @defgroup CANFD_Exported_Constants CANFD Exported Constants
  * @{
  
  */
#define CANFD_TIMEOUT                    (0xFFFFFFFFU)  

/** @defgroup CANFD_filter_mode CANFD filter mode
  * @{
  */
#define CANFD_SINGLE_FILTER               CANFD_CONFIG0_AFM
#define CANFD_DOUBLE_FILTER               0x00000000U     
  
/**
  * @}
  */
  
/** @defgroup CANFD_MODE_define CANFD MODE
  * @{
  */
#define CANFD_LISTEN_MODE               CANFD_CONFIG0_LOM
#define CANFD_NORMAL_MODE               0x00000000U    
/**
  * @}
  */  
  
/** @defgroup CANFD_RESET_WORKMODE_define CANFD RESET WORT MODE
  * @{
  */
#define CANFD_RESET_WORKMODE               CANFD_CONFIG0_RM
#define CANFD_OTHER_WORKMODE               0x00000000U    
/**
  * @}
  */  
  
/** @defgroup CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_define CANFD RECEIVE FIFO TRIGGER DEPTH
  * @{
  */ 
#define CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_1        0x00000000U
#define CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_4        CANFD_CONFIG0_RT_0
#define CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_8        CANFD_CONFIG0_RT_1
#define CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_32       CANFD_CONFIG0_RT_2
#define CANFD_RECEIVE_FIFO_TRIGGER_DEPTH_64       ((uint32_t)CANFD_CONFIG0_RT_0|CANFD_CONFIG0_RT_2)

/**
  * @}
  */

/** @defgroup CANFD_DMA_MODE_ENABLE_define CANFD DMA MODE ENABLE
  * @{
  */
#define CANFD_DMA_MODE_ENABLE                     CANFD_CONFIG0_DMA
#define CANFD_DMA_MODE_DISABLE                    0x00000000U    
/**
  * @}
  */  


 /** @defgroup CANFD_SLEEP_MODE_define CANFD SLEEP MODE
  * @{
  */
#define CANFD_ENTER_SLEEP_MODE                    CANFD_CONFIG0_SLEEP
#define CANFD_QUIT_SLEEP_MODE                     0x00000000U    
/**
  * @}
  */  



/** @defgroup CANFD_Flags   CANFD FLags
  * @{
  */
/* Flags in the SR register */
#define CANFD_FLAG_FIFO_ONE_MESSAGE               CANFD_CONFIG0_RBS
#define CANFD_FLAG_RX_FIFO_OVF                    CANFD_CONFIG0_DSO
#define CANFD_FLAG_TBS                            CANFD_CONFIG0_TBS
#define CANFD_FLAG_ERROR_PASSIVE                  CANFD_CONFIG0_EP
#define CANFD_FLAG_RECEIVING                      CANFD_CONFIG0_RS
#define CANFD_FLAG_TRANSFERRING                   CANFD_CONFIG0_TS
#define CANFD_FLAG_ES                             CANFD_CONFIG0_ES
#define CANFD_FLAG_OFFLINE                        CANFD_CONFIG0_BS


/**
  * @}
  */



/** @defgroup CANFD_IT_Flags   CANFD IT FLags
  * @{
  */
/* Flags in the ISR register */
#define CANFD_IT_FLAG_WAKEUP                      CANFD_CONFIG0_WUI
#define CANFD_IT_FLAG_ARBITRATION_LOST            CANFD_CONFIG0_ALI
#define CANFD_IT_FLAG_ERROR_WARNING               CANFD_CONFIG0_EWI
#define CANFD_IT_FLAG_ERROR_PASSIVE               CANFD_CONFIG0_EPI
#define CANFD_IT_FLAG_RECEIVE                     CANFD_CONFIG0_RI
#define CANFD_IT_FLAG_TRANSFER                    CANFD_CONFIG0_TI
#define CANFD_IT_FLAG_BUS_ERROR                   CANFD_CONFIG0_BEI
#define CANFD_IT_FLAG_RECEIVE_OVF                 CANFD_CONFIG0_DOI
#define CANFD_IT_FLAG_Msk                         (uint32_t)(CANFD_IT_FLAG_WAKEUP|CANFD_IT_FLAG_ARBITRATION_LOST |\
                                                CANFD_IT_FLAG_ERROR_WARNING|CANFD_IT_FLAG_ERROR_PASSIVE|CANFD_IT_FLAG_RECEIVE|\
                                                CANFD_IT_FLAG_TRANSFER|CANFD_IT_FLAG_BUS_ERROR|CANFD_IT_FLAG_RECEIVE_OVF)

/**
  * @}
  */
  
  
/** @defgroup CANFD_Interrupt_definition  CANFD Interrupt Definitions
  *           IT mask in the IMR register
  * @{
  */
#define CANFD_IT_WAKEUP                           CANFD_CONFIG1_WUIM
#define CANFD_IT_ARBITRATION_LOST                 CANFD_CONFIG1_ALIM
#define CANFD_IT_ERROR_WARNING                    CANFD_CONFIG1_EWIM
#define CANFD_IT_ERROR_PASSIVE                    CANFD_CONFIG1_EPIM
#define CANFD_IT_RECEIVE                          CANFD_CONFIG1_RIM
#define CANFD_IT_TRANSFER                         CANFD_CONFIG1_TIM
#define CANFD_IT_BUS_ERROR                        CANFD_CONFIG1_BEIM
#define CANFD_IT_RECEIVE_OVF                      CANFD_CONFIG1_DOIM

/**
  * @}
  */


/** @defgroup CANFD_synchronisation_jump_width CANFD Synchronization Jump Width
  * @{
  */
#define CANFD_SJW_1TQ                 (0x00000000U)               /*!< 1 time quantum */
#define CANFD_SJW_2TQ                 CANFD_CONFIG1_SJW_0          /*!< 2 time quantum */
#define CANFD_SJW_3TQ                 CANFD_CONFIG1_SJW_1          /*!< 3 time quantum */
#define CANFD_SJW_4TQ                 CANFD_CONFIG1_SJW            /*!< 4 time quantum */
/**
  * @}
  */


/** @defgroup CANFD_time_quantum_in_bit_segment_1 CANFD Time Quantum in Bit Segment 1
  * @{
  */

#define CANFD_BS1_1TQ                 (0x00000000U)                                                                   /*!< 1 time quantum  */
#define CANFD_BS1_2TQ                 ((uint32_t)CANFD_CONFIG1_TSEG1_0)                                                 /*!< 2 time quantum  */
#define CANFD_BS1_3TQ                 ((uint32_t)CANFD_CONFIG1_TSEG1_1)                                                 /*!< 3 time quantum  */
#define CANFD_BS1_4TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG1_1 | CANFD_CONFIG1_TSEG1_0))                         /*!< 4 time quantum  */
#define CANFD_BS1_5TQ                 ((uint32_t)CANFD_CONFIG1_TSEG1_2)                                                 /*!< 5 time quantum  */
#define CANFD_BS1_6TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG1_2 | CANFD_CONFIG1_TSEG1_0))                         /*!< 6 time quantum  */
#define CANFD_BS1_7TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG1_2 | CANFD_CONFIG1_TSEG1_1))                         /*!< 7 time quantum  */
#define CANFD_BS1_8TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG1_2 | CANFD_CONFIG1_TSEG1_1 | CANFD_CONFIG1_TSEG1_0))   /*!< 8 time quantum  */
#define CANFD_BS1_9TQ                 ((uint32_t)CANFD_CONFIG1_TSEG1_3)                                                 /*!< 9 time quantum  */
#define CANFD_BS1_10TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_0))                         /*!< 10 time quantum */
#define CANFD_BS1_11TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_1))                         /*!< 11 time quantum */
#define CANFD_BS1_12TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_1 | CANFD_CONFIG1_TSEG1_0))   /*!< 12 time quantum */
#define CANFD_BS1_13TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_2))                         /*!< 13 time quantum */
#define CANFD_BS1_14TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_2 | CANFD_CONFIG1_TSEG1_0))   /*!< 14 time quantum */
#define CANFD_BS1_15TQ                ((uint32_t)(CANFD_CONFIG1_TSEG1_3 | CANFD_CONFIG1_TSEG1_2 | CANFD_CONFIG1_TSEG1_1))   /*!< 15 time quantum */
#define CANFD_BS1_16TQ                ((uint32_t)CANFD_CONFIG1_TSEG1)                                                   /*!< 16 time quantum */
/**
  * @}
  */


/** @defgroup CANFD_time_quantum_in_bit_segment_2 CANFD Time Quantum in Bit Segment 2
  * @{
  */
#define CANFD_BS2_1TQ                 (0x00000000U)                                           /*!< 1 time quantum */
#define CANFD_BS2_2TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_0))                       /*!< 2 time quantum */
#define CANFD_BS2_3TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_1))                       /*!< 3 time quantum */
#define CANFD_BS2_4TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_1 | CANFD_CONFIG1_TSEG2_0)) /*!< 4 time quantum */
#define CANFD_BS2_5TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_2))                       /*!< 5 time quantum */
#define CANFD_BS2_6TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_2 | CANFD_CONFIG1_TSEG2_0)) /*!< 6 time quantum */
#define CANFD_BS2_7TQ                 ((uint32_t)(CANFD_CONFIG1_TSEG2_2 | CANFD_CONFIG1_TSEG2_1)) /*!< 7 time quantum */
#define CANFD_BS2_8TQ                 ((uint32_t)CANFD_CONFIG1_TSEG2)                           /*!< 8 time quantum */
/**
  * @}
  */
  

/** @defgroup CANFD_US_LEVEL_SAMPLES_NUM_define CANFD Number of bus level samples
  * @{
  */
#define CANFD_US_LEVEL_SAMPLES_NUM_1        (0x00000000U)                         
#define CANFD_US_LEVEL_SAMPLES_NUM_3        CANFD_CONFIG1_SAM

/**
  * @}
  */


/** @defgroup CANFD_identifier_type CANFD Identifier Type
  * @{
  */
#define CANFD_ID_STD                  (0x00000000U)  /*!< Standard Id */
#define CANFD_ID_EXT                  (0x00000001U)  /*!< Extended Id */

#define CANFD_RTR_DATA                (0x00000000U)  
#define CANFD_RTR_CONTROL             (0x00000001U)  /*!< Extended Id */
/**
  * @}
  */


  
/** @defgroup CANFD_Filter_MASK_define CANFD Filter MASK define
  * @{
  */
#define CANFD_Filter_MASK_ALL                     (0xffffffffU)
#define CANFD_Filter_MASK_NONE                    (0x00000000U)
#define CANFD_Filter_MASK_ID_SINGLE               (0x0000FFFFU) 
#define CANFD_Filter_MASK_DATA_SINGLE             (0xFFFF0000U)
#define CANFD_Filter_MASK_DATA_DOUBLE             (0x0F000F00U)

/**
  * @}
  */
  


/** @defgroup CANFD_frame_format_define CANFD frame format
  * @{
  */

#define CANFD_FRAME_FORMAT_NORMAL         0x00000000U
#define CANFD_FRAME_FORMAT_FD             CANFD_CONFIG2_FDEN

/**
  * @}
  */

/** @defgroup CANFD_Txd_Status_config CANFD Txd in loop back Status config
  * @{
  */


#define CANFD_TXD_INTXPORT                0x00000000U
#define CANFD_TXD_KEEPHIDDEN              CANFD_WUPTEST_TXC
/**
  * @}
  */


/** @defgroup CANFD_BaudRate_Switch_define CANFD BaudRate Switch define
  * @{
  */

#define CANFD_BAUDRATE_SWITCH           CANFD_CONFIG2_BRSEN
#define CANFD_BAUDRATE_NON_SWITCH       0x00000000U

/**
  * @}
  */


/** @defgroup CANFD_EXTBT_define CANFD EXTBT define
  * @{
  */

#define CANFD_EXTBT_SELECT_BTR          0x00000000U
#define CANFD_EXTBT_SELECT_NBT          CANFD_CONFIG2_EXTBT

/**
  * @}
  */

/** @defgroup CANFD_FormatSelect_define CANFD Format Select define
  * @{
  */

#define CANFD_FORMAT_SELECT_BOSCH        0x00000000U
#define CANFD_FORMAT_SELECT_ISO          CANFD_CONFIG2_ISO

/**
  * @}
  */
  
/** @defgroup CANFD_SETTX_BUFFER_define CANFD SETTX BUFFER define
  * @{
  */

#define CANFD_SETTX_BUFFER0               0x00000000U
#define CANFD_SETTX_BUFFER1               CANFD_RTCONFIG_SELTXSEL_0
#define CANFD_SETTX_BUFFER2               CANFD_RTCONFIG_SELTXSEL_1
/**
  * @}
  */  


/** @defgroup CANFD_TXB_define CANFD TXB define 
  * @{
  */

#define CANFD_TXB0                    CANFD_CONFIG0_CMD0
#define CANFD_TXB1                    CANFD_CONFIG0_CMD1
#define CANFD_TXB2                    CANFD_CONFIG0_CMD2
/**
  * @}
  */  



/** @defgroup CANFD_TXB_CMD_define CANFD TXB CMD define 
  * @{
  */

#define CANFD_TXB_CMD_ABORT_TRAMSMIT      0x00000001U
#define CANFD_TXB_CMD_TRAMSMIT_REQ        0x00000002U
#define CANFD_TXB_CMD_SINGLE_TRAMSMIT_REQ 0x00000003U
/**
  * @}
  */  


/* Exported macro ------------------------------------------------------------*/
/** @defgroup CANFD_Exported_Macros CANFD Exported Macros
  * @{
  */

/*CANFD module operation reset*/
#define __HAL_CANFD_RESET_ENABLE(__HANDLE__)                SET_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_RM)

/*Exit CANFD module operation reset*/
#define __HAL_CANFD_RESET_DISABLE(__HANDLE__)               CLEAR_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_RM)

/*CANFD module monitoring mode*/
#define __HAL_CANFD_MONITOR_MODE(__HANDLE__)                SET_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_LOM)

/*CANFD module normal mode*/
#define __HAL_CANFD_NORMAL_MODE(__HANDLE__)                 CLEAR_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_LOM)

/*CANFD module setting single filter*/
#define __HAL_CANFD_SINGLE_FILTER(__HANDLE__)               SET_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_AFM)

/*CANFD module setting double filter*/
#define __HAL_CANFD_DOUBLE_FILTER(__HANDLE__)               CLEAR_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_AFM)

/** @brief  Enable the specified CANFD interrupts.
  * @param  __HANDLE__ specifies the CANFD Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:             
  *            @arg CANFD_IT_WAKEUP                         
  *            @arg CANFD_IT_ARBITRATION_LOST                
  *            @arg CANFD_IT_ERROR_WARNING                   
  *            @arg CANFD_IT_ERROR_PASSIVE                    
  *            @arg CANFD_IT_RECEIVE                          
  *            @arg CANFD_IT_TRANSFER                         
  *            @arg CANFD_IT_BUS_ERROR                       
  *            @arg CANFD_IT_RECEIVE_OVF                      
  * @retval None
  */
#define __HAL_CANFD_ENABLE_IT(__HANDLE__, __INTERRUPT__)        SET_BIT((__HANDLE__)->Instance->CONFIG1, (__INTERRUPT__))  

/** @brief  Disable the specified CANFD interrupts.
  * @param  __HANDLE__ specifies the CANFD handle.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg CANFD_IT_WAKEUP                         
  *            @arg CANFD_IT_ARBITRATION_LOST                
  *            @arg CANFD_IT_ERROR_WARNING                   
  *            @arg CANFD_IT_ERROR_PASSIVE                    
  *            @arg CANFD_IT_RECEIVE                          
  *            @arg CANFD_IT_TRANSFER                         
  *            @arg CANFD_IT_BUS_ERROR                       
  *            @arg CANFD_IT_RECEIVE_OVF                
  * @retval None
  */
#define __HAL_CANFD_DISABLE_IT(__HANDLE__, __INTERRUPT__)       CLEAR_BIT((__HANDLE__)->Instance->CONFIG1, (__INTERRUPT__))  

/*CANFD RX fifo clear*/
#define __HAL_CANFD_RXF_CLR(__HANDLE__)                     SET_BIT((__HANDLE__)->Instance->CONFIG0,CANFD_CONFIG0_RXFIFORST)


/** @brief  Clear the CANFD interrupt flag.
  * @param  __HANDLE__ specifies the CANFD Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg CANFD_IT_FLAG_WAKEUP                     
  *            @arg CANFD_IT_FLAG_ARBITRATION_LOST            
  *            @arg CANFD_IT_FLAG_ERROR_WARNING               
  *            @arg CANFD_IT_FLAG_ERROR_PASSIVE               
  *            @arg CANFD_IT_FLAG_RECEIVE                     
  *            @arg CANFD_IT_FLAG_TRANSFER                    
  *            @arg CANFD_IT_FLAG_BUS_ERROR                   
  *            @arg CANFD_IT_FLAG_RECEIVE_OVF                             
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CANFD_CLEAR_ITFLAG(__HANDLE__, __INTERRUPT__)     SET_BIT((__HANDLE__)->Instance->CONFIG0, (__INTERRUPT__))  


/** @brief  Check whether the specified CANFD interrupt flag is set or not.
  * @note   If an interrupt is not enabled, the corresponding interrupt flag in this register will not be set.
  * @param  __HANDLE__ specifies the CANFD Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg CANFD_IT_FLAG_WAKEUP                     
  *            @arg CANFD_IT_FLAG_ARBITRATION_LOST            
  *            @arg CANFD_IT_FLAG_ERROR_WARNING               
  *            @arg CANFD_IT_FLAG_ERROR_PASSIVE               
  *            @arg CANFD_IT_FLAG_RECEIVE                     
  *            @arg CANFD_IT_FLAG_TRANSFER                    
  *            @arg CANFD_IT_FLAG_BUS_ERROR                   
  *            @arg CANFD_IT_FLAG_RECEIVE_OVF                   
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CANFD_GET_FLAG(__HANDLE__, __FLAG__)              ((((__HANDLE__)->Instance->CONFIG0) & (__FLAG__)) == (__FLAG__))  


/*Get can error status*/
#define __HAL_CANFD_GET_ERR(__HANDLE__)                        READ_REG((__HANDLE__)->Instance->ERRCR)  



 /**
  * @}
  */ 


HAL_StatusTypeDef HAL_CANFD_Init(CANFD_HandleTypeDef *hcan);


void HAL_CANFD_MspInit(CANFD_HandleTypeDef *hcan);
void HAL_CANFD_MspDeInit(CANFD_HandleTypeDef *hcan);


HAL_StatusTypeDef HAL_CANFD_AddTxMessage(CANFD_HandleTypeDef *hcan, uint8_t pData[]);
HAL_StatusTypeDef HAL_CANFD_Transmit(CANFD_HandleTypeDef *hcan, uint32_t Timeout);

/**
  * @brief  CANFD TX buffer send command.  
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  CMD Send command.
  *       
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  *     @retval    HAL_TIMEOUT something wrong
  */
void HAL_CANFD_TXCMD(CANFD_HandleTypeDef *hcan, uint32_t CMD);

/**
  * @brief  Get an CANFD frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  pHeader pointer to a CANFD_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @param  pData array where the payload of the Rx frame will be stored.
  * @param  Timeout Timeout duration.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  *     @retval    HAL_TIMEOUT something wrong
  */
HAL_StatusTypeDef HAL_CANFD_Receive(CANFD_HandleTypeDef *hcan, uint32_t Timeout);


/**
  * @brief  Configures the CANFD reception filter according to the specified
  *         parameters in the CANFD_FilterInitStruct.
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  sFilterConfig pointer to a CANFD_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_CANFD_ConfigFilter(CANFD_HandleTypeDef *hcan, CANFD_FilterTypeDef *sFilterConfig);

#ifdef HAL_DMA_MODULE_ENABLED
/**
  * @brief  Sends an amount of data in DMA mode.
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *              the configuration information for the specified CANFD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CANFD_Transmit_DMA(CANFD_HandleTypeDef *hcan);
#endif

/**
  * @brief  Get an CANFD frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  pHeader pointer to a CANFD_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @param  pData array where the payload of the Rx frame will be stored.
  * @param  Timeout Timeout duration.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  *     @retval    HAL_TIMEOUT something wrong
  */
HAL_StatusTypeDef HAL_CANFD_GetRxMessage(CANFD_HandleTypeDef *hcan, uint8_t* pData);

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *               the configuration information for the specified CANFD module.
  * @param  pHeader pointer to a CANFD_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CANFD_Receive_IT(CANFD_HandleTypeDef *hcan);

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CANFD_Transmit_IT(CANFD_HandleTypeDef *hcan);


/**
  * @brief  This function handles CANFD interrupt request.
  * @param  huart  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @param  pHeader pointer to a CANFD_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @retval None
  */
void HAL_CANFD_IRQHandler(CANFD_HandleTypeDef *hcan);

#if USE_HAL_CANFD_REGISTER_CALLBACKS == 1
/**
  * @brief  Unregister a CANFD callback
  *         CANFD callback is redirected to the weak predefined callback
  * @param hcan can handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_CANFD_TX_COMPLETE_CB_ID CANFD Tx Complete ID
  *          @arg @ref HAL_CANFD_RX_COMPLETE_CB_ID CANFD Rx Complete ID
  *          @arg @ref HAL_CANFD_ERROR_CB_ID CANFD Error ID
  *          @retval status
  */
HAL_StatusTypeDef HAL_CANFD_UnRegisterCallback(CANFD_HandleTypeDef *hcan, HAL_CANFD_CallbackIDTypeDef CallbackID);

/**
  * @brief Register a User CANFD callback to be used instead of the weak predefined callback
  * @param hcan can handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_CANFD_TX_COMPLETE_CB_ID CANFD Tx Complete ID
  *          @arg @ref HAL_CANFD_RX_COMPLETE_CB_ID CANFD Rx Complete ID
  *          @arg @ref HAL_CANFD_ERROR_CB_ID CANFD Error ID

  *          @param pCallback pointer to the callback function
  *          @retval status
  */
HAL_StatusTypeDef HAL_CANFD_RegisterCallback(CANFD_HandleTypeDef *hcan, HAL_CANFD_CallbackIDTypeDef CallbackID,
        pCANFD_CallbackTypeDef pCallback);

#endif /* (USE_HAL_CANFD_REGISTER_CALLBACKS) */

void HAL_CANFD_TxCpltCallback(CANFD_HandleTypeDef *hcan);
void HAL_CANFD_RxCpltCallback(CANFD_HandleTypeDef *hcan);
void HAL_CANFD_ErrorCallback(CANFD_HandleTypeDef *hcan);


/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __UM32x42x_HAL_CANFD_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/





