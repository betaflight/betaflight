 /**
  ******************************************************************************
  * @file     um324xx_hal_usart.h
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-05-04
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
#ifndef __UM324XX_HAL_USART_H__
#define __UM324XX_HAL_USART_H__



#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup USART
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup USART_Exported_typedefs USART Exported Typedefs
  * @{
  */ 

/**
  * @brief USART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (16 * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5*/

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

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
	
  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref USART_Over_Sampling */

} USART_InitTypeDef;

/**
  * @brief  USART SPI Configuration Structure definition
  */
typedef struct
{
    uint32_t Mode;                  /*!< Specifies the SPI Master-slave mode.
                                         This parameter can be a value of @ref USART_SPI_Mode */
    
    uint32_t BaudRatePrescaler;     /*!< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         This parameter can be a value  customizes by yourself
                                         @note The communication clock is derived from the master
                                         clock. The slave clock does not need to be set. */
    
    uint32_t CLKPolarity;           /*!< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref USART_SPI_Clock_Polarity */

    uint32_t CLKPhase;              /*!< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref USART_SPI_Clock_Phase */

}USART_SPI_InitTypeDef;

/**
  * @brief  USART SPI Configuration Structure definition
  */
typedef struct
{
    uint32_t Mode;                  /*!< Specifies the LIN Master-slave mode.
                                         This parameter can be a value of @ref USART_LIN_Mode */
    
	uint32_t BaudRate;              /*!< This member configures the USART LIN communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (16 * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5*/
	
	uint32_t OverSampling;          /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                         This parameter can be a value of @ref USART_Over_Sampling */

}USART_LIN_InitTypeDef;

/**
  * @brief HAL USART State structures definition
  */
typedef enum
{
  HAL_USART_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  HAL_USART_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  HAL_USART_STATE_BUSY              = 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  HAL_USART_STATE_BUSY_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  HAL_USART_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  HAL_USART_STATE_BUSY_TX_RX        = 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  HAL_USART_STATE_TIMEOUT           = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  HAL_USART_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} HAL_USART_StateTypeDef;

/**
  * @brief  USART handle Structure definition
  */
typedef struct __USART_HandleTypeDef
{
  USART_TypeDef                 *Instance;        /*!< USART registers base address        */

  USART_InitTypeDef             Init;             /*!< USART communication parameters      */
	
  USART_SPI_InitTypeDef         SpiInit;          /*!< SPI communication parameters        */
	
  USART_LIN_InitTypeDef         LinInit;          /*!< LIN communication parameters        */	

  const uint8_t                 *pTxBuffPtr;      /*!< Pointer to USART Tx transfer Buffer */

  uint16_t                      TxXferSize;       /*!< USART Tx Transfer size              */

  __IO uint16_t                 TxXferCount;      /*!< USART Tx Transfer Counter           */

  uint8_t                       *pRxBuffPtr;      /*!< Pointer to USART Rx transfer Buffer */

  uint16_t                      RxXferSize;       /*!< USART Rx Transfer size              */

  __IO uint16_t                 RxXferCount;      /*!< USART Rx Transfer Counter           */
	
  DMA_HandleTypeDef             *hdmatx;          /*!< USART Tx DMA Handle parameters      */

  DMA_HandleTypeDef             *hdmarx;          /*!< USART Rx DMA Handle parameters      */

  HAL_LockTypeDef               Lock;             /*!< Locking object                      */

  __IO HAL_USART_StateTypeDef    gState;          /*!< USART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_USART_StateTypeDef */

  __IO HAL_USART_StateTypeDef    RxState;         /*!< USART state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_USART_StateTypeDef */

  __IO uint32_t                 ErrorCode;        /*!< USART Error code                    */

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
  void (* TxCpltCallback)(struct __USART_HandleTypeDef *huart);            /*!< USART Tx Complete Callback             */
  void (* RxCpltCallback)(struct __USART_HandleTypeDef *huart);            /*!< USART Rx Complete Callback             */
  void (* ErrorCallback)(struct __USART_HandleTypeDef *huart);             /*!< USART Error Callback                   */

  void (* MspInitCallback)(struct __USART_HandleTypeDef *huart);           /*!< USART Msp Init callback                */
  void (* MspDeInitCallback)(struct __USART_HandleTypeDef *huart);         /*!< USART Msp DeInit callback              */
#endif  /* USE_HAL_USART_REGISTER_CALLBACKS */

} USART_HandleTypeDef;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL USART Callback ID enumeration definition
  */
typedef enum
{
  HAL_USART_TX_COMPLETE_CB_ID             = 0x01U,    /*!< USART Tx Complete Callback ID             */
  HAL_USART_RX_COMPLETE_CB_ID             = 0x03U,    /*!< USART Rx Complete Callback ID             */
  HAL_USART_ERROR_CB_ID                   = 0x04U,    /*!< USART Error Callback ID                   */

  HAL_USART_MSPINIT_CB_ID                 = 0x0BU,    /*!< USART MspInit callback ID                 */
  HAL_USART_MSPDEINIT_CB_ID               = 0x0CU     /*!< USART MspDeInit callback ID               */

} HAL_USART_CallbackIDTypeDef;

/**
  * @brief  HAL USART Callback pointer definition
  */
typedef  void (*pUSART_CallbackTypeDef)(USART_HandleTypeDef *huart);  /*!< pointer to an USART callback function */

#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup USART_Exported_constants USART Exported Constants
  * @{
  */ 

/** @defgroup USART_Error_Code USART Error Code
  * @{
  */
#define HAL_USART_ERROR_NONE              0x00000000U   /*!< No error            */
#define HAL_USART_ERROR_PARE              0x00000001U   /*!< Parity error        */
#define HAL_USART_ERROR_FRAME             0x00000002U   /*!< Frame error         */
#define HAL_USART_ERROR_OVRE              0x00000004U   /*!< Overrun error       */
#define HAL_USART_ERROR_TIMEOUT           0x00000010U   /*!< Time out error      */
#define HAL_USART_ERROR_DMA           	  0x00000020U   /*!< DMA error           */
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
#define  HAL_USART_ERROR_INVALID_CALLBACK 0x00000008U   /*!< Invalid Callback error  */
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */
  
/** @defgroup USART_Word_Length USART Word Length
  * @{
  */
#define USART_WORDLENGTH_5B                  0x00000000U
#define USART_WORDLENGTH_6B                  ((uint32_t)USART_MR_CHRL)
#define USART_WORDLENGTH_7B                  ((uint32_t)USART_MR_CHRL_1)
#define USART_WORDLENGTH_8B                  ((uint32_t)(USART_MR_CHRL_0 | USART_MR_CHRL_1))
#define USART_WORDLENGTH_9B                  ((uint32_t)USART_MR_MODE9)
/**
  * @}
  */
  
/** @defgroup USART_Stop_Bits USART Number of Stop Bits
  * @{
  */
#define USART_STOPBITS_1                     0x00000000U
#define USART_STOPBITS_1_5                   ((uint32_t)USART_MR_NBSTOP_0)
#define USART_STOPBITS_2                     ((uint32_t)USART_MR_NBSTOP_1)
/**
  * @}
  */
  
/** @defgroup USART_Parity USART Parity
  * @{
  */
#define USART_PARITY_EVEN                    0x00000000U
#define USART_PARITY_ODD                     ((uint32_t)USART_MR_PAR_0)
#define USART_PARITY_SPACE                   ((uint32_t)USART_MR_PAR_1)
#define USART_PARITY_MARK                    ((uint32_t)(USART_MR_PAR_0 | USART_MR_PAR_1))
#define USART_PARITY_NONE                    ((uint32_t)USART_MR_PAR_2)
#define USART_PARITY_MULTIDROP               ((uint32_t)(USART_MR_PAR_1 | USART_MR_PAR_2))
/**
  * @}
  */

/** @defgroup USART_Hardware_Flow_Control USART Hardware Flow Control
  * @{
  */
#define USART_HWCONTROL_NONE                  0x00000000U
#define USART_HWCONTROL_ENABLE                ((uint32_t)USART_MR_MODE_1)
/**
  * @}
  */
  
/** @defgroup USART_Over_Sampling USART Over Sampling
  * @{
  */
#define USART_OVERSAMPLING_16                    0x00000000U
#define USART_OVERSAMPLING_8                     ((uint32_t)USART_MR_OVER)
/**
  * @}
  */

/** @defgroup USART_Mode USART Transfer Mode
  * @{
  */
#define USART_MODE_RX                        ((uint32_t)USART_CR_RXEN)
#define USART_MODE_TX                        ((uint32_t)USART_CR_TXEN)
#define USART_MODE_TX_RX                     ((uint32_t)(USART_CR_RXEN | USART_CR_TXEN))
/**
  * @}
  */

/** @defgroup USART_Flags   USART FLags
  * @{
  */
/* Flags in the CSR register */
#define USART_FLAG_MANERR                  	  ((uint32_t)USART_CSR_MANERR)
#define USART_FLAG_CTS                        ((uint32_t)USART_CSR_CTS)
#define USART_FLAG_DCD                        ((uint32_t)USART_CSR_DCD)
#define USART_FLAG_DSR                        ((uint32_t)USART_CSR_DSR)
#define USART_FLAG_RI                         ((uint32_t)USART_CSR_RI)
#define USART_FLAG_CTSIC                      ((uint32_t)USART_CSR_CTSIC)
#define USART_FLAG_DCDIC                      ((uint32_t)USART_CSR_DCDIC)
#define USART_FLAG_DSRIC                      ((uint32_t)USART_CSR_DSRIC)
#define USART_FLAG_RIIC                       ((uint32_t)USART_CSR_RIIC)
#define USART_FLAG_NACK                  	  ((uint32_t)USART_CSR_NACK)
#define USART_FLAG_LINTC                  	  ((uint32_t)USART_CSR_LIN_LINTC)
#define USART_FLAG_LINID                  	  ((uint32_t)USART_CSR_LIN_LINID)
#define USART_FLAG_LINBK                  	  ((uint32_t)USART_CSR_LIN_LINBK)
#define USART_FLAG_RXBUFF                     ((uint32_t)USART_CSR_RXBUFF)
#define USART_FLAG_TXBUFE                     ((uint32_t)USART_CSR_TXBUFE)
#define USART_FLAG_ITER                       ((uint32_t)USART_CSR_ITER)
#define USART_FLAG_TXEMPTY                    ((uint32_t)USART_CSR_TXEMPTY)
#define USART_FLAG_TIMEOUT                    ((uint32_t)USART_CSR_TIMEOUT)
#define USART_FLAG_PARE                       ((uint32_t)USART_CSR_PARE)
#define USART_FLAG_FRAME                      ((uint32_t)USART_CSR_FRAME)
#define USART_FLAG_OVRE                       ((uint32_t)USART_CSR_OVRE)
#define USART_FLAG_ENDTX                      ((uint32_t)USART_CSR_ENDTX)
#define USART_FLAG_ENDRX                      ((uint32_t)USART_CSR_ENDRX)
#define USART_FLAG_RXBRK                      ((uint32_t)USART_CSR_RXBRK)
#define USART_FLAG_TXRDY                      ((uint32_t)USART_CSR_TXRDY)
#define USART_FLAG_RXRDY                      ((uint32_t)USART_CSR_RXRDY)

/**
  * @}
  */

/** @defgroup USART_Interrupt_definition  USART Interrupt Definitions
  *           IT mask in the IMR register
  * @{
  */
#define USART_IT_MANE                   	((uint32_t)USART_IER_MANE)
#define USART_IT_CTSIC                      ((uint32_t)USART_IER_CTSIC)
#define USART_IT_DCDIC                      ((uint32_t)USART_IER_DCDIC)
#define USART_IT_DSRIC                      ((uint32_t)USART_IER_DSRIC)
#define USART_IT_RIIC                       ((uint32_t)USART_IER_RIIC)
#define USART_IT_NACK                  	    ((uint32_t)USART_IER_NACK)
#define USART_IT_RXBUFF                     ((uint32_t)USART_IER_RXBUFF)
#define USART_IT_TXBUFE                     ((uint32_t)USART_IER_TXBUFE)
#define USART_IT_ITER                       ((uint32_t)USART_IER_ITER)
#define USART_IT_TXEMPTY                    ((uint32_t)USART_IER_TXEMPTY)
#define USART_IT_TIMEOUT                    ((uint32_t)USART_IER_TIMEOUT)
#define USART_IT_PARE                       ((uint32_t)USART_IER_PARE)
#define USART_IT_FRAME                      ((uint32_t)USART_IER_FRAME)
#define USART_IT_OVRE                       ((uint32_t)USART_IER_OVRE)
#define USART_IT_ENDTX                      ((uint32_t)USART_IER_ENDTX)
#define USART_IT_ENDRX                      ((uint32_t)USART_IER_ENDRX)
#define USART_IT_RXBRK                      ((uint32_t)USART_IER_RXBRK)
#define USART_IT_TXRDY                      ((uint32_t)USART_IER_TXRDY)
#define USART_IT_RXRDY                      ((uint32_t)USART_IER_RXRDY)

/**
  * @}
  */

/** @defgroup USART_SPI_Mode  USART SPI Mode
  * @{
  */
#define USART_SPI_MODE_MASTER                 	((uint32_t)(USART_MR_SPI_MODE_1 | USART_MR_SPI_MODE_2 | USART_MR_SPI_MODE_3))
#define USART_SPI_MODE_SLAVE                    ((uint32_t)USART_MR_SPI_MODE)
/**
  * @}
  */

/** @defgroup USART_LIN_Mode  USART LIN Mode
  * @{
  */
#define USART_LIN_MODE_MASTER                 	((uint32_t)(USART_MR_USART_MODE_1 | USART_MR_USART_MODE_3))
#define USART_LIN_MODE_SLAVE                    ((uint32_t)(USART_MR_USART_MODE_0 | USART_MR_USART_MODE_1 | USART_MR_USART_MODE_3))
/**
  * @}
  */

/** @defgroup USART_LIN_Work_Mode  USART LIN Work Mode
  * @{
  */
#define USART_LIN_WORK_MODE_PUBLISH             (0x00000000U)
#define USART_LIN_WORK_MODE_SUBSCRIBE           ((uint32_t)USART_LINMR_NACT_0)
#define USART_LIN_WORK_MODE_IGNORE	            ((uint32_t)USART_LINMR_NACT_1)
/**
  * @}
  */

/** @defgroup USART_SPI_Clock_Polarity USART SPI Clock Polarity
  * @{
  */
#define USART_SPI_POLARITY_LOW                (0x00000000U)
#define USART_SPI_POLARITY_HIGH               USART_MR_SPI_CPOL
/**
  * @}
  */
  
/** @defgroup USART_SPI_Clock_Phase USART SPI Clock Phase
  * @{
  */
#define USART_SPI_PHASE_1EDGE                 USART_MR_SPI_CPHA
#define USART_SPI_PHASE_2EDGE                 (0x00000000U)
/**
  * @}
  */  

/** @defgroup RCM_CFGR1_USART6_DIV USART6 Clock Division
  * @{
  */
#define USART6_CLK_DIV2        		RCM_CFGR1_USART6_DIV2 
#define USART6_CLK_DIV3        		RCM_CFGR1_USART6_DIV3 
#define USART6_CLK_DIV4        		RCM_CFGR1_USART6_DIV4 
#define USART6_CLK_DIV5        		RCM_CFGR1_USART6_DIV5 
#define USART6_CLK_DIV6        		RCM_CFGR1_USART6_DIV6 
#define USART6_CLK_DIV7        		RCM_CFGR1_USART6_DIV7 
#define USART6_CLK_DIV8        		RCM_CFGR1_USART6_DIV8 
#define USART6_CLK_DIV9        		RCM_CFGR1_USART6_DIV9 
#define USART6_CLK_DIV10       		RCM_CFGR1_USART6_DIV10
#define USART6_CLK_DIV11       		RCM_CFGR1_USART6_DIV11
#define USART6_CLK_DIV12       		RCM_CFGR1_USART6_DIV12
#define USART6_CLK_DIV13       		RCM_CFGR1_USART6_DIV13
#define USART6_CLK_DIV14       		RCM_CFGR1_USART6_DIV14
#define USART6_CLK_DIV15       		RCM_CFGR1_USART6_DIV15
#define USART6_CLK_DIV16       		RCM_CFGR1_USART6_DIV16
/**
  * @}
  */

/** @defgroup RCM_CFGR1_USART7_DIV USART7 Clock Division
  * @{
  */
#define USART7_CLK_DIV2        		RCM_CFGR1_USART7_DIV2 
#define USART7_CLK_DIV3        		RCM_CFGR1_USART7_DIV3 
#define USART7_CLK_DIV4        		RCM_CFGR1_USART7_DIV4 
#define USART7_CLK_DIV5        		RCM_CFGR1_USART7_DIV5 
#define USART7_CLK_DIV6        		RCM_CFGR1_USART7_DIV6 
#define USART7_CLK_DIV7        		RCM_CFGR1_USART7_DIV7 
#define USART7_CLK_DIV8        		RCM_CFGR1_USART7_DIV8 
#define USART7_CLK_DIV9        		RCM_CFGR1_USART7_DIV9 
#define USART7_CLK_DIV10       		RCM_CFGR1_USART7_DIV10
#define USART7_CLK_DIV11       		RCM_CFGR1_USART7_DIV11
#define USART7_CLK_DIV12       		RCM_CFGR1_USART7_DIV12
#define USART7_CLK_DIV13       		RCM_CFGR1_USART7_DIV13
#define USART7_CLK_DIV14       		RCM_CFGR1_USART7_DIV14
#define USART7_CLK_DIV15       		RCM_CFGR1_USART7_DIV15
#define USART7_CLK_DIV16       		RCM_CFGR1_USART7_DIV16
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USART_Exported_macro USART Exported Macro
  * @{
  */ 

/** @brief  Checks whether the specified USART flag is set or not.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg USART_FLAG_MANERR:   	Manchester Error flag
  *            @arg USART_FLAG_CTS:      	Image of CTS Input flag
  *            @arg USART_FLAG_DCD:      	Image of DCD Input flag
  *            @arg USART_FLAG_DSR:      	Image of DSR Input flag
  *            @arg USART_FLAG_RI:       	Image of RI Input flag
  *            @arg USART_FLAG_CTSIC:    	Clear to Send Input Change Flag
  *            @arg USART_FLAG_DCDIC:    	Data Carrier Detect Input Change Flag
  *            @arg USART_FLAG_DSRIC:    	Data Set Ready Input Change Flag
  *            @arg USART_FLAG_RIIC:     	Ring Indicator Input Change Flag
  *            @arg USART_FLAG_NACK:     	Non Acknowledge Interrupt flag
  *            @arg USART_FLAG_RXBUFF:   	Reception Buffer Full flag
  *            @arg USART_FLAG_TXBUFE:   	Transmission Buffer Empty flag
  *            @arg USART_FLAG_ITER:     	Max Number of Repetitions Reached flag
  *            @arg USART_FLAG_TXEMPTY:  	Transmitter Empty flag
  *            @arg USART_FLAG_TIMEOUT:  	Receiver Time-out flag
  *            @arg USART_FLAG_PARE:     	Parity error flag
  *            @arg USART_FLAG_FRAME:    	Framing error flag
  *            @arg USART_FLAG_OVRE:     	Overrun error flag
  *            @arg USART_FLAG_ENDTX:    	End of Transmitter Transfer flag
  *            @arg USART_FLAG_ENDRX:    	End of Receiver Transfer flag
  *            @arg USART_FLAG_RXBRK:    	Break Received/End of Break flag
  *            @arg USART_FLAG_TXRDY:    	Transmitter Ready flag
  *            @arg USART_FLAG_RXRDY:    	Receiver Ready flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_USART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->CSR & (__FLAG__)) == (__FLAG__))    

/** @brief  Enable the specified USART interrupt.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral
  * @param  __INTERRUPT__ specifies the USART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_MANE:   	Manchester Error Interrupt
  *            @arg USART_IT_CTSIC:    	Clear to Send Input Change Interrupt
  *            @arg USART_IT_DCDIC:    	Data Carrier Detect Input Change Interrupt
  *            @arg USART_IT_DSRIC:    	Data Set Ready Input Change Interrupt
  *            @arg USART_IT_RIIC:     	Ring Indicator Input Change Interrupt
  *            @arg USART_IT_NACK:     	Non Acknowledge Interrupt Interrupt
  *            @arg USART_IT_RXBUFF:   	Reception Buffer Full Interrupt
  *            @arg USART_IT_TXBUFE:   	Transmission Buffer Empty Interrupt
  *            @arg USART_IT_ITER:     	Max Number of Repetitions Reached Interrupt
  *            @arg USART_IT_TXEMPTY:  	Transmitter Empty Interrupt
  *            @arg USART_IT_TIMEOUT:  	Receiver Time-out Interrupt
  *            @arg USART_IT_PARE:     	Parity error Interrupt
  *            @arg USART_IT_FRAME:    	Framing error Interrupt
  *            @arg USART_IT_OVRE:     	Overrun error Interrupt
  *            @arg USART_IT_ENDTX:    	End of Transmitter Transfer Interrupt
  *            @arg USART_IT_ENDRX:    	End of Receiver Transfer Interrupt
  *            @arg USART_IT_RXBRK:    	Break Received/End of Break Interrupt
  *            @arg USART_IT_TXRDY:    	Transmitter Ready Interrupt
  *            @arg USART_IT_RXRDY:    	Receiver Ready Interrupt
  * @retval None
  */
#define __HAL_USART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->IER |= (__INTERRUPT__))

/** @brief  Disable the specified USART interrupt.
  * @param  __HANDLE__ specifies the USART Handle.
  *         USART Handle selects the USARTx peripheral
  * @param  __INTERRUPT__ specifies the USART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_MANE:   	Manchester Error Interrupt
  *            @arg USART_IT_CTSIC:    	Clear to Send Input Change Interrupt
  *            @arg USART_IT_DCDIC:    	Data Carrier Detect Input Change Interrupt
  *            @arg USART_IT_DSRIC:    	Data Set Ready Input Change Interrupt
  *            @arg USART_IT_RIIC:     	Ring Indicator Input Change Interrupt
  *            @arg USART_IT_NACK:     	Non Acknowledge Interrupt Interrupt
  *            @arg USART_IT_RXBUFF:   	Reception Buffer Full Interrupt
  *            @arg USART_IT_TXBUFE:   	Transmission Buffer Empty Interrupt
  *            @arg USART_IT_ITER:     	Max Number of Repetitions Reached Interrupt
  *            @arg USART_IT_TXEMPTY:  	Transmitter Empty Interrupt
  *            @arg USART_IT_TIMEOUT:  	Receiver Time-out Interrupt
  *            @arg USART_IT_PARE:     	Parity error Interrupt
  *            @arg USART_IT_FRAME:    	Framing error Interrupt
  *            @arg USART_IT_OVRE:     	Overrun error Interrupt
  *            @arg USART_IT_ENDTX:    	End of Transmitter Transfer Interrupt
  *            @arg USART_IT_ENDRX:    	End of Receiver Transfer Interrupt
  *            @arg USART_IT_RXBRK:    	Break Received/End of Break Interrupt
  *            @arg USART_IT_TXRDY:    	Transmitter Ready Interrupt
  *            @arg USART_IT_RXRDY:    	Receiver Ready Interrupt
  * @retval None
  */
#define __HAL_USART_DISABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->IDR |= (__INTERRUPT__))

/** @brief  Unlock WPMR register
  * @retval none
  */
#define __HAL_USART_UNLOCK_REGISTER(__HANDLE__)			(WRITE_REG((__HANDLE__)->Instance->WPMR, 0x55534100))

/** @brief  Set whether slave chip selection is enabled.
  * @param  __HANDLE__ specifies the USART Handle. 
  * @retval None
  */
#define __HAL_USART_SPI_NSS_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_SPI_FCS)

/** @brief  Set whether slave chip selection is disabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define __HAL_USART_SPI_NSS_DISABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_SPI_RCS)

/** @brief  Set the USART SPI send data is enabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_SPI_TX_ENABLE(__HANDLE__)  WRITE_REG((__HANDLE__)->Instance->CR, USART_CR_SPI_TXEN); 

/** @brief  Set the USART SPI receive data is enabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_SPI_RX_ENABLE(__HANDLE__)  WRITE_REG((__HANDLE__)->Instance->CR, USART_CR_SPI_RXEN); 

/** @brief  Set the USART SPI send data is disabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_SPI_TX_DISABLE(__HANDLE__)   SET_BIT((__HANDLE__)->Instance->CR, USART_CR_SPI_TXDIS); 

/** @brief  Set the USART SPI receive data is disabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_SPI_RX_DISABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_SPI_RXDIS);

/** @brief  Set the USART send data is enabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_TX_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_TXEN); 

/** @brief  Set the USART receive data is enabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_RX_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_RXEN); 

/** @brief  Set the USART send data is disabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_TX_DISABLE(__HANDLE__)   SET_BIT((__HANDLE__)->Instance->CR, USART_CR_TXDIS); 

/** @brief  Set the USART receive data is disabled.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define USART_RX_DISABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR, USART_CR_RXDIS);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USART_Exported_Functions
  * @{
  */ 

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup USART_Private_Macros 
  * @{
  */
/** @brief  Set the USART LIN DLC length.
  * @param  __LENGTH__ DLC length.
  * @retval None
  */
#define USART_LIN_DLC_LENGTH(__LENGTH__)  (((uint32_t)(__LENGTH__) - 1U) << 8U)
/**
  * @}
  */
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup USART_Private_Functions USART Private Functions
  * @{
  */  
  
/** @addtogroup USART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_SPI_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_LIN_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_USART_RegisterCallback(USART_HandleTypeDef *husart, HAL_USART_CallbackIDTypeDef CallbackID,
                                            pUSART_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_USART_UnRegisterCallback(USART_HandleTypeDef *husart, HAL_USART_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */
  
/* IO operation functions *******************************************************/
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_SPI_Transmit(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_SPI_Receive(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_SPI_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_LIN_Publish(USART_HandleTypeDef *husart, uint8_t *ID, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_LIN_Subscribe(USART_HandleTypeDef *husart, uint8_t *ID, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Transfer Abort functions */
HAL_StatusTypeDef HAL_USART_Abort(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_AbortTransmit(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_AbortReceive(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);

/** @addtogroup USART_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  **************************************************/
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               HAL_USART_GetError(USART_HandleTypeDef *husart);
/**
  * @}
  */

/** @defgroup USART_Private_Functions USART Private Functions
  * @{
  */  
HAL_StatusTypeDef USART_Start_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef USART_Start_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size);

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
