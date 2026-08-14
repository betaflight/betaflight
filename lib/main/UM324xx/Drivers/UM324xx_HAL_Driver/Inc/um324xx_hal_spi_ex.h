/**
  ******************************************************************************
  * @file    UM324XX_hal_spi_ex.h
  * @author  MCU Team
  * @version V1.00 
  * @date    10-February-2023  
  * @brief   SPI_EX HAL module driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __UM324XX_HAL_SPI_EX_H__
#define __UM324XX_HAL_SPI_EX_H__

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup SPI_EX
  * @{
  */
  
/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI2 Exported Types
  * @{
  */

/**
  * @brief  SPI2 Configuration Structure definition
  */
typedef struct
{
  uint32_t Mode;                  /*!< Specifies the SPI Master-slave mode.
                                         This parameter can be a value of @ref SPI_Mode */

    uint32_t Direction_Mode;        /*!< Specifies the SPI operating mode.
                                         This parameter can be a value of @ref SPI_Direction_Mode */
    
    uint32_t DataSize;              /*!< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_Data_Size */

    
    uint32_t FirstBit;              /*!< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_MSB_LSB_transmission */

    uint32_t TIMode;                /*!< Specifies if the TI mode is enabled or not.
                                         This parameter can be a value of @ref SPI_TIMode */
    
    uint32_t BaudRatePrescaler;     /*!< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of customizes by yourself
                                         @note The communication clock is derived from the master
                                         clock. The slave clock does not need to be set. */
    
    uint32_t CLKPolarity;           /*!< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_Clock_Polarity */

    uint32_t CLKPhase;              /*!< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_Clock_Phase */

    uint32_t TXEDGE;                /*!< Specifies the data transmission phase in slave mode.
                                         This parameter can be a value of @ref SPI_Tx_Edge */
    
    uint32_t RXEDGE;                /*!< Specifies the data transmission phase in master mode.
                                         This parameter can be a value of @ref SPI_Rx_Edge */
     
    uint32_t TX_Stitch;             /*!< Specifies the send data splicing.
                                         This parameter can be a value of @ref SPI_Tx_Stitch */

    uint32_t RX_Stitch;             /*!< Specifies the receive data splicing.
                                         This parameter can be a value of @ref SPI_Rx_Stitch */

    uint32_t SPI_Len;               /*!< Specifies the length of SPI character.
                                         This parameter can be a value of @ref SPI_SPILen */

    uint32_t NSS;                   /*!< Specifies the chip selection signal controller.
                                         This parameter can be a value of @ref SPI_NSS */

    uint32_t DMA_Mode;              /*!< Specifies the DMA access mode.
                                         This parameter can be a value of @ref SPI_DMAMode */

    uint32_t TXTLF;                 /*!< Specifies the number of vacancies that can trigger the TXFIFO.
                                         This parameter can be a value of @ref SPI_TXTLF */

    uint32_t RXTLF;                 /*!< Specifies the number of vacancies that can trigger the RXFIFO.
                                         This parameter can be a value of @ref SPI_RXTLF */
} SPI_EX_InitTypeDef;

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
    HAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
    HAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
    HAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
    HAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
    HAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
    HAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
    HAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} HAL_SPI_EX_StateTypeDef;

/** @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief SPI MODE enumeration
  * @{
  */

typedef enum
{
  HAL_SPI_MODE_NONE           = 0x00U,   /*!< No SPI01 communication on going             */
  HAL_SPI_MODE_MASTER         = 0x10U,   /*!< SPI01 communication is in Master Mode       */
  HAL_SPI_MODE_SLAVE          = 0x20U,   /*!< SPI01 communication is in Slave Mode        */
} HAL_SPI_EX_ModeTypeDef;


typedef struct __SPI_EX_HandleTypeDef
{
    SPI_EX_TypeDef             *Instance;               /*!< SPI registers base address               */
            
    SPI_EX_InitTypeDef         Init;                    /*!< SPI communication parameters             */
            
    uint8_t                    *pTxBuffPtr;             /*!< Pointer to SPI Tx transfer Buffer        */
            
    uint16_t                   TxXferSize;              /*!< SPI Tx Transfer size                     */
            
    __IO uint16_t              TxXferCount;             /*!< SPI Tx Transfer Counter                  */
            
    uint8_t                    *pRxBuffPtr;             /*!< Pointer to SPI Rx transfer Buffer        */
            
    uint16_t                   RxXferSize;              /*!< SPI Rx Transfer size                     */
            
    __IO uint16_t              RxXferCount;             /*!< SPI Rx Transfer Counter                  */

    void (*RxISR)(struct __SPI_EX_HandleTypeDef *hspi);    /*!< function pointer on Rx ISR               */
                                                                                                      
    void (*TxISR)(struct __SPI_EX_HandleTypeDef *hspi);    /*!< function pointer on Tx ISR               */

    DMA_HandleTypeDef          *hdmatx;                 /*!< SPI Tx DMA Handle parameters             */
            
		DMA_HandleTypeDef          *hdmarx;                 /*!< SPI Rx DMA Handle parameters             */
                
    HAL_LockTypeDef            Lock;                    /*!< Locking object                           */
            
    __IO HAL_SPI_EX_StateTypeDef  State;                   /*!< SPI communication state                  */
            
    __IO uint32_t              ErrorCode;               /*!< SPI Error code                           */

#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
  void (* TxCpltCallback)(struct __SPI_EX_HandleTypeDef *hspi);             /*!< SPI Tx Completed callback          */
  void (* RxCpltCallback)(struct __SPI_EX_HandleTypeDef *hspi);             /*!< SPI Rx Completed callback          */
  void (* TxRxCpltCallback)(struct __SPI_EX_HandleTypeDef *hspi);           /*!< SPI TxRx Completed callback        */
  void (* ErrorCallback)(struct __SPI_EX_HandleTypeDef *hspi);              /*!< SPI Error callback                 */
  void (* AbortCpltCallback)(struct __SPI_EX_HandleTypeDef *hspi);          /*!< SPI Abort callback                 */
  void (* MspInitCallback)(struct __SPI_EX_HandleTypeDef *hspi);            /*!< SPI Msp Init callback              */
  void (* MspDeInitCallback)(struct __SPI_EX_HandleTypeDef *hspi);          /*!< SPI Msp DeInit callback            */
#endif  /* USE_HAL_SPI_REGISTER_CALLBACKS */    
} SPI_EX_HandleTypeDef;
  
  
/**
  * @brief  HAL SPI CSS definition
  */
typedef enum 
{
    HAL_SPI_CS_LOW = 0,                               // CS is low
    HAL_SPI_CS_HIGH,                                  // CS is high
}SPI_EX_CssTypeDef;


#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
/**
  * @brief  HAL SPI Callback ID enumeration definition
  */
typedef enum
{
    HAL_SPI_EX_TX_COMPLETE_CB_ID             = 0x00U,    /*!< SPI Tx Completed callback ID         */
    HAL_SPI_EX_RX_COMPLETE_CB_ID             = 0x01U,    /*!< SPI Rx Completed callback ID         */
    HAL_SPI_EX_TX_RX_COMPLETE_CB_ID          = 0x02U,    /*!< SPI TxRx Completed callback ID       */
    HAL_SPI_EX_ERROR_CB_ID                   = 0x03U,    /*!< SPI Error callback ID                */
    HAL_SPI_EX_ABORT_CB_ID                   = 0x04U,    /*!< SPI Abort callback ID                */
    HAL_SPI_EX_MSPINIT_CB_ID                 = 0x05U,    /*!< SPI Msp Init callback ID             */
    HAL_SPI_EX_MSPDEINIT_CB_ID               = 0x06U     /*!< SPI Msp DeInit callback ID           */

} HAL_SPI_EX_CallbackIDTypeDef;


/**
  * @brief  HAL SPI Callback pointer definition
  */
typedef  void (*pSPI_CallbackTypeDef)(SPI_EX_HandleTypeDef *hspi); /*!< pointer to an SPI callback function */

#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
//  
///**
//  * @}
//  */

#define SPI_EX_SPBRG_SPBRG_DEFAULT           (0x2UL<<SPI_EX_SPBRG_SPBRG_Pos)                                /*!< 0x00000002 */
///* Exported constants --------------------------------------------------------*/

/** @defgroup SPI_Exported_constants SPI Exported Constants
  * @{
  */ 

/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_EX_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define HAL_SPI_EX_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define HAL_SPI_EX_ERROR_FLAG              (0x00000020U)   /*!< Error on RXOERR/UNDERRUN Flag          */
#define HAL_SPI_EX_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
#define HAL_SPI_EX_ERROR_INVALID_CALLBACK  (0x00000080U)   /*!< Invalid Callback error                 */
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */



/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_EX_MODE_SLAVE                  (0x00000000U)
#define SPI_EX_MODE_MASTER                 (SPI_EX_GCTL_MM)

/**
  * @}
  */
  
 /** @defgroup SPI_Direction_Mode SPI Direction Mode
  * @{
  */
#define SPI_EX_FULL_DUPLEX                 (0x00000018U)
#define SPI_EX_HALF_DUPLEX                 (0x00000818U)
#define SPI_EX_SINGLE_DUPLEX_RECEIVE       (0x10000010U)
#define SPI_EX_SINGLE_DUPLEX_SEND          (0x10000008U)

/**
  * @}
  */
  
  
/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_EX_DATASIZE_8BIT               (0x00000000U)
#define SPI_EX_DATASIZE_16BIT              (0x00000001U)
#define SPI_EX_DATASIZE_32BIT              (0x00000002U)

/**
  * @}
  */ 
  
/** @defgroup SPI_TIMode SPI Time series mode
  * @{
  */
#define SPI_EX_TIMODE_MOTOROLA             (0x00000000U)
#define SPI_EX_TIMODE_TI                   (SPI_EX_CCTL_TI_MOD)

/**
  * @}
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_EX_POLARITY_LOW                (0x00000000U)
#define SPI_EX_POLARITY_HIGH               SPI_EX_CCTL_CKPL

/**
  * @}
  */
  
  /** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_EX_PHASE_1EDGE                 (0x00000000U)
#define SPI_EX_PHASE_2EDGE                 SPI_EX_CCTL_CKPH

/**
  * @}
  */  
  
 /** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
  * @{
  */
#define SPI_EX_FIRSTBIT_MSB                (0x00000000U)
#define SPI_EX_FIRSTBIT_LSB                SPI_EX_CCTL_LSBFE

/**
  * @}
  */ 

/** @defgroup SPI_Tx_Edge SPI Phase adjustment of data transmission in slave mode
  * @{
  */
#define SPI_EX_TXEDGE_SCLK                (0x00000000U)
#define SPI_EX_TXEDGE_PCLK                SPI_EX_CCTL_TXEDGE

/**
  * @}
  */ 

/** @defgroup SPI_Rx_Edge SPI Phase adjustment of data reception in master mode
  * @{
  */
#define SPI_EX_RXEDGE_SCLK                (0x00000000U)
#define SPI_EX_RXEDGE_NEXTSCLK            SPI_EX_CCTL_RXEDGE

/**
  * @}
  */   
/** @defgroup SPI_Rx_Stitch  the switch for the receive data splicing function of SPI 
  * @{
  */
#define SPI_EX_RX_STITCH_CLOSE            (0x00000000U)
#define SPI_EX_RX_STITCH_OPEN             SPI_EX_CCTL_RX_STITCH

/**
  * @}
  */   
  
/** @defgroup SPI_Tx_Stitch  the switch for the send data splicing function of SPI 
  * @{
  */
#define SPI_EX_TX_STITCH_CLOSE            (0x00000000U)
#define SPI_EX_TX_STITCH_OPEN             SPI_EX_CCTL_TX_STITCH

/**
  * @}
  */    
  
/** @defgroup SPI_DMAMode   DMA access mode of SPI 
  * @{
  */
#define SPI_EX_DMAMODE_CLOSE              (0x00000000U)
#define SPI_EX_DMAMODE_OPEN               SPI_EX_GCTL_DMAMODE

/**
  * @}
  */   
 
/** @defgroup SPI_TXTLF   The number of vacancies that can trigger the TXFIFO 
  * @{
  */
#define SPI_EX_TXTLF_1                   (0x00000000U)
#define SPI_EX_TXTLF_4                   SPI_EX_GCTL_TXTLF_0
#define SPI_EX_TXTLF_8                   SPI_EX_GCTL_TXTLF_1

/**
  * @}
  */ 
/** @defgroup SPI_RXTLF   The number of vacancies that can trigger the RXFIFO 
  * @{
  */
#define SPI_EX_RXTLF_1                   (0x00000000U)
#define SPI_EX_RXTLF_4                   SPI_EX_GCTL_RXTLF_0
#define SPI_EX_RXTLF_8                   SPI_EX_GCTL_RXTLF_1

/**
  * @}
  */

/** @defgroup SPI_SPILen  the length of SPI character control 
  * @{
  */
#define SPI_EX_SPILEN_BIT_1               (0x00000000U)
#define SPI_EX_SPILEN_BIT_2               (SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_3               (SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_4               (SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_5               (SPI_EX_CCTL_SPILEN_2)
#define SPI_EX_SPILEN_BIT_6               (SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_7               (SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_8               (SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_9               (SPI_EX_CCTL_SPILEN_3)
#define SPI_EX_SPILEN_BIT_10              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_11              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_12              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_13              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2)
#define SPI_EX_SPILEN_BIT_14              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_15              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_16              (SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_17              (SPI_EX_CCTL_SPILEN_4)
#define SPI_EX_SPILEN_BIT_18              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_19              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_20              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_21              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_2)
#define SPI_EX_SPILEN_BIT_22              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_23              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_24              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_25              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3)
#define SPI_EX_SPILEN_BIT_26              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_27              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_28              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_29              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2)
#define SPI_EX_SPILEN_BIT_30              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_0)
#define SPI_EX_SPILEN_BIT_31              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1)
#define SPI_EX_SPILEN_BIT_32              (SPI_EX_CCTL_SPILEN_4 | SPI_EX_CCTL_SPILEN_3 | SPI_EX_CCTL_SPILEN_2 | SPI_EX_CCTL_SPILEN_1 | SPI_EX_CCTL_SPILEN_0)

/**
  * @}
  */  
  
  /** @defgroup SPI_NSS  the function for NSS signal controller. 
  * @{
  */
#define SPI_EX_NSS_SOFT                (0x00000000U)
#define SPI_EX_NSS_HARDWARE            SPI_EX_GCTL_CSN_SEL

/**
  * @}
  */ 

/** @defgroup SPI_INTEN  the spi interrupt master switch. 
  * @{
  */
#define SPI_EX_INTEN_CLOSE                (0x00000000U)
#define SPI_EX_INTEN_OPEN                 SPI_EX_GCTL_INT_EN

/**
  * @}
  */ 

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
#define SPI_EX_IT_TX                       SPI_EX_INTEN_TXIEN                     /* Transmitter FIFO vacancy available interrupt         */
#define SPI_EX_IT_RX                       SPI_EX_INTEN_RXIEN                     /* Receiver FIFO vacancy available interrupt            */
#define SPI_EX_IT_UNDERRUNN                SPI_EX_INTEN_UNDERRUNEN                /* Slave transmitter underload interrupt                */
#define SPI_EX_IT_RXOERR                   SPI_EX_INTEN_RXOERREN                  /* Receiver overflow error interrupt                    */
#define SPI_EX_IT_RXMATCH                  SPI_EX_INTEN_RXMATCHEN                 /* Receive completion interrupt                         */
#define SPI_EX_IT_RXFIFO_FULL              SPI_EX_INTEN_RXFIFO_FULL_IEN           /* Receiver FIFO full interrupt                         */
#define SPI_EX_IT_TXEPT                    SPI_EX_INTEN_TXEPT_IEN                 /* Transmitter variable air interruption                */
#define SPI_EX_IT_TXMATCH                  SPI_EX_INTEN_TXMATCHEN                 /* Send complete interrupt                              */
#define SPI_EX_IT_MASK                    (SPI_EX_INTEN_TXIEN | SPI_EX_INTEN_RXIEN | SPI_EX_INTEN_UNDERRUNEN | SPI_EX_INTEN_RXOERREN\
                                          | SPI_EX_INTEN_RXMATCHEN | SPI_EX_INTEN_RXFIFO_FULL_IEN | SPI_EX_INTEN_TXEPT_IEN | SPI_EX_INTEN_TXMATCHEN)

/**
  * @}
  */  

/** @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */
#define SPI_EX_FLAG_TX                     SPI_EX_INTSTAT_TX_INTF                   /* Transmitter FIFO vacancy available interrupt flag    */
#define SPI_EX_FLAG_RX                     SPI_EX_INTSTAT_RX_INTF                   /* Receiver FIFO data available interrupt flag          */
#define SPI_EX_FLAG_UNDERRUN               SPI_EX_INTSTAT_UNDERRUN_INTF             /* Slave sender under-load interrupt flag               */
#define SPI_EX_FLAG_RXOERR                 SPI_EX_INTSTAT_RXOERR_INTF               /* Receiver overflow error interrupt flag               */
#define SPI_EX_FLAG_RXMATCH                SPI_EX_INTSTAT_RXMATCH_INTF              /* Receive complete interrupt flag                      */
#define SPI_EX_FLAG_RXFIFO_FULL            SPI_EX_INTSTAT_RXFIFO_FULL_INTF          /* RX FIFO is full interrupt flag                       */
#define SPI_EX_FLAG_TXEPT                  SPI_EX_INTSTAT_TXEPT_INTF                /* Transmitter to empty interrupt flag                  */
#define SPI_EX_FLAG_TXMATCH                SPI_EX_INTSTAT_TXMATCH_INTF              /* Send complete interrupt flag                         */
#define SPI_EX_FLAG_MASK                   (SPI_EX_INTSTAT_TX_INTF | SPI_EX_INTSTAT_RX_INTF | SPI_EX_INTSTAT_UNDERRUN_INTF | SPI_EX_INTSTAT_RXOERR_INTF\
                                         | SPI_EX_INTSTAT_RXMATCH_INTF | SPI_EX_INTSTAT_RXFIFO_FULL_INTF | SPI_EX_INTSTAT_TXEPT_INTF | SPI_EX_INTSTAT_TXMATCH_INTF)
/**
  * @}
  */


/** @defgroup SPI_MINTFlags_definition SPI Flags Definition In SPI_MINTSTAT Register
  * @{
  */
#define SPI_EX_MINTFLAG_TX                     SPI_EX_MINTSTAT_TX_MINTF             /* Transmitter FIFO vacancy available interrupt flag    */
#define SPI_EX_MINTFLAG_RX                     SPI_EX_MINTSTAT_RX_MINTF             /* Receiver FIFO data available interrupt flag          */
#define SPI_EX_MINTFLAG_UNDERRUN               SPI_EX_MINTSTAT_UNDERRUN_MINTF       /* Slave sender under-load interrupt flag               */
#define SPI_EX_MINTFLAG_RXOERR                 SPI_EX_MINTSTAT_RXOERR_MINTF         /* Receiver overflow error interrupt flag               */
#define SPI_EX_MINTFLAG_RXMATCH                SPI_EX_MINTSTAT_RXMATCH_MINTF        /* Receive complete interrupt flag                      */
#define SPI_EX_MINTFLAG_RXFIFO_FULL            SPI_EX_MINTSTAT_RXFIFO_FULL_MINTF    /* RX FIFO is full interrupt flag                       */
#define SPI_EX_MINTFLAG_TXEPT                  SPI_EX_MINTSTAT_TXEPT_MINTF          /* Transmitter to empty interrupt flag                  */
#define SPI_EX_MINTFLAG_TXMATCH                SPI_EX_MINTSTAT_TXMATCH_MINTF        /* Send complete interrupt flag                         */
#define SPI_EX_MINTFLAG_MASK                   (SPI_EX_MINTSTAT_TX_MINTF | SPI_EX_MINTSTAT_RX_MINTF | SPI_EX_MINTSTAT_UNDERRUN_MINTF | SPI_EX_MINTSTAT_RXOERR_MINTF\
                                         | SPI_EX_MINTSTAT_RXMATCH_MINTF | SPI_EX_MINTSTAT_RXFIFO_FULL_MINTF | SPI_EX_MINTSTAT_TXEPT_MINTF | SPI_EX_MINTSTAT_TXMATCH_MINTF)
/**
  * @}
  */  

/** @defgroup SPI_MINTFlags_definition SPI Flags Definition In SPI_MINTSTAT Register
  * @{
  */
#define SPI_EX_STATE_TXEPT                    SPI_EX_CSTAT_TXEPT               /* TX FIFO empty status flag                        */
#define SPI_EX_STATE_RXAVL                    SPI_EX_CSTAT_RXAVL               /* Received available data flag                     */
#define SPI_EX_STATE_TXFULL                   SPI_EX_CSTAT_TXFULL              /* TX FIFO is full flag                             */
#define SPI_EX_STATE_RXAVL4BYTE               SPI_EX_CSTAT_RXAVL_4BYTE         /* RX FIFO with 4 words of available data flag      */
#define SPI_EX_STATE_CSSTATUS                 SPI_EX_CSTAT_CSSTATUS            /* CS status when used as slave, only for UM32x42x  */
/**
  * @}
  */ 



/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_Exported_macro SPI Exported Macro
  * @{
  */ 


/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_IT_TX             Transmitter FIFO vacancy available interrupt  
  *            @arg SPI_EX_IT_RX             Receiver FIFO vacancy available interrupt     
  *            @arg SPI_EX_IT_UNDERRUNN      Slave transmitter underload interrupt         
  *            @arg SPI_EX_IT_RXOERR         Receiver overflow error interrupt      
  *            @arg SPI_EX_IT_RXMATCH        Receive completion interrupt           
  *            @arg SPI_EX_IT_RXFIFO_FULL    Receiver FIFO full interrupt           
  *            @arg SPI_EX_IT_TXEPT          Transmitter variable air interruption  
  *            @arg SPI_EX_IT_TXMATCH        Send complete interrupt                
  * @retval None
  */
#define __HAL_SPI_EX_ENABLE_IT(__HANDLE__, __INTERRUPT__)   do{ \
                                                           SET_BIT(((__HANDLE__)->Instance->GCTL),SPI_EX_GCTL_INT_EN); \
                                                           SET_BIT(((__HANDLE__)->Instance->INTEN),__INTERRUPT__); \
                                                           }while(0U)

/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI handle.
  *         This parameter can be SPIx where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_IT_TX             Transmitter FIFO vacancy available interrupt  
  *            @arg SPI_EX_IT_RX             Receiver FIFO vacancy available interrupt     
  *            @arg SPI_EX_IT_UNDERRUNN      Slave transmitter underload interrupt         
  *            @arg SPI_EX_IT_RXOERR         Receiver overflow error interrupt      
  *            @arg SPI_EX_IT_RXMATCH        Receive completion interrupt           
  *            @arg SPI_EX_IT_RXFIFO_FULL    Receiver FIFO full interrupt           
  *            @arg SPI_EX_IT_TXEPT          Transmitter variable air interruption  
  *            @arg SPI_EX_IT_TXMATCH        Send complete interrupt                
  * @retval None
  */
#define __HAL_SPI_EX_DISABLE_IT(__HANDLE__, __INTERRUPT__)    CLEAR_BIT(((__HANDLE__)->Instance->INTEN), __INTERRUPT__);

/** @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPI_EX_IT_TX             Transmitter FIFO vacancy available interrupt  
  *            @arg SPI_EX_IT_RX             Receiver FIFO vacancy available interrupt     
  *            @arg SPI_EX_IT_UNDERRUNN      Slave transmitter underload interrupt         
  *            @arg SPI_EX_IT_RXOERR         Receiver overflow error interrupt      
  *            @arg SPI_EX_IT_RXMATCH        Receive completion interrupt           
  *            @arg SPI_EX_IT_RXFIFO_FULL    Receiver FIFO full interrupt           
  *            @arg SPI_EX_IT_TXEPT          Transmitter variable air interruption  
  *            @arg SPI_EX_IT_TXMATCH        Send complete interrupt    
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_SPI_EX_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->INTEN \
                                                              & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified SPI flag is set or not.
  * @note   If an interrupt is not enabled, the corresponding interrupt flag in this register will not be set.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_FLAG_TX              Transmitter FIFO vacancy available interrupt flag  
  *            @arg SPI_EX_FLAG_RX              Receiver FIFO data available interrupt flag        
  *            @arg SPI_EX_FLAG_UNDERRUN        Slave sender under-load interrupt flag             
  *            @arg SPI_EX_FLAG_RXOERR          Receiver overflow error interrupt flag             
  *            @arg SPI_EX_FLAG_RXMATCH         Receive complete interrupt flag                    
  *            @arg SPI_EX_FLAG_RXFIFO_FULL     RX FIFO is full interrupt flag                     
  *            @arg SPI_EX_FLAG_TXEPT           Transmitter to empty interrupt flag                
  *            @arg SPI_EX_FLAG_TXMATCH         Send complete interrupt flag                       
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_EX_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->INTSTAT) & (__FLAG__)) == (__FLAG__))





/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__  specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.  
  * @param  __FLAG__    specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_STATE_TXEPT          TX FIFO empty status flag  
  *            @arg SPI_EX_STATE_RXAVL          Received available data flag  
  *            @arg SPI_EX_STATE_TXFULL         TX FIFO is full flag  
  *            @arg SPI_EX_STATE_RXAVL4BYTE     RX FIFO with 4 words of available data flag  
  *            @arg SPI_EX_STATE_CSSTATUS       CS status when used as slave, only for UM32x42x
  * @retval SET or RESET.
  */
#define __HAL_SPI_EX_CURRENT_STATE(__HANDLE__,__FLAG__)      ((((__HANDLE__)->Instance->CSTAT) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_FLAG_TX              Transmitter FIFO vacancy available interrupt flag  
  *            @arg SPI_EX_FLAG_RX              Receiver FIFO data available interrupt flag        
  *            @arg SPI_EX_FLAG_UNDERRUN        Slave sender under-load interrupt flag             
  *            @arg SPI_EX_FLAG_RXOERR          Receiver overflow error interrupt flag             
  *            @arg SPI_EX_FLAG_RXMATCH         Receive complete interrupt flag                    
  *            @arg SPI_EX_FLAG_RXFIFO_FULL     RX FIFO is full interrupt flag                     
  *            @arg SPI_EX_FLAG_TXEPT           Transmitter to empty interrupt flag                
  *            @arg SPI_EX_FLAG_TXMATCH         Send complete interrupt flag                       
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_EX_CLEAR_FLAG(__HANDLE__, __FLAG__)      SET_BIT((__HANDLE__)->Instance->INTCLR, (__FLAG__))

/** @brief  Enable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_EX_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_EN)

/** @brief  Disable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_EX_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_EN)

/** @brief  Set the number of received data register.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __VALUE__ This parameter can be set in 0x1-0xFF. 
  * @retval None
  */
#define __HAL_SPI_EX_SETRXDNR(__HANDLE__,__VALUE__)    WRITE_REG((__HANDLE__)->Instance->RXDNR,__VALUE__)

/** @brief  Set the number of send data register.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __VALUE__ This parameter can be set in 0x1-0xFF. 
  * @retval None
  */
#define __HAL_SPI_EX_SETTXDNR(__HANDLE__,__VALUE__)    WRITE_REG((__HANDLE__)->Instance->TXDNR,__VALUE__)

/** @brief  Set whether slave chip selection is enabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __STATE__ This parameter can be HAL_SPI_CS_LOW or HAL_SPI_CS_HIGH. 
  * @retval None
  */
#define __HAL_SPI_EX_NSS_ENABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->SCSR,SPI_EX_SCSR_CS0)

/** @brief  Set whether slave chip selection is enabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @param  __STATE__ This parameter can be HAL_SPI_CS_LOW or HAL_SPI_CS_HIGH. 
  * @retval None
  */
#define __HAL_SPI_EX_NSS_DISABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->SCSR,SPI_EX_SCSR_CS0)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */

/** @brief  Set the SPI transmit-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_1LINE_TX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_HALF_DIR)

/** @brief  Set the SPI receive-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_1LINE_RX(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_HALF_DIR)

/** @brief  Set the SPI send data is enabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_TX_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_TXEN); 

/** @brief  Set the SPI receive data is enabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_RX_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_RXEN); 
                                      
/** @brief  Set the SPI send data is disabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_TX_DISABLE(__HANDLE__)   CLEAR_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_TXEN); 

/** @brief  Set the SPI receive data is disabled.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 0, 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_EX_RX_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->GCTL, SPI_EX_GCTL_RXEN); 

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __INTSTAT__  copy of SPI INTSTAT register.
  * @param  __FLAG__     specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_FLAG_TX              Transmitter FIFO vacancy available interrupt flag  
  *            @arg SPI_EX_FLAG_RX              Receiver FIFO data available interrupt flag        
  *            @arg SPI_EX_FLAG_UNDERRUN        Slave sender under-load interrupt flag             
  *            @arg SPI_EX_FLAG_RXOERR          Receiver overflow error interrupt flag             
  *            @arg SPI_EX_FLAG_RXMATCH         Receive complete interrupt flag                    
  *            @arg SPI_EX_FLAG_RXFIFO_FULL     RX FIFO is full interrupt flag                     
  *            @arg SPI_EX_FLAG_TXEPT           Transmitter to empty interrupt flag                
  *            @arg SPI_EX_FLAG_TXMATCH         Send complete interrupt flag 
  * @retval SET or RESET.
  */
#define SPI_EX_CHECK_FLAG(__INTSTAT__, __FLAG__) ((((__INTSTAT__) & ((__FLAG__) & SPI_EX_FLAG_MASK)) == \
                                               ((__FLAG__) & SPI_EX_FLAG_MASK)) ? SET : RESET)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __MINTSTAT__  copy of SPI MINTSTAT register.
  * @param  __FLAG__     specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg  SPI_EX_MINTFLAG_TX              Transmitter FIFO vacancy available interrupt flag  
  *            @arg  SPI_EX_MINTFLAG_RX              Receiver FIFO data available interrupt flag        
  *            @arg  SPI_EX_MINTFLAG_UNDERRUN        Slave sender under-load interrupt flag             
  *            @arg  SPI_EX_MINTFLAG_RXOERR          Receiver overflow error interrupt flag             
  *            @arg  SPI_EX_MINTFLAG_RXMATCH         Receive complete interrupt flag                    
  *            @arg  SPI_EX_MINTFLAG_RXFIFO_FULL     RX FIFO is full interrupt flag                     
  *            @arg  SPI_EX_MINTFLAG_TXEPT           Transmitter to empty interrupt flag                
  *            @arg  SPI_EX_MINTFLAG_TXMATCH         Send complete interrupt flag      
  * @retval SET or RESET.
  */
#define SPI_EX_CHECK_MINTFLAG(__MINTSTAT__, __FLAG__) ((((__MINTSTAT__) & ((__FLAG__) & SPI_EX_MINTFLAG_MASK)) == \
                                                    ((__FLAG__) & SPI_EX_MINTFLAG_MASK)) ? SET : RESET)


/** @brief  Check whether the specified SPI Interrupt is set or not.
  * @param  __INTEN__  copy of SPI INTEN register.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_EX_IT_TX             Transmitter FIFO vacancy available interrupt  
  *            @arg SPI_EX_IT_RX             Receiver FIFO vacancy available interrupt     
  *            @arg SPI_EX_IT_UNDERRUNN      Slave transmitter underload interrupt         
  *            @arg SPI_EX_IT_RXOERR         Receiver overflow error interrupt      
  *            @arg SPI_EX_IT_RXMATCH        Receive completion interrupt           
  *            @arg SPI_EX_IT_RXFIFO_FULL    Receiver FIFO full interrupt           
  *            @arg SPI_EX_IT_TXEPT          Transmitter variable air interruption  
  *            @arg SPI_EX_IT_TXMATCH        Send complete interrupt    
  * @retval SET or RESET.
  */
#define SPI_EX_CHECK_IT_SOURCE(__INTEN__, __INTERRUPT__) ((((__INTEN__) & (__INTERRUPT__)) == \
                                                     (__INTERRUPT__)) ? SET : RESET)

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions
  * @{
  */ 

/** @addtogroup SPI_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_SPI_EX_Init(SPI_EX_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_EX_DeInit(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_MspInit(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_MspDeInit(SPI_EX_HandleTypeDef *hspi);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
HAL_StatusTypeDef HAL_SPI_EX_RegisterCallback(SPI_EX_HandleTypeDef *hspi, HAL_SPI_EX_CallbackIDTypeDef CallbackID,
                                           pSPI_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_SPI_EX_UnRegisterCallback(SPI_EX_HandleTypeDef *hspi, HAL_SPI_EX_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
/**
  * @}
  */
/** @addtogroup SPI_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
HAL_StatusTypeDef HAL_SPI_EX_Transmit(SPI_EX_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_EX_Receive(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_EX_Transmit_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_Receive_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_Transmit_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_Receive_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_EX_DMAPause(SPI_EX_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_EX_DMAResume(SPI_EX_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_EX_DMAStop(SPI_EX_HandleTypeDef *hspi);
/* Transfer Abort functions */
HAL_StatusTypeDef HAL_SPI_EX_Abort(SPI_EX_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_EX_Abort_IT(SPI_EX_HandleTypeDef *hspi);

void HAL_SPI_EX_IRQHandler(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_TxCpltCallback(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_RxCpltCallback(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_TxRxCpltCallback(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_TxRxHalfCpltCallback(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_ErrorCallback(SPI_EX_HandleTypeDef *hspi);
void HAL_SPI_EX_AbortCpltCallback(SPI_EX_HandleTypeDef *hspi);
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_SPI_EX_StateTypeDef HAL_SPI_EX_GetState(SPI_EX_HandleTypeDef *hspi);
uint32_t             HAL_SPI_EX_GetError(SPI_EX_HandleTypeDef *hspi);
/**
  * @}
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

/**
  * @}
  */  
  
#ifdef __cplusplus
}
#endif

#endif /* __UM32X42X_HAL_SPI_H__ */  
  
  







