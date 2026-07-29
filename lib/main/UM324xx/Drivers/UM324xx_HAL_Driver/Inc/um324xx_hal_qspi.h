 /**
  ******************************************************************************
  * @file     um324xF_hal_qspi.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-17  
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
#ifndef __UM324XF_HAL_QSPI_H__
#define __UM324XF_HAL_QSPI_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup QSPI
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup QSPI_Exported_typedefs QSPI Exported Typedefs
  * @{
  */ 

/**
  * @brief  QSPI Init structure definition
  */
typedef struct
{
    uint32_t ClockPrescaler;         /* Specifies the prescaler factor for generating clock based on the AHB clock.
                                        This parameter can be a value of @ref QSPI_ClockPrescaler */
 
    uint32_t ClockMode;              /* Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                        This parameter can be a value of @ref QSPI_ClockMode */
      
    uint32_t WorkMode;               /* Specifies the QSPI Work Mode. It indicates the communication mode of QSPI.
                                        This parameter can be a value of @ref QSPI_WorkMode */
 
    uint32_t WriteProtectPin;        /* Specifies the Write protection pin setting. 
                                        Note that the WP pin is only valid in SINGLE or DUAL transmission mode. 
                                        This parameter can be a value of @ref QSPI_WriteProtectPin */
     
    uint32_t AddressRemap;           /* Specifies the AHB address remapping enabled or not. 
                                        Note that the function is only valid in Direct access mode only. 
                                        This parameter can be a value of @ref QSPI_AddressRemap */

    uint32_t Enter_XIPIM;            /* Specifies the Enter XIP mode immediately.
                                        This parameter can be a value of @ref QSPI_Enter_XIPIM */
 
    uint32_t Enter_XIPNext;          /* Specifies the Enter XIP mode at the next read instruction.
                                        This parameter can be a value of @ref QSPI_Enter_XIPNext */

    uint32_t AHB_Decoder;            /* Specifies the AHB decoder enable or not.
                                        Note that the function is only valid in Direct access mode only. 
                                        This parameter can be a value of @ref QSPI_AHB_Decoder */

    uint32_t DTR_Protocol;           /* Specifies the DTR protocol enable or not.
                                        This parameter can be a value of @ref QSPI_DTR_Protocol */

    uint32_t AddrSizes;             /* Specifies the Number of device address bytes.
                                        This parameter can be a value of @ref QSPI_AddrSizes */

    uint32_t PageSizes;             /* Specifies the Number of device page bytes.
                                        This parameter can be a number between 0 and 4095 */

    uint32_t BlockSizes;            /* Specifies the Number of device block bytes.
                                       @note The number of bytes per block must be 2^n.
                                       This parameter can be a value of @ref QSPI_BlockSizes */

    uint32_t CSSizes;               /* Specifies Size of Flash device connected to CS pin.
                                       @note This bit is valid when AHB decoder is enabled.
                                        This parameter can be a value of @ref QSPI_CSSizes */

    uint32_t ReadDelay;             /* Specifies the Number of Read data capture delay.
                                        This parameter can be a value of @ref QSPI_ReadDelay */

    uint32_t TransDelay;            /* Specifies the Transmission data delay.
                                        This parameter can be a value of @ref QSPI_TransDelay */

    uint32_t Sampling_Edge;         /* Specifies the Sampling edge selection.
                                        This parameter can be a value of @ref QSPI_Sampling_Edge */

    uint32_t CSStartDelay;          /* Specifies the delay time of CS transmission start.
                                        This parameter can be a number between 0 and 0xFF */

    uint32_t CSStopDelay;           /* Specifies the delay time of CS transmission stop.
                                        This parameter can be a number between 0 and 0xFF */

    uint32_t CSInvalidDelay;        /* Specifies the delay time of CS became invalid.
                                        This parameter can be a number between 0 and 0xFF */

}QSPI_InitTypeDef;


/**
  * @brief  QSPI Direct access structure definition
  */
typedef struct
{
    uint32_t ReadCommand;               /* Specifies the read command of QSPI in DAC mode .
                                            This parameter can be a value by customing */
    
    uint32_t ReadAddr_Type;             /* Specifies the address of QSPI in DAC read mode .
                                            This parameter can be a value of @ref QSPI_AddrType */
 
    uint32_t ReadData_Type;             /* Specifies the data's way of QSPI in DAC read mode .
                                            This parameter can be a value of @ref QSPI_DataType */ 
 
    uint32_t ReadData_Dummy;            /* Specifies the read command dummy of QSPI in DAC read mode .
                                            This parameter can be a value of @ref HAL_QSPI_DUMMYTypeDef */ 
 
    uint32_t WriteCommand;              /* Specifies the read command of QSPI in DAC mode .
                                            This parameter can be a value by customing */
    
    uint32_t WriteAddr_Type;            /* Specifies the address of QSPI in DAC read mode .
                                            This parameter can be a value of @ref QSPI_AddrType */
 
    uint32_t WriteData_Type;            /* Specifies the data's way of QSPI in DAC read mode .
                                            This parameter can be a value of @ref QSPI_DataType */ 
 
    uint32_t WriteData_Dummy;           /* Specifies the read command dummy of QSPI in DAC read mode .
                                            This parameter can be a value of @ref HAL_QSPI_DUMMYTypeDef */ 
  
}QSPI_DACModeTypeDef;


/**
  * @brief  QSPI Indirect access structure definition
  */
typedef struct
{
    uint32_t WriteAddrStart;            /* Specifies the address for write of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t WriteNum;                  /* Specifies the bytes for write of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
    
    uint32_t ReadAddrStart;             /* Specifies the address for read of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t ReadNum;                   /* Specifies the bytes for read of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t WriteTrigAddress;          /* Specifies the Indirect trigger address  of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
 
    uint32_t WriteTrigAddressRange;     /* Specifies the range of Indirect trigger address for QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t ReadTrigAddress;           /* Specifies the Indirect trigger address  of QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
 
    uint32_t ReadTrigAddressRange;      /* Specifies the range of Indirect trigger address for QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
    
    uint32_t DMA_SingleBytes;           /* Specifies the Number of bytes of Single type in DMA peripheral request for QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xF */

    uint32_t DMA_BurstBytes;            /* Specifies the Number of bytes of Burst type in DMA peripheral request for QSPI in INDAC mode .
                                            This parameter can be a value (32-bits) between 0x0 and 0xF */
 
}QSPI_INDACModeTypeDef;

/**
  * @brief  QSPI Command structure definition
  */
typedef struct
{
    uint32_t Instruction;            /* Specifies the Instruction to be sent.
                                         This parameter can be a value (8-bit) between 0x00 and 0xFF */
                                     
    uint32_t ReadEn;                 /* Specifies the read data enable or not.
                                         This parameter can be a value of @ref QSPI_ReadEn */
                                     
    uint32_t ReadDataNum;            /* Specifies the number of read data.
                                         This parameter can be a value of @ref QSPI_ReadDataNum */
         
    uint32_t WriteEn;                /* Specifies the write data enable or not.
                                         This parameter can be a value of @ref QSPI_WriteEn */
         
    uint32_t WriteDataNum;           /* Specifies the number of write data.
                                         This parameter can be a value of @ref QSPI_WriteDataNum */
         
    uint32_t AddressEn;              /* Specifies the address enable or not.
                                         This parameter can be a value of @ref QSPI_AddressEn */
         
    uint32_t AddressByte;            /* Specifies the Address Byte.
                                         This parameter can be a value of @ref QSPI_AddressByte */
         
    uint32_t DummyCycles;            /* Specifies the Number of Dummy Cycles.
                                         This parameter can be a value of HAL_QSPI_DUMMYTypeDef */

    uint32_t ModeBitEn;              /* Specifies the model bit enable or not.
                                         This parameter can be a value of QSPI_ModeBitEn */
  
}QSPI_CommandTypeDef;

/**
  * @brief  QSPI Direct access structure definition
  */
typedef struct
{
    uint32_t PollingCycleDelay;      /* Specifies the delay time for the polling cycle.
                                         This parameter can be a value (8-bits) between 0x0 and 0xFF */

    uint32_t PollingCount;           /* Specifies the number of times for the polling mode.
                                         This parameter can be a value (8-bits) between 0x0 and 0xFF */

    uint32_t PollingInterruptEn;     /* Specifies the interrupt of polling mode enable or not.
                                         This parameter can be a value of QSPI_PollingInterruptEn */

    uint32_t PollingPolarity;        /* Specifies the polarity of polling mode.
                                         This parameter can be a value of QSPI_PollingPolarity */
        
    uint32_t PollingBitCheck;        /* Specifies Polling bit retrieval of polling mode.
                                         This parameter can be a value of QSPI_PollingBitCheck */

    uint32_t PollingCommand;         /* Specifies the command for the polling mode.
                                         This parameter can be a value (8-bits) between 0x0 and 0xFF */

    uint32_t PollingCycles;          /* Specifies the number of polling cycles for the polling mode.
                                          This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */

    uint32_t PollingEn;              /* Specifies the function for the polling mode.
                                         This parameter can be a value of QSPI_PollingEn */
    
}QSPI_AutoPollingTypeDef;


/**
  * @brief HAL QSPI State structures definition
  */
typedef enum
{
  HAL_QSPI_STATE_RESET             = 0x00U,    /*!< Peripheral not initialized                            */
  HAL_QSPI_STATE_READY             = 0x01U,    /*!< Peripheral initialized and ready for use              */
  HAL_QSPI_STATE_BUSY              = 0x02U,    /*!< Peripheral in indirect mode and busy                  */
  HAL_QSPI_STATE_BUSY_TX           = 0x12U,    /*!< Peripheral in indirect mode with transmission ongoing */
  HAL_QSPI_STATE_BUSY_RX           = 0x22U,    /*!< Peripheral in indirect mode with reception ongoing    */
  HAL_QSPI_STATE_BUSY_AUTO_POLLING = 0x42U,    /*!< Peripheral in auto polling mode ongoing               */
  HAL_QSPI_STATE_BUSY_MEM_MAPPED   = 0x82U,    /*!< Peripheral in memory mapped mode ongoing              */
  HAL_QSPI_STATE_ABORT             = 0x08U,    /*!< Peripheral with abort request ongoing                 */
  HAL_QSPI_STATE_ERROR             = 0x04U     /*!< Peripheral in error                                   */
}HAL_QSPI_StateTypeDef;

/**
  * @brief QSPI Number of empty instruction clock cycles required by the device instruction 
  */ 
typedef enum
{
	QSPI_DUMMY_CLKS_0,
	QSPI_DUMMY_CLKS_1,
	QSPI_DUMMY_CLKS_2,
	QSPI_DUMMY_CLKS_3,
	QSPI_DUMMY_CLKS_4,
	QSPI_DUMMY_CLKS_5,
	QSPI_DUMMY_CLKS_6,	
	QSPI_DUMMY_CLKS_7,
	QSPI_DUMMY_CLKS_8,
	QSPI_DUMMY_CLKS_9,	
	QSPI_DUMMY_CLKS_10,
	QSPI_DUMMY_CLKS_11,	
	QSPI_DUMMY_CLKS_12,
	QSPI_DUMMY_CLKS_13,	
	QSPI_DUMMY_CLKS_14,
	QSPI_DUMMY_CLKS_15,	
    QSPI_DUMMY_CLKS_16,
	QSPI_DUMMY_CLKS_17,	
	QSPI_DUMMY_CLKS_18,
	QSPI_DUMMY_CLKS_19,	
	QSPI_DUMMY_CLKS_20,
	QSPI_DUMMY_CLKS_21,	
	QSPI_DUMMY_CLKS_22,
	QSPI_DUMMY_CLKS_23,	
    QSPI_DUMMY_CLKS_24,
	QSPI_DUMMY_CLKS_25,
	QSPI_DUMMY_CLKS_26,	
	QSPI_DUMMY_CLKS_27,
	QSPI_DUMMY_CLKS_28,	
    QSPI_DUMMY_CLKS_29,	
	QSPI_DUMMY_CLKS_30,
	QSPI_DUMMY_CLKS_31,	
    QSPI_DUMMY_CLKS_32,		
}HAL_QSPI_DUMMYTypeDef;

/**
  * @brief  QSPI Handle Structure definition
  */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
typedef struct __QSPI_HandleTypeDef
#else
typedef struct
#endif
{
    QSPI_TypeDef               *Instance;        /* QSPI registers base address        */
    
    QSPI_InitTypeDef           Init;             /* QSPI communication parameters      */
    
    QSPI_CommandTypeDef        Command;          /* QSPI communication parameters      */
    
    QSPI_INDACModeTypeDef      IndacMode;        /* QSPI communication parameters      */
    
    QSPI_DACModeTypeDef        DacMode;          /* QSPI communication parameters      */  
    
    QSPI_AutoPollingTypeDef    AutoPolling;      /* QSPI communication parameters      */  
   
    __IO uint32_t              DataSize;         /* Specifies the SPI data size  
                                                    @note This parameter don't apply to INDAC DMA mode.
                                                    This parameter can be a value of @ref SPI_Data_Size */
    
    uint8_t                    *pTxBuffPtr;      /* Pointer to QSPI Tx transfer Buffer */
    
    __IO uint32_t              TxXferCount;      /* QSPI Tx Transfer Counter           */
    
    __IO uint32_t              TxXAddress;       /* QSPI Tx Transfer Start Address     */ 
    
    uint8_t                    *pRxBuffPtr;      /* Pointer to QSPI Rx transfer Buffer */
    
    __IO uint32_t              RxXferCount;      /* QSPI Rx Transfer Counter           */
    
    __IO uint32_t              RxXAddress;       /* QSPI Rx Transfer Start Address     */   
    
    DMA_HandleTypeDef          *hdma_write;      /*!< QSPI Wrte DMA Handle parameters  */
            
    DMA_HandleTypeDef          *hdma_read;       /*!< QSPI Read DMA Handle parameters  */
       
    __IO HAL_LockTypeDef       Lock;             /* Locking object                     */
    
    __IO HAL_QSPI_StateTypeDef State;            /* QSPI communication state           */
    
    __IO uint32_t              ErrorCode;        /* QSPI Error code                    */
    
    uint32_t                   Timeout;          /* Timeout for the QSPI memory access */
    
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
  void (* ErrorCallback)                         (struct __QSPI_HandleTypeDef *hqspi);
  void (* AbortCpltCallback)                     (struct __QSPI_HandleTypeDef *hqspi);
  void (* CmdCpltCallback)                       (struct __QSPI_HandleTypeDef *hqspi);
  void (* UnderFlowCallback)                     (struct __QSPI_HandleTypeDef *hqspi);
  void (* IndacOperationCallback)                (struct __QSPI_HandleTypeDef *hqspi);
  void (* AttemptToWriteProtectCallback)         (struct __QSPI_HandleTypeDef *hqspi);
  void (* IllegalAHBDetectedCallback)            (struct __QSPI_HandleTypeDef *hqspi);
  void (* ReceiveFlowCallback)                   (struct __QSPI_HandleTypeDef *hqspi);
  void (* MaxPollingCyclesCallback)              (struct __QSPI_HandleTypeDef *hqspi);     
  void (* RxCpltCallback)                        (struct __QSPI_HandleTypeDef *hqspi);
  void (* TxCpltCallback)                        (struct __QSPI_HandleTypeDef *hqspi);
  void (* RxNotEmptyCpltCallback)                (struct __QSPI_HandleTypeDef *hqspi);
  void (* TxNotFullCpltCallback)                 (struct __QSPI_HandleTypeDef *hqspi);
  void (* TimeOutCallback)                       (struct __QSPI_HandleTypeDef *hqspi);
                   
  void (* MspInitCallback)                       (struct __QSPI_HandleTypeDef *hqspi);
  void (* MspDeInitCallback)                     (struct __QSPI_HandleTypeDef *hqspi);
#endif
}QSPI_HandleTypeDef;


#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL QSPI Callback ID enumeration definition
  */
typedef enum
{
  HAL_QSPI_ERROR_CB_ID                       = 0x00U,  /*!< QSPI Error Callback ID            */
  HAL_QSPI_ABORT_CB_ID                       = 0x01U,  /*!< QSPI Abort Callback ID            */
  HAL_QSPI_CMD_CPLT_CB_ID                    = 0x02U,  /*!< QSPI Command Complete Callback ID */
  HAL_QSPI_UNDERFOLW_CB_ID                   = 0x03U,  /*!< QSPI Underflow Callback ID */
  HAL_QSPI_INDACOPERATION_CB_ID              = 0x04U,  /*!< QSPI Operation of the indac mode Callback ID */
  HAL_QSPI_ATTEMPTTOWRITEPROTECT_CB_ID       = 0x05U,  /*!< QSPI Attempted to write protection Callback ID */
  HAL_QSPI_ILLEGALAHBDETECTED_CB_ID          = 0x06U,  /*!< QSPI Illegal AHB access detected Callback ID */
  HAL_QSPI_RECEIVEFLOW_CB_ID                 = 0x07U,  /*!< QSPI Receive overflow Callback ID */
  HAL_QSPI_MAXPOLLINGCYCLES_CB_ID            = 0x08U,  /*!< QSPI Maximum number of polling cycles Callback ID */
  HAL_QSPI_RX_CPLT_CB_ID                     = 0x09U,  /*!< QSPI Rx Complete Callback ID      */
  HAL_QSPI_TX_CPLT_CB_ID                     = 0x0AU,  /*!< QSPI Tx Complete Callback ID      */
  HAL_QSPI_RX_NOTEMPTY_CPLT_CB_ID            = 0x0BU,  /*!< QSPI Small capacity RXFIFO is not empty Callback ID      */
  HAL_QSPI_TX_NOTFULL_CPLT_CB_ID             = 0x0CU,  /*!< QSPI Small capacity TXFIFO is not full Callback ID      */
  HAL_QSPI_TIMEOUT_CB_ID                     = 0x0DU,  /*!< QSPI Timeout Callback ID          */

  HAL_QSPI_MSP_INIT_CB_ID                    = 0x0EU,  /*!< QSPI MspInit Callback ID          */
  HAL_QSPI_MSP_DEINIT_CB_ID                  = 0x0F0   /*!< QSPI MspDeInit Callback ID        */
}HAL_QSPI_CallbackIDTypeDef;

/**
  * @brief  HAL QSPI Callback pointer definition
  */
typedef void (*pQSPI_CallbackTypeDef)(QSPI_HandleTypeDef *hqspi);
#endif

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPI_Exported_constants QSPI Exported Constants
  * @{
  */ 

/** @defgroup QSPI_ErrorCode QSPI Error Code
  * @{
  */
#define HAL_QSPI_ERROR_NONE             0x00000000U /*!< No error                 */
#define HAL_QSPI_ERROR_TIMEOUT          0x00000001U /*!< Timeout error            */
#define HAL_QSPI_ERROR_TRANSFER         0x00000002U /*!< Transfer error           */
#define HAL_QSPI_ERROR_RECEIVE          0x00000004U /*!< Transfer error           */
#define HAL_QSPI_ERROR_DMA              0x00000008U /*!< DMA transfer error       */
#define HAL_QSPI_ERROR_INVALID_PARAM    0x00000010U /*!< Invalid parameters error */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
#define HAL_QSPI_ERROR_INVALID_CALLBACK 0x00000010U /*!< Invalid callback error   */
#endif
/**
  * @}
  */

/** @defgroup QSPI_AddrType QSPI Address transmission type in standard SPI mode
  * @{
  */
#define QSPI_ADMODE_DQ0                       0x00000000U
#define QSPI_ADMODE_DQ0DQ1                    0x00001000U
#define QSPI_ADMODE_DQ0DQ1DQ2DQ3              0x00002000U

/**
  * @}
  */  
   
/** @defgroup QSPI_PollingInterruptEn QSPI the interrupt of polling mode enable or not
  * @{
  */
#define QSPI_POLLINGINTERRUPT_CLOSE           0x00000000U
#define QSPI_POLLINGINTERRUPT_OPEN            QSPI_WCR_POLL_EXP_EN

/**
  * @}
  */  
  
/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define QSPI_DATASIZE_8BIT                    (0x00000000U)
#define QSPI_DATASIZE_16BIT                   (0x00000100U)
#define QSPI_DATASIZE_32BIT                   (0x00000200U)

/**
  * @}
  */  

/** @defgroup QSPI_PollingPolarity QSPI the polarity of polling mode
  * @{
  */
#define QSPI_POLLINGPOLARITY_0               0x00000000U
#define QSPI_POLLINGPOLARITY_1               QSPI_WCR_PPLT

/**
  * @}
  */ 

/** @defgroup QSPI_PollingEn QSPI the function for the polling mode
  * @{
  */
#define QSPI_POLLINGEN_CLOSE                 0x00000000U
#define QSPI_POLLINGEN_OPEN                  QSPI_WCR_PDIS

/**
  * @}
  */
  
/** @defgroup QSPI_PollingBitCheck QSPI Polling bit retrieval of polling mode
  * @{
  */
#define QSPI_POLLINGBITCHECK_0               0x00000000U
#define QSPI_POLLINGBITCHECK_1               QSPI_WCR_PBIND_0
#define QSPI_POLLINGBITCHECK_2               QSPI_WCR_PBIND_1 
#define QSPI_POLLINGBITCHECK_3               (QSPI_WCR_PBIND_1 | QSPI_WCR_PBIND_0)
#define QSPI_POLLINGBITCHECK_4               QSPI_WCR_PBIND_2
#define QSPI_POLLINGBITCHECK_5               (QSPI_WCR_PBIND_2 | QSPI_WCR_PBIND_0)
#define QSPI_POLLINGBITCHECK_6               (QSPI_WCR_PBIND_2 | QSPI_WCR_PBIND_1)
#define QSPI_POLLINGBITCHECK_7               (QSPI_WCR_PBIND_2 | QSPI_WCR_PBIND_1 |QSPI_WCR_PBIND_0)

/**
  * @}
  */

/** @defgroup QSPI_DataType QSPI Data transmission type in standard SPI mode
  * @{
  */
#define QSPI_DMODE_SIO                       0x00000000U 
#define QSPI_DMODE_DUAL                      0x00010000U 
#define QSPI_DMODE_QUAD                      0x00020000U 

/**
  * @}
  */  
   
/** @defgroup QSPI_ReadEn QSPI read data enable or not
  * @{
  */
#define QSPI_RDEN_DISABLE                    0x00000000U 
#define QSPI_RDEN_ENABLE                     QSPI_FCR_RD_EN 

/**
  * @}
  */   

/** @defgroup QSPI_WriteEn QSPI write data enable or not
  * @{
  */
#define QSPI_WREN_DISABLE                    0x00000000U 
#define QSPI_WREN_ENABLE                     QSPI_FCR_WR_EN 

/**
  * @}
  */   
 
/** @defgroup QSPI_AddressEn QSPI address of command enable or not
  * @{
  */
#define QSPI_ADDREN_DISABLE                  0x00000000U 
#define QSPI_ADDREN_ENABLE                   QSPI_FCR_ADDR_EN 

/**
  * @}
  */  
  
/** @defgroup QSPI_ModeBitEn QSPI model bit enable or not
  * @{
  */
#define QSPI_MODBEN_DISABLE                  0x00000000U 
#define QSPI_MODBEN_ENABLE                   QSPI_FCR_MODB_EN 

/**
  * @}
  */    

/** @defgroup QSPI_AddressByte QSPI the number for address of command
  * @{
  */
#define QSPI_ADNUM_BYTES_1                   0x00000000U 
#define QSPI_ADNUM_BYTES_2                   ((uint32_t)(QSPI_FCR_AD_NUM_0))
#define QSPI_ADNUM_BYTES_3                   ((uint32_t)(QSPI_FCR_AD_NUM_1)) 
#define QSPI_ADNUM_BYTES_4                   ((uint32_t)(QSPI_FCR_AD_NUM_1 | QSPI_FCR_AD_NUM_0))                   

/**
  * @}
  */

/** @defgroup QSPI_ReadDataNum QSPI the number of read data
  * @{
  */
#define QSPI_RD_NUM_1                       0x00000000U 
#define QSPI_RD_NUM_2                       ((uint32_t)(QSPI_FCR_RD_NUM_0))
#define QSPI_RD_NUM_3                       ((uint32_t)(QSPI_FCR_RD_NUM_1))
#define QSPI_RD_NUM_4                       ((uint32_t)(QSPI_FCR_RD_NUM_1 | QSPI_FCR_RD_NUM_0))
#define QSPI_RD_NUM_5                       ((uint32_t)(QSPI_FCR_RD_NUM_2))
#define QSPI_RD_NUM_6                       ((uint32_t)(QSPI_FCR_RD_NUM_2 | QSPI_FCR_RD_NUM_0))
#define QSPI_RD_NUM_7                       ((uint32_t)(QSPI_FCR_RD_NUM_2 | QSPI_FCR_RD_NUM_1))
#define QSPI_RD_NUM_8                       ((uint32_t)(QSPI_FCR_RD_NUM_2 | QSPI_FCR_RD_NUM_1 | QSPI_FCR_RD_NUM_0))

/**
  * @}
  */   
 
/** @defgroup QSPI_WriteDataNum QSPI the number of write data 
  * @{
  */
#define QSPI_WD_NUM_1                       0x00000000U 
#define QSPI_WD_NUM_2                       ((uint32_t)(QSPI_FCR_WD_NUM_0))
#define QSPI_WD_NUM_3                       ((uint32_t)(QSPI_FCR_WD_NUM_1))
#define QSPI_WD_NUM_4                       ((uint32_t)(QSPI_FCR_WD_NUM_1 | QSPI_FCR_WD_NUM_0))
#define QSPI_WD_NUM_5                       ((uint32_t)(QSPI_FCR_WD_NUM_2))
#define QSPI_WD_NUM_6                       ((uint32_t)(QSPI_FCR_WD_NUM_2 | QSPI_FCR_WD_NUM_0))
#define QSPI_WD_NUM_7                       ((uint32_t)(QSPI_FCR_WD_NUM_2 | QSPI_FCR_WD_NUM_1))
#define QSPI_WD_NUM_8                       ((uint32_t)(QSPI_FCR_WD_NUM_2 | QSPI_FCR_WD_NUM_1 | QSPI_FCR_WD_NUM_0))

/**
  * @}
  */  

/** @defgroup QSPI_ClockPrescaler QSPI Clock Prescaler
  * @note Main mode baud rate division  QSPI baud rate = (master reference clock) /BD 
  *                      [master reference clock comes from clk_sys_pre]
  * @{
  */
#define QSPI_BRDIV_2                   0x00000000U                   
#define QSPI_BRDIV_4                   ((uint32_t)(QSPI_CR_BAUD_0))     
#define QSPI_BRDIV_6                   ((uint32_t)(QSPI_CR_BAUD_1))
#define QSPI_BRDIV_8                   ((uint32_t)(QSPI_CR_BAUD_0 | QSPI_CR_BAUD_1))  
#define QSPI_BRDIV_10                  ((uint32_t)(QSPI_CR_BAUD_2))
#define QSPI_BRDIV_12                  ((uint32_t)(QSPI_CR_BAUD_2 | QSPI_CR_BAUD_0))
#define QSPI_BRDIV_14                  ((uint32_t)(QSPI_CR_BAUD_2 | QSPI_CR_BAUD_1))
#define QSPI_BRDIV_16                  ((uint32_t)(QSPI_CR_BAUD_2 | QSPI_CR_BAUD_1 | QSPI_CR_BAUD_0))
#define QSPI_BRDIV_18                  ((uint32_t)(QSPI_CR_BAUD_3))
#define QSPI_BRDIV_20                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_0))
#define QSPI_BRDIV_22                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_1))
#define QSPI_BRDIV_24                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_1 | QSPI_CR_BAUD_0))
#define QSPI_BRDIV_26                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_2))
#define QSPI_BRDIV_28                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_2 | QSPI_CR_BAUD_0))
#define QSPI_BRDIV_30                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_2 | QSPI_CR_BAUD_1))
#define QSPI_BRDIV_32                  ((uint32_t)(QSPI_CR_BAUD_3 | QSPI_CR_BAUD_2 | QSPI_CR_BAUD_1 | QSPI_CR_BAUD_0))

/**
  * @}
  */


/** @defgroup QSPI_ClockMode QSPI Clock Mode
  * @{
  */
#define QSPI_CLOCK_MODE_0              0x00000000U                    
#define QSPI_CLOCK_MODE_1              ((uint32_t)QSPI_CR_CPOL) 
#define QSPI_CLOCK_MODE_2              ((uint32_t)QSPI_CR_CPHA)                   
#define QSPI_CLOCK_MODE_3              ((uint32_t)(QSPI_CR_CPHA | QSPI_CR_CPOL)) 

/**
  * @}
  */


/** @defgroup QSPI_WorkMode QSPI Work Mode
  * @{
  */
#define	QSPI_WORKMODE_INDACDMA            0x10008000U      
#define	QSPI_WORKMODE_INDAC               0x10000000U      
#define	QSPI_WORKMODE_DAC                 0x00000080U      
#define	QSPI_WORKMODE_STIG                0x00000000U      
#define	QSPI_WORKMODE_STANDARD            0x00000180U      

/**
  * @}
  */

/** @defgroup QSPI_WriteProtectPin QSPI ProtectPin Setting
  * @{
  */
#define	QSPI_SWPP_CLOSE                0x00000000U      
#define	QSPI_SWPP_OPEN                 ((uint32_t)QSPI_CR_SWPP)          

/**
  * @}
  */


/** @defgroup QSPI_AddressRemap QSPI AHB_Address remapping Setting
  * @{
  */
#define	QSPI_AREN_CLOSE                0x00000000U      
#define	QSPI_AREN_OPEN                 ((uint32_t)QSPI_CR_AR_EN)          

/**
  * @}
  */


/** @defgroup QSPI_Enter_XIPIM QSPI Enter XIP mode immediately or not
  * @{
  */
#define	QSPI_XIPIM_CLOSE                0x00000000U      
#define	QSPI_XIPIM_OPEN                 ((uint32_t)QSPI_CR_XIPIM)          

/**
  * @}
  */

/** @defgroup QSPI_Enter_XIPNext QSPI Enter XIP mode at the next read instruction or not.
  * @{
  */
#define	QSPI_XIPNX_CLOSE                0x00000000U      
#define	QSPI_XIPNX_OPEN                 ((uint32_t)QSPI_CR_XIPNX)          

/**
  * @}
  */

/** @defgroup QSPI_AHB_Decoder QSPI AHB decoder enable or not
  * @{
  */
#define	QSPI_ADEN_CLOSE                0x00000000U      
#define	QSPI_ADEN_OPEN                 ((uint32_t)QSPI_CR_AD_EN)          

/**
  * @}
  */
  
/** @defgroup QSPI_DTR_Protocol QSPI DTR protocol enable or not
  * @{
  */
#define	QSPI_DTRM_CLOSE                0x00000000U      
#define	QSPI_DTRM_OPEN                 ((uint32_t)QSPI_CR_DTRM)          

/**
  * @}
  */
  

/** @defgroup QSPI_AddrSizes QSPI Number of device address bytes
  * @{
  */
#define QSPI_ADDRBYTES_1                   0x00000000U
#define	QSPI_ADDRBYTES_2                   ((uint32_t)(QSPI_DSCR_AD_SIZE_0))
#define	QSPI_ADDRBYTES_3                   ((uint32_t)(QSPI_DSCR_AD_SIZE_1))
#define QSPI_ADDRBYTES_4                   ((uint32_t)(QSPI_DSCR_AD_SIZE_1 | QSPI_DSCR_AD_SIZE_0))   
#define QSPI_ADDRBYTES_5                   ((uint32_t)(QSPI_DSCR_AD_SIZE_2)) 
#define	QSPI_ADDRBYTES_6                   ((uint32_t)(QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_0))
#define	QSPI_ADDRBYTES_7                   ((uint32_t)(QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_1)) 
#define QSPI_ADDRBYTES_8                   ((uint32_t)(QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_1 | QSPI_DSCR_AD_SIZE_0))     
#define QSPI_ADDRBYTES_9                   ((uint32_t)(QSPI_DSCR_AD_SIZE_3))  
#define	QSPI_ADDRBYTES_10                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_0)) 
#define	QSPI_ADDRBYTES_11                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_1)) 
#define QSPI_ADDRBYTES_12                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_1 | QSPI_DSCR_AD_SIZE_0))                       
#define QSPI_ADDRBYTES_13                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_2)) 
#define	QSPI_ADDRBYTES_14                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_0))
#define	QSPI_ADDRBYTES_15                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_1))
#define QSPI_ADDRBYTES_16                  ((uint32_t)(QSPI_DSCR_AD_SIZE_3 | QSPI_DSCR_AD_SIZE_2 | QSPI_DSCR_AD_SIZE_1 | QSPI_DSCR_AD_SIZE_0))                      

/**
  * @}
  */
 
/** @defgroup QSPI_BlockSizes QSPI Number of device block bytes
  * @{
  */
#define	QSPI_BLOCKBYTES_1                  0x00000000U      
#define	QSPI_BLOCKBYTES_2                  ((uint32_t)(QSPI_DSCR_BK_SIZE_0)       
#define	QSPI_BLOCKBYTES_4                  ((uint32_t)(QSPI_DSCR_BK_SIZE_1)  
#define	QSPI_BLOCKBYTES_8                  ((uint32_t)(QSPI_DSCR_BK_SIZE_1 | QSPI_DSCR_BK_SIZE_0 )        
#define	QSPI_BLOCKBYTES_16                 ((uint32_t)(QSPI_DSCR_BK_SIZE_2)             
#define	QSPI_BLOCKBYTES_32                 ((uint32_t)(QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_64                 ((uint32_t)(QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_1 )              
#define	QSPI_BLOCKBYTES_128                ((uint32_t)(QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_1 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_256                ((uint32_t)(QSPI_DSCR_BK_SIZE_3))       
#define	QSPI_BLOCKBYTES_512                ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_1024               ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_1))       
#define	QSPI_BLOCKBYTES_2048               ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_1 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_4096               ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_2))       
#define	QSPI_BLOCKBYTES_8192               ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_16384              ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_1))       
#define	QSPI_BLOCKBYTES_32768              ((uint32_t)(QSPI_DSCR_BK_SIZE_3 | QSPI_DSCR_BK_SIZE_2 | QSPI_DSCR_BK_SIZE_1 | QSPI_DSCR_BK_SIZE_0))       
#define	QSPI_BLOCKBYTES_65536              ((uint32_t)(QSPI_DSCR_BK_SIZE_4))       

/**
  * @}
  */ 
  
  
/** @defgroup QSPI_CSSizes QSPI Size of Flash device connected to CS pin
  * @{
  */
#define	QSPI_CSSIZES_512M                0x00000000U      
#define	QSPI_CSSIZES_1G                 ((uint32_t)QSPI_DSCR_CS_SIZE_0)          
#define	QSPI_CSSIZES_2G                 ((uint32_t)QSPI_DSCR_CS_SIZE_1)          
#define	QSPI_CSSIZES_4G                 ((uint32_t)QSPI_DSCR_CS_SIZE)          

/**
  * @}
  */

/** @defgroup QSPI_ReadDelay QSPI Read data capture delay
  * @{
  */
#define	QSPI_DLYR_0                    0x00000000U      
#define	QSPI_DLYR_1                    ((uint32_t)(QSPI_RDCR_DLYR_0))          
#define	QSPI_DLYR_2                    ((uint32_t)(QSPI_RDCR_DLYR_1))          
#define	QSPI_DLYR_3                    ((uint32_t)(QSPI_RDCR_DLYR_1 | QSPI_RDCR_DLYR_0)) 
#define	QSPI_DLYR_4                    ((uint32_t)(QSPI_RDCR_DLYR_2))          
#define	QSPI_DLYR_5                    ((uint32_t)(QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_0))          
#define	QSPI_DLYR_6                    ((uint32_t)(QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_1)) 
#define	QSPI_DLYR_7                    ((uint32_t)(QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_1 | QSPI_RDCR_DLYR_0))          
#define	QSPI_DLYR_8                    ((uint32_t)(QSPI_RDCR_DLYR_3))          
#define	QSPI_DLYR_9                    ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_0)) 
#define	QSPI_DLYR_10                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_1))          
#define	QSPI_DLYR_11                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_1 | QSPI_RDCR_DLYR_0))          
#define	QSPI_DLYR_12                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_2)) 
#define	QSPI_DLYR_13                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_0))         
#define	QSPI_DLYR_14                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_1))          
#define	QSPI_DLYR_15                   ((uint32_t)(QSPI_RDCR_DLYR_3 | QSPI_RDCR_DLYR_2 | QSPI_RDCR_DLYR_1 | QSPI_RDCR_DLYR_0)) 

/**
  * @}
  */
  
/** @defgroup QSPI_TransDelay QSPI Transmission data delay
  * @{
  */
#define	QSPI_DLYT_0                    0x00000000U      
#define	QSPI_DLYT_1                    ((uint32_t)(QSPI_RDCR_DLYT_0))             
#define	QSPI_DLYT_2                    ((uint32_t)(QSPI_RDCR_DLYT_1))             
#define	QSPI_DLYT_3                    ((uint32_t)(QSPI_RDCR_DLYT_1 | QSPI_RDCR_DLYT_0)) 
#define	QSPI_DLYT_4                    ((uint32_t)(QSPI_RDCR_DLYT_2))              
#define	QSPI_DLYT_5                    ((uint32_t)(QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_0))             
#define	QSPI_DLYT_6                    ((uint32_t)(QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_1)) 
#define	QSPI_DLYT_7                    ((uint32_t)(QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_1 | QSPI_RDCR_DLYT_0))             
#define	QSPI_DLYT_8                    ((uint32_t)(QSPI_RDCR_DLYT_3))              
#define	QSPI_DLYT_9                    ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_0)) 
#define	QSPI_DLYT_10                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_1))              
#define	QSPI_DLYT_11                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_1 | QSPI_RDCR_DLYT_0))              
#define	QSPI_DLYT_12                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_2)) 
#define	QSPI_DLYT_13                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_0))             
#define	QSPI_DLYT_14                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_1))              
#define	QSPI_DLYT_15                   ((uint32_t)(QSPI_RDCR_DLYT_3 | QSPI_RDCR_DLYT_2 | QSPI_RDCR_DLYT_1 | QSPI_RDCR_DLYT_0)) 

/**
  * @}
  */  
  
/** @defgroup QSPI_Sampling_Edge QSPI Sampling edge selection
  * @{
  */  
#define QSPI_SMES_TRAILING             0x00000000U       
#define QSPI_SMES_RISING               ((uint32_t)(QSPI_RDCR_SMES))           

/**
  * @}
  */    
 
/** @defgroup QSPI Write protection inversion control
  * @{
  */  
#define QSPI_WPINV_NORMAL             0x00000000U       
#define QSPI_WPINV_REVERSE            ((uint32_t)(QSPI_WPCR_WPINV))           

/**
  * @}
  */  
 
/** @defgroup QSPI Write protection control
  * @{
  */  
#define QSPI_WPEN_PROHIBIT            0x00000000U       
#define QSPI_WPEN_ENABLE              ((uint32_t)(QSPI_WPCR_WP_EN))           

/**
  * @}
  */ 
 
/** @defgroup QSPI_Flags QSPI Flags
  * @{
  */
#define QSPI_FLAG_POLLF                QSPI_IFR_POLLF             /*!<Maximum number of polling cycles flag*/
#define QSPI_FLAG_INDRSFF              QSPI_IFR_IND_RSFF          /*!<The indirect read area in SRAM is full, and cannot be completed immediately flag*/
#define QSPI_FLAG_SRFFF                QSPI_IFR_SRFFF             /*!<Small capacity RXFIFO full flag[Current FIFO status]*/
#define QSPI_FLAG_SRFNEF               QSPI_IFR_SRFNEF            /*!<Small capacity RXFIFO non-empty flag[Current FIFO status]*/
#define QSPI_FLAG_STFFF                QSPI_IFR_STFFF             /*!<Small capacity TXFIFO full flag[Current FIFO status]*/
#define QSPI_FLAG_STFNFF               QSPI_IFR_STFNFF            /*!<Small capacity TXFIFO non-empty flag[Current FIFO status]*/
#define QSPI_FLAG_ROVF                 QSPI_IFR_ROVF              /*!<Receive overflow flag[Only occurs in the traditional mode]*/
#define QSPI_FLAG_INDTWF               QSPI_IFR_IND_TWF           /*!<Exceeds the indirect transmission depth threshold flag*/
#define QSPI_FLAG_AHBAEF               QSPI_IFR_AHB_AEF           /*!<Illegal AHB access flag*/
#define QSPI_FLAG_WPAF                 QSPI_IFR_WPAF              /*!<Attempt to write protected area denied flag*/
#define QSPI_FLAG_INDRRF               QSPI_IFR_IND_RRF           /*!<Not received the indirect operation request flag*/
#define QSPI_FLAG_INDCF                QSPI_IFR_IND_CF            /*!<The controller has completed the last indirect operation flag*/
#define QSPI_FLAG_UDFF                 QSPI_IFR_UDFF              /*!<Check underflow flag*/

/**
  * @}
  */

/** @defgroup QSPI_Interrupts QSPI Interrupts
  * @{
  */
#define QSPI_IT_POLLF                QSPI_IMR_POLLF               /*!<Maximum number of polling cycles*/
#define QSPI_IT_INDRSFF              QSPI_IMR_IND_RSFF            /*!<The indirect read area in SRAM is full, and cannot be completed immediately*/
#define QSPI_IT_SRFFF                QSPI_IMR_SRFFF               /*!<Small capacity RXFIFO full[Current FIFO status]*/
#define QSPI_IT_SRFNEF               QSPI_IMR_SRFNEF              /*!<Small capacity RXFIFO non-empty[Current FIFO status]*/
#define QSPI_IT_STFFF                QSPI_IMR_STFFF               /*!<Small capacity TXFIFO full[Current FIFO status]*/
#define QSPI_IT_STFNFF               QSPI_IMR_STFNFF              /*!<Small capacity TXFIFO non-empty[Current FIFO status]*/
#define QSPI_IT_ROVF                 QSPI_IMR_ROVF                /*!<Receive overflow[Only occurs in the traditional mode]*/
#define QSPI_IT_INDTWF               QSPI_IMR_IND_TWF             /*!<Exceeds the indirect transmission depth threshold*/
#define QSPI_IT_AHBAEF               QSPI_IMR_AHB_AEF             /*!<Illegal AHB access*/
#define QSPI_IT_WPAF                 QSPI_IMR_WPAF                /*!<Attempt to write protected area denied*/ 
#define QSPI_IT_INDRRF               QSPI_IMR_IND_RRF             /*!<To received the indirect operation request*/ 
#define QSPI_IT_INDCF                QSPI_IMR_IND_CF              /*!<The controller has completed the last indirect operation*/
#define QSPI_IT_UDFF                 QSPI_IMR_UDFF                /*!<Check underflow*/

/**
  * @}
  */

/** @defgroup QSPI_Interrupts QSPI Interrupts
  * @{
  */
#define QSPI_STATE_IDLE             QSPI_CR_IDLES

/**
  * @}
  */
  
/** @defgroup QSPI_Timeout_definition QSPI Timeout definition
  * @brief QSPI Timeout definition
  * @{
  */
#define HAL_QSPI_TIMEOUT_DEFAULT_VALUE 5000U /* 5 s */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup QSPI_Exported_macro QSPI Exported Macro
  * @{
  */ 
/** @brief Reset QSPI handle state.
  * @param  __HANDLE__ QSPI handle.
  * @retval None
  */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
#define __HAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)           do {                                              \
                                                                  (__HANDLE__)->State = HAL_QSPI_STATE_RESET; \
                                                                  (__HANDLE__)->MspInitCallback = NULL;       \
                                                                  (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                               } while(0)
#else
#define __HAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)           ((__HANDLE__)->State = HAL_QSPI_STATE_RESET)
#endif

/** @brief  Enable the QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_ENABLE(__HANDLE__)                       SET_BIT((__HANDLE__)->Instance->CR, QSPI_CR_EN)

/** @brief  Disable the QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval None
  */
#define __HAL_QSPI_DISABLE(__HANDLE__)                      CLEAR_BIT((__HANDLE__)->Instance->CR, QSPI_CR_EN)

/** @brief  Wait idle the QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __FLAG__   This parameter  can be QSPI_STATE_IDLE only
  * @retval None
  */
#define __HAL_QSPI_GET_IDLE(__HANDLE__ , __FLAG__)         ((READ_BIT((__HANDLE__)->Instance->CR, (__FLAG__)) != 0U) ? SET : RESET)

/** @brief  Enable the specified QSPI interrupt.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __INTERRUPT__ specifies the QSPI interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_POLLF   : Maximum number of polling cycles interrupt
  *            @arg QSPI_IT_INDRSFF : The indirect read area in SRAM is full, and cannot be completed immediately interrupt
  *            @arg QSPI_IT_SRFFF   : Small capacity RXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_SRFNEF  : Small capacity RXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFFF   : Small capacity TXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFNFF  : Small capacity TXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_ROVF    : Receive overflow[Only occurs in the traditional mode] interrupt
  *            @arg QSPI_IT_INDTWF  : Exceeds the indirect transmission depth threshold interrupt
  *            @arg QSPI_IT_AHBAEF  : Illegal AHB access interrupt
  *            @arg QSPI_IT_WPAF    : Attempt to write protected area denied interrupt
  *            @arg QSPI_IT_INDRRF  : To received the indirect operation request interrupt
  *            @arg QSPI_IT_INDCF   : The controller has completed the last indirect operation interrupt
  *            @arg QSPI_IT_UDFF    : Check underflow interrupt
  * @retval None
  */
#define __HAL_QSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)     SET_BIT((__HANDLE__)->Instance->IMR, (__INTERRUPT__))

/** @brief  Disable the specified QSPI interrupt.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __INTERRUPT__ specifies the QSPI interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_POLLF   : Maximum number of polling cycles interrupt
  *            @arg QSPI_IT_INDRSFF : The indirect read area in SRAM is full, and cannot be completed immediately interrupt
  *            @arg QSPI_IT_SRFFF   : Small capacity RXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_SRFNEF  : Small capacity RXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFFF   : Small capacity TXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFNFF  : Small capacity TXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_ROVF    : Receive overflow[Only occurs in the traditional mode] interrupt
  *            @arg QSPI_IT_INDTWF  : Exceeds the indirect transmission depth threshold interrupt
  *            @arg QSPI_IT_AHBAEF  : Illegal AHB access interrupt
  *            @arg QSPI_IT_WPAF    : Attempt to write protected area denied interrupt
  *            @arg QSPI_IT_INDRRF  : To received the indirect operation request interrupt
  *            @arg QSPI_IT_INDCF   : The controller has completed the last indirect operation interrupt
  *            @arg QSPI_IT_UDFF    : Check underflow interrupt
  * @retval None
  */
#define __HAL_QSPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)    CLEAR_BIT((__HANDLE__)->Instance->IMR, (__INTERRUPT__))

/** @brief  Check whether the specified QSPI interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __INTERRUPT__ specifies the QSPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_POLLF   : Maximum number of polling cycles interrupt
  *            @arg QSPI_IT_INDRSFF : The indirect read area in SRAM is full, and cannot be completed immediately interrupt
  *            @arg QSPI_IT_SRFFF   : Small capacity RXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_SRFNEF  : Small capacity RXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFFF   : Small capacity TXFIFO full[Current FIFO status] interrupt
  *            @arg QSPI_IT_STFNFF  : Small capacity TXFIFO non-empty[Current FIFO status] interrupt
  *            @arg QSPI_IT_ROVF    : Receive overflow[Only occurs in the traditional mode] interrupt
  *            @arg QSPI_IT_INDTWF  : Exceeds the indirect transmission depth threshold interrupt
  *            @arg QSPI_IT_AHBAEF  : Illegal AHB access interrupt
  *            @arg QSPI_IT_WPAF    : Attempt to write protected area denied interrupt
  *            @arg QSPI_IT_INDRRF  : To received the indirect operation request interrupt
  *            @arg QSPI_IT_INDCF   : The controller has completed the last indirect operation interrupt
  *            @arg QSPI_IT_UDFF    : Check underflow interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_QSPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)         READ_BIT((__HANDLE__)->Instance->IMR, (__INTERRUPT__)) == (__INTERRUPT__)


/**
  * @brief  Check whether the selected QSPI flag is set or not.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __FLAG__ specifies the QSPI flag to check.
  *          This parameter can be one of the following values:
  *            @arg QSPI_FLAG_POLLF   : Maximum number of polling cycles flag
  *            @arg QSPI_FLAG_INDRSFF : The indirect read area in SRAM is full, and cannot be completed immediately flag
  *            @arg QSPI_FLAG_SRFFF   : Small capacity RXFIFO full flag[Current FIFO status]
  *            @arg QSPI_FLAG_SRFNEF  : Small capacity RXFIFO non-empty flag[Current FIFO status]
  *            @arg QSPI_FLAG_STFFF   : Small capacity TXFIFO full flag[Current FIFO status]
  *            @arg QSPI_FLAG_STFNFF  : Small capacity TXFIFO non-empty flag[Current FIFO status]
  *            @arg QSPI_FLAG_ROVF    : Receive overflow flag[Only occurs in the traditional mode]
  *            @arg QSPI_FLAG_INDTWF  : Exceeds the indirect transmission depth threshold flag
  *            @arg QSPI_FLAG_AHBAEF  : Illegal AHB access flag
  *            @arg QSPI_FLAG_WPAF    : Attempt to write protected area denied flag
  *            @arg QSPI_FLAG_INDRRF  : Not received the indirect operation request flag
  *            @arg QSPI_FLAG_INDCF   : The controller has completed the last indirect operation flag
  *            @arg QSPI_FLAG_UDFF    : Check underflow flag
  * @retval None
  */ 
#define __HAL_QSPI_GET_FLAG(__HANDLE__, __FLAG__)           ((READ_BIT((__HANDLE__)->Instance->IFR, (__FLAG__)) != 0U) ? SET : RESET)
    
/** @brief  Clears the specified QSPI's flag status.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __FLAG__ specifies the QSPI clear register flag that needs to be set
  *          This parameter can be one of the following values:
  *            @arg QSPI_FLAG_POLLF   : Maximum number of polling cycles flag
  *            @arg QSPI_FLAG_INDRSFF : The indirect read area in SRAM is full, and cannot be completed immediately flag
  *            @arg QSPI_FLAG_SRFFF   : Small capacity RXFIFO full flag[Current FIFO status]
  *            @arg QSPI_FLAG_SRFNEF  : Small capacity RXFIFO non-empty flag[Current FIFO status]
  *            @arg QSPI_FLAG_STFFF   : Small capacity TXFIFO full flag[Current FIFO status]
  *            @arg QSPI_FLAG_STFNFF  : Small capacity TXFIFO non-empty flag[Current FIFO status]
  *            @arg QSPI_FLAG_ROVF    : Receive overflow flag[Only occurs in the traditional mode]
  *            @arg QSPI_FLAG_INDTWF  : Exceeds the indirect transmission depth threshold flag
  *            @arg QSPI_FLAG_AHBAEF  : Illegal AHB access flag
  *            @arg QSPI_FLAG_WPAF    : Attempt to write protected area denied flag
  *            @arg QSPI_FLAG_INDRRF  : Not received the indirect operation request flag
  *            @arg QSPI_FLAG_INDCF   : The controller has completed the last indirect operation flag
  *            @arg QSPI_FLAG_UDFF    : Check underflow flag
  * @retval None
  */
#define __HAL_QSPI_CLEAR_FLAG(__HANDLE__, __FLAG__)         SET_BIT((__HANDLE__)->Instance->IFR, (__FLAG__))
  
/** @brief  Setting the QSPI_RXHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  This parameter can be a value (5-bits) between 0x0 and 0x1F.  
  * @retval None.
  */ 
#define __HAL_QSPI_SET_RXHR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->RXHR,__VALUE__)
 
/** @brief  Setting the QSPI_TXHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  This parameter can be a value (5-bits) between 0x0 and 0x1F.  
  * @retval None.
  */  
#define __HAL_QSPI_SET_TXHR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->TXHR,__VALUE__)

/** @brief  Setting the QSPI_RAR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF.   
  * @retval None.
  */ 
#define __HAL_QSPI_SET_RAR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->RAR,__VALUE__)

/** @brief  Getting the QSPI_RAR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval None.
  */ 
#define __HAL_QSPI_GET_RAR(__HANDLE__)        	            READ_REG(__HANDLE__->Instance->RAR)

/** @brief  Setting the QSPI_MBR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  This parameter can be a value (8-bits) between 0x0 and 0xFF.   
  * @retval None.
  */ 
#define __HAL_QSPI_SET_MBR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->MBR,__VALUE__)

/** @brief  Getting the QSPI_MBR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.  
  * @retval None.
  */ 
#define __HAL_QSPI_GET_MBR(__HANDLE__)        	            READ_REG(__HANDLE__->Instance->MBR)

/** @brief  Setting the QSPI_WPLR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting.  
  * @retval None.
  */ 
#define __HAL_QSPI_SET_WPLR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->MPLR,__VALUE__)

/** @brief  Setting the QSPI_WPHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting.  
  * @retval None.
  */ 
#define __HAL_QSPI_SET_WPHR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->MPHR,__VALUE__)

/** @brief  Getting the QSPI_WPLR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.  
  * @retval None.
  */ 
#define __HAL_QSPI_GET_WPLR(__HANDLE__)        	            READ_REG(__HANDLE__->Instance->MPLR)

/** @brief  Getting the QSPI_WPHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval None.
  */ 
#define __HAL_QSPI_GET_WPHR(__HANDLE__)        	            READ_REG(__HANDLE__->Instance->MPHR)

/** @brief  Setting the QSPI_SPR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting.  
  * @retval None.
  */ 
#define __HAL_QSPI_SET_SPR(__HANDLE__, __VALUE__)        	WRITE_REG(__HANDLE__->Instance->SPR,__VALUE__)

/** @brief  Setting ahb write protection area.
  * @param  __HANDLE__  specifies the QSPI Handle.
  * @param  __DIR__     The value of want to setting.
  *          This parameter can be one of the following values:  
  *            @arg  QSPI_WPINV_NORMAL   : Write protection not reverse
  *            @arg  QSPI_WPINV_REVERSE  : Write protection reverse
  * @param  __EN__      The value of want to setting. 
  *          This parameter can be one of the following values:    
  *            @arg  QSPI_WPEN_PROHIBIT  : Write protection prohibition
  *            @arg  QSPI_WPEN_ENABLE    : Write protection enable
  * @retval None.
  */ 
#define __HAL_QSPI_WRITEPROTECT_AHB(__HANDLE__, __DIR__,__EN__)    do{ \
                                                                    __IO uint32_t temg; \
                                                                    temg = (__DIR__ | __EN__); \
                                                                    WRITE_REG(__HANDLE__->Instance->WPCR,temg); \
                                                                    UNUSED(temg); \
                                                                   }while(0U)    
 
/** @brief  Getting the QSPI_PFSR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval The data of QSPI_PFSR register.
  */ 
#define __HAL_QSPI_GET_PFSR(__HANDLE__)        	            READ_REG(__HANDLE__->Instance->PFSR)

/** @brief  getting the QSPI_FCRLR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval The data of QSPI_FCRLR register.
  */ 
#define __HAL_QSPI_GET_FCRLR(__HANDLE__)        	        READ_REG(__HANDLE__->Instance->FCRLR)
  
/** @brief  getting the QSPI_FCRHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval The data of QSPI_FCRHR register.
  */ 
#define __HAL_QSPI_GET_FCRHR(__HANDLE__)        	        READ_REG(__HANDLE__->Instance->FCRHR)
                                                                    
 /** @brief  Wait command run idle for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval None
  */ 
#define __HAL_QSPI_WAIT_COMMAND_IDLE(__HANDLE__)        	while(((__HANDLE__)->Instance->FCR) & 0x3)
                                                                  
/** @brief  Setting the QSPI_FCWLR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting. 
  * @retval None
  */ 
#define __HAL_QSPI_SET_FCWLR(__HANDLE__ ,__VALUE__)        	 WRITE_REG(__HANDLE__->Instance->FCWLR,__VALUE__)

/** @brief  Setting the QSPI_FCWHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting. 
  * @retval None
  */ 
#define __HAL_QSPI_SET_FCWHR(__HANDLE__ ,__VALUE__)        	 WRITE_REG(__HANDLE__->Instance->FCWHR,__VALUE__)

/** @brief  Getting the QSPI_FCWHR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @param  __VALUE__  The value of want to setting. 
  * @retval The data of QSPI_FCWHR register.
  */ 
#define __HAL_QSPI_GET_FCWHR(__HANDLE__)        	         READ_REG(__HANDLE__->Instance->FCWHR)

/** @brief  Getting the QSPI_FCWLR register for QSPI peripheral.
  * @param  __HANDLE__ specifies the QSPI Handle.
  * @retval The data of QSPI_FCWLR register.
  */ 
#define __HAL_QSPI_GET_FCWLR(__HANDLE__)        	         READ_REG(__HANDLE__->Instance->FCWLR)
 
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup QSPI_Exported_Functions
  * @{
  */ 

/** @addtogroup QSPI_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef     HAL_QSPI_Init                 (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_DeInit               (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspInit              (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspDeInit            (QSPI_HandleTypeDef *hqspi);

/**
  * @}
  */

/** @addtogroup QSPI_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
/* QSPI IRQ handler method */
void                  HAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi);

/* QSPI direct mode */
HAL_StatusTypeDef     HAL_QSPI_DirectTransmit       (QSPI_HandleTypeDef *hqspi, uint32_t StartAddress,uint8_t *pData, uint32_t DataLen, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_DirectReceive        (QSPI_HandleTypeDef *hqspi, uint32_t StartAddress,uint8_t *pData, uint32_t DataLen, uint32_t Timeout);

/* QSPI stig mode */
HAL_StatusTypeDef     HAL_QSPI_Command              (QSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t Data, uint32_t Timeout);

/* QSPI indirect mode */
HAL_StatusTypeDef     HAL_QSPI_IndirectTransmit     (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t DataLen, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_IndirectReceive      (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t DataLen, uint32_t Timeout);

/* QSPI indac dma mode */
HAL_StatusTypeDef     HAL_QSPI_Transmit_DMA         (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t DataLen, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Receive_DMA          (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t DataLen, uint32_t Timeout);


/* Callback functions in non-blocking modes ***********************************/
void                  HAL_QSPI_ErrorCallback                    (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_AbortCpltCallback                (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_CmdCpltCallback                  (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_UnderFlowCallback                (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_IndacOperationCallback           (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_AttemptToWriteProtectCallback    (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_IllegalAHBDetectedCallback       (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_ReceiveFlowCallback              (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MaxPollingCyclesCallback         (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxCpltCallback                   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxCpltCallback                   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxNotEmptyCpltCallback           (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxNotFullCpltCallback            (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TimeOutCallback                  (QSPI_HandleTypeDef *hqspi);

#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
/* QSPI callback registering/unregistering */
HAL_StatusTypeDef     HAL_QSPI_RegisterCallback     (QSPI_HandleTypeDef *hqspi, HAL_QSPI_CallbackIDTypeDef CallbackId, pQSPI_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_QSPI_UnRegisterCallback   (QSPI_HandleTypeDef *hqspi, HAL_QSPI_CallbackIDTypeDef CallbackId);
#endif


/**
  * @}
  */
  
/** @addtogroup QSPI_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control and State functions  ************************************/
HAL_QSPI_StateTypeDef HAL_QSPI_GetState             (QSPI_HandleTypeDef *hqspi);
uint32_t              HAL_QSPI_GetError             (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_Abort_IT             (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_SetTimeout           (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_WaitIndacWriteCmpl   (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_WaitIndacReadCmpl    (QSPI_HandleTypeDef *hqspi);

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup QSPI_Private_Macros 
  * @{
  */
  
   
  
  
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup QSPI_Private_Functions QSPI Private Functions
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
