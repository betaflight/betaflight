/**
  *
  * @file    apm32f4xx_dal_mmc.h
  * @brief   Header file of MMC DAL module.
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
#ifndef APM32F4xx_DAL_MMC_H
#define APM32F4xx_DAL_MMC_H

#if defined(SDIO)

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_sdmmc.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup MMC
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/ 
/** @defgroup MMC_Exported_Types MMC Exported Types
  * @{
  */

/** @defgroup MMC_Exported_Types_Group1 MMC State enumeration structure
  * @{
  */   
typedef enum
{
  DAL_MMC_STATE_RESET                  = 0x00000000U,  /*!< MMC not yet initialized or disabled  */
  DAL_MMC_STATE_READY                  = 0x00000001U,  /*!< MMC initialized and ready for use    */
  DAL_MMC_STATE_TIMEOUT                = 0x00000002U,  /*!< MMC Timeout state                    */
  DAL_MMC_STATE_BUSY                   = 0x00000003U,  /*!< MMC process ongoing                  */
  DAL_MMC_STATE_PROGRAMMING            = 0x00000004U,  /*!< MMC Programming State                */
  DAL_MMC_STATE_RECEIVING              = 0x00000005U,  /*!< MMC Receinving State                 */
  DAL_MMC_STATE_TRANSFER               = 0x00000006U,  /*!< MMC Transfer State                  */
  DAL_MMC_STATE_ERROR                  = 0x0000000FU   /*!< MMC is in error state                */
}DAL_MMC_StateTypeDef;
/** 
  * @}
  */

/** @defgroup MMC_Exported_Types_Group2 MMC Card State enumeration structure
  * @{
  */   
typedef uint32_t DAL_MMC_CardStateTypeDef;

#define DAL_MMC_CARD_READY          0x00000001U  /*!< Card state is ready                     */
#define DAL_MMC_CARD_IDENTIFICATION 0x00000002U  /*!< Card is in identification state         */
#define DAL_MMC_CARD_STANDBY        0x00000003U  /*!< Card is in standby state                */
#define DAL_MMC_CARD_TRANSFER       0x00000004U  /*!< Card is in transfer state               */
#define DAL_MMC_CARD_SENDING        0x00000005U  /*!< Card is sending an operation            */
#define DAL_MMC_CARD_RECEIVING      0x00000006U  /*!< Card is receiving operation information */
#define DAL_MMC_CARD_PROGRAMMING    0x00000007U  /*!< Card is in programming state            */
#define DAL_MMC_CARD_DISCONNECTED   0x00000008U  /*!< Card is disconnected                    */
#define DAL_MMC_CARD_ERROR          0x000000FFU  /*!< Card response Error                     */
/** 
  * @}
  */

/** @defgroup MMC_Exported_Types_Group3 MMC Handle Structure definition   
  * @{
  */
#define MMC_InitTypeDef      SDIO_InitTypeDef 
#define MMC_TypeDef          SDIO_TypeDef

/** 
  * @brief  MMC Card Information Structure definition
  */ 
typedef struct
{
  uint32_t CardType;                     /*!< Specifies the card Type                         */

  uint32_t Class;                        /*!< Specifies the class of the card class           */

  uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */
  
  uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */

  uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */
  
  uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */

  uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */

}DAL_MMC_CardInfoTypeDef;

/** 
  * @brief  MMC handle Structure definition
  */ 
#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
typedef struct __MMC_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_MMC_REGISTER_CALLBACKS */
{
  MMC_TypeDef                  *Instance;        /*!< MMC registers base address           */
  
  MMC_InitTypeDef              Init;             /*!< MMC required parameters              */
  
  DAL_LockTypeDef              Lock;             /*!< MMC locking object                   */
  
  uint8_t                      *pTxBuffPtr;      /*!< Pointer to MMC Tx transfer Buffer    */

  uint32_t                     TxXferSize;       /*!< MMC Tx Transfer size                 */

  uint8_t                      *pRxBuffPtr;      /*!< Pointer to MMC Rx transfer Buffer    */

  uint32_t                     RxXferSize;       /*!< MMC Rx Transfer size                 */
  
  __IO uint32_t                Context;          /*!< MMC transfer context                 */
 
  __IO DAL_MMC_StateTypeDef    State;            /*!< MMC card State                       */
  
  __IO uint32_t                ErrorCode;        /*!< MMC Card Error codes                 */  
 
  DMA_HandleTypeDef            *hdmarx;          /*!< MMC Rx DMA handle parameters         */
  
  DMA_HandleTypeDef            *hdmatx;          /*!< MMC Tx DMA handle parameters         */
  
  DAL_MMC_CardInfoTypeDef      MmcCard;          /*!< MMC Card information                 */

  uint32_t                     CSD[4U];          /*!< MMC card specific data table         */
  
  uint32_t                     CID[4U];          /*!< MMC card identification number table */
  
  uint32_t                     Ext_CSD[128];

#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
  void (* TxCpltCallback)                 (struct __MMC_HandleTypeDef *hmmc);
  void (* RxCpltCallback)                 (struct __MMC_HandleTypeDef *hmmc);
  void (* ErrorCallback)                  (struct __MMC_HandleTypeDef *hmmc);
  void (* AbortCpltCallback)              (struct __MMC_HandleTypeDef *hmmc);

  void (* MspInitCallback)                (struct __MMC_HandleTypeDef *hmmc);
  void (* MspDeInitCallback)              (struct __MMC_HandleTypeDef *hmmc);
#endif  
}MMC_HandleTypeDef;

/** 
  * @}
  */

/** @defgroup MMC_Exported_Types_Group4 Card Specific Data: CSD Register 
  * @{
  */
typedef struct
{
  __IO uint8_t  CSDStruct;            /*!< CSD structure                         */
  __IO uint8_t  SysSpecVersion;       /*!< System specification version          */
  __IO uint8_t  Reserved1;            /*!< Reserved                              */
  __IO uint8_t  TAAC;                 /*!< Data read access time 1               */
  __IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
  __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
  __IO uint16_t CardComdClasses;      /*!< Card command classes                  */
  __IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
  __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
  __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
  __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
  __IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
  __IO uint8_t  Reserved2;            /*!< Reserved                              */
  __IO uint32_t DeviceSize;           /*!< Device Size                           */
  __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
  __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
  __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
  __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
  __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
  __IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
  __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
  __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
  __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
  __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
  __IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
  __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
  __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
  __IO uint8_t  Reserved3;            /*!< Reserved                              */
  __IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
  __IO uint8_t  FileFormatGroup;      /*!< File format group                     */
  __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
  __IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
  __IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
  __IO uint8_t  FileFormat;           /*!< File format                           */
  __IO uint8_t  ECC;                  /*!< ECC code                              */
  __IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
  __IO uint8_t  Reserved4;            /*!< Always 1                              */
  
}DAL_MMC_CardCSDTypeDef;
/** 
  * @}
  */

/** @defgroup MMC_Exported_Types_Group5 Card Identification Data: CID Register
  * @{
  */
typedef struct
{
  __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
  __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
  __IO uint32_t ProdName1;       /*!< Product Name part1    */
  __IO uint8_t  ProdName2;       /*!< Product Name part2    */
  __IO uint8_t  ProdRev;         /*!< Product Revision      */
  __IO uint32_t ProdSN;          /*!< Product Serial Number */
  __IO uint8_t  Reserved1;       /*!< Reserved1             */
  __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
  __IO uint8_t  CID_CRC;         /*!< CID CRC               */
  __IO uint8_t  Reserved2;       /*!< Always 1              */

}DAL_MMC_CardCIDTypeDef;
/** 
  * @}
  */

#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
/** @defgroup MMC_Exported_Types_Group6 MMC Callback ID enumeration definition 
  * @{
  */
typedef enum
{
  DAL_MMC_TX_CPLT_CB_ID                 = 0x00U,  /*!< MMC Tx Complete Callback ID                     */
  DAL_MMC_RX_CPLT_CB_ID                 = 0x01U,  /*!< MMC Rx Complete Callback ID                     */
  DAL_MMC_ERROR_CB_ID                   = 0x02U,  /*!< MMC Error Callback ID                           */
  DAL_MMC_ABORT_CB_ID                   = 0x03U,  /*!< MMC Abort Callback ID                           */

  DAL_MMC_MSP_INIT_CB_ID                = 0x10U,  /*!< MMC MspInit Callback ID                         */
  DAL_MMC_MSP_DEINIT_CB_ID              = 0x11U   /*!< MMC MspDeInit Callback ID                       */
}DAL_MMC_CallbackIDTypeDef;
/** 
  * @}
  */

/** @defgroup MMC_Exported_Types_Group7 MMC Callback pointer definition 
  * @{
  */
typedef void (*pMMC_CallbackTypeDef)           (MMC_HandleTypeDef *hmmc);
/** 
  * @}
  */
#endif
/** 
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup MMC_Exported_Constants Exported Constants
  * @{
  */

#define MMC_BLOCKSIZE              512U  /*!< Block size is 512 bytes */

/** @defgroup MMC_Exported_Constansts_Group1 MMC Error status enumeration Structure definition 
  * @{
  */  
#define DAL_MMC_ERROR_NONE                     SDMMC_ERROR_NONE                    /*!< No error                                                      */
#define DAL_MMC_ERROR_CMD_CRC_FAIL             SDMMC_ERROR_CMD_CRC_FAIL            /*!< Command response received (but CRC check failed)              */
#define DAL_MMC_ERROR_DATA_CRC_FAIL            SDMMC_ERROR_DATA_CRC_FAIL           /*!< Data block sent/received (CRC check failed)                   */
#define DAL_MMC_ERROR_CMD_RSP_TIMEOUT          SDMMC_ERROR_CMD_RSP_TIMEOUT         /*!< Command response timeout                                      */
#define DAL_MMC_ERROR_DATA_TIMEOUT             SDMMC_ERROR_DATA_TIMEOUT            /*!< Data timeout                                                  */
#define DAL_MMC_ERROR_TX_UNDERRUN              SDMMC_ERROR_TX_UNDERRUN             /*!< Transmit FIFO underrun                                        */
#define DAL_MMC_ERROR_RX_OVERRUN               SDMMC_ERROR_RX_OVERRUN              /*!< Receive FIFO overrun                                          */
#define DAL_MMC_ERROR_ADDR_MISALIGNED          SDMMC_ERROR_ADDR_MISALIGNED         /*!< Misaligned address                                            */
#define DAL_MMC_ERROR_BLOCK_LEN_ERR            SDMMC_ERROR_BLOCK_LEN_ERR           /*!< Transferred block length is not allowed for the card or the 
                                                                                       number of transferred bytes does not match the block length   */
#define DAL_MMC_ERROR_ERASE_SEQ_ERR            SDMMC_ERROR_ERASE_SEQ_ERR           /*!< An error in the sequence of erase command occurs              */
#define DAL_MMC_ERROR_BAD_ERASE_PARAM          SDMMC_ERROR_BAD_ERASE_PARAM         /*!< An invalid selection for erase groups                         */
#define DAL_MMC_ERROR_WRITE_PROT_VIOLATION     SDMMC_ERROR_WRITE_PROT_VIOLATION    /*!< Attempt to program a write protect block                      */
#define DAL_MMC_ERROR_LOCK_UNLOCK_FAILED       SDMMC_ERROR_LOCK_UNLOCK_FAILED      /*!< Sequence or password error has been detected in unlock 
                                                                                       command or if there was an attempt to access a locked card    */
#define DAL_MMC_ERROR_COM_CRC_FAILED           SDMMC_ERROR_COM_CRC_FAILED          /*!< CRC check of the previous command failed                      */
#define DAL_MMC_ERROR_ILLEGAL_CMD              SDMMC_ERROR_ILLEGAL_CMD             /*!< Command is not legal for the card state                       */
#define DAL_MMC_ERROR_CARD_ECC_FAILED          SDMMC_ERROR_CARD_ECC_FAILED         /*!< Card internal ECC was applied but failed to correct the data  */
#define DAL_MMC_ERROR_CC_ERR                   SDMMC_ERROR_CC_ERR                  /*!< Internal card controller error                                */
#define DAL_MMC_ERROR_GENERAL_UNKNOWN_ERR      SDMMC_ERROR_GENERAL_UNKNOWN_ERR     /*!< General or unknown error                                      */
#define DAL_MMC_ERROR_STREAM_READ_UNDERRUN     SDMMC_ERROR_STREAM_READ_UNDERRUN    /*!< The card could not sustain data reading in stream rmode       */
#define DAL_MMC_ERROR_STREAM_WRITE_OVERRUN     SDMMC_ERROR_STREAM_WRITE_OVERRUN    /*!< The card could not sustain data programming in stream mode    */
#define DAL_MMC_ERROR_CID_CSD_OVERWRITE        SDMMC_ERROR_CID_CSD_OVERWRITE       /*!< CID/CSD overwrite error                                       */
#define DAL_MMC_ERROR_WP_ERASE_SKIP            SDMMC_ERROR_WP_ERASE_SKIP           /*!< Only partial address space was erased                         */
#define DAL_MMC_ERROR_CARD_ECC_DISABLED        SDMMC_ERROR_CARD_ECC_DISABLED       /*!< Command has been executed without using internal ECC          */
#define DAL_MMC_ERROR_ERASE_RESET              SDMMC_ERROR_ERASE_RESET             /*!< Erase sequence was cleared before executing because an out 
                                                                                       of erase sequence command was received                        */
#define DAL_MMC_ERROR_AKE_SEQ_ERR              SDMMC_ERROR_AKE_SEQ_ERR             /*!< Error in sequence of authentication                           */
#define DAL_MMC_ERROR_INVALID_VOLTRANGE        SDMMC_ERROR_INVALID_VOLTRANGE       /*!< Error in case of invalid voltage range                        */        
#define DAL_MMC_ERROR_ADDR_OUT_OF_RANGE        SDMMC_ERROR_ADDR_OUT_OF_RANGE       /*!< Error when addressed block is out of range                    */        
#define DAL_MMC_ERROR_REQUEST_NOT_APPLICABLE   SDMMC_ERROR_REQUEST_NOT_APPLICABLE  /*!< Error when command request is not applicable                  */  
#define DAL_MMC_ERROR_PARAM                    SDMMC_ERROR_INVALID_PARAMETER       /*!< the used parameter is not valid                               */  
#define DAL_MMC_ERROR_UNSUPPORTED_FEATURE      SDMMC_ERROR_UNSUPPORTED_FEATURE     /*!< Error when feature is not insupported                         */
#define DAL_MMC_ERROR_BUSY                     SDMMC_ERROR_BUSY                    /*!< Error when transfer process is busy                           */ 
#define DAL_MMC_ERROR_DMA                      SDMMC_ERROR_DMA                     /*!< Error while DMA transfer                                      */
#define DAL_MMC_ERROR_TIMEOUT                  SDMMC_ERROR_TIMEOUT                 /*!< Timeout error                                                 */
                                                
#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
#define DAL_MMC_ERROR_INVALID_CALLBACK         SDMMC_ERROR_INVALID_PARAMETER       /*!< Invalid callback error                                        */
#endif
/** 
  * @}
  */
 
/** @defgroup MMC_Exported_Constansts_Group2 MMC context enumeration
  * @{
  */ 
#define   MMC_CONTEXT_NONE                            0x00000000U   /*!< None                             */
#define   MMC_CONTEXT_READ_SINGLE_BLOCK               0x00000001U   /*!< Read single block operation      */
#define   MMC_CONTEXT_READ_MULTIPLE_BLOCK             0x00000002U   /*!< Read multiple blocks operation   */
#define   MMC_CONTEXT_WRITE_SINGLE_BLOCK              0x00000010U   /*!< Write single block operation     */
#define   MMC_CONTEXT_WRITE_MULTIPLE_BLOCK            0x00000020U   /*!< Write multiple blocks operation  */
#define   MMC_CONTEXT_IT                              0x00000008U   /*!< Process in Interrupt mode        */
#define   MMC_CONTEXT_DMA                             0x00000080U   /*!< Process in DMA mode              */

/**
  * @}
  */

/** @defgroup MMC_Exported_Constansts_Group3 MMC Voltage mode
  * @{
  */
/** 
  * @brief 
  */
#define MMC_HIGH_VOLTAGE_RANGE         0x80FF8000U  /*!< High voltage in byte mode    */
#define MMC_DUAL_VOLTAGE_RANGE         0x80FF8080U  /*!< Dual voltage in byte mode    */
#define MMC_LOW_VOLTAGE_RANGE          0x80000080U  /*!< Low voltage in byte mode     */
#define EMMC_HIGH_VOLTAGE_RANGE        0xC0FF8000U  /*!< High voltage in sector mode  */
#define EMMC_DUAL_VOLTAGE_RANGE        0xC0FF8080U  /*!< Dual voltage in sector mode  */
#define EMMC_LOW_VOLTAGE_RANGE         0xC0000080U  /*!< Low voltage in sector mode   */
#define MMC_INVALID_VOLTAGE_RANGE      0x0001FF01U 
/**
  * @}
  */

/** @defgroup MMC_Exported_Constansts_Group4 MMC Memory Cards
  * @{
  */
#define  MMC_LOW_CAPACITY_CARD                0x00000000U    /*!< MMC Card Capacity <=2Gbytes   */
#define  MMC_HIGH_CAPACITY_CARD               0x00000001U    /*!< MMC Card Capacity >2Gbytes and <2Tbytes   */

/**
  * @}
  */
      
/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup MMC_Exported_macros MMC Exported Macros
 *  @brief macros to handle interrupts and specific clock configurations
 * @{
 */
/** @brief Reset MMC handle state.
  * @param  __HANDLE__ : MMC handle.
  * @retval None
  */
#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
#define __DAL_MMC_RESET_HANDLE_STATE(__HANDLE__)           do {                                              \
                                                               (__HANDLE__)->State = DAL_MMC_STATE_RESET; \
                                                               (__HANDLE__)->MspInitCallback = NULL;       \
                                                               (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                             } while(0)
#else
#define __DAL_MMC_RESET_HANDLE_STATE(__HANDLE__)           ((__HANDLE__)->State = DAL_MMC_STATE_RESET)
#endif
 
/**
  * @brief  Enable the MMC device.
  * @retval None
  */ 
#define __DAL_MMC_ENABLE(__HANDLE__) __SDIO_ENABLE((__HANDLE__)->Instance)

/**
  * @brief  Disable the MMC device.
  * @retval None
  */
#define __DAL_MMC_DISABLE(__HANDLE__) __SDIO_DISABLE((__HANDLE__)->Instance)

/**
  * @brief  Enable the SDMMC DMA transfer.
  * @retval None
  */ 
#define __DAL_MMC_DMA_ENABLE(__HANDLE__) __SDIO_DMA_ENABLE((__HANDLE__)->Instance)

/**
  * @brief  Disable the SDMMC DMA transfer.
  * @retval None
  */
#define __DAL_MMC_DMA_DISABLE(__HANDLE__)  __SDIO_DMA_DISABLE((__HANDLE__)->Instance)
 
/**
  * @brief  Enable the MMC device interrupt.
  * @param  __HANDLE__: MMC Handle  
  * @param  __INTERRUPT__: specifies the SDMMC interrupt sources to be enabled.
  *         This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, DATACOUNT, is zero) interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval None
  */
#define __DAL_MMC_ENABLE_IT(__HANDLE__, __INTERRUPT__) __SDIO_ENABLE_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @brief  Disable the MMC device interrupt.
  * @param  __HANDLE__: MMC Handle   
  * @param  __INTERRUPT__: specifies the SDMMC interrupt sources to be disabled.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, DATACOUNT, is zero) interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt   
  * @retval None
  */
#define __DAL_MMC_DISABLE_IT(__HANDLE__, __INTERRUPT__) __SDIO_DISABLE_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @brief  Check whether the specified MMC flag is set or not. 
  * @param  __HANDLE__: MMC Handle   
  * @param  __FLAG__: specifies the flag to check. 
  *          This parameter can be one of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, DATACOUNT, is zero)
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_CMDACT:   Command transfer in progress
  *            @arg SDIO_FLAG_TXACT:    Data transmit in progress
  *            @arg SDIO_FLAG_RXACT:    Data receive in progress
  *            @arg SDIO_FLAG_TXFIFOHE: Transmit FIFO Half Empty
  *            @arg SDIO_FLAG_RXFIFOHF: Receive FIFO Half Full
  *            @arg SDIO_FLAG_TXFIFOF:  Transmit FIFO full
  *            @arg SDIO_FLAG_RXFIFOF:  Receive FIFO full
  *            @arg SDIO_FLAG_TXFIFOE:  Transmit FIFO empty
  *            @arg SDIO_FLAG_RXFIFOE:  Receive FIFO empty
  *            @arg SDIO_FLAG_TXDAVL:   Data available in transmit FIFO
  *            @arg SDIO_FLAG_RXDAVL:   Data available in receive FIFO
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  * @retval The new state of MMC FLAG (SET or RESET).
  */
#define __DAL_MMC_GET_FLAG(__HANDLE__, __FLAG__) __SDIO_GET_FLAG((__HANDLE__)->Instance, (__FLAG__))

/**
  * @brief  Clear the MMC's pending flags.
  * @param  __HANDLE__: MMC Handle  
  * @param  __FLAG__: specifies the flag to clear.  
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, DATACOUNT, is zero)
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  * @retval None
  */
#define __DAL_MMC_CLEAR_FLAG(__HANDLE__, __FLAG__) __SDIO_CLEAR_FLAG((__HANDLE__)->Instance, (__FLAG__))

/**
  * @brief  Check whether the specified MMC interrupt has occurred or not.
  * @param  __HANDLE__: MMC Handle   
  * @param  __INTERRUPT__: specifies the SDMMC interrupt source to check. 
  *          This parameter can be one of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, DATACOUNT, is zero) interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval The new state of MMC IT (SET or RESET).
  */
#define __DAL_MMC_GET_IT(__HANDLE__, __INTERRUPT__) __SDIO_GET_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @brief  Clear the MMC's interrupt pending bits.
  * @param  __HANDLE__: MMC Handle
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear. 
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, DATACOUNT, is zero) interrupt
  *            @arg SDIO_IT_DBCKEND:    Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_TXFIFOHE:   Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF:   Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_RXFIFOF:    Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:    Transmit FIFO empty interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  * @retval None
  */
#define __DAL_MMC_CLEAR_IT(__HANDLE__, __INTERRUPT__) __SDIO_CLEAR_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @}
  */
  
/* Exported functions --------------------------------------------------------*/
/** @defgroup MMC_Exported_Functions MMC Exported Functions
  * @{
  */
  
/** @defgroup MMC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
DAL_StatusTypeDef DAL_MMC_Init(MMC_HandleTypeDef *hmmc);
DAL_StatusTypeDef DAL_MMC_InitCard(MMC_HandleTypeDef *hmmc);
DAL_StatusTypeDef DAL_MMC_DeInit (MMC_HandleTypeDef *hmmc);
void DAL_MMC_MspInit(MMC_HandleTypeDef *hmmc);
void DAL_MMC_MspDeInit(MMC_HandleTypeDef *hmmc);

/**
  * @}
  */
  
/** @defgroup MMC_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_MMC_ReadBlocks(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
DAL_StatusTypeDef DAL_MMC_WriteBlocks(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
DAL_StatusTypeDef DAL_MMC_Erase(MMC_HandleTypeDef *hmmc, uint32_t BlockStartAdd, uint32_t BlockEndAdd);
/* Non-Blocking mode: IT */
DAL_StatusTypeDef DAL_MMC_ReadBlocks_IT(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
DAL_StatusTypeDef DAL_MMC_WriteBlocks_IT(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_MMC_ReadBlocks_DMA(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
DAL_StatusTypeDef DAL_MMC_WriteBlocks_DMA(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

void DAL_MMC_IRQHandler(MMC_HandleTypeDef *hmmc);

/* Callback in non blocking modes (DMA) */
void DAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc);
void DAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc);
void DAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc);
void DAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc);

#if defined (USE_DAL_MMC_REGISTER_CALLBACKS) && (USE_DAL_MMC_REGISTER_CALLBACKS == 1U)
/* MMC callback registering/unregistering */
DAL_StatusTypeDef DAL_MMC_RegisterCallback  (MMC_HandleTypeDef *hmmc, DAL_MMC_CallbackIDTypeDef CallbackId, pMMC_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_MMC_UnRegisterCallback(MMC_HandleTypeDef *hmmc, DAL_MMC_CallbackIDTypeDef CallbackId);
#endif
/**
  * @}
  */
  
/** @defgroup MMC_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
DAL_StatusTypeDef DAL_MMC_ConfigWideBusOperation(MMC_HandleTypeDef *hmmc, uint32_t WideMode);
/**
  * @}
  */

/** @defgroup MMC_Exported_Functions_Group4 MMC card related functions
  * @{
  */
DAL_MMC_CardStateTypeDef DAL_MMC_GetCardState(MMC_HandleTypeDef *hmmc);
DAL_StatusTypeDef DAL_MMC_GetCardCID(MMC_HandleTypeDef *hmmc, DAL_MMC_CardCIDTypeDef *pCID);
DAL_StatusTypeDef DAL_MMC_GetCardCSD(MMC_HandleTypeDef *hmmc, DAL_MMC_CardCSDTypeDef *pCSD);
DAL_StatusTypeDef DAL_MMC_GetCardInfo(MMC_HandleTypeDef *hmmc, DAL_MMC_CardInfoTypeDef *pCardInfo);
DAL_StatusTypeDef DAL_MMC_GetCardExtCSD(MMC_HandleTypeDef *hmmc, uint32_t *pExtCSD, uint32_t Timeout);
/**
  * @}
  */

/** @defgroup MMC_Exported_Functions_Group5 Peripheral State and Errors functions
  * @{
  */
DAL_MMC_StateTypeDef DAL_MMC_GetState(MMC_HandleTypeDef *hmmc);
uint32_t DAL_MMC_GetError(MMC_HandleTypeDef *hmmc);
/**
  * @}
  */

/** @defgroup MMC_Exported_Functions_Group6 Peripheral Abort management
  * @{
  */
DAL_StatusTypeDef DAL_MMC_Abort(MMC_HandleTypeDef *hmmc);
DAL_StatusTypeDef DAL_MMC_Abort_IT(MMC_HandleTypeDef *hmmc);
/**
  * @}
  */
    
/* Private types -------------------------------------------------------------*/
/** @defgroup MMC_Private_Types MMC Private Types
  * @{
  */

/**
  * @}
  */ 

/* Private defines -----------------------------------------------------------*/
/** @defgroup MMC_Private_Defines MMC Private Defines
  * @{
  */

/**
  * @}
  */ 
          
/* Private variables ---------------------------------------------------------*/
/** @defgroup MMC_Private_Variables MMC Private Variables
  * @{
  */

/**
  * @}
  */ 

/* Private constants ---------------------------------------------------------*/
/** @defgroup MMC_Private_Constants MMC Private Constants
  * @{
  */

/**
  * @}
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup MMC_Private_Macros MMC Private Macros
  * @{
  */

/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup MMC_Private_Functions_Prototypes MMC Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup MMC_Private_Functions MMC Private Functions
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* SDIO */

#endif /* APM32F4xx_DAL_MMC_H */ 
