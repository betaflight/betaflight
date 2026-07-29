 /**
  ******************************************************************************
  * @file     um324xx_hal_exflash.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-26  
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
#ifndef __GT25QXX_H__
#define __GT25QXX_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @addtogroup GT25QXX
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup GT25QXX_Exported_typedefs GT25QXX Exported Typedefs
  * @{
  */ 

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup GT25QXX_Exported_constants GT25QXX Exported Constants
  * @{
  */ 

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GT25QXX_Exported_macro GT25QXX Exported Macro
  * @{
  */ 
//gt25qxx FLASH CMD
#define CMD_READ				0x03			// normal read 
#define CMD_FAST_READ			0x0B			// fast read 
#define CMD_2READ				0xBB			// 2x I/O read 
#define CMD_DREAD				0x3B			// dual read 
#define CMD_4READ				0xEB			// 4x I/O read
#define CMD_QREAD				0x6B			// quad read
#define CMD_PP					0x02			// page program
#define CMD_4PP					0x32			// Quad Input Page Program
#define CMD_SE					0x20			// sector erase
#define CMD_BE_32K				0x52			// block erase 32KB
#define CMD_BE_64K				0xD8			// block erase 64KB
#define CMD_CE					0x60			// chip erase
#define CMD_PAGE_ERASE          0x81            // page erase
#define CMD_WREN				0x06			// write enable
#define CMD_WRDI				0x04			// write disable
#define CMD_WSR_EN	            0x50            // Volatile SR Write Enable 
#define CMD_WRSR				0x01			// write status reg1 
#define CMD_WRSR2				0x31			// write status reg2
#define CMD_WRSR3				0x11			// write status reg3 
#define CMD_RDSR				0x05			// read status reg1 
#define CMD_RDSR2				0x35			// read status reg2 
#define CMD_RDSR3			    0x15			// read status reg3 
#define CMD_ESR      	  	    0x44            // Erase Security Register 
#define CMD_PSR       	 	    0x42            // Program Security Register 
#define CMD_RSR            	    0x48            // Read Security Register 
#define CMD_GBL          	    0x7E            // Global Block Lock 
#define CMD_GBUL           		0x98            // Global Block Unlock 
#define CMD_RBL           	    0x3D            // Read Block Lock 
#define CMD_IBL          	    0x36            // Individual Block Lock 
#define CMD_IBUL          		0x39            // Individual Block Unlock 
#define CMD_RDCR				0x15			// read status reg 
#define CMD_WPSEL               0x68            // write protect selection 
#define CMD_EQIO				0x35			// enter QPI mode 
#define CMD_RSTQIO				0xF5			// exit QPI mode 
#define CMD_ERS                 0xB0            // suspends program 
#define CMD_PGM                 0x30            // resumes program 
#define CMD_DP                  0xB9            // deep power down 
#define CMD_RDP                 0xAB            // release from deep power down 
#define CMD_SBL                 0xC0            // set burst length 
#define CMD_RDFBR               0x16            // read fast boot register 
#define CMD_WRFBR               0x17            // write fast boot register 
#define CMD_ESFBR               0x18            // erase fast boot register 
#define FLASH_ID                0x9F            // read identification 
#define CMD_RES                 0xAB            // read electronic ID 
#define READ_DEVICE_ID          0x90            // read electronic manufacturer ID 
#define READ_QDEVICE_ID         0x94            // read electronic manufacturer ID 
#define CMD_UNIQUEID	        0x4B            // Read Unique ID 
#define CMD_QPIID               0xAF            // QPI ID read 
#define CMD_RDSFDP              0x5A            // read sfdp mode 
#define CMD_ENSO                0xB1            // enter secured OTP 
#define CMD_EXSO                0xC1            // exit secured OTP  
#define CMD_RDSCUR              0x2B            // read security register 
#define CMD_WRSCUR              0x2F            // write security register 
#define CMD_GBLK                0x7E            // whole chip write protect 
#define CMD_GBULK               0x98            // whole chip unlock 
#define CMD_WRLR                0x2C            // write lock register 
#define CMD_RDLR                0x2D            // read lock register 
#define CMD_WRPASS              0x28            // write password register 
#define CMD_RDPASS              0x27            // read password register 
#define CMD_PASSULK             0x29            // password unlock
#define CMD_WRSPB               0xE3            // spb bit program 
#define CMD_ESSPB               0xE4            // all spb bit erase 
#define CMD_RDSPB               0xE2            // read spb status 
#define CMD_SPBLK               0xA6            // spb lock set 
#define CMD_RDSPBLK             0xA7            // spb lock register read 
#define CMD_WRDPB               0xE1            // write dpb register 
#define CMD_RDDPB               0xE0            // read dpb register 
#define CMD_NOP                 0x00            // no operation 
#define CMD_RSTEN				0x66			// reset enable 
#define CMD_RST					0x99			// reset memory

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GT25QXX_Exported_Functions
  * @{
  */ 
void        QSPI_WriteEnable       (QSPI_HandleTypeDef *hqspi);
void        QSPI_QuadEn            (QSPI_HandleTypeDef *hqspi);
void        QSPI_QuadDisable       (QSPI_HandleTypeDef *hqspi);
void        QSPI_WriteSr           (QSPI_HandleTypeDef *hqspi,uint8_t sr);
uint8_t     QSPI_ReadSr            (QSPI_HandleTypeDef *hqspi);
void        QSPI_WaitUntilIdle     (QSPI_HandleTypeDef *hqspi);
uint32_t    QSPI_ReadDeviceId      (QSPI_HandleTypeDef *hqspi);
uint32_t    QSPI_ReadDeviceId_9F   (QSPI_HandleTypeDef *hqspi);
void	      QSPI_ReadUniqueId      (QSPI_HandleTypeDef *hqspi, uint32_t arr[2]);
void        QSPI_EraseSector       (QSPI_HandleTypeDef *hqspi,uint32_t addr);
void        QSPI_PageWrite         (QSPI_HandleTypeDef *hqspi,uint8_t* buf,uint32_t write_addr,uint16_t len);
void        QSPI_PageRead          (QSPI_HandleTypeDef *hqspi,uint8_t* buf,uint32_t read_addr,uint16_t len);
/* Private macros ------------------------------------------------------------*/
/** @defgroup GT25QXX_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup GT25QXX_Private_Functions GT25QXX Private Functions
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
