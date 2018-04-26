/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Original author: Alain (https://github.com/aroyer-qc)
 * Modified for F4 and BF source: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#ifndef __fatfs_sd_sdio_H__
#define __fatfs_sd_sdio_H__

#include <stdint.h>

#include "platform.h"
#ifdef STM32F4
#include "stm32f4xx.h"
#endif

#ifdef STM32F7
#include "stm32f7xx.h"
#endif

 /* SDCARD pinouts
 *
 * SD CARD PINS
   _________________
  / 1 2 3 4 5 6 7 8 |  NR   |SDIO INTERFACE
 /                  |       |NAME     STM32F746     DESCRIPTION
/ 9                 |       |         4-BIT  1-BIT
|                   |       |
|                   |   1   |CD/DAT3  PC11   -      Connector data line 3
|                   |   2   |CMD      PD2    PD2    Command/Response line
|                   |   3   |VSS1     GND    GND    GND
|   SD CARD Pinout  |   4   |VDD      3.3V   3.3V   3.3V Power supply
|                   |   5   |CLK      PC12   PC12   Clock
|                   |   6   |VSS2     GND    GND    GND
|                   |   7   |DAT0     PC8    PC8    Connector data line 0
|                   |   8   |DAT1     PC9    -      Connector data line 1
|___________________|   9   |DAT2     PC10   -      Connector data line 2

 */

/* Define(s) --------------------------------------------------------------------------------------------------------*/

//#define MSD_OK                        		    ((uint8_t)0x00)
#define MSD_ERROR                     		    ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      		    ((uint8_t)0x02)

#define SD_PRESENT               				((uint8_t)0x01)
#define SD_NOT_PRESENT           				((uint8_t)0x00)

#define SD_DATATIMEOUT           				((uint32_t)100000000)

#define SD_DETECT_GPIO_PORT                   	GPIOC
#define SD_DETECT_PIN             				GPIO_PIN_13

/* Structure(s) -----------------------------------------------------------------------------------------------------*/

typedef enum
{
  // SD specific error defines
    SD_CMD_CRC_FAIL                    = (1),   // Command response received (but CRC check failed)
    SD_DATA_CRC_FAIL                   = (2),   // Data block sent/received (CRC check failed)
    SD_CMD_RSP_TIMEOUT                 = (3),   // Command response TimeOut
    SD_DATA_TIMEOUT                    = (4),   // Data TimeOut
    SD_TX_UNDERRUN                     = (5),   // Transmit FIFO underrun
    SD_RX_OVERRUN                      = (6),   // Receive FIFO overrun
    SD_START_BIT_ERR                   = (7),   // Start bit not detected on all data signals in wide bus mode
    SD_CMD_OUT_OF_RANGE                = (8),   // Command's argument was out of range.
    SD_ADDR_MISALIGNED                 = (9),   // Misaligned address
    SD_BLOCK_LEN_ERR                   = (10),  // Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length
    SD_ERASE_SEQ_ERR                   = (11),  // An error in the sequence of erase command occurs.
    SD_BAD_ERASE_PARAM                 = (12),  // An invalid selection for erase groups
    SD_WRITE_PROT_VIOLATION            = (13),  // Attempt to program a write protect block
    SD_LOCK_UNLOCK_FAILED              = (14),  // Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card
    SD_COM_CRC_FAILED                  = (15),  // CRC check of the previous command failed
    SD_ILLEGAL_CMD                     = (16),  // Command is not legal for the card state
    SD_CARD_ECC_FAILED                 = (17),  // Card internal ECC was applied but failed to correct the data
    SD_CC_ERROR                        = (18),  // Internal card controller error
    SD_GENERAL_UNKNOWN_ERROR           = (19),  // General or unknown error
    SD_STREAM_READ_UNDERRUN            = (20),  // The card could not sustain data transfer in stream read operation.
    SD_STREAM_WRITE_OVERRUN            = (21),  // The card could not sustain data programming in stream mode
    SD_CID_CSD_OVERWRITE               = (22),  // CID/CSD overwrite error
    SD_WP_ERASE_SKIP                   = (23),  // Only partial address space was erased
    SD_CARD_ECC_DISABLED               = (24),  // Command has been executed without using internal ECC
    SD_ERASE_RESET                     = (25),  // Erase sequence was cleared before executing because an out of erase sequence command was received
    SD_AKE_SEQ_ERROR                   = (26),  // Error in sequence of authentication.
    SD_INVALID_VOLTRANGE               = (27),
    SD_ADDR_OUT_OF_RANGE               = (28),
    SD_SWITCH_ERROR                    = (29),
    SD_SDMMC_DISABLED                  = (30),
    SD_SDMMC_FUNCTION_BUSY             = (31),
    SD_SDMMC_FUNCTION_FAILED           = (32),
    SD_SDMMC_UNKNOWN_FUNCTION          = (33),
    SD_OUT_OF_BOUND                    = (34),


    // Standard error defines
    SD_INTERNAL_ERROR                  = (35),
    SD_NOT_CONFIGURED                  = (36),
    SD_REQUEST_PENDING                 = (37),
    SD_REQUEST_NOT_APPLICABLE          = (38),
    SD_INVALID_PARAMETER               = (39),
    SD_UNSUPPORTED_FEATURE             = (40),
    SD_UNSUPPORTED_HW                  = (41),
    SD_ERROR                           = (42),
    SD_BUSY                            = (43),
    SD_OK                              = (0)
} SD_Error_t;


typedef struct
{
    uint8_t  DAT_BUS_WIDTH;           // Shows the currently defined data bus width
    uint8_t  SECURED_MODE;            // Card is in secured mode of operation
    uint16_t SD_CARD_TYPE;            // Carries information about card type
    uint32_t SIZE_OF_PROTECTED_AREA;  // Carries information about the capacity of protected area
    uint8_t  SPEED_CLASS;             // Carries information about the speed class of the card
    uint8_t  PERFORMANCE_MOVE;        // Carries information about the card's performance move
    uint8_t  AU_SIZE;                 // Carries information about the card's allocation unit size
    uint16_t ERASE_SIZE;              // Determines the number of AUs to be erased in one operation
    uint8_t  ERASE_TIMEOUT;           // Determines the TimeOut for any number of AU erase
    uint8_t  ERASE_OFFSET;            // Carries information about the erase offset
} SD_CardStatus_t;

typedef struct
{
    uint8_t  CSDStruct;                 // CSD structure
    uint8_t  SysSpecVersion;            // System specification version
    uint8_t  Reserved1;                 // Reserved
    uint8_t  TAAC;                      // Data read access time 1
    uint8_t  NSAC;                      // Data read access time 2 in CLK cycles
    uint8_t  MaxBusClkFrec;             // Max. bus clock frequency
    uint16_t CardComdClasses;           // Card command classes
    uint8_t  RdBlockLen;                // Max. read data block length
    uint8_t  PartBlockRead;             // Partial blocks for read allowed
    uint8_t  WrBlockMisalign;           // Write block misalignment
    uint8_t  RdBlockMisalign;           // Read block misalignment
    uint8_t  DSRImpl;                   // DSR implemented
    uint8_t  Reserved2;                 // Reserved
    uint32_t DeviceSize;                // Device Size
    uint8_t  MaxRdCurrentVDDMin;        // Max. read current @ VDD min
    uint8_t  MaxRdCurrentVDDMax;        // Max. read current @ VDD max
    uint8_t  MaxWrCurrentVDDMin;        // Max. write current @ VDD min
    uint8_t  MaxWrCurrentVDDMax;        // Max. write current @ VDD max
    uint8_t  DeviceSizeMul;             // Device size multiplier
    uint8_t  EraseGrSize;               // Erase group size
    uint8_t  EraseGrMul;                // Erase group size multiplier
    uint8_t  WrProtectGrSize;           // Write protect group size
    uint8_t  WrProtectGrEnable;         // Write protect group enable
    uint8_t  ManDeflECC;                // Manufacturer default ECC
    uint8_t  WrSpeedFact;               // Write speed factor
    uint8_t  MaxWrBlockLen;             // Max. write data block length
    uint8_t  WriteBlockPaPartial;       // Partial blocks for write allowed
    uint8_t  Reserved3;                 // Reserved
    uint8_t  ContentProtectAppli;       // Content protection application
    uint8_t  FileFormatGrouop;          // File format group
    uint8_t  CopyFlag;                  // Copy flag (OTP)
    uint8_t  PermWrProtect;             // Permanent write protection
    uint8_t  TempWrProtect;             // Temporary write protection
    uint8_t  FileFormat;                // File format
    uint8_t  ECC;                       // ECC code
    uint8_t  CSD_CRC;                   // CSD CRC
    uint8_t  Reserved4;                 // Always 1
} SD_CSD_t;

typedef struct
{
    uint8_t  ManufacturerID;            // Manufacturer ID
    uint16_t OEM_AppliID;               // OEM/Application ID
    uint32_t ProdName1;                 // Product Name part1
    uint8_t  ProdName2;                 // Product Name part2
    uint8_t  ProdRev;                   // Product Revision
    uint32_t ProdSN;                    // Product Serial Number
    uint8_t  Reserved1;                 // Reserved1
    uint16_t ManufactDate;              // Manufacturing Date
    uint8_t  CID_CRC;                   // CID CRC
    uint8_t  Reserved2;                 // Always 1

} SD_CID_t;

typedef enum
{
    SD_STD_CAPACITY_V1_1       = 0,
    SD_STD_CAPACITY_V2_0       = 1,
    SD_HIGH_CAPACITY           = 2,
    SD_MULTIMEDIA              = 3,
    SD_SECURE_DIGITAL_IO       = 4,
    SD_HIGH_SPEED_MULTIMEDIA   = 5,
    SD_SECURE_DIGITAL_IO_COMBO = 6,
    SD_HIGH_CAPACITY_MMC       = 7,
} SD_CardType_t;

typedef struct
{
  volatile SD_CSD_t    SD_csd;          // SD card specific data register
  volatile SD_CID_t    SD_cid;          // SD card identification number register
  uint64_t             CardCapacity;    // Card capacity
  uint32_t             CardBlockSize;   // Card block size
} SD_CardInfo_t;

/* Prototype(s) -----------------------------------------------------------------------------------------------------*/

extern           SD_CardInfo_t               SD_CardInfo;
extern           SD_CardType_t               SD_CardType;

void             SD_Initialize_LL            (DMA_Stream_TypeDef *dma);
bool             SD_Init                     (void);
bool             SD_IsDetected				(void);
bool             SD_GetState                 (void);
SD_Error_t       SD_GetCardInfo              (void);

SD_Error_t       SD_ReadBlocks_DMA           (uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks);
SD_Error_t       SD_CheckRead                (void);
SD_Error_t       SD_WriteBlocks_DMA          (uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks);
SD_Error_t       SD_CheckWrite               (void);

SD_Error_t       SD_Erase                    (uint64_t StartAddress, uint64_t EndAddress);
SD_Error_t       SD_GetCardStatus            (SD_CardStatus_t* pCardStatus);

/* ------------------------------------------------------------------------------------------------------------------*/

#endif // __fatfs_sd_sdio_H__
