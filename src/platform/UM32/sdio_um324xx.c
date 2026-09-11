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
 * UM324xx SDIO driver based on Synopsys DesignWare Mobile Storage Host Controller (DWC_mshc).
 *
 * The UM324xx SDIO peripheral is a completely different architecture from STM32F4 SDIO.
 * It uses an internal DMA (IDMA) with descriptor chains instead of external DMA streams.
 *
 * Reference: UM324xF Datasheet, Chapter 33 - SDIO
 *
 * Adapted from STM32F4 sdio_f4xx.c for BF source.
 */

/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "drivers/sdmmc_sdio.h"
#include "um324xx_hal_gpio.h"

#include "pg/sdio.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"
#include "drivers/light_led.h"

#include "build/debug.h"


/* Define(s) --------------------------------------------------------------------------------------------------------*/

#define BLOCK_SIZE                      ((uint32_t)(512))

// SDIO clock divider register offset within SDIO peripheral
// CLKDIV = AHB_CLK / (2 * n) where n = clk_div0
// clk_div0=0 => no clock output
#define SDIO_INIT_CLK_DIV               ((uint8_t)0x50)     // ~400kHz for card init
#define SDIO_TRANSFER_CLK_DIV           ((uint8_t)0x01)     // ~3MHz transfer clock (0x01=24MHz is too fast, causes CMD timeouts on read)

#define SD_SOFTWARE_COMMAND_TIMEOUT     ((uint32_t)0x2000000)

#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRITE        ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_RESP_HIGH_CAPACITY           ((uint32_t)0x40000000)
#define SD_RESP_STD_CAPACITY            ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)

#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_SDIO_SEND_IF_COND           ((uint32_t)SD_CMD_HS_SEND_EXT_CSD)

// Bus width definitions for UM324xx SDIO WIDTH register
#define SD_BUS_WIDE_1B                  ((uint32_t)0x00000000)    // WIDTH0=0
#define SD_BUS_WIDE_4B                  ((uint32_t)0x00000001)    // WIDTH0=1
#define SD_BUS_WIDE_8B                  ((uint32_t)0x00010001)    // WIDTH1=1, WIDTH0=1

// Response types
#define SD_CMD_RESPONSE_NO              ((uint32_t)0x00000000)
#define SD_CMD_RESPONSE_SHORT           SDIO_CMD_REP_EXPECT
#define SD_CMD_RESPONSE_LONG            (SDIO_CMD_REP_EXPECT | SDIO_CMD_REP_LONG)

// RINTSTS interrupt flags used
#define SDIO_RINTSTS_CMD_ERROR_FLAGS    (SDIO_RINTSTS_HARDLOCK_WRITE_ERROR | SDIO_RINTSTS_FINISH_BIT_ERROR | \
                                         SDIO_RINTSTS_START_BIT_ERROR_BUSY | SDIO_RINTSTS_ACK_TIMEOUT | \
                                         SDIO_RINTSTS_ACK_CRC_ERROR | SDIO_RINTSTS_ACK_ERROR)
#define SDIO_RINTSTS_DATA_ERROR_FLAGS   (SDIO_RINTSTS_DATA_CRC_ERROR | SDIO_RINTSTS_READ_DATA_TIMEOUT | \
                                         SDIO_RINTSTS_DATA_LOSS_TIMEOUT | SDIO_RINTSTS_FIFO_UNDER_OVER_RUN)

// IDMA descriptor flags (DES0)
#define IDMA_DES0_OWN                   ((uint32_t)0x80000000)    // Descriptor owned by DMA
#define IDMA_DES0_CES                   ((uint32_t)0x40000000)    // Card Error Summary (stop on error)
#define IDMA_DES0_ER                    ((uint32_t)0x00000020)    // End of Ring
#define IDMA_DES0_CH                    ((uint32_t)0x00000010)    // Second Address Chained
#define IDMA_DES0_FS                    ((uint32_t)0x00000008)    // First Descriptor
#define IDMA_DES0_LD                    ((uint32_t)0x00000004)    // Last Descriptor
#define IDMA_DES0_DIC                   ((uint32_t)0x00000002)    // Disable Interrupt on Completion

#define SDIO_DIR_TX                     1
#define SDIO_DIR_RX                     0


/* Typedef(s) -------------------------------------------------------------------------------------------------------*/

typedef enum
{
    SD_SINGLE_BLOCK    = 0,             // Single block operation
    SD_MULTIPLE_BLOCK  = 1,             // Multiple blocks operation
} SD_Operation_t;


typedef struct
{
    uint32_t          CSD[4];           // SD card specific data table
    uint32_t          CID[4];           // SD card identification number table
    volatile uint32_t TransferComplete; // SD transfer complete flag in non blocking mode
    volatile uint32_t TransferError;    // SD transfer error flag in non blocking mode
    volatile uint32_t RXCplt;          // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;          // SD TX Complete is equal 0 when no transfer
    volatile uint32_t Operation;        // SD transfer operation (read/write)
} SD_Handle_t;

typedef enum
{
    SD_CARD_READY                  = ((uint32_t)0x00000001),  // Card state is ready
    SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002),  // Card is in identification state
    SD_CARD_STANDBY                = ((uint32_t)0x00000003),  // Card is in standby state
    SD_CARD_TRANSFER               = ((uint32_t)0x00000004),  // Card is in transfer state
    SD_CARD_SENDING                = ((uint32_t)0x00000005),  // Card is sending an operation
    SD_CARD_RECEIVING              = ((uint32_t)0x00000006),  // Card is receiving operation information
    SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007),  // Card is in programming state
    SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008),  // Card is disconnected
    SD_CARD_ERROR                  = ((uint32_t)0x000000FF)   // Card is in error state
} SD_CardState_t;

// IDMA Descriptor structure (Synopsys DesignWare format)
typedef struct
{
    uint32_t DES0;  // OWN(31) | CES(30) | RSV(29:6) | ER(5) | CH(4) | FS(3) | LD(2) | DIC(1) | RSV(0)
    uint32_t DES1;  // RSV(31:26) | BS2(25:13) | BS1(12:0) — Buffer sizes
    uint32_t DES2;  // Buffer Address Pointer 1
    uint32_t DES3;  // Buffer Address Pointer 2 / Next Descriptor Address
} SDIO_IDMA_Descriptor_t;

/* Variable(s) ------------------------------------------------------------------------------------------------------*/

static SD_Handle_t                 SD_Handle;
SD_CardInfo_t                      SD_CardInfo;
static uint32_t                    SD_Status;
static uint32_t                    SD_CardRCA;
SD_CardType_t                      SD_CardType;
static volatile uint32_t           TimeOut;

// IDMA descriptor in DMA-capable memory
DMA_DATA SDIO_IDMA_Descriptor_t    SDIO_DMADescriptorTx;
DMA_DATA SDIO_IDMA_Descriptor_t    SDIO_DMADescriptorRx;

/* SD Commands -------------------------------------------------------------------------------------------------------*/

#define SD_CMD_GO_IDLE_STATE            ((uint8_t)0)   // Resets the SD memory card.
#define SD_CMD_SEND_OP_COND             ((uint8_t)1)   // Sends host capacity support information
#define SD_CMD_ALL_SEND_CID             ((uint8_t)2)   // Asks any card to send CID numbers on CMD line.
#define SD_CMD_SET_REL_ADDR             ((uint8_t)3)   // Asks the card to publish a new relative address (RCA).
#define SD_CMD_HS_SWITCH                ((uint8_t)6)   // Checks switchable function / switch card function.
#define SD_CMD_SEL_DESEL_CARD           ((uint8_t)7)   // Selects/deselects the card by its RCA.
#define SD_CMD_HS_SEND_EXT_CSD          ((uint8_t)8)   // Sends SD Memory Card interface condition.
#define SD_CMD_SEND_CSD                 ((uint8_t)9)   // Addressed card sends its CSD on the CMD line.
#define SD_CMD_SEND_CID                 ((uint8_t)10)  // Addressed card sends its CID on the CMD line.
#define SD_CMD_STOP_TRANSMISSION        ((uint8_t)12)  // Forces the card to stop transmission.
#define SD_CMD_SEND_STATUS              ((uint8_t)13)  // Addressed card sends its status register.
#define SD_CMD_SET_BLOCKLEN             ((uint8_t)16)  // Sets the block length for all following block commands.
#define SD_CMD_READ_SINGLE_BLOCK        ((uint8_t)17)  // Reads single block.
#define SD_CMD_READ_MULT_BLOCK          ((uint8_t)18)  // Continuously transfers data blocks from card to host.
#define SD_CMD_WRITE_SINGLE_BLOCK       ((uint8_t)24)  // Writes single block.
#define SD_CMD_WRITE_MULT_BLOCK         ((uint8_t)25)  // Continuously writes blocks of data.
#define SD_CMD_SD_ERASE_GRP_START       ((uint8_t)32)  // Sets address of first write block to be erased.
#define SD_CMD_SD_ERASE_GRP_END         ((uint8_t)33)  // Sets address of last write block to be erased.
#define SD_CMD_ERASE                    ((uint8_t)38)  // Erases all previously selected write blocks.
#define SD_CMD_APP_CMD                  ((uint8_t)55)  // Indicates next command is an application specific command.

/* SD Card Specific commands (preceded by APP_CMD) */
#define SD_CMD_APP_SD_SET_BUSWIDTH      ((uint8_t)6)   // (ACMD6) Defines the data bus width.
#define SD_CMD_SD_APP_STATUS            ((uint8_t)13)  // (ACMD13) Sends the SD status.
#define SD_CMD_SD_APP_OP_COND           ((uint8_t)41)  // (ACMD41) Sends host capacity support information.
#define SD_CMD_SD_APP_SEND_SCR          ((uint8_t)51)  // Reads the SD Configuration Register (SCR).


/* Private function(s) ----------------------------------------------------------------------------------------------*/

static void             SD_DataTransferInit         (uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard);
static SD_Error_t       SD_TransmitCommand          (uint32_t Command, uint32_t Argument, int8_t ResponseType);
static SD_Error_t       SD_CmdResponse              (uint8_t SD_CMD, int8_t ResponseType);
static void             SD_GetResponse              (uint32_t* pResponse);
static SD_Error_t       CheckOCR_Response           (uint32_t Response_R1);
static SD_Error_t       SD_InitializeCard           (void);
static SD_Error_t       SD_PowerON                  (void);
static SD_Error_t       SD_WideBusOperationConfig   (uint32_t WideMode);
static SD_Error_t       SD_FindSCR                  (uint32_t *pSCR);
static SD_Error_t       SD_DoInit                   (void);
static void             SD_StartBlockTransfer       (uint32_t* pBuffer, uint32_t BlockSize, uint32_t NumberOfBlocks, uint8_t dir);


/** -----------------------------------------------------------------------------------------------------------------*/
/**     DataTransferInit
  *
  * @brief  Set up data transfer parameters for non-DMA (PIO) transfers (SCR read, CMD6, etc.)
  * @param  Size: Total data size in bytes
  * @param  DataBlockSize: Block size
  * @param  IsItReadFromCard: true for read, false for write
  */
static void SD_DataTransferInit(uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard)
{
    UNUSED(IsItReadFromCard);

    SDIO->CTRL &= ~SDIO_CTRL_USE_INTERNAL_DMAC;
    SDIO->BMOD &= ~SDIO_BMOD_DE;
    SDIO->CTRL |= SDIO_CTRL_FIFO_RST;
    SDIO->CTRL &= ~SDIO_CTRL_FIFO_RST;

    SDIO->TIMEOUT = 0xFFFFFF40;      // Set the SDIO Data TimeOut value
    SDIO->BLKSIZ  = DataBlockSize;        // Set block size
    SDIO->BYTCNT  = Size;                 // Set byte count
    return;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**     SD_TransmitCommand
  *
  * @brief  Send the command to SDIO
  * @param  uint32_t Command (with response type bits)
  * @param  uint32_t Argument
  * @param  uint8_t ResponseType
  * @retval SD Card error state
  */
static SD_Error_t SD_TransmitCommand(uint32_t Command, uint32_t Argument, int8_t ResponseType)
{
    SD_Error_t ErrorState;

    // Clear raw interrupt status flags before sending command
    SDIO->RINTSTS = 0xFFFFFFFF;

    // Set the SDIO Argument value
    SDIO->CMDARG = (uint32_t)Argument;

    // Build command register: index | response bits | START_CMD | USE_HOLD_REG
    uint32_t cmdReg = (Command & 0x3F);          // CMD_INDEX is bits 5:0
    if (Command & SDIO_CMD_REP_EXPECT) {
        cmdReg |= SDIO_CMD_REP_EXPECT;
    }
    if (Command & SDIO_CMD_REP_LONG) {
        cmdReg |= SDIO_CMD_REP_LONG;
    }
    if (Command & SDIO_CMD_CHECK_REP_CRC) {
        cmdReg |= SDIO_CMD_CHECK_REP_CRC;
    }
    if (Command & SDIO_CMD_DATA_TRANSFER_EXPECTED) {
        cmdReg |= SDIO_CMD_DATA_TRANSFER_EXPECTED;
    }
    if (Command & SDIO_CMD_READ_WRITE) {
        cmdReg |= SDIO_CMD_READ_WRITE;
    }
    if (Command & SDIO_CMD_SEND_AUTO_STOP) {
        cmdReg |= SDIO_CMD_SEND_AUTO_STOP;
    }
    if (Command & SDIO_CMD_WAIT_PRV_DATA_FINISH) {
        cmdReg |= SDIO_CMD_WAIT_PRV_DATA_FINISH;
    }
    if (Command & SDIO_CMD_STOP_ABORT_CMD) {
        cmdReg |= SDIO_CMD_STOP_ABORT_CMD;
    }
    if (Command & SDIO_CMD_SEND_INI_SEQ) {
        cmdReg |= SDIO_CMD_SEND_INI_SEQ;
    }

    if((Command & 0x3F) == SD_CMD_GO_IDLE_STATE)
    {
        cmdReg |= SDIO_CMD_SEND_INI_SEQ;
    }

    if((Command & 0x3F) == SD_CMD_SD_APP_OP_COND)
    {
        cmdReg &= ~SDIO_CMD_CHECK_REP_CRC;
    }

    if((Command & 0x3F) == SD_CMD_STOP_TRANSMISSION)
    {
        cmdReg &= ~SDIO_CMD_WAIT_PRV_DATA_FINISH;
    }

    cmdReg |= SDIO_CMD_START_CMD;                // Start the command

    while(SDIO->STATUS & SDIO_STATUS_DATA_BUSY);

    SDIO->CMD = cmdReg;

    if ((Argument == 0) && (ResponseType == 0)) {
        ResponseType = -1;       // Go idle command — no response
    }

    ErrorState  = SD_CmdResponse(Command & SDIO_CMD_CMD_INDEX, ResponseType);

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks for error conditions for any response.
  * @param  SD_CMD: The sent command Index
  * @param  ResponseType: -1: no response, 0: CMD sent only, 1: short response, 2: long response,
  *                        3: OCR response, 6: RCA response, 7: R7 response
  * @retval SD Card error state
  */
static SD_Error_t SD_CmdResponse(uint8_t SD_CMD, int8_t ResponseType)
{
    uint32_t Response_R1;
    uint32_t TimeOutLocal;
    uint32_t Flag;

    if (ResponseType == -1) {
        // For commands with no response (e.g., CMD0), wait for CMD_FINISH
        Flag = SDIO_RINTSTS_CMD_FINISH;
    } else if (ResponseType == 0) {
        Flag = SDIO_RINTSTS_CMD_FINISH;
    } else {
        // For commands with response, check for completion or errors
        Flag = SDIO_RINTSTS_CMD_FINISH | SDIO_RINTSTS_ACK_ERROR |
               SDIO_RINTSTS_ACK_CRC_ERROR | SDIO_RINTSTS_ACK_TIMEOUT;
    }

    TimeOutLocal = SD_SOFTWARE_COMMAND_TIMEOUT;
    do {
        SD_Status = SDIO->RINTSTS;
        if ((SD_Status & (SDIO_RINTSTS_HARDLOCK_WRITE_ERROR | SDIO_RINTSTS_FINISH_BIT_ERROR |
                          SDIO_RINTSTS_START_BIT_ERROR_BUSY))) {
            // Hardware errors
            SDIO->RINTSTS = SDIO_RINTSTS_CMD_ERROR_FLAGS;
            return SD_CMD_CRC_FAIL;
        }
        TimeOutLocal--;
    } while (((SD_Status & Flag) == 0) && (TimeOutLocal > 0));

    if (ResponseType <= 0)
    {
        if (TimeOutLocal == 0) {
            return SD_CMD_RSP_TIMEOUT;
        } else {
            // Clear the CMD_FINISH flag
            SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
            return SD_OK;
        }
    }

    // Check for timeout
    if (SD_Status & SDIO_RINTSTS_ACK_TIMEOUT) {
        SDIO->RINTSTS = SDIO_RINTSTS_ACK_TIMEOUT;
        return SD_CMD_RSP_TIMEOUT;
    }

    if (ResponseType == 3)
    {
        // For ACMD41 (OCR response type R3), no CRC check
        if (TimeOutLocal == 0) {
            return SD_CMD_RSP_TIMEOUT;  // Card is not V2.0 compliant
        } else {
            SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
            return SD_OK;               // Card is SD V2.0 compliant
        }
    }

    // Check for CRC error
    if (SD_Status & SDIO_RINTSTS_ACK_CRC_ERROR) {
        SDIO->RINTSTS = SDIO_RINTSTS_ACK_CRC_ERROR;
        return SD_CMD_CRC_FAIL;
    }

    // For long response (R2), we're done after CRC check
    if (ResponseType == 2) {
        SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
        return SD_OK;
    }

    // Verify the response matches the command we sent
    if (((SDIO->STATUS & SDIO_STATUS_RESPONSE_INDEX) >> 11) != SD_CMD) {
        SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
        return SD_ILLEGAL_CMD;      // Check if response is of desired command
    }

    Response_R1 = SDIO->RESP0;                    // We have received response, retrieve it for analysis

    if (ResponseType == 1)
    {
        SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
        return CheckOCR_Response(Response_R1);
    }
    else if (ResponseType == 6)
    {
        SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
        if ((Response_R1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)) == SD_ALLZERO)
        {
            SD_CardRCA = Response_R1;
        }
        if ((Response_R1 & SD_R6_GENERAL_UNKNOWN_ERROR) == SD_R6_GENERAL_UNKNOWN_ERROR) {
            return SD_GENERAL_UNKNOWN_ERROR;
        }
        if ((Response_R1 & SD_R6_ILLEGAL_CMD) == SD_R6_ILLEGAL_CMD) {
            return SD_ILLEGAL_CMD;
        }
        if ((Response_R1 & SD_R6_COM_CRC_FAILED) == SD_R6_COM_CRC_FAILED) {
            return SD_COM_CRC_FAILED;
        }
    }
    else if (ResponseType == 7)
    {
        SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
        // R7 response — no error check needed beyond CRC/timeout (already checked above)
    }

    SDIO->RINTSTS = SDIO_RINTSTS_CMD_FINISH;
    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Get response data from the SDIO response registers.
  * @param  pResponse: pointer to buffer (4 x uint32_t)
  */
static void SD_GetResponse(uint32_t* pResponse)
{
    pResponse[0] = SDIO->RESP3;
    pResponse[1] = SDIO->RESP2;
    pResponse[2] = SDIO->RESP1;
    pResponse[3] = SDIO->RESP0;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Analyze the OCR response and return the appropriate error code
  * @param  Response_R1: OCR Response code
  * @retval SD Card error state
  */
static SD_Error_t CheckOCR_Response(uint32_t Response_R1)
{
    if ((Response_R1 & SD_OCR_ADDR_OUT_OF_RANGE) == SD_OCR_ADDR_OUT_OF_RANGE) {
        return SD_ADDR_OUT_OF_RANGE;
    }
    if ((Response_R1 & SD_OCR_ADDR_MISALIGNED) == SD_OCR_ADDR_MISALIGNED) {
        return SD_ADDR_MISALIGNED;
    }
    if ((Response_R1 & SD_OCR_BLOCK_LEN_ERR) == SD_OCR_BLOCK_LEN_ERR) {
        return SD_BLOCK_LEN_ERR;
    }
    if ((Response_R1 & SD_OCR_ERASE_SEQ_ERR) == SD_OCR_ERASE_SEQ_ERR) {
        return SD_ERASE_SEQ_ERR;
    }
    if ((Response_R1 & SD_OCR_BAD_ERASE_PARAM) == SD_OCR_BAD_ERASE_PARAM) {
        return SD_BAD_ERASE_PARAM;
    }
    if ((Response_R1 & SD_OCR_WRITE_PROT_VIOLATION) == SD_OCR_WRITE_PROT_VIOLATION) {
        return SD_WRITE_PROT_VIOLATION;
    }
    if ((Response_R1 & SD_OCR_LOCK_UNLOCK_FAILED) == SD_OCR_LOCK_UNLOCK_FAILED) {
        return SD_LOCK_UNLOCK_FAILED;
    }
    if ((Response_R1 & SD_OCR_COM_CRC_FAILED) == SD_OCR_COM_CRC_FAILED) {
        return SD_COM_CRC_FAILED;
    }
    if ((Response_R1 & SD_OCR_ILLEGAL_CMD) == SD_OCR_ILLEGAL_CMD) {
        return SD_ILLEGAL_CMD;
    }
    if ((Response_R1 & SD_OCR_CARD_ECC_FAILED) == SD_OCR_CARD_ECC_FAILED) {
        return SD_CARD_ECC_FAILED;
    }
    if ((Response_R1 & SD_OCR_CC_ERROR) == SD_OCR_CC_ERROR) {
        return SD_CC_ERROR;
    }
    if ((Response_R1 & SD_OCR_GENERAL_UNKNOWN_ERROR) == SD_OCR_GENERAL_UNKNOWN_ERROR) {
        return SD_GENERAL_UNKNOWN_ERROR;
    }
    if ((Response_R1 & SD_OCR_STREAM_READ_UNDERRUN) == SD_OCR_STREAM_READ_UNDERRUN) {
        return SD_STREAM_READ_UNDERRUN;
    }
    if ((Response_R1 & SD_OCR_STREAM_WRITE_OVERRUN) == SD_OCR_STREAM_WRITE_OVERRUN) {
        return SD_STREAM_WRITE_OVERRUN;
    }
    if ((Response_R1 & SD_OCR_CID_CSD_OVERWRITE) == SD_OCR_CID_CSD_OVERWRITE) {
        return SD_CID_CSD_OVERWRITE;
    }
    if ((Response_R1 & SD_OCR_WP_ERASE_SKIP) == SD_OCR_WP_ERASE_SKIP) {
        return SD_WP_ERASE_SKIP;
    }
    if ((Response_R1 & SD_OCR_CARD_ECC_DISABLED) == SD_OCR_CARD_ECC_DISABLED) {
        return SD_CARD_ECC_DISABLED;
    }
    if ((Response_R1 & SD_OCR_ERASE_RESET) == SD_OCR_ERASE_RESET) {
        return SD_ERASE_RESET;
    }
    if ((Response_R1 & SD_OCR_AKE_SEQ_ERROR) == SD_OCR_AKE_SEQ_ERROR) {
        return SD_AKE_SEQ_ERROR;
    }

    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**     SD_InitializeCard
  *
  * @brief  Initialize the SD card: get CID, RCA, and CSD.
  * @retval SD Card error state
  */
static SD_Error_t SD_InitializeCard(void)
{
    SD_Error_t ErrorState = SD_OK;

    if ((SDIO->POWEN & SDIO_POWEN_POWER_ENABLE) != 0) // Power on
    {
        if (SD_CardType != SD_SECURE_DIGITAL_IO)
        {
            // Send CMD2 ALL_SEND_CID
            if ((ErrorState = SD_TransmitCommand((SD_CMD_ALL_SEND_CID | SD_CMD_RESPONSE_LONG), 0, 2)) != SD_OK)
            {
                return ErrorState;
            }

            // Get Card identification number data
            SD_GetResponse(SD_Handle.CID);
        }

        if ((SD_CardType == SD_STD_CAPACITY_V1_1)    || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
           (SD_CardType == SD_SECURE_DIGITAL_IO_COMBO) || (SD_CardType == SD_HIGH_CAPACITY))
        {
            // Send CMD3 SET_REL_ADDR with argument 0 — SD Card publishes its RCA
            if ((ErrorState = SD_TransmitCommand((SD_CMD_SET_REL_ADDR | SD_CMD_RESPONSE_SHORT), 0, 6)) != SD_OK)
            {
                return ErrorState;
            }
        }

        if (SD_CardType != SD_SECURE_DIGITAL_IO)
        {
            // Send CMD9 SEND_CSD with argument as card's RCA
            if ((ErrorState = SD_TransmitCommand((SD_CMD_SEND_CSD | SD_CMD_RESPONSE_LONG), SD_CardRCA, 2)) == SD_OK)
            {
                // Get Card Specific Data
                SD_GetResponse(SD_Handle.CSD);
            }
        }
    }
    else
    {
        ErrorState = SD_REQUEST_NOT_APPLICABLE;
    }

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**     SD_StartBlockTransfer
  *
  * @brief  Set up the IDMA descriptor and prepare for data transfer.
  * @param  pBuffer: Pointer to the buffer for data transfer
  * @param  BlockSize: The SD card Data block size (must be 512 bytes)
  * @param  NumberOfBlocks: Number of blocks to transfer
  * @param  dir: SDIO_DIR_RX (read from card) or SDIO_DIR_TX (write to card)
  */
static void SD_StartBlockTransfer(uint32_t* pBuffer, uint32_t BlockSize, uint32_t NumberOfBlocks, uint8_t dir)
{
    uint32_t totalBytes = BlockSize * NumberOfBlocks;

    // Enable internal DMA in CTRL
    SDIO->CTRL |= SDIO_CTRL_FIFO_RST;                 // Reset FIFO
    SDIO->CTRL |= SDIO_CTRL_USE_INTERNAL_DMAC;
    SDIO->CTRL |= SDIO_CTRL_DMA_RST;

    SDIO->BLKSIZ  = BlockSize;                         // Set block size
    SDIO->BYTCNT  = totalBytes;                        // Set total byte count

    SD_Handle.TransferComplete = 0;                    // Initialize handle flags
    SD_Handle.TransferError    = SD_OK;
    SD_Handle.Operation        = (NumberOfBlocks > 1) ? SD_MULTIPLE_BLOCK : SD_SINGLE_BLOCK;
    SD_Handle.Operation       |= dir << 1;

    // Clear raw interrupts
    SDIO->RINTSTS = 0xFFFFFFFF;

    // Configure FIFO threshold with MULTI_TRANSACTION_SIZE matching PBL
    // PBL=32 bytes (8 beats of 4 bytes) → MTS=010 (8 transactions)
    // FIFOTH: bits 28-30=MTS, bits 16-27=RX_WMARK, bits 0-11=TX_WMARK
    if (dir == SDIO_DIR_RX) {
        // For read: RX_WMARK=0x0F (15 entries), MTS=2 (8 transactions)
        SDIO->FIFOTH = (2 << 28) | (0x07 << 16) | 0x08;
        // Setup IDMA descriptor
        memset(&SDIO_DMADescriptorTx, 0, sizeof(SDIO_IDMA_Descriptor_t));
        SDIO_DMADescriptorTx.DES0 = IDMA_DES0_OWN | IDMA_DES0_ER| IDMA_DES0_FS | IDMA_DES0_LD;           // FS+LD: single descriptor, no chain (CH=0 → don't load DES3)
        SDIO_DMADescriptorTx.DES1 = totalBytes & 0x1FFF;       // BS1 = total bytes
        SDIO_DMADescriptorTx.DES2 = (uint32_t)pBuffer;          // Buffer address

        // Set descriptor base address
        SDIO->DBADDR = (uint32_t)&SDIO_DMADescriptorTx;
    } else {
        // For write: TX_WMARK=0x0F (15 entries of free space), MTS=2 (8 transactions)
        SDIO->FIFOTH = (2 << 28) | (0x07 << 16) | 0x08;
        // Setup IDMA descriptor
        memset(&SDIO_DMADescriptorRx, 0, sizeof(SDIO_IDMA_Descriptor_t));
        SDIO_DMADescriptorRx.DES0 = IDMA_DES0_OWN | IDMA_DES0_ER| IDMA_DES0_FS | IDMA_DES0_LD;           // FS+LD: single descriptor, no chain (CH=0 → don't load DES3)
        SDIO_DMADescriptorRx.DES1 = totalBytes & 0x1FFF;       // BS1 = total bytes
        SDIO_DMADescriptorRx.DES2 = (uint32_t)pBuffer;          // Buffer address
        // Set descriptor base address
        SDIO->DBADDR = (uint32_t)&SDIO_DMADescriptorRx;
    }


    SDIO->BMOD = 0x01; //reset Idma
    // Configure Bus Mode: IDMA enabled, PBL=32 bytes
    SDIO->BMOD = SDIO_BMOD_DE | (0x4 << 2) | (0x1<<1);                // DE=1, PBL=32 bytes (010)

    // Enable IDMA interrupts: Transfer Complete, Receive Complete
    SDIO->IDINTEN = SDIO_IDINTEN_TI | SDIO_IDINTEN_RI | SDIO_IDINTEN_NIS;

    // Enable host interrupts
    SDIO->CTRL |= SDIO_CTRL_INT_EN;

    // Poll Demand — kick-start the IDMA
    SDIO->PLDMND = 0x1;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Reads block(s) from a specified address in a card. Data transfer is managed by IDMA.
  * @note   This API should be followed by the function SD_CheckRead() to check completion.
  * @param  ReadAddress: Address from where data is to be read
  * @param  buffer: Pointer to the buffer that will contain the received data
  * @param  BlockSize: SD card Data block size (must be 512 bytes)
  * @param  NumberOfBlocks: Number of blocks to read
  * @retval SD Card error state
  */
SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState;
    uint32_t   CmdIndex;
    SD_Handle.RXCplt = 1;

    if (SD_CardType != SD_HIGH_CAPACITY)
    {
        ReadAddress *= 512;
    }

    SD_StartBlockTransfer(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_RX);

    // Set Block Size for Card
    ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT | SDIO_CMD_WAIT_PRV_DATA_FINISH | SDIO_CMD_CHECK_REP_CRC), BlockSize, 1);

    // Send CMD18 READ_MULT_BLOCK or CMD17 READ_SINGLE_BLOCK
    uint8_t retries = 10;
    CmdIndex = (NumberOfBlocks > 1) ? SD_CMD_READ_MULT_BLOCK : SD_CMD_READ_SINGLE_BLOCK;

    // Build command: data transfer expected, read direction, wait for previous data finish
    uint32_t cmd = CmdIndex | SD_CMD_RESPONSE_SHORT |
                   SDIO_CMD_DATA_TRANSFER_EXPECTED | SDIO_CMD_CHECK_REP_CRC |
                   SDIO_CMD_WAIT_PRV_DATA_FINISH;
    if (NumberOfBlocks > 1) {
        cmd |= SDIO_CMD_SEND_AUTO_STOP;
    }

    do {
        ErrorState = SD_TransmitCommand(cmd, (uint32_t)ReadAddress, 1);
        retries--;
    } while (ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
        SD_Handle.RXCplt = 0;
    }

    // Update the SD transfer error in SD handle
    SD_Handle.TransferError = ErrorState;

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Writes block(s) to a specified address in a card. Data transfer is managed by IDMA.
  * @note   This API should be followed by the function SD_CheckWrite() to check completion.
  * @param  WriteAddress: Address from where data is to be written
  * @param  buffer: Pointer to the buffer that will contain the data to transmit
  * @param  BlockSize: SD card Data block size (must be 512 bytes)
  * @param  NumberOfBlocks: Number of blocks to write
  * @retval SD Card error state
  */
SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState;
    uint32_t   CmdIndex;
    // if(SD_Handle.TXCplt != 0)
    // {
    //  return ErrorState;
    // }
    SD_Handle.TXCplt = 1;

    if (SD_CardType != SD_HIGH_CAPACITY)
    {
        WriteAddress *= 512;
    }

    // Set Block Size for Card
    ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), BlockSize, 1);

    // For writes, DMA must be set up BEFORE sending the write command.
    // The card expects data as soon as it responds to CMD24/CMD25,
    // so the DMA descriptor must already have OWN=1 (DMA ready).
    SD_StartBlockTransfer(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_TX);

    // Send CMD25 WRITE_MULT_BLOCK or CMD24 WRITE_SINGLE_BLOCK
    uint8_t retries = 10;
    CmdIndex = (NumberOfBlocks > 1) ? SD_CMD_WRITE_MULT_BLOCK : SD_CMD_WRITE_SINGLE_BLOCK;

    // Build command: data transfer expected, write direction (READ_WRITE=0 for write)
    uint32_t cmd = CmdIndex | SD_CMD_RESPONSE_SHORT | SDIO_CMD_CHECK_REP_CRC |
                   SDIO_CMD_DATA_TRANSFER_EXPECTED | SDIO_CMD_READ_WRITE |
                   SDIO_CMD_WAIT_PRV_DATA_FINISH;
    if (NumberOfBlocks > 1) {
        cmd |= SDIO_CMD_SEND_AUTO_STOP;
    }

    do {
        ErrorState = SD_TransmitCommand(cmd, (uint32_t)WriteAddress, 1);
        retries--;
    } while (ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
        SD_Handle.TXCplt = 0;
    }

    // Update the SD transfer error in SD handle
    SD_Handle.TransferError = ErrorState;

    return ErrorState;
}


SD_Error_t SD_CheckWrite(void)
{
    if (SD_Handle.TXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_CheckRead(void)
{
    if (SD_Handle.RXCplt != 0) return SD_BUSY;
    return SD_OK;
}

/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddress: Start byte address
  * @param  EndAddress: End byte address
  * @retval SD Card error state
  */
SD_Error_t SD_Erase(uint64_t StartAddress, uint64_t EndAddress)
{
    SD_Error_t ErrorState;

    if (SD_CardType != SD_HIGH_CAPACITY)
    {
        StartAddress *= 512;
        EndAddress   *= 512;
    }

    // Send CMD32 SD_ERASE_GRP_START with argument as start address
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SD_ERASE_GRP_START | SD_CMD_RESPONSE_SHORT), (uint32_t)StartAddress, 1)) != SD_OK)
    {
        return ErrorState;
    }

    // Send CMD33 SD_ERASE_GRP_END with argument as end address
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SD_ERASE_GRP_END | SD_CMD_RESPONSE_SHORT), (uint32_t)EndAddress, 1)) != SD_OK)
    {
        return ErrorState;
    }

    // Send CMD38 ERASE
    if ((ErrorState = SD_TransmitCommand((SD_CMD_ERASE | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK)
    {
        return ErrorState;
    }

    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the SD card info from CSD and CID registers.
  * @retval SD Card error state
  */
SD_Error_t SD_GetCardInfo(void)
{
    SD_Error_t ErrorState = SD_OK;
    uint32_t Temp = 0;

    // Byte 0
    Temp = (SD_Handle.CSD[0] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CSDStruct      = (uint8_t)((Temp & 0xC0) >> 6);
    SD_CardInfo.SD_csd.SysSpecVersion = (uint8_t)((Temp & 0x3C) >> 2);
    SD_CardInfo.SD_csd.Reserved1      = Temp & 0x03;

    // Byte 1
    Temp = (SD_Handle.CSD[0] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.TAAC = (uint8_t)Temp;

    // Byte 2
    Temp = (SD_Handle.CSD[0] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.NSAC = (uint8_t)Temp;

    // Byte 3
    Temp = SD_Handle.CSD[0] & 0x000000FF;
    SD_CardInfo.SD_csd.MaxBusClkFrec = (uint8_t)Temp;

    // Byte 4
    Temp = (SD_Handle.CSD[1] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CardComdClasses = (uint16_t)(Temp << 4);

    // Byte 5
    Temp = (SD_Handle.CSD[1] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.CardComdClasses |= (uint16_t)((Temp & 0xF0) >> 4);
    SD_CardInfo.SD_csd.RdBlockLen       = (uint8_t)(Temp & 0x0F);

    // Byte 6
    Temp = (SD_Handle.CSD[1] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.PartBlockRead   = (uint8_t)((Temp & 0x80) >> 7);
    SD_CardInfo.SD_csd.WrBlockMisalign = (uint8_t)((Temp & 0x40) >> 6);
    SD_CardInfo.SD_csd.RdBlockMisalign = (uint8_t)((Temp & 0x20) >> 5);
    SD_CardInfo.SD_csd.DSRImpl         = (uint8_t)((Temp & 0x10) >> 4);
    SD_CardInfo.SD_csd.Reserved2       = 0;

    if ((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0))
    {
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x03) << 10;

        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp) << 2;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp & 0xC0) >> 6;

        SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = (Temp & 0x38) >> 3;
        SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = (Temp & 0x07);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = (Temp & 0xE0) >> 5;
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = (Temp & 0x1C) >> 2;
        SD_CardInfo.SD_csd.DeviceSizeMul      = (Temp & 0x03) << 1;

        // Byte 10
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);
        SD_CardInfo.SD_csd.DeviceSizeMul |= (Temp & 0x80) >> 7;

        SD_CardInfo.CardCapacity  = (SD_CardInfo.SD_csd.DeviceSize + 1) ;
        SD_CardInfo.CardCapacity *= (1 << (SD_CardInfo.SD_csd.DeviceSizeMul + 2));
        SD_CardInfo.CardBlockSize = 1 << (SD_CardInfo.SD_csd.RdBlockLen);
        SD_CardInfo.CardCapacity = SD_CardInfo.CardCapacity * SD_CardInfo.CardBlockSize / 512;
    }
    else if (SD_CardType == SD_HIGH_CAPACITY)
    {
        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x3F) << 16;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp << 8);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp);

        // Byte 10
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);

        SD_CardInfo.CardCapacity  = ((uint64_t)SD_CardInfo.SD_csd.DeviceSize + 1) * 1024;
        SD_CardInfo.CardBlockSize = 512;
    }
    else
    {
        // Not supported card type
        ErrorState = SD_ERROR;
    }

    SD_CardInfo.SD_csd.EraseGrSize = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.EraseGrMul  = (Temp & 0x3F) << 1;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CSD[2] & 0x000000FF);
    SD_CardInfo.SD_csd.EraseGrMul     |= (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.WrProtectGrSize = (Temp & 0x7F);

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_csd.WrProtectGrEnable = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.ManDeflECC        = (Temp & 0x60) >> 5;
    SD_CardInfo.SD_csd.WrSpeedFact       = (Temp & 0x1C) >> 2;
    SD_CardInfo.SD_csd.MaxWrBlockLen     = (Temp & 0x03) << 2;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_csd.MaxWrBlockLen      |= (Temp & 0xC0) >> 6;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.Reserved3           = 0;
    SD_CardInfo.SD_csd.ContentProtectAppli = (Temp & 0x01);

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_csd.FileFormatGrouop = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.CopyFlag         = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.PermWrProtect    = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.TempWrProtect    = (Temp & 0x10) >> 4;
    SD_CardInfo.SD_csd.FileFormat       = (Temp & 0x0C) >> 2;
    SD_CardInfo.SD_csd.ECC              = (Temp & 0x03);

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CSD[3] & 0x000000FF);
    SD_CardInfo.SD_csd.CSD_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_csd.Reserved4 = 1;

    // Byte 0
    Temp = (uint8_t)((SD_Handle.CID[0] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ManufacturerID = Temp;

    // Byte 1
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.OEM_AppliID = Temp << 8;

    // Byte 2
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x000000FF00) >> 8);
    SD_CardInfo.SD_cid.OEM_AppliID |= Temp;

    // Byte 3
    Temp = (uint8_t)(SD_Handle.CID[0] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName1 = Temp << 24;

    // Byte 4
    Temp = (uint8_t)((SD_Handle.CID[1] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 16;

    // Byte 5
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 8;

    // Byte 6
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdName1 |= Temp;

    // Byte 7
    Temp = (uint8_t)(SD_Handle.CID[1] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName2 = Temp;

    // Byte 8
    Temp = (uint8_t)((SD_Handle.CID[2] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdRev = Temp;

    // Byte 9
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdSN = Temp << 24;

    // Byte 10
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 16;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CID[2] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 8;

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CID[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdSN |= Temp;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.Reserved1   |= (Temp & 0xF0) >> 4;
    SD_CardInfo.SD_cid.ManufactDate = (Temp & 0x0F) << 8;

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ManufactDate |= Temp;

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CID[3] & 0x000000FF);
    SD_CardInfo.SD_cid.CID_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Enables wide bus operation for the requested card if supported by card.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          SD_BUS_WIDE_4B: 4-bit data transfer
  *          SD_BUS_WIDE_1B: 1-bit data transfer
  * @retval SD Card error state
  */
static SD_Error_t SD_WideBusOperationConfig(uint32_t WideMode)
{
    SD_Error_t ErrorState = SD_OK;
    uint32_t   Temp;
    uint32_t   SCR[2] = {0, 0};

    if ((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
            (SD_CardType == SD_HIGH_CAPACITY))
    {
        if (WideMode == SD_BUS_WIDE_8B)
        {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        }
        else if ((WideMode == SD_BUS_WIDE_4B) ||
                (WideMode == SD_BUS_WIDE_1B))
        {
            if ((SDIO->RESP0 & SD_CARD_LOCKED) != SD_CARD_LOCKED)
            {
                // Get SCR Register
                ErrorState = SD_FindSCR(SCR);
                if (ErrorState == SD_OK)
                {
                    Temp = (WideMode == SD_BUS_WIDE_4B) ? SD_WIDE_BUS_SUPPORT : SD_SINGLE_BUS_SUPPORT;

                    // If requested card supports wide bus operation
                    if ((SCR[1] & Temp) != SD_ALLZERO)
                    {
                        // Send CMD55 APP_CMD with argument as card's RCA
                        ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);
                        if (ErrorState == SD_OK)
                        {
                            Temp = (WideMode == SD_BUS_WIDE_4B) ? 2 : 0;

                            // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
                            ErrorState = SD_TransmitCommand((SD_CMD_APP_SD_SET_BUSWIDTH | SD_CMD_RESPONSE_SHORT), Temp, 1);
                        }
                    }
                    else
                    {
                        ErrorState = SD_REQUEST_NOT_APPLICABLE;
                    }
                }
            }
            else
            {
                ErrorState = SD_LOCK_UNLOCK_FAILED;
            }
        }
        else
        {
            ErrorState = SD_INVALID_PARAMETER;  // WideMode is not a valid argument
        }

        if (ErrorState == SD_OK)
        {
            // Configure the SDIO peripheral bus width
            if (WideMode == SD_BUS_WIDE_4B) {
                SDIO->WIDTH = SDIO_WIDTH_WIDTH0;             // 4-bit bus width
            } else {
                SDIO->WIDTH = 0;                             // 1-bit bus width
            }
        }
    }
    else {
        ErrorState = SD_UNSUPPORTED_FEATURE;
    }

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Switches the SD card to High Speed mode.
  * @retval SD Card error state
  */
SD_Error_t SD_HighSpeed(void)
{
    SD_Error_t  ErrorState;
    uint8_t     SD_hs[64]  = {0};
    uint32_t    SD_scr[2]  = {0, 0};
    uint32_t    SD_SPEC    = 0;
    uint32_t    Count      = 0;
    uint32_t*   Buffer     = (uint32_t *)SD_hs;

    // Get SCR Register
    if ((ErrorState = SD_FindSCR(SD_scr)) != SD_OK)
    {
        return ErrorState;
    }

    // Test the Version supported by the card
    SD_SPEC = (SD_scr[1]  & 0x01000000) | (SD_scr[1]  & 0x02000000);

    if (SD_SPEC != SD_ALLZERO)
    {
        // Set Block Size for Card
        if ((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK)
        {
            return ErrorState;
        }

        // Configure the SD data transfer
        SD_DataTransferInit(64, 64, true);

        // Send CMD6 switch mode
        if ((ErrorState = SD_TransmitCommand((SD_CMD_HS_SWITCH | SD_CMD_RESPONSE_SHORT | SDIO_CMD_DATA_TRANSFER_EXPECTED | SDIO_CMD_READ_WRITE), 0x80FFFF01, 1)) != SD_OK)
        {
            return ErrorState;
        }

        // Read data from FIFO until transfer complete
        while ((SDIO->RINTSTS & (SDIO_RINTSTS_FIFO_UNDER_OVER_RUN | SDIO_RINTSTS_DATA_CRC_ERROR |
                                  SDIO_RINTSTS_READ_DATA_TIMEOUT | SDIO_RINTSTS_DATA_FINISH)) == 0)
        {
            if ((SDIO->STATUS & SDIO_STATUS_FIFO_RX_MATERMARK) != 0)
            {
                for (Count = 0; Count < 8; Count++)
                {
                    *(Buffer + Count) = SDIO->DATA;
                }
                Buffer += 8;
            }
        }

        if ((SDIO->RINTSTS & SDIO_RINTSTS_READ_DATA_TIMEOUT) != 0)        return SD_DATA_TIMEOUT;
        else if ((SDIO->RINTSTS & SDIO_RINTSTS_DATA_CRC_ERROR) != 0)      return SD_DATA_CRC_FAIL;
        else if ((SDIO->RINTSTS & SDIO_RINTSTS_FIFO_UNDER_OVER_RUN) != 0) return SD_RX_OVERRUN;

        // Drain remaining FIFO data
        Count = SD_DATATIMEOUT;
        while (((SDIO->STATUS & SDIO_STATUS_FIFO_EMPTY) == 0) && (Count > 0))
        {
            *Buffer = SDIO->DATA;
            Buffer++;
            Count--;
        }

        // Test if the switch mode HS is ok
        if ((SD_hs[13] & 2) != 2)
        {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        }
    }

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the current card's data status.
  * @retval Data Transfer state
  */
SD_Error_t SD_GetStatus(void)
{
    SD_Error_t     ErrorState;
    uint32_t       Response1;
    SD_CardState_t CardState;

    // Send Status command
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SEND_STATUS | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK)
    {
        Response1 = SDIO->RESP0;
        CardState = (SD_CardState_t)((Response1 >> 9) & 0x0F);

        // Find SD status according to card state
        if (CardState == SD_CARD_TRANSFER)  ErrorState = SD_OK;
        else if (CardState == SD_CARD_ERROR)     ErrorState = SD_ERROR;
        else                                    ErrorState = SD_BUSY;
    }
    else
    {
        ErrorState = SD_ERROR;
    }

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Gets the SD card status.
  * @param  pCardStatus: pointer to the card status structure
  * @retval SD Card error state
  */
SD_Error_t SD_GetCardStatus(SD_CardStatus_t* pCardStatus)
{
    SD_Error_t ErrorState;
    uint32_t   Temp = 0;
    uint32_t   Status[16];
    uint32_t   Count;

    // Check SD response
    if ((SDIO->RESP0 & SD_CARD_LOCKED) == SD_CARD_LOCKED)
    {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Set block size for card if it is not equal to current block size for card
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK)
    {
        return ErrorState;
    }

    // Send CMD55
    if ((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) != SD_OK)
    {
        return ErrorState;
    }

    // Configure data transfer
    SD_DataTransferInit(64, 64, true);

    // Send ACMD13 (SD_APP_STAUS) with argument as 0
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_STATUS | SD_CMD_RESPONSE_SHORT | SDIO_CMD_DATA_TRANSFER_EXPECTED | SDIO_CMD_READ_WRITE), 0, 1)) != SD_OK)
    {
        return ErrorState;
    }

    // Get status data
    int32_t statusIndex = 0;
    while ((SDIO->RINTSTS & (SDIO_RINTSTS_FIFO_UNDER_OVER_RUN | SDIO_RINTSTS_DATA_CRC_ERROR |
                              SDIO_RINTSTS_READ_DATA_TIMEOUT | SDIO_RINTSTS_DATA_FINISH)) == 0)
    {
        if ((SDIO->STATUS & SDIO_STATUS_FIFO_RX_MATERMARK) != 0)
        {
            for (Count = 0; Count < 8 && (statusIndex + Count)<16 ; Count++)
            {
                Status[statusIndex + Count] = SDIO->DATA;
            }
            statusIndex += 8;
        }
    }

    while((SDIO->STATUS & SDIO_STATUS_FIFO_EMPTY) == 0 && statusIndex < 16)
    {
        Status[statusIndex++]= SDIO->DATA;
    }
    if ((SDIO->RINTSTS & SDIO_RINTSTS_READ_DATA_TIMEOUT) != 0)         return SD_DATA_TIMEOUT;
    else if ((SDIO->RINTSTS & SDIO_RINTSTS_DATA_CRC_ERROR) != 0)       return SD_DATA_CRC_FAIL;
    else if ((SDIO->RINTSTS & SDIO_RINTSTS_FIFO_UNDER_OVER_RUN) != 0)  return SD_RX_OVERRUN;

    // Byte 0
    Temp = (Status[0] & 0xC0) >> 6;
    pCardStatus->DAT_BUS_WIDTH = (uint8_t)Temp;

    // Byte 0
    Temp = (Status[0] & 0x20) >> 5;
    pCardStatus->SECURED_MODE = (uint8_t)Temp;

    // Byte 2
    Temp = (Status[2] & 0xFF);
    pCardStatus->SD_CARD_TYPE = (uint8_t)(Temp << 8);

    // Byte 3
    Temp = (Status[3] & 0xFF);
    pCardStatus->SD_CARD_TYPE |= (uint8_t)Temp;

    // Byte 4
    Temp = (Status[4] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA = (uint8_t)(Temp << 24);

    // Byte 5
    Temp = (Status[5] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)(Temp << 16);

    // Byte 6
    Temp = (Status[6] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)(Temp << 8);

    // Byte 7
    Temp = (Status[7] & 0xFF);
    pCardStatus->SIZE_OF_PROTECTED_AREA |= (uint8_t)Temp;

    // Byte 8
    Temp = (Status[8] & 0xFF);
    pCardStatus->SPEED_CLASS = (uint8_t)Temp;

    // Byte 9
    Temp = (Status[9] & 0xFF);
    pCardStatus->PERFORMANCE_MOVE = (uint8_t)Temp;

    // Byte 10
    Temp = (Status[10] & 0xF0) >> 4;
    pCardStatus->AU_SIZE = (uint8_t)Temp;

    // Byte 11
    Temp = (Status[11] & 0xFF);
    pCardStatus->ERASE_SIZE = (uint16_t)(Temp << 8);

    // Byte 12
    Temp = (Status[12] & 0xFF);
    pCardStatus->ERASE_SIZE |= (uint16_t)Temp;

    // Byte 13
    Temp = (Status[13] & 0xFC) >> 2;
    pCardStatus->ERASE_TIMEOUT = (uint8_t)Temp;

    // Byte 13
    Temp = (Status[13] & 0x3);
    pCardStatus->ERASE_OFFSET = (uint8_t)Temp;

    return SD_OK;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Prepares the SDIO controller for power-on and sends the initialization sequence.
  * @retval SD Card error state
  */
static SD_Error_t SD_PowerON(void)
{
    SD_Error_t ErrorState;
    uint32_t   Response;
    uint32_t   Count;
    uint32_t   ValidVoltage;
    uint32_t   SD_Type;

    Count        = 0;
    ValidVoltage = 0;
    SD_Type      = SD_RESP_STD_CAPACITY;

    SDIO->POWEN   = SDIO_POWEN_POWER_ENABLE;       // Set Power State to ON

    /* open all card clk */
    SDIO->CLKENA |= SDIO_CLKENA_CLK_EN;
    // Send UPDATE_CLK command to apply clock settings (DWC_mshc requirement)
    SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;

    delay(5);
    // CMD0: GO_IDLE_STATE -----------------------------------------------------
    // No CMD response required
    if ((ErrorState = SD_TransmitCommand(SD_CMD_GO_IDLE_STATE, 0, 0)) != SD_OK)
    {
        // CMD Response Timeout (wait for CMDSENT flag)
        return ErrorState;
    }

    // CMD8: SEND_IF_COND ------------------------------------------------------
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //- [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //- [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7
    if ((ErrorState = SD_TransmitCommand((SD_SDIO_SEND_IF_COND | SD_CMD_RESPONSE_SHORT), SD_CHECK_PATTERN, 7)) == SD_OK)
    {
        // SD Card 2.0
        SD_CardType = SD_STD_CAPACITY_V2_0;
        SD_Type     = SD_RESP_HIGH_CAPACITY;
    }

    // Send CMD55
    // If ErrorState is Command Timeout, it is a MMC card
    // If ErrorState is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x
    if ((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) == SD_OK)
    {
        // SD CARD
        // Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
        while ((ValidVoltage == 0) && (Count < SD_MAX_VOLT_TRIAL))
        {
            // SEND CMD55 APP_CMD with RCA as 0
            if ((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK)
            {
                return ErrorState;
            }

            // Send CMD41 (ACMD41)
            if ((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_OP_COND | SD_CMD_RESPONSE_SHORT),
                                                  SD_VOLTAGE_WINDOW_SD | SD_Type, 3)) != SD_OK)
            {
                return ErrorState;
            }

            Response = SDIO->RESP0;                               // Get command response
            ValidVoltage = (((Response >> 31) == 1) ? 1 : 0);     // Get operating voltage
            Count++;
            delay(1);
        }

        if (Count >= SD_MAX_VOLT_TRIAL)
        {
            return SD_INVALID_VOLTRANGE;
        }

        if ((Response & SD_RESP_HIGH_CAPACITY) == SD_RESP_HIGH_CAPACITY)
        {
            SD_CardType = SD_HIGH_CAPACITY;
        }
    } // else MMC Card

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Finds the SD card SCR register value.
  * @param  pSCR: pointer to the buffer that will contain the SCR value (2 x uint32_t)
  * @retval SD Card error state
  */
static SD_Error_t SD_FindSCR(uint32_t *pSCR)
{
    SD_Error_t ErrorState;
    uint32_t Index = 0;
    uint32_t tempscr[2] = {0, 0};

    // Set Block Size To 8 Bytes
    // Send CMD55 APP_CMD with argument as card's RCA
    if ((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 8, 1)) == SD_OK)
    {
        // Send CMD55 APP_CMD with argument as card's RCA
        if ((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK)
        {
            SD_DataTransferInit(8, 8, true);

            // Send ACMD51 SD_APP_SEND_SCR with argument as 0
            if ((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_SEND_SCR | SD_CMD_RESPONSE_SHORT | SDIO_CMD_DATA_TRANSFER_EXPECTED |
                                                  SDIO_CMD_DATA_TRANSFER_EXPECTED | SDIO_CMD_WAIT_PRV_DATA_FINISH | SDIO_CMD_CHECK_REP_CRC), 0, 1)) == SD_OK)
            {
                while ((SDIO->RINTSTS & (SDIO_RINTSTS_FIFO_UNDER_OVER_RUN | SDIO_RINTSTS_DATA_CRC_ERROR |
                                          SDIO_RINTSTS_READ_DATA_TIMEOUT | SDIO_RINTSTS_DATA_FINISH)) == 0)
                {
                    if ((SDIO->STATUS & SDIO_STATUS_FIFO_RX_MATERMARK) != 0)
                    {
                        *(tempscr + Index) = SDIO->DATA;
                        Index++;
                    }
                }
                if(ErrorState == SD_OK)
                {
                    while((SDIO->STATUS & SDIO_STATUS_FIFO_EMPTY) == 0 && Index < 2)
                    {
                        *(tempscr + Index) = SDIO->DATA;
                        Index++;
                    }
                }

                if ((SDIO->RINTSTS & SDIO_RINTSTS_READ_DATA_TIMEOUT) != 0)      ErrorState = SD_DATA_TIMEOUT;
                else if ((SDIO->RINTSTS & SDIO_RINTSTS_DATA_CRC_ERROR) != 0)    ErrorState = SD_DATA_CRC_FAIL;
                else if ((SDIO->RINTSTS & SDIO_RINTSTS_FIFO_UNDER_OVER_RUN) != 0) ErrorState = SD_RX_OVERRUN;
                else
                {
                    *(pSCR + 1) = ((tempscr[0] & SD_0TO7BITS) << 24)  | ((tempscr[0] & SD_8TO15BITS) << 8) |
                                  ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

                    *(pSCR) = ((tempscr[1] & SD_0TO7BITS) << 24)  | ((tempscr[1] & SD_8TO15BITS) << 8) |
                              ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
                }
            }
        }
    }

    return ErrorState;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initialize the SDIO module, GPIO pins, NVIC, and clocks
  * @note   UM324xx uses internal DMA (IDMA) — no external DMA stream needed.
  *          The dma parameter is ignored (kept for API compatibility).
  */
bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);

    // Enable SDIO clock FIRST — the peripheral must be clocked before it can be
    // reset/released. Resetting before the clock is enabled leaves the SDIO in an
    // undefined state on a cold power-on (it only happens to work after a soft

    RCC_ResetCmd(RCC_AHB0(SDIO), ENABLE);
    delay(1);

    // Configure Pins
    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t d0 = IOGetByTag(IO_TAG(PC8));
    const IO_t d1 = IOGetByTag(IO_TAG(PC9));
    const IO_t d2 = IOGetByTag(IO_TAG(PC10));
    const IO_t d3 = IOGetByTag(IO_TAG(PC11));
    const IO_t clk = IOGetByTag(IO_TAG(PC12));
    const IO_t cmd = IOGetByTag(IO_TAG(PD2));

    IOInit(d0, OWNER_SDCARD, 0);
    if (is4BitWidth) {
        IOInit(d1, OWNER_SDCARD, 0);
        IOInit(d2, OWNER_SDCARD, 0);
        IOInit(d3, OWNER_SDCARD, 0);
    }
    IOInit(clk, OWNER_SDCARD, 0);
    IOInit(cmd, OWNER_SDCARD, 0);

#define SDIO_DATA       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SDIO_CMD        IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SDIO_CLK        IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

    IOConfigGPIOAF(d0, SDIO_DATA, GPIO_AF12_SDIO);
    if (is4BitWidth) {
        IOConfigGPIOAF(d1, SDIO_DATA, GPIO_AF12_SDIO);
        IOConfigGPIOAF(d2, SDIO_DATA, GPIO_AF12_SDIO);
        IOConfigGPIOAF(d3, SDIO_DATA, GPIO_AF12_SDIO);
    }
    IOConfigGPIOAF(clk, SDIO_CLK, GPIO_AF12_SDIO);
    IOConfigGPIOAF(cmd, SDIO_CMD, GPIO_AF12_SDIO);
    //set clk io Driving level
    GPIOC->DS = 0xAB00AAAA;     
    //set clk io Driving level
    GPIOD->DS &= ~(0x00000030); 
    // reset because RCC keeps the clock enable bit set).
    RCC_ClockCmd(RCC_AHB0(SDIO), ENABLE);
    // Set bus width to 1-bit initially
    SDIO->WIDTH = 0;
    /* close all card clk */
    SDIO->CLKENA &= ~(SDIO_CLKENA_CLK_EN);
    // Initialize SDIO Controller

    /*until START_CMD & UPDATE_CLK_ONLY is set ,CLK register is set*/
    SDIO->CMD = SDIO_CMD_START_CMD | SDIO_CMD_UPDATE_CLK | SDIO_CMD_WAIT_PRV_DATA_FINISH;

    /*waiting for START_CMD clear*/
    while(SDIO->CMD & SDIO_CMD_START_CMD);
    // Set initial clock divider for identification (400kHz or less)
    SDIO->CLKDIV = SDIO_INIT_CLK_DIV;

    /*until START_CMD & UPDATE_CLK_ONLY is set ,CLK register is set*/
    SDIO->CMD = SDIO_CMD_START_CMD | SDIO_CMD_UPDATE_CLK | SDIO_CMD_WAIT_PRV_DATA_FINISH;

    /*waiting for START_CMD clear*/
    while(SDIO->CMD & SDIO_CMD_START_CMD);

    /* open all card clk */
    SDIO->CLKENA |= SDIO_CLKENA_CLK_EN;


    // Send UPDATE_CLK command to apply clock settings (DWC_mshc requirement)
    SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;
    while (SDIO->CMD & SDIO_CMD_START_CMD);

    // Clear all pending interrupts
    SDIO->RINTSTS = 0xFFFFFFFF;

    // Enable all SDIO interrupt sources (INTMASK: 0 = interrupt enabled, 1 = masked)
    // Bits 0-15 correspond to RINTSTS bits 0-15; bit 16 = SDIO_INT_MASK (global NVIC mask)
    SDIO->INTMASK = 0;

    // NVIC configuration for SDIO interrupts
    HAL_NVIC_SetPriority(SDIO_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_SDIO_DMA), NVIC_PRIORITY_SUB(NVIC_PRIO_SDIO_DMA));
    HAL_NVIC_EnableIRQ(SDIO_IRQn);

    return true;
}


/** -----------------------------------------------------------------------------------------------------------------*/
bool SD_GetState(void)
{
    // Check SDCARD status
    if (SD_GetStatus() == SD_OK) return true;
    return false;
}


/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  Checks if SD card is detected
  * @retval true if card is present
  */
bool SD_IsDetected(void)
{
    // Check card detect pin (PB4 / CDETECT register)
    // CDETECT bit 0: 1 = card present
    if (SDIO->CDETECT & SDIO_CDETECT_CARD_DETECT) {
        return true;
    }
    return false;
}


/** -----------------------------------------------------------------------------------------------------------------*/
static SD_Error_t SD_DoInit(void)
{
    SD_Error_t errorState;

    // Ensure init clock divider is set (SD_PowerON will handle clock toggling)
    // CLKDIV may have been changed by previous operations; reset to init value
    SDIO->CLKDIV = SDIO_INIT_CLK_DIV;
    // Apply via SD_PowerON which properly handles clock disable/reenable with UPDATE_CLK

    // Identify card operating voltage
    errorState = SD_PowerON();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Initialize the present card and put them in idle state
    errorState = SD_InitializeCard();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Read CSD/CID MSD registers
    errorState = SD_GetCardInfo();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Select the Card - Send CMD7 SDIO_SEL_DESEL_CARD
    errorState = SD_TransmitCommand((SD_CMD_SEL_DESEL_CARD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);

    // Configure SDIO peripheral interface for high speed operation
    // Per DWC_mshc databook: CLKDIV must be changed only when clock is disabled
    SDIO->CLKENA &= ~SDIO_CLKENA_CLK_EN;               // Disable SDIO Clock
    SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;
    while (SDIO->CMD & SDIO_CMD_START_CMD);
    SDIO->CLKDIV = SDIO_TRANSFER_CLK_DIV;              // Set transfer clock divider
    SDIO->CLKENA |= SDIO_CLKENA_CLK_EN;                // Re-enable SDIO Clock
    SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;
    while (SDIO->CMD & SDIO_CMD_START_CMD);

    // Configure SD Bus width
    if (errorState == SD_OK)
    {
        // Enable wide operation
        if (sdioConfig()->use4BitWidth) {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_4B);
        } else {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_1B);
        }
        if (errorState == SD_OK && sdioConfig()->clockBypass) {
            if (SD_HighSpeed() == SD_OK) {
                // Set max speed — CLKDIV=1 => AHB/2
                // Per DWC_mshc: disable clock before changing divider
                SDIO->CLKENA &= ~SDIO_CLKENA_CLK_EN;
                SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;
                while (SDIO->CMD & SDIO_CMD_START_CMD);
                SDIO->CLKDIV = 1;
                SDIO->CLKENA |= SDIO_CLKENA_CLK_EN;
                SDIO->CMD = SDIO_CMD_UPDATE_CLK | SDIO_CMD_START_CMD;
                while (SDIO->CMD & SDIO_CMD_START_CMD);
            }
        }
    }

    return errorState;
}


SD_Error_t SD_Init(void)
{
    static bool sdInitAttempted = false;
    static SD_Error_t result = SD_ERROR;

    if (sdInitAttempted) {
        return result;
    }

    sdInitAttempted = true;

    result = SD_DoInit();
    return result;
}

bool mscSdioInitDma(void) 
{ 
    return true; 
}
/** -----------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function handles SD card interrupt request.
  *         Handles data transfer completion and errors via IDMA and RINTSTS.
  */
void SDIO_IRQHandler(void)
{
    // Check IDMA status
    uint32_t idsts = SDIO->IDSTS;

    // Transfer complete (TI)
    if ((idsts & SDIO_IDSTS_TI) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_TI;
        if ((SD_Handle.Operation & 0x02) == (SDIO_DIR_TX << 1))
        {
            // Write operation complete
            SD_Handle.TXCplt = 0;

            if ((SD_Handle.Operation & 0x01) == SD_MULTIPLE_BLOCK)
            {
                // Send stop command in multiblock write
                SD_TransmitCommand((SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
        }
    }

    // Receive interrupt (RI) — read IDMA finished moving data into the buffer
    if ((idsts & SDIO_IDSTS_RI) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_RI;
        SD_Handle.RXCplt = 0;   // Read complete (data now in memory)
    }

    // Fatal Bus Error
    if ((idsts & SDIO_IDSTS_FBE) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_FBE;
        SD_Handle.TransferError = SD_ERROR;
        SD_Handle.RXCplt = 0;
        SD_Handle.TXCplt = 0;
    }

    // Descriptor Unavailable
    if ((idsts & SDIO_IDSTS_DU) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_DU;
    }

    // Card Error Summary
    if ((idsts & SDIO_IDSTS_CES) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_CES;
        SD_Handle.TransferError = SD_ERROR;
        SD_Handle.RXCplt = 0;
        SD_Handle.TXCplt = 0;
    }

    // Normal Interrupt Summary
    if ((idsts & SDIO_IDSTS_NIS) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_NIS;
    }

    // Abnormal Interrupt Summary
    if ((idsts & SDIO_IDSTS_AIS) != 0)
    {
        SDIO->IDSTS = SDIO_IDSTS_AIS;
    }

    // Note: IDINTEN is disabled in the write-complete (DATA_FINISH TX) and
    // read-complete (IDSTS RI) branches above, not here, so that RI can still
    // fire after DATA_FINISH on a read operation.
}

#endif // USE_SDCARD_SDIO
