/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "drivers/sdmmc_sdio.h"
#include "gd32h7xx_gpio.h"
#include "gd32h7xx_sdio.h"

#include "pg/sdio.h"

#include "drivers/io.h"
#include "drivers/sdio.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "build/debug.h"

static uint32_t sdio_periph = SDIO0;  // Set by SD_Initialize_LL based on sdioConfig()->device
#define SDIO sdio_periph
//#define SD_SPEED_HIGH


#define BLOCK_SIZE                       ((uint32_t)(512))

#define SDIO_INTC_STATIC_FLAGS           ((uint32_t)(SDIO_INTC_CCRCERRC | SDIO_INTC_DTCRCERRC | SDIO_INTC_CMDTMOUTC |\
                                                     SDIO_INTC_DTTMOUTC | SDIO_INTC_TXUREC | SDIO_INTC_RXOREC  |\
                                                     SDIO_INTC_CMDRECVC  | SDIO_INTC_CMDSENDC  | SDIO_INTC_DTENDC  |\
                                                     SDIO_INTC_DTBLKENDC))
#define SDIO_MASK_DATA_FLAGS                ((uint32_t)0x18000F3A)    /* mask flags of DATA FLAGS */

#define SD_SOFTWARE_COMMAND_TIMEOUT      ((uint32_t)0x00220000)
#define SD_TRANSFER_TIMEOUT_MS           ((uint32_t)5000)

#define SD_OCR_ADDR_OUT_OF_RANGE         ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED           ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR             ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR             ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM           ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION      ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED        ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED            ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD               ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED           ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                  ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN      ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN      ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRITE         ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP             ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED         ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET               ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR             ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                 ((uint32_t)0xFDFFE008)

#define SD_R6_GENERAL_UNKNOWN_ERROR      ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD                ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED             ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD             ((uint32_t)0x80100000)
#define SD_RESP_HIGH_CAPACITY            ((uint32_t)0x40000000)
#define SD_RESP_STD_CAPACITY             ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                 ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL                ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                       ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT              ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT            ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                   ((uint32_t)0x02000000)

#define SD_0TO7BITS                      ((uint32_t)0x000000FF)
#define SD_8TO15BITS                     ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                    ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                    ((uint32_t)0xFF000000)
#define SD_SDIO_SEND_IF_COND             ((uint32_t)SD_CMD_HS_SEND_EXT_CSD)

#define SD_BUS_WIDE_1B                   ((uint32_t)0x00000000)
#define SD_BUS_WIDE_4B                   SDIO_BUSMODE_4BIT
#define SD_BUS_WIDE_8B                   SDIO_BUSMODE_8BIT

#define SD_CMD_RESPONSE_SHORT            SDIO_RESPONSETYPE_SHORT
#define SD_CMD_RESPONSE_LONG             SDIO_RESPONSETYPE_LONG

#define SD_DATABLOCK_SIZE_8B             SDIO_DATABLOCKSIZE_8BYTES
#define SD_DATABLOCK_SIZE_64B            SDIO_DATABLOCKSIZE_64BYTES
#define SD_DATABLOCK_SIZE_512B           SDIO_DATABLOCKSIZE_512BYTES

#define CLKCTL_CLEAR_MASK                ((uint32_t)(SDIO_CLKCTL_DIV  | SDIO_CLKCTL_CLKPWRSAV |\
                                                     SDIO_CLKCTL_BUSMODE | SDIO_CLKCTL_CLKEDGE))

#define SDIO_INIT_CLK_DIV                ((uint8_t)0x1F4)
#define SD_CLK_DIV_TRANS_DSPEED          ((uint32_t)0x0008)        /* SD clock division in default speed transmission phase */
#define SD_CLK_DIV_TRANS_HSPEED          ((uint32_t)0x0004)        /* SD clock division in high speed transmission phase */

#define SD_CMD_GO_IDLE_STATE            ((uint8_t)0)   // Resets the SD memory card.
#define SD_CMD_ALL_SEND_CID             ((uint8_t)2)   // Asks any card connected to the host to send the CID numbers on the CMD line.
#define SD_CMD_SET_REL_ADDR             ((uint8_t)3)   // Asks the card to publish a new relative address (RCA).
#define SD_CMD_SEL_DESEL_CARD           ((uint8_t)7)   // Selects the card by its own relative address and gets deselected by any other address
#define SD_CMD_HS_SEND_EXT_CSD          ((uint8_t)8)   // Sends SD Memory Card interface condition, which includes host supply voltage information
                                                       // and asks the card whether card supports voltage.
#define SD_CMD_SEND_CSD                 ((uint8_t)9)   // Addressed card sends its card specific data (CSD) on the CMD line.
#define SD_CMD_SEND_CID                 ((uint8_t)10)  // Addressed card sends its card identification (CID) on the CMD line.
#define SD_CMD_STOP_TRANSMISSION        ((uint8_t)12)  // Forces the card to stop transmission.
#define SD_CMD_SEND_STATUS              ((uint8_t)13)  // Addressed card sends its status register.
#define SD_CMD_SET_BLOCKLEN             ((uint8_t)16)  // Sets the block length (in bytes for SDSC) for all following block commands
                                                       // (read, write, lock). Default block length is fixed to 512 Bytes. Not effective
                                                       // for SDHS and SDXC.
#define SD_CMD_READ_SINGLE_BLOCK        ((uint8_t)17)  // Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                       // fixed 512 bytes in case of SDHC and SDXC.
#define SD_CMD_READ_MULT_BLOCK          ((uint8_t)18)  // Continuously transfers data blocks from card to host until interrupted by
                                                       // STOP_TRANSMISSION command.
#define SD_CMD_WRITE_SINGLE_BLOCK       ((uint8_t)24)  // Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of
                                                       // fixed 512 bytes in case of SDHC and SDXC.
#define SD_CMD_WRITE_MULT_BLOCK         ((uint8_t)25)  // Continuously writes blocks of data until a STOP_TRANSMISSION follows.
#define SD_CMD_APP_CMD                  ((uint8_t)55)  // Indicates to the card that the next command is an application specific command rather
                                                       // than a standard command.

#define SD_NO_RESPONSE                  ((int8_t)-1)

/* Following commands are SD Card Specific commands.
   SDIO_APP_CMD should be sent before sending these commands. */
#define SD_CMD_APP_SD_SET_BUSWIDTH      ((uint8_t)6)   // (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus
                                                       // widths are given in SCR register.
#define SD_CMD_SD_APP_STATUS            ((uint8_t)13)  // (ACMD13) Sends the SD status.
#define SD_CMD_SD_APP_OP_COND           ((uint8_t)41)  // (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to
                                                       // send its operating condition register (OCR) content in the response on the CMD line.
#define SD_CMD_SD_APP_SEND_SCR          ((uint8_t)51)  // Reads the SD Configuration Register (SCR).

#define SDIO_DIR_TX 1
#define SDIO_DIR_RX 0

#define SD_DMA_ERROR ((uint8_t)44)

typedef enum {
    SD_SINGLE_BLOCK    = 0,             // Single block operation
    SD_MULTIPLE_BLOCK  = 1,             // Multiple blocks operation
} SD_Operation_t;

typedef struct {
    uint32_t          CSD[4];           // SD card specific data table
    uint32_t          CID[4];           // SD card identification number table
    volatile uint32_t TransferComplete; // SD transfer complete flag in non blocking mode
    volatile uint32_t TransferError;    // SD transfer error flag in non blocking mode
    volatile uint32_t RXCplt;           // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;           // SD TX Complete is equal 0 when no transfer
    volatile uint32_t Operation;        // SD transfer operation (read/write)
} SD_Handle_t;

typedef enum {
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


static SD_Handle_t                 SD_Handle;
 SD_CardInfo_t                     SD_CardInfo;
static uint32_t                    SD_Status;
static uint32_t                    SD_CardRCA;
 SD_CardType_t                     SD_CardType;

static void             SD_DataTransferInit         (uint32_t SdioPeriph, uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard);
static SD_Error_t       SD_TransmitCommand          (uint32_t SdioPeriph, uint32_t Command, uint32_t Argument, int8_t ResponseType);
static SD_Error_t       SD_CmdResponse              (uint8_t SD_CMD, int8_t ResponseType);
static void             SD_GetResponse              (uint32_t* pResponse);
static SD_Error_t       CheckOCR_Response           (uint32_t Response_R1);

static SD_Error_t       SD_PowerON                  (void);
static SD_Error_t       SD_WideBusOperationConfig   (uint32_t WideMode);
static SD_Error_t       SD_FindSCR                  (uint32_t *pSCR);


// TODO
void sdioPinConfigure(void)
{

}


/*!
    \brief      Prepare the state machine for transfer
    \param[in]  SdioPeriph: sdio_periph: SDIOx(x=0,1)
    \param[in]  Size: data transfer size
    \param[in]  DataBlockSize: data block size
    \param[in]  IsItReadFromCard: transfer direction flag
    \param[out] none
    \retval     none
*/
static void SD_DataTransferInit(uint32_t SdioPeriph, uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard)
 {
    uint32_t Direction;

    SDIO_DATATO(SdioPeriph) = SD_DATATIMEOUT;
    SDIO_DATALEN(SdioPeriph)  = Size;
    Direction      = (IsItReadFromCard == true) ? SDIO_DATACTL_DATADIR : 0;
    SDIO_DATACTL(SdioPeriph) |=  (uint32_t)(DataBlockSize | Direction | SDIO_DATACTL_DATAEN  | 0x01);
    return;
}

/*!
    \brief      Send the command to SDIO
    \param[in]  SdioPeriph: sdio_periph: SDIOx(x=0,1)
    \param[in]  Command: SDIO command
    \param[in]  Argument: command argument
    \param[in]  ResponseType: response type (must provide the response size)
    \param[out] none
    \retval     SD Card error state
*/
static SD_Error_t SD_TransmitCommand(uint32_t SdioPeriph, uint32_t Command, uint32_t Argument, int8_t ResponseType)
{
    SD_Error_t ErrorState;

    SDIO_INTC(SdioPeriph) = SDIO_INTC_STATIC_FLAGS;                                         // Clear the Command Flags
    /* disable the CSM */
    SDIO_CMDCTL(SdioPeriph) &= ~SDIO_CMDCTL_CSMEN;
    SDIO_CMDAGMT(SdioPeriph) = (uint32_t)Argument;                                          // Set the SDIO Argument value
    SDIO_CMDCTL(SdioPeriph) = (uint32_t)(Command | SDIO_CMDCTL_CSMEN);                      // Set SDIO command parameters
    if((Argument == 0) && (ResponseType == 0)) ResponseType = SD_NO_RESPONSE;   // Go idle command
    ErrorState  = SD_CmdResponse(Command & SDIO_CMDCTL_CMDIDX, ResponseType);
    SDIO_INTC(SdioPeriph) = SDIO_INTC_STATIC_FLAGS;                                         // Clear the Command Flags

    return ErrorState;
}

/*!
    \brief      Checks for error conditions for any response (R2, R3, etc.)
    \param[in]  SD_CMD: the sent command Index
    \param[in]  ResponseType: response type
    \param[out] none
    \retval     SD Card error state
*/
static SD_Error_t SD_CmdResponse(uint8_t SD_CMD, int8_t ResponseType)
{
    uint32_t Response_R1;
    uint32_t TimeOut;
    uint32_t Flag;

    if(ResponseType == SD_NO_RESPONSE) {
        Flag = SDIO_STAT_CMDSEND;
    } else {
        Flag = SDIO_STAT_CCRCERR | SDIO_STAT_CMDRECV | SDIO_STAT_CMDTMOUT;
    }

    TimeOut = SD_SOFTWARE_COMMAND_TIMEOUT;
    do {
        SD_Status = SDIO_STAT(SDIO);
        TimeOut--;
    } while(((SD_Status & Flag) == 0) && (TimeOut > 0));

    if(ResponseType <= 0) {
        if(TimeOut == 0) {
            return SD_CMD_RSP_TIMEOUT;
        } else {
            return SD_OK;
        }
    }

    if((SDIO_STAT(SDIO) & SDIO_STAT_CMDTMOUT) != 0) {
        return SD_CMD_RSP_TIMEOUT;
    }

    if(ResponseType == 3) {
        if(TimeOut == 0) {
            return SD_CMD_RSP_TIMEOUT;  // Card is not V2.0 compliant or card does not support the set voltage range
        } else {
            return SD_OK;               // Card is SD V2.0 compliant
        }
    }

    if((SDIO_STAT(SDIO) & SDIO_STAT_CCRCERR) != 0) {
        return SD_CMD_CRC_FAIL;
    }
    if(ResponseType == 2) {
        return SD_OK;
    }
    if((uint8_t)SDIO_RSPCMDIDX(SDIO) != SD_CMD) {
        return SD_ILLEGAL_CMD;
    }

    Response_R1 = SDIO_RESP0(SDIO);                    // We have received response, retrieve it for analysis

    if(ResponseType == 1) {
        return CheckOCR_Response(Response_R1);
    } else if(ResponseType == 6) {
        if((Response_R1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)) == SD_ALLZERO) {
            SD_CardRCA = Response_R1;
        }
        if((Response_R1 & SD_R6_GENERAL_UNKNOWN_ERROR) == SD_R6_GENERAL_UNKNOWN_ERROR) {
            return SD_GENERAL_UNKNOWN_ERROR;
        }
        if((Response_R1 & SD_R6_ILLEGAL_CMD) == SD_R6_ILLEGAL_CMD) {
            return SD_ILLEGAL_CMD;
        }
        if((Response_R1 & SD_R6_COM_CRC_FAILED) == SD_R6_COM_CRC_FAILED) {
            return SD_COM_CRC_FAILED;
        }
    }

    return SD_OK;
}

/*!
    \brief      Analyze the OCR response and return the appropriate error code
    \param[in]  Response_R1: OCR Response code
    \param[out] none
    \retval     SD Card error state
*/
static SD_Error_t CheckOCR_Response(uint32_t Response_R1)
{
    if((Response_R1 & SD_OCR_ERRORBITS)             == SD_ALLZERO)                  return SD_OK;
    if((Response_R1 & SD_OCR_ADDR_OUT_OF_RANGE)     == SD_OCR_ADDR_OUT_OF_RANGE)    return SD_ADDR_OUT_OF_RANGE;
    if((Response_R1 & SD_OCR_ADDR_MISALIGNED)       == SD_OCR_ADDR_MISALIGNED)      return SD_ADDR_MISALIGNED;
    if((Response_R1 & SD_OCR_BLOCK_LEN_ERR)         == SD_OCR_BLOCK_LEN_ERR)        return SD_BLOCK_LEN_ERR;
    if((Response_R1 & SD_OCR_ERASE_SEQ_ERR)         == SD_OCR_ERASE_SEQ_ERR)        return SD_ERASE_SEQ_ERR;
    if((Response_R1 & SD_OCR_BAD_ERASE_PARAM)       == SD_OCR_BAD_ERASE_PARAM)      return SD_BAD_ERASE_PARAM;
    if((Response_R1 & SD_OCR_WRITE_PROT_VIOLATION)  == SD_OCR_WRITE_PROT_VIOLATION) return SD_WRITE_PROT_VIOLATION;
    if((Response_R1 & SD_OCR_LOCK_UNLOCK_FAILED)    == SD_OCR_LOCK_UNLOCK_FAILED)   return SD_LOCK_UNLOCK_FAILED;
    if((Response_R1 & SD_OCR_COM_CRC_FAILED)        == SD_OCR_COM_CRC_FAILED)       return SD_COM_CRC_FAILED;
    if((Response_R1 & SD_OCR_ILLEGAL_CMD)           == SD_OCR_ILLEGAL_CMD)          return SD_ILLEGAL_CMD;
    if((Response_R1 & SD_OCR_CARD_ECC_FAILED)       == SD_OCR_CARD_ECC_FAILED)      return SD_CARD_ECC_FAILED;
    if((Response_R1 & SD_OCR_CC_ERROR)              == SD_OCR_CC_ERROR)             return SD_CC_ERROR;
    if((Response_R1 & SD_OCR_GENERAL_UNKNOWN_ERROR) == SD_OCR_GENERAL_UNKNOWN_ERROR)return SD_GENERAL_UNKNOWN_ERROR;
    if((Response_R1 & SD_OCR_STREAM_READ_UNDERRUN)  == SD_OCR_STREAM_READ_UNDERRUN) return SD_STREAM_READ_UNDERRUN;
    if((Response_R1 & SD_OCR_STREAM_WRITE_OVERRUN)  == SD_OCR_STREAM_WRITE_OVERRUN) return SD_STREAM_WRITE_OVERRUN;
    if((Response_R1 & SD_OCR_CID_CSD_OVERWRITE)     == SD_OCR_CID_CSD_OVERWRITE)    return SD_CID_CSD_OVERWRITE;
    if((Response_R1 & SD_OCR_WP_ERASE_SKIP)         == SD_OCR_WP_ERASE_SKIP)        return SD_WP_ERASE_SKIP;
    if((Response_R1 & SD_OCR_CARD_ECC_DISABLED)     == SD_OCR_CARD_ECC_DISABLED)    return SD_CARD_ECC_DISABLED;
    if((Response_R1 & SD_OCR_ERASE_RESET)           == SD_OCR_ERASE_RESET)          return SD_ERASE_RESET;
    if((Response_R1 & SD_OCR_AKE_SEQ_ERROR)         == SD_OCR_AKE_SEQ_ERROR)        return SD_AKE_SEQ_ERROR;

    return SD_OK;
}

/*!
    \brief      Get response from SD device
    \param[in]  none
    \param[out] pResponse: pointer to response buffer
    \retval     none
*/
static void SD_GetResponse(uint32_t* pResponse)
{
    pResponse[0] = SDIO_RESP0(SDIO);
    pResponse[1] = SDIO_RESP1(SDIO);
    pResponse[2] = SDIO_RESP2(SDIO);
    pResponse[3] = SDIO_RESP3(SDIO);
}

/*!
    \brief      initialize the card
    \param[in]  none
    \param[out] none
    \retval     none
    \note       BlockSize must be 512 bytes
*/
static SD_Error_t SD_InitializeCard(void)
{
    SD_Error_t ErrorState = SD_OK;

    if((SDIO_PWRCTL(SDIO) & SDIO_PWRCTL_PWRCTL) != 0)
    {
        if(SD_CardType != SD_SECURE_DIGITAL_IO) {
            // Send CMD2 ALL_SEND_CID
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_ALL_SEND_CID | SD_CMD_RESPONSE_LONG), 0, 2)) != SD_OK) {
                return ErrorState;
            }

            // Get Card identification number data
            SD_GetResponse(SD_Handle.CID);
        }

        if((SD_CardType == SD_STD_CAPACITY_V1_1)    || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
           (SD_CardType == SD_SECURE_DIGITAL_IO_COMBO) || (SD_CardType == SD_HIGH_CAPACITY)) {
            // Send CMD3 SET_REL_ADDR with argument 0
            // SD Card publishes its RCA.
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SET_REL_ADDR | SD_CMD_RESPONSE_SHORT), 0, 6)) != SD_OK) {
                return ErrorState;
            }
        }

        if(SD_CardType != SD_SECURE_DIGITAL_IO) {
            // Send CMD9 SEND_CSD with argument as card's RCA
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SEND_CSD | SD_CMD_RESPONSE_LONG), SD_CardRCA, 2)) == SD_OK) {
                // Get Card Specific Data
                SD_GetResponse(SD_Handle.CSD);
            }
        }
    } else {
        ErrorState = SD_REQUEST_NOT_APPLICABLE;
    }

    return ErrorState;
}

/*!
    \brief      Reads block(s) from a specified address in a card. The Data transfer is managed by DMA mode
    \param[in]  ReadAddress: address from where data is to be read
    \param[in]  buffer: pointer to the buffer that will contain the received data
    \param[in]  BlockSize: SD card Data block size (must be 512 bytes)
    \param[in]  NumberOfBlocks: number of blocks to read
    \param[out] none
    \retval     SD Card error state
    \note       This API should be followed by the function SD_CheckOperation() to check the completion of the read process. BlockSize must be 512 bytes
*/
static void SD_StartBlockTransfert(uint32_t* pBuffer, uint32_t BlockSize, uint32_t NumberOfBlocks, uint8_t dir)
{
    SDIO_DATACTL(SDIO)                = 0;                                                                 // Initialize data control register
    SD_Handle.TransferComplete = 0;                                                                  // Initialize handle flags
    SD_Handle.TransferError    = SD_OK;
    SD_Handle.Operation        = (NumberOfBlocks > 1) ? SD_MULTIPLE_BLOCK : SD_SINGLE_BLOCK;         // Initialize SD Read operation
    SD_Handle.Operation       |= dir << 1;
    SDIO_INTEN(SDIO)                 = 0;
    if (dir == SDIO_DIR_RX) {
        SDIO_INTEN(SDIO)            |= (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |                     // Enable transfer interrupts
                                      SDIO_INTEN_DTENDIE  | SDIO_INTEN_RXOREIE);
    } else {
        SDIO_INTEN(SDIO)            |= (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |                     // Enable transfer interrupts
                                      SDIO_INTEN_DTENDIE  | SDIO_INTEN_TXUREIE);
    }
    sdio_idma_set(SDIO, SDIO_IDMA_SINGLE_BUFFER, (BlockSize >> 5));
    sdio_idma_buffer0_address_set(SDIO, (uint32_t)pBuffer);
    SDIO_IDMACTL(SDIO) |= SDIO_IDMACTL_IDMAEN;

}

/*!
    \brief      Writes block(s) to a specified address in a card. The Data transfer is managed by DMA mode
    \param[in]  WriteAddress: address from where data is to be read
    \param[in]  buffer: pointer to the buffer that will contain the data to transmit
    \param[in]  BlockSize: the SD card Data block size (must be 512 bytes)
    \param[in]  NumberOfBlocks: number of blocks to write
    \param[out] none
    \retval     SD Card error state
    \note       This API should be followed by the function SD_CheckOperation() to check the completion of the write process (by SD current status polling). BlockSize must be 512 bytes
*/
SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;
    uint32_t   CmdIndex;
    SD_Handle.RXCplt = 1;

    if (BlockSize != 512) {
        return SD_ERROR;
    }
    if ((uint32_t)buffer & 0x1F) {
        return SD_ADDR_MISALIGNED;
    }

    SD_Handle.TransferComplete = 0;
    if(SD_CardType != SD_HIGH_CAPACITY)
    {
        ReadAddress *= 512;
    }

    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_RX);

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(SDIO, BlockSize * NumberOfBlocks, SD_DATABLOCK_SIZE_512B, true);

    // Set Block Size for Card
    ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), BlockSize, 1);

    // Send CMD18 READ_MULT_BLOCK with argument data address
    // or send CMD17 READ_SINGLE_BLOCK depending on number of block
    uint8_t retries = 10;
    CmdIndex   = (NumberOfBlocks > 1) ? SD_CMD_READ_MULT_BLOCK : SD_CMD_READ_SINGLE_BLOCK;
    do {
            ErrorState = SD_TransmitCommand(SDIO, (CmdIndex | SD_CMD_RESPONSE_SHORT), (uint32_t)ReadAddress, 1);
            if (ErrorState != SD_OK && retries--) {
                ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
    } while (ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
            SD_Handle.RXCplt = 0;
            return ErrorState;
    }

    uint32_t transferStart = millis();
    while(0U == SD_Handle.TransferComplete) {
        if (millis() - transferStart > SD_TRANSFER_TIMEOUT_MS) {
            SD_Handle.TransferError = SD_DATA_TIMEOUT;
            SD_Handle.RXCplt = 0;
            return SD_DATA_TIMEOUT;
        }
    }

    // Update the SD transfer error in SD handle
    SD_Handle.TransferError = ErrorState;
    uint32_t alignedAddr = (uint32_t)buffer & ~0x1FU;
    SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr,
        (int32_t)(NumberOfBlocks * BlockSize + ((uint32_t)buffer - alignedAddr)));

    return ErrorState;
}

/*!
    \brief      Writes block(s) to a specified address in a card. The Data transfer is managed by DMA mode
    \param[in]  WriteAddress: address from where data is to be read
    \param[in]  buffer: pointer to the buffer that will contain the data to transmit
    \param[in]  BlockSize: the SD card Data block size (must be 512 bytes)
    \param[in]  NumberOfBlocks: number of blocks to write
    \param[out] none
    \retval     SD Card error state
    \note       This API should be followed by the function SD_CheckOperation() to check the completion of the write process (by SD current status polling). BlockSize must be 512 bytes
*/
SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;
    uint32_t   CmdIndex;
    SD_Handle.TXCplt = 1;

    if (BlockSize != 512) {
        return SD_ERROR;
    }
    if ((uint32_t)buffer & 0x1F) {
        return SD_ADDR_MISALIGNED;
    }

    if(SD_CardType != SD_HIGH_CAPACITY)
    {
        WriteAddress *= 512;
    }


    // Check number of blocks command
    // Send CMD24 WRITE_SINGLE_BLOCK
    // Send CMD25 WRITE_MULT_BLOCK with argument data address
    CmdIndex = (NumberOfBlocks > 1) ? SD_CMD_WRITE_MULT_BLOCK : SD_CMD_WRITE_SINGLE_BLOCK;

    // Set Block Size for Card
    uint8_t retries = 10;
    do {
            ErrorState = SD_TransmitCommand(SDIO, (CmdIndex | SD_CMD_RESPONSE_SHORT), (uint32_t)WriteAddress, 1);
            if (ErrorState != SD_OK && retries--) {
                ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
    } while(ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
            SD_Handle.TXCplt = 0;
            return ErrorState;
    }

    SCB_CleanDCache_by_Addr((uint32_t*)buffer, (int32_t)(NumberOfBlocks * BlockSize));
    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_TX);

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(SDIO, BlockSize * NumberOfBlocks, SD_DATABLOCK_SIZE_512B, false);

    uint32_t transferStart = millis();
    while(0U == SD_Handle.TransferComplete) {
        if (millis() - transferStart > SD_TRANSFER_TIMEOUT_MS) {
            SD_Handle.TransferError = SD_DATA_TIMEOUT;
            SD_Handle.TXCplt = 0;
            return SD_DATA_TIMEOUT;
        }
    }

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

/*!
    \brief      Returns information about specific card, contains all SD card information
    \param[in]  none
    \param[out] none
    \retval     SD Card error state
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
    SD_CardInfo.SD_csd.Reserved2       = 0; /*!< Reserved */

    if((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0)) {
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
        SD_CardInfo.CardCapacity = SD_CardInfo.CardCapacity * SD_CardInfo.CardBlockSize / 512; // In 512 byte blocks
    } else if(SD_CardType == SD_HIGH_CAPACITY) {
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
    } else {
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

/*!
    \brief      Enables wide bus operation for the requested card if supported by card
    \param[in]  WideMode: specifies the SD card wide bus mode (SD_BUS_WIDE_8B/4B/1B)
    \param[out] none
    \retval     SD Card error state
    \note       WideMode can be: SD_BUS_WIDE_8B (8-bit data transfer, only for MMC), SD_BUS_WIDE_4B (4-bit data transfer), SD_BUS_WIDE_1B (1-bit data transfer)
*/
static SD_Error_t SD_WideBusOperationConfig(uint32_t WideMode)
{
    SD_Error_t ErrorState = SD_OK;
    uint32_t   Temp;
    uint32_t   SCR[2] = {0, 0};

    if((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
        (SD_CardType == SD_HIGH_CAPACITY)) {

        if(WideMode == SD_BUS_WIDE_8B) {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        } else if((WideMode == SD_BUS_WIDE_4B) || (WideMode == SD_BUS_WIDE_1B)) {
            if((SDIO_RESP0(SDIO) & SD_CARD_LOCKED) != SD_CARD_LOCKED) {
                // Get SCR Register
                ErrorState = SD_FindSCR(SCR);
                if(ErrorState == SD_OK) {
                    Temp = (WideMode == SD_BUS_WIDE_4B) ? SD_WIDE_BUS_SUPPORT : SD_SINGLE_BUS_SUPPORT;

                    // If requested card supports wide bus operation
                    if((SCR[1] & Temp) != SD_ALLZERO) {
                        // Send CMD55 APP_CMD with argument as card's RCA.
                        ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);
                        if(ErrorState == SD_OK) {
                            Temp = (WideMode == SD_BUS_WIDE_4B) ? 2 : 0;

                            // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
                            ErrorState =  SD_TransmitCommand(SDIO, (SD_CMD_APP_SD_SET_BUSWIDTH | SD_CMD_RESPONSE_SHORT), Temp, 1);
                        }
                    } else {
                        ErrorState = SD_REQUEST_NOT_APPLICABLE;
                    }
                }
            } else {
                ErrorState = SD_LOCK_UNLOCK_FAILED;
            }
        } else {
            ErrorState = SD_INVALID_PARAMETER;  // WideMode is not a valid argument
        }

        if(ErrorState == SD_OK) {
                /* reset the SDIO card bus mode bits */
                SDIO_CLKCTL(SDIO) &= ~SDIO_CLKCTL_BUSMODE;
                /* set the bus mode according to bus_mode */
                SDIO_CLKCTL(SDIO) |= (uint32_t) WideMode;
        }
    } else {
            ErrorState = SD_UNSUPPORTED_FEATURE;
    }

    return ErrorState;
}


static SD_Error_t SD_GetStatus(void)
{
    SD_Error_t     ErrorState;
    uint32_t       Response1;
    SD_CardState_t CardState;

    // Send Status command
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SEND_STATUS | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK) {
        Response1 = SDIO_RESP0(SDIO);
        CardState = (SD_CardState_t)((Response1 >> 9) & 0x0F);

        // Find SD status according to card state
        if     (CardState == SD_CARD_TRANSFER)  ErrorState = SD_OK;
        else if(CardState == SD_CARD_ERROR)     ErrorState = SD_ERROR;
        else                                    ErrorState = SD_BUSY;
    } else {
        ErrorState = SD_ERROR;
    }

    return ErrorState;
}

/*!
    \brief      Gets the SD card status
    \param[in]  none
    \param[out] pCardStatus: pointer to SD card status structure
    \retval     SD Card error state
*/
SD_Error_t SD_GetCardStatus(SD_CardStatus_t* pCardStatus)
{
    SD_Error_t ErrorState;
    uint32_t   Temp = 0;
    uint32_t   Status[16];
    uint32_t   Count;

    // Check SD response
    if((SDIO_RESP0(SDIO) & SD_CARD_LOCKED) == SD_CARD_LOCKED) {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Set block size for card if it is not equal to current block size for card
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK) {
        return ErrorState;
    }

    // Send CMD55
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) != SD_OK) {
        return ErrorState;
    }

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(SDIO, 64, SD_DATABLOCK_SIZE_64B, true);

    // Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SD_APP_STATUS | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK) {
        return ErrorState;
    }

    // Get status data
    while((SDIO_STAT(SDIO) & (SDIO_STAT_RXORE | SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_DTBLKEND)) == 0) {
        if((SDIO_STAT(SDIO) & SDIO_STAT_RFH) != 0) {
            for(Count = 0; Count < 8; Count++) {
                Status[Count] = SDIO_FIFO(SDIO);
            }
        }
    }

    if((SDIO_STAT(SDIO) & SDIO_STAT_DTTMOUT) != 0)         return SD_DATA_TIMEOUT;
    else if((SDIO_STAT(SDIO) & SDIO_STAT_DTCRCERR) != 0)    return SD_DATA_CRC_FAIL;
    else if((SDIO_STAT(SDIO) & SDIO_STAT_RXORE) != 0)     return SD_RX_OVERRUN;
    else
    {
    }

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
    pCardStatus->ERASE_SIZE = (uint8_t)(Temp << 8);

    // Byte 12
    Temp = (Status[12] & 0xFF);
    pCardStatus->ERASE_SIZE |= (uint8_t)Temp;

    // Byte 13
    Temp = (Status[13] & 0xFC) >> 2;
    pCardStatus->ERASE_TIMEOUT = (uint8_t)Temp;

    // Byte 13
    Temp = (Status[13] & 0x3);
    pCardStatus->ERASE_OFFSET = (uint8_t)Temp;

    return SD_OK;
}

static SD_Error_t SD_PowerON(void)
{
    SD_Error_t ErrorState;
    uint32_t   Response = 0;
    uint32_t   Count;
    uint32_t   ValidVoltage;
    uint32_t   SD_Type;

    Count        = 0;
    ValidVoltage = 0;
    SD_Type      = SD_RESP_STD_CAPACITY;

    SDIO_PWRCTL(SDIO)  = SDIO_PWRCTL_PWRCTL;        // Set Power State to ON

    delay(2);

    // CMD0: GO_IDLE_STATE -----------------------------------------------------
    // No CMD response required
    if((ErrorState = SD_TransmitCommand(SDIO, SD_CMD_GO_IDLE_STATE, 0, 0)) != SD_OK) {
        // CMD Response Timeout (wait for CMDSENT flag)
        return ErrorState;
    }

    // CMD8: SEND_IF_COND ------------------------------------------------------
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //- [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //- [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7 */
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_SDIO_SEND_IF_COND | SD_CMD_RESPONSE_SHORT), SD_CHECK_PATTERN, 7)) == SD_OK) {
        // SD Card 2.0
        SD_CardType = SD_STD_CAPACITY_V2_0;
        SD_Type     = SD_RESP_HIGH_CAPACITY;
    }

    // Send CMD55
    // If ErrorState is Command Timeout, it is a MMC card
    // If ErrorState is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) == SD_OK) {
        // SD CARD
        // Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
        while((ValidVoltage == 0) && (Count < SD_MAX_VOLT_TRIAL)) {
            // SEND CMD55 APP_CMD with RCA as 0
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK) {
                return ErrorState;
            }

            // Send CMD41
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SD_APP_OP_COND | SD_CMD_RESPONSE_SHORT), SD_VOLTAGE_WINDOW_SD | SD_Type, 3)) != SD_OK) {
                return ErrorState;
            }

            Response = SDIO_RESP0(SDIO);                                  // Get command response
            ValidVoltage = (((Response >> 31) == 1) ? 1 : 0);       // Get operating voltage
            Count++;
        }

        if(Count >= SD_MAX_VOLT_TRIAL) {
            return SD_INVALID_VOLTRANGE;
        }

        if((Response & SD_RESP_HIGH_CAPACITY) == SD_RESP_HIGH_CAPACITY) {
            SD_CardType = SD_HIGH_CAPACITY;
        }
    } // else MMC Card

    return ErrorState;
}

static SD_Error_t SD_FindSCR(uint32_t *pSCR)
{
    SD_Error_t ErrorState;
    uint32_t Index = 0;
    uint32_t tempscr[2] = {0, 0};

    // Set Block Size To 8 Bytes
    // Send CMD55 APP_CMD with argument as card's RCA
    if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 8, 1)) == SD_OK) {
        // Send CMD55 APP_CMD with argument as card's RCA
        if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK) {
            SD_DataTransferInit(SDIO, 8, SD_DATABLOCK_SIZE_8B, true);

            // Send ACMD51 SD_APP_SEND_SCR with argument as 0
            if((ErrorState = SD_TransmitCommand(SDIO, (SD_CMD_SD_APP_SEND_SCR | SD_CMD_RESPONSE_SHORT), 0, 1)) == SD_OK) {
                while((SDIO_STAT(SDIO) & (SDIO_STAT_RXORE | SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_DTBLKEND | SDIO_FLAG_DTEND)) == 0) {
                    if(((SDIO_STAT(SDIO) & SDIO_FLAG_RFE) == 0) && ((SDIO_STAT(SDIO) & SDIO_FLAG_DATSTA) != 0)) {
                        *(tempscr + Index) = SDIO_FIFO(SDIO);
                        Index++;
                    }
                }

                if     ((SDIO_STAT(SDIO) & SDIO_STAT_DTTMOUT) != 0) ErrorState = SD_DATA_TIMEOUT;
                else if((SDIO_STAT(SDIO) & SDIO_STAT_DTCRCERR) != 0) ErrorState = SD_DATA_CRC_FAIL;
                else if((SDIO_STAT(SDIO) & SDIO_STAT_RXORE)  != 0) ErrorState = SD_RX_OVERRUN;
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

void sdioInitialize(void)
{

}

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);
    uint32_t timeout = 0;

    // Select SDIO peripheral based on configuration
    sdio_periph = (sdioConfig()->device == SDIO_DEV_TO_CFG(SDIODEV_2)) ? SDIO1 : SDIO0;

    // Reset only the selected SDIO module
    if (sdio_periph == SDIO0) {
        RCU_AHB3RST |=  RCU_AHB3RST_SDIO0RST;
        delay(1);
        RCU_AHB3RST &= ~RCU_AHB3RST_SDIO0RST;
    } else {
        RCU_AHB2RST |=  RCU_AHB2RST_SDIO1RST;
        delay(1);
        RCU_AHB2RST &= ~RCU_AHB2RST_SDIO1RST;
    }
    delay(1);

    // Enable SDIO clock
    /* SDIO clock 200M */
    /* configure the pll1 input and output clock range */
    rcu_pll_input_output_clock_range_config(IDX_PLL1, RCU_PLL1RNG_4M_8M, RCU_PLL1VCO_192M_836M);
    /* configure the PLL1 clock: CK_PLL1P/CK_PLL1Q/CK_PLL1R = HXTAL_VALUE / 5 * 40 / 1 */
    rcu_pll1_config(5, 40, 1, 1, 1);
    /* enable PLL1R clock output */
    rcu_pll_clock_output_enable(RCU_PLL1R);
    /* enable PLL1 clock */
    rcu_osci_on(RCU_PLL1_CK);

    while(timeout < 0xFFFF && ERROR == rcu_osci_stab_wait(RCU_PLL1_CK)) {
        delay(1);
        timeout++;
    }
    if(timeout >= 0xFFFF) {
        return false; // PLL1 failed to stabilize
    }

    // Enable clock for the selected SDIO
    if (sdio_periph == SDIO0) {
        rcu_sdio_clock_config(IDX_SDIO0, RCU_SDIO0SRC_PLL1R);
        rcu_periph_clock_enable(RCU_SDIO0);
    } else {
        rcu_sdio_clock_config(IDX_SDIO1, RCU_SDIO1SRC_PLL1R);
        rcu_periph_clock_enable(RCU_SDIO1);
    }

    // Configure GPIO pins from sdioPinConfig (IOConfigGPIOAF enables port clocks automatically)
    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t d0  = IOGetByTag(sdioPinConfig()->D0Pin);
    const IO_t d1  = IOGetByTag(sdioPinConfig()->D1Pin);
    const IO_t d2  = IOGetByTag(sdioPinConfig()->D2Pin);
    const IO_t d3  = IOGetByTag(sdioPinConfig()->D3Pin);
    const IO_t clk = IOGetByTag(sdioPinConfig()->CKPin);
    const IO_t cmd = IOGetByTag(sdioPinConfig()->CMDPin);

    IOInit(d0, OWNER_SDCARD, 0);
    if (is4BitWidth) {
        IOInit(d1, OWNER_SDCARD, 0);
        IOInit(d2, OWNER_SDCARD, 0);
        IOInit(d3, OWNER_SDCARD, 0);
    }
    IOInit(clk, OWNER_SDCARD, 0);
    IOInit(cmd, OWNER_SDCARD, 0);

#define SDIO_DATA       IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_100_220MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SDIO_CMD        IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_100_220MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SDIO_CLK        IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_100_220MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)

    IOConfigGPIOAF(d0, SDIO_DATA, GPIO_AF_12);

    if (is4BitWidth) {
        IOConfigGPIOAF(d1, SDIO_DATA, GPIO_AF_12);
        IOConfigGPIOAF(d2, SDIO_DATA, GPIO_AF_12);
        IOConfigGPIOAF(d3, SDIO_DATA, GPIO_AF_12);
    }

    IOConfigGPIOAF(clk, SDIO_CLK, GPIO_AF_12);
    IOConfigGPIOAF(cmd, SDIO_CMD, GPIO_AF_12);

    // NVIC configuration for selected SDIO interrupts
    if (sdio_periph == SDIO0) {
        nvic_irq_enable(SDIO0_IRQn, 2, 0);
    } else {
        nvic_irq_enable(SDIO1_IRQn, 2, 0);
    }

    return true;
}

bool SD_GetState(void)
{
    // Check SDCARD status
    if(SD_GetStatus() == SD_OK) return true;
    return false;
}

static SD_Error_t SD_DoInit(void)
{
    SD_Error_t errorState;
    uint32_t reg;

    // Initialize SDIO peripheral interface with default configuration for SD card initialization.
    reg = SDIO_CLKCTL(SDIO);
    reg &= ~CLKCTL_CLEAR_MASK;
    reg |= (uint32_t)SDIO_INIT_CLK_DIV;
    SDIO_CLKCTL(SDIO) = reg;

    // Identify card operating voltage.
    errorState = SD_PowerON();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Initialize the present card and put them in idle state.
    errorState = SD_InitializeCard();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Read CSD/CID MSD registers.
    errorState = SD_GetCardInfo();
    if (errorState != SD_OK) {
        return errorState;
    }

    // Select the Card - Send CMD7 SDIO_SEL_DESEL_CARD.
    errorState = SD_TransmitCommand(SDIO, (SD_CMD_SEL_DESEL_CARD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);
    // Configure SDIO peripheral interface.
    reg = SDIO_CLKCTL(SDIO);
    reg &= ~CLKCTL_CLEAR_MASK;
    reg |= (uint32_t) SD_CLK_DIV_TRANS_DSPEED;
    SDIO_CLKCTL(SDIO) = reg;

    // Configure SD Bus width.
    if (errorState == SD_OK) {
        // Enable wide operation.
        if (sdioConfig()->use4BitWidth) {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_4B);
        } else {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_1B);
        }
#ifdef SD_SPEED_HIGH
        /* change the clock to high speed , user according to the speed configuration */
        sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS_HSPEED);
        sdio_hardware_clock_enable(SDIO);
#else
        /* change the clock to default speed , user according to the speed configuration */
        sdio_clock_config(SDIO, SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS_DSPEED);
        sdio_hardware_clock_enable(SDIO);
#endif

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

/*!
    \brief      This function handles SD card interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SDIO0_IRQHandler(void)
{
    // Check for SDIO interrupt flags
    if ((SDIO_STAT(SDIO) & SDIO_STAT_DTEND) != 0) {
        SDIO_INTC(SDIO) = SDIO_INTC_DTENDC;
        /* disable idma for idma transfer */
        SDIO_IDMACTL(SDIO) &= ~SDIO_IDMACTL_IDMAEN;

        SDIO_INTEN(SDIO) &= ~(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        SDIO_IDMACTL(SDIO) &= ~SDIO_CMDCTL_TREN;
        if ((SD_Handle.Operation & 0x02) == (SDIO_DIR_TX << 1)) {
            /* Transfer is complete */
            SD_Handle.TXCplt = 0;
            if ((SD_Handle.Operation & 0x01) == SD_MULTIPLE_BLOCK) {
                /* Send stop command in multiblock write */
                SD_TransmitCommand(SDIO, (SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
        } else {
            /* RX transfer is complete */
            SD_Handle.RXCplt = 0;
            if ((SD_Handle.Operation & 0x01) == SD_MULTIPLE_BLOCK) {
                /* Send stop command in multiblock read */
                SD_TransmitCommand(SDIO, (SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
        }

        SDIO_INTC(SDIO) = SDIO_MASK_DATA_FLAGS;
        SD_Handle.TransferComplete = 1;
        SD_Handle.TransferError = SD_OK;
    } else if( (SDIO_STAT(SDIO) & (SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_RXORE | SDIO_STAT_TXURE)) != 0) {

        if ((SDIO_STAT(SDIO) & SDIO_STAT_DTCRCERR) != 0)
            SD_Handle.TransferError = SD_DATA_CRC_FAIL;
        else if ((SDIO_STAT(SDIO) & SDIO_STAT_DTTMOUT) != 0)
            SD_Handle.TransferError = SD_DATA_TIMEOUT;
        else if ((SDIO_STAT(SDIO) & SDIO_STAT_RXORE) != 0)
            SD_Handle.TransferError = SD_RX_OVERRUN;
        else if ((SDIO_STAT(SDIO) & SDIO_STAT_TXURE) != 0)
            SD_Handle.TransferError = SD_TX_UNDERRUN;

        SDIO_INTC(SDIO) = SDIO_MASK_DATA_FLAGS;
        // Disable all SDIO peripheral interrupt sources
        SDIO_INTEN(SDIO) &= ~(SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |
                        SDIO_INTEN_DTENDIE | SDIO_INTEN_TFHIE | SDIO_INTEN_RFHIE | SDIO_INTEN_TXUREIE |
                        SDIO_INTEN_RXOREIE);
        SDIO_IDMACTL(SDIO) &= ~SDIO_CMDCTL_TREN;
        SDIO_DATACTL(SDIO) |= SDIO_DATACTL_FIFOREST;
        SDIO_DATACTL(SDIO) &= ~SDIO_DATACTL_FIFOREST;
        /* Send stop command to abort transfer on error */
        SD_TransmitCommand(SDIO, (SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
        SDIO_INTC(SDIO) = SDIO_FLAG_DTABORT;
        SDIO_IDMACTL(SDIO) &= ~SDIO_IDMACTL_IDMAEN;

    } else if ((SDIO_STAT(SDIO) & SDIO_INT_FLAG_IDMAERR) != 0){
        SDIO_INTC(SDIO) = SDIO_INT_FLAG_IDMAERR;
        SD_Handle.TransferError = SD_DMA_ERROR;
        // Disable all SDIO peripheral interrupt sources
        SDIO_INTEN(SDIO) &= ~(SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |
                        SDIO_INTEN_DTENDIE | SDIO_INTEN_TFHIE | SDIO_INTEN_RFHIE | SDIO_INTEN_TXUREIE |
                        SDIO_INTEN_RXOREIE);
        SDIO_IDMACTL(SDIO) &= ~SDIO_CMDCTL_TREN;
        SDIO_DATACTL(SDIO) |= SDIO_DATACTL_FIFOREST;
        SDIO_DATACTL(SDIO) &= ~SDIO_DATACTL_FIFOREST;
        /* Send stop command to abort transfer on error */
        SD_TransmitCommand(SDIO, (SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
        SDIO_INTC(SDIO) = SDIO_FLAG_DTABORT;
        SDIO_IDMACTL(SDIO) &= ~SDIO_IDMACTL_IDMAEN;

    }


}

void SDIO1_IRQHandler(void)
{
    SDIO0_IRQHandler();
}

#endif
