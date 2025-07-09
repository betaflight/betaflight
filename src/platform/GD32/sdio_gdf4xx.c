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
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_sdio.h"

#include "pg/sdio.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "drivers/light_led.h"

#include "build/debug.h"

#define DMA_CHANNEL_4                    ((uint32_t)0x08000000)
#define DMA_MINC_ENABLE                  ((uint32_t)DMA_CHXCTL_MNAGA)
#define DMA_MDATAALIGN_WORD              ((uint32_t)DMA_MEMORY_WIDTH_32BIT)
#define DMA_PDATAALIGN_WORD              ((uint32_t)DMA_PERIPH_WIDTH_32BIT)
#define DMA_MBURST_INC4                  ((uint32_t)DMA_MEMORY_BURST_4_BEAT)
#define DMA_PBURST_INC4                  ((uint32_t)DMA_PERIPH_BURST_4_BEAT)

#define BLOCK_SIZE                       ((uint32_t)(512))

#define INTC_CLEAR_MASK_CH3              ((uint32_t)0x0F40000)
#define INTC_CLEAR_MASK_CH6              ((uint32_t)0x003D000)

#define SDIO_INTC_STATIC_FLAGS           ((uint32_t)(SDIO_INTC_CCRCERRC | SDIO_INTC_DTCRCERRC | SDIO_INTC_CMDTMOUTC |\
                                                     SDIO_INTC_DTTMOUTC | SDIO_INTC_TXUREC | SDIO_INTC_RXOREC  |\
                                                     SDIO_INTC_CMDRECVC  | SDIO_INTC_CMDSENDC  | SDIO_INTC_DTENDC  |\
                                                     SDIO_INTC_DTBLKENDC))

#define SD_SOFTWARE_COMMAND_TIMEOUT      ((uint32_t)0x00220000)

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
#define SD_MAX_DATA_LENGTH               ((uint32_t)0x01FFFFFF)

#define SD_CCCC_ERASE                    ((uint32_t)0x00000020)

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
                                                     SDIO_CLKCTL_CLKBYP  | SDIO_CLKCTL_BUSMODE |\
                                                     SDIO_CLKCTL_CLKEDGE | SDIO_CLKCTL_HWCLKEN | SDIO_CLKCTL_DIV8))

#define DATACTRL_CLEAR_MASK              ((uint32_t)(SDIO_DATACTL_DATAEN    | SDIO_DATACTL_DATADIR |\
                                                     SDIO_DATACTL_TRANSMOD  | SDIO_DATACTL_BLKSZ))

#define CMDCTL_CLEAR_MASK                ((uint32_t)(SDIO_CMDCTL_CMDIDX | SDIO_CMDCTL_CMDRESP |\
                                                     SDIO_CMDCTL_INTWAIT  | SDIO_CMDCTL_WAITDEND |\
                                                     SDIO_CMDCTL_CSMEN   | SDIO_CMDCTL_SUSPEND))

#define SDIO_INIT_CLK_DIV                ((uint8_t)0x76)
#define SDIO_CLK_DIV                     ((uint8_t)0x02)

#define SD_CMD_GO_IDLE_STATE            ((uint8_t)0)   // Resets the SD memory card.
#define SD_CMD_SEND_OP_COND             ((uint8_t)1)   // Sends host capacity support information and activates the card's initialization process.
#define SD_CMD_ALL_SEND_CID             ((uint8_t)2)   // Asks any card connected to the host to send the CID numbers on the CMD line.
#define SD_CMD_SET_REL_ADDR             ((uint8_t)3)   // Asks the card to publish a new relative address (RCA).
#define SD_CMD_HS_SWITCH                ((uint8_t)6)   // Checks switchable function (mode 0) and switch card function (mode 1).
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
#define SD_CMD_SD_ERASE_GRP_START       ((uint8_t)32)  // Sets the address of the first write block to be erased. (For SD card only).
#define SD_CMD_SD_ERASE_GRP_END         ((uint8_t)33)  // Sets the address of the last write block of the continuous range to be erased.
                                                       // system set by switch function command (CMD6).
#define SD_CMD_ERASE                    ((uint8_t)38)  // Reserved for SD security applications.
#define SD_CMD_FAST_IO                  ((uint8_t)39)  // SD card doesn't support it (Reserved).
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

#define SDIO_DMA_ST3 1


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
SD_CardInfo_t                      SD_CardInfo;
static uint32_t                    SD_Status;
static uint32_t                    SD_CardRCA;
SD_CardType_t                      SD_CardType;
static volatile uint32_t           TimeOut;
static DMA_Stream_TypeDef          *dmaStream;
static uint32_t                    dma_periph_sdio;
static int                         dma_channel_sdio;


static void             SD_DataTransferInit         (uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard);
static SD_Error_t       SD_TransmitCommand          (uint32_t Command, uint32_t Argument, int8_t ResponseType);
static SD_Error_t       SD_CmdResponse              (uint8_t SD_CMD, int8_t ResponseType);
static void             SD_GetResponse              (uint32_t* pResponse);
static SD_Error_t       CheckOCR_Response           (uint32_t Response_R1);
static void             SD_DMA_Complete             (uint32_t dma_periph, dma_channel_enum channelx);
static SD_Error_t       SD_InitializeCard           (void);

static SD_Error_t       SD_PowerON                  (void);
static SD_Error_t       SD_WideBusOperationConfig   (uint32_t WideMode);
static SD_Error_t       SD_FindSCR                  (uint32_t *pSCR);

void SDIO_DMA_ST3_IRQHandler(dmaChannelDescriptor_t *dma);
void SDIO_DMA_ST6_IRQHandler(dmaChannelDescriptor_t *dma);


/*!
    \brief      Prepare the state machine for transfer
    \param[in]  Size: data transfer size
    \param[in]  DataBlockSize: data block size
    \param[in]  IsItReadFromCard: transfer direction flag
    \param[out] none
    \retval     none
*/
static void SD_DataTransferInit(uint32_t Size, uint32_t DataBlockSize, bool IsItReadFromCard)
 {
    uint32_t Direction;

    SDIO_DATATO = SD_DATATIMEOUT;
    SDIO_DATALEN  = Size;
    Direction      = (IsItReadFromCard == true) ? SDIO_DATACTL_DATADIR : 0;
    SDIO_DATACTL |=  (uint32_t)(DataBlockSize | Direction | SDIO_DATACTL_DATAEN  | 0x01);
    return;
}

/*!
    \brief      Send the command to SDIO
    \param[in]  Command: SDIO command
    \param[in]  Argument: command argument
    \param[in]  ResponseType: response type (must provide the response size)
    \param[out] none
    \retval     SD Card error state
*/
static SD_Error_t SD_TransmitCommand(uint32_t Command, uint32_t Argument, int8_t ResponseType)
{
    SD_Error_t ErrorState;

    SDIO_INTC = SDIO_INTC_STATIC_FLAGS;                                         // Clear the Command Flags
    /* disable the CSM */
    SDIO_CMDCTL &= ~SDIO_CMDCTL_CSMEN;
    SDIO_CMDAGMT = (uint32_t)Argument;                                          // Set the SDIO Argument value
    SDIO_CMDCTL = (uint32_t)(Command | SDIO_CMDCTL_CSMEN);                      // Set SDIO command parameters
    if((Argument == 0) && (ResponseType == 0)) ResponseType = SD_NO_RESPONSE;   // Go idle command
    ErrorState  = SD_CmdResponse(Command & SDIO_CMDCTL_CMDIDX, ResponseType);
    SDIO_INTC = SDIO_INTC_STATIC_FLAGS;                                         // Clear the Command Flags

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
        SD_Status = SDIO_STAT;
        TimeOut--;
    } while(((SD_Status & Flag) == 0) && (TimeOut > 0));

    if(ResponseType <= 0) {
        if(TimeOut == 0) {
            return SD_CMD_RSP_TIMEOUT;
        } else {
            return SD_OK;
        }
    }

    if((SDIO_STAT & SDIO_STAT_CMDTMOUT) != 0) {
        return SD_CMD_RSP_TIMEOUT;
    }

    if(ResponseType == 3) {
        if(TimeOut == 0) {
            return SD_CMD_RSP_TIMEOUT;  // Card is not V2.0 compliant or card does not support the set voltage range
        } else {
            return SD_OK;               // Card is SD V2.0 compliant
        }
    }

    if((SDIO_STAT & SDIO_STAT_CCRCERR) != 0) {
        return SD_CMD_CRC_FAIL;
    }
    if(ResponseType == 2) {
        return SD_OK;
    }
    if((uint8_t)SDIO_RSPCMDIDX != SD_CMD) {
        return SD_ILLEGAL_CMD;
    }

    Response_R1 = SDIO_RESP0;                    // We have received response, retrieve it for analysis

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
    pResponse[0] = SDIO_RESP0;
    pResponse[1] = SDIO_RESP1;
    pResponse[2] = SDIO_RESP2;
    pResponse[3] = SDIO_RESP3;
}

/*!
    \brief      SD DMA transfer complete RX and TX callback
    \param[in]  dma_periph: DMA peripheral
    \param[in]  channelx: DMA channel
    \param[out] none
    \retval     none
*/
static void SD_DMA_Complete(uint32_t dma_periph, dma_channel_enum channelx)
{
    if (SD_Handle.RXCplt) {
            if (SD_Handle.Operation == ((SDIO_DIR_RX << 1) | SD_MULTIPLE_BLOCK)) {
                /* Send stop command in multiblock write */
                SD_TransmitCommand((SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
            }

            SDIO_DATACTL &= (uint32_t)~((uint32_t)SDIO_DATACTL_DMAEN);

            /* Clear all the static flags */
            SDIO_INTC = SDIO_INTC_STATIC_FLAGS;

            /* Clear flag */
            SD_Handle.RXCplt = 0;

            /* Disable the channel */
            DMA_CHCTL(dma_periph, channelx) &= ~DMA_CHXCTL_CHEN;
    } else {
            /* Enable Dataend IE */
            SDIO_INTEN |= SDIO_INTEN_DTENDIE;
    }
}

/*!
    \brief      Prepare the DMA transfer
    \param[in]  pBuffer: pointer to the buffer that will contain the data to transmit
    \param[in]  BlockSize: the SD card Data block size (must be 512 bytes)
    \param[in]  NumberOfBlocks: number of blocks to write
    \param[in]  dir: transfer direction
    \param[out] none
    \retval     none
    \note       BlockSize must be 512 bytes
*/
static SD_Error_t SD_InitializeCard(void)
{
    SD_Error_t ErrorState = SD_OK;

    if((SDIO_PWRCTL & SDIO_PWRCTL_PWRCTL) != 0)
    {
        if(SD_CardType != SD_SECURE_DIGITAL_IO) {
            // Send CMD2 ALL_SEND_CID
            if((ErrorState = SD_TransmitCommand((SD_CMD_ALL_SEND_CID | SD_CMD_RESPONSE_LONG), 0, 2)) != SD_OK) {
                return ErrorState;
            }

            // Get Card identification number data
            SD_GetResponse(SD_Handle.CID);
        }

        if((SD_CardType == SD_STD_CAPACITY_V1_1)    || (SD_CardType == SD_STD_CAPACITY_V2_0) ||
           (SD_CardType == SD_SECURE_DIGITAL_IO_COMBO) || (SD_CardType == SD_HIGH_CAPACITY)) {
            // Send CMD3 SET_REL_ADDR with argument 0
            // SD Card publishes its RCA.
            if((ErrorState = SD_TransmitCommand((SD_CMD_SET_REL_ADDR | SD_CMD_RESPONSE_SHORT), 0, 6)) != SD_OK) {
                return ErrorState;
            }
        }

        if(SD_CardType != SD_SECURE_DIGITAL_IO) {
            // Send CMD9 SEND_CSD with argument as card's RCA
            if((ErrorState = SD_TransmitCommand((SD_CMD_SEND_CSD | SD_CMD_RESPONSE_LONG), SD_CardRCA, 2)) == SD_OK) {
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
    SDIO_DATACTL                = 0;                                                                 // Initialize data control register
    SD_Handle.TransferComplete = 0;                                                                  // Initialize handle flags
    SD_Handle.TransferError    = SD_OK;
    SD_Handle.Operation        = (NumberOfBlocks > 1) ? SD_MULTIPLE_BLOCK : SD_SINGLE_BLOCK;         // Initialize SD Read operation
    SD_Handle.Operation       |= dir << 1;
    SDIO_INTEN                 = 0;
    if (dir == SDIO_DIR_RX) {
        SDIO_INTEN            |= (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |                     // Enable transfer interrupts
                                      SDIO_INTEN_DTENDIE  | SDIO_INTEN_RXOREIE);
    } else {
        SDIO_INTEN            |= (SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |                     // Enable transfer interrupts
                                      SDIO_INTEN_TXUREIE);
    }
    if (dir == SDIO_DIR_TX) {
        SDIO_DATACTL               |= SDIO_DATACTL_DMAEN;                                              // Enable SDIO DMA transfer
    }
    DMA_CHCTL(dma_periph_sdio,dma_channel_sdio)                 &= ~DMA_CHXCTL_CHEN;                                                      // Disable the Peripheral
    while (DMA_CHCTL(dma_periph_sdio,dma_channel_sdio) & DMA_CHXCTL_CHEN);
    DMA_CHCNT(dma_periph_sdio,dma_channel_sdio)                 = (uint32_t) (BlockSize * NumberOfBlocks) / 4;                       // Configure DMA Stream data length
    DMA_CHM0ADDR(dma_periph_sdio,dma_channel_sdio)              = (uint32_t) pBuffer;                                                // Configure DMA Stream memory address
    if (dir == SDIO_DIR_RX) {
       DMA_CHCTL(dma_periph_sdio,dma_channel_sdio)  &= ~(0x01U << 6U);                                                               // Sets peripheral to memory
    } else {
       DMA_CHCTL(dma_periph_sdio,dma_channel_sdio)  |= DMA_MEMORY_TO_PERIPH;                                                         // Sets memory to peripheral
    }
    if ((uint32_t)dmaStream == DMA1_CH3_BASE) {
            // clear dma flags
            dma_flag_clear(DMA1, DMA1_CH3, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);
    } else {
            // clear dma flags
            dma_flag_clear(DMA1, DMA1_CH6, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);                         // Clear the transfer error flag
    }
    DMA_CHCTL(dma_periph_sdio,dma_channel_sdio)                 |= DMA_CHXCTL_FTFIE | DMA_CHXCTL_HTFIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_SDEIE;    // Enable all interrupts
    DMA_CHFCTL(dma_periph_sdio,dma_channel_sdio)                |= DMA_CHXFCTL_FEEIE;
    DMA_CHCTL(dma_periph_sdio,dma_channel_sdio)                 |= DMA_CHXCTL_CHEN;                                                       // Enable the Peripheral
    if (dir == SDIO_DIR_RX) {
        SDIO_DATACTL               |= SDIO_DATACTL_DMAEN;                                              // Enable SDIO DMA transfer
    }
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

    if(SD_CardType != SD_HIGH_CAPACITY)
    {
        ReadAddress *= 512;
    }

    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_RX);

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(BlockSize * NumberOfBlocks, SD_DATABLOCK_SIZE_512B, true);

    // Set Block Size for Card
    ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), BlockSize, 1);

    // Send CMD18 READ_MULT_BLOCK with argument data address
    // or send CMD17 READ_SINGLE_BLOCK depending on number of block
    uint8_t retries = 10;
    CmdIndex   = (NumberOfBlocks > 1) ? SD_CMD_READ_MULT_BLOCK : SD_CMD_READ_SINGLE_BLOCK;
    do {
            ErrorState = SD_TransmitCommand((CmdIndex | SD_CMD_RESPONSE_SHORT), (uint32_t)ReadAddress, 1);
            if (ErrorState != SD_OK && retries--) {
                ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
    } while (ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
            SD_Handle.RXCplt = 0;
    }

    // Update the SD transfer error in SD handle
    SD_Handle.TransferError = ErrorState;

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

    //printf("Reading at %ld into %p %ld blocks\n", (uint32_t)WriteAddress, (void*)buffer, NumberOfBlocks);

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
            ErrorState = SD_TransmitCommand((CmdIndex | SD_CMD_RESPONSE_SHORT), (uint32_t)WriteAddress, 1);
            if (ErrorState != SD_OK && retries--) {
                ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
    } while(ErrorState != SD_OK && retries);

    if (ErrorState != SD_OK) {
            SD_Handle.TXCplt = 0;
            return ErrorState;
    }

    SD_StartBlockTransfert(buffer, BlockSize, NumberOfBlocks, SDIO_DIR_TX);

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(BlockSize * NumberOfBlocks, SD_DATABLOCK_SIZE_512B, false);

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
    uint32_t   reg;
    uint32_t   SCR[2] = {0, 0};

    if((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0) || 
        (SD_CardType == SD_HIGH_CAPACITY)) {

        if(WideMode == SD_BUS_WIDE_8B) {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        } else if((WideMode == SD_BUS_WIDE_4B) || (WideMode == SD_BUS_WIDE_1B)) {
            if((SDIO_RESP0 & SD_CARD_LOCKED) != SD_CARD_LOCKED) {
                // Get SCR Register
                ErrorState = SD_FindSCR(SCR);
                if(ErrorState == SD_OK) {
                    Temp = (WideMode == SD_BUS_WIDE_4B) ? SD_WIDE_BUS_SUPPORT : SD_SINGLE_BUS_SUPPORT;

                    // If requested card supports wide bus operation
                    if((SCR[1] & Temp) != SD_ALLZERO) {
                        // Send CMD55 APP_CMD with argument as card's RCA.
                        ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);
                        if(ErrorState == SD_OK) {
                            Temp = (WideMode == SD_BUS_WIDE_4B) ? 2 : 0;

                            // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
                            ErrorState =  SD_TransmitCommand((SD_CMD_APP_SD_SET_BUSWIDTH | SD_CMD_RESPONSE_SHORT), Temp, 1);
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
            // Configure the SDIO peripheral, we need this delay for some reason...
            while ((SDIO_CLKCTL & 0x800) != WideMode) {
                reg = SDIO_CLKCTL;
                reg &= ~CLKCTL_CLEAR_MASK;
                reg |=(uint32_t) WideMode;
                    SDIO_CLKCTL = reg;
            }
        }
    } else {
            ErrorState = SD_UNSUPPORTED_FEATURE;
    }

    return ErrorState;
}

static SD_Error_t SD_HighSpeed(void)
{
    SD_Error_t  ErrorState;
    uint8_t     SD_hs[64]  = {0};
    uint32_t    SD_scr[2]  = {0, 0};
    uint32_t    SD_SPEC    = 0;
    uint32_t    Count      = 0;
    uint32_t*   Buffer     = (uint32_t *)SD_hs;

    // Initialize the Data control register
    SDIO_DATACTL = 0;

    // Get SCR Register
    if((ErrorState = SD_FindSCR(SD_scr)) != SD_OK) {
        return ErrorState;
    }

    // Test the Version supported by the card
    SD_SPEC = (SD_scr[1]  & 0x01000000) | (SD_scr[1]  & 0x02000000);

    if(SD_SPEC != SD_ALLZERO) {
        // Set Block Size for Card
        if((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK) {
            return ErrorState;
        }

        // Configure the SD DPSM (Data Path State Machine)
        SD_DataTransferInit(64, SD_DATABLOCK_SIZE_64B, true);

        // Send CMD6 switch mode
        if((ErrorState =SD_TransmitCommand((SD_CMD_HS_SWITCH | SD_CMD_RESPONSE_SHORT), 0x80FFFF01, 1)) != SD_OK) {
            return ErrorState;
        }

        while((SDIO_STAT & (SDIO_STAT_RXORE | SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_DTBLKEND)) == 0) {
            if((SDIO_STAT & SDIO_STAT_RFH) != 0) {
                for(Count = 0; Count < 8; Count++) {
                    *(Buffer + Count) = SDIO_FIFO;
                }

                Buffer += 8;
            }
        }

        if((SDIO_STAT & SDIO_STAT_DTTMOUT) != 0)        return SD_DATA_TIMEOUT;
        else if((SDIO_STAT & SDIO_STAT_DTCRCERR) != 0)   return SD_DATA_CRC_FAIL;
        else if((SDIO_STAT & SDIO_STAT_RXORE) != 0)    return SD_RX_OVERRUN;

        Count = SD_DATATIMEOUT;

        while(((SDIO_STAT & SDIO_STAT_RXDTVAL) != 0) && (Count > 0)) {
            *Buffer = SDIO_FIFO;
            Buffer++;
            Count--;
        }

        // Test if the switch mode HS is ok
        if((SD_hs[13] & 2) != 2) {
            ErrorState = SD_UNSUPPORTED_FEATURE;
        }
    }

    return ErrorState;
}

static SD_Error_t SD_GetStatus(void)
{
    SD_Error_t     ErrorState;
    uint32_t       Response1;
    SD_CardState_t CardState;

    // Send Status command
    if((ErrorState = SD_TransmitCommand((SD_CMD_SEND_STATUS | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK) {
        Response1 = SDIO_RESP0;
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
    if((SDIO_RESP0 & SD_CARD_LOCKED) == SD_CARD_LOCKED) {
        return SD_LOCK_UNLOCK_FAILED;
    }

    // Set block size for card if it is not equal to current block size for card
    if((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 64, 1)) != SD_OK) {
        return ErrorState;
    }

    // Send CMD55
    if((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) != SD_OK) {
        return ErrorState;
    }

    // Configure the SD DPSM (Data Path State Machine)
    SD_DataTransferInit(64, SD_DATABLOCK_SIZE_64B, true);

    // Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA
    if((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_STATUS | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK) {
        return ErrorState;
    }

    // Get status data
    while((SDIO_STAT & (SDIO_STAT_RXORE | SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_DTBLKEND)) == 0) {
        if((SDIO_STAT & SDIO_STAT_RFH) != 0) {
            for(Count = 0; Count < 8; Count++) {
                Status[Count] = SDIO_FIFO;
            }
        }
    }

    if((SDIO_STAT & SDIO_STAT_DTTMOUT) != 0)         return SD_DATA_TIMEOUT;
    else if((SDIO_STAT & SDIO_STAT_DTCRCERR) != 0)    return SD_DATA_CRC_FAIL;
    else if((SDIO_STAT & SDIO_STAT_RXORE) != 0)     return SD_RX_OVERRUN;
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

    SDIO_CLKCTL &= ~SDIO_CLKCTL_CLKEN;        // Disable SDIO Clock
    SDIO_PWRCTL  = SDIO_PWRCTL_PWRCTL;        // Set Power State to ON

    delay(2);

    SDIO_CLKCTL |= SDIO_CLKCTL_CLKEN;         // Enable SDIO Clock

    // CMD0: GO_IDLE_STATE -----------------------------------------------------
    // No CMD response required
    if((ErrorState = SD_TransmitCommand(SD_CMD_GO_IDLE_STATE, 0, 0)) != SD_OK) {
        // CMD Response Timeout (wait for CMDSENT flag)
        return ErrorState;
    }

    // CMD8: SEND_IF_COND ------------------------------------------------------
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //- [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //- [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7 */
    if((ErrorState = SD_TransmitCommand((SD_SDIO_SEND_IF_COND | SD_CMD_RESPONSE_SHORT), SD_CHECK_PATTERN, 7)) == SD_OK) {
        // SD Card 2.0
        SD_CardType = SD_STD_CAPACITY_V2_0;
        SD_Type     = SD_RESP_HIGH_CAPACITY;
    }

    // Send CMD55
    // If ErrorState is Command Timeout, it is a MMC card
    // If ErrorState is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x
    if((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) == SD_OK) {
        // SD CARD
        // Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
        while((ValidVoltage == 0) && (Count < SD_MAX_VOLT_TRIAL)) {
            // SEND CMD55 APP_CMD with RCA as 0
            if((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), 0, 1)) != SD_OK) {
                return ErrorState;
            }

            // Send CMD41
            if((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_OP_COND | SD_CMD_RESPONSE_SHORT), SD_VOLTAGE_WINDOW_SD | SD_Type, 3)) != SD_OK) {
                return ErrorState;
            }

            Response = SDIO_RESP0;                                  // Get command response
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
    if((ErrorState = SD_TransmitCommand((SD_CMD_SET_BLOCKLEN | SD_CMD_RESPONSE_SHORT), 8, 1)) == SD_OK) {
        // Send CMD55 APP_CMD with argument as card's RCA
        if((ErrorState = SD_TransmitCommand((SD_CMD_APP_CMD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1)) == SD_OK) {
            SD_DataTransferInit(8, SD_DATABLOCK_SIZE_8B, true);

            // Send ACMD51 SD_APP_SEND_SCR with argument as 0
            if((ErrorState = SD_TransmitCommand((SD_CMD_SD_APP_SEND_SCR | SD_CMD_RESPONSE_SHORT), 0, 1)) == SD_OK) {
                while((SDIO_STAT & (SDIO_STAT_RXORE | SDIO_STAT_DTCRCERR | SDIO_STAT_DTTMOUT | SDIO_STAT_DTBLKEND)) == 0) {
                    if((SDIO_STAT & SDIO_STAT_RXDTVAL) != 0) {
                        *(tempscr + Index) = SDIO_FIFO;
                        Index++;
                    }
                }

                if     ((SDIO_STAT & SDIO_STAT_DTTMOUT) != 0) ErrorState = SD_DATA_TIMEOUT;
                else if((SDIO_STAT & SDIO_STAT_DTCRCERR) != 0) ErrorState = SD_DATA_CRC_FAIL;
                else if((SDIO_STAT & SDIO_STAT_RXORE)  != 0) ErrorState = SD_RX_OVERRUN;
                else if((SDIO_STAT & SDIO_STAT_RXDTVAL)   != 0) ErrorState = SD_OUT_OF_BOUND;
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

/*!
    \brief      Initialize the SDIO module, DMA, and IO
    \param[in]  dma: DMA stream pointer
    \param[out] none
    \retval     initialization success status
*/
bool SD_Initialize_LL(DMA_Stream_TypeDef *dma)
{
    const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier((dmaResource_t *)dma);

    dmaStream = dma;

    if (!((uint32_t)dma == DMA1_CH3_BASE || (uint32_t)dma == DMA1_CH6_BASE) || !dmaAllocate(dmaIdentifier, OWNER_SDCARD, 0)) {
        return false;
    }

    // Reset SDIO Module
    RCU_APB2RST |=  RCU_APB2RST_SDIORST;
    delay(1);
    RCU_APB2RST &= ~RCU_APB2RST_SDIORST;
    delay(1);

    // Enable SDIO clock
    RCU_APB2EN |= RCU_APB2EN_SDIOEN;

    // Enable DMA2 clocks
    RCU_AHB1EN |= RCU_AHB1EN_DMA1EN;

    //Configure Pins
    RCU_AHB1EN |= RCU_AHB1EN_PCEN | RCU_AHB1EN_PDEN;

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

#define SDIO_DATA       IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_25MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SDIO_CMD        IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_25MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SDIO_CLK        IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_25MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)

    IOConfigGPIOAF(d0, SDIO_DATA, GPIO_AF_12);

    if (is4BitWidth) {
        IOConfigGPIOAF(d1, SDIO_DATA, GPIO_AF_12);
        IOConfigGPIOAF(d2, SDIO_DATA, GPIO_AF_12);
        IOConfigGPIOAF(d3, SDIO_DATA, GPIO_AF_12);
    }

    IOConfigGPIOAF(clk, SDIO_CLK, GPIO_AF_12);
    IOConfigGPIOAF(cmd, SDIO_CMD, GPIO_AF_12);

    // NVIC configuration for SDIO interrupts
    nvic_irq_enable(SDIO_IRQn, 1, 0);

    gd32_dma_chbase_parse((uint32_t)dma, &dma_periph_sdio, &dma_channel_sdio);

    RCU_AHB1EN |= RCU_AHB1EN_DMA1EN;
    // Initialize DMA
    DMA_CHCTL(dma_periph_sdio,dma_channel_sdio) = 0;      // Reset DMA Stream control register
    DMA_CHPADDR(dma_periph_sdio,dma_channel_sdio)  = (uint32_t)&SDIO_FIFO;

    if ((uint32_t)dmaStream == DMA1_CH3_BASE) {
        DMA_INTC0(dma_periph_sdio) = INTC_CLEAR_MASK_CH3; // Clear all interrupt flags
    } else {
        DMA_INTC1(dma_periph_sdio) = INTC_CLEAR_MASK_CH6; // Clear all interrupt flags
    }

    DMA_CHCTL(dma_periph_sdio,dma_channel_sdio) = (DMA_CHANNEL_4 | DMA_CHXCTL_TFCS |       // Prepare the DMA Stream configuration
                                                   DMA_MINC_ENABLE | DMA_PDATAALIGN_WORD | // And write to DMA Stream CR register
                                                   DMA_MDATAALIGN_WORD | DMA_PRIORITY_ULTRA_HIGH |
                                                   DMA_MBURST_INC4 | DMA_PBURST_INC4 |
                                                   DMA_MEMORY_TO_PERIPH);
    DMA_CHFCTL(dma_periph_sdio,dma_channel_sdio)  = (DMA_CHXFCTL_MDMEN | DMA_FIFO_4_WORD); // Configuration FIFO control register
    dmaEnable(dmaIdentifier);

    if ((uint32_t)dmaStream == DMA1_CH3_BASE) {
        dmaSetHandler(dmaIdentifier, SDIO_DMA_ST3_IRQHandler, 1, 0);
    } else {
        dmaSetHandler(dmaIdentifier, SDIO_DMA_ST6_IRQHandler, 1, 0);
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
    reg = SDIO_CLKCTL;
    reg &= ~CLKCTL_CLEAR_MASK;
    reg |= (uint32_t)SDIO_INIT_CLK_DIV;
    SDIO_CLKCTL = reg;

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
    errorState = SD_TransmitCommand((SD_CMD_SEL_DESEL_CARD | SD_CMD_RESPONSE_SHORT), SD_CardRCA, 1);
    // Configure SDIO peripheral interface.
    reg = SDIO_CLKCTL;
    reg &= ~CLKCTL_CLEAR_MASK;
    reg |= (uint32_t) SDIO_CLK_DIV;
    SDIO_CLKCTL = reg;

    // Configure SD Bus width.
    if (errorState == SD_OK) {
        // Enable wide operation.
        if (sdioConfig()->use4BitWidth) {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_4B);
        } else {
            errorState = SD_WideBusOperationConfig(SD_BUS_WIDE_1B);
        }

        if (errorState == SD_OK && sdioConfig()->clockBypass) {
            if (SD_HighSpeed()) {
                SDIO_CLKCTL |= SDIO_CLKCTL_CLKBYP;
                SDIO_CLKCTL |= SDIO_CLKCTL_CLKEDGE;
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

/*!
    \brief      This function handles SD card interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SDIO_IRQHandler(void)
{
    // Check for SDIO interrupt flags
    if ((SDIO_STAT & SDIO_STAT_DTEND) != 0) {
        SDIO_INTC = SDIO_INTC_DTENDC;
        SDIO_INTC = SDIO_INTC_STATIC_FLAGS;
        SDIO_INTEN &= ~(SDIO_INTEN_DTENDIE | SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |
                        SDIO_INTEN_TXUREIE | SDIO_INTEN_RXOREIE | SDIO_INTEN_TFHIE | SDIO_INTEN_RFHIE);

        /* Currently doesn't implement multiple block write handling */
        if ((SD_Handle.Operation & 0x02) == (SDIO_DIR_TX << 1)) {
            /* Disable the stream */
            DMA_CHCTL(dma_periph_sdio,dma_channel_sdio) &= ~DMA_CHXCTL_CHEN;
            SDIO_DATACTL &= ~(SDIO_DATACTL_DMAEN);
            /* Transfer is complete */
            SD_Handle.TXCplt = 0;
            if ((SD_Handle.Operation & 0x01) == SD_MULTIPLE_BLOCK) {
                /* Send stop command in multiblock write */
                SD_TransmitCommand((SD_CMD_STOP_TRANSMISSION | SD_CMD_RESPONSE_SHORT), 0, 1);
            }
        }

        SD_Handle.TransferComplete = 1;
        SD_Handle.TransferError = SD_OK;
    } else if ((SDIO_STAT & SDIO_STAT_DTCRCERR) != 0)
        SD_Handle.TransferError = SD_DATA_CRC_FAIL;
    else if ((SDIO_STAT & SDIO_STAT_DTTMOUT) != 0)
        SD_Handle.TransferError = SD_DATA_TIMEOUT;
    else if ((SDIO_STAT & SDIO_STAT_RXORE) != 0)
        SD_Handle.TransferError = SD_RX_OVERRUN;
    else if ((SDIO_STAT & SDIO_STAT_TXURE) != 0)
        SD_Handle.TransferError = SD_TX_UNDERRUN;

    SDIO_INTC = SDIO_INTC_STATIC_FLAGS;

    // Disable all SDIO peripheral interrupt sources
    SDIO_INTEN &= ~(SDIO_INTEN_DTCRCERRIE | SDIO_INTEN_DTTMOUTIE |
                    SDIO_INTEN_DTENDIE |
                    SDIO_INTEN_TFHIE | SDIO_INTEN_RFHIE | SDIO_INTEN_TXUREIE |
                    SDIO_INTEN_RXOREIE);
}

/*!
    \brief      This function handles DMA2 Stream 3 interrupt request
    \param[in]  dma: DMA channel descriptor
    \param[out] none
    \retval     none
*/
void SDIO_DMA_ST3_IRQHandler(dmaChannelDescriptor_t *dma)
{
    UNUSED(dma);
    // Transfer Error Interrupt management
    if((DMA_INTF0(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_TAE, DMA_CH3)) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_TAEIE) != 0) {
            DMA_CHCTL(DMA1, DMA_CH3)   &= ~DMA_CHXCTL_TAEIE;            // Disable the transfer error interrupt
            DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_TAE, DMA_CH3);  // Clear the transfer error flag
        }
    }

    // FIFO Error Interrupt management
    if((DMA_INTF0(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_FEE, DMA_CH3)) != 0) {
        if((DMA_CHFCTL(DMA1, DMA_CH3) & DMA_CHXFCTL_FEEIE) != 0) {
            DMA_CHFCTL(DMA1, DMA_CH3)   &= ~DMA_CHXFCTL_FEEIE;          // Disable the FIFO Error interrupt
            DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_FEE, DMA_CH3);  // Clear the FIFO error flag
        }
    }

    // Single data mode exception flag
    if((DMA_INTF0(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_SDE, DMA_CH3)) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_SDEIE) != 0) {
            DMA_CHCTL(DMA1, DMA_CH3)   &= ~DMA_CHXCTL_SDEIE;          // Disable the single data mode Error interrupt
            DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_SDE, DMA_CH3);
        }
    }

    // Half Transfer Complete Interrupt management
    if((DMA_INTF0(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_HTF, DMA_CH3)) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_HTFIE) != 0) {
            if(((DMA_CHCTL(DMA1, DMA_CH3)) & (uint32_t)(DMA_CHXCTL_SBMEN)) != 0) {
                DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_HTF, DMA_CH3);
            } else {
                if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_CMEN) == 0) {
                    DMA_CHCTL(DMA1, DMA_CH3)   &= ~DMA_CHXCTL_HTFIE;
                }

                DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_HTF, DMA_CH3);
            }
        }
    }

    // Transfer Complete Interrupt management
    if((DMA_INTF0(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_FTF, DMA_CH3)) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_FTFIE) != 0) {
            if((DMA_CHCTL(DMA1, DMA_CH3) & (uint32_t)(DMA_CHXCTL_SBMEN)) != 0) {
                DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_FTF, DMA_CH3);
            } else {
                if((DMA_CHCTL(DMA1, DMA_CH3) & DMA_CHXCTL_CMEN) == 0) {
                    DMA_CHCTL(DMA1, DMA_CH3) &= ~DMA_CHXCTL_FTFIE;
                }

                DMA_INTC0(DMA1) = DMA_FLAG_ADD(DMA_INT_FLAG_FTF, DMA_CH3);
                SD_DMA_Complete(DMA1, DMA_CH3);
            }
        }
    }
}

/*!
    \brief      This function handles DMA2 Stream 6 interrupt request
    \param[in]  dma: DMA channel descriptor
    \param[out] none
    \retval     none
*/
void SDIO_DMA_ST6_IRQHandler(dmaChannelDescriptor_t *dma)
{
    UNUSED(dma);

    dma_channel_enum channel_flag_offset = channelx;

    // Transfer Error Interrupt management
    if((DMA_INTF1(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_TAE, (DMA_CH6-DMA_CH4))) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH6)& DMA_CHXCTL_TAEIE) != 0) {
            DMA_CHCTL(DMA1, DMA_CH6)  &= ~DMA_CHXCTL_TAEIE;
            DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_TAE, (DMA_CH6-DMA_CH4));
        }
    }

    // FIFO Error Interrupt management
    if((DMA_INTF1(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_FEE, (DMA_CH6-DMA_CH4))) != 0) {
        if((DMA_CHFCTL(DMA1, DMA_CH6) & DMA_CHXFCTL_FEEIE) != 0) {
            DMA_CHFCTL(DMA1, DMA_CH6)   &= ~DMA_CHXFCTL_FEEIE;
            DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_FEE, (DMA_CH6-DMA_CH4));
        }
    }

    // Single data mode exception flag
    if((DMA_INTF1(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_SDE, (DMA_CH6-DMA_CH4))) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH6)& DMA_CHXCTL_SDEIE) != 0) {
            DMA_CHCTL(DMA1, DMA_CH6)  &= ~DMA_CHXCTL_SDEIE;
            DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_SDE, (DMA_CH6-DMA_CH4));
        }
    }

    // Half Transfer Complete Interrupt management
    if((DMA_INTF1(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_HTF, (DMA_CH6-DMA_CH4))) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH6) & DMA_CHXCTL_HTFIE) != 0) {
            if(((DMA_CHCTL(DMA1, DMA_CH6)) & (uint32_t)(DMA_CHXCTL_SBMEN)) != 0) {
                DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_HTF, (DMA_CH6-DMA_CH4));
            } else {
                if((DMA_CHCTL(DMA1, DMA_CH6)& DMA_CHXCTL_CMEN) == 0) {
                    DMA_CHCTL(DMA1, DMA_CH6)&= ~DMA_CHXCTL_HTFIE;
                }

                DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_HTF, (DMA_CH6-DMA_CH4));
            }
        }
    }

    // Transfer Complete Interrupt management
    if((DMA_INTF1(DMA1) & DMA_FLAG_ADD(DMA_INT_FLAG_FTF, (DMA_CH6-DMA_CH4))) != 0) {
        if((DMA_CHCTL(DMA1, DMA_CH6)& DMA_CHXCTL_FTFIE) != 0) {
            if((DMA_CHCTL(DMA1, DMA_CH6)& (uint32_t)(DMA_CHXCTL_SBMEN)) != 0) {
                DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_FTF, (DMA_CH6-DMA_CH4));
            } else {
                if((DMA_CHCTL(DMA1, DMA_CH6)& DMA_CHXCTL_CMEN) == 0) {
                    DMA_CHCTL(DMA1, DMA_CH6)  &= ~DMA_CHXCTL_FTFIE;
                }

                DMA_INTC1(dma_periph_sdio) = DMA_FLAG_ADD(DMA_INT_FLAG_FTF, (DMA_CH6-DMA_CH4));
                SD_DMA_Complete(DMA1, DMA_CH6);
            }
        }
    }
}

#endif
