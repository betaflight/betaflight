/**
  *
  * @file    apm32f4xx_dal_qspi.h
  * @brief   Header file of QSPI DAL module.
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023-2024 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APM32F4XX_DAL_QSPI_H
#define __APM32F4XX_DAL_QSPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

#if defined (QSPI)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup QSPI
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/** @defgroup QSPI_Exported_Types QSPI Exported Types
  * @{
  */

/**
 * @brief  QSPI Init structure definition
 */
typedef struct
{
    uint32_t ClockPrescaler;        /*!< Specifies the prescaler factor for generating clock based on the AHB clock.
                                         This parameter can be a even number between 2 and 65534. */

    uint32_t ClockPhase;            /*!< Specifies the clock data sample edge.
                                         This parameter can be a value of @ref QSPI_ClockPhase */

    uint32_t ClockPolarity;         /*!< Specifies the clock valid level in the idle state.
                                         This parameter can be a value of @ref QSPI_ClockPolarity */

    uint32_t ClockStretch;          /*!< Specifies the clock stretch enable/disable.
                                         This parameter can be a value of ENABLE or DISABLE */

    uint32_t TxFifoThreshold;       /*!< Specifies the threshold level of the transmit FIFO.
                                         This parameter can be a value between 0 and 7. */

    uint32_t TxFifoLevel;           /*!< Specifies the level of the transmit FIFO startup level.
                                         This parameter can be a value between 0 and 7. */

    uint32_t RxFifoThreshold;       /*!< Specifies the threshold level of the receive FIFO.
                                            This parameter can be a value between 0 and 7. */

    uint32_t ChipSelectToggle;      /*!< Specifies the Chip Select Toggle.
                                         This parameter can be a value of @ref QSPI_ChipSelectToggle */
} QSPI_InitTypeDef;

/**
 * @brief DAL QSPI State structures definition
 */
typedef enum
{
    DAL_QSPI_STATE_RESET             = 0x00,  /*!< QSPI not yet initialized or disabled                 */
    DAL_QSPI_STATE_READY             = 0x01,  /*!< QSPI initialized and ready for use                   */
    DAL_QSPI_STATE_BUSY              = 0x02,  /*!< QSPI internal process is ongoing                     */
    DAL_QSPI_STATE_BUSY_TX           = 0x12,  /*!< Data Transmission process is ongoing                 */
    DAL_QSPI_STATE_BUSY_RX           = 0x22,  /*!< Data Reception process is ongoing                    */
    DAL_QSPI_STATE_BUSY_TX_RX        = 0x42,  /*!< Data Transmission and Reception process is ongoing   */
    DAL_QSPI_STATE_ERROR             = 0x04,  /*!< QSPI error state                                     */
    DAL_QSPI_STATE_ABORT             = 0x08,  /*!< QSPI abort is ongoing                                */
} DAL_QSPI_StateTypeDef;

/**
 * @brief  QSPI handle Structure definition
 */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
typedef struct __QSPI_HandleTypeDef
#else
typedef struct
#endif
{
    QSPI_TypeDef                *Instance;      /*!< QSPI registers base address               */

    QSPI_InitTypeDef            Init;           /*!< QSPI communication parameters             */

    uint8_t                     *pTxBuffPtr;    /*!< Pointer to QSPI Tx transfer Buffer        */

    __IO uint32_t               TxXferSize;     /*!< QSPI Tx Transfer size                     */

    __IO uint32_t               TxXferCount;    /*!< QSPI Tx Transfer Counter                  */

    uint8_t                     *pRxBuffPtr;    /*!< Pointer to QSPI Rx transfer Buffer        */

    __IO uint32_t               RxXferSize;     /*!< QSPI Rx Transfer size                     */

    __IO uint32_t               RxXferCount;    /*!< QSPI Rx Transfer Counter                  */

    DMA_HandleTypeDef          *hdmatx;         /*!< QSPI Tx DMA handle parameters             */

    DMA_HandleTypeDef          *hdmarx;         /*!< QSPI Rx DMA handle parameters             */

    __IO DAL_LockTypeDef        Lock;           /*!< QSPI locking object                       */

    __IO DAL_QSPI_StateTypeDef  State;          /*!< QSPI communication state                  */

    __IO uint32_t               ErrorCode;      /*!< QSPI Error code                           */

    uint32_t                    Timeout;        /*!< QSPI timeout value                        */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
    void (* ErrorCallback)                  (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Error Callback               */
    void (* AbortCpltCallback)              (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Abort Complete Callback      */
    void (* CmdCpltCallback)                (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Command Complete Callback    */
    void (* RxCpltCallback)                 (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Rx Complete Callback         */
    void (* TxCpltCallback)                 (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Tx Complete Callback         */
    void (* TxRxCpltCallback)               (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Transfer Complete Callback   */

    void (* MspInitCallback)                (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Msp Init callback            */
    void (* MspDeInitCallback)              (struct __QSPI_HandleTypeDef *hqspi);               /*!< QSPI Msp DeInit callback          */
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
} QSPI_HandleTypeDef;

/**
  * @brief  QSPI Command structure definition
  */
typedef struct
{
    uint32_t Instruction;       /*!< Specifies the Instruction to be sent.
                                     This parameter can be a value between 0x00 and 0xFF */

    uint32_t Address;           /*!< Specifies the Address to be sent.
                                     This parameter can be a value between 0x00 and 0xFFFFFFFF */

    uint32_t AddressSize;       /*!< Specifies the Address Size.
                                     This parameter can be a value of @ref QSPI_AddressSize */

    uint32_t InstructionMode;   /*!< Specifies the Instruction Mode.
                                     This parameter can be a value of @ref QSPI_InstructionMode */

    uint32_t InstructionSize;   /*!< Specifies the Instruction Size.
                                     This parameter can be a value of @ref QSPI_InstructionSize */

    uint32_t TransferMode;      /*!< Specifies the Transfer Mode.
                                     This parameter can be a value of @ref QSPI_TransferMode */

    uint32_t FrameFormat;       /*!< Specifies the frame format.
                                         This parameter can be a value of @ref QSPI_FrameFormat */

    uint32_t DataFrameSize;     /*!< Specifies the data frame size.
                                         This parameter can be a value of @ref QSPI_DataFrameSize */

    uint32_t DummyCycles;       /*!< Specifies the Number of Dummy Cycles.
                                     This parameter can be a number between 0 and 31 */

    uint32_t NbData;            /*!< Specifies the Number of Data to transfer.
                                     This parameter can be a number between 0 and 0xFFFF */
} QSPI_CommandTypeDef;

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL QSPI Callback ID enumeration definition
  */
typedef enum
{
    DAL_QSPI_ERROR_CB_ID                   = 0x00U,    /*!< QSPI Error Callback ID               */
    DAL_QSPI_ABORT_CB_ID                   = 0x01U,    /*!< QSPI Abort Callback ID               */
    DAL_QSPI_CMD_CPLT_CB_ID                = 0x02U,    /*!< QSPI Command Complete Callback ID    */
    DAL_QSPI_RX_CPLT_CB_ID                 = 0x03U,    /*!< QSPI Rx Complete Callback ID         */
    DAL_QSPI_TX_CPLT_CB_ID                 = 0x04U,    /*!< QSPI Tx Complete Callback ID         */
    DAL_QSPI_TX_RX_CPLT_CB_ID              = 0x05U,    /*!< QSPI Transfer Complete Callback ID   */

    DAL_QSPI_MSP_INIT_CB_ID                = 0x06U,    /*!< QSPI MspInit callback ID             */
    DAL_QSPI_MSP_DEINIT_CB_ID              = 0x07U     /*!< QSPI MspDeInit callback ID           */
} DAL_QSPI_CallbackIDTypeDef;

/**
  * @brief  DAL QSPI Callback pointer definition
  */
typedef  void (*pQSPI_CallbackTypeDef)(QSPI_HandleTypeDef *hqspi); /*!< pointer to an QSPI callback function */
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

/**
* @}
*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @defgroup QSPI_Error_Code QSPI Error Code
 * @{
 */
#define DAL_QSPI_ERROR_NONE              0x00000000U    /*!< No error              */
#define DAL_QSPI_ERROR_TIMEOUT           0x00000001U    /*!< Timeout error         */
#define DAL_QSPI_ERROR_TRANSFER          0x00000002U    /*!< Transfer error        */
#define DAL_QSPI_ERROR_DMA               0x00000004U    /*!< DMA transfer error    */
#define DAL_QSPI_ERROR_TX_OVR            0x00000008U    /*!< Transmit FIFO Overrun */
#define DAL_QSPI_ERROR_RX_OVR            0x00000010U    /*!< Receive FIFO Overrun  */
#define DAL_QSPI_ERROR_RX_UDR            0x00000020U    /*!< Receive FIFO UnderRun */
#define DAL_QSPI_ERROR_MST               0x00000040U    /*!< Master operation error*/
#define DAL_QSPI_ERROR_INVALID_PARAM     0x00000080U    /*!< Invalid parameter error */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
#define DAL_QSPI_ERROR_INVALID_CALLBACK  0x00000100U    /*!< Invalid Callback error */
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup QSPI_FrameFormat QSPI Frame Format
 * @{
 */
#define QSPI_FRAME_FORMAT_STANDARD    0x00000000U               /*!< QSPI Standard frame format */
#define QSPI_FRAME_FORMAT_DUAL        QSPI_CTRL1_FRF_0          /*!< QSPI Dual frame format     */
#define QSPI_FRAME_FORMAT_QUAD        QSPI_CTRL1_FRF_1          /*!< QSPI Quad frame format     */
/**
  * @}
  */

/** @defgroup QSPI_DataFrameSize QSPI Data Frame Size
 * @{
 */
#define QSPI_DATA_FRAME_SIZE_4BITS    0x00000003U               /*!< QSPI data frame size 4 bits  */
#define QSPI_DATA_FRAME_SIZE_5BITS    0x00000004U               /*!< QSPI data frame size 5 bits  */
#define QSPI_DATA_FRAME_SIZE_6BITS    0x00000005U               /*!< QSPI data frame size 6 bits  */
#define QSPI_DATA_FRAME_SIZE_7BITS    0x00000006U               /*!< QSPI data frame size 7 bits  */
#define QSPI_DATA_FRAME_SIZE_8BITS    0x00000007U               /*!< QSPI data frame size 8 bits  */
#define QSPI_DATA_FRAME_SIZE_9BITS    0x00000008U               /*!< QSPI data frame size 9 bits  */
#define QSPI_DATA_FRAME_SIZE_10BITS   0x00000009U               /*!< QSPI data frame size 10 bits */
#define QSPI_DATA_FRAME_SIZE_11BITS   0x0000000AU               /*!< QSPI data frame size 11 bits */
#define QSPI_DATA_FRAME_SIZE_12BITS   0x0000000BU               /*!< QSPI data frame size 12 bits */
#define QSPI_DATA_FRAME_SIZE_13BITS   0x0000000CU               /*!< QSPI data frame size 13 bits */
#define QSPI_DATA_FRAME_SIZE_14BITS   0x0000000DU               /*!< QSPI data frame size 14 bits */
#define QSPI_DATA_FRAME_SIZE_15BITS   0x0000000EU               /*!< QSPI data frame size 15 bits */
#define QSPI_DATA_FRAME_SIZE_16BITS   0x0000000FU               /*!< QSPI data frame size 16 bits */
#define QSPI_DATA_FRAME_SIZE_17BITS   0x00000010U               /*!< QSPI data frame size 17 bits */
#define QSPI_DATA_FRAME_SIZE_18BITS   0x00000011U               /*!< QSPI data frame size 18 bits */
#define QSPI_DATA_FRAME_SIZE_19BITS   0x00000012U               /*!< QSPI data frame size 19 bits */
#define QSPI_DATA_FRAME_SIZE_20BITS   0x00000013U               /*!< QSPI data frame size 20 bits */
#define QSPI_DATA_FRAME_SIZE_21BITS   0x00000014U               /*!< QSPI data frame size 21 bits */
#define QSPI_DATA_FRAME_SIZE_22BITS   0x00000015U               /*!< QSPI data frame size 22 bits */
#define QSPI_DATA_FRAME_SIZE_23BITS   0x00000016U               /*!< QSPI data frame size 23 bits */
#define QSPI_DATA_FRAME_SIZE_24BITS   0x00000017U               /*!< QSPI data frame size 24 bits */
#define QSPI_DATA_FRAME_SIZE_25BITS   0x00000018U               /*!< QSPI data frame size 25 bits */
#define QSPI_DATA_FRAME_SIZE_26BITS   0x00000019U               /*!< QSPI data frame size 26 bits */
#define QSPI_DATA_FRAME_SIZE_27BITS   0x0000001AU               /*!< QSPI data frame size 27 bits */
#define QSPI_DATA_FRAME_SIZE_28BITS   0x0000001BU               /*!< QSPI data frame size 28 bits */
#define QSPI_DATA_FRAME_SIZE_29BITS   0x0000001CU               /*!< QSPI data frame size 29 bits */
#define QSPI_DATA_FRAME_SIZE_30BITS   0x0000001DU               /*!< QSPI data frame size 30 bits */
#define QSPI_DATA_FRAME_SIZE_31BITS   0x0000001EU               /*!< QSPI data frame size 31 bits */
#define QSPI_DATA_FRAME_SIZE_32BITS   0x0000001FU               /*!< QSPI data frame size 32 bits */

/**
  * @}
  */

/** @defgroup QSPI_ClockPhase QSPI Clock Phase
 * @{
 */
#define QSPI_CLOCK_PHASE_1ST_EDGE     0x00000000U               /*!< QSPI clock active high on the first edge and inactive low on the second edge */
#define QSPI_CLOCK_PHASE_2ND_EDGE     QSPI_CTRL1_CPHA           /*!< QSPI clock active low on the first edge and inactive high on the second edge */
/**
  * @}
  */

/** @defgroup QSPI_ClockPolarity QSPI Clock Polarity
 * @{
 */
#define QSPI_CLOCK_POLARITY_LOW       0x00000000U               /*!< QSPI clock inactive state is low  */
#define QSPI_CLOCK_POLARITY_HIGH      QSPI_CTRL1_CPOL           /*!< QSPI clock inactive state is high */
/**
  * @}
  */

/** @defgroup QSPI_ChipSelectToggle QSPI Chip Select ChipSelectToggle
 * @{
 */
#define QSPI_CS_TOGGLE_DISABLE        0x00000000U               /*!< QSPI chip select signal doesn't toggle between frames */
#define QSPI_CS_TOGGLE_ENABLE         QSPI_CTRL1_SSTEN          /*!< QSPI chip select signal toggles between frames        */
/**
  * @}
  */

/** @defgroup QSPI_TransferMode QSPI Transfer Mode
 * @{
 */
#define QSPI_TRANSFER_MODE_TX_RX        0x00000000U                                 /*!< QSPI in transmit/receive mode */
#define QSPI_TRANSFER_MODE_TX           QSPI_CTRL1_TXMODE_0                         /*!< QSPI in transmit only mode    */
#define QSPI_TRANSFER_MODE_RX           QSPI_CTRL1_TXMODE_1                         /*!< QSPI in receive only mode     */
#define QSPI_TRANSFER_MODE_EEPROM_READ  (QSPI_CTRL1_TXMODE_1 | QSPI_CTRL1_TXMODE_0) /*!< QSPI in EEPROM read mode */
/**
  * @}
  */

/** @defgroup QSPI_RXDSampleEdge QSPI RXD Sample Edge
 * @{
 */
#define QSPI_RXD_SAMPLE_EDGE_RISING     0x00000000U         /*!< QSPI RXD sample on rising edge  */
#define QSPI_RXD_SAMPLE_EDGE_FALLING    QSPI_RSD_RSE        /*!< QSPI RXD sample on falling edge */
/**
  * @}
  */

/** @defgroup QSPI_InstructionSize QSPI Instruction Size
 * @{
 */
#define QSPI_INSTRUCTION_SIZE_NONE      0x00000000U                                     /*!< No instruction         */
#define QSPI_INSTRUCTION_SIZE_4_BITS    QSPI_CTRL3_INSLEN_0                             /*!< Instruction on 4 bits  */
#define QSPI_INSTRUCTION_SIZE_8_BITS    QSPI_CTRL3_INSLEN_1                             /*!< Instruction on 8 bits  */
#define QSPI_INSTRUCTION_SIZE_16_BITS   (QSPI_CTRL3_INSLEN_1 | QSPI_CTRL3_INSLEN_0)     /*!< Instruction on 16 bits */
/**
  * @}
  */

/** @defgroup QSPI_InstructionMode QSPI Instruction Mode
 * @{
 */
#define QSPI_INSTRUCTION_STANDARD_INS_ADDR      0x00000000U                                 /*!< Send instruction and address in standard SPI mode */
#define QSPI_INSTRUCTION_STANDARD_INS           QSPI_CTRL3_IAT_0                            /*!< Send instruction only in standard SPI mode        */
#define QSPI_INSTRUCTION_FRF_INS_ADDR           QSPI_CTRL3_IAT_1                            /*!< Send instruction and address in FRF mode          */
/**
  * @}
  */

/** @defgroup QSPI_AddressSize QSPI Address Size
 * @{
 */
#define QSPI_ADDRESS_SIZE_NONE           0x00000000U                                         /*!< No address         */
#define QSPI_ADDRESS_SIZE_4_BITS         (QSPI_CTRL3_ADDRLEN_0)                              /*!< Address on 4 bits  */
#define QSPI_ADDRESS_SIZE_8_BITS         (QSPI_CTRL3_ADDRLEN_1)                              /*!< Address on 8 bits  */
#define QSPI_ADDRESS_SIZE_12_BITS        (QSPI_CTRL3_ADDRLEN_1 | QSPI_CTRL3_ADDRLEN_0)       /*!< Address on 12 bits */
#define QSPI_ADDRESS_SIZE_16_BITS        (QSPI_CTRL3_ADDRLEN_2)                              /*!< Address on 16 bits */
#define QSPI_ADDRESS_SIZE_20_BITS        (QSPI_CTRL3_ADDRLEN_2 | QSPI_CTRL3_ADDRLEN_0)       /*!< Address on 20 bits */
#define QSPI_ADDRESS_SIZE_24_BITS        (QSPI_CTRL3_ADDRLEN_2 | QSPI_CTRL3_ADDRLEN_1)       /*!< Address on 24 bits */
#define QSPI_ADDRESS_SIZE_28_BITS        (QSPI_CTRL3_ADDRLEN_2 | QSPI_CTRL3_ADDRLEN_1 |      \
                                         QSPI_CTRL3_ADDRLEN_0)                               /*!< Address on 28 bits */
#define QSPI_ADDRESS_SIZE_32_BITS        (QSPI_CTRL3_ADDRLEN_3)                              /*!< Address on 32 bits */
#define QSPI_ADDRESS_SIZE_36_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_0)       /*!< Address on 36 bits */
#define QSPI_ADDRESS_SIZE_40_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_1)       /*!< Address on 40 bits */
#define QSPI_ADDRESS_SIZE_44_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_1 |      \
                                         QSPI_CTRL3_ADDRLEN_0)                               /*!< Address on 44 bits */
#define QSPI_ADDRESS_SIZE_48_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_2)       /*!< Address on 48 bits */
#define QSPI_ADDRESS_SIZE_52_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_2 |      \
                                         QSPI_CTRL3_ADDRLEN_0)                               /*!< Address on 52 bits */
#define QSPI_ADDRESS_SIZE_56_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_2 |      \
                                         QSPI_CTRL3_ADDRLEN_1)                               /*!< Address on 56 bits */
#define QSPI_ADDRESS_SIZE_60_BITS        (QSPI_CTRL3_ADDRLEN_3 | QSPI_CTRL3_ADDRLEN_2 |      \
                                         QSPI_CTRL3_ADDRLEN_1 | QSPI_CTRL3_ADDRLEN_0)        /*!< Address on 60 bits */

/**
  * @}
  */

/** @defgroup QSPI_Flags QSPI Flags
 * @{
 */
#define QSPI_FLAG_BUSY                  QSPI_STS_BUSYF      /*!< QSPI busy flag                     */
#define QSPI_FLAG_TFN                   QSPI_STS_TFNF       /*!< QSPI transmit FIFO not full flag   */
#define QSPI_FLAG_TFE                   QSPI_STS_TFEF       /*!< QSPI transmit FIFO empty flag      */
#define QSPI_FLAG_RFNE                  QSPI_STS_RFNEF      /*!< QSPI receive FIFO not empty flag   */
#define QSPI_FLAG_RFF                   QSPI_STS_RFFF       /*!< QSPI receive FIFO full flag        */
#define QSPI_FLAG_DCE                   QSPI_STS_DCEF       /*!< QSPI data collision error flag     */
/**
  * @}
  */

/** @defgroup QSPI_Interrupts QSPI Interrupts
 * @{
 */
#define QSPI_IT_TFE                     QSPI_INTEN_TFEIE    /*!< QSPI transmit FIFO interrupt                       */
#define QSPI_IT_TFO                     QSPI_INTEN_TFOIE    /*!< QSPI transmit FIFO overflow interrupt              */
#define QSPI_IT_RFU                     QSPI_INTEN_RFUIE    /*!< QSPI receive FIFO underflow interrupt              */
#define QSPI_IT_RFO                     QSPI_INTEN_RFOIE    /*!< QSPI receive FIFO overflow interrupt               */
#define QSPI_IT_RFF                     QSPI_INTEN_RFFIE    /*!< QSPI receive FIFO full interrupt                   */
#define QSPI_IT_MST                     QSPI_INTEN_MSTIE    /*!< QSPI master operation complete interrupt           */
#define QSPI_IT_ICF                     0xFFFFFFFFU         /*!< QSPI FIFO status and master operation interrupt    */
/**
  * @}
  */

/** @defgroup QSPI_Timeout_definition QSPI Timeout definition
 * @{
 */
#define DAL_QSPI_TIMEOUT_DEFAULT_VALUE      5000U   /*!< 5 s */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup QSPI_Exported_Macros QSPI Exported Macros
  * @{
  */

/** @brief Reset QSPI handle state
 * @param  __HANDLE__ specifies the QSPI handle.
 * @retval None
 */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
#define __DAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)   do{                                                   \
                                                        (__HANDLE__)->State             = DAL_QSPI_STATE_RESET; \
                                                        (__HANDLE__)->MspInitCallback   = NULL;                \
                                                        (__HANDLE__)->MspDeInitCallback = NULL;                \
                                                      } while(0)
#else
#define __DAL_QSPI_RESET_HANDLE_STATE(__HANDLE__)   ((__HANDLE__)->State = DAL_QSPI_STATE_RESET)
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

/**
 * @brief  Enable the specified QSPI peripheral.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_ENABLE(__HANDLE__)               ((__HANDLE__)->Instance->SSIEN |= QSPI_SSIEN_EN)

/**
 * @brief  Disable the specified QSPI peripheral.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_DISABLE(__HANDLE__)              ((__HANDLE__)->Instance->SSIEN &= ~QSPI_SSIEN_EN)

/**
 * @brief  Enable the slave select signal.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_ENABLE_SS(__HANDLE__)            ((__HANDLE__)->Instance->SLAEN |= QSPI_SLAEN_SLAEN)

/**
 * @brief  Disable the slave select signal.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_DISABLE_SS(__HANDLE__)           ((__HANDLE__)->Instance->SLAEN &= ~QSPI_SLAEN_SLAEN)

/**
 * @brief  Disable the specified QSPI clock output.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_DISABLE_CLK(__HANDLE__)          ((__HANDLE__)->Instance->BR &= ~QSPI_BR_CLKDIV)

/**
 * @brief  Enable the specified QSPI interrupts.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to enable.
 *        This parameter can be one of the following values:
 *          @arg QSPI_IT_TFE: QSPI transmit FIFO interrupt
 *          @arg QSPI_IT_TFO: QSPI transmit FIFO overflow interrupt
 *          @arg QSPI_IT_RFU: QSPI receive FIFO underflow interrupt
 *          @arg QSPI_IT_RFO: QSPI receive FIFO overflow interrupt
 *          @arg QSPI_IT_RFF: QSPI receive FIFO full interrupt
 *          @arg QSPI_IT_MST: QSPI master operation complete interrupt
 * @retval None
 */
#define __DAL_QSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->INTEN |= (__INTERRUPT__))

/**
 * @brief  Disable the specified QSPI interrupts.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to disable.
 *       This parameter can be one of the following values:
 *         @arg QSPI_IT_TFE: QSPI transmit FIFO interrupt
 *         @arg QSPI_IT_TFO: QSPI transmit FIFO overflow interrupt
 *         @arg QSPI_IT_RFU: QSPI receive FIFO underflow interrupt
 *         @arg QSPI_IT_RFO: QSPI receive FIFO overflow interrupt
 *         @arg QSPI_IT_RFF: QSPI receive FIFO full interrupt
 *         @arg QSPI_IT_MST: QSPI master operation complete interrupt
 * @retval None
 */
#define __DAL_QSPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->INTEN &= ~(__INTERRUPT__))

/**
 * @brief  Check whether the specified QSPI interrupt source is enabled or not.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @param  __INTERRUPT__ specifies the QSPI interrupt source to check.
 *        This parameter can be one of the following values:
 *         @arg QSPI_IT_TFE: QSPI transmit FIFO interrupt
 *         @arg QSPI_IT_TFO: QSPI transmit FIFO overflow interrupt
 *         @arg QSPI_IT_RFU: QSPI receive FIFO underflow interrupt
 *         @arg QSPI_IT_RFO: QSPI receive FIFO overflow interrupt
 *         @arg QSPI_IT_RFF: QSPI receive FIFO full interrupt
 *         @arg QSPI_IT_MST: QSPI master operation complete interrupt
 * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
 */
#define __DAL_QSPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  ((((__HANDLE__)->Instance->ISTS & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/**
 * @brief  Check whether the specified QSPI flag is set or not.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @param  __FLAG__ specifies the QSPI flag to check.
 *       This parameter can be one of the following values:
 *        @arg QSPI_FLAG_BUSY: QSPI busy flag
 *        @arg QSPI_FLAG_TFN: QSPI transmit FIFO not full flag
 *        @arg QSPI_FLAG_TFE: QSPI transmit FIFO empty flag
 *        @arg QSPI_FLAG_RFNE: QSPI receive FIFO not empty flag
 *        @arg QSPI_FLAG_RFF: QSPI receive FIFO full flag
 *        @arg QSPI_FLAG_DCE: QSPI data collision error flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __DAL_QSPI_GET_FLAG(__HANDLE__, __FLAG__)  ((((__HANDLE__)->Instance->STS & (__FLAG__)) != 0U) ? SET : RESET)

/**
 * @brief  Clear the specified QSPI flag.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @param  __FLAG__ specifies the QSPI flag to clear.
 *       This parameter can be one of the following values:
 *        @arg QSPI_IT_TFO: QSPI transmit FIFO overflow interrupt
 *        @arg QSPI_IT_RFU: QSPI receive FIFO underflow interrupt
 *        @arg QSPI_IT_RFO: QSPI receive FIFO overflow interrupt
 *        @arg QSPI_IT_MST: QSPI master operation complete interrupt
 *        @arg QSPI_IT_ICF: QSPI FIFO status and master operation interrupt
 * @retval None
 */
#define __DAL_QSPI_CLEAR_FLAG(__HANDLE__, __FLAG__)   do {                                                          \
                                                            __IO uint32_t tmpreg = 0x00U;                           \
                                                            if ((__FLAG__) == QSPI_IT_TFO)                          \
                                                            {                                                       \
                                                                tmpreg = (__HANDLE__)->Instance->TFOIC;             \
                                                            }                                                       \
                                                            else if ((__FLAG__) == QSPI_IT_RFU)                     \
                                                            {                                                       \
                                                                tmpreg = (__HANDLE__)->Instance->RFUIC;             \
                                                            }                                                       \
                                                            else if ((__FLAG__) == QSPI_IT_RFO)                     \
                                                            {                                                       \
                                                                tmpreg = (__HANDLE__)->Instance->RFOIC;             \
                                                            }                                                       \
                                                            else if ((__FLAG__) == QSPI_IT_MST)                     \
                                                            {                                                       \
                                                                tmpreg = (__HANDLE__)->Instance->MIC;               \
                                                            }                                                       \
                                                            else if ((__FLAG__) == QSPI_IT_ICF)                     \
                                                            {                                                       \
                                                                tmpreg = (__HANDLE__)->Instance->ICF;               \
                                                            }                                                       \
                                                            else                                                    \
                                                            {                                                       \
                                                                /* Do nothing */                                    \
                                                            }                                                       \
                                                            UNUSED(tmpreg);                                         \
                                                        } while(0)

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
DAL_StatusTypeDef   DAL_QSPI_Init(QSPI_HandleTypeDef *hqspi);
DAL_StatusTypeDef   DAL_QSPI_DeInit(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi);
/**
  * @}
  */

/** @addtogroup QSPI_Exported_Functions_Group2
 * @{
 */

/* I/O operation functions  ***************************************************/
/* QSPI IRQ handler method */
void                DAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi);

/* Transmit/Receive functions  ***********************************************/
DAL_StatusTypeDef   DAL_QSPI_Command(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout);
DAL_StatusTypeDef   DAL_QSPI_TransmitReceive(QSPI_HandleTypeDef *hqspi, uint8_t *data_out, uint8_t *data_in, uint32_t Size, uint32_t Timeout);
DAL_StatusTypeDef   DAL_QSPI_Transmit(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
DAL_StatusTypeDef   DAL_QSPI_Receive(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);

DAL_StatusTypeDef   DAL_QSPI_Command_IT(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd);
DAL_StatusTypeDef   DAL_QSPI_TransmitReceive_IT(QSPI_HandleTypeDef *hqspi, uint8_t *data_out, uint8_t *data_in, uint32_t Size);
DAL_StatusTypeDef   DAL_QSPI_Transmit_IT(QSPI_HandleTypeDef *hqspi, uint8_t *pData);
DAL_StatusTypeDef   DAL_QSPI_Receive_IT(QSPI_HandleTypeDef *hqspi, uint8_t *pData);

DAL_StatusTypeDef   DAL_QSPI_Transmit_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData);
DAL_StatusTypeDef   DAL_QSPI_Receive_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData);

/* Callbacks functions  *******************************************************/
void                DAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi);

void                DAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi);
void                DAL_QSPI_TxRxCpltCallback(QSPI_HandleTypeDef *hqspi);

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
DAL_StatusTypeDef   DAL_QSPI_RegisterCallback(QSPI_HandleTypeDef *hqspi, DAL_QSPI_CallbackIDTypeDef CallbackID, pQSPI_CallbackTypeDef pCallback);
DAL_StatusTypeDef   DAL_QSPI_UnRegisterCallback(QSPI_HandleTypeDef *hqspi, DAL_QSPI_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup QSPI_Exported_Functions_Group3
 * @{
 */

/* Peripheral Control and State functions  ************************************/
DAL_QSPI_StateTypeDef   DAL_QSPI_GetState(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetError(QSPI_HandleTypeDef *hqspi);
DAL_StatusTypeDef       DAL_QSPI_Abort(QSPI_HandleTypeDef *hqspi);
DAL_StatusTypeDef       DAL_QSPI_Abort_IT(QSPI_HandleTypeDef *hqspi);
void                    DAL_QSPI_SetTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
DAL_StatusTypeDef       DAL_QSPI_SetTxFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold);
DAL_StatusTypeDef       DAL_QSPI_SetTxFifoLevel(QSPI_HandleTypeDef *hqspi, uint32_t Level);
DAL_StatusTypeDef       DAL_QSPI_SetRxFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold);
uint32_t                DAL_QSPI_GetTxFifoThreshold(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetTxFifoLevel(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetRxFifoThreshold(QSPI_HandleTypeDef *hqspi);

DAL_StatusTypeDef       DAL_QSPI_SetFrameFormat(QSPI_HandleTypeDef *hqspi, uint32_t FrameFormat);
DAL_StatusTypeDef       DAL_QSPI_SetDataFrameSize(QSPI_HandleTypeDef *hqspi, uint32_t DataFrameSize);
DAL_StatusTypeDef       DAL_QSPI_SetTransferMode(QSPI_HandleTypeDef *hqspi, uint32_t TransferMode);
DAL_StatusTypeDef       DAL_QSPI_SetFrameNbData(QSPI_HandleTypeDef *hqspi, uint32_t NbData);
uint32_t                DAL_QSPI_GetFrameFormat(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetDataFrameSize(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetTransferMode(QSPI_HandleTypeDef *hqspi);
uint32_t                DAL_QSPI_GetFrameNbData(QSPI_HandleTypeDef *hqspi);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup QSPI_Private_Macros QSPI Private Macros
  * @{
  */
#define IS_QSPI_CLOCK_PRESCALER(__PRESCALER__)  (((__PRESCALER__) >= 2U) && ((__PRESCALER__) <= 65534U) && (((__PRESCALER__) & 0x1U) == 0U))

#define IS_QSPI_FRAME_FORMAT(__FORMAT__)        (((__FORMAT__) == QSPI_FRAME_FORMAT_STANDARD) || \
                                                 ((__FORMAT__) == QSPI_FRAME_FORMAT_DUAL)     || \
                                                 ((__FORMAT__) == QSPI_FRAME_FORMAT_QUAD))

#define IS_QSPI_DATA_FRAME_SIZE(__SIZE__)       (((__SIZE__) >= 3U) && ((__SIZE__) <= 31U))

#define IS_QSPI_CLOCK_PHASE(__PHASE__)          (((__PHASE__) == QSPI_CLOCK_PHASE_1ST_EDGE) || \
                                                 ((__PHASE__) == QSPI_CLOCK_PHASE_2ND_EDGE))

#define IS_QSPI_CLOCK_POLARITY(__POLARITY__)    (((__POLARITY__) == QSPI_CLOCK_POLARITY_LOW) || \
                                                 ((__POLARITY__) == QSPI_CLOCK_POLARITY_HIGH))

#define IS_QSPI_CLOCK_STRETCH(__STRETCH__)      (((__STRETCH__) == ENABLE) || \
                                                 ((__STRETCH__) == DISABLE))

#define IS_QSPI_TX_FIFO_THRESHOLD(__THRESHOLD__)  (((__THRESHOLD__) >= 0U) && ((__THRESHOLD__) <= 7U))

#define IS_QSPI_TX_FIFO_LEVEL(__LEVEL__)        (((__LEVEL__) >= 0U) && ((__LEVEL__) <= 7U))

#define IS_QSPI_RX_FIFO_THRESHOLD(__THRESHOLD__)  (((__THRESHOLD__) >= 0U) && ((__THRESHOLD__) <= 7U))

#define IS_QSPI_CHIP_SELECT_TOGGLE(__TOGGLE__)  (((__TOGGLE__) == QSPI_CS_TOGGLE_DISABLE) || \
                                                 ((__TOGGLE__) == QSPI_CS_TOGGLE_ENABLE))

#define IS_QSPI_TRANSFER_MODE(__MODE__)         (((__MODE__) == QSPI_TRANSFER_MODE_TX_RX)       || \
                                                 ((__MODE__) == QSPI_TRANSFER_MODE_TX)          || \
                                                 ((__MODE__) == QSPI_TRANSFER_MODE_RX)          || \
                                                 ((__MODE__) == QSPI_TRANSFER_MODE_EEPROM_READ))

#define IS_QSPI_RXD_SAMPLE_EDGE(__EDGE__)       (((__EDGE__) == QSPI_RXD_SAMPLE_EDGE_RISING) || \
                                                 ((__EDGE__) == QSPI_RXD_SAMPLE_EDGE_FALLING))

#define IS_QSPI_INSTRUCTION_SIZE(__SIZE__)      (((__SIZE__) == QSPI_INSTRUCTION_SIZE_NONE)  || \
                                                 ((__SIZE__) == QSPI_INSTRUCTION_SIZE_4_BITS) || \
                                                 ((__SIZE__) == QSPI_INSTRUCTION_SIZE_8_BITS) || \
                                                 ((__SIZE__) == QSPI_INSTRUCTION_SIZE_16_BITS))

#define IS_QSPI_INSTRUCTION_MODE(__MODE__)      (((__MODE__) == QSPI_INSTRUCTION_STANDARD_INS_ADDR) || \
                                                 ((__MODE__) == QSPI_INSTRUCTION_STANDARD_INS)      || \
                                                 ((__MODE__) == QSPI_INSTRUCTION_FRF_INS_ADDR))

#define IS_QSPI_INSTRUCTION(__INSTRUCTION__)    ((__INSTRUCTION__) <= 0xFFU)

#define IS_QSPI_ADDRESS_SIZE(__SIZE__)     ((__SIZE__) >= QSPI_ADDRESS_SIZE_NONE && (__SIZE__) <= QSPI_ADDRESS_SIZE_60_BITS)

#define IS_QSPI_ADDRESS(__ADDRESS__)            ((__ADDRESS__) <= 0xFFFFFFFFU)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* QSPI */

#ifdef __cplusplus
}
#endif

#endif /* __APM32F4XX_DAL_QSPI_H */

