/**
  *
  * @file    apm32f4xx_dal_spi.h
  * @brief   Header file of SPI DAL module.
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
#ifndef APM32F4xx_DAL_SPI_H
#define APM32F4xx_DAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup SPI
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
  uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */

  uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */

  uint32_t DataSize;            /*!< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */

  uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */

  uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */

  uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;              /*!< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode */

  uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */

  uint32_t CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535 */
} SPI_InitTypeDef;

/**
  * @brief  DAL SPI State structure definition
  */
typedef enum
{
  DAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
  DAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
  DAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
  DAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
  DAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
  DAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
  DAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
  DAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} DAL_SPI_StateTypeDef;

/**
  * @brief  SPI handle Structure definition
  */
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

  SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

  uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

  uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

  __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

  uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

  uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

  __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */

  DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

  DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

  DAL_LockTypeDef            Lock;           /*!< Locking object                           */

  __IO DAL_SPI_StateTypeDef  State;          /*!< SPI communication state                  */

  __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  void (* TxCpltCallback)(struct __SPI_HandleTypeDef *hspi);             /*!< SPI Tx Completed callback          */
  void (* RxCpltCallback)(struct __SPI_HandleTypeDef *hspi);             /*!< SPI Rx Completed callback          */
  void (* TxRxCpltCallback)(struct __SPI_HandleTypeDef *hspi);           /*!< SPI TxRx Completed callback        */
  void (* TxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);         /*!< SPI Tx Half Completed callback     */
  void (* RxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);         /*!< SPI Rx Half Completed callback     */
  void (* TxRxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);       /*!< SPI TxRx Half Completed callback   */
  void (* ErrorCallback)(struct __SPI_HandleTypeDef *hspi);              /*!< SPI Error callback                 */
  void (* AbortCpltCallback)(struct __SPI_HandleTypeDef *hspi);          /*!< SPI Abort callback                 */
  void (* MspInitCallback)(struct __SPI_HandleTypeDef *hspi);            /*!< SPI Msp Init callback              */
  void (* MspDeInitCallback)(struct __SPI_HandleTypeDef *hspi);          /*!< SPI Msp DeInit callback            */

#endif  /* USE_DAL_SPI_REGISTER_CALLBACKS */
} SPI_HandleTypeDef;

#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
/**
  * @brief  DAL SPI Callback ID enumeration definition
  */
typedef enum
{
  DAL_SPI_TX_COMPLETE_CB_ID             = 0x00U,    /*!< SPI Tx Completed callback ID         */
  DAL_SPI_RX_COMPLETE_CB_ID             = 0x01U,    /*!< SPI Rx Completed callback ID         */
  DAL_SPI_TX_RX_COMPLETE_CB_ID          = 0x02U,    /*!< SPI TxRx Completed callback ID       */
  DAL_SPI_TX_HALF_COMPLETE_CB_ID        = 0x03U,    /*!< SPI Tx Half Completed callback ID    */
  DAL_SPI_RX_HALF_COMPLETE_CB_ID        = 0x04U,    /*!< SPI Rx Half Completed callback ID    */
  DAL_SPI_TX_RX_HALF_COMPLETE_CB_ID     = 0x05U,    /*!< SPI TxRx Half Completed callback ID  */
  DAL_SPI_ERROR_CB_ID                   = 0x06U,    /*!< SPI Error callback ID                */
  DAL_SPI_ABORT_CB_ID                   = 0x07U,    /*!< SPI Abort callback ID                */
  DAL_SPI_MSPINIT_CB_ID                 = 0x08U,    /*!< SPI Msp Init callback ID             */
  DAL_SPI_MSPDEINIT_CB_ID               = 0x09U     /*!< SPI Msp DeInit callback ID           */

} DAL_SPI_CallbackIDTypeDef;

/**
  * @brief  DAL SPI Callback pointer definition
  */
typedef  void (*pSPI_CallbackTypeDef)(SPI_HandleTypeDef *hspi); /*!< pointer to an SPI callback function */

#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define DAL_SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define DAL_SPI_ERROR_MODF              (0x00000001U)   /*!< MODF error                             */
#define DAL_SPI_ERROR_CRC               (0x00000002U)   /*!< CRC error                              */
#define DAL_SPI_ERROR_OVR               (0x00000004U)   /*!< OVR error                              */
#define DAL_SPI_ERROR_FRE               (0x00000008U)   /*!< FRE error                              */
#define DAL_SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define DAL_SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXNE/TXE/BSY Flag             */
#define DAL_SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
#define DAL_SPI_ERROR_INVALID_CALLBACK  (0x00000080U)   /*!< Invalid Callback error                 */
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_MODE_SLAVE                  (0x00000000U)
#define SPI_MODE_MASTER                 (SPI_CTRL1_MSMCFG | SPI_CTRL1_ISSEL)
/**
  * @}
  */

/** @defgroup SPI_Direction SPI Direction Mode
  * @{
  */
#define SPI_DIRECTION_2LINES            (0x00000000U)
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CTRL1_RXOMEN
#define SPI_DIRECTION_1LINE             SPI_CTRL1_BMEN
/**
  * @}
  */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_DATASIZE_8BIT               (0x00000000U)
#define SPI_DATASIZE_16BIT              SPI_CTRL1_DFLSEL
/**
  * @}
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                (0x00000000U)
#define SPI_POLARITY_HIGH               SPI_CTRL1_CPOL
/**
  * @}
  */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 (0x00000000U)
#define SPI_PHASE_2EDGE                 SPI_CTRL1_CPHA
/**
  * @}
  */

/** @defgroup SPI_Slave_Select_management SPI Slave Select Management
  * @{
  */
#define SPI_NSS_SOFT                    SPI_CTRL1_SSEN
#define SPI_NSS_HARD_INPUT              (0x00000000U)
#define SPI_NSS_HARD_OUTPUT             (SPI_CTRL2_SSOEN << 16U)
/**
  * @}
  */

/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
#define SPI_BAUDRATEPRESCALER_2         (0x00000000U)
#define SPI_BAUDRATEPRESCALER_4         (SPI_CTRL1_BRSEL_0)
#define SPI_BAUDRATEPRESCALER_8         (SPI_CTRL1_BRSEL_1)
#define SPI_BAUDRATEPRESCALER_16        (SPI_CTRL1_BRSEL_1 | SPI_CTRL1_BRSEL_0)
#define SPI_BAUDRATEPRESCALER_32        (SPI_CTRL1_BRSEL_2)
#define SPI_BAUDRATEPRESCALER_64        (SPI_CTRL1_BRSEL_2 | SPI_CTRL1_BRSEL_0)
#define SPI_BAUDRATEPRESCALER_128       (SPI_CTRL1_BRSEL_2 | SPI_CTRL1_BRSEL_1)
#define SPI_BAUDRATEPRESCALER_256       (SPI_CTRL1_BRSEL_2 | SPI_CTRL1_BRSEL_1 | SPI_CTRL1_BRSEL_0)
/**
  * @}
  */

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
  * @{
  */
#define SPI_FIRSTBIT_MSB                (0x00000000U)
#define SPI_FIRSTBIT_LSB                SPI_CTRL1_LSBSEL
/**
  * @}
  */

/** @defgroup SPI_TI_mode SPI TI Mode
  * @{
  */
#define SPI_TIMODE_DISABLE              (0x00000000U)
#define SPI_TIMODE_ENABLE               SPI_CTRL2_FRFCFG
/**
  * @}
  */

/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
#define SPI_CRCCALCULATION_DISABLE      (0x00000000U)
#define SPI_CRCCALCULATION_ENABLE       SPI_CTRL1_CRCEN
/**
  * @}
  */

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
#define SPI_IT_TXE                      SPI_CTRL2_TXBEIEN
#define SPI_IT_RXNE                     SPI_CTRL2_RXBNEIEN
#define SPI_IT_ERR                      SPI_CTRL2_ERRIEN
/**
  * @}
  */

/** @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */
#define SPI_FLAG_RXNE                   SPI_STS_RXBNEFLG   /* SPI status flag: Rx buffer not empty flag       */
#define SPI_FLAG_TXE                    SPI_STS_TXBEFLG    /* SPI status flag: Tx buffer empty flag           */
#define SPI_FLAG_BSY                    SPI_STS_BSYFLG    /* SPI status flag: Busy flag                      */
#define SPI_FLAG_CRCERR                 SPI_STS_CRCEFLG /* SPI Error flag: CRC error flag                  */
#define SPI_FLAG_MODF                   SPI_STS_MEFLG   /* SPI Error flag: Mode fault flag                 */
#define SPI_FLAG_OVR                    SPI_STS_OVRFLG    /* SPI Error flag: Overrun flag                    */
#define SPI_FLAG_FRE                    SPI_STS_FFERR    /* SPI Error flag: TI mode frame format error flag */
#define SPI_FLAG_MASK                   (SPI_STS_RXBNEFLG | SPI_STS_TXBEFLG | SPI_STS_BSYFLG | SPI_STS_CRCEFLG\
                                         | SPI_STS_MEFLG | SPI_STS_OVRFLG | SPI_STS_FFERR)
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

/** @brief  Reset SPI handle state.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
#define __DAL_SPI_RESET_HANDLE_STATE(__HANDLE__)                do{                                                  \
                                                                    (__HANDLE__)->State = DAL_SPI_STATE_RESET;       \
                                                                    (__HANDLE__)->MspInitCallback = NULL;            \
                                                                    (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                                  } while(0)
#else
#define __DAL_SPI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_SPI_STATE_RESET)
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */

/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __DAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->CTRL2, (__INTERRUPT__))

/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI handle.
  *         This parameter can be SPIx where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __DAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->CTRL2, (__INTERRUPT__))

/** @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CTRL2\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  *            @arg SPI_FLAG_FRE: Frame format error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->STS) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI CRCERR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_CLEAR_CRCERRFLAG(__HANDLE__) ((__HANDLE__)->Instance->STS = (uint16_t)(~SPI_FLAG_CRCERR))

/** @brief  Clear the SPI MODF pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_CLEAR_MODFFLAG(__HANDLE__)             \
  do{                                                    \
    __IO uint32_t tmpreg_modf = 0x00U;                   \
    tmpreg_modf = (__HANDLE__)->Instance->STS;            \
    CLEAR_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_SPIEN); \
    UNUSED(tmpreg_modf);                                 \
  } while(0U)

/** @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_CLEAR_OVRFLAG(__HANDLE__)        \
  do{                                              \
    __IO uint32_t tmpreg_ovr = 0x00U;              \
    tmpreg_ovr = (__HANDLE__)->Instance->DATA;       \
    tmpreg_ovr = (__HANDLE__)->Instance->STS;       \
    UNUSED(tmpreg_ovr);                            \
  } while(0U)

/** @brief  Clear the SPI FRE pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_CLEAR_FREFLAG(__HANDLE__)        \
  do{                                              \
    __IO uint32_t tmpreg_fre = 0x00U;              \
    tmpreg_fre = (__HANDLE__)->Instance->STS;       \
    UNUSED(tmpreg_fre);                            \
  }while(0U)

/** @brief  Enable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_SPIEN)

/** @brief  Disable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __DAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_SPIEN)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */

/** @brief  Set the SPI transmit-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_1LINE_TX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_BMOEN)

/** @brief  Set the SPI receive-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_1LINE_RX(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_BMOEN)

/** @brief  Reset the CRC calculation of the SPI.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_RESET_CRC(__HANDLE__) do{CLEAR_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_CRCEN);\
                                       SET_BIT((__HANDLE__)->Instance->CTRL1, SPI_CTRL1_CRCEN);}while(0U)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __STS__  copy of SPI STS register.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  *            @arg SPI_FLAG_FRE: Frame format error flag
  * @retval SET or RESET.
  */
#define SPI_CHECK_FLAG(__STS__, __FLAG__) ((((__STS__) & ((__FLAG__) & SPI_FLAG_MASK)) == \
                                          ((__FLAG__) & SPI_FLAG_MASK)) ? SET : RESET)

/** @brief  Check whether the specified SPI Interrupt is set or not.
  * @param  __CTRL2__  copy of SPI CTRL2 register.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval SET or RESET.
  */
#define SPI_CHECK_IT_SOURCE(__CTRL2__, __INTERRUPT__) ((((__CTRL2__) & (__INTERRUPT__)) == \
                                                     (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks if SPI Mode parameter is in allowed range.
  * @param  __MODE__ specifies the SPI Mode.
  *         This parameter can be a value of @ref SPI_Mode
  * @retval None
  */
#define IS_SPI_MODE(__MODE__)      (((__MODE__) == SPI_MODE_SLAVE)   || \
                                    ((__MODE__) == SPI_MODE_MASTER))

/** @brief  Checks if SPI Direction Mode parameter is in allowed range.
  * @param  __MODE__ specifies the SPI Direction Mode.
  *         This parameter can be a value of @ref SPI_Direction
  * @retval None
  */
#define IS_SPI_DIRECTION(__MODE__) (((__MODE__) == SPI_DIRECTION_2LINES)        || \
                                    ((__MODE__) == SPI_DIRECTION_2LINES_RXONLY) || \
                                    ((__MODE__) == SPI_DIRECTION_1LINE))

/** @brief  Checks if SPI Direction Mode parameter is 2 lines.
  * @param  __MODE__ specifies the SPI Direction Mode.
  * @retval None
  */
#define IS_SPI_DIRECTION_2LINES(__MODE__) ((__MODE__) == SPI_DIRECTION_2LINES)

/** @brief  Checks if SPI Direction Mode parameter is 1 or 2 lines.
  * @param  __MODE__ specifies the SPI Direction Mode.
  * @retval None
  */
#define IS_SPI_DIRECTION_2LINES_OR_1LINE(__MODE__) (((__MODE__) == SPI_DIRECTION_2LINES) || \
                                                    ((__MODE__) == SPI_DIRECTION_1LINE))

/** @brief  Checks if SPI Data Size parameter is in allowed range.
  * @param  __DATASIZE__ specifies the SPI Data Size.
  *         This parameter can be a value of @ref SPI_Data_Size
  * @retval None
  */
#define IS_SPI_DATASIZE(__DATASIZE__) (((__DATASIZE__) == SPI_DATASIZE_16BIT) || \
                                       ((__DATASIZE__) == SPI_DATASIZE_8BIT))

/** @brief  Checks if SPI Serial clock steady state parameter is in allowed range.
  * @param  __CPOL__ specifies the SPI serial clock steady state.
  *         This parameter can be a value of @ref SPI_Clock_Polarity
  * @retval None
  */
#define IS_SPI_CPOL(__CPOL__)      (((__CPOL__) == SPI_POLARITY_LOW) || \
                                    ((__CPOL__) == SPI_POLARITY_HIGH))

/** @brief  Checks if SPI Clock Phase parameter is in allowed range.
  * @param  __CPHA__ specifies the SPI Clock Phase.
  *         This parameter can be a value of @ref SPI_Clock_Phase
  * @retval None
  */
#define IS_SPI_CPHA(__CPHA__)      (((__CPHA__) == SPI_PHASE_1EDGE) || \
                                    ((__CPHA__) == SPI_PHASE_2EDGE))

/** @brief  Checks if SPI Slave Select parameter is in allowed range.
  * @param  __NSS__ specifies the SPI Slave Select management parameter.
  *         This parameter can be a value of @ref SPI_Slave_Select_management
  * @retval None
  */
#define IS_SPI_NSS(__NSS__)        (((__NSS__) == SPI_NSS_SOFT)       || \
                                    ((__NSS__) == SPI_NSS_HARD_INPUT) || \
                                    ((__NSS__) == SPI_NSS_HARD_OUTPUT))

/** @brief  Checks if SPI Baudrate prescaler parameter is in allowed range.
  * @param  __PRESCALER__ specifies the SPI Baudrate prescaler.
  *         This parameter can be a value of @ref SPI_BaudRate_Prescaler
  * @retval None
  */
#define IS_SPI_BAUDRATE_PRESCALER(__PRESCALER__) (((__PRESCALER__) == SPI_BAUDRATEPRESCALER_2)   || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_4)   || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_8)   || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_16)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_32)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_64)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_128) || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_256))

/** @brief  Checks if SPI MSB LSB transmission parameter is in allowed range.
  * @param  __BIT__ specifies the SPI MSB LSB transmission (whether data transfer starts from MSB or LSB bit).
  *         This parameter can be a value of @ref SPI_MSB_LSB_transmission
  * @retval None
  */
#define IS_SPI_FIRST_BIT(__BIT__)  (((__BIT__) == SPI_FIRSTBIT_MSB) || \
                                    ((__BIT__) == SPI_FIRSTBIT_LSB))

/** @brief  Checks if SPI TI mode parameter is in allowed range.
  * @param  __MODE__ specifies the SPI TI mode.
  *         This parameter can be a value of @ref SPI_TI_mode
  * @retval None
  */
#define IS_SPI_TIMODE(__MODE__)    (((__MODE__) == SPI_TIMODE_DISABLE) || \
                                    ((__MODE__) == SPI_TIMODE_ENABLE))

/** @brief  Checks if SPI CRC calculation enabled state is in allowed range.
  * @param  __CALCULATION__ specifies the SPI CRC calculation enable state.
  *         This parameter can be a value of @ref SPI_CRC_Calculation
  * @retval None
  */
#define IS_SPI_CRC_CALCULATION(__CALCULATION__) (((__CALCULATION__) == SPI_CRCCALCULATION_DISABLE) || \
                                                 ((__CALCULATION__) == SPI_CRCCALCULATION_ENABLE))

/** @brief  Checks if SPI polynomial value to be used for the CRC calculation, is in allowed range.
  * @param  __POLYNOMIAL__ specifies the SPI polynomial value to be used for the CRC calculation.
  *         This parameter must be a number between Min_Data = 0 and Max_Data = 65535
  * @retval None
  */
#define IS_SPI_CRC_POLYNOMIAL(__POLYNOMIAL__) (((__POLYNOMIAL__) >= 0x1U)    && \
                                               ((__POLYNOMIAL__) <= 0xFFFFU) && \
                                              (((__POLYNOMIAL__)&0x1U) != 0U))

/** @brief  Checks if DMA handle is valid.
  * @param  __HANDLE__ specifies a DMA Handle.
  * @retval None
  */
#define IS_SPI_DMA_HANDLE(__HANDLE__) ((__HANDLE__) != NULL)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions
  * @{
  */

/** @addtogroup SPI_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
DAL_StatusTypeDef DAL_SPI_Init(SPI_HandleTypeDef *hspi);
DAL_StatusTypeDef DAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void DAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void DAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
DAL_StatusTypeDef DAL_SPI_RegisterCallback(SPI_HandleTypeDef *hspi, DAL_SPI_CallbackIDTypeDef CallbackID,
                                           pSPI_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_SPI_UnRegisterCallback(SPI_HandleTypeDef *hspi, DAL_SPI_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
DAL_StatusTypeDef DAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
DAL_StatusTypeDef DAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
DAL_StatusTypeDef DAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
DAL_StatusTypeDef DAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
DAL_StatusTypeDef DAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
DAL_StatusTypeDef DAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);
/* Transfer Abort functions */
DAL_StatusTypeDef DAL_SPI_Abort(SPI_HandleTypeDef *hspi);
DAL_StatusTypeDef DAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void DAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void DAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void DAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
DAL_SPI_StateTypeDef DAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             DAL_SPI_GetError(SPI_HandleTypeDef *hspi);
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

#endif /* APM32F4xx_DAL_SPI_H */

