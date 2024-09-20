/**
  *
  * @file    apm32f4xx_dal_i2s.h
  * @brief   Header file of I2S DAL module.
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
#ifndef APM32F4xx_DAL_I2S_H
#define APM32F4xx_DAL_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup I2S
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup I2S_Exported_Types I2S Exported Types
  * @{
  */

/**
  * @brief I2S Init structure definition
  */
typedef struct
{
  uint32_t Mode;                /*!< Specifies the I2S operating mode.
                                     This parameter can be a value of @ref I2S_Mode */

  uint32_t Standard;            /*!< Specifies the standard used for the I2S communication.
                                     This parameter can be a value of @ref I2S_Standard */

  uint32_t DataFormat;          /*!< Specifies the data format for the I2S communication.
                                     This parameter can be a value of @ref I2S_Data_Format */

  uint32_t MCLKOutput;          /*!< Specifies whether the I2S MCLK output is enabled or not.
                                     This parameter can be a value of @ref I2S_MCLK_Output */

  uint32_t AudioFreq;           /*!< Specifies the frequency selected for the I2S communication.
                                     This parameter can be a value of @ref I2S_Audio_Frequency */

  uint32_t CPOL;                /*!< Specifies the idle state of the I2S clock.
                                     This parameter can be a value of @ref I2S_Clock_Polarity */

  uint32_t ClockSource;     /*!< Specifies the I2S Clock Source.
                                 This parameter can be a value of @ref I2S_Clock_Source */
  uint32_t FullDuplexMode;  /*!< Specifies the I2S FullDuplex mode.
                                 This parameter can be a value of @ref I2S_FullDuplex_Mode */
} I2S_InitTypeDef;

/**
  * @brief  DAL State structures definition
  */
typedef enum
{
  DAL_I2S_STATE_RESET      = 0x00U,  /*!< I2S not yet initialized or disabled                */
  DAL_I2S_STATE_READY      = 0x01U,  /*!< I2S initialized and ready for use                  */
  DAL_I2S_STATE_BUSY       = 0x02U,  /*!< I2S internal process is ongoing                    */
  DAL_I2S_STATE_BUSY_TX    = 0x03U,  /*!< Data Transmission process is ongoing               */
  DAL_I2S_STATE_BUSY_RX    = 0x04U,  /*!< Data Reception process is ongoing                  */
  DAL_I2S_STATE_BUSY_TX_RX = 0x05U,  /*!< Data Transmission and Reception process is ongoing */
  DAL_I2S_STATE_TIMEOUT    = 0x06U,  /*!< I2S timeout state                                  */
  DAL_I2S_STATE_ERROR      = 0x07U   /*!< I2S error state                                    */
} DAL_I2S_StateTypeDef;

/**
  * @brief I2S handle Structure definition
  */
typedef struct __I2S_HandleTypeDef
{
  SPI_TypeDef                *Instance;    /*!< I2S registers base address */

  I2S_InitTypeDef            Init;         /*!< I2S communication parameters */

  uint16_t                   *pTxBuffPtr;  /*!< Pointer to I2S Tx transfer buffer */

  __IO uint16_t              TxXferSize;   /*!< I2S Tx transfer size */

  __IO uint16_t              TxXferCount;  /*!< I2S Tx transfer Counter */

  uint16_t                   *pRxBuffPtr;  /*!< Pointer to I2S Rx transfer buffer */

  __IO uint16_t              RxXferSize;   /*!< I2S Rx transfer size */

  __IO uint16_t              RxXferCount;  /*!< I2S Rx transfer counter
                                              (This field is initialized at the
                                               same value as transfer size at the
                                               beginning of the transfer and
                                               decremented when a sample is received
                                               NbSamplesReceived = RxBufferSize-RxBufferCount) */
  void (*IrqHandlerISR)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S function pointer on IrqHandler   */

  DMA_HandleTypeDef          *hdmatx;      /*!< I2S Tx DMA handle parameters */

  DMA_HandleTypeDef          *hdmarx;      /*!< I2S Rx DMA handle parameters */

  __IO DAL_LockTypeDef       Lock;         /*!< I2S locking object */

  __IO DAL_I2S_StateTypeDef  State;        /*!< I2S communication state */

  __IO uint32_t              ErrorCode;    /*!< I2S Error code
                                                This parameter can be a value of @ref I2S_Error */

#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  void (* TxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);             /*!< I2S Tx Completed callback          */
  void (* RxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);             /*!< I2S Rx Completed callback          */
  void (* TxRxCpltCallback)(struct __I2S_HandleTypeDef *hi2s);           /*!< I2S TxRx Completed callback        */
  void (* TxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S Tx Half Completed callback     */
  void (* RxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);         /*!< I2S Rx Half Completed callback     */
  void (* TxRxHalfCpltCallback)(struct __I2S_HandleTypeDef *hi2s);       /*!< I2S TxRx Half Completed callback   */
  void (* ErrorCallback)(struct __I2S_HandleTypeDef *hi2s);              /*!< I2S Error callback                 */
  void (* MspInitCallback)(struct __I2S_HandleTypeDef *hi2s);            /*!< I2S Msp Init callback              */
  void (* MspDeInitCallback)(struct __I2S_HandleTypeDef *hi2s);          /*!< I2S Msp DeInit callback            */

#endif  /* USE_DAL_I2S_REGISTER_CALLBACKS */
} I2S_HandleTypeDef;

#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
/**
  * @brief  DAL I2S Callback ID enumeration definition
  */
typedef enum
{
  DAL_I2S_TX_COMPLETE_CB_ID             = 0x00U,    /*!< I2S Tx Completed callback ID         */
  DAL_I2S_RX_COMPLETE_CB_ID             = 0x01U,    /*!< I2S Rx Completed callback ID         */
  DAL_I2S_TX_RX_COMPLETE_CB_ID          = 0x02U,    /*!< I2S TxRx Completed callback ID       */
  DAL_I2S_TX_HALF_COMPLETE_CB_ID        = 0x03U,    /*!< I2S Tx Half Completed callback ID    */
  DAL_I2S_RX_HALF_COMPLETE_CB_ID        = 0x04U,    /*!< I2S Rx Half Completed callback ID    */
  DAL_I2S_TX_RX_HALF_COMPLETE_CB_ID     = 0x05U,    /*!< I2S TxRx Half Completed callback ID  */
  DAL_I2S_ERROR_CB_ID                   = 0x06U,    /*!< I2S Error callback ID                */
  DAL_I2S_MSPINIT_CB_ID                 = 0x07U,    /*!< I2S Msp Init callback ID             */
  DAL_I2S_MSPDEINIT_CB_ID               = 0x08U     /*!< I2S Msp DeInit callback ID           */

} DAL_I2S_CallbackIDTypeDef;

/**
  * @brief  DAL I2S Callback pointer definition
  */
typedef  void (*pI2S_CallbackTypeDef)(I2S_HandleTypeDef *hi2s); /*!< pointer to an I2S callback function */

#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2S_Exported_Constants I2S Exported Constants
  * @{
  */
/** @defgroup I2S_Error I2S Error
  * @{
  */
#define DAL_I2S_ERROR_NONE               (0x00000000U)  /*!< No error                    */
#define DAL_I2S_ERROR_TIMEOUT            (0x00000001U)  /*!< Timeout error               */
#define DAL_I2S_ERROR_OVR                (0x00000002U)  /*!< OVR error                   */
#define DAL_I2S_ERROR_UDR                (0x00000004U)  /*!< UDR error                   */
#define DAL_I2S_ERROR_DMA                (0x00000008U)  /*!< DMA transfer error          */
#define DAL_I2S_ERROR_PRESCALER          (0x00000010U)  /*!< Prescaler Calculation error */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
#define DAL_I2S_ERROR_INVALID_CALLBACK   (0x00000020U)  /*!< Invalid Callback error      */
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
#define DAL_I2S_ERROR_BUSY_LINE_RX       (0x00000040U)  /*!< Busy Rx Line error          */
/**
  * @}
  */

/** @defgroup I2S_Mode I2S Mode
  * @{
  */
#define I2S_MODE_SLAVE_TX                (0x00000000U)
#define I2S_MODE_SLAVE_RX                (SPI_I2SCFG_I2SMOD_0)
#define I2S_MODE_MASTER_TX               (SPI_I2SCFG_I2SMOD_1)
#define I2S_MODE_MASTER_RX               ((SPI_I2SCFG_I2SMOD_0 | SPI_I2SCFG_I2SMOD_1))
/**
  * @}
  */

/** @defgroup I2S_Standard I2S Standard
  * @{
  */
#define I2S_STANDARD_PHILIPS             (0x00000000U)
#define I2S_STANDARD_MSB                 (SPI_I2SCFG_I2SSSEL_0)
#define I2S_STANDARD_LSB                 (SPI_I2SCFG_I2SSSEL_1)
#define I2S_STANDARD_PCM_SHORT           ((SPI_I2SCFG_I2SSSEL_0 | SPI_I2SCFG_I2SSSEL_1))
#define I2S_STANDARD_PCM_LONG            ((SPI_I2SCFG_I2SSSEL_0 | SPI_I2SCFG_I2SSSEL_1 | SPI_I2SCFG_PFSSEL))
/**
  * @}
  */

/** @defgroup I2S_Data_Format I2S Data Format
  * @{
  */
#define I2S_DATAFORMAT_16B               (0x00000000U)
#define I2S_DATAFORMAT_16B_EXTENDED      (SPI_I2SCFG_CHLEN)
#define I2S_DATAFORMAT_24B               ((SPI_I2SCFG_CHLEN | SPI_I2SCFG_DATALEN_0))
#define I2S_DATAFORMAT_32B               ((SPI_I2SCFG_CHLEN | SPI_I2SCFG_DATALEN_1))
/**
  * @}
  */

/** @defgroup I2S_MCLK_Output I2S MCLK Output
  * @{
  */
#define I2S_MCLKOUTPUT_ENABLE            (SPI_I2SPSC_MCOEN)
#define I2S_MCLKOUTPUT_DISABLE           (0x00000000U)
/**
  * @}
  */

/** @defgroup I2S_Audio_Frequency I2S Audio Frequency
  * @{
  */
#define I2S_AUDIOFREQ_192K               (192000U)
#define I2S_AUDIOFREQ_96K                (96000U)
#define I2S_AUDIOFREQ_48K                (48000U)
#define I2S_AUDIOFREQ_44K                (44100U)
#define I2S_AUDIOFREQ_32K                (32000U)
#define I2S_AUDIOFREQ_22K                (22050U)
#define I2S_AUDIOFREQ_16K                (16000U)
#define I2S_AUDIOFREQ_11K                (11025U)
#define I2S_AUDIOFREQ_8K                 (8000U)
#define I2S_AUDIOFREQ_DEFAULT            (2U)
/**
  * @}
  */

/** @defgroup I2S_FullDuplex_Mode I2S FullDuplex Mode
  * @{
  */
#define I2S_FULLDUPLEXMODE_DISABLE       (0x00000000U)
#define I2S_FULLDUPLEXMODE_ENABLE        (0x00000001U)
/**
  * @}
  */

/** @defgroup I2S_Clock_Polarity I2S Clock Polarity
  * @{
  */
#define I2S_CPOL_LOW                     (0x00000000U)
#define I2S_CPOL_HIGH                    (SPI_I2SCFG_CPOL)
/**
  * @}
  */

/** @defgroup I2S_Interrupts_Definition I2S Interrupts Definition
  * @{
  */
#define I2S_IT_TXE                       SPI_CTRL2_TXBEIEN
#define I2S_IT_RXNE                      SPI_CTRL2_RXBNEIEN
#define I2S_IT_ERR                       SPI_CTRL2_ERRIEN
/**
  * @}
  */

/** @defgroup I2S_Flags_Definition I2S Flags Definition
  * @{
  */
#define I2S_FLAG_TXE                     SPI_STS_TXBEFLG
#define I2S_FLAG_RXNE                    SPI_STS_RXBNEFLG

#define I2S_FLAG_UDR                     SPI_STS_UDRFLG
#define I2S_FLAG_OVR                     SPI_STS_OVRFLG
#define I2S_FLAG_FRE                     SPI_STS_FFERR

#define I2S_FLAG_CHSIDE                  SPI_STS_SCHDIR
#define I2S_FLAG_BSY                     SPI_STS_BSYFLG

#define I2S_FLAG_MASK                   (SPI_STS_RXBNEFLG\
                                         | SPI_STS_TXBEFLG | SPI_STS_UDRFLG | SPI_STS_OVRFLG | SPI_STS_FFERR | SPI_STS_SCHDIR | SPI_STS_BSYFLG)
/**
  * @}
  */

/** @defgroup I2S_Clock_Source I2S Clock Source Definition
  * @{
  */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx) 
#define I2S_CLOCK_PLL                    (0x00000000U)
#define I2S_CLOCK_EXTERNAL               (0x00000001U)
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup I2S_Exported_macros I2S Exported Macros
  * @{
  */

/** @brief  Reset I2S handle state
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
#define __DAL_I2S_RESET_HANDLE_STATE(__HANDLE__)                do{                                                  \
                                                                    (__HANDLE__)->State = DAL_I2S_STATE_RESET;       \
                                                                    (__HANDLE__)->MspInitCallback = NULL;            \
                                                                    (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                                  } while(0)
#else
#define __DAL_I2S_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_I2S_STATE_RESET)
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */

/** @brief  Enable the specified SPI peripheral (in I2S mode).
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2S_ENABLE(__HANDLE__)    (SET_BIT((__HANDLE__)->Instance->I2SCFG, SPI_I2SCFG_I2SEN))

/** @brief  Disable the specified SPI peripheral (in I2S mode).
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2S_DISABLE(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->I2SCFG, SPI_I2SCFG_I2SEN))

/** @brief  Enable the specified I2S interrupts.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *         This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __DAL_I2S_ENABLE_IT(__HANDLE__, __INTERRUPT__)    (SET_BIT((__HANDLE__)->Instance->CTRL2,(__INTERRUPT__)))

/** @brief  Disable the specified I2S interrupts.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *         This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __DAL_I2S_DISABLE_IT(__HANDLE__, __INTERRUPT__) (CLEAR_BIT((__HANDLE__)->Instance->CTRL2,(__INTERRUPT__)))

/** @brief  Checks if the specified I2S interrupt source is enabled or disabled.
  * @param  __HANDLE__ specifies the I2S Handle.
  *         This parameter can be I2S where x: 1, 2, or 3 to select the I2S peripheral.
  * @param  __INTERRUPT__ specifies the I2S interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __DAL_I2S_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CTRL2\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks whether the specified I2S flag is set or not.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg I2S_FLAG_RXNE: Receive buffer not empty flag
  *            @arg I2S_FLAG_TXE: Transmit buffer empty flag
  *            @arg I2S_FLAG_UDR: Underrun flag
  *            @arg I2S_FLAG_OVR: Overrun flag
  *            @arg I2S_FLAG_FRE: Frame error flag
  *            @arg I2S_FLAG_CHSIDE: Channel Side flag
  *            @arg I2S_FLAG_BSY: Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __DAL_I2S_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->STS) & (__FLAG__)) == (__FLAG__))

/** @brief Clears the I2S OVR pending flag.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2S_CLEAR_OVRFLAG(__HANDLE__) do{ \
                                                __IO uint32_t tmpreg_ovr = 0x00U; \
                                                tmpreg_ovr = (__HANDLE__)->Instance->DATA; \
                                                tmpreg_ovr = (__HANDLE__)->Instance->STS; \
                                                UNUSED(tmpreg_ovr); \
                                              }while(0U)
/** @brief Clears the I2S UDR pending flag.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2S_CLEAR_UDRFLAG(__HANDLE__) do{\
                                                __IO uint32_t tmpreg_udr = 0x00U;\
                                                tmpreg_udr = ((__HANDLE__)->Instance->STS);\
                                                UNUSED(tmpreg_udr); \
                                              }while(0U)
/** @brief Flush the I2S DR Register.
  * @param  __HANDLE__ specifies the I2S Handle.
  * @retval None
  */
#define __DAL_I2S_FLUSH_RX_DR(__HANDLE__)  do{\
                                                __IO uint32_t tmpreg_dr = 0x00U;\
                                                tmpreg_dr = ((__HANDLE__)->Instance->DATA);\
                                                UNUSED(tmpreg_dr); \
                                              }while(0U)
/**
  * @}
  */

/* Include I2S Extension module */
#include "apm32f4xx_dal_i2s_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2S_Exported_Functions
  * @{
  */

/** @addtogroup I2S_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
DAL_StatusTypeDef DAL_I2S_Init(I2S_HandleTypeDef *hi2s);
DAL_StatusTypeDef DAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void DAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void DAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
DAL_StatusTypeDef DAL_I2S_RegisterCallback(I2S_HandleTypeDef *hi2s, DAL_I2S_CallbackIDTypeDef CallbackID,
                                           pI2S_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_I2S_UnRegisterCallback(I2S_HandleTypeDef *hi2s, DAL_I2S_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
DAL_StatusTypeDef DAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);

/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void DAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
DAL_StatusTypeDef DAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

DAL_StatusTypeDef DAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
DAL_StatusTypeDef DAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
DAL_StatusTypeDef DAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

/* Callbacks used in non blocking modes (Interrupt and DMA) *******************/
void DAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void DAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void DAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void DAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void DAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control and State functions  ************************************/
DAL_I2S_StateTypeDef DAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t DAL_I2S_GetError(I2S_HandleTypeDef *hi2s);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup I2S_Private_Macros I2S Private Macros
  * @{
  */

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __STS__  copy of I2S STS register.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg I2S_FLAG_RXNE: Receive buffer not empty flag
  *            @arg I2S_FLAG_TXE: Transmit buffer empty flag
  *            @arg I2S_FLAG_UDR: Underrun error flag
  *            @arg I2S_FLAG_OVR: Overrun flag
  *            @arg I2S_FLAG_CHSIDE: Channel side flag
  *            @arg I2S_FLAG_BSY: Busy flag
  * @retval SET or RESET.
  */
#define I2S_CHECK_FLAG(__STS__, __FLAG__)         ((((__STS__)\
                                                    & ((__FLAG__) & I2S_FLAG_MASK)) == ((__FLAG__) & I2S_FLAG_MASK)) ? SET : RESET)

/** @brief  Check whether the specified SPI Interrupt is set or not.
  * @param  __CTRL2__  copy of I2S CTRL2 register.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg I2S_IT_TXE: Tx buffer empty interrupt enable
  *            @arg I2S_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg I2S_IT_ERR: Error interrupt enable
  * @retval SET or RESET.
  */
#define I2S_CHECK_IT_SOURCE(__CTRL2__, __INTERRUPT__)      ((((__CTRL2__)\
                                                            & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks if I2S Mode parameter is in allowed range.
  * @param  __MODE__ specifies the I2S Mode.
  *         This parameter can be a value of @ref I2S_Mode
  * @retval None
  */
#define IS_I2S_MODE(__MODE__) (((__MODE__) == I2S_MODE_SLAVE_TX)  || \
                               ((__MODE__) == I2S_MODE_SLAVE_RX)  || \
                               ((__MODE__) == I2S_MODE_MASTER_TX) || \
                               ((__MODE__) == I2S_MODE_MASTER_RX))

#define IS_I2S_STANDARD(__STANDARD__) (((__STANDARD__) == I2S_STANDARD_PHILIPS)   || \
                                       ((__STANDARD__) == I2S_STANDARD_MSB)       || \
                                       ((__STANDARD__) == I2S_STANDARD_LSB)       || \
                                       ((__STANDARD__) == I2S_STANDARD_PCM_SHORT) || \
                                       ((__STANDARD__) == I2S_STANDARD_PCM_LONG))

#define IS_I2S_DATA_FORMAT(__FORMAT__) (((__FORMAT__) == I2S_DATAFORMAT_16B)          || \
                                        ((__FORMAT__) == I2S_DATAFORMAT_16B_EXTENDED) || \
                                        ((__FORMAT__) == I2S_DATAFORMAT_24B)          || \
                                        ((__FORMAT__) == I2S_DATAFORMAT_32B))

#define IS_I2S_MCLK_OUTPUT(__OUTPUT__) (((__OUTPUT__) == I2S_MCLKOUTPUT_ENABLE) || \
                                        ((__OUTPUT__) == I2S_MCLKOUTPUT_DISABLE))

#define IS_I2S_AUDIO_FREQ(__FREQ__) ((((__FREQ__) >= I2S_AUDIOFREQ_8K)    && \
                                      ((__FREQ__) <= I2S_AUDIOFREQ_192K)) || \
                                     ((__FREQ__) == I2S_AUDIOFREQ_DEFAULT))

#define IS_I2S_FULLDUPLEX_MODE(MODE) (((MODE) == I2S_FULLDUPLEXMODE_DISABLE) || \
                                      ((MODE) == I2S_FULLDUPLEXMODE_ENABLE))

/** @brief  Checks if I2S Serial clock steady state parameter is in allowed range.
  * @param  __CPOL__ specifies the I2S serial clock steady state.
  *         This parameter can be a value of @ref I2S_Clock_Polarity
  * @retval None
  */
#define IS_I2S_CPOL(__CPOL__) (((__CPOL__) == I2S_CPOL_LOW) || \
                               ((__CPOL__) == I2S_CPOL_HIGH))

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx) || defined(APM32F411xx)
#define IS_I2S_CLOCKSOURCE(CLOCK) (((CLOCK) == I2S_CLOCK_EXTERNAL) ||\
                                   ((CLOCK) == I2S_CLOCK_PLL))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

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

#endif /* APM32F4xx_DAL_I2S_H */

