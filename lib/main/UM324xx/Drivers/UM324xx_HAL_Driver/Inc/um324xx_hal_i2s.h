/**
 ******************************************************************************
 * @file     um324xx_hal_i2s.h
 * @author   MCU Team
 * @version  V1.00
 * @date     2023-04-11
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

#ifndef __UM324XX_HAL_I2S_H__
#define __UM324XX_HAL_I2S_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "um324xx_hal_def.h"
/** @addtogroup UM324xx_HAL_Driver
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
    uint32_t Mode;    /*!< Specifies the I2S operating mode.
                         This parameter can be a value of @ref I2S_Mode */
    uint32_t MclkDiv; /*!< Specifies whether the I2S MCLK output is enabled or not.
                         This parameter can be a value of @ref I2S_MCLK_DIV */

    uint32_t AudioFreq; /*!< Specifies the frequency selected for the I2S communication.
                           This parameter can be a value of @ref I2S_Audio_Frequency */

    uint32_t ClockSource;    /*!< Specifies the I2S Clock Source. */
                                 
    uint32_t FullDuplexMode; /*!< Specifies the I2S FullDuplex mode.
                                This parameter can be a value of @ref I2S_FullDuplex_Mode */

    uint32_t AccessFiFoMode; /*!< Access FIFO mode */

    uint32_t IrqEn; /*!< Interrupt Enable */

    uint32_t FillDateSel; /*!< Fill data selection when TXFIFO is underloaded*/

    uint32_t TxfifoWtmk; /*!< TXFIFO watermark (number of triggers) selection */

    uint32_t RxfifoWtmk; /*!< RXFIFO watermark (number of triggers) selection */

    uint32_t PcmSync; /*!< PCM frame synchronization format selection */

    uint32_t MonoDataSel; /*!< Left/Right Mono Selection */

    uint32_t LeftRightFirst; /*!< Two-channel audio priority channel selection */

    uint32_t SckEdgeSel; /*!< SCK edge selection where data switching is located */

    uint32_t WsPolarity; /*!< WS Polarity Selection */

    uint32_t ChannelSel; /*!< Binaural or mono data selection */

    uint32_t Channellen; /*!< Channel Length */

    uint32_t DataFormat; /*!< Specifies the data format for the I2S communication.
                            This parameter can be a value of @ref I2S_Data_Format */

    uint32_t Standard; /*!< Specifies the standard used for the I2S communication.
                          This parameter can be a value of @ref I2S_Standard */

} I2S_InitTypeDef;

/**
 * @brief  HAL State structures definition
 */
typedef enum {
    HAL_I2S_STATE_RESET      = 0x00U, /*!< I2S not yet initialized or disabled                */
    HAL_I2S_STATE_READY      = 0x01U, /*!< I2S initialized and ready for use                  */
    HAL_I2S_STATE_BUSY       = 0x02U, /*!< I2S internal process is ongoing                    */
    HAL_I2S_STATE_BUSY_TX    = 0x03U, /*!< Data Transmission process is ongoing               */
    HAL_I2S_STATE_BUSY_RX    = 0x04U, /*!< Data Reception process is ongoing                  */
    HAL_I2S_STATE_BUSY_TX_RX = 0x05U, /*!< Data Transmission and Reception process is ongoing */
    HAL_I2S_STATE_TIMEOUT    = 0x06U, /*!< I2S timeout state                                  */
    HAL_I2S_STATE_ERROR      = 0x07U  /*!< I2S error state                                    */
} HAL_I2S_StateTypeDef;

/**
 * @brief I2S handle Structure definition
 */
typedef struct __I2S_HandleTypeDef {
    I2S_TypeDef *Instance; /*!< I2S registers base address */

    I2S_InitTypeDef Init; /*!< I2S communication parameters */

    uint32_t *pTxBuffPtr; /*!< Pointer to I2S Tx transfer buffer */

    __IO uint16_t TxXferSize; /*!< I2S Tx transfer size */

    __IO uint16_t TxXferCount; /*!< I2S Tx transfer Counter */

    uint32_t *pRxBuffPtr; /*!< Pointer to I2S Rx transfer buffer */

    __IO uint16_t RxXferSize; /*!< I2S Rx transfer size */

    __IO uint16_t RxXferCount;                               /*!< I2S Rx transfer counter
                                                                (This field is initialized at the
                                                                 same value as transfer size at the
                                                                 beginning of the transfer and
                                                                 decremented when a sample is received
                                                                 NbSamplesReceived = RxBufferSize-RxBufferCount) */
    void (*IrqHandlerISR)(struct __I2S_HandleTypeDef *hi2s); /*!< I2S function pointer on IrqHandler   */

    DMA_HandleTypeDef *hdmatx; /*!< I2S Tx DMA handle parameters */

    DMA_HandleTypeDef *hdmarx; /*!< I2S Rx DMA handle parameters */

    __IO HAL_LockTypeDef Lock; /*!< I2S locking object */

    __IO HAL_I2S_StateTypeDef State; /*!< I2S communication state */

    __IO uint32_t ErrorCode; /*!< I2S Error code
                                  This parameter can be a value of @ref I2S_Error */

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
    void (*TxFifoEnoughCallback)(struct __I2S_HandleTypeDef *hi2s); /*!< I2S Tx Completed callback          */
    void (*RxFifoEnoughCallback)(struct __I2S_HandleTypeDef *hi2s); /*!< I2S Rx Completed callback          */
    void (*RxFifoOvfCallback)(struct __I2S_HandleTypeDef *hi2s);    /*!< I2S TxRx Completed callback        */
    void (*TxFifoUdrCallback)(struct __I2S_HandleTypeDef *hi2s);    /*!< I2S Tx Half Completed callback     */
    void (*FrameErrCallback)(struct __I2S_HandleTypeDef *hi2s);     /*!< I2S Rx Half Completed callback     */
    void (*MspInitCallback)(struct __I2S_HandleTypeDef *hi2s);      /*!< I2S Msp Init callback              */
    void (*MspDeInitCallback)(struct __I2S_HandleTypeDef *hi2s);    /*!< I2S Msp DeInit callback            */

#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
} I2S_HandleTypeDef;

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
/**
 * @brief  HAL I2S Callback ID enumeration definition
 */
typedef enum {
    HAL_I2S_TXFIFO_ENOUGH_CB_ID   = 0x00U, /*!< I2S Tx Completed callback ID         */
    HAL_I2S_RXFIFO_ENOUGH_CB_ID   = 0x01U, /*!< I2S Rx Completed callback ID         */
    HAL_I2S_RXFIFO_OVERFLOW_CB_ID = 0x02U, /*!< I2S TxRx Completed callback ID       */
    HAL_I2S_TXFIFO_UDR_ID         = 0x03U, /*!< I2S Tx Half Completed callback ID    */
    HAL_I2S_FRAME_ERR_CB_ID       = 0x04U, /*!< I2S Rx Half Completed callback ID    */

    HAL_I2S_MSPINIT_CB_ID   = 0x05U, /*!< I2S Msp Init callback ID             */
    HAL_I2S_MSPDEINIT_CB_ID = 0x06U  /*!< I2S Msp DeInit callback ID           */

} HAL_I2S_CallbackIDTypeDef;

/**
 * @brief  HAL I2S Callback pointer definition
 */
typedef void (*pI2S_CallbackTypeDef)(I2S_HandleTypeDef *hi2s); /*!< pointer to an I2S callback function */

#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */

typedef enum {
    I2S_MCLK_DIV_0   = 0x7,
    I2S_MCLK_DIV_2   = 0x8,
    I2S_MCLK_DIV_4   = 0x9,
    I2S_MCLK_DIV_8   = 0xA,
    I2S_MCLK_DIV_16  = 0xB,
    I2S_MCLK_DIV_32  = 0xC,
    I2S_MCLK_DIV_64  = 0xD,
    I2S_MCLK_DIV_128 = 0xE,
    I2S_MCLK_DIV_256 = 0xF,
} i2s_mclk_div_t;

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
#define HAL_I2S_ERROR_NONE      (0x00000000U) /*!< No error                    */
#define HAL_I2S_ERROR_TIMEOUT   (0x00000001U) /*!< Timeout error               */
#define HAL_I2S_ERROR_OVR       (0x00000002U) /*!< OVR error                   */
#define HAL_I2S_ERROR_UDR       (0x00000004U) /*!< UDR error                   */
#define HAL_I2S_ERROR_DMA       (0x00000008U) /*!< DMA transfer error          */
#define HAL_I2S_ERROR_PRESCALER (0x00000010U) /*!< Prescaler Calculation error */
#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
#define HAL_I2S_ERROR_INVALID_CALLBACK (0x00000020U) /*!< Invalid Callback error      */
#endif                                               /* USE_HAL_I2S_REGISTER_CALLBACKS */
#define HAL_I2S_ERROR_BUSY_LINE_RX (0x00000040U)     /*!< Busy Rx Line error          */
/**
 * @}
 */

/** @defgroup I2S_Mode I2S Mode
 * @{
 */
#define I2S_MODE_MASTER (I2S_GCR_MST_MODE)

#define I2S_MODE_SLAVE (0x00000000U)

/**
 * @}
 */

/** @defgroup I2S_Standard I2S Standard
 * @{
 */
#define I2S_STANDARD_PHILIPS   (0x00000000U)
#define I2S_STANDARD_MSB       (I2S_DFR_I2S_STD_0)
#define I2S_STANDARD_LSB       (I2S_DFR_I2S_STD_1)
#define I2S_STANDARD_PCM_SHORT ((I2S_DFR_I2S_STD_0 | I2S_DFR_I2S_STD_1))
#define I2S_STANDARD_PCM_LONG  ((I2S_DFR_I2S_STD_0 | I2S_DFR_I2S_STD_1 | I2S_DFR_PCM_SYNC))
/**
 * @}
 */

/** @defgroup I2S_Data_Format I2S Data Format
 * @{
 */
#define I2S_DATAFORMAT_8B  (0x00000000U)
#define I2S_DATAFORMAT_16B (I2S_DFR_DATA_LENSEL_0)
#define I2S_DATAFORMAT_24B ((I2S_DFR_DATA_LENSEL_1))
#define I2S_DATAFORMAT_32B ((I2S_DFR_DATA_LENSEL_1 | I2S_DFR_DATA_LENSEL_0))
/**
 * @}
 */

/** @defgroup I2S_Audio_Frequency I2S Audio Frequency
 * @{
 */
#define I2S_AUDIOFREQ_192K    (192000U)
#define I2S_AUDIOFREQ_96K     (96000U)
#define I2S_AUDIOFREQ_48K     (48000U)
#define I2S_AUDIOFREQ_44K     (44100U)
#define I2S_AUDIOFREQ_32K     (32000U)
#define I2S_AUDIOFREQ_22K     (22050U)
#define I2S_AUDIOFREQ_16K     (16000U)
#define I2S_AUDIOFREQ_11K     (11025U)
#define I2S_AUDIOFREQ_8K      (8000U)
#define I2S_AUDIOFREQ_DEFAULT (2U)
/**
 * @}
 */

/** @defgroup I2S_FullDuplex_Mode I2S FullDuplex Mode
 * @{
 */
#define I2S_FULLDUPLEXMODE_DISABLE (0x00000000U)
#define I2S_FULLDUPLEXMODE_ENABLE  (0x00000001U)
/**
 * @}
 */

/** @defgroup I2S_Clock_Polarity I2S Clock Polarity
 * @{
 */
#define I2S_CPOL_LOW  (0x00000000U)
#define I2S_CPOL_HIGH (I2S_DFR_SCK_EDGESEL)
/**
 * @}
 */

/** @defgroup I2S_Interrupts_Definition I2S Interrupts Definition
 * @{
 */
#define I2S_IT_FRAME_ERR I2S_IER_FRAME_ERR_INTEN
#define I2S_IT_UDR       I2S_IER_UNDERRUN_INTEN

#define I2S_IT_RX_OVF         I2S_IER_RXOERR_INTEN
#define I2S_IT_RXFIFO_ENOUGH  I2S_IER_RX_INTEN
#define I2S_IT_TXSPACE_ENOUGH I2S_IER_TX_INTEN
/**
 * @}
 */

/** @defgroup I2S_Flags_Definition I2S Flags Definition
 * @{
 */
#define I2S_FLAG_TXE         I2S_CSR_TXFIFO_EMPTY
#define I2S_FLAG_TXFIFO_FULL I2S_CSR_TXFIFO_FULL
#define I2S_FLAG_TXFIFO_TRIG I2S_CSR_TXFIFO_TRIG

#define I2S_FLAG_RXE         I2S_CSR_RXFIFO_EMPTY
#define I2S_FLAG_RXFIFO_FULL I2S_CSR_RXFIFO_FULL
#define I2S_FLAG_RXFIFO_TRIG I2S_CSR_RXFIFO_TRIG

#define I2S_FLAG_MASK (I2S_CSR_TXFIFO_EMPTY | I2S_CSR_TXFIFO_FULL | I2S_CSR_TXFIFO_TRIG | \
                       I2S_CSR_RXFIFO_EMPTY | I2S_CSR_RXFIFO_FULL |                       \
                       I2S_CSR_RXFIFO_TRIG | I2S_CSR_RXFIFO_LEVEL I2S_CSR_TXFIFO_LEVEL)
/**
 * @}
 */

/** @defgroup I2S_Flags_Definition I2S Flags Definition
 * @{
 */
#define I2S_FLAG_FRAME_ERR I2S_ISR_FRAME_ERR_INTF
#define I2S_FLAG_UDR       I2S_ISR_UNDERRUN_INTF

#define I2S_FLAG_RX_OVF         I2S_ISR_RXOERR_INTF
#define I2S_FLAG_RXFIFO_ENOUGH  I2S_ISR_RX_INTF
#define I2S_FLAG_TXSPACE_ENOUGH I2S_ISR_TX_INTF

/**
 * @}
 */
/** @defgroup I2S_Flags_Definition I2S Flags Definition
 * @{
 */
#define I2S_DMA_ACCESS_MODE    (I2S_GCR_DMA_MODE)
#define I2S_NORMAL_ACCESS_MODE (0x00000000U)

#define I2S_IRQ_ENABLE  (I2S_GCR_INTEN)
#define I2S_IRQ_DISABLE (0x00000000U)

#define I2S_FILL_DUMMY_DATA    ((uint32_t)0x00000000U)
#define I2S_FILL_PREVIOUS_DATA ((uint32_t)I2S_GCR_FILLDATASEL)

#define I2S_TX_FIFO_WTMK_VACANCY_1 ((uint32_t)0x00000000)
#define I2S_TX_FIFO_WTMK_VACANCY_4 ((uint32_t)I2S_GCR_TXFIFO_WTMK)

#define I2S_RX_FIFO_WTMK_DATA_1 ((uint32_t)0x00000000)
#define I2S_RX_FIFO_WTMK_DATA_4 ((uint32_t)I2S_GCR_RXFIFO_WTMK)

#define I2S_SHORT_FRAME_SYNC ((uint32_t)0x00000000)
#define I2S_LONG_FRAME_SYNC  ((uint32_t)I2S_DFR_PCM_SYNC)

#define I2S_SEL_LEFT_MONO  ((uint32_t)0x00000000)
#define I2S_SEL_RIGHT_MONO ((uint32_t)I2S_DFR_RIGHT_MONO)

#define I2S_LEFT_MONO_FIRST  ((uint32_t)0x00000000)
#define I2S_RIGHT_MONO_FIRST ((uint32_t)I2S_DFR_RIGHT_FIRST)

#define I2S_SCK_EDGE_RISING ((uint32_t)I2S_DFR_SCK_EDGESEL)
#define I2S_SCK_EDGE_FALING ((uint32_t)0x00000000)

#define I2S_WS_POL_LOW  ((uint32_t)0x00000000)
#define I2S_WS_POL_HIGH ((uint32_t)I2S_DFR_WSPOL)

#define I2S_MONO   ((uint32_t)0x00000000)
#define I2S_STEREO ((uint32_t)I2S_DFR_STER_MONO)

#define I2S_CHANNEL_LEN_16BITS ((uint32_t)0x00000000)
#define I2S_CHANNEL_LEN_32BITS ((uint32_t)I2S_DFR_CHANLEN32)

#define I2S_AUDIO_DATA_8BITS  ((uint32_t)0x00000000)
#define I2S_AUDIO_DATA_16BITS ((uint32_t)I2S_DFR_DATA_LENSEL_0)
#define I2S_AUDIO_DATA_24BITS ((uint32_t)I2S_DFR_DATA_LENSEL_1)
#define I2S_AUDIO_DATA_32BITS ((uint32_t)(I2S_DFR_DATA_LENSEL_1) | (uint32_t)(I2S_DFR_DATA_LENSEL_0))

#define I2S_PHILIP_STANDARD ((uint32_t)0x00000000)
#define I2S_MSB_STANDARD    ((uint32_t)I2S_DFR_I2S_STD_0)
#define I2S_LSB_STANDARD    ((uint32_t)I2S_DFR_I2S_STD_1)
#define I2S_PCM_STANDARD    ((uint32_t)(I2S_DFR_I2S_STD_1) | (uint32_t)(I2S_DFR_I2S_STD_0))

/**
 * @}
 */


/** @defgroup I2S_Flags_Definition I2S clear IT Flags Definition
 * @{
 */
#define  TX_INTCLR            I2S_ICR_TX_INTCLR
#define  RX_INTCLR            I2S_ICR_RX_INTCLR
#define  RXOERR_INTCLR        I2S_ICR_RXOERR_INTCLR
#define  UNDERRUN_INTCLR      I2S_ICR_UNDERRUN_INTCLR
#define  FRAME_ERR_INTCLR     I2S_ICR_FRAME_ERR_INTCLR
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
#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
#define __HAL_I2S_RESET_HANDLE_STATE(__HANDLE__)               \
    do {                                                       \
        (__HANDLE__)->State             = HAL_I2S_STATE_RESET; \
        (__HANDLE__)->MspInitCallback   = NULL;                \
        (__HANDLE__)->MspDeInitCallback = NULL;                \
    } while (0)
#else
#define __HAL_I2S_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_I2S_STATE_RESET)
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */

/** @brief  Enable the specified SPI peripheral (in I2S mode).
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_ENABLE(__HANDLE__) (SET_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_I2S_EN))

/** @brief  Disable the specified SPI peripheral (in I2S mode).
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_DISABLE(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_I2S_EN))

/** @brief  Enable the specified I2S interrupts.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
 *         This parameter can be one of the following values:
 *            @arg I2S_IT_TXSPACE_ENOUGH: Tx buffer empty interrupt enable
 *            @arg I2S_IT_RXFIFO_ENOUGH: RX buffer not empty interrupt enable
 *            @arg I2S_IT_RX_OVF: Error interrupt enable
    
 * @retval None
 */
#define __HAL_I2S_ENABLE_IT(__HANDLE__, __INTERRUPT__) (SET_BIT((__HANDLE__)->Instance->IER, (__INTERRUPT__)))

/** @brief  Disable the specified I2S interrupts.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
 *         This parameter can be one of the following values:
 *            @arg I2S_IT_TXSPACE_ENOUGH: Tx buffer empty interrupt enable
 *            @arg I2S_IT_RXFIFO_ENOUGH: RX buffer not empty interrupt enable
 *            @arg I2S_IT_RX_OVF: Error interrupt enable
 * @retval None
 */
#define __HAL_I2S_DISABLE_IT(__HANDLE__, __INTERRUPT__) (CLEAR_BIT((__HANDLE__)->Instance->IER, (__INTERRUPT__)))


/** @brief  Enable the  I2S total interrupts.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_ENABLE_GCR_IT(__HANDLE__)         (SET_BIT((__HANDLE__)->Instance->GCR,I2S_GCR_INTEN))

/** @brief  Disable the  I2S total interrupts.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_DisABLE_GCR_IT(__HANDLE__)         (CLEAR_BIT((__HANDLE__)->Instance->GCR,I2S_GCR_INTEN))

/** @brief  Checks if the specified I2S interrupt source is enabled or disabled.
 * @param  __HANDLE__ specifies the I2S Handle.
 *         This parameter can be I2S where x: 1, 2, or 3 to select the I2S peripheral.
 * @param  __INTERRUPT__ specifies the I2S interrupt source to check.
 *          This parameter can be one of the following values:
 *            @arg I2S_IT_TXSPACE_ENOUGH: Tx buffer empty interrupt enable
 *            @arg I2S_IT_RXFIFO_ENOUGH: RX buffer not empty interrupt enable
 *            @arg I2S_IT_RX_OVF: Error interrupt enable
 * @retval The new state of __IT__ (TRUE or FALSE).
 */
#define __HAL_I2S_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->IER & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)


/** @brief  clear the specified I2S interrupts.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
 *         This parameter can be one of the following values:
 *            @arg  TX_INTCLR: Tx buffer   interrupt clear
 *            @arg  RX_INTCLR: Rx buffer   interrupt clear
 *            @arg  RXOERR_INTCLR: Rx buffer over error interrupt clear
 *            @arg  UNDERRUN_INTCLR£ºTXFIFO underload  interrupt clear
 *            @arg  FRAME_ERR_INTCLR£ºframe error interrupt clear
 * @retval None
 */
#define __HAL_I2S_CLR_IT(__HANDLE__, __INTERRUPT__)      (WRITE_REG((__HANDLE__)->Instance->ICR, (__INTERRUPT__)))


/** @brief  Checks whether the specified I2S flag is set or not.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @param  __FLAG__ specifies the flag to check.
 *         This parameter can be one of the following values:
 *            @arg I2S_FLAG_RXE: Receive buffer empty flag
 *            @arg I2S_FLAG_RXFIFO_FULL: Receive buffer not empty flag
 *            @arg I2S_FLAG_RXFIFO_TRIG: Receive buffer enough flags
 *            @arg I2S_FLAG_TXE: Transmit buffer empty flag
 *            @arg I2S_FLAG_TXFIFO_FULL: Transmit buffer not empty flag
 *            @arg I2S_FLAG_TXFIFO_TRIG: Transmit buffer empty and enough flag
 *            @arg I2S_FLAG_MASK: All the above flags
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
 
#define __HAL_I2S_GET_CSR_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->CSR) & (__FLAG__)) == (__FLAG__))

/** @brief  Checks whether the specified I2S flag is set or not.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @param  __FLAG__ specifies the flag to check.
 *         This parameter can be one of the following values:
 *            @arg I2S_FLAG_RXFIFO_ENOUGH: Receive buffer not empty flag
 *            @arg I2S_FLAG_TXSPACE_ENOUGH: Transmit buffer empty flag
 *            @arg I2S_FLAG_UDR: Underrun flag
 *            @arg I2S_FLAG_RX_OVF: Overrun error flag
 *            @arg I2S_FLAG_FRAME_ERR: Frame error flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __HAL_I2S_GET_ISR_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->ISR) & (__FLAG__)) == (__FLAG__))

/** @brief Clears the I2S OVR pending flag.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_CLEAR_OVRFLAG(__HANDLE__)                    \
    do {                                                       \
        __IO uint32_t tmpreg_ovr = 0x00U;                      \
        tmpreg_ovr               = (__HANDLE__)->Instance->DR; \
        tmpreg_ovr               = (__HANDLE__)->Instance->SR; \
        UNUSED(tmpreg_ovr);                                    \
    } while (0U)
/** @brief Clears the I2S UDR pending flag.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_CLEAR_UDRFLAG(__HANDLE__)                      \
    do {                                                         \
        __IO uint32_t tmpreg_udr = 0x00U;                        \
        tmpreg_udr               = ((__HANDLE__)->Instance->SR); \
        UNUSED(tmpreg_udr);                                      \
    } while (0U)
/** @brief Flush the I2S DR Register.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_FLUSH_RX_DR(__HANDLE__)                       \
    do {                                                        \
        __IO uint32_t tmpreg_dr = 0x00U;                        \
        tmpreg_dr               = ((__HANDLE__)->Instance->DR); \
        UNUSED(tmpreg_dr);                                      \
    } while (0U)

/** @brief  Set SD direction output.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_SD_DIR_OUTPUT(__HANDLE__) (SET_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_SD_DIR))

/** @brief  Set SD direction input.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_SD_DIR_INPUT(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_SD_DIR))

/** @brief  Enable tx.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_TX_ENABLE(__HANDLE__) (SET_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_TXEN))

/** @brief  Disable tx.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_TX_DISABLE(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_TXEN))

/** @brief  Enable rx.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_RX_ENABLE(__HANDLE__) (SET_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_RXEN))

/** @brief  Disable rx.
 * @param  __HANDLE__ specifies the I2S Handle.
 * @retval None
 */
#define __HAL_I2S_RX_DISABLE(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->GCR, I2S_GCR_RXEN))

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup I2S_Private_Macros I2S Private Macros
 * @{
 */

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2S_Exported_Functions
 * @{
 */

/** @addtogroup I2S_Exported_Functions_Group1
 * @{
 */
/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void              HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void              HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
HAL_StatusTypeDef HAL_I2S_RegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID,
                                           pI2S_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_I2S_UnRegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
/**
 * @}
 */

/** @addtogroup I2S_Exported_Functions_Group2
 * @{
 */
/* I/O operation functions  ***************************************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size, uint32_t Timeout);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size);
void              HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

/* Callbacks used in non blocking modes (Interrupt and DMA) *******************/
void HAL_I2S_TxFifoEnoughCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxFifoEnoughCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxFifoOvfCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxFifoUdrCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_FrameErrCallback(I2S_HandleTypeDef *hi2s);

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
/**
 * @}
 */

/** @addtogroup I2S_Exported_Functions_Group3
 * @{
 */
/* Peripheral Control and State functions  ************************************/
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t             HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);
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

#endif /* __UM32X42X_HAL_UART_H__ */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
