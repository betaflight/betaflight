/**
  *
  * @file    apm32f4xx_ddl_usart.h
  * @brief   Header file of USART DDL module.
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
#ifndef APM32F4xx_DDL_USART_H
#define APM32F4xx_DDL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (USART1) || defined (USART2) || defined (USART3) || defined (USART6) || defined (UART4) || defined (UART5) || defined (UART7) || defined (UART8) || defined (UART9) || defined (UART10)

/** @defgroup USART_DDL USART
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup USART_DDL_Private_Constants USART Private Constants
  * @{
  */

/* Defines used for the bit position in the register and perform offsets*/
#define USART_POSITION_GTPR_GT                  USART_GTPSC_GRDT_Pos
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_Private_Macros USART Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_ES_INIT USART Exported Init structures
  * @{
  */

/**
  * @brief LL USART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                  /*!< This field defines expected Usart communication baud rate.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetBaudRate().*/

  uint32_t DataWidth;                 /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_DDL_EC_DATAWIDTH.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetDataWidth().*/

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_DDL_EC_STOPBITS.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetStopBitsLength().*/

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_DDL_EC_PARITY.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetParity().*/

  uint32_t TransferDirection;         /*!< Specifies whether the Receive and/or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_DIRECTION.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetTransferDirection().*/

  uint32_t HardwareFlowControl;       /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_HWCONTROL.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetHWFlowCtrl().*/

  uint32_t OverSampling;              /*!< Specifies whether USART oversampling mode is 16 or 8.
                                           This parameter can be a value of @ref USART_DDL_EC_OVERSAMPLING.

                                           This feature can be modified afterwards using unitary function @ref DDL_USART_SetOverSampling().*/

} DDL_USART_InitTypeDef;

/**
  * @brief LL USART Clock Init Structure definition
  */
typedef struct
{
  uint32_t ClockOutput;               /*!< Specifies whether the USART clock is enabled or disabled.
                                           This parameter can be a value of @ref USART_DDL_EC_CLOCK.

                                           USART HW configuration can be modified afterwards using unitary functions
                                           @ref DDL_USART_EnableSCLKOutput() or @ref DDL_USART_DisableSCLKOutput().
                                           For more details, refer to description of this function. */

  uint32_t ClockPolarity;             /*!< Specifies the steady state of the serial clock.
                                           This parameter can be a value of @ref USART_DDL_EC_POLARITY.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetClockPolarity().
                                           For more details, refer to description of this function. */

  uint32_t ClockPhase;                /*!< Specifies the clock transition on which the bit capture is made.
                                           This parameter can be a value of @ref USART_DDL_EC_PHASE.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetClockPhase().
                                           For more details, refer to description of this function. */

  uint32_t LastBitClockPulse;         /*!< Specifies whether the clock pulse corresponding to the last transmitted
                                           data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                                           This parameter can be a value of @ref USART_DDL_EC_LASTCLKPULSE.

                                           USART HW configuration can be modified afterwards using unitary functions @ref DDL_USART_SetLastClkPulseOutput().
                                           For more details, refer to description of this function. */

} DDL_USART_ClockInitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USART_DDL_Exported_Constants USART Exported Constants
  * @{
  */

/** @defgroup USART_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_USART_ReadReg function
  * @{
  */
#define DDL_USART_STS_PEFLG                          USART_STS_PEFLG                   /*!< Parity error flag */
#define DDL_USART_STS_FEFLG                          USART_STS_FEFLG                   /*!< Framing error flag */
#define DDL_USART_STS_NEFLG                          USART_STS_NEFLG                   /*!< Noise detected flag */
#define DDL_USART_STS_OVREFLG                         USART_STS_OVREFLG                  /*!< Overrun error flag */
#define DDL_USART_STS_IDLEFLG                        USART_STS_IDLEFLG                 /*!< Idle line detected flag */
#define DDL_USART_STS_RXBNEFLG                        USART_STS_RXBNEFLG                 /*!< Read data register not empty flag */
#define DDL_USART_STS_TXCFLG                          USART_STS_TXCFLG                   /*!< Transmission complete flag */
#define DDL_USART_STS_TXBEFLG                         USART_STS_TXBEFLG                  /*!< Transmit data register empty flag */
#define DDL_USART_STS_LBDFLG                         USART_STS_LBDFLG                  /*!< LIN break detection flag */
#define DDL_USART_STS_CTSFLG                         USART_STS_CTSFLG                  /*!< CTS flag */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_USART_ReadReg and  DDL_USART_WriteReg functions
  * @{
  */
#define DDL_USART_CTRL1_IDLEIEN                     USART_CTRL1_IDLEIEN              /*!< IDLE interrupt enable */
#define DDL_USART_CTRL1_RXBNEIEN                     USART_CTRL1_RXBNEIEN              /*!< Read data register not empty interrupt enable */
#define DDL_USART_CTRL1_TXCIEN                       USART_CTRL1_TXCIEN                /*!< Transmission complete interrupt enable */
#define DDL_USART_CTRL1_TXBEIEN                      USART_CTRL1_TXBEIEN               /*!< Transmit data register empty interrupt enable */
#define DDL_USART_CTRL1_PEIEN                       USART_CTRL1_PEIEN                /*!< Parity error */
#define DDL_USART_CTRL2_LBDIEN                      USART_CTRL2_LBDIEN               /*!< LIN break detection interrupt enable */
#define DDL_USART_CTRL3_ERRIEN                        USART_CTRL3_ERRIEN                 /*!< Error interrupt enable */
#define DDL_USART_CTRL3_CTSIEN                      USART_CTRL3_CTSIEN               /*!< CTS interrupt enable */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_DIRECTION Communication Direction
  * @{
  */
#define DDL_USART_DIRECTION_NONE                 0x00000000U                        /*!< Transmitter and Receiver are disabled */
#define DDL_USART_DIRECTION_RX                   USART_CTRL1_RXEN                       /*!< Transmitter is disabled and Receiver is enabled */
#define DDL_USART_DIRECTION_TX                   USART_CTRL1_TXEN                       /*!< Transmitter is enabled and Receiver is disabled */
#define DDL_USART_DIRECTION_TX_RX                (USART_CTRL1_TXEN |USART_CTRL1_RXEN)       /*!< Transmitter and Receiver are enabled */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_PARITY Parity Control
  * @{
  */
#define DDL_USART_PARITY_NONE                    0x00000000U                          /*!< Parity control disabled */
#define DDL_USART_PARITY_EVEN                    USART_CTRL1_PCEN                        /*!< Parity control enabled and Even Parity is selected */
#define DDL_USART_PARITY_ODD                     (USART_CTRL1_PCEN | USART_CTRL1_PCFG)       /*!< Parity control enabled and Odd Parity is selected */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_WAKEUP Wakeup
  * @{
  */
#define DDL_USART_WAKEUP_IDLELINE                0x00000000U           /*!<  USART wake up from Mute mode on Idle Line */
#define DDL_USART_WAKEUP_ADDRESSMARK             USART_CTRL1_WUPMCFG        /*!<  USART wake up from Mute mode on Address Mark */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_DATAWIDTH Datawidth
  * @{
  */
#define DDL_USART_DATAWIDTH_8B                   0x00000000U             /*!< 8 bits word length : Start bit, 8 data bits, n stop bits */
#define DDL_USART_DATAWIDTH_9B                   USART_CTRL1_DBLCFG             /*!< 9 bits word length : Start bit, 9 data bits, n stop bits */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_OVERSAMPLING Oversampling
  * @{
  */
#define DDL_USART_OVERSAMPLING_16                0x00000000U            /*!< Oversampling by 16 */
#define DDL_USART_OVERSAMPLING_8                 USART_CTRL1_OSMCFG        /*!< Oversampling by 8 */
/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_EC_CLOCK Clock Signal
  * @{
  */

#define DDL_USART_CLOCK_DISABLE                  0x00000000U            /*!< Clock signal not provided */
#define DDL_USART_CLOCK_ENABLE                   USART_CTRL2_CLKEN        /*!< Clock signal provided */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/** @defgroup USART_DDL_EC_LASTCLKPULSE Last Clock Pulse
  * @{
  */
#define DDL_USART_LASTCLKPULSE_NO_OUTPUT         0x00000000U           /*!< The clock pulse of the last data bit is not output to the SCLK pin */
#define DDL_USART_LASTCLKPULSE_OUTPUT            USART_CTRL2_LBCPOEN        /*!< The clock pulse of the last data bit is output to the SCLK pin */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_PHASE Clock Phase
  * @{
  */
#define DDL_USART_PHASE_1EDGE                    0x00000000U           /*!< The first clock transition is the first data capture edge */
#define DDL_USART_PHASE_2EDGE                    USART_CTRL2_CPHA        /*!< The second clock transition is the first data capture edge */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_POLARITY Clock Polarity
  * @{
  */
#define DDL_USART_POLARITY_LOW                   0x00000000U           /*!< Steady low value on SCLK pin outside transmission window*/
#define DDL_USART_POLARITY_HIGH                  USART_CTRL2_CPOL        /*!< Steady high value on SCLK pin outside transmission window */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_STOPBITS Stop Bits
  * @{
  */
#define DDL_USART_STOPBITS_0_5                   USART_CTRL2_STOPCFG_0                           /*!< 0.5 stop bit */
#define DDL_USART_STOPBITS_1                     0x00000000U                                /*!< 1 stop bit */
#define DDL_USART_STOPBITS_1_5                   (USART_CTRL2_STOPCFG_0 | USART_CTRL2_STOPCFG_1)      /*!< 1.5 stop bits */
#define DDL_USART_STOPBITS_2                     USART_CTRL2_STOPCFG_1                           /*!< 2 stop bits */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_HWCONTROL Hardware Control
  * @{
  */
#define DDL_USART_HWCONTROL_NONE                 0x00000000U                          /*!< CTS and RTS hardware flow control disabled */
#define DDL_USART_HWCONTROL_RTS                  USART_CTRL3_RTSEN                       /*!< RTS output enabled, data is only requested when there is space in the receive buffer */
#define DDL_USART_HWCONTROL_CTS                  USART_CTRL3_CTSEN                       /*!< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
#define DDL_USART_HWCONTROL_RTS_CTS              (USART_CTRL3_RTSEN | USART_CTRL3_CTSEN)    /*!< CTS and RTS hardware flow control enabled */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_IRDA_POWER IrDA Power
  * @{
  */
#define DDL_USART_IRDA_POWER_NORMAL              0x00000000U           /*!< IrDA normal power mode */
#define DDL_USART_IRDA_POWER_LOW                 USART_CTRL3_IRLPEN        /*!< IrDA low power mode */
/**
  * @}
  */

/** @defgroup USART_DDL_EC_LINBREAK_DETECT LIN Break Detection Length
  * @{
  */
#define DDL_USART_LINBREAK_DETECT_10B            0x00000000U           /*!< 10-bit break detection method selected */
#define DDL_USART_LINBREAK_DETECT_11B            USART_CTRL2_LBDLCFG        /*!< 11-bit break detection method selected */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USART_DDL_Exported_Macros USART Exported Macros
  * @{
  */

/** @defgroup USART_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in USART register
  * @param  __INSTANCE__ USART Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_USART_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in USART register
  * @param  __INSTANCE__ USART Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_USART_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup USART_DDL_EM_Exported_Macros_Helper Exported_Macros_Helper
  * @{
  */

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 8 bits sampling mode (32 bits value of USARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for USART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_8 case
  */
#define __DDL_USART_DIV_SAMPLING8_100(__PERIPHCLK__, __BAUDRATE__)      ((uint32_t)((((uint64_t)(__PERIPHCLK__))*25)/(2*((uint64_t)(__BAUDRATE__)))))
#define __DDL_USART_DIVMANT_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)      (__DDL_USART_DIV_SAMPLING8_100((__PERIPHCLK__), (__BAUDRATE__))/100)
#define __DDL_USART_DIVFRAQ_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)      ((((__DDL_USART_DIV_SAMPLING8_100((__PERIPHCLK__), (__BAUDRATE__)) - (__DDL_USART_DIVMANT_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) * 100)) * 8)\
                                                                         + 50) / 100)
/* UART BRR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + ((UART DIVFRAQ & 0xF8) << 1) + (UART DIVFRAQ & 0x07) */
#define __DDL_USART_DIV_SAMPLING8(__PERIPHCLK__, __BAUDRATE__)             (((__DDL_USART_DIVMANT_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) << 4) + \
                                                                            ((__DDL_USART_DIVFRAQ_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) & 0xF8) << 1)) + \
                                                                           (__DDL_USART_DIVFRAQ_SAMPLING8((__PERIPHCLK__), (__BAUDRATE__)) & 0x07))

/**
  * @brief  Compute USARTDIV value according to Peripheral Clock and
  *         expected Baud Rate in 16 bits sampling mode (32 bits value of USARTDIV is returned)
  * @param  __PERIPHCLK__ Peripheral Clock frequency used for USART instance
  * @param  __BAUDRATE__ Baud rate value to achieve
  * @retval USARTDIV value to be used for BRR register filling in OverSampling_16 case
  */
#define __DDL_USART_DIV_SAMPLING16_100(__PERIPHCLK__, __BAUDRATE__)     ((uint32_t)((((uint64_t)(__PERIPHCLK__))*25)/(4*((uint64_t)(__BAUDRATE__)))))
#define __DDL_USART_DIVMANT_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     (__DDL_USART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__))/100)
#define __DDL_USART_DIVFRAQ_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)     ((((__DDL_USART_DIV_SAMPLING16_100((__PERIPHCLK__), (__BAUDRATE__)) - (__DDL_USART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) * 100)) * 16)\
                                                                         + 50) / 100)
/* USART BRR = mantissa + overflow + fraction
            = (USART DIVMANT << 4) + (USART DIVFRAQ & 0xF0) + (USART DIVFRAQ & 0x0F) */
#define __DDL_USART_DIV_SAMPLING16(__PERIPHCLK__, __BAUDRATE__)            (((__DDL_USART_DIVMANT_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) << 4) + \
                                                                            (__DDL_USART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0xF0)) + \
                                                                           (__DDL_USART_DIVFRAQ_SAMPLING16((__PERIPHCLK__), (__BAUDRATE__)) & 0x0F))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup USART_DDL_Exported_Functions USART Exported Functions
  * @{
  */

/** @defgroup USART_DDL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  USART Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_Enable(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL1, USART_CTRL1_UEN);
}

/**
  * @brief  USART Disable (all USART prescalers and outputs are disabled)
  * @note   When USART is disabled, USART prescalers and outputs are stopped immediately,
  *         and current operations are discarded. The configuration of the USART is kept, but all the status
  *         flags, in the USARTx_SR are set to their default values.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_Disable(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_UEN);
}

/**
  * @brief  Indicate if USART is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabled(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_UEN) == (USART_CTRL1_UEN));
}

/**
  * @brief  Receiver Enable (Receiver is enabled and begins searching for a start bit)
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDirectionRx(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_RXEN);
}

/**
  * @brief  Receiver Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDirectionRx(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_RXEN);
}

/**
  * @brief  Transmitter Enable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDirectionTx(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_TXEN);
}

/**
  * @brief  Transmitter Disable
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDirectionTx(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_TXEN);
}

/**
  * @brief  Configure simultaneously enabled/disabled states
  *         of Transmitter and Receiver
  * @param  USARTx USART Instance
  * @param  TransferDirection This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DIRECTION_NONE
  *         @arg @ref DDL_USART_DIRECTION_RX
  *         @arg @ref DDL_USART_DIRECTION_TX
  *         @arg @ref DDL_USART_DIRECTION_TX_RX
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetTransferDirection(USART_TypeDef *USARTx, uint32_t TransferDirection)
{
  ATOMIC_MODIFY_REG(USARTx->CTRL1, USART_CTRL1_RXEN | USART_CTRL1_TXEN, TransferDirection);
}

/**
  * @brief  Return enabled/disabled states of Transmitter and Receiver
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_DIRECTION_NONE
  *         @arg @ref DDL_USART_DIRECTION_RX
  *         @arg @ref DDL_USART_DIRECTION_TX
  *         @arg @ref DDL_USART_DIRECTION_TX_RX
  */
__STATIC_INLINE uint32_t DDL_USART_GetTransferDirection(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL1, USART_CTRL1_RXEN | USART_CTRL1_TXEN));
}

/**
  * @brief  Configure Parity (enabled/disabled and parity mode if enabled).
  * @note   This function selects if hardware parity control (generation and detection) is enabled or disabled.
  *         When the parity control is enabled (Odd or Even), computed parity bit is inserted at the MSB position
  *         (9th or 8th bit depending on data width) and parity is checked on the received data.
  * @param  USARTx USART Instance
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetParity(USART_TypeDef *USARTx, uint32_t Parity)
{
  MODIFY_REG(USARTx->CTRL1, USART_CTRL1_PCFG | USART_CTRL1_PCEN, Parity);
}

/**
  * @brief  Return Parity configuration (enabled/disabled and parity mode if enabled)
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  */
__STATIC_INLINE uint32_t DDL_USART_GetParity(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL1, USART_CTRL1_PCFG | USART_CTRL1_PCEN));
}

/**
  * @brief  Set Receiver Wake Up method from Mute mode.
  * @param  USARTx USART Instance
  * @param  Method This parameter can be one of the following values:
  *         @arg @ref DDL_USART_WAKEUP_IDLELINE
  *         @arg @ref DDL_USART_WAKEUP_ADDRESSMARK
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetWakeUpMethod(USART_TypeDef *USARTx, uint32_t Method)
{
  MODIFY_REG(USARTx->CTRL1, USART_CTRL1_WUPMCFG, Method);
}

/**
  * @brief  Return Receiver Wake Up method from Mute mode
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_WAKEUP_IDLELINE
  *         @arg @ref DDL_USART_WAKEUP_ADDRESSMARK
  */
__STATIC_INLINE uint32_t DDL_USART_GetWakeUpMethod(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL1, USART_CTRL1_WUPMCFG));
}

/**
  * @brief  Set Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  USARTx USART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetDataWidth(USART_TypeDef *USARTx, uint32_t DataWidth)
{
  MODIFY_REG(USARTx->CTRL1, USART_CTRL1_DBLCFG, DataWidth);
}

/**
  * @brief  Return Word length (i.e. nb of data bits, excluding start and stop bits)
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  */
__STATIC_INLINE uint32_t DDL_USART_GetDataWidth(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL1, USART_CTRL1_DBLCFG));
}

/**
  * @brief  Set Oversampling to 8-bit or 16-bit mode
  * @param  USARTx USART Instance
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetOverSampling(USART_TypeDef *USARTx, uint32_t OverSampling)
{
  MODIFY_REG(USARTx->CTRL1, USART_CTRL1_OSMCFG, OverSampling);
}

/**
  * @brief  Return Oversampling mode
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  */
__STATIC_INLINE uint32_t DDL_USART_GetOverSampling(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL1, USART_CTRL1_OSMCFG));
}

/**
  * @brief  Configure if Clock pulse of the last data bit is output to the SCLK pin or not
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  LastBitClockPulse This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetLastClkPulseOutput(USART_TypeDef *USARTx, uint32_t LastBitClockPulse)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_LBCPOEN, LastBitClockPulse);
}

/**
  * @brief  Retrieve Clock pulse of the last data bit output configuration
  *         (Last bit Clock pulse output to the SCLK pin or not)
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  */
__STATIC_INLINE uint32_t DDL_USART_GetLastClkPulseOutput(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_LBCPOEN));
}

/**
  * @brief  Select the phase of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  ClockPhase This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetClockPhase(USART_TypeDef *USARTx, uint32_t ClockPhase)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_CPHA, ClockPhase);
}

/**
  * @brief  Return phase of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  */
__STATIC_INLINE uint32_t DDL_USART_GetClockPhase(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_CPHA));
}

/**
  * @brief  Select the polarity of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetClockPolarity(USART_TypeDef *USARTx, uint32_t ClockPolarity)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_CPOL, ClockPolarity);
}

/**
  * @brief  Return polarity of the clock output on the SCLK pin in synchronous mode
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  */
__STATIC_INLINE uint32_t DDL_USART_GetClockPolarity(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_CPOL));
}

/**
  * @brief  Configure Clock signal format (Phase Polarity and choice about output of last bit clock pulse)
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clock Phase configuration using @ref DDL_USART_SetClockPhase() function
  *         - Clock Polarity configuration using @ref DDL_USART_SetClockPolarity() function
  *         - Output of Last bit Clock pulse configuration using @ref DDL_USART_SetLastClkPulseOutput() function
  * @param  USARTx USART Instance
  * @param  Phase This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PHASE_1EDGE
  *         @arg @ref DDL_USART_PHASE_2EDGE
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_POLARITY_LOW
  *         @arg @ref DDL_USART_POLARITY_HIGH
  * @param  LBCPOutput This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LASTCLKPULSE_NO_OUTPUT
  *         @arg @ref DDL_USART_LASTCLKPULSE_OUTPUT
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigClock(USART_TypeDef *USARTx, uint32_t Phase, uint32_t Polarity, uint32_t LBCPOutput)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_CPHA | USART_CTRL2_CPOL | USART_CTRL2_LBCPOEN, Phase | Polarity | LBCPOutput);
}

/**
  * @brief  Enable Clock output on SCLK pin
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableSCLKOutput(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL2, USART_CTRL2_CLKEN);
}

/**
  * @brief  Disable Clock output on SCLK pin
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableSCLKOutput(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL2, USART_CTRL2_CLKEN);
}

/**
  * @brief  Indicate if Clock output on SCLK pin is enabled
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledSCLKOutput(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL2, USART_CTRL2_CLKEN) == (USART_CTRL2_CLKEN));
}

/**
  * @brief  Set the length of the stop bits
  * @param  USARTx USART Instance
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_0_5
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_1_5
  *         @arg @ref DDL_USART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetStopBitsLength(USART_TypeDef *USARTx, uint32_t StopBits)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_STOPCFG, StopBits);
}

/**
  * @brief  Retrieve the length of the stop bits
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_0_5
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_1_5
  *         @arg @ref DDL_USART_STOPBITS_2
  */
__STATIC_INLINE uint32_t DDL_USART_GetStopBitsLength(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_STOPCFG));
}

/**
  * @brief  Configure Character frame format (Datawidth, Parity control, Stop Bits)
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Data Width configuration using @ref DDL_USART_SetDataWidth() function
  *         - Parity Control and mode configuration using @ref DDL_USART_SetParity() function
  *         - Stop bits configuration using @ref DDL_USART_SetStopBitsLength() function
  * @param  USARTx USART Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref DDL_USART_DATAWIDTH_8B
  *         @arg @ref DDL_USART_DATAWIDTH_9B
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref DDL_USART_PARITY_NONE
  *         @arg @ref DDL_USART_PARITY_EVEN
  *         @arg @ref DDL_USART_PARITY_ODD
  * @param  StopBits This parameter can be one of the following values:
  *         @arg @ref DDL_USART_STOPBITS_0_5
  *         @arg @ref DDL_USART_STOPBITS_1
  *         @arg @ref DDL_USART_STOPBITS_1_5
  *         @arg @ref DDL_USART_STOPBITS_2
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigCharacter(USART_TypeDef *USARTx, uint32_t DataWidth, uint32_t Parity,
                                              uint32_t StopBits)
{
  MODIFY_REG(USARTx->CTRL1, USART_CTRL1_PCFG | USART_CTRL1_PCEN | USART_CTRL1_DBLCFG, Parity | DataWidth);
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_STOPCFG, StopBits);
}

/**
  * @brief  Set Address of the USART node.
  * @note   This is used in multiprocessor communication during Mute mode or Stop mode,
  *         for wake up with address mark detection.
  * @param  USARTx USART Instance
  * @param  NodeAddress 4 bit Address of the USART node.
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetNodeAddress(USART_TypeDef *USARTx, uint32_t NodeAddress)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_ADDR, (NodeAddress & USART_CTRL2_ADDR));
}

/**
  * @brief  Return 4 bit Address of the USART node as set in ADD field of CTRL2.
  * @note   only 4bits (b3-b0) of returned value are relevant (b31-b4 are not relevant)
  * @param  USARTx USART Instance
  * @retval Address of the USART node (Value between Min_Data=0 and Max_Data=255)
  */
__STATIC_INLINE uint32_t DDL_USART_GetNodeAddress(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_ADDR));
}

/**
  * @brief  Enable RTS HW Flow Control
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_RTSEN);
}

/**
  * @brief  Disable RTS HW Flow Control
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableRTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_RTSEN);
}

/**
  * @brief  Enable CTS HW Flow Control
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_CTSEN);
}

/**
  * @brief  Disable CTS HW Flow Control
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableCTSHWFlowCtrl(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_CTSEN);
}

/**
  * @brief  Configure HW Flow Control mode (both CTS and RTS)
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  HardwareFlowControl This parameter can be one of the following values:
  *         @arg @ref DDL_USART_HWCONTROL_NONE
  *         @arg @ref DDL_USART_HWCONTROL_RTS
  *         @arg @ref DDL_USART_HWCONTROL_CTS
  *         @arg @ref DDL_USART_HWCONTROL_RTS_CTS
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetHWFlowCtrl(USART_TypeDef *USARTx, uint32_t HardwareFlowControl)
{
  MODIFY_REG(USARTx->CTRL3, USART_CTRL3_RTSEN | USART_CTRL3_CTSEN, HardwareFlowControl);
}

/**
  * @brief  Return HW Flow Control configuration (both CTS and RTS)
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_HWCONTROL_NONE
  *         @arg @ref DDL_USART_HWCONTROL_RTS
  *         @arg @ref DDL_USART_HWCONTROL_CTS
  *         @arg @ref DDL_USART_HWCONTROL_RTS_CTS
  */
__STATIC_INLINE uint32_t DDL_USART_GetHWFlowCtrl(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL3, USART_CTRL3_RTSEN | USART_CTRL3_CTSEN));
}

/**
  * @brief  Enable One bit sampling method
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableOneBitSamp(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_SAMCFG);
}

/**
  * @brief  Disable One bit sampling method
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableOneBitSamp(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_SAMCFG);
}

/**
  * @brief  Indicate if One bit sampling method is enabled
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledOneBitSamp(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_SAMCFG) == (USART_CTRL3_SAMCFG));
}

/**
  * @brief  Configure USART BRR register for achieving expected Baud Rate value.
  * @note   Compute and set USARTDIV value in BRR Register (full BRR content)
  *         according to used Peripheral Clock, Oversampling mode, and expected Baud Rate values
  * @note   Peripheral clock and Baud rate values provided as function parameters should be valid
  *         (Baud rate value != 0)
  * @param  USARTx USART Instance
  * @param  PeriphClk Peripheral Clock
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @param  BaudRate Baud Rate
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling,
                                          uint32_t BaudRate)
{
  if (OverSampling == DDL_USART_OVERSAMPLING_8)
  {
    USARTx->BR = (uint16_t)(__DDL_USART_DIV_SAMPLING8(PeriphClk, BaudRate));
  }
  else
  {
    USARTx->BR = (uint16_t)(__DDL_USART_DIV_SAMPLING16(PeriphClk, BaudRate));
  }
}

/**
  * @brief  Return current Baud Rate value, according to USARTDIV present in BRR register
  *         (full BRR content), and to used Peripheral Clock and Oversampling mode values
  * @note   In case of non-initialized or invalid value stored in BRR register, value 0 will be returned.
  * @param  USARTx USART Instance
  * @param  PeriphClk Peripheral Clock
  * @param  OverSampling This parameter can be one of the following values:
  *         @arg @ref DDL_USART_OVERSAMPLING_16
  *         @arg @ref DDL_USART_OVERSAMPLING_8
  * @retval Baud Rate
  */
__STATIC_INLINE uint32_t DDL_USART_GetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling)
{
  uint32_t usartdiv = 0x0U;
  uint32_t brrresult = 0x0U;

  usartdiv = USARTx->BR;

  if (OverSampling == DDL_USART_OVERSAMPLING_8)
  {
    if ((usartdiv & 0xFFF7U) != 0U)
    {
      usartdiv = (uint16_t)((usartdiv & 0xFFF0U) | ((usartdiv & 0x0007U) << 1U)) ;
      brrresult = (PeriphClk * 2U) / usartdiv;
    }
  }
  else
  {
    if ((usartdiv & 0xFFFFU) != 0U)
    {
      brrresult = PeriphClk / usartdiv;
    }
  }
  return (brrresult);
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_IRDA Configuration functions related to Irda feature
  * @{
  */

/**
  * @brief  Enable IrDA mode
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIrda(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_IREN);
}

/**
  * @brief  Disable IrDA mode
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIrda(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_IREN);
}

/**
  * @brief  Indicate if IrDA mode is enabled
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIrda(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_IREN) == (USART_CTRL3_IREN));
}

/**
  * @brief  Configure IrDA Power Mode (Normal or Low Power)
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  PowerMode This parameter can be one of the following values:
  *         @arg @ref DDL_USART_IRDA_POWER_NORMAL
  *         @arg @ref DDL_USART_IRDA_POWER_LOW
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetIrdaPowerMode(USART_TypeDef *USARTx, uint32_t PowerMode)
{
  MODIFY_REG(USARTx->CTRL3, USART_CTRL3_IRLPEN, PowerMode);
}

/**
  * @brief  Retrieve IrDA Power Mode configuration (Normal or Low Power)
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_IRDA_POWER_NORMAL
  *         @arg @ref DDL_USART_PHASE_2EDGE
  */
__STATIC_INLINE uint32_t DDL_USART_GetIrdaPowerMode(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL3, USART_CTRL3_IRLPEN));
}

/**
  * @brief  Set Irda prescaler value, used for dividing the USART clock source
  *         to achieve the Irda Low Power frequency (8 bits value)
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  PrescalerValue Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetIrdaPrescaler(USART_TypeDef *USARTx, uint32_t PrescalerValue)
{
  MODIFY_REG(USARTx->GTPSC, USART_GTPSC_PSC, PrescalerValue);
}

/**
  * @brief  Return Irda prescaler value, used for dividing the USART clock source
  *         to achieve the Irda Low Power frequency (8 bits value)
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Irda prescaler value (Value between Min_Data=0x00 and Max_Data=0xFF)
  */
__STATIC_INLINE uint32_t DDL_USART_GetIrdaPrescaler(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->GTPSC, USART_GTPSC_PSC));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_Smartcard Configuration functions related to Smartcard feature
  * @{
  */

/**
  * @brief  Enable Smartcard NACK transmission
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableSmartcardNACK(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_SCNACKEN);
}

/**
  * @brief  Disable Smartcard NACK transmission
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableSmartcardNACK(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_SCNACKEN);
}

/**
  * @brief  Indicate if Smartcard NACK transmission is enabled
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledSmartcardNACK(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_SCNACKEN) == (USART_CTRL3_SCNACKEN));
}

/**
  * @brief  Enable Smartcard mode
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableSmartcard(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_SCEN);
}

/**
  * @brief  Disable Smartcard mode
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableSmartcard(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_SCEN);
}

/**
  * @brief  Indicate if Smartcard mode is enabled
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledSmartcard(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_SCEN) == (USART_CTRL3_SCEN));
}

/**
  * @brief  Set Smartcard prescaler value, used for dividing the USART clock
  *         source to provide the SMARTCARD Clock (5 bits value)
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  PrescalerValue Value between Min_Data=0 and Max_Data=31
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetSmartcardPrescaler(USART_TypeDef *USARTx, uint32_t PrescalerValue)
{
  MODIFY_REG(USARTx->GTPSC, USART_GTPSC_PSC, PrescalerValue);
}

/**
  * @brief  Return Smartcard prescaler value, used for dividing the USART clock
  *         source to provide the SMARTCARD Clock (5 bits value)
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Smartcard prescaler value (Value between Min_Data=0 and Max_Data=31)
  */
__STATIC_INLINE uint32_t DDL_USART_GetSmartcardPrescaler(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->GTPSC, USART_GTPSC_PSC));
}

/**
  * @brief  Set Smartcard Guard time value, expressed in nb of baud clocks periods
  *         (GT[7:0] bits : Guard time value)
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  GuardTime Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetSmartcardGuardTime(USART_TypeDef *USARTx, uint32_t GuardTime)
{
  MODIFY_REG(USARTx->GTPSC, USART_GTPSC_GRDT, GuardTime << USART_POSITION_GTPR_GT);
}

/**
  * @brief  Return Smartcard Guard time value, expressed in nb of baud clocks periods
  *         (GT[7:0] bits : Guard time value)
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Smartcard Guard time value (Value between Min_Data=0x00 and Max_Data=0xFF)
  */
__STATIC_INLINE uint32_t DDL_USART_GetSmartcardGuardTime(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->GTPSC, USART_GTPSC_GRDT) >> USART_POSITION_GTPR_GT);
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_HalfDuplex Configuration functions related to Half Duplex feature
  * @{
  */

/**
  * @brief  Enable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableHalfDuplex(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL3, USART_CTRL3_HDEN);
}

/**
  * @brief  Disable Single Wire Half-Duplex mode
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableHalfDuplex(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_HDEN);
}

/**
  * @brief  Indicate if Single Wire Half-Duplex mode is enabled
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledHalfDuplex(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_HDEN) == (USART_CTRL3_HDEN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Configuration_LIN Configuration functions related to LIN feature
  * @{
  */

/**
  * @brief  Set LIN Break Detection Length
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @param  LINBDLength This parameter can be one of the following values:
  *         @arg @ref DDL_USART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_USART_LINBREAK_DETECT_11B
  * @retval None
  */
__STATIC_INLINE void DDL_USART_SetLINBrkDetectionLen(USART_TypeDef *USARTx, uint32_t LINBDLength)
{
  MODIFY_REG(USARTx->CTRL2, USART_CTRL2_LBDLCFG, LINBDLength);
}

/**
  * @brief  Return LIN Break Detection Length
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_USART_LINBREAK_DETECT_10B
  *         @arg @ref DDL_USART_LINBREAK_DETECT_11B
  */
__STATIC_INLINE uint32_t DDL_USART_GetLINBrkDetectionLen(USART_TypeDef *USARTx)
{
  return (uint32_t)(READ_BIT(USARTx->CTRL2, USART_CTRL2_LBDLCFG));
}

/**
  * @brief  Enable LIN mode
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableLIN(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL2, USART_CTRL2_LINMEN);
}

/**
  * @brief  Disable LIN mode
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableLIN(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL2, USART_CTRL2_LINMEN);
}

/**
  * @brief  Indicate if LIN mode is enabled
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledLIN(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL2, USART_CTRL2_LINMEN) == (USART_CTRL2_LINMEN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_AdvancedConfiguration Advanced Configurations services
  * @{
  */

/**
  * @brief  Perform basic configuration of USART for enabling use in Asynchronous Mode (UART)
  * @note   In UART mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - IREN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Asynchronous Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigAsyncMode(USART_TypeDef *USARTx)
{
  /* In Asynchronous mode, the following bits must be kept cleared:
  - LINEN, CLKEN bits in the USART_CTRL2 register,
  - SCEN, IREN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_IREN | USART_CTRL3_HDEN));
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Synchronous Mode
  * @note   In Synchronous mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - IREN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also sets the USART in Synchronous mode.
  * @note   Macro @ref IS_USART_INSTANCE(USARTx) can be used to check whether or not
  *         Synchronous mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Set CLKEN in CTRL2 using @ref DDL_USART_EnableSCLKOutput() function
  * @note   Other remaining configurations items related to Synchronous Mode
  *         (as Baud Rate, Word length, Parity, Clock Polarity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigSyncMode(USART_TypeDef *USARTx)
{
  /* In Synchronous mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CTRL2 register,
  - SCEN, IREN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_IREN | USART_CTRL3_HDEN));
  /* set the UART/USART in Synchronous mode */
  SET_BIT(USARTx->CTRL2, USART_CTRL2_CLKEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in LIN Mode
  * @note   In LIN mode, the following bits must be kept cleared:
  *           - STOP and CLKEN bits in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - IREN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also set the UART/USART in LIN mode.
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear STOP in CTRL2 using @ref DDL_USART_SetStopBitsLength() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Set LINEN in CTRL2 using @ref DDL_USART_EnableLIN() function
  * @note   Other remaining configurations items related to LIN Mode
  *         (as Baud Rate, Word length, LIN Break Detection Length, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigLINMode(USART_TypeDef *USARTx)
{
  /* In LIN mode, the following bits must be kept cleared:
  - STOP and CLKEN bits in the USART_CTRL2 register,
  - IREN, SCEN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_CLKEN | USART_CTRL2_STOPCFG));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_IREN | USART_CTRL3_SCEN | USART_CTRL3_HDEN));
  /* Set the UART/USART in LIN mode */
  SET_BIT(USARTx->CTRL2, USART_CTRL2_LINMEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Half Duplex Mode
  * @note   In Half Duplex mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - IREN bit in the USART_CTRL3 register,
  *         This function also sets the UART/USART in Half Duplex mode.
  * @note   Macro @ref IS_UART_HALFDUPLEX_INSTANCE(USARTx) can be used to check whether or not
  *         Half-Duplex mode is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Set HDSEL in CTRL3 using @ref DDL_USART_EnableHalfDuplex() function
  * @note   Other remaining configurations items related to Half Duplex Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigHalfDuplexMode(USART_TypeDef *USARTx)
{
  /* In Half Duplex mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CTRL2 register,
  - SCEN and IREN bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_IREN));
  /* set the UART/USART in Half Duplex mode */
  SET_BIT(USARTx->CTRL3, USART_CTRL3_HDEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Smartcard Mode
  * @note   In Smartcard mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - IREN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also configures Stop bits to 1.5 bits and
  *         sets the USART in Smartcard mode (SCEN bit).
  *         Clock Output is also enabled (CLKEN).
  * @note   Macro @ref IS_SMARTCARD_INSTANCE(USARTx) can be used to check whether or not
  *         Smartcard feature is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Configure STOP in CTRL2 using @ref DDL_USART_SetStopBitsLength() function
  *         - Set CLKEN in CTRL2 using @ref DDL_USART_EnableSCLKOutput() function
  *         - Set SCEN in CTRL3 using @ref DDL_USART_EnableSmartcard() function
  * @note   Other remaining configurations items related to Smartcard Mode
  *         (as Baud Rate, Word length, Parity, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigSmartcardMode(USART_TypeDef *USARTx)
{
  /* In Smartcard mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CTRL2 register,
  - IREN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_IREN | USART_CTRL3_HDEN));
  /* Configure Stop bits to 1.5 bits */
  /* Synchronous mode is activated by default */
  SET_BIT(USARTx->CTRL2, (USART_CTRL2_STOPCFG_0 | USART_CTRL2_STOPCFG_1 | USART_CTRL2_CLKEN));
  /* set the UART/USART in Smartcard mode */
  SET_BIT(USARTx->CTRL3, USART_CTRL3_SCEN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Irda Mode
  * @note   In IRDA mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - STOP and CLKEN bits in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  *         This function also sets the UART/USART in IRDA mode (IREN bit).
  * @note   Macro @ref IS_IRDA_INSTANCE(USARTx) can be used to check whether or not
  *         IrDA feature is supported by the USARTx instance.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  *         - Configure STOP in CTRL2 using @ref DDL_USART_SetStopBitsLength() function
  *         - Set IREN in CTRL3 using @ref DDL_USART_EnableIrda() function
  * @note   Other remaining configurations items related to Irda Mode
  *         (as Baud Rate, Word length, Power mode, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigIrdaMode(USART_TypeDef *USARTx)
{
  /* In IRDA mode, the following bits must be kept cleared:
  - LINEN, STOP and CLKEN bits in the USART_CTRL2 register,
  - SCEN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN | USART_CTRL2_STOPCFG));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN));
  /* set the UART/USART in IRDA mode */
  SET_BIT(USARTx->CTRL3, USART_CTRL3_IREN);
}

/**
  * @brief  Perform basic configuration of USART for enabling use in Multi processor Mode
  *         (several USARTs connected in a network, one of the USARTs can be the master,
  *         its TX output connected to the RX inputs of the other slaves USARTs).
  * @note   In MultiProcessor mode, the following bits must be kept cleared:
  *           - LINEN bit in the USART_CTRL2 register,
  *           - CLKEN bit in the USART_CTRL2 register,
  *           - SCEN bit in the USART_CTRL3 register,
  *           - IREN bit in the USART_CTRL3 register,
  *           - HDSEL bit in the USART_CTRL3 register.
  * @note   Call of this function is equivalent to following function call sequence :
  *         - Clear LINEN in CTRL2 using @ref DDL_USART_DisableLIN() function
  *         - Clear CLKEN in CTRL2 using @ref DDL_USART_DisableSCLKOutput() function
  *         - Clear SCEN in CTRL3 using @ref DDL_USART_DisableSmartcard() function
  *         - Clear IREN in CTRL3 using @ref DDL_USART_DisableIrda() function
  *         - Clear HDSEL in CTRL3 using @ref DDL_USART_DisableHalfDuplex() function
  * @note   Other remaining configurations items related to Multi processor Mode
  *         (as Baud Rate, Wake Up Method, Node address, ...) should be set using
  *         dedicated functions
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ConfigMultiProcessMode(USART_TypeDef *USARTx)
{
  /* In Multi Processor mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CTRL2 register,
  - IREN, SCEN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(USARTx->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(USARTx->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN | USART_CTRL3_IREN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if the USART Parity Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_PE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_PEFLG) == (USART_STS_PEFLG));
}

/**
  * @brief  Check if the USART Framing Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_FE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_FEFLG) == (USART_STS_FEFLG));
}

/**
  * @brief  Check if the USART Noise error detected Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_NE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_NEFLG) == (USART_STS_NEFLG));
}

/**
  * @brief  Check if the USART OverRun Error Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_ORE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_OVREFLG) == (USART_STS_OVREFLG));
}

/**
  * @brief  Check if the USART IDLE line detected Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_IDLE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_IDLEFLG) == (USART_STS_IDLEFLG));
}

/**
  * @brief  Check if the USART Read Data Register Not Empty Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_RXNE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_RXBNEFLG) == (USART_STS_RXBNEFLG));
}

/**
  * @brief  Check if the USART Transmission Complete Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_TC(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_TXCFLG) == (USART_STS_TXCFLG));
}

/**
  * @brief  Check if the USART Transmit Data Register Empty Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_TXBEFLG) == (USART_STS_TXBEFLG));
}

/**
  * @brief  Check if the USART LIN Break Detection Flag is set or not
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_LBD(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_LBDFLG) == (USART_STS_LBDFLG));
}

/**
  * @brief  Check if the USART CTS Flag is set or not
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_nCTS(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->STS, USART_STS_CTSFLG) == (USART_STS_CTSFLG));
}

/**
  * @brief  Check if the USART Send Break Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_SBK(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_TXBF) == (USART_CTRL1_TXBF));
}

/**
  * @brief  Check if the USART Receive Wake Up from mute mode Flag is set or not
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsActiveFlag_RWU(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_RXMUTEEN) == (USART_CTRL1_RXMUTEEN));
}

/**
  * @brief  Clear Parity Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         NE, FE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_PE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->STS;
  (void) tmpreg;
  tmpreg = USARTx->DATA;
  (void) tmpreg;
}

/**
  * @brief  Clear Framing Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_FE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->STS;
  (void) tmpreg;
  tmpreg = USARTx->DATA;
  (void) tmpreg;
}

/**
  * @brief  Clear Noise detected Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, FE, ORE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_NE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->STS;
  (void) tmpreg;
  tmpreg = USARTx->DATA;
  (void) tmpreg;
}

/**
  * @brief  Clear OverRun Error Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, IDLE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_ORE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->STS;
  (void) tmpreg;
  tmpreg = USARTx->DATA;
  (void) tmpreg;
}

/**
  * @brief  Clear IDLE line detected Flag
  * @note   Clearing this flag is done by a read access to the USARTx_SR
  *         register followed by a read access to the USARTx_DR register.
  * @note   Please also consider that when clearing this flag, other flags as
  *         PE, NE, FE, ORE would also be cleared.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_IDLE(USART_TypeDef *USARTx)
{
  __IO uint32_t tmpreg;
  tmpreg = USARTx->STS;
  (void) tmpreg;
  tmpreg = USARTx->DATA;
  (void) tmpreg;
}

/**
  * @brief  Clear Transmission Complete Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_TC(USART_TypeDef *USARTx)
{
  WRITE_REG(USARTx->STS, ~(USART_STS_TXCFLG));
}

/**
  * @brief  Clear RX Not Empty Flag
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_RXNE(USART_TypeDef *USARTx)
{
  WRITE_REG(USARTx->STS, ~(USART_STS_RXBNEFLG));
}

/**
  * @brief  Clear LIN Break Detection Flag
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_LBD(USART_TypeDef *USARTx)
{
  WRITE_REG(USARTx->STS, ~(USART_STS_LBDFLG));
}

/**
  * @brief  Clear CTS Interrupt Flag
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_ClearFlag_nCTS(USART_TypeDef *USARTx)
{
  WRITE_REG(USARTx->STS, ~(USART_STS_CTSFLG));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable IDLE Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_IDLE(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_IDLEIEN);
}

/**
  * @brief  Enable RX Not Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_RXNE(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_RXBNEIEN);
}

/**
  * @brief  Enable Transmission Complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_TC(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_TXCIEN);
}

/**
  * @brief  Enable TX Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_TXE(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_TXBEIEN);
}

/**
  * @brief  Enable Parity Error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_PE(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL1, USART_CTRL1_PEIEN);
}

/**
  * @brief  Enable LIN Break Detection Interrupt
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_LBD(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL2, USART_CTRL2_LBDIEN);
}

/**
  * @brief  Enable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the USARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the USARTx_SR register.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_ERROR(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL3, USART_CTRL3_ERRIEN);
}

/**
  * @brief  Enable CTS Interrupt
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableIT_CTS(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL3, USART_CTRL3_CTSIEN);
}

/**
  * @brief  Disable IDLE Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_IDLE(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_IDLEIEN);
}

/**
  * @brief  Disable RX Not Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_RXNE(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_RXBNEIEN);
}

/**
  * @brief  Disable Transmission Complete Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_TC(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_TXCIEN);
}

/**
  * @brief  Disable TX Empty Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_TXE(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_TXBEIEN);
}

/**
  * @brief  Disable Parity Error Interrupt
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_PE(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_PEIEN);
}

/**
  * @brief  Disable LIN Break Detection Interrupt
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_LBD(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL2, USART_CTRL2_LBDIEN);
}

/**
  * @brief  Disable Error Interrupt
  * @note   When set, Error Interrupt Enable Bit is enabling interrupt generation in case of a framing
  *         error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the USARTx_SR register).
  *           0: Interrupt is inhibited
  *           1: An interrupt is generated when FE=1 or ORE=1 or NF=1 in the USARTx_SR register.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_ERROR(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_ERRIEN);
}

/**
  * @brief  Disable CTS Interrupt
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableIT_CTS(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_CTSIEN);
}

/**
  * @brief  Check if the USART IDLE Interrupt  source is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_IDLE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_IDLEIEN) == (USART_CTRL1_IDLEIEN));
}

/**
  * @brief  Check if the USART RX Not Empty Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_RXNE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_RXBNEIEN) == (USART_CTRL1_RXBNEIEN));
}

/**
  * @brief  Check if the USART Transmission Complete Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_TC(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_TXCIEN) == (USART_CTRL1_TXCIEN));
}

/**
  * @brief  Check if the USART TX Empty Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_TXE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_TXBEIEN) == (USART_CTRL1_TXBEIEN));
}

/**
  * @brief  Check if the USART Parity Error Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_PE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL1, USART_CTRL1_PEIEN) == (USART_CTRL1_PEIEN));
}

/**
  * @brief  Check if the USART LIN Break Detection Interrupt is enabled or disabled.
  * @note   Macro @ref IS_UART_LIN_INSTANCE(USARTx) can be used to check whether or not
  *         LIN feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_LBD(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL2, USART_CTRL2_LBDIEN) == (USART_CTRL2_LBDIEN));
}

/**
  * @brief  Check if the USART Error Interrupt is enabled or disabled.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_ERROR(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_ERRIEN) == (USART_CTRL3_ERRIEN));
}

/**
  * @brief  Check if the USART CTS Interrupt is enabled or disabled.
  * @note   Macro @ref IS_UART_HWFLOW_INSTANCE(USARTx) can be used to check whether or not
  *         Hardware Flow control feature is supported by the USARTx instance.
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledIT_CTS(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_CTSIEN) == (USART_CTRL3_CTSIEN));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Enable DMA Mode for reception
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDMAReq_RX(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL3, USART_CTRL3_DMARXEN);
}

/**
  * @brief  Disable DMA Mode for reception
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDMAReq_RX(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_DMARXEN);
}

/**
  * @brief  Check if DMA Mode is enabled for reception
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledDMAReq_RX(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_DMARXEN) == (USART_CTRL3_DMARXEN));
}

/**
  * @brief  Enable DMA Mode for transmission
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_EnableDMAReq_TX(USART_TypeDef *USARTx)
{
  ATOMIC_SET_BIT(USARTx->CTRL3, USART_CTRL3_DMATXEN);
}

/**
  * @brief  Disable DMA Mode for transmission
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_DisableDMAReq_TX(USART_TypeDef *USARTx)
{
  ATOMIC_CLEAR_BIT(USARTx->CTRL3, USART_CTRL3_DMATXEN);
}

/**
  * @brief  Check if DMA Mode is enabled for transmission
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_USART_IsEnabledDMAReq_TX(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->CTRL3, USART_CTRL3_DMATXEN) == (USART_CTRL3_DMATXEN));
}

/**
  * @brief  Get the data register address used for DMA transfer
  * @note   Address of Data Register is valid for both Transmit and Receive transfers.
  * @param  USARTx USART Instance
  * @retval Address of data register
  */
__STATIC_INLINE uint32_t DDL_USART_DMA_GetRegAddr(USART_TypeDef *USARTx)
{
  /* return address of DR register */
  return ((uint32_t) &(USARTx->DATA));
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Read Receiver Data register (Receive Data value, 8 bits)
  * @param  USARTx USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t DDL_USART_ReceiveData8(USART_TypeDef *USARTx)
{
  return (uint8_t)(READ_BIT(USARTx->DATA, USART_DATA_DATA));
}

/**
  * @brief  Read Receiver Data register (Receive Data value, 9 bits)
  * @param  USARTx USART Instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x1FF
  */
__STATIC_INLINE uint16_t DDL_USART_ReceiveData9(USART_TypeDef *USARTx)
{
  return (uint16_t)(READ_BIT(USARTx->DATA, USART_DATA_DATA));
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 8 bits)
  * @param  USARTx USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value)
{
  USARTx->DATA = Value;
}

/**
  * @brief  Write in Transmitter Data Register (Transmit Data value, 9 bits)
  * @param  USARTx USART Instance
  * @param  Value between Min_Data=0x00 and Max_Data=0x1FF
  * @retval None
  */
__STATIC_INLINE void DDL_USART_TransmitData9(USART_TypeDef *USARTx, uint16_t Value)
{
  USARTx->DATA = Value & 0x1FFU;
}

/**
  * @}
  */

/** @defgroup USART_DDL_EF_Execution Execution
  * @{
  */

/**
  * @brief  Request Break sending
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestBreakSending(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL1, USART_CTRL1_TXBF);
}

/**
  * @brief  Put USART in Mute mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestEnterMuteMode(USART_TypeDef *USARTx)
{
  SET_BIT(USARTx->CTRL1, USART_CTRL1_RXMUTEEN);
}

/**
  * @brief  Put USART in Active mode
  * @param  USARTx USART Instance
  * @retval None
  */
__STATIC_INLINE void DDL_USART_RequestExitMuteMode(USART_TypeDef *USARTx)
{
  CLEAR_BIT(USARTx->CTRL1, USART_CTRL1_RXMUTEEN);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup USART_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */
ErrorStatus DDL_USART_DeInit(USART_TypeDef *USARTx);
ErrorStatus DDL_USART_Init(USART_TypeDef *USARTx, DDL_USART_InitTypeDef *USART_InitStruct);
void        DDL_USART_StructInit(DDL_USART_InitTypeDef *USART_InitStruct);
ErrorStatus DDL_USART_ClockInit(USART_TypeDef *USARTx, DDL_USART_ClockInitTypeDef *USART_ClockInitStruct);
void        DDL_USART_ClockStructInit(DDL_USART_ClockInitTypeDef *USART_ClockInitStruct);
/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* USART1 || USART2 || USART3 || USART6 || UART4 || UART5 || UART7 || UART8 || UART9 || UART10 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_USART_H */

