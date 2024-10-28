/**
  *
  * @file    apm32f4xx_ddl_dac.h
  * @brief   Header file of DAC DDL module.
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
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DDL_DAC_H
#define APM32F4xx_DDL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(DAC)

/** @defgroup DAC_DDL DAC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup DAC_DDL_Private_Constants DAC Private Constants
  * @{
  */

/* Internal masks for DAC channels definition */
/* To select into literal DDL_DAC_CHANNEL_x the relevant bits for:             */
/* - channel bits position into registers CR, MCR, CCR, SHHR, SHRR            */
/* - channel bits position into register SWTRIG                               */
/* - channel register offset of data holding register DHRx                    */
/* - channel register offset of data output register DORx                     */
#define DAC_CTRL_CH1_BITOFFSET           0UL   /* Position of channel bits into registers
                                                CR, MCR, CCR, SHHR, SHRR of channel 1 */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_CTRL_CH2_BITOFFSET           16UL  /* Position of channel bits into registers
                                                CR, MCR, CCR, SHHR, SHRR of channel 2 */
#define DAC_CTRL_CHX_BITOFFSET_MASK      (DAC_CTRL_CH1_BITOFFSET | DAC_CTRL_CH2_BITOFFSET)
#else
#define DAC_CTRL_CHX_BITOFFSET_MASK      (DAC_CTRL_CH1_BITOFFSET)
#endif /* DAC_CHANNEL2_SUPPORT */

#define DAC_SWTR_CH1                   (DAC_SWTRG_SWTRG1) /* Channel bit into register SWTRIGR of channel 1. */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_SWTR_CH2                   (DAC_SWTRG_SWTRG2) /* Channel bit into register SWTRIGR of channel 2. */
#define DAC_SWTR_CHX_MASK              (DAC_SWTR_CH1 | DAC_SWTR_CH2)
#else
#define DAC_SWTR_CHX_MASK              (DAC_SWTR_CH1)
#endif /* DAC_CHANNEL2_SUPPORT */

#define DAC_REG_DHR12R1_REGOFFSET      0x00000000UL            /* Register DHR12Rx channel 1 taken as reference */
#define DAC_REG_DHR12L1_REGOFFSET      0x00100000UL            /* Register offset of DHR12Lx channel 1 versus
                                                                  DHR12Rx channel 1 (shifted left of 20 bits)   */
#define DAC_REG_DHR8R1_REGOFFSET       0x02000000UL            /* Register offset of DHR8Rx  channel 1 versus
                                                                  DHR12Rx channel 1 (shifted left of 24 bits)   */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_REG_DHR12R2_REGOFFSET      0x00030000UL            /* Register offset of DHR12Rx channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 16 bits)   */
#define DAC_REG_DHR12L2_REGOFFSET      0x00400000UL            /* Register offset of DHR12Lx channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 20 bits)   */
#define DAC_REG_DHR8R2_REGOFFSET       0x05000000UL            /* Register offset of DHR8Rx  channel 2 versus
                                                                  DHR12Rx channel 1 (shifted left of 24 bits)   */
#endif /* DAC_CHANNEL2_SUPPORT */
#define DAC_REG_DHR12RX_REGOFFSET_MASK 0x000F0000UL
#define DAC_REG_DHR12LX_REGOFFSET_MASK 0x00F00000UL
#define DAC_REG_DHR8RX_REGOFFSET_MASK  0x0F000000UL
#define DAC_REG_DHRX_REGOFFSET_MASK    (DAC_REG_DHR12RX_REGOFFSET_MASK\
                                        | DAC_REG_DHR12LX_REGOFFSET_MASK | DAC_REG_DHR8RX_REGOFFSET_MASK)

#define DAC_REG_DOR1_REGOFFSET         0x00000000UL            /* Register DORx channel 1 taken as reference */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_REG_DOR2_REGOFFSET         0x10000000UL            /* Register offset of DORx channel 1 versus
                                                                  DORx channel 2 (shifted left of 28 bits)   */
#define DAC_REG_DORX_REGOFFSET_MASK    (DAC_REG_DOR1_REGOFFSET | DAC_REG_DOR2_REGOFFSET)
#endif /* DAC_CHANNEL2_SUPPORT */


#define DAC_REG_DHR_REGOFFSET_MASK_POSBIT0         0x0000000FUL /* Mask of data hold registers offset (DHR12Rx,
                                                                   DHR12Lx, DHR8Rx, ...) when shifted to position 0 */
#define DAC_REG_DORX_REGOFFSET_MASK_POSBIT0        0x00000001UL /* Mask of DORx registers offset when shifted
                                                                   to position 0                                    */
#define DAC_REG_SHSRX_REGOFFSET_MASK_POSBIT0       0x00000001UL /* Mask of SHSRx registers offset when shifted
                                                                   to position 0                                    */

#define DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS           16UL  /* Position of bits register offset of DHR12Rx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 16 bits)                   */
#define DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS           20UL  /* Position of bits register offset of DHR12Lx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 20 bits)                   */
#define DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS            24UL  /* Position of bits register offset of DHR8Rx
                                                                   channel 1 or 2 versus DHR12Rx channel 1
                                                                   (shifted left of 24 bits)                   */
#define DAC_REG_DORX_REGOFFSET_BITOFFSET_POS              28UL  /* Position of bits register offset of DORx
                                                                   channel 1 or 2 versus DORx channel 1
                                                                   (shifted left of 28 bits)                   */

/* DAC registers bits positions */
#if defined(DAC_CHANNEL2_SUPPORT)
#endif
#define DAC_DH12RDUAL_DATACH2_BITOFFSET_POS                DAC_DH12RDUAL_DATACH2_Pos
#define DAC_DH12LDUAL_DATACH2_BITOFFSET_POS                DAC_DH12LDUAL_DATACH2_Pos
#define DAC_DH8RDUAL_DATACH2_BITOFFSET_POS                 DAC_DH8RDUAL_DATACH2_Pos

/* Miscellaneous data */
#define DAC_DIGITAL_SCALE_12BITS                  4095UL   /* Full-scale digital value with a resolution of 12
                                                              bits (voltage range determined by analog voltage
                                                              references Vref+ and Vref-, refer to reference manual) */

/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/** @defgroup DAC_DDL_Private_Macros DAC Private Macros
  * @{
  */

/**
  * @brief  Driver macro reserved for internal use: set a pointer to
  *         a register from a register basis from which an offset
  *         is applied.
  * @param  __REG__ Register basis from which the offset is applied.
  * @param  __REG_OFFFSET__ Offset to be applied (unit: number of registers).
  * @retval Pointer to register address
  */
#define __DAC_PTR_REG_OFFSET(__REG__, __REG_OFFFSET__)                         \
  ((uint32_t *)((uint32_t) ((uint32_t)(&(__REG__)) + ((__REG_OFFFSET__) << 2UL))))

/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DAC_DDL_ES_INIT DAC Exported Init structure
  * @{
  */

/**
  * @brief  Structure definition of some features of DAC instance.
  */
typedef struct
{
  uint32_t TriggerSource;               /*!< Set the conversion trigger source for the selected DAC channel:
                                             internal (SW start) or from external peripheral
                                             (timer event, external interrupt line).
                                             This parameter can be a value of @ref DAC_DDL_EC_TRIGGER_SOURCE

                                             This feature can be modified afterwards using unitary
                                             function @ref DDL_DAC_SetTriggerSource(). */

  uint32_t WaveAutoGeneration;          /*!< Set the waveform automatic generation mode for the selected DAC channel.
                                             This parameter can be a value of @ref DAC_DDL_EC_WAVE_AUTO_GENERATION_MODE

                                             This feature can be modified afterwards using unitary
                                             function @ref DDL_DAC_SetWaveAutoGeneration(). */

  uint32_t WaveAutoGenerationConfig;    /*!< Set the waveform automatic generation mode for the selected DAC channel.
                                             If waveform automatic generation mode is set to noise, this parameter
                                             can be a value of @ref DAC_DDL_EC_WAVE_NOISE_LFSR_UNMASK_BITS
                                             If waveform automatic generation mode is set to triangle,
                                             this parameter can be a value of @ref DAC_DDL_EC_WAVE_TRIANGLE_AMPLITUDE
                                             @note If waveform automatic generation mode is disabled,
                                              this parameter is discarded.

                                             This feature can be modified afterwards using unitary
                                             function @ref DDL_DAC_SetWaveNoiseLFSR(),
                                             @ref DDL_DAC_SetWaveTriangleAmplitude()
                                             depending on the wave automatic generation selected. */

  uint32_t OutputBuffer;                /*!< Set the output buffer for the selected DAC channel.
                                             This parameter can be a value of @ref DAC_DDL_EC_OUTPUT_BUFFER

                                             This feature can be modified afterwards using unitary
                                             function @ref DDL_DAC_SetOutputBuffer(). */
} DDL_DAC_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DAC_DDL_Exported_Constants DAC Exported Constants
  * @{
  */

/** @defgroup DAC_DDL_EC_GET_FLAG DAC flags
  * @brief    Flags defines which can be used with DDL_DAC_ReadReg function
  * @{
  */
/* DAC channel 1 flags */
#define DDL_DAC_FLAG_DMAUDR1                (DAC_STS_DMAUDFLG1)   /*!< DAC channel 1 flag DMA underrun */
#if defined(DAC_CHANNEL2_SUPPORT)
/* DAC channel 2 flags */
#define DDL_DAC_FLAG_DMAUDR2                (DAC_STS_DMAUDFLG2)   /*!< DAC channel 2 flag DMA underrun */
#endif /* DAC_CHANNEL2_SUPPORT */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_IT DAC interruptions
  * @brief    IT defines which can be used with DDL_DAC_ReadReg and  DDL_DAC_WriteReg functions
  * @{
  */
#define DDL_DAC_IT_DMAUDRIE1                (DAC_CTRL_DMAUDIEN1) /*!< DAC channel 1 interruption DMA underrun */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DDL_DAC_IT_DMAUDRIE2                (DAC_CTRL_DMAUDIEN2) /*!< DAC channel 2 interruption DMA underrun */
#endif /* DAC_CHANNEL2_SUPPORT */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_CHANNEL DAC channels
  * @{
  */
#define DDL_DAC_CHANNEL_1                   (DAC_REG_DOR1_REGOFFSET | DAC_REG_DHR12R1_REGOFFSET | DAC_REG_DHR12L1_REGOFFSET | DAC_REG_DHR8R1_REGOFFSET | DAC_CTRL_CH1_BITOFFSET | DAC_SWTR_CH1) /*!< DAC channel 1 */
#if defined(DAC_CHANNEL2_SUPPORT)
#define DDL_DAC_CHANNEL_2                   (DAC_REG_DOR2_REGOFFSET | DAC_REG_DHR12R2_REGOFFSET | DAC_REG_DHR12L2_REGOFFSET | DAC_REG_DHR8R2_REGOFFSET | DAC_CTRL_CH2_BITOFFSET | DAC_SWTR_CH2) /*!< DAC channel 2 */
#endif /* DAC_CHANNEL2_SUPPORT */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_TRIGGER_SOURCE DAC trigger source
  * @{
  */
#define DDL_DAC_TRIG_SOFTWARE               (DAC_CTRL_TRGSELCH1_2 | DAC_CTRL_TRGSELCH1_1 | DAC_CTRL_TRGSELCH1_0) /*!< DAC channel conversion trigger internal (SW start) */
#define DDL_DAC_TRIG_EXT_TMR2_TRGO          (DAC_CTRL_TRGSELCH1_2                                  ) /*!< DAC channel conversion trigger from external peripheral: TIM2 TRGO. */
#define DDL_DAC_TRIG_EXT_TMR8_TRGO          (                                  DAC_CTRL_TRGSELCH1_0) /*!< DAC channel conversion trigger from external peripheral: TIM8 TRGO. */
#define DDL_DAC_TRIG_EXT_TMR4_TRGO          (DAC_CTRL_TRGSELCH1_2                  | DAC_CTRL_TRGSELCH1_0) /*!< DAC channel conversion trigger from external peripheral: TIM4 TRGO. */
#define DDL_DAC_TRIG_EXT_TMR6_TRGO          0x00000000UL                                       /*!< DAC channel conversion trigger from external peripheral: TIM6 TRGO. */
#define DDL_DAC_TRIG_EXT_TMR7_TRGO          (                 DAC_CTRL_TRGSELCH1_1                 ) /*!< DAC channel conversion trigger from external peripheral: TIM7 TRGO. */
#define DDL_DAC_TRIG_EXT_TMR5_TRGO          (                 DAC_CTRL_TRGSELCH1_1 | DAC_CTRL_TRGSELCH1_0) /*!< DAC channel conversion trigger from external peripheral: TIM5 TRGO. */
#define DDL_DAC_TRIG_EXT_EINT_LINE9         (DAC_CTRL_TRGSELCH1_2 | DAC_CTRL_TRGSELCH1_1                 ) /*!< DAC channel conversion trigger from external peripheral: external interrupt line 9. */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_WAVE_AUTO_GENERATION_MODE DAC waveform automatic generation mode
  * @{
  */
#define DDL_DAC_WAVE_AUTO_GENERATION_NONE     0x00000000UL                    /*!< DAC channel wave auto generation mode disabled. */
#define DDL_DAC_WAVE_AUTO_GENERATION_NOISE    (               DAC_CTRL_WAVENCH1_0) /*!< DAC channel wave auto generation mode enabled, set generated noise waveform. */
#define DDL_DAC_WAVE_AUTO_GENERATION_TRIANGLE (DAC_CTRL_WAVENCH1_1               ) /*!< DAC channel wave auto generation mode enabled, set generated triangle waveform. */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_WAVE_NOISE_LFSR_UNMASK_BITS DAC wave generation - Noise LFSR unmask bits
  * @{
  */
#define DDL_DAC_NOISE_LFSR_UNMASK_BIT0      0x00000000UL                                                        /*!< Noise wave generation, unmask LFSR bit0, for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS1_0   (                                                   DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[1:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS2_0   (                                  DAC_CTRL_MAMPSELCH1_1                 ) /*!< Noise wave generation, unmask LFSR bits[2:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS3_0   (                                  DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[3:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS4_0   (                 DAC_CTRL_MAMPSELCH1_2                                  ) /*!< Noise wave generation, unmask LFSR bits[4:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS5_0   (                 DAC_CTRL_MAMPSELCH1_2                  | DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[5:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS6_0   (                 DAC_CTRL_MAMPSELCH1_2 | DAC_CTRL_MAMPSELCH1_1                 ) /*!< Noise wave generation, unmask LFSR bits[6:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS7_0   (                 DAC_CTRL_MAMPSELCH1_2 | DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[7:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS8_0   (DAC_CTRL_MAMPSELCH1_3                                                   ) /*!< Noise wave generation, unmask LFSR bits[8:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS9_0   (DAC_CTRL_MAMPSELCH1_3                                   | DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[9:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS10_0  (DAC_CTRL_MAMPSELCH1_3                  | DAC_CTRL_MAMPSELCH1_1                 ) /*!< Noise wave generation, unmask LFSR bits[10:0], for the selected DAC channel */
#define DDL_DAC_NOISE_LFSR_UNMASK_BITS11_0  (DAC_CTRL_MAMPSELCH1_3                  | DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Noise wave generation, unmask LFSR bits[11:0], for the selected DAC channel */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_WAVE_TRIANGLE_AMPLITUDE DAC wave generation - Triangle amplitude
  * @{
  */
#define DDL_DAC_TRIANGLE_AMPLITUDE_1        0x00000000UL                                                        /*!< Triangle wave generation, amplitude of 1 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_3        (                                                   DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 3 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_7        (                                  DAC_CTRL_MAMPSELCH1_1                 ) /*!< Triangle wave generation, amplitude of 7 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_15       (                                  DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 15 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_31       (                 DAC_CTRL_MAMPSELCH1_2                                  ) /*!< Triangle wave generation, amplitude of 31 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_63       (                 DAC_CTRL_MAMPSELCH1_2                  | DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 63 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_127      (                 DAC_CTRL_MAMPSELCH1_2 | DAC_CTRL_MAMPSELCH1_1                 ) /*!< Triangle wave generation, amplitude of 127 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_255      (                 DAC_CTRL_MAMPSELCH1_2 | DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 255 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_511      (DAC_CTRL_MAMPSELCH1_3                                                   ) /*!< Triangle wave generation, amplitude of 512 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_1023     (DAC_CTRL_MAMPSELCH1_3                                   | DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 1023 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_2047     (DAC_CTRL_MAMPSELCH1_3                  | DAC_CTRL_MAMPSELCH1_1                 ) /*!< Triangle wave generation, amplitude of 2047 LSB of DAC output range, for the selected DAC channel */
#define DDL_DAC_TRIANGLE_AMPLITUDE_4095     (DAC_CTRL_MAMPSELCH1_3                  | DAC_CTRL_MAMPSELCH1_1 | DAC_CTRL_MAMPSELCH1_0) /*!< Triangle wave generation, amplitude of 4095 LSB of DAC output range, for the selected DAC channel */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_OUTPUT_BUFFER DAC channel output buffer
  * @{
  */
#define DDL_DAC_OUTPUT_BUFFER_ENABLE        0x00000000UL            /*!< The selected DAC channel output is buffered: higher drive current capability, but also higher current consumption */
#define DDL_DAC_OUTPUT_BUFFER_DISABLE       (DAC_CTRL_BUFFDCH1)          /*!< The selected DAC channel output is not buffered: lower drive current capability, but also lower current consumption */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_RESOLUTION  DAC channel output resolution
  * @{
  */
#define DDL_DAC_RESOLUTION_12B              0x00000000UL            /*!< DAC channel resolution 12 bits */
#define DDL_DAC_RESOLUTION_8B               0x00000002UL            /*!< DAC channel resolution 8 bits */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_REGISTERS  DAC registers compliant with specific purpose
  * @{
  */
/* List of DAC registers intended to be used (most commonly) with             */
/* DMA transfer.                                                              */
/* Refer to function @ref DDL_DAC_DMA_GetRegAddr().                            */
#define DDL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED  DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS /*!< DAC channel data holding register 12 bits right aligned */
#define DDL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED   DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS /*!< DAC channel data holding register 12 bits left aligned */
#define DDL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED   DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS  /*!< DAC channel data holding register 8 bits right aligned */
/**
  * @}
  */

/** @defgroup DAC_DDL_EC_HW_DELAYS  Definitions of DAC hardware constraints delays
  * @note   Only DAC peripheral HW delays are defined in DAC LL driver driver,
  *         not timeout values.
  *         For details on delays values, refer to descriptions in source code
  *         above each literal definition.
  * @{
  */

/* Delay for DAC channel voltage settling time from DAC channel startup       */
/* (transition from disable to enable).                                       */
/* Note: DAC channel startup time depends on board application environment:   */
/*       impedance connected to DAC channel output.                           */
/*       The delay below is specified under conditions:                       */
/*        - voltage maximum transition (lowest to highest value)              */
/*        - until voltage reaches final value +-1LSB                          */
/*        - DAC channel output buffer enabled                                 */
/*        - load impedance of 5kOhm (min), 50pF (max)                         */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tWAKEUP").                                                      */
/* Unit: us                                                                   */
#define DDL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US             15UL /*!< Delay for DAC channel voltage settling time from DAC channel startup (transition from disable to enable) */

/* Delay for DAC channel voltage settling time.                               */
/* Note: DAC channel startup time depends on board application environment:   */
/*       impedance connected to DAC channel output.                           */
/*       The delay below is specified under conditions:                       */
/*        - voltage maximum transition (lowest to highest value)              */
/*        - until voltage reaches final value +-1LSB                          */
/*        - DAC channel output buffer enabled                                 */
/*        - load impedance of 5kOhm min, 50pF max                             */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSETTLING").                                                    */
/* Unit: us                                                                   */
#define DDL_DAC_DELAY_VOLTAGE_SETTLING_US                    12UL /*!< Delay for DAC channel voltage settling time */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DAC_DDL_Exported_Macros DAC Exported Macros
  * @{
  */

/** @defgroup DAC_DDL_EM_WRITE_READ Common write and read registers macros
  * @{
  */

/**
  * @brief  Write a value in DAC register
  * @param  __INSTANCE__ DAC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_DAC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DAC register
  * @param  __INSTANCE__ DAC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_DAC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)

/**
  * @}
  */

/** @defgroup DAC_DDL_EM_HELPER_MACRO DAC helper macro
  * @{
  */

/**
  * @brief  Helper macro to get DAC channel number in decimal format
  *         from literals DDL_DAC_CHANNEL_x.
  *         Example:
  *            __DDL_DAC_CHANNEL_TO_DECIMAL_NB(DDL_DAC_CHANNEL_1)
  *            will return decimal number "1".
  * @note   The input can be a value from functions where a channel
  *         number is returned.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval 1...2 (value "2" depending on DAC channel 2 availability)
  */
#define __DDL_DAC_CHANNEL_TO_DECIMAL_NB(__CHANNEL__)                            \
  ((__CHANNEL__) & DAC_SWTR_CHX_MASK)

/**
  * @brief  Helper macro to get DAC channel in literal format DDL_DAC_CHANNEL_x
  *         from number in decimal format.
  *         Example:
  *           __DDL_DAC_DECIMAL_NB_TO_CHANNEL(1)
  *           will return a data equivalent to "DDL_DAC_CHANNEL_1".
  * @note  If the input parameter does not correspond to a DAC channel,
  *        this macro returns value '0'.
  * @param  __DECIMAL_NB__ 1...2 (value "2" depending on DAC channel 2 availability)
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  */
#if defined(DAC_CHANNEL2_SUPPORT)
#define __DDL_DAC_DECIMAL_NB_TO_CHANNEL(__DECIMAL_NB__)                         \
  (((__DECIMAL_NB__) == 1UL)                                                   \
    ? (                                                                        \
       DDL_DAC_CHANNEL_1                                                        \
      )                                                                        \
      :                                                                        \
      (((__DECIMAL_NB__) == 2UL)                                               \
        ? (                                                                    \
           DDL_DAC_CHANNEL_2                                                    \
          )                                                                    \
          :                                                                    \
          (                                                                    \
           0UL                                                                 \
          )                                                                    \
      )                                                                        \
  )
#else
#define __DDL_DAC_DECIMAL_NB_TO_CHANNEL(__DECIMAL_NB__)                         \
  (((__DECIMAL_NB__) == 1UL)                                                   \
    ? (                                                                        \
       DDL_DAC_CHANNEL_1                                                        \
      )                                                                        \
      :                                                                        \
      (                                                                        \
       0UL                                                                     \
      )                                                                        \
  )
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Helper macro to define the DAC conversion data full-scale digital
  *         value corresponding to the selected DAC resolution.
  * @note   DAC conversion data full-scale corresponds to voltage range
  *         determined by analog voltage references Vref+ and Vref-
  *         (refer to reference manual).
  * @param  __DAC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_RESOLUTION_12B
  *         @arg @ref DDL_DAC_RESOLUTION_8B
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __DDL_DAC_DIGITAL_SCALE(__DAC_RESOLUTION__)                             \
  ((0x00000FFFUL) >> ((__DAC_RESOLUTION__) << 1UL))

/**
  * @brief  Helper macro to calculate the DAC conversion data (unit: digital
  *         value) corresponding to a voltage (unit: mVolt).
  * @note   This helper macro is intended to provide input data in voltage
  *         rather than digital value,
  *         to be used with LL DAC functions such as
  *         @ref DDL_DAC_ConvertData12RightAligned().
  * @note   Analog reference voltage (Vref+) must be either known from
  *         user board environment or can be calculated using ADC measurement
  *         and ADC helper macro __DDL_ADC_CALC_VREFANALOG_VOLTAGE().
  * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
  * @param  __DAC_VOLTAGE__ Voltage to be generated by DAC channel
  *                         (unit: mVolt).
  * @param  __DAC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_RESOLUTION_12B
  *         @arg @ref DDL_DAC_RESOLUTION_8B
  * @retval DAC conversion data (unit: digital value)
  */
#define __DDL_DAC_CALC_VOLTAGE_TO_DATA(__VREFANALOG_VOLTAGE__,\
                                      __DAC_VOLTAGE__,\
                                      __DAC_RESOLUTION__)                      \
((__DAC_VOLTAGE__) * __DDL_DAC_DIGITAL_SCALE(__DAC_RESOLUTION__)              \
 / (__VREFANALOG_VOLTAGE__)                                                  \
)

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup DAC_DDL_Exported_Functions DAC Exported Functions
  * @{
  */
/**
  * @brief  Set the conversion trigger source for the selected DAC channel.
  * @note   For conversion trigger source to be effective, DAC trigger
  *         must be enabled using function @ref DDL_DAC_EnableTrigger().
  * @note   To set conversion trigger source, DAC channel must be disabled.
  *         Otherwise, the setting is discarded.
  * @note   Availability of parameters of trigger sources from timer
  *         depends on timers availability on the selected device.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  TriggerSource This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_TRIG_SOFTWARE
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR8_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR7_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR6_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR5_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR4_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR2_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_EINT_LINE9
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_SetTriggerSource(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t TriggerSource)
{
  MODIFY_REG(DACx->CTRL,
             DAC_CTRL_TRGSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK),
             TriggerSource << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get the conversion trigger source for the selected DAC channel.
  * @note   For conversion trigger source to be effective, DAC trigger
  *         must be enabled using function @ref DDL_DAC_EnableTrigger().
  * @note   Availability of parameters of trigger sources from timer
  *         depends on timers availability on the selected device.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_TRIG_SOFTWARE
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR8_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR7_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR6_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR5_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR4_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_TMR2_TRGO
  *         @arg @ref DDL_DAC_TRIG_EXT_EINT_LINE9
  */
__STATIC_INLINE uint32_t DDL_DAC_GetTriggerSource(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return (uint32_t)(READ_BIT(DACx->CTRL, DAC_CTRL_TRGSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
                    >> (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                   );
}

/**
  * @brief  Set the waveform automatic generation mode
  *         for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  WaveAutoGeneration This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_NONE
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_NOISE
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_TRIANGLE
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_SetWaveAutoGeneration(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t WaveAutoGeneration)
{
  MODIFY_REG(DACx->CTRL,
             DAC_CTRL_WAVENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK),
             WaveAutoGeneration << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get the waveform automatic generation mode
  *         for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_NONE
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_NOISE
  *         @arg @ref DDL_DAC_WAVE_AUTO_GENERATION_TRIANGLE
  */
__STATIC_INLINE uint32_t DDL_DAC_GetWaveAutoGeneration(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return (uint32_t)(READ_BIT(DACx->CTRL, DAC_CTRL_WAVENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
                    >> (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                   );
}

/**
  * @brief  Set the noise waveform generation for the selected DAC channel:
  *         Noise mode and parameters LFSR (linear feedback shift register).
  * @note   For wave generation to be effective, DAC channel
  *         wave generation mode must be enabled using
  *         function @ref DDL_DAC_SetWaveAutoGeneration().
  * @note   This setting can be set when the selected DAC channel is disabled
  *         (otherwise, the setting operation is ignored).
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  NoiseLFSRMask This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BIT0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS1_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS2_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS3_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS4_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS5_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS6_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS7_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS8_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS9_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS10_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS11_0
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_SetWaveNoiseLFSR(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t NoiseLFSRMask)
{
  MODIFY_REG(DACx->CTRL,
             DAC_CTRL_MAMPSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK),
             NoiseLFSRMask << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get the noise waveform generation for the selected DAC channel:
  *         Noise mode and parameters LFSR (linear feedback shift register).
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BIT0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS1_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS2_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS3_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS4_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS5_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS6_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS7_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS8_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS9_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS10_0
  *         @arg @ref DDL_DAC_NOISE_LFSR_UNMASK_BITS11_0
  */
__STATIC_INLINE uint32_t DDL_DAC_GetWaveNoiseLFSR(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return (uint32_t)(READ_BIT(DACx->CTRL, DAC_CTRL_MAMPSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
                    >> (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                   );
}

/**
  * @brief  Set the triangle waveform generation for the selected DAC channel:
  *         triangle mode and amplitude.
  * @note   For wave generation to be effective, DAC channel
  *         wave generation mode must be enabled using
  *         function @ref DDL_DAC_SetWaveAutoGeneration().
  * @note   This setting can be set when the selected DAC channel is disabled
  *         (otherwise, the setting operation is ignored).
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  TriangleAmplitude This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_1
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_3
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_7
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_15
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_31
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_63
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_127
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_255
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_511
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_1023
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_2047
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_4095
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_SetWaveTriangleAmplitude(DAC_TypeDef *DACx, uint32_t DAC_Channel,
                                                     uint32_t TriangleAmplitude)
{
  MODIFY_REG(DACx->CTRL,
             DAC_CTRL_MAMPSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK),
             TriangleAmplitude << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get the triangle waveform generation for the selected DAC channel:
  *         triangle mode and amplitude.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_1
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_3
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_7
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_15
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_31
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_63
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_127
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_255
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_511
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_1023
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_2047
  *         @arg @ref DDL_DAC_TRIANGLE_AMPLITUDE_4095
  */
__STATIC_INLINE uint32_t DDL_DAC_GetWaveTriangleAmplitude(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return (uint32_t)(READ_BIT(DACx->CTRL, DAC_CTRL_MAMPSELCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
                    >> (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                   );
}

/**
  * @brief  Set the output buffer for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  OutputBuffer This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_OUTPUT_BUFFER_ENABLE
  *         @arg @ref DDL_DAC_OUTPUT_BUFFER_DISABLE
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_SetOutputBuffer(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t OutputBuffer)
{
  MODIFY_REG(DACx->CTRL,
             DAC_CTRL_BUFFDCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK),
             OutputBuffer << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get the output buffer state for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DAC_OUTPUT_BUFFER_ENABLE
  *         @arg @ref DDL_DAC_OUTPUT_BUFFER_DISABLE
  */
__STATIC_INLINE uint32_t DDL_DAC_GetOutputBuffer(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return (uint32_t)(READ_BIT(DACx->CTRL, DAC_CTRL_BUFFDCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
                    >> (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK)
                   );
}

/**
  * @}
  */

/** @defgroup DAC_DDL_EF_DMA_Management DMA Management
  * @{
  */

/**
  * @brief  Enable DAC DMA transfer request of the selected channel.
  * @note   To configure DMA source address (peripheral address),
  *         use function @ref DDL_DAC_DMA_GetRegAddr().
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_EnableDMAReq(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  SET_BIT(DACx->CTRL,
          DAC_CTRL_DMAENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Disable DAC DMA transfer request of the selected channel.
  * @note   To configure DMA source address (peripheral address),
  *         use function @ref DDL_DAC_DMA_GetRegAddr().
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_DisableDMAReq(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  CLEAR_BIT(DACx->CTRL,
            DAC_CTRL_DMAENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get DAC DMA transfer request state of the selected channel.
  *         (0: DAC DMA transfer request is disabled, 1: DAC DMA transfer request is enabled)
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsDMAReqEnabled(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return ((READ_BIT(DACx->CTRL,
                    DAC_CTRL_DMAENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
           == (DAC_CTRL_DMAENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Function to help to configure DMA transfer to DAC: retrieve the
  *         DAC register address from DAC instance and a list of DAC registers
  *         intended to be used (most commonly) with DMA transfer.
  * @note   These DAC registers are data holding registers:
  *         when DAC conversion is requested, DAC generates a DMA transfer
  *         request to have data available in DAC data holding registers.
  * @note   This macro is intended to be used with LL DMA driver, refer to
  *         function "DDL_DMA_ConfigAddresses()".
  *         Example:
  *           DDL_DMA_ConfigAddresses(DMA1,
  *                                  DDL_DMA_CHANNEL_1,
  *                                  (uint32_t)&< array or variable >,
  *                                  DDL_DAC_DMA_GetRegAddr(DAC1, DDL_DAC_CHANNEL_1,
  *                                  DDL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED),
  *                                  DDL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  Register This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED
  *         @arg @ref DDL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED
  *         @arg @ref DDL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED
  * @retval DAC register address
  */
__STATIC_INLINE uint32_t DDL_DAC_DMA_GetRegAddr(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t Register)
{
  /* Retrieve address of register DHR12Rx, DHR12Lx or DHR8Rx depending on     */
  /* DAC channel selected.                                                    */
  return ((uint32_t)(__DAC_PTR_REG_OFFSET((DACx)->DH12R1, ((DAC_Channel >> (Register & 0x1FUL))
                                                            & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0))));
}
/**
  * @}
  */

/** @defgroup DAC_DDL_EF_Operation Operation on DAC channels
  * @{
  */

/**
  * @brief  Enable DAC selected channel.
  * @note   After enable from off state, DAC channel requires a delay
  *         for output voltage to reach accuracy +/- 1 LSB.
  *         Refer to device datasheet, parameter "tWAKEUP".
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_Enable(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  SET_BIT(DACx->CTRL,
          DAC_CTRL_ENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Disable DAC selected channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_Disable(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  CLEAR_BIT(DACx->CTRL,
            DAC_CTRL_ENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get DAC enable state of the selected channel.
  *         (0: DAC channel is disabled, 1: DAC channel is enabled)
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsEnabled(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return ((READ_BIT(DACx->CTRL,
                    DAC_CTRL_ENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
           == (DAC_CTRL_ENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Enable DAC trigger of the selected channel.
  * @note   - If DAC trigger is disabled, DAC conversion is performed
  *           automatically once the data holding register is updated,
  *           using functions "DDL_DAC_ConvertData{8; 12}{Right; Left} Aligned()":
  *           @ref DDL_DAC_ConvertData12RightAligned(), ...
  *         - If DAC trigger is enabled, DAC conversion is performed
  *           only when a hardware of software trigger event is occurring.
  *           Select trigger source using
  *           function @ref DDL_DAC_SetTriggerSource().
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_EnableTrigger(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  SET_BIT(DACx->CTRL,
          DAC_CTRL_TRGENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Disable DAC trigger of the selected channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_DisableTrigger(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  CLEAR_BIT(DACx->CTRL,
            DAC_CTRL_TRGENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK));
}

/**
  * @brief  Get DAC trigger state of the selected channel.
  *         (0: DAC trigger is disabled, 1: DAC trigger is enabled)
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsTriggerEnabled(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  return ((READ_BIT(DACx->CTRL,
                    DAC_CTRL_TRGENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))
           == (DAC_CTRL_TRGENCH1 << (DAC_Channel & DAC_CTRL_CHX_BITOFFSET_MASK))) ? 1UL : 0UL);
}

/**
  * @brief  Trig DAC conversion by software for the selected DAC channel.
  * @note   Preliminarily, DAC trigger must be set to software trigger
  *         using function
  *           @ref DDL_DAC_Init()
  *           @ref DDL_DAC_SetTriggerSource()
  *         with parameter "DDL_DAC_TRIGGER_SOFTWARE".
  *         and DAC trigger must be enabled using
  *         function @ref DDL_DAC_EnableTrigger().
  * @note   For devices featuring DAC with 2 channels: this function
  *         can perform a SW start of both DAC channels simultaneously.
  *         Two channels can be selected as parameter.
  *         Example: (DDL_DAC_CHANNEL_1 | DDL_DAC_CHANNEL_2)
  * @param  DACx DAC instance
  * @param  DAC_Channel  This parameter can a combination of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_TrigSWConversion(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  SET_BIT(DACx->SWTRG,
          (DAC_Channel & DAC_SWTR_CHX_MASK));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (LSB aligned on bit 0),
  *         for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  Data Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertData12RightAligned(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t Data)
{
  __IO uint32_t *preg = __DAC_PTR_REG_OFFSET(DACx->DH12R1, (DAC_Channel >> DAC_REG_DHR12RX_REGOFFSET_BITOFFSET_POS)
                                             & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  MODIFY_REG(*preg, DAC_DH12R1_DATA, Data);
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (MSB aligned on bit 15),
  *         for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  Data Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertData12LeftAligned(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t Data)
{
  __IO uint32_t *preg = __DAC_PTR_REG_OFFSET(DACx->DH12R1, (DAC_Channel >> DAC_REG_DHR12LX_REGOFFSET_BITOFFSET_POS)
                                             & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  MODIFY_REG(*preg, DAC_DH12L1_DATA, Data);
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 8 bits left alignment (LSB aligned on bit 0),
  *         for the selected DAC channel.
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @param  Data Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertData8RightAligned(DAC_TypeDef *DACx, uint32_t DAC_Channel, uint32_t Data)
{
  __IO uint32_t *preg = __DAC_PTR_REG_OFFSET(DACx->DH12R1, (DAC_Channel >> DAC_REG_DHR8RX_REGOFFSET_BITOFFSET_POS)
                                             & DAC_REG_DHR_REGOFFSET_MASK_POSBIT0);

  MODIFY_REG(*preg, DAC_DH8R1_DATA, Data);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (LSB aligned on bit 0),
  *         for both DAC channels.
  * @param  DACx DAC instance
  * @param  DataChannel1 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @param  DataChannel2 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertDualData12RightAligned(DAC_TypeDef *DACx, uint32_t DataChannel1,
                                                          uint32_t DataChannel2)
{
  MODIFY_REG(DACx->DH12RDUAL,
             (DAC_DH12RDUAL_DATACH2 | DAC_DH12RDUAL_DATACH1),
             ((DataChannel2 << DAC_DH12RDUAL_DATACH2_BITOFFSET_POS) | DataChannel1));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 12 bits left alignment (MSB aligned on bit 15),
  *         for both DAC channels.
  * @param  DACx DAC instance
  * @param  DataChannel1 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @param  DataChannel2 Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertDualData12LeftAligned(DAC_TypeDef *DACx, uint32_t DataChannel1,
                                                         uint32_t DataChannel2)
{
  /* Note: Data of DAC channel 2 shift value subtracted of 4 because          */
  /*       data on 16 bits and DAC channel 2 bits field is on the 12 MSB,     */
  /*       the 4 LSB must be taken into account for the shift value.          */
  MODIFY_REG(DACx->DH12LDUAL,
             (DAC_DH12LDUAL_DATACH2 | DAC_DH12LDUAL_DATACH1),
             ((DataChannel2 << (DAC_DH12LDUAL_DATACH2_BITOFFSET_POS - 4U)) | DataChannel1));
}

/**
  * @brief  Set the data to be loaded in the data holding register
  *         in format 8 bits left alignment (LSB aligned on bit 0),
  *         for both DAC channels.
  * @param  DACx DAC instance
  * @param  DataChannel1 Value between Min_Data=0x00 and Max_Data=0xFF
  * @param  DataChannel2 Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ConvertDualData8RightAligned(DAC_TypeDef *DACx, uint32_t DataChannel1,
                                                         uint32_t DataChannel2)
{
  MODIFY_REG(DACx->DH8RDUAL,
             (DAC_DH8RDUAL_DATACH2 | DAC_DH8RDUAL_DATACH1),
             ((DataChannel2 << DAC_DH8RDUAL_DATACH2_BITOFFSET_POS) | DataChannel1));
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Retrieve output data currently generated for the selected DAC channel.
  * @note   Whatever alignment and resolution settings
  *         (using functions "DDL_DAC_ConvertData{8; 12}{Right; Left} Aligned()":
  *         @ref DDL_DAC_ConvertData12RightAligned(), ...),
  *         output data format is 12 bits right aligned (LSB aligned on bit 0).
  * @param  DACx DAC instance
  * @param  DAC_Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DAC_CHANNEL_1
  *         @arg @ref DDL_DAC_CHANNEL_2 (1)
  *
  *         (1) On this APM32 serie, parameter not available on all devices.
  *             Refer to device datasheet for channels availability.
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t DDL_DAC_RetrieveOutputData(DAC_TypeDef *DACx, uint32_t DAC_Channel)
{
  __IO uint32_t const *preg = __DAC_PTR_REG_OFFSET(DACx->DATAOCH1, (DAC_Channel >> DAC_REG_DORX_REGOFFSET_BITOFFSET_POS)
                                                   & DAC_REG_DORX_REGOFFSET_MASK_POSBIT0);

  return (uint16_t) READ_BIT(*preg, DAC_DATAOCH1_DATA);
}

/**
  * @}
  */

/** @defgroup DAC_DDL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Get DAC underrun flag for DAC channel 1
  * @param  DACx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsActiveFlag_DMAUDR1(DAC_TypeDef *DACx)
{
  return ((READ_BIT(DACx->STS, DDL_DAC_FLAG_DMAUDR1) == (DDL_DAC_FLAG_DMAUDR1)) ? 1UL : 0UL);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Get DAC underrun flag for DAC channel 2
  * @param  DACx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsActiveFlag_DMAUDR2(DAC_TypeDef *DACx)
{
  return ((READ_BIT(DACx->STS, DDL_DAC_FLAG_DMAUDR2) == (DDL_DAC_FLAG_DMAUDR2)) ? 1UL : 0UL);
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Clear DAC underrun flag for DAC channel 1
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ClearFlag_DMAUDR1(DAC_TypeDef *DACx)
{
  WRITE_REG(DACx->STS, DDL_DAC_FLAG_DMAUDR1);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Clear DAC underrun flag for DAC channel 2
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_ClearFlag_DMAUDR2(DAC_TypeDef *DACx)
{
  WRITE_REG(DACx->STS, DDL_DAC_FLAG_DMAUDR2);
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @}
  */

/** @defgroup DAC_DDL_EF_IT_Management IT management
  * @{
  */

/**
  * @brief  Enable DMA underrun interrupt for DAC channel 1
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_EnableIT_DMAUDR1(DAC_TypeDef *DACx)
{
  SET_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE1);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Enable DMA underrun interrupt for DAC channel 2
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_EnableIT_DMAUDR2(DAC_TypeDef *DACx)
{
  SET_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE2);
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Disable DMA underrun interrupt for DAC channel 1
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_DisableIT_DMAUDR1(DAC_TypeDef *DACx)
{
  CLEAR_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE1);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Disable DMA underrun interrupt for DAC channel 2
  * @param  DACx DAC instance
  * @retval None
  */
__STATIC_INLINE void DDL_DAC_DisableIT_DMAUDR2(DAC_TypeDef *DACx)
{
  CLEAR_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE2);
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @brief  Get DMA underrun interrupt for DAC channel 1
  * @param  DACx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsEnabledIT_DMAUDR1(DAC_TypeDef *DACx)
{
  return ((READ_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE1) == (DDL_DAC_IT_DMAUDRIE1)) ? 1UL : 0UL);
}

#if defined(DAC_CHANNEL2_SUPPORT)
/**
  * @brief  Get DMA underrun interrupt for DAC channel 2
  * @param  DACx DAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DAC_IsEnabledIT_DMAUDR2(DAC_TypeDef *DACx)
{
  return ((READ_BIT(DACx->CTRL, DDL_DAC_IT_DMAUDRIE2) == (DDL_DAC_IT_DMAUDRIE2)) ? 1UL : 0UL);
}
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DAC_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus DDL_DAC_DeInit(DAC_TypeDef *DACx);
ErrorStatus DDL_DAC_Init(DAC_TypeDef *DACx, uint32_t DAC_Channel, DDL_DAC_InitTypeDef *DAC_InitStruct);
void        DDL_DAC_StructInit(DDL_DAC_InitTypeDef *DAC_InitStruct);

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

#endif /* DAC */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_DAC_H */

