/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * HPMicro MCU compatibility layer: FPU detection, common type definitions and
 * STM32-style peripheral instance aliases on top of the HPM SDK.
 */

#pragma once

/* ============================================================================
 * FPU Detection for RISC-V
 * ========================================================================== */
/*
 * CMSIS-Core __FPU_PRESENT / __FPU_USED macros do not exist on RISC-V.
 * Use the GCC built-in __riscv_flen which is defined when -march includes 'F' or 'D'.
 * RV32IMAFC → __riscv_flen = 32 (single-precision FPU present).
 */
#ifdef __riscv_flen
#define __FPU_PRESENT 1
#define __FPU_USED    1
#else
#define __FPU_PRESENT 0
#define __FPU_USED    0
#endif

/* ============================================================================
 * Common Libraries
 * ========================================================================== */
/*
 * Ensure Betaflight's statement-expression MIN/MAX are defined
 * before HPM SDK's hpm_common.h, which defines MIN as a simple
 * macro that evaluates arguments twice (breaking MIN(x, sbufReadU8(buf))
 * in MSP2_SET_TEXT name processing).
 */
#include "common/maths.h"

/* ============================================================================
 * HPM SDK Headers
 * ========================================================================== */

#include "hpm_soc.h"
#include "hpm_interrupt.h"
#include "hpm_romapi.h"
#include "hpm_clock_drv.h"
#include "hpm_dma_drv.h"

/* ============================================================================
 * System Variables
 * ========================================================================== */

extern uint32_t SystemCoreClock;

/* ============================================================================
 * Common Type Definitions
 * ========================================================================== */

typedef uint32_t FunctionalState;
typedef uint32_t IRQn_Type;

typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10,
} EXTITrigger_TypeDef;

/* ============================================================================
 * GPIO Configuration Definitions
 * ========================================================================== */

/** @defgroup GPIO_mode_define GPIO mode define
 * @brief GPIO Configuration Mode
 *        Elements values convention: 0xX0yz00YZ
 *           - X  : GPIO mode or EXTI Mode
 *           - y  : External IT or Event trigger detection
 *           - z  : IO configuration on External IT or Event
 *           - Y  : Output type (Push Pull or Open Drain)
 *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
 * @{
 */
#define GPIO_MODE_INPUT         0x00000000U     /*!< Input Floating Mode                   */
#define GPIO_MODE_OUTPUT_PP     0x00000001U     /*!< Output Push Pull Mode                 */
#define GPIO_MODE_OUTPUT_OD     0x00000011U     /*!< Output Open Drain Mode                */
#define GPIO_MODE_AF_PP         0x00000002U     /*!< Alternate Function Push Pull Mode     */
#define GPIO_MODE_AF_OD         0x00000012U     /*!< Alternate Function Open Drain Mode    */
#define GPIO_SPEED_FREQ_LOW     0x00000000U     /*!< IO works at 2 MHz                     */
#define GPIO_MODE_ANALOG        0x00000003U     /*!< Analog Mode  */
/**
 * @}
 */

/** @defgroup GPIO_pull_define GPIO pull define
 * @brief GPIO Pull-Up or Pull-Down Activation
 * @{
 */
#define GPIO_NOPULL    0x00000000U      /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP    0x00000001U      /*!< Pull-up activation                  */
#define GPIO_PULLDOWN  0x00000002U      /*!< Pull-down activation                */
/**
 * @}
 */

/* ============================================================================
 * Peripheral Type Definitions
 * ========================================================================== */

#ifdef HPM6750
typedef ADC12_Type ADC_TypeDef;
#else
typedef ADC16_Type ADC_TypeDef;
#endif

typedef PWM_Type TIM_TypeDef;
typedef dma_channel_config_t DMA_InitTypeDef;
typedef SPI_Type SPI_TypeDef;
typedef I2C_Type I2C_TypeDef;

/* ============================================================================
 * Peripheral Instance Aliases
 * ========================================================================== */

/* SPI instances */
#ifndef SPI1
#define SPI1 ((SPI_TypeDef *) HPM_SPI0)
#endif

#ifndef SPI2
#define SPI2 ((SPI_TypeDef *) HPM_SPI1)
#endif

#ifndef SPI3
#define SPI3 ((SPI_TypeDef *) HPM_SPI2)
#endif

#ifndef SPI4
#define SPI4 ((SPI_TypeDef *) HPM_SPI3)
#endif

/* ADC instances */
#ifndef ADC1
#define ADC1 ((ADC_TypeDef *) HPM_ADC0)
#endif

#ifndef ADC2
#define ADC2 ((ADC_TypeDef *) HPM_ADC1)
#endif

#ifndef ADC3
#define ADC3 ((ADC_TypeDef *) HPM_ADC2)
#endif
