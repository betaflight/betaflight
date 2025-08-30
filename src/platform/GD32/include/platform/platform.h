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

#pragma once

#if defined(GD32F405) || defined(GD32F407) || defined(GD32F425) || defined(GD32F427) || defined(GD32F450) || defined(GD32F460) || defined(GD32F470)

#include "gd32f4xx.h"

// Chip Unique ID on F4xx
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef GD32F4
#define GD32F4
#endif

#endif

#ifdef GD32F4

/* DMA data mode type */
typedef enum {
    DMA_DATA_MODE_SINGLE = 0,
    DMA_DATA_MODE_MULTI  = 1
} dma_data_mode_enum;

/* DMA general configuration struct */
typedef struct
{
    dma_data_mode_enum data_mode;
    dma_subperipheral_enum sub_periph;
    union {
        dma_single_data_parameter_struct init_struct_s;
        dma_multi_data_parameter_struct  init_struct_m;
    } config;
} dma_general_config_struct;  //todo: move to dma_bsp.h

#endif // GD32F4

#ifdef GD32F4

#define SPI_TRAIT_AF_PORT       1
#define SPI_TRAIT_AF_PIN        1
#define I2C_TRAIT_STATE         1
#define I2C_TRAIT_AF_PIN        1
#define I2CDEV_COUNT            3
#define PLATFORM_TRAIT_RCC      1
#define UART_TRAIT_AF_PORT      1
#define UART_TRAIT_AF_PIN       1
#define UART_TRAIT_PINSWAP      1
#define SERIAL_TRAIT_PIN_CONFIG 1
#define DMA_TRAIT_CHANNEL       1

#define PLATFORM_TRAIT_ADC_DEVICE 1

#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

typedef struct I2C_TypeDef         I2C_TypeDef;
typedef struct I2C_HandleTypeDef   I2C_HandleTypeDef;
typedef struct GPIO_TypeDef        GPIO_TypeDef;
typedef struct GPIO_InitTypeDef    GPIO_InitTypeDef;
typedef struct TIM_TypeDef         TIM_TypeDef;
typedef struct DMA_TypeDef         DMA_TypeDef;
typedef struct DMA_Stream_TypeDef  DMA_Stream_TypeDef;
typedef struct DMA_Channel_TypeDef DMA_Channel_TypeDef;
typedef struct SPI_TypeDef         SPI_TypeDef;
typedef struct ADC_TypeDef         ADC_TypeDef;
typedef struct USART_TypeDef       USART_TypeDef;
typedef struct TIM_Cmd             TIM_Cmd;
typedef struct TIM_CtrlPWMOutputs  TIM_CtrlPWMOutputs;
typedef struct TIM_TimeBaseInit    TIM_TimeBaseInit;
typedef struct TIM_ARRPreloadConfig TIM_ARRPreloadConfig;
typedef struct EXTI_TypeDef         EXTI_TypeDef;
typedef struct EXTI_InitTypeDef     EXTI_InitTypeDef;
#define DMA_InitTypeDef            dma_general_config_struct
#define TIM_OCInitTypeDef          timer_oc_parameter_struct
#define TIM_ICInitTypeDef          timer_ic_parameter_struct
#define TIM_OCStructInit           timer_channel_output_struct_para_init
#define TIM_TimeBaseInitTypeDef    timer_parameter_struct

#define TIM_ICPolarity_Falling      TIMER_IC_POLARITY_FALLING
#define TIM_ICPolarity_Rising       TIMER_IC_POLARITY_RISING

#define Bit_RESET                  0

#define DMA0_CH0_BASE        (DMA0 + 0x10)
#define DMA0_CH1_BASE        (DMA0 + 0x28)
#define DMA0_CH2_BASE        (DMA0 + 0x40)
#define DMA0_CH3_BASE        (DMA0 + 0x58)
#define DMA0_CH4_BASE        (DMA0 + 0x70)
#define DMA0_CH5_BASE        (DMA0 + 0x88)
#define DMA0_CH6_BASE        (DMA0 + 0xA0)
#define DMA0_CH7_BASE        (DMA0 + 0xB8)
#define DMA1_CH0_BASE        (DMA1 + 0x10)
#define DMA1_CH1_BASE        (DMA1 + 0x28)
#define DMA1_CH2_BASE        (DMA1 + 0x40)
#define DMA1_CH3_BASE        (DMA1 + 0x58)
#define DMA1_CH4_BASE        (DMA1 + 0x70)
#define DMA1_CH5_BASE        (DMA1 + 0x88)
#define DMA1_CH6_BASE        (DMA1 + 0xA0)
#define DMA1_CH7_BASE        (DMA1 + 0xB8)

extern void timerOCModeConfig(void *tim, uint8_t channel, uint16_t ocmode);
extern void gd32_timer_input_capture_config(void* timer, uint16_t channel, uint8_t state);
extern uint32_t timerPrescaler(const TIM_TypeDef *tim);

#define UART_TX_BUFFER_ATTRIBUTE /* EMPTY */
#define UART_RX_BUFFER_ATTRIBUTE /* EMPTY */

#endif // GD32F4

#define USE_ADC_DEVICE_0

#define GPIOA_BASE    GPIOA

#if defined(GD32F4)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#if defined(GD32F4)
#define FAST_IRQ_HANDLER
#endif

#if defined(GD32F4)
// F4 can't DMA to/from TCM (tightly-coupled memory) SRAM (where the stack lives)
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static
#endif

#if defined(GD32F4)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
// Reserved for other GD series
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#endif  // USE_DMA_RAM

#define USE_TIMER_MGMT
#define USE_TIMER_AF

#if defined(GD32F4)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_25MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_OD, GPIO_PUPD_NONE)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_NONE)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_25MHZ, 0, GPIO_PUPD_PULLUP)

#endif

#if defined(GD32F4)
#define FLASH_CONFIG_BUFFER_TYPE uint32_t
#endif

#if defined(GD32F4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#else
#error "Invalid GD32 MCU defined - requires SPI implementation"
#endif

#if defined(GD32F4)
#if defined(GD32F425)
#define SPIDEV_COUNT 3
#elif defined(GD32F460)
#define SPIDEV_COUNT 5
#endif
#else
#define SPIDEV_COUNT 4
#endif

#define SPI_RX_DATA_REGISTER(base) ((base)->DR)

#if defined(GD32F4)
#define MAX_SPI_PIN_SEL 3
#else
#error Unknown MCU family
#endif

#if defined(GD32F4)
#define USE_TX_IRQ_HANDLER
#endif

// all pins on given uart use same AF


#if defined(GD32F4)
#define UARTHARDWARE_MAX_PINS 4
#endif

#if defined(GD32F4)
#define UART_REG_RXD(base) (USART_DATA((uint32_t)base))
#define UART_REG_TXD(base) (USART_DATA((uint32_t)base))
#endif

#define SERIAL_TRAIT_PIN_CONFIG 1
#define USB_DP_PIN PA12

// Select UART prefix according to UART_DEV
#define _UART_GET_PREFIX(dev) _UART_GET_PREFIX_##dev

#define _UART_GET_PREFIX_UARTDEV_0 USART
#define _UART_GET_PREFIX_UARTDEV_1 USART
#define _UART_GET_PREFIX_UARTDEV_2 USART
#define _UART_GET_PREFIX_UARTDEV_3 UART
#define _UART_GET_PREFIX_UARTDEV_4 UART
#define _UART_GET_PREFIX_UARTDEV_5 USART
#define _UART_GET_PREFIX_UARTDEV_6 UART
#define _UART_GET_PREFIX_UARTDEV_7 UART
#define _UART_GET_PREFIX_UARTDEV_8 UART
#define _UART_GET_PREFIX_UARTDEV_9 UART
#define _UART_GET_PREFIX_UARTDEV_10 USART
#define _UART_GET_PREFIX_UARTDEV_LP1 LPUART

// #if defined(GD32F4)
// We need to redefine ADC0, ADC1, etc.,
// in the GD firmware library to be compatible with
// such as the ADC_TypeDef * type in BF.
#define GD_ADC0    ((ADC_TypeDef*)ADC_BASE)
#define GD_ADC1    ((ADC_TypeDef*)(ADC_BASE + 0x100))
#define GD_ADC2    ((ADC_TypeDef*)(ADC_BASE + 0x200))
#undef ADC0
#define ADC0       ((ADC_TypeDef*)GD_ADC0)
#undef ADC1
#define ADC1       ((ADC_TypeDef*)GD_ADC1)
#undef ADC2
#define ADC2       ((ADC_TypeDef*)GD_ADC2)

#define GD_SPI0    ((SPI_TypeDef*)(SPI_BASE + 0x0000F800U))
#define GD_SPI1    ((SPI_TypeDef*)SPI_BASE)
#define GD_SPI2    ((SPI_TypeDef*)(SPI_BASE + 0x00000400U))
#define GD_SPI3    ((SPI_TypeDef*)(SPI_BASE + 0x0000FC00U))
#define GD_SPI4    ((SPI_TypeDef*)(SPI_BASE + 0x00011800U))
#define GD_SPI5    ((SPI_TypeDef*)(SPI_BASE + 0x00011C00U))
#undef SPI0
#define SPI0       ((SPI_TypeDef*)GD_SPI0)
#undef SPI1
#define SPI1       ((SPI_TypeDef*)GD_SPI1)
#undef SPI2
#define SPI2       ((SPI_TypeDef*)GD_SPI2)
#undef SPI3
#define SPI3       ((SPI_TypeDef*)GD_SPI3)
#undef SPI4
#define SPI4       ((SPI_TypeDef*)GD_SPI4)
#undef SPI5
#define SPI5       ((SPI_TypeDef*)GD_SPI5)

// We also need to convert the pointer to the uint32_t
// type required by the GD firmware library.
#define PERIPH_INT(periph)    ((uint32_t)periph)
// #endif
