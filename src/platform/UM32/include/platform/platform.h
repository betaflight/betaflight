/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


#include "um324xx.h"
#include "um324xx_hal.h"
#include "system_um324xx.h"

#include "um324xx_ll_ex.h"
#include "um324xx_ll_tim.h"

#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#define USE_ITCM_RAM
#define ITCM_RAM_OPTIMISATION "-Ofast", "-freorder-blocks-algorithm=simple"

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
// #define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
//#define USE_MCO
//#define USE_MCO_DEVICE2
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10

// Helps with looptime stability as the CPU is borderline when running native gyro sampling
#define USE_OVERCLOCK
#define ENABLE_OVERCLOCK_288_MHZ 1
#define ENABLE_OVERCLOCK_336_MHZ 1
#define DEFAULT_CPU_OVERCLOCK 0

// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
  
// DMA to/from any memory
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))               
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM                     __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI                  DMA_RAM
#define DMA_RAM_R                   DMA_RAM
#define DMA_RAM_W                   DMA_RAM
#define DMA_RAM_RW                  DMA_RAM

#define USE_TIMER_MGMT
#define USE_TIMER_AF

// Camera control PWM availability 
#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE

// speed is packed between modebits 4 and 1,
// 7       6        5        4         3         2        1        0
// 0 <pupd-1> <pupd-0> <mode-4> <speed-1> <speed-0> <mode-1> <mode-0>
#define IO_CONFIG(mode, speed, pupd) (uint8_t)((mode) | ((speed) << 2) | (((pupd >> 15) | pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IOCFG_AF_PP_UP

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 5) & 0x03)

//#define FLASH_CONFIG_STREAMER_BUFFER_SIZE 16  // Flash word = 128-bits (4 rows, uint32_t per row - 4 x 32)
#define FLASH_CONFIG_BUFFER_TYPE uint32_t

#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

#define SPI_TRAIT_AF_PORT 1
#define SPI_TRAIT_AF_PIN  0

#define I2C_TRAIT_STATE   0
#define I2C_TRAIT_AF_PIN  0

#define MAX_SPI_PIN_SEL 2
// #define USE_TX_IRQ_HANDLER

#define I2CDEV_COUNT 1
#define SPIDEV_COUNT 4
#define QUADSPIDEV_COUNT 1

#define I2C_TRAIT_HANDLE 1
#define SPI_TRAIT_HANDLE 1
#define QSPI_TRAIT_HANDLE 1

//UART
#define UART_TX_BUFFER_ATTRIBUTE 
#define UART_RX_BUFFER_ATTRIBUTE  

// pin AF mode is configured for each pin individually
#define UART_TRAIT_AF_PORT 1
#define UART_TRAIT_AF_PIN  0

#define UART_TRAIT_PINSWAP 0
#define UARTHARDWARE_MAX_PINS 3

#define SERIAL_TRAIT_PIN_CONFIG 1
#define SERIAL_UART_FIRST_INDEX     1

// QUAD SPI
#define MAX_QUADSPI_PIN_SEL 4

#define PLATFORM_TRAIT_RCC 1
#define PLATFORM_TRAIT_CONFIG_HSE 1
#define PLATFORM_TRAIT_ADC_DEVICE 1
#define PLATFORM_TRAIT_SDIO_INIT 1

#define MCO_SOURCE_COUNT   8
#define MCO_DIVIDER_COUNT  5

#define DMA_TRAIT_CHANNEL 1
#define DMA_CHANREQ_STRING "Handshake"
#define DMA_STCH_STRING    "Stream"


#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
// If a particular target is short of ITCM RAM, defining FAST_CODE_PREF in the target.h file will
// cause functions decorated FAST_CODE_PREF to *not* go into ITCM RAM but if FAST_CODE_PREF is not
// defined for the target, FAST_CODE_PREF will become an alias to FAST_CODE (in the common post
// header file), and functions decorated with FAST_CODE_PREF *will* go into ITCM RAM.

#define FAST_CODE_NOINLINE          FAST_CODE NOINLINE
#else
#define FAST_CODE
#define FAST_CODE_NOINLINE          NOINLINE
#endif // USE_ITCM_RAM

#define FAST_IRQ_HANDLER            FAST_CODE      


#if defined(USE_EXST) && !defined(RAMBASED)
#define USE_FLASH_BOOT_LOADER
#endif

#if defined(USE_FLASH_MEMORY_MAPPED)
#if !defined(USE_RAM_CODE)
#define USE_RAM_CODE
#endif

#define MMFLASH_CODE RAM_CODE
#define MMFLASH_CODE_NOINLINE RAM_CODE NOINLINE

#define MMFLASH_DATA FAST_DATA
#define MMFLASH_DATA_ZERO_INIT FAST_DATA_ZERO_INIT
#endif

// NVIC priority utility macros
#define NVIC_PRIORITY_GROUPING NVIC_PRIORITYGROUP_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)>>4)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING))))

//Legacy
typedef SPI_EX_TypeDef          SPI_TypeDef;
typedef SPI_EX_HandleTypeDef    SPI_HandleTypeDef;       

typedef I2C_EX_TypeDef          I2C_TypeDef;             
typedef I2C_EX_HandleTypeDef    I2C_HandleTypeDef;       

typedef UART_EX_TypeDef         UART_TypeDef;            
typedef UART_EX_HandleTypeDef   UART_HandleTypeDef;

#define GPIO_MODE_OUTPUT_PP          GPIO_MODE_OUTPUT
#define GPIO_MODE_OUTPUT_OD          GPIO_MODE_OUTPUT
#define GPIO_SPEED_FREQ_VERY_HIGH    GPIO_SPEED_FREQ_HIGH
#define GPIO_MODE_AF_PP              GPIO_MODE_AF
#define GPIO_MODE_AF_OD              GPIO_MODE_AF

#define __HAL_UART_ENABLE_IT            __HAL_UART_EX_ENABLE_IT
#define __HAL_TIM_GetAutoreload         __HAL_TIM_GET_AUTORELOAD
#define __HAL_TIM_SetCounter            __HAL_TIM_SET_COUNTER
#define __HAL_RCC_TIM1_CLK_ENABLE       __HAL_RCM_TIM1_CLK_ENABLE
#define __HAL_RCC_TIM2_CLK_ENABLE       __HAL_RCM_TIM2_CLK_ENABLE
#define __HAL_RCC_TIM3_CLK_ENABLE       __HAL_RCM_TIM3_CLK_ENABLE
#define __HAL_RCC_TIM4_CLK_ENABLE       __HAL_RCM_TIM4_CLK_ENABLE
#define __HAL_RCC_TIM5_CLK_ENABLE       __HAL_RCM_TIM5_CLK_ENABLE
#define __HAL_RCC_TIM6_CLK_ENABLE       __HAL_RCM_TIM6_CLK_ENABLE
#define __HAL_RCC_TIM7_CLK_ENABLE       __HAL_RCM_TIM7_CLK_ENABLE
#define __HAL_RCC_TIM8_CLK_ENABLE       __HAL_RCM_TIM8_CLK_ENABLE
#define __HAL_RCC_TIM9_CLK_ENABLE       __HAL_RCM_TIM9_CLK_ENABLE
#define __HAL_RCC_TIM10_CLK_ENABLE      __HAL_RCM_TIM10_CLK_ENABLE
#define __HAL_RCC_TIM11_CLK_ENABLE      __HAL_RCM_TIM11_CLK_ENABLE
#define __HAL_RCC_TIM12_CLK_ENABLE      __HAL_RCM_TIM12_CLK_ENABLE
#define __HAL_RCC_TIM13_CLK_ENABLE      __HAL_RCM_TIM13_CLK_ENABLE
#define __HAL_RCC_TIM14_CLK_ENABLE      __HAL_RCM_TIM14_CLK_ENABLE
#define HAL_RCC_GetSysClockFreq         HAL_RCM_GetSysClockFreq
#define HAL_TIM_DMADelayPulseCplt       TIM_DMADelayPulseHalfCplt
#define HAL_TIM_DMAError                TIM_DMAError

#define UART_IT_TXE               UART_EX_IT_ETBEI
#define USART_TypeDef             UART_EX_TypeDef

#define USART1_IRQHandler         UART1_IRQHandler
#define USART2_IRQHandler         UART2_IRQHandler
#define USART3_IRQHandler         UART3_IRQHandler
#define USART6_IRQHandler         UART6_IRQHandler

#define UART_REG_RXD(base) (((USART_TypeDef *)(base))->RBR)
#define UART_REG_TXD(base) (((USART_TypeDef *)(base))->THR)


