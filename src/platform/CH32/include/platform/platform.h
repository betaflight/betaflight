/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * porting for ch32h41x by Temperslee
 */
#pragma once

#if defined(CH32H415)


#include "ch32h417.h"
#include "ch32_debug.h"


// Chip Unique ID on H41X
#define U_ID_0 (*(uint32_t*)0x1ffff7e8)
#define U_ID_1 (*(uint32_t*)0x1ffff7ec)
#define U_ID_2 (*(uint32_t*)0x1ffff7f0)

#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   256     // fast program is 256bytes
#define FLASH_CONFIG_BUFFER_TYPE            uint32_t

#define __FAST_INTERRUPT       __attribute__((interrupt("WCH-Interrupt-fast")))


#ifndef CH32H4
#define CH32H4
#endif

#define SPI_TRAIT_AF_PIN 1
#define I2C_TRAIT_AF_PIN 1
#define I2C_TRAIT_STATE 1

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#endif


#ifdef CH32H4

#define USE_ITCM_RAM
#define ITCM_RAM_OPTIMISATION "-O2", "-freorder-blocks-algorithm=simple"
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
// #define USE_OVERCLOCK
#define USE_ADC_INTERNAL
#define ADC_VOLTAGE_REFERENCE_MV  3300  //maybe better  cali internal
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
// #define USE_MCO
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#define PLATFORM_TRAIT_ADC_DEVICE 1

#endif



#ifdef CH32H4
#define TASK_GYROPID_DESIRED_PERIOD     125 // 1000us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#ifdef CH32H4
// Move ISRs to fast ram to avoid flash latency.
#define FAST_IRQ_HANDLER FAST_CODE 
#else 
#define FAST_IRQ_HANDLER  
#endif

//load all data to DTCM
#define DMA_DATA_ZERO_INIT __attribute__((aligned(32)))
#define DMA_DATA   __attribute__((aligned(32)))
#define STATIC_DMA_DATA_AUTO      __attribute__((aligned(32)))  static

#define DMA_RAM      __attribute__((aligned(32)))
#define DMA_RW_AXI  
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
//--------------------------

#define USE_TIMER_MGMT
#define USE_TIMER_AF


#if defined(CH32H4)
#define GPIO_PIN_RESET       0
#define DIR_OUT              0x03
#define DIR_IN               0x00

#define GPIO_MODE_IN_AN      0x00
#define GPIO_MODE_IN_FLOAT   0x01
#define GPIO_MODE_IN_PULL    0x02

#define GPIO_MODE_OUT_PP     0x00
#define GPIO_MODE_OUT_OD     0x01
#define GPIO_MODE_OUT_AF_PP  0x02
#define GPIO_MODE_OUT_AF_OD  0x03

#define GPIO_SPEED_LOW          0x00
#define GPIO_SPEED_Medium       0x01
#define GPIO_SPEED_HIGH         0x02
#define GPIO_SPEED_VERY_HIGH    0x03

#define GPIO_PULL_NONE       0x00          
#define GPIO_PULL_DOWN       0x01          
#define GPIO_PULL_UP         0x02          

//bit 0   1      2   3      4    5       6  7  
//  DIR[1:0]   MODE[1:0]   SPEED[1:0]   PD=0,PU=1  
#define IO_CONFIG(dir, mode, speed, pupd) ((dir) | ((mode) << 2) | ((speed) << 4) | ((pupd) << 6))


#define IOCFG_OUT_PP         IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_PP , GPIO_SPEED_VERY_HIGH , GPIO_PULL_NONE )  // TODO
#define IOCFG_OUT_PP_UP      IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_PP , GPIO_SPEED_VERY_HIGH , GPIO_PULL_UP )
#define IOCFG_OUT_PP_25      IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_PP , GPIO_SPEED_VERY_HIGH,  GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_OD , GPIO_SPEED_VERY_HIGH , GPIO_PULL_NONE)

#define IOCFG_AF_PP          IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH , GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH , GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH , GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(DIR_OUT , GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH , GPIO_PULL_NONE)

#define IOCFG_IPD            IO_CONFIG(DIR_IN  , GPIO_MODE_IN_PULL,   GPIO_SPEED_VERY_HIGH , GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(DIR_IN  , GPIO_MODE_IN_PULL,   GPIO_SPEED_VERY_HIGH , GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(DIR_IN  , GPIO_MODE_IN_FLOAT,  GPIO_SPEED_VERY_HIGH , GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(DIR_IN  , GPIO_MODE_IN_PULL,   GPIO_SPEED_VERY_HIGH , GPIO_PULL_UP)

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 4) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 0) & 0x03)  //don't need
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 6) & 0x03)



#define SPI_IO_AF_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
// #define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_DOWN)
// #define SPI_IO_AF_SDI_CFG       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define SPI_IO_AF_SDI_CFG       IO_CONFIG(DIR_IN, GPIO_MODE_IN_FLOAT, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define SPI_IO_CS_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

#define SPIDEV_COUNT       4

#define CHECK_SPI_RX_DATA_AVAILABLE(instance)  (READ_BIT(instance->STATR, SPI_I2S_FLAG_RXNE) == (SPI_I2S_FLAG_RXNE))
#define SPI_RX_DATA_REGISTER(base) ((base)->DATAR)

#define MAX_SPI_PIN_SEL    5        
// #define USE_TX_IRQ_HANDLER  


#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

#define PLATFORM_TRAIT_RCC      1
#define UART_TRAIT_AF_PIN       1
// #define UART_TRAIT_PINSWAP      1
#define SERIAL_TRAIT_PIN_CONFIG 1
#define I2C_TRAIT_AF_PIN        1
#define I2CDEV_COUNT            4
// #define I2C_TRAIT_HANDLE        1
#define SPI_TRAIT_AF_PIN        1
#define UARTHARDWARE_MAX_PINS   5

#define UART_REG_RXD(base) ((base)->DATAR)
#define UART_REG_TXD(base) ((base)->DATAR)

#define DMA_TRAIT_MUX 1
// #define DMA_TRAIT_CHANNEL 1

#define MAX_MSP_PORT_COUNT 4  //default msp count is 3, it would not be enough for us

#endif

#define FLASH_CONFIG_BUFFER_TYPE      uint32_t
#define USB_DP_PIN PB8
