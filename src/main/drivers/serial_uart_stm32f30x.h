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

#pragma once


#ifndef UART1_GPIO
#define UART1_TX_PIN                GPIO_Pin_9  // PA9
#define UART1_RX_PIN                GPIO_Pin_10 // PA10
#define UART1_GPIO                  GPIOA
#define UART1_GPIO_AF               GPIO_AF_7
#define UART1_TX_PINSOURCE          GPIO_PinSource9
#define UART1_RX_PINSOURCE          GPIO_PinSource10
#endif

#ifndef UART2_GPIO
#define UART2_TX_PIN                GPIO_Pin_5 // PD5
#define UART2_RX_PIN                GPIO_Pin_6 // PD6
#define UART2_GPIO                  GPIOD
#define UART2_GPIO_AF               GPIO_AF_7
#define UART2_TX_PINSOURCE          GPIO_PinSource5
#define UART2_RX_PINSOURCE          GPIO_PinSource6
#endif

#ifndef UART3_GPIO
#define UART3_TX_PIN                GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN                GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF               GPIO_AF_7
#define UART3_GPIO                  GPIOB
#define UART3_TX_PINSOURCE          GPIO_PinSource10
#define UART3_RX_PINSOURCE          GPIO_PinSource11
#endif

// pins for UART4 are fixed by hardware
#define UART4_TX_PIN                GPIO_Pin_10 // PC10
#define UART4_RX_PIN                GPIO_Pin_11 // PC11
#define UART4_GPIO_AF               GPIO_AF_5
#define UART4_GPIO                  GPIOC
#define UART4_APB1_PERIPHERALS      RCC_APB1Periph_UART4
#define UART4_AHB_PERIPHERALS       RCC_AHBPeriph_GPIOC
#define UART4_TX_PINSOURCE          GPIO_PinSource10
#define UART4_RX_PINSOURCE          GPIO_PinSource11


// pins for UART5 are fixed by hardware and on GPIOC and D
#define UART5_TX_PIN                Pin_12 // PC12
#define UART5_RX_PIN                Pin_2  // PD2
#define UART5_GPIO_AF               GPIO_AF_5
#define UART5_GPIO_TX               GPIOC
#define UART5_GPIO_RX               GPIOD
#define UART5_APB1_PERIPHERALS      RCC_APB1Periph_UART5
#define UART5_AHB_PERIPHERALS_TX    RCC_AHBPeriph_GPIOC
#define UART5_AHB_PERIPHERALS_RX    RCC_AHBPeriph_GPIOD
#define UART5_TX_PINSOURCE          GPIO_PinSource12
#define UART5_RX_PINSOURCE          GPIO_PinSource2
