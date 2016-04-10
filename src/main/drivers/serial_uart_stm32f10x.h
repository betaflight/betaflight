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
#define UART1_TX_PIN                Pin_9  // PA9
#define UART1_RX_PIN                Pin_10 // PA10
#define UART1_GPIO                  GPIOA
#define UART1_APB2_PERIPHERALS      RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA
#define UART1_AHB_PERIPHERALS       RCC_AHBPeriph_DMA1
#endif

#ifndef UART2_GPIO
#define UART2_TX_PIN                Pin_2 // PA2
#define UART2_RX_PIN                Pin_3 // PA3
#define UART2_GPIO                  GPIOA
#define UART2_APB1_PERIPHERALS      RCC_APB1Periph_USART2
#define UART2_APB2_PERIPHERALS      RCC_APB2Periph_GPIOA
#define UART2_AHB_PERIPHERALS       RCC_AHBPeriph_DMA1
#endif

#ifndef UART3_GPIO
#define UART3_TX_PIN                Pin_10 // PB10
#define UART3_RX_PIN                Pin_11 // PB11
#define UART3_GPIO                  GPIOB
#define UART3_APB1_PERIPHERALS      RCC_APB1Periph_USART3
#define UART3_APB2_PERIPHERALS      RCC_APB2Periph_GPIOB
#endif

// pins for UART4 are fixed by hardware
#define UART4_TX_PIN                Pin_10 // PC10
#define UART4_RX_PIN                Pin_11 // PC11
#define UART4_GPIO                  GPIOC
#define UART4_APB1_PERIPHERALS      RCC_APB1Periph_UART4
#define UART4_APB2_PERIPHERALS      RCC_APB2Periph_GPIOC

// pins for UART5 are fixed by hardware and on GPIOC and D
#define UART5_TX_PIN                Pin_12 // PC12
#define UART5_RX_PIN                Pin_2  // PD2
#define UART5_GPIO_TX               GPIOC
#define UART5_GPIO_RX               GPIOD
#define UART5_APB1_PERIPHERALS      RCC_APB1Periph_UART5
#define UART5_APB2_PERIPHERALS_TX   RCC_APB2Periph_GPIOC
#define UART5_APB2_PERIPHERALS_RX   RCC_APB2Periph_GPIOD

