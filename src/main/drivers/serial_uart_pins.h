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

 #if defined(STM32F1)

 #ifdef USE_UART1
 #ifndef UART1_TX_PIN
 #define UART1_TX_PIN        PA9
 #endif
 #ifndef UART1_RX_PIN
 #define UART1_RX_PIN        PA10
 #endif
 #endif

 #ifdef USE_UART2
 #ifndef UART2_TX_PIN
 #define UART2_TX_PIN        PA2
 #endif
 #ifndef UART2_RX_PIN
 #define UART2_RX_PIN        PA3
 #endif
 #endif

 #ifdef USE_UART3
 #ifndef UART3_TX_PIN
 #define UART3_TX_PIN        PB10
 #endif
 #ifndef UART3_RX_PIN
 #define UART3_RX_PIN        PB11
 #endif
 #endif

 #elif defined(STM32F3)

 #ifdef USE_UART1
 #ifndef UART1_TX_PIN
 #define UART1_TX_PIN        PA9
 #endif
 #ifndef UART1_RX_PIN
 #define UART1_RX_PIN        PA10
 #endif
 #endif

 #ifdef USE_UART2
 #ifndef UART2_TX_PIN
 #define UART2_TX_PIN        PD5
 #endif
 #ifndef UART2_RX_PIN
 #define UART2_RX_PIN        PD6
 #endif
 #endif

 #ifdef USE_UART3
 #ifndef UART3_TX_PIN
 #define UART3_TX_PIN        PB10
 #endif
 #ifndef UART3_RX_PIN
 #define UART3_RX_PIN        PB11
 #endif
 #endif

 #ifdef USE_UART4
 #ifndef UART4_TX_PIN
 #define UART4_TX_PIN        PC10
 #endif
 #ifndef UART4_RX_PIN
 #define UART4_RX_PIN        PC11
 #endif
 #endif

 #ifdef USE_UART5
 #ifndef UART5_TX_PIN		// The real UART5_RX is on PD2, no board is using.
 #define UART5_TX_PIN        PC12
 #endif
 #ifndef UART5_RX_PIN
 #define UART5_RX_PIN        PC12
 #endif
 #endif

#endif
