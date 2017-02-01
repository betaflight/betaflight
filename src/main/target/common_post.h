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

// Touch up configuration

#pragma once

// Targets with built-in vtx do not need external vtx
#if defined(VTX) || defined(USE_RTC6705)
# undef VTX_CONTROL
# undef VTX_COMMON
# undef VTX_SMARTAUDIO
# undef VTX_TRAMP
#endif

#if defined(USE_SOFTSERIAL1) && defined(USE_SOFTSERIAL2)
# define SOFTSERIAL_COUNT 2
#elif defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
# define SOFTSERIAL_COUNT 1
#else
# define SOFTSERIAL_COUNT 0
#endif

// Backward compatibility with F1 targets not specifying UART1 & UART2 pins

#ifdef STM32F1
# ifdef USE_UART1
#  ifndef UART1_RX_PIN
#   define UART1_RX_PIN PA10
#  endif
#  ifndef UART1_TX_PIN
#   define UART1_TX_PIN PA9
#  endif
# endif

# ifdef USE_UART2
#  ifndef UART2_RX_PIN
#   define UART2_RX_PIN PA3
#  endif
#  ifndef UART2_TX_PIN
#   define UART2_TX_PIN PA2
#  endif
# endif
#endif // STM32F1
