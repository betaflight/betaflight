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

// Forced configuration of two software serials and SERIAL_PORT_COUNT recalc
#ifndef USE_SOFTSERIAL1
#  define USE_SOFTSERIAL1
#endif

#ifndef USE_SOFTSERIAL2
#  define USE_SOFTSERIAL2
#endif

#ifdef USE_VCP
#  define N_VCP 1
#else
#  define N_VCP 0
#endif

#ifdef USE_UART1
  #define N_UART1 1
#else
  #define N_UART1 0
#endif

#ifdef USE_UART2
  #define N_UART2 1
#else
  #define N_UART2 0
#endif

#ifdef USE_UART3
  #define N_UART3 1
#else
  #define N_UART3 0
#endif

#ifdef USE_UART4
  #define N_UART4 1
#else
  #define N_UART4 0
#endif

#ifdef USE_UART5
  #define N_UART5 1
#else
  #define N_UART5 0
#endif

#ifdef USE_UART6
  #define N_UART6 1
#else
  #define N_UART6 0
#endif

#ifdef USE_SOFTSERIAL1
  #define N_SSERIAL1 1
#else
  #define N_SSERIAL1 0
#endif

#ifdef USE_SOFTSERIAL2
  #define N_SSERIAL2 1
#else
  #define N_SSERIAL2 0
#endif

#undef SERIAL_PORT_COUNT
#define SERIAL_PORT_COUNT (N_VCP + N_UART1 + N_UART2 + N_UART3 + N_UART4 + N_UART5 + N_UART6 + N_SSERIAL1 + N_SSERIAL2)
