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

#define CMS
#define CMS_MAX_DEVICE 4
#define MAG
#define BARO
#define GPS
#define DISPLAY
#define SERIAL_RX
#define USE_RX_MSP
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_SPEKTRUM   // DSM2 and DSMX protocol
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_SERIALRX_XBUS       // JR
#define TELEMETRY
#define TELEMETRY_CRSF
#define TELEMETRY_FRSKY
#define TELEMETRY_HOTT
#define TELEMETRY_IBUS
#define TELEMETRY_JETIEXBUS
#define TELEMETRY_LTM
#define TELEMETRY_MAVLINK
#define TELEMETRY_SMARTPORT
#define LED_STRIP
#define USE_SERVOS
#define TRANSPONDER
#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8

#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 6

#define TARGET_BOARD_IDENTIFIER "TEST"

#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff

