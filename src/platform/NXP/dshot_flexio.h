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

#include <stdbool.h>
#include <stdint.h>
#if !defined(__ARM_FP)
#include <math.h>
#endif

// lrintf() without the call into XIP flash; vcvtr rounds by the FPSCR mode,
// round to nearest, ties to even.
static inline int32_t dshotFlexioLrintf(float value)
{
#if defined(__ARM_FP)
    int32_t result;
    __asm__ ("vcvtr.s32.f32 %0, %1" : "=t" (result) : "t" (value));
    return result;
#else
    return (int32_t)lrintf(value);
#endif
}

// Motor outputs one FlexIO instance can drive, one shifter and timer each.
#define FLEXIO_DSHOT_CHANNELS 8

// The turnaround to receive must finish within about 30 us of the last bit at
// DSHOT600, so only preemption group 0 (gyro EXTI and SPI) may preempt it; the
// ATOMIC_BLOCK sites mask at this level too, the FlexIO UART's included.
#define DSHOT_FLEXIO_PRIORITY NVIC_BUILD_PRIORITY(1, 0)

// A good reply decodes to 12 bits, so any larger value can mean failure.
#define DSHOT_FLEXIO_DECODE_INVALID 0xffffffffu

// Timer compare values for the shifter and timer pair, dual 8 bit baud mode.
uint32_t dshotFlexioTxTimerCompare(uint32_t flexioClockHz, uint32_t bitRateHz);
uint32_t dshotFlexioRxTimerCompare(uint32_t flexioClockHz, uint32_t bitRateHz, int offset);

// Whether the receive divider plus every swept offset fits in eight bits.
bool dshotFlexioRxDividerFits(uint32_t flexioClockHz, uint32_t bitRateHz);

// Expand a 16 bit DShot frame into the 48 bit shifter pattern, three shift
// clock ticks per DShot bit.
uint64_t dshotFlexioExpandFrame(uint16_t packet);

// Decode one word read back from the receive shifter into the 12 bit telemetry
// value, or DSHOT_FLEXIO_DECODE_INVALID if the frame or the checksum is bad.
uint32_t dshotFlexioDecodeTelemetry(uint32_t shifterWord);

// Receive baud rate search: the compare low byte is swept over these offsets
// and the middle of the range that decoded is kept.
#define DSHOT_FLEXIO_TCMP_OFFSET_MIN (-16)
#define DSHOT_FLEXIO_TCMP_OFFSET_MAX (15)
// An offset passes when 24 of 25 frames decode.
#define DSHOT_FLEXIO_TRAIN_FRAMES    25
#define DSHOT_FLEXIO_TRAIN_PASSES    24

typedef struct dshotFlexioTraining_s {
    uint32_t passMask;  // bit per offset already tried, set if that offset worked
    uint8_t frames;     // frames seen so far at the offset being tried
    uint8_t passes;     // of those, how many decoded
    int8_t offset;      // offset being tried, and the chosen one once done
    bool done;
} dshotFlexioTraining_t;

// Start a search; the caller programs the receive compare from training->offset.
void dshotFlexioTrainReset(dshotFlexioTraining_t *training);

// Feed one received word; returns true when the caller must reprogram the
// receive compare from training->offset.
bool dshotFlexioTrainStep(dshotFlexioTraining_t *training, uint32_t shifterWord);

#if SERIAL_PIOUART_COUNT > 0
#include "drivers/serial.h"

// The PIOUART ports, a UART on FlexIO shifters and timers, in serial_flexio_nxp.c.
// DShot holds shifter n and timer n for motor n; a port takes the highest free ones.

// The shifters, and so the timers, DShot holds on an instance.
uint32_t dshotFlexioChannelMask(const FLEXIO_Type *base);

// Called from the instance interrupt; it handles only the ports' own shifter flags.
void flexioUartIrqHandler(FLEXIO_Type *base);

// The serial_uart_nxp.c entry points for a PIOUART port.
struct uartDevice_s;
struct uartPort_s;
void flexioUartPinConfigure(const serialPinConfig_t *pSerialPinConfig);
struct uartPort_s *flexioUartOpen(struct uartDevice_s *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options);
void flexioUartReconfigure(struct uartPort_s *s);
void flexioUartEnableTxInterrupt(struct uartPort_s *s);
#endif
