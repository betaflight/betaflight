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
 *
 * Author: jflyper
 */

// Shared DShot/ProShot timing, DMA buffer and motor output state definitions
// for the PWM-output motor driver.

#pragma once

#include "drivers/dshot.h"
#include "drivers/motor_types.h"
#include "trgm_dshot_resource.h"

// Timer clock frequency for the dshot speeds
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)
// These three constants are times in timer clock ticks, e.g. with a 6 MHz clock
// 20 ticks for bitlength = 300kHz bit rate
#define MOTOR_BIT_0(dutyCount) ((590 * (dutyCount) / 1667) << 4)
#define MOTOR_BIT_1(dutyCount) ((1167 * (dutyCount) / 1667) << 4)
#define MOTOR_BITLENGTH       20

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define PROSHOT_BASE_SYMBOL          24 // 1uS
#define PROSHOT_BIT_WIDTH            3
#define MOTOR_NIBBLE_LENGTH_PROSHOT  (PROSHOT_BASE_SYMBOL * 4)  // 4uS

#define DSHOT_TELEMETRY_DEADTIME_US   (30 + 5)  // 30 to switch lines and 5 to switch lines back

uint32_t getDshotHz(motorProtocolTypes_e pwmProtocolType);

/* Motor DMA related, moved from pwm_output.h */

#define MAX_DMA_TIMERS        8

#define DSHOT_DMA_BUFFER_SIZE   22      /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6       /* resolution + frame reset (2us) */

#define GCR_TELEMETRY_INPUT_LEN MAX_GCR_EDGES

#define DSHOT_DMA_BUFFER_ATTRIBUTE __attribute__((section(".ahb_sram")))  __attribute__((aligned(8)))

#define DSHOT_DMA_BUFFER_UNIT uint32_t

#ifdef USE_DSHOT_TELEMETRY
STATIC_ASSERT(GCR_TELEMETRY_INPUT_LEN >= DSHOT_DMA_BUFFER_SIZE, dshotBufferSizeConstraint);
#define DSHOT_DMA_BUFFER_ALLOC_SIZE GCR_TELEMETRY_INPUT_LEN
#else
#define DSHOT_DMA_BUFFER_ALLOC_SIZE DSHOT_DMA_BUFFER_SIZE
#endif

extern DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

#ifdef USE_DSHOT_TELEMETRY
extern DSHOT_DMA_BUFFER_UNIT dshot_telemetry_pos_buf[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
extern DSHOT_DMA_BUFFER_UNIT dshot_telemetry_neg_buf[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
#endif

extern motorDevice_t motorPwmDevice;

// Per-timer DShot output state: PWM timer, TRGM routing and DMA channel.
typedef struct motorDmaTimer_s {
    void *timer;
#if defined(USE_DSHOT)
    uint16_t outputPeriod;

#endif
    uint16_t timerDmaSources;
    uint8_t trgmIndex;
    dmaResource_t *dmaRef;
    uint32_t dma_ch;
    bool inited;
} motorDmaTimer_t;

// Per-motor DShot output state: protocol config, timer/DMA state and its bound
// TRGM resource.
typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;
    ioTag_t ioTag;
    const timerHardware_t *timerHardware;
    uint16_t timerDmaSource;
    uint8_t timerDmaIndex;
    bool configured;
    uint8_t output;
    uint8_t index;
    uint32_t dshotDutyCount;

#ifdef USE_DSHOT_TELEMETRY
    volatile bool isInput;
    timeDelta_t dshotTelemetryDeadtimeUs;
    uint8_t dmaInputLen;
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer_pos_edge;
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer_neg_edge;
    dma_channel_config_t capPosEdgeDmaConfig;
    dma_channel_config_t capNegEdgeDmaConfig;

#endif                          // USE_DSHOT_TELEMETRY

    dmaResource_t *dmaRef;
    dma_channel_config_t outputDmaConfig;
    trgmDshotResource_t trgmRes;
    motorDmaTimer_t *timer;
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer;
} motorDmaOutput_t;

// Function pointer used to encode a digital motor value into its DMA buffer.
typedef uint8_t loadDmaBufferFn(motorDmaOutput_t *motor, uint16_t packet);
extern FAST_DATA_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
uint8_t loadDmaBufferDshot(motorDmaOutput_t *motor, uint16_t packet);
uint8_t loadDmaBufferProshot(motorDmaOutput_t *motor, uint16_t packet);

motorDmaOutput_t *getMotorDmaOutput(unsigned index);
void pwmDshotStartTransfer(motorDmaOutput_t *motor, uint32_t size);

void pwmWriteDshotInt(uint8_t index, uint16_t value);
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex,
                                 motorProtocolTypes_e pwmProtocolType, uint8_t output);
#ifdef USE_DSHOT_TELEMETRY
bool pwmTelemetryDecode(void);
#endif
void pwmCompleteDshotMotorUpdate(void);
