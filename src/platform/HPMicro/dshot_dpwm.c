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

// DShot motor output device: DShot/ProShot DMA buffer construction,
// motor IO configuration, and the Betaflight motor device interface.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "io_hpmicro.h"
#include "pwm_output_dshot_shared.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/motor_impl.h"

#include "pg/motor.h"

DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

#ifdef USE_DSHOT_TELEMETRY
DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT
dshot_telemetry_pos_buf[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT
dshot_telemetry_neg_buf[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
#endif

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT bool useDshotTelemetry = false;
#endif

FAST_DATA_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
static FAST_DATA_ZERO_INIT struct {
    IO_t io;
    bool enabled;
} dshotPwmMotors[MAX_SUPPORTED_MOTORS];

FAST_CODE_NOINLINE uint8_t loadDmaBufferDshot(motorDmaOutput_t *motor, uint16_t packet)
{
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer = motor->dmaBuffer;
    const uint32_t dutyCount = motor->dshotDutyCount;
    const uint32_t idleValue = dutyCount << 4;
    const uint32_t bitValue0 = MOTOR_BIT_0(dutyCount);
    const uint32_t bitValue1 = MOTOR_BIT_1(dutyCount);

    // Leading pause: 4 bit-periods of idle level.
    for (int i = 0; i < 4; i++) {
        dmaBuffer[i] = idleValue;
    }
    for (int i = 4; i < 20; i++) {
        dmaBuffer[i] = (packet & 0x8000) ? bitValue1 : bitValue0;  // MSB first
        packet <<= 1;
    }
    dmaBuffer[20] = idleValue;
    dmaBuffer[21] = idleValue;

    return DSHOT_DMA_BUFFER_SIZE;
}

FAST_CODE_NOINLINE uint8_t loadDmaBufferProshot(motorDmaOutput_t *motor, uint16_t packet)
{
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer = motor->dmaBuffer;

    for (int i = 0; i < 4; i++) {
        dmaBuffer[i] = PROSHOT_BASE_SYMBOL + ((packet & 0xF000) >> 12) *
                       PROSHOT_BIT_WIDTH;    // Most significant nibble first
        packet <<= 4;           // Shift 4 bits
    }
    dmaBuffer[4] = 0;
    dmaBuffer[5] = 0;

    return PROSHOT_DMA_BUFFER_SIZE;
}

uint32_t getDshotHz(motorProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
    case (MOTOR_PROTOCOL_PROSHOT1000):
        return MOTOR_PROSHOT1000_HZ;
    case (MOTOR_PROTOCOL_DSHOT600):
        return MOTOR_DSHOT600_HZ;
    case (MOTOR_PROTOCOL_DSHOT300):
        return MOTOR_DSHOT300_HZ;
    default:
    case (MOTOR_PROTOCOL_DSHOT150):
        return MOTOR_DSHOT150_HZ;
    }
}

static void dshotPwmShutdown(void)
{
    // DShot signal is only generated if write to motors happen,
    // and that is prevented by enabled checking at write.
    // Hence there's no special processing required here.
    return;
}

static void dshotPwmDisableMotors(void)
{
    // No special processing required
    return;
}

static bool dshotPwmEnableMotors(void)
{
    for (int i = 0; i < dshotMotorCount; i++) {
        motorDmaOutput_t *motor = getMotorDmaOutput(i);
        if (motor == NULL) {
            return false;
        }
        const IO_t motorIO = IOGetByTag(motor->timerHardware->tag);
        IOConfigGPIOAF(motorIO, IOCFG_AF_PP_UP, motor->timerHardware->alternateFunction);
    }

    // No special processing required
    return true;
}

static bool dshotPwmIsMotorEnabled(unsigned index)
{
    return index < dshotMotorCount && dshotPwmMotors[index].enabled;
}

static IO_t pwmDshotGetMotorIO(unsigned index)
{
    if (index >= dshotMotorCount) {
        return IO_NONE;
    }
    return dshotPwmMotors[index].io;
}

static FAST_CODE void dshotWriteInt(uint8_t index, uint16_t value)
{
    pwmWriteDshotInt(index, value);
}

static FAST_CODE void dshotWrite(uint8_t index, float value)
{
    pwmWriteDshotInt(index, lrintf(value));
}

static const motorVTable_t dshotPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = dshotPwmEnableMotors,
    .disable = dshotPwmDisableMotors,
    .isMotorEnabled = dshotPwmIsMotorEnabled,
    .decodeTelemetry = pwmTelemetryDecode,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = pwmCompleteDshotMotorUpdate,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotPwmShutdown,
    .requestTelemetry = pwmDshotRequestTelemetry,
    .isMotorIdle = pwmDshotIsMotorIdle,
    .getMotorIO = pwmDshotGetMotorIO,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    device->vTable = &dshotPwmVTable;
    dshotMotorCount = device->count;
#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif

    /* Roll back resources held by any earlier init attempt so the allocations
     * below start from a clean slate without requiring a reboot.
     */
    pwmDshotReleaseAllMotors();

    switch (motorConfig->motorProtocol) {
    case MOTOR_PROTOCOL_PROSHOT1000:
        loadDmaBuffer = loadDmaBufferProshot;
        break;
    case MOTOR_PROTOCOL_DSHOT600:
    case MOTOR_PROTOCOL_DSHOT300:
    case MOTOR_PROTOCOL_DSHOT150:
        loadDmaBuffer = loadDmaBufferDshot;
        break;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        if (timerHardware != NULL) {
            uint8_t output = timerHardware->output;

            dshotPwmMotors[motorIndex].io = IOGetByTag(tag);
            IOInit(dshotPwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

            if (motorConfig->motorInversion) {
                output ^= TIMER_OUTPUT_INVERTED;
            }

            if (pwmDshotMotorHardwareConfig(timerHardware,
                                            motorIndex, reorderedMotorIndex, motorConfig->motorProtocol, output)) {
                dshotPwmMotors[motorIndex].enabled = true;

                continue;
            }
        }

        /* not enough motors initialised for the mixer or a break in the motors */
        pwmDshotReleaseAllMotors();
        dshotMotorCount = 0;
        /* TODO: block arming and add reason system cannot arm */
        return false;
    }

    return true;
}

#endif                          // USE_DSHOT
