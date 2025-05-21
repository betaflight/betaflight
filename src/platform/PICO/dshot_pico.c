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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot_command.h"
#include "drivers/motor_types.h"

#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

// Maximum time to wait for telemetry reception to complete
#define DSHOT_TELEMETRY_TIMEOUT 2000

FAST_DATA_ZERO_INIT timeUs_t dshotFrameUs;
//static FAST_DATA_ZERO_INIT timeUs_t lastSendUs;

typedef struct motorOutput_s {
    int offset;
    dshotProtocolControl_t protocolControl;
    int pinIndex;            // pinIndex of this motor output within a group that bbPort points to
    IO_t io;                 // IO_t for this output
    bool configured;
    bool enabled;
    PIO pio;
    uint16_t pio_sm;
} motorOutput_t;

typedef struct picoDshotTelemetryBuffer_s {
    union {
        uint32_t u32[2];
        uint16_t u16[4];
    };
} picoDshotTelemetryBuffer_t;

static motorProtocolTypes_e motorProtocol;
//uint8_t dshotMotorCount = 0;
static motorOutput_t dshotMotors[MAX_SUPPORTED_MOTORS];
bool useDshotTelemetry = false;

#define DSHOT_BIT_PERIOD 40

static const uint16_t dshot_bidir_PIO_instructions[] = {
            //     .wrap_target
    0xff81, //  0: set    pindirs, 1             [31]
    0xff01, //  1: set    pins, 1                [31]
    0x80a0, //  2: pull   block
    0x6050, //  3: out    y, 16
    0x00e6, //  4: jmp    !osre, 6
    0x000e, //  5: jmp    14
    0x6041, //  6: out    y, 1
    0x006b, //  7: jmp    !y, 11
    0xfd00, //  8: set    pins, 0                [29]
    0xe501, //  9: set    pins, 1                [5]
    0x0004, // 10: jmp    4
    0xee00, // 11: set    pins, 0                [14]
    0xf401, // 12: set    pins, 1                [20]
    0x0004, // 13: jmp    4
    0xe055, // 14: set    y, 21
    0x1f8f, // 15: jmp    y--, 15                [31]
    0xe080, // 16: set    pindirs, 0
    0xe05f, // 17: set    y, 31
    0xa142, // 18: nop                           [1]
    0x0076, // 19: jmp    !y, 22
    0x0095, // 20: jmp    y--, 21
    0x00d2, // 21: jmp    pin, 18
    0xe05e, // 22: set    y, 30
    0x4901, // 23: in     pins, 1                [9]
    0x0097, // 24: jmp    y--, 23
    0x4801, // 25: in     pins, 1                [8]
    0x8020, // 26: push   block
    0xe05f, // 27: set    y, 31
    0x4a01, // 28: in     pins, 1                [10]
    0x009c, // 29: jmp    y--, 28
    0x8020, // 30: push   block
    0x0000, // 31: jmp    0
            //     .wrap
};

static float getPeriodTiming(void)
{
    switch(motorProtocol) {
        case MOTOR_PROTOCOL_DSHOT600:
            return 1.67f;
        case MOTOR_PROTOCOL_DSHOT300:
            return 3.33f;
        default:
        case MOTOR_PROTOCOL_DSHOT150:
            return 6.66f;
    }
}

static const struct pio_program dshot_bidir_program = {
    .instructions = dshot_bidir_PIO_instructions,
    .length = ARRAYLEN(dshot_bidir_PIO_instructions),
    .origin = -1,
};

static void dshot_bidir_program_init(PIO pio, uint sm, int offset, uint pin)
{
    pio_sm_config config = pio_get_default_sm_config();
    sm_config_set_wrap(&config, offset + ARRAYLEN(dshot_bidir_PIO_instructions), offset);

    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_in_pins(&config, pin);
    sm_config_set_jmp_pin (&config, pin);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    gpio_set_pulls(pin, true, false);
    sm_config_set_out_shift(&config, false, false, ARRAYLEN(dshot_bidir_PIO_instructions));
    sm_config_set_in_shift(&config, false, false, ARRAYLEN(dshot_bidir_PIO_instructions));

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
    sm_config_set_clkdiv(&config, getPeriodTiming() / DSHOT_BIT_PERIOD * clocks_per_us);

    pio_sm_init(pio, sm, offset, &config);
}

static uint32_t decodeTelemetry(const uint32_t first, const uint32_t second)
{
    UNUSED(first);
    UNUSED(second);
    return DSHOT_TELEMETRY_INVALID;
}

static bool dshotTelemetryWait(void)
{
    // Wait for telemetry reception to complete
    bool telemetryPending;
    bool telemetryWait = false;
    const timeUs_t startTimeUs = micros();

    do {
        telemetryPending = false;
        for (unsigned motorIndex = 0; motorIndex < dshotMotorCount && !telemetryPending; motorIndex++) {
            const motorOutput_t *motor = &dshotMotors[motorIndex];
            const int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
            telemetryPending |= fifo_words >= 2;
        }

        telemetryWait |= telemetryPending;

        if (cmpTimeUs(micros(), startTimeUs) > DSHOT_TELEMETRY_TIMEOUT) {
            break;
        }
    } while (telemetryPending);

    if (telemetryWait) {
        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 2, debug[2] + 1);
    }

    return telemetryWait;
}

static void dshotUpdateInit(void)
{
    // NO OP as no buffer needed for PIO
}

static bool dshotDecodeTelemetry(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        const motorOutput_t *motor = &dshotMotors[motorIndex];

        if (dshotTelemetryState.rawValueState == DSHOT_RAW_VALUE_STATE_NOT_PROCESSED) {
            int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
            if (fifo_words < 2) {
                continue;
            }

            const uint32_t rawOne = pio_sm_get_blocking(motor->pio, motor->pio_sm);
            const uint32_t rawTwo = pio_sm_get_blocking(motor->pio, motor->pio_sm);

            uint32_t rawValue = decodeTelemetry(rawOne, rawTwo);

            DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
            dshotTelemetryState.readCount++;

            if (rawValue != DSHOT_TELEMETRY_INVALID) {
                // Check EDT enable or store raw value
                if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(motorIndex) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                    dshotTelemetryState.motorState[motorIndex].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
                } else {
                    dshotTelemetryState.motorState[motorIndex].rawValue = rawValue;
                }
            } else {
                dshotTelemetryState.invalidPacketCount++;
            }

#ifdef USE_DSHOT_TELEMETRY_STATS
            updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
        }

        dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    }
#endif

    return true;
}

static void dshotWriteInt(uint8_t motorIndex, uint16_t value)
{
    motorOutput_t *const motor = &dshotMotors[motorIndex];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(motorIndex);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    pio_sm_put(motor->pio, motor->pio_sm, packet);
}

static void dshotWrite(uint8_t motorIndex, float value)
{
    dshotWriteInt(motorIndex, lrintf(value));
}

static void dshotUpdateComplete(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotMotorCount)) {
            return;
        }
    }
}

static bool dshotEnableMotors(void)
{
    for (int i = 0; i < dshotMotorCount; i++) {
        const motorOutput_t *motor = &dshotMotors[i];
        if (motor->configured) {
            IOConfigGPIO(motor->io, 0);
        }
    }
    return true;
}

static void dshotDisableMotors(void)
{
    return;
}

static void dshotShutdown(void)
{
    return;
}

static bool dshotIsMotorEnabled(unsigned index)
{
    return dshotMotors[index].enabled;
}

static void dshotPostInit(void)
{
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        dshotMotors[motorIndex].enabled = true;
    }
}

static bool dshotIsMotorIdle(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(dshotMotors)) {
        return false;
    }
    return dshotMotors[motorIndex].protocolControl.value == 0;
}

static void dshotRequestTelemetry(unsigned index)
{
#ifdef USE_DSHOT_TELEMETRY
    if (index < dshotMotorCount) {
        dshotMotors[index].protocolControl.requestTelemetry = true;
    }
#endif
}

static motorVTable_t dshotVTable = {
    .postInit = dshotPostInit,
    .enable = dshotEnableMotors,
    .disable = dshotDisableMotors,
    .isMotorEnabled = dshotIsMotorEnabled,
    .telemetryWait = dshotTelemetryWait,
    .decodeTelemetry = dshotDecodeTelemetry,
    .updateInit = dshotUpdateInit,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = dshotUpdateComplete,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotShutdown,
    .isMotorIdle = dshotIsMotorIdle,
    .requestTelemetry = dshotRequestTelemetry,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    dbgPinLo(0);
    dbgPinLo(1);

    motorProtocol = motorConfig->motorProtocol;
    device->vTable = &dshotVTable;
    dshotMotorCount = device->count;

#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        IO_t io = IOGetByTag(motorConfig->ioTags[motorIndex]);

        const PIO pio = pio0;
        const int pio_sm = pio_claim_unused_sm(pio, false);

        if (!IOIsFreeOrPreinit(io) || pio_sm < 0) {
            /* not enough motors initialised for the mixer or a break in the motors or issue with PIO */
            device->vTable = NULL;
            dshotMotorCount = 0;
            return false;
        }

        int pinIndex = DEFIO_TAG_PIN(motorConfig->ioTags[motorIndex]);

        dshotMotors[motorIndex].pinIndex = pinIndex;
        dshotMotors[motorIndex].io = io;
        dshotMotors[motorIndex].pio = pio;
        dshotMotors[motorIndex].pio_sm = pio_sm;

        uint8_t iocfg = 0;
        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        IOConfigGPIO(io, iocfg);

        int offset = pio_add_program(pio, &dshot_bidir_program);
        if (offset == -1) {
            /* error loading PIO */
            pio_sm_unclaim(pio, pio_sm);
            device->vTable = NULL;
            dshotMotorCount = 0;
            return false;
        } else {
            dshotMotors[motorIndex].offset = offset;
        }

        dshot_bidir_program_init(pio, pio_sm, dshotMotors[motorIndex].offset, pinIndex);
        pio_sm_set_enabled(pio, pio_sm, true);
    }
    return true;
}

// TODO do we just undef USE_DSHOT_BITBANG ?
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
    /* DSHOT BIT BANG not required for PICO */
    UNUSED(motorDevConfig);
    return false;
}

bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // TODO: not required
    UNUSED(device);
    UNUSED(motorConfig);
    return false;
}

dshotBitbangStatus_e dshotBitbangGetStatus(void)
{
    // TODO: not required
    return 0;
}

#endif // USE_DSHOT
