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

#include "dshot_pico.h"
#include "common/maths.h"

#ifdef DSHOT_DEBUG_PIO
#include "drivers/io_impl.h"

#define DSHOT_DEBUG_SIDE(index)               \
            (index == 0 ? DSHOT_DEBUG_SIDE0 : \
             index == 1 ? DSHOT_DEBUG_SIDE1 : \
             index == 2 ? DSHOT_DEBUG_SIDE2 : \
             DSHOT_DEBUG_SIDE3)
#endif

static int32_t outgoingPacket[MAX_SUPPORTED_MOTORS]; // 16-bit packet or -1 for none pending.
const PIO dshotPio = PIO_INSTANCE(PIO_DSHOT_INDEX); // currently only single pio supported => 4 motors.

motorProtocolTypes_e dshotMotorProtocol;
motorOutput_t dshotMotors[MAX_SUPPORTED_MOTORS];

bool useDshotTelemetry = false;

float dshotGetPeriodTiming(void)
{
    switch(dshotMotorProtocol) {
        case MOTOR_PROTOCOL_DSHOT600:
            return 1.666667f;
        case MOTOR_PROTOCOL_DSHOT300:
            return 3.333333f;
        default:
        case MOTOR_PROTOCOL_DSHOT150:
            return 6.666667f;
    }
}

#define DSHOT_BIT_PERIOD 40

// TODO DSHOT150, 300
static bool dshot_program_init(PIO pio, uint sm, int offset, uint pin)
{
    bprintf("dshot_program_init on pin %d", pin);
    pio_sm_config config = dshot_600_program_get_default_config(offset);

    sm_config_set_set_pins(&config, pin, 1);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output
    gpio_set_pulls(pin, false, true); // Pull down - idle low when awaiting next frame to output.

    sm_config_set_out_shift(&config, PIO_SHIFT_LEFT, PIO_NO_AUTO_PUSHPULL, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + dshotGetPeriodTiming()) / DSHOT_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, dshotGetPeriodTiming() / DSHOT_BIT_PERIOD * clocks_per_us);
#endif

    return PICO_OK == pio_sm_init(pio, sm, offset, &config);
}


static void dshotUpdateInit(void)
{
    for (int motorIndex = 0; motorIndex < dshotMotorCount; ++motorIndex) {
        outgoingPacket[motorIndex] = -1;
    }
}

// Prepare packet for sending on .updateComplete (dshotUpdateComplete)
static void dshotWriteInt(uint8_t motorIndex, uint16_t value)
{
    motorOutput_t *const motor = &dshotMotors[motorIndex];
    if (!motor->configured) {
        bprintf("dshotWriteInt motor %d not configured", motorIndex);
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(motorIndex);
#ifdef PICO_TRACE
        // testing
        static uint16_t lastvalue = 12345u;
        if (value != lastvalue) {
            bprintf("dshotWriteInt command number %d (%x)", value, value);
            lastvalue = value;
        }
#endif
        if (value) {
            // Request telemetry on other return wire (this isn't DSHOT bidir)
            // [TODO check, on all commands??? cf. pwm_output_dshot_shared.c]
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;
    outgoingPacket[motorIndex] = prepareDshotPacket(&motor->protocolControl);
}

static void dshotWrite(uint8_t motorIndex, float value)
{
    dshotWriteInt(motorIndex, lrintf(value));
}

static void dshotUpdateComplete(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotMotorCount)) { // Are we ok to proceed? (This function affects the dshot command queue)
            return;
        }
    }

    uint32_t motorMask = 0;
    for (int motorIndex = 0; motorIndex < dshotMotorCount; ++motorIndex) {
        if (outgoingPacket[motorIndex] >= 0) {
            motorMask |= 1 << dshotMotors[motorIndex].pio_sm;
        }
    }

    // stop SMs before sending data to SM, for simultaneous restart
    pio_set_sm_mask_enabled(dshotPio, motorMask, false);

    if (useDshotTelemetry) {
        // protect against SM getting stuck waiting for telemetry.
        // TODO we could be more sophisticated about detecting that we are not stuck,
        // then skipping the restart, to avoid unnecessary delays in the PIO code.
        pio_restart_sm_mask(dshotPio, motorMask);
        for (int motorIndex = 0; motorIndex < dshotMotorCount; ++motorIndex) {
            if (outgoingPacket[motorIndex] >= 0) {
                const motorOutput_t *motor = &dshotMotors[motorIndex];
                pio_sm_clear_fifos(motor->pio, motor->pio_sm);
                pio_sm_exec_wait_blocking(motor->pio, motor->pio_sm, pio_encode_jmp(motor->offset));
            }
        }
    }

    for (int motorIndex = 0; motorIndex < dshotMotorCount; ++motorIndex) {
        if (outgoingPacket[motorIndex] >= 0) {
            const motorOutput_t *motor = &dshotMotors[motorIndex];
            pio_sm_put(motor->pio, motor->pio_sm, outgoingPacket[motorIndex]);
        }
    }

    // Send simultaneously.
    pio_set_sm_mask_enabled(dshotPio, motorMask, true);
}

static bool dshotEnableMotors(void)
{
    bprintf("pico dshotEnableMotors (useDshotTelemetry = %d)", useDshotTelemetry);
    // No special processing required
    return true;
}

static void dshotDisableMotors(void)
{
    bprintf("pico dshotDisableMotors");
    // No special processing required
    return;
}

static void dshotShutdown(void)
{
    // TODO: implement?
    bprintf("pico dshotShutdown");
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
    if (useDshotTelemetry) {
        if (index < dshotMotorCount) {
            dshotMotors[index].protocolControl.requestTelemetry = true;
        }
    }
}

static motorVTable_t dshotVTable = {
    .postInit = dshotPostInit,
    .enable = dshotEnableMotors,
    .disable = dshotDisableMotors,
    .isMotorEnabled = dshotIsMotorEnabled,
#ifdef USE_DSHOT_TELEMETRY
    .telemetryWait = dshotTelemetryWait,
    .decodeTelemetry = dshotDecodeTelemetry,
#else
    .telemetryWait = NULL,
    .decodeTelemetry = NULL,
#endif
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
    // Return false if not enough motors initialised for the mixer or a break in the motors or issue with PIO
    dbgPinLo(0);
    dbgPinLo(1);

    device->vTable = NULL; // Only set vTable if initialisation is succesful (TODO: check)
    dshotMotorCount = 0;   // Only set dshotMotorCount ble if initialisation is succesful (TODO: check)

    uint8_t motorCountProvisional = device->count;
    if (motorCountProvisional > 4) {
        // Currently support 4 motors with one PIO block, four state machines
        // TODO (possible future) support more than 4 motors
        // (more than one pio? by reconfiguring state machines perhaps? batching + dma if required)
        bprintf("*** dshot Pico %d motors unsupported", motorCountProvisional);
        return false;
    }

    dshotMotorProtocol = motorConfig->motorProtocol;
    if (dshotMotorProtocol != MOTOR_PROTOCOL_DSHOT600) {
        bprintf("\n*** DSHOT motor protocol [%d] not currently supported", dshotMotorProtocol);
        return false;
        // TODO support DSHOT300, DSHOT150
    }

#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry; // as per config set dshotBidir ON/OFF, or from Bidirectional Dshot toggle on motor page in Configurator.
#endif

    int pinIndexMin = 48;
    int pinIndexMax = -1;
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCountProvisional; motorIndex++) {
        int pinIndex = DEFIO_TAG_PIN(motorConfig->ioTags[motorIndex]);
        pinIndexMin = pinIndex < pinIndexMin ? pinIndex : pinIndexMin;
        pinIndexMax = pinIndex > pinIndexMax ? pinIndex : pinIndexMax;

#ifdef DSHOT_DEBUG_PIO
        pinIndexMin = MIN(pinIndexMin, DSHOT_DEBUG_SIDE(motorIndex));
        pinIndexMax = MAX(pinIndexMax, DSHOT_DEBUG_SIDE(motorIndex));
#endif
    }

    int pioBase = 0;
    if (pinIndexMax >= 32) {
        if (pinIndexMin < 16) {
            bprintf("* dshot PIO can't span motor pins min %d max %d", pinIndexMin, pinIndexMax);
            return false;
        } else {
            pioBase = 16;
        }
    }

    bprintf("dshot pio%d pin min, max = %d, %d; setting gpio base to %d", PIO_NUM(dshotPio), pinIndexMin, pinIndexMax, pioBase);
    // The GPIO base must be set before adding the program.
    pio_set_gpio_base(dshotPio, pioBase);

    // Use one program for all motors.
    // NB the PIO block is limited to 32 instructions (shared across 4 state machines)
    int offset;
    if (useDshotTelemetry) {
#ifdef DSHOT_DEBUG_PIO
        offset = pio_add_program(dshotPio, &dshot_600_bidir_debug_program);
#else
        offset = pio_add_program(dshotPio, &dshot_600_bidir_program);
#endif
    } else {
        offset = pio_add_program(dshotPio, &dshot_600_program);
    }

    bprintf("pio added program with useDshotTelemetry %d", useDshotTelemetry);
    if (offset < 0) {
        /* error loading PIO */
        bprintf("*** dshot pio failed to add program [useDshotTelemetry = %d]", useDshotTelemetry);
        return false;
    }

#ifdef DSHOT_DEBUG_PIO
    for (int index = 0; index < 4; ++index) {
        unsigned int sidePin = DSHOT_DEBUG_SIDE(index);
        ioRecs[sidePin].owner = OWNER_SYSTEM;
        pio_gpio_init(dshotPio, sidePin);
        bprintf("dshot init sideset pin %d", sidePin);
    }
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCountProvisional; motorIndex++) {
        outgoingPacket[motorIndex] = -1;
        int pinIndex = DEFIO_TAG_PIN(motorConfig->ioTags[motorIndex]);
        IO_t io = IOGetByTag(motorConfig->ioTags[motorIndex]);
        bprintf("dshot motor index %d on pin %d",motorIndex, IO_Pin(io));
        if (!IOIsFreeOrPreinit(io)) {
            bprintf("io pin not free");
            return false;
        }

        // TODO: might make use of pio_claim_free_sm_and_add_program_for_gpio_range
        // -> automatically sets the GPIO base if we might be using pins in 32..47
        const int pio_sm = pio_claim_unused_sm(dshotPio, false);

        if (pio_sm < 0) {
            bprintf("\n *** dshotPwmDevInit: failed to claim state machine\n");
            return false;
        }

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        // TODO: take account of motor reordering,
        // cf. versions of  pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
        dshotMotors[motorIndex].pinIndex = pinIndex;
        dshotMotors[motorIndex].io = io;
        dshotMotors[motorIndex].pio = dshotPio;
        dshotMotors[motorIndex].pio_sm = pio_sm;
        dshotMotors[motorIndex].offset = offset;

        bool dshotInit;
        if (useDshotTelemetry) {
            dshotInit = dshot_program_bidir_init(dshotPio, pio_sm, dshotMotors[motorIndex].offset, pinIndex);
#ifdef DSHOT_DEBUG_PIO
            int sidesetpin = DSHOT_DEBUG_SIDE(motorIndex);
            pio_sm_set_consecutive_pindirs(dshotPio, pio_sm, sidesetpin, 1, true); // set pin to output
            pio_sm_set_sideset_pins(dshotPio, pio_sm, sidesetpin);
#endif
        } else {
            dshotInit = dshot_program_init(dshotPio, pio_sm, dshotMotors[motorIndex].offset, pinIndex);
        }

        if (!dshotInit) {
            bprintf("dshot failed to init pio program for motor index %d, pin %d, useDshotTelemetry %d", motorIndex, pinIndex, useDshotTelemetry);
            return false;
        }

        dshotMotors[motorIndex].configured = true;
    }

    device->vTable = &dshotVTable;
    bprintf("pico dshot: Set device %p vtable (at %p) to pico dshotvtable %p", device, &device->vTable, device->vTable);
    dshotMotorCount = motorCountProvisional;
    return true;
}

#endif // USE_DSHOT
