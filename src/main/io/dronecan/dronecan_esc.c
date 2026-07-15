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

#include "platform.h"

#if ENABLE_DRONECAN_ESC

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/motor_types.h"
#include "drivers/time.h"

#include "flight/mixer.h"

#include "pg/dronecan.h"
#include "pg/motor.h"
#include "pg/pg.h"

#include "rx/rx.h"      // PWM_RANGE_MIN / PWM_RANGE_MAX

#include "sensors/esc_sensor.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_esc.h"
#include "io/dronecan/dronecan_msg.h"

//-----------------------------------------------------------------------------
// Command out: esc.RawCommand
//
// motorWriteAll() runs in the PID-loop task (kHz). Betaflight's scheduler is
// cooperative — the PID task and the dronecan task never interleave — so the
// PID-context updateComplete() can call libcanard directly without racing the
// dronecan task's own libcanard use (the only true concurrency is the CAN ISR,
// which touches driver-level rings only). We therefore emit RawCommand straight
// from updateComplete(), rate-gated to esc_rate_hz, for low and jitter-free
// command latency that does not depend on the LOW-priority task being serviced.
//-----------------------------------------------------------------------------

// Throttle staging, written per-motor by write() and read by the broadcast in
// the same PID iteration — no cross-context sharing, so no seqlock needed.
static int16_t escStaging[MAX_SUPPORTED_MOTORS];

static uint8_t escRawCommandTransferId;

// Rate gate for PID-loop emission. Period is derived from esc_rate_hz at init.
static uint32_t escBroadcastPeriodUs;
static timeUs_t escLastBroadcastUs;

void dronecanEscWrite(uint8_t motorIndex, float value)
{
    if (motorIndex >= MAX_SUPPORTED_MOTORS) {
        return;
    }
    // Endpoints hand us a value already in RawCommand units (0..8191, disarm 0);
    // clamp to the int14 positive range to be safe against transient overshoot.
    escStaging[motorIndex] = (int16_t)constrainf(value, 0, UAVCAN_ESC_RAWCOMMAND_MAX);
}

static void broadcastRawCommand(void)
{
    const uint8_t motorCount = getMotorCount();
    if (motorCount == 0) {
        return;
    }

    // Pack motorCount int14 values, MSB-first per field. TAO: no length prefix.
    // 8 motors * 14 bits = 112 bits = 14 bytes worst case.
    uint8_t payload[(MAX_SUPPORTED_MOTORS * UAVCAN_ESC_RAWCOMMAND_BITS + 7U) / 8U];
    memset(payload, 0, sizeof(payload));

    uint32_t bitOffset = 0;
    for (uint8_t i = 0; i < motorCount; i++) {
        const int16_t v = constrain(escStaging[i], 0, UAVCAN_ESC_RAWCOMMAND_MAX);
        canardEncodeScalar(payload, bitOffset, UAVCAN_ESC_RAWCOMMAND_BITS, &v);
        bitOffset += UAVCAN_ESC_RAWCOMMAND_BITS;
    }
    const uint16_t payloadLen = (uint16_t)((bitOffset + 7U) / 8U);

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeBroadcast;
    tx.data_type_signature = UAVCAN_ESC_RAWCOMMAND_SIGNATURE;
    tx.data_type_id        = UAVCAN_ESC_RAWCOMMAND_ID;
    tx.inout_transfer_id   = &escRawCommandTransferId;
    tx.priority            = CANARD_TRANSFER_PRIORITY_HIGH;
    tx.payload             = payload;
    tx.payload_len         = payloadLen;

    canardBroadcastObj(dronecanGetInstance(), &tx);
}

// Encode the current throttles into a RawCommand transfer and push it to the
// driver immediately. Ungated — callers handle rate limiting / forcing.
static void emitRawCommand(void)
{
    if (!dronecanIsInitialised()) {
        return;
    }
    broadcastRawCommand();
    // Flush straight to the driver so the frame rides out on this PID iteration
    // if the bus is free; the driver's SW ring + TC interrupt absorb the rest.
    dronecanFlushTx();
}

void dronecanEscUpdateComplete(void)
{
    // Only command ESCs when DRONECAN is the active motor protocol; otherwise
    // another protocol owns the motors and we must stay off the bus.
    if (!isMotorProtocolDronecan()) {
        return;
    }

    // Rate-gate to esc_rate_hz. We run every PID loop (kHz) but the bus can
    // only carry frames at the configured rate; emit the instant a period is
    // due rather than waiting on the dronecan task tick.
    const timeUs_t now = micros();
    if (cmpTimeUs(now, escLastBroadcastUs) < (timeDelta_t)escBroadcastPeriodUs) {
        return;
    }
    escLastBroadcastUs = now;

    emitRawCommand();
}

//-----------------------------------------------------------------------------
// Telemetry in: esc.Status -> escSensorData[]
//-----------------------------------------------------------------------------

// esc.Status ingestion lands in esc_sensor.c's store, which only exists on
// USE_ESC_SENSOR targets; RawCommand output above has no such dependency.
#if defined(USE_ESC_SENSOR)

// Last-heard timestamp per ESC index, for aging stale telemetry slots.
static timeUs_t escStatusLastUs[MAX_SUPPORTED_MOTORS];

#define ESC_STATUS_STALE_US     1000000     // 1s without a Status -> age the slot
#define KELVIN_TO_CELSIUS       273.15f

static void handleEscStatus(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    uint32_t errorCount = 0;
    uint16_t voltageF16 = 0;
    uint16_t currentF16 = 0;
    uint16_t tempF16 = 0;
    int32_t rpm = 0;
    uint8_t powerPct = 0;
    uint8_t escIndex = 0;

    canardDecodeScalar(t, ESC_STATUS_OFFSET_ERROR_COUNT, 32, false, &errorCount);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_VOLTAGE,     16, false, &voltageF16);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_CURRENT,     16, false, &currentF16);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_TEMPERATURE, 16, false, &tempF16);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_RPM,         18, true,  &rpm);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_POWER_PCT,    7, false, &powerPct);
    canardDecodeScalar(t, ESC_STATUS_OFFSET_ESC_INDEX,    5, false, &escIndex);

    if (escIndex >= MAX_SUPPORTED_MOTORS) {
        return;
    }

    const float voltage = canardConvertFloat16ToNativeFloat(voltageF16);
    const float current = canardConvertFloat16ToNativeFloat(currentF16);
    const float tempK   = canardConvertFloat16ToNativeFloat(tempF16);
    const float tempC   = tempK - KELVIN_TO_CELSIUS;

    escSensorData_t data;
    memset(&data, 0, sizeof(data));
    data.voltage     = (uint16_t)constrainf(voltage * 100.0f, 0, UINT16_MAX);   // 0.01V
    data.current     = (int32_t)constrainf(current * 100.0f, INT32_MIN, INT32_MAX); // 0.01A
    data.temperature = (uint8_t)constrainf(tempC, 0, UINT8_MAX);                // °C
    // Status.rpm is electrical RPM; escSensorData.rpm is eRPM/100 (LSB = 100),
    // matching the DShot telemetry convention so erpmToRpm() agrees downstream.
    data.rpm         = (int16_t)constrainf(rpm / 100.0f, INT16_MIN, INT16_MAX);
    // consumption (mAh) isn't carried by esc.Status; leave 0 (battery falls
    // back to its own coulomb counter).

    escSensorSetExternal(escIndex, &data);
    escStatusLastUs[escIndex] = micros();
}

#endif // USE_ESC_SENSOR

//-----------------------------------------------------------------------------
// Public surface
//-----------------------------------------------------------------------------

void dronecanEscInit(void)
{
    memset(escStaging, 0, sizeof(escStaging));
    escRawCommandTransferId = 0;

    // Derive the PID-loop emission period from the configured rate. The PG
    // value is already range-checked elsewhere; constrain again defensively.
    const uint16_t rateHz = constrain(dronecanConfig()->esc_rate_hz, 50, 500);
    escBroadcastPeriodUs = 1000000U / rateHz;
    escLastBroadcastUs = micros();

#if defined(USE_ESC_SENSOR)
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escStatusLastUs[i] = 0;
    }

    escSensorExternalInit();

    const dronecanSubscriber_t sub = {
        .signature    = UAVCAN_ESC_STATUS_SIGNATURE,
        .dataTypeId   = UAVCAN_ESC_STATUS_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleEscStatus,
    };
    (void)dronecanRegisterSubscriber(&sub);
#endif
}

void dronecanEscUpdate(timeUs_t currentTimeUs)
{
    // RawCommand is emitted from the PID loop (dronecanEscUpdateComplete) for
    // low, jitter-free latency, so the task only handles telemetry here.

    // Age telemetry slots that have gone quiet so consumers see stale data
    // rather than a frozen last reading.
#if defined(USE_ESC_SENSOR)
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (escStatusLastUs[i] != 0
                && cmpTimeUs(currentTimeUs, escStatusLastUs[i]) >= ESC_STATUS_STALE_US) {
            escSensorExternalAge(i);
            escStatusLastUs[i] = currentTimeUs;
        }
    }
#else
    UNUSED(currentTimeUs);
#endif
}

//-----------------------------------------------------------------------------
// Motor driver (vtable) — bridges drivers/motor.c to the throttle buffer above.
//
// write() stages throttles and updateComplete() encodes + broadcasts RawCommand,
// both in the PID-loop task. The cooperative scheduler guarantees this never
// interleaves with the dronecan task's libcanard use, so libcanard's
// single-context requirement is still met.
//-----------------------------------------------------------------------------

static bool dronecanMotorEnable(void)
{
    return true;
}

static void dronecanMotorDisable(void)
{
    // Park every motor at stop and force a zero RawCommand out now (bypassing
    // the rate gate) so the ESCs see the stop immediately rather than on the
    // next due period.
    memset(escStaging, 0, sizeof(escStaging));
    escLastBroadcastUs = micros();
    emitRawCommand();
}

static bool dronecanMotorIsEnabled(unsigned index)
{
    UNUSED(index);
    return true;
}

static void dronecanMotorShutdown(void)
{
    dronecanMotorDisable();
}

// MSP motor passthrough works in the external [1000,2000] µs convention.
static float dronecanMotorConvertFromExternal(uint16_t externalValue)
{
    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);
    return scaleRangef(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX, 0, UAVCAN_ESC_RAWCOMMAND_MAX);
}

static uint16_t dronecanMotorConvertToExternal(float motorValue)
{
    return (uint16_t)lrintf(scaleRangef(motorValue, 0, UAVCAN_ESC_RAWCOMMAND_MAX, PWM_RANGE_MIN, PWM_RANGE_MAX));
}

static const motorVTable_t dronecanMotorVTable = {
    .enable                 = dronecanMotorEnable,
    .disable                = dronecanMotorDisable,
    .isMotorEnabled         = dronecanMotorIsEnabled,
    .write                  = dronecanEscWrite,
    .updateComplete         = dronecanEscUpdateComplete,
    .convertExternalToMotor = dronecanMotorConvertFromExternal,
    .convertMotorToExternal = dronecanMotorConvertToExternal,
    .shutdown               = dronecanMotorShutdown,
    .requestTelemetry       = NULL,
    .isMotorIdle            = NULL,
    .getMotorIO             = NULL,
};

bool dronecanMotorDevInit(motorDevice_t *device, uint8_t motorCount)
{
    device->vTable = &dronecanMotorVTable;
    device->count = motorCount;
    return true;
}

void dronecanMotorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit,
                                float *outputLow, float *outputHigh, float *disarm,
                                float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    UNUSED(deadbandMotor3dHigh);
    UNUSED(deadbandMotor3dLow);

    // Map the mixer's normalised output onto the int14 throttle range, mirroring
    // the DShot endpoint model. 3D/reverse is not yet wired, so forward-only.
    const float range = UAVCAN_ESC_RAWCOMMAND_MAX;
    const float motorIdlePercent = CONVERT_PARAMETER_TO_PERCENT(motorConfig->motorIdle * 0.01f);

    *disarm = 0;                                            // RawCommand 0 = stop
    *outputLow = motorIdlePercent * range;
    *outputHigh = range * outputLimit;
}

#endif // ENABLE_DRONECAN_ESC
