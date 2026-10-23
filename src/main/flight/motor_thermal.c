/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include "platform.h"

#ifdef USE_MOTOR_THERMAL

#include "build/debug.h"
#include "common/maths.h"
#include "config/feature.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/motor_noise.h"
#include "flight/position.h"
#include "io/gps.h"
#include "pg/motor.h"
#include "pg/motor_thermal.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "motor_thermal.h"

// Bench reference: X-NOVA 2203.5 2800KV, no prop, two-node fit on random + flight-like regimes
// (research/motor-thermal, RMSE ~1 °C). The fitted model is scaled as a whole (P·s, C·s, R/s) so that
// the ripple copper loss equals the measured excess bus power (2.0 W at 35 rms command noise, 16.2 V). Units: J/K, K/W, W/krpm², W/(rpm·duty)², Ohm, m/s.
// Size and magnet scaling checked on GEPRC 1408 3500KV and XING 1204 4500KV (12 magnets): RMSE 1.1-1.2 °C, the reference fit unchanged.
#define REF_D_MM        22.0f
#define REF_H_MM        3.5f
#define REF_KV          2800.0f
#define REF_POLES       14.0f
#define HEAT_CAP_H_MM   12.0f   // heat capacity ∝ D²·(H + this): the parts that do not grow with the stator height
#define REF_CS          2.88f   // windings heat capacity, J/K
#define REF_CC          21.9f   // stator core heat capacity, J/K
#define REF_RSC         3.63f   // windings -> core, K/W
#define REF_RCA         11.3f   // core -> air, still air, K/W
#define REF_A_IRON      0.0449f   // iron loss per (rpm/1000)², W
#define REF_K_CU        7.94e-07f   // ripple copper loss per (KV·Vbat·duty_rms)², W
#define REF_R_LL        0.070f     // line-to-line winding resistance, datasheet (load-current copper loss)
#define AIR_GAIN        1.56f   // convection: 1 + AIR_GAIN·(v/10 m/s)^1.5
#define PROP_VI_K       0.252f     // induced velocity = n[rev/s]·D[m]·sqrt(2·Ct/π), Ct≈0.1
// Prop wash over the motor: forced convection on the windings (end turns in the wash), Newton cooling with the
// turbulent h ∝ v^0.8. The core already sees the prop's induced flow through airFactor(), so the wash is not added
// there again. The bench motor had no prop, so the bench fit is unchanged. Checked on 81 published BrotherHobby
// winding temperatures at full throttle (research/motor-thermal/DATASHEET_VALIDATION.md): model/datasheet rise
// median 0.99, IQR 0.77-1.33 (3.25 without this term), and on a 5" flight against a thermistor on a winding.
#define WASH_REF_MS     1.0f       // wash speed at which the extra cooling equals the still-air core cooling
#define WASH_N          0.8f
#define WASH_MAX        100.0f     // keeps the 10 ms explicit step stable on the smallest motors
#define UPDATE_HZ       100
#define EDT_TRIES       10         // ESCs that ignore the extended telemetry request are left alone after this
#define IDLE_AMPS_TAU_S 2.0f       // the non-motor current (FC, VTX, receiver), averaged while disarmed
#define AMBIENT_MIN_C   (-30)
#define AMBIENT_MAX_C   45
#define AMBIENT_DEFAULT_C 25.0f

typedef struct { float ts, tc; } thermalNode_t;

static thermalNode_t node[MAX_SUPPORTED_MOTORS];
static struct {
    float invCs, invCc;         // inverse windings and core heat capacities, K/J
    float rsc, rca, rLoad, tAmb;
    float aIron;                // iron loss per rpm², W
    float kCu;                  // ripple copper loss per (Vbat·command noise)², W
    float kv;                   // effective KV
    float airK;                 // squared air speed at the stator from the bell and the prop per rpm², (m/s)²
    float washK;                // prop wash speed per rpm, m/s
    timeUs_t lastUs;
    float idleAmps;             // battery current with the motors stopped: not motor current
    int ambientCounter;
    int boardLowest;            // lowest FC-board reading (gyro, baro, MCU) since power-up, °C
    int escLowest;              // lowest ESC reading since power-up, °C
    uint8_t edtTries;
    bool armedOnce;
} rt;

static void ambientConsider(int t, int *best)
{
    // 0 is what a sensor holds before its first reading; a real 0 °C is skipped too, the other sensors
    // on the craft read close to it
    if (t != 0 && t > AMBIENT_MIN_C && t < AMBIENT_MAX_C && t < *best) {
        *best = t;
    }
}

#ifdef USE_DSHOT_TELEMETRY
// Bidirectional DShot ESCs report only eRPM until asked for the extended telemetry (temperature, ...), which the
// core asks for at arming. A cold ESC is the best ambient reference, so with dshot_edt on ask while disarmed:
// only with every motor stopped (no disarmed motor test or excitation running) and nothing else in the queue.
static void edtRequest(void)
{
    if (!motorConfig()->dev.useDshotEdt || rt.edtTries >= EDT_TRIES || !isDshotTelemetryActive() || !dshotCommandQueueEmpty()) {
        return;
    }
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return;
    }
#endif
    bool missing = false;
    for (int i = 0; i < getMotorCount(); i++) {
        if (!motorIsMotorIdle(i)) {
            return;
        }
        missing |= !(dshotTelemetryState.motorState[i].telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK);
    }
    if (missing) {
        rt.edtTries++;
        dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, DSHOT_CMD_TYPE_INLINE);
    }
}
#endif

// The ambient before the first arm: the lowest plausible reading of the ESCs (DShot extended telemetry or ESC sensor;
// a cold ESC was within ~1 °C of a thermistor on a 5") and of the FC board's sensors (gyro, baro, MCU). Each source
// keeps its lowest reading since power-up: the board warms up afterwards (+9 °C over the air after an hour on USB),
// so its first readings are the closest to the air. Looked at once a second while disarmed, so a source that comes
// later (ESC telemetry once the battery is in) still counts; frozen at the first arm. Without any, the default stays.
static void ambientUpdate(void)
{
    if (rt.armedOnce || ++rt.ambientCounter < UPDATE_HZ) {
        return;
    }
    rt.ambientCounter = 0;
#ifdef USE_DSHOT_TELEMETRY
    edtRequest();
#endif
    int board = INT16_MAX;
    gyroReadTemperature();                      // the cached value is otherwise refreshed by the telemetry task only
    ambientConsider(gyroGetTemperature(), &board);
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        ambientConsider(baro.temperature / 100, &board);
    }
#endif
#ifdef USE_ADC_INTERNAL
    ambientConsider(getCoreTemperatureCelsius(), &board);
#endif
    int escTemp = INT16_MAX;
    for (int i = 0; i < getMotorCount() && i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_DSHOT_TELEMETRY
        if (dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) {
            ambientConsider(dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE], &escTemp);
        }
#endif
#ifdef USE_ESC_SENSOR
        const escSensorData_t *esc = getEscSensorData(i);
        if (esc && esc->dataAge < ESC_DATA_INVALID) {
            ambientConsider(esc->temperature, &escTemp);
        }
#endif
    }
    rt.boardLowest = MIN(rt.boardLowest, board);
    rt.escLowest = MIN(rt.escLowest, escTemp);
    const int ambient = MIN(rt.escLowest, rt.boardLowest);
    if (ambient != INT16_MAX) {
        // shift by the change, not overwrite: a disarmed motor test, excitation or frame wizard run has heated them
        const float delta = ambient - rt.tAmb;
        rt.tAmb = ambient;
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            node[i].ts += delta;
            node[i].tc += delta;
        }
    }
}

// motor_prop_diameter 0 (auto): the usual prop for the stator diameter
static const struct {
    uint8_t maxStatorMm;
    uint8_t propDinch;
} autoProp[] = {
    { 9, 16 }, { 12, 25 }, { 15, 30 }, { 19, 35 }, { 22, 45 }, { 23, 50 }, { 27, 55 }, { 30, 70 }, { UINT8_MAX, 80 },
};

// convection: 1 + AIR_GAIN·(v/10 m/s)^1.5, from the squared air speed at the stator
static float airFactor(float airSpeedSq)
{
    const float v = sqrtf(airSpeedSq) * 0.1f;
    return 1.0f + AIR_GAIN * v * sqrtf(v);
}

// Cooling to the air at this rpm, as multiples of the still-air conductance 1/rca: the core through the airflow over
// the stator (bell, prop induced flow, craft speed), the windings through the prop wash over the end turns. Shared by
// the time step and the steady-state ripple rise, so the two cannot drift apart.
static float coreCooling(float rpm, float vCraftSq)
{
    return airFactor(sq(rpm) * rt.airK + vCraftSq);
}

static float windingCooling(float rpm)
{
    return MIN(powf(MAX(rpm, 0.0f) * rt.washK / WASH_REF_MS, WASH_N), WASH_MAX);
}

void motorThermalInit(void)
{
    const motorThermalConfig_t *c = motorThermalConfig();
    const float d = MAX(c->stator_diameter_mm, 5), h = MAX(c->stator_height_dmm, 10) * 0.1f;
    // Size laws fitted on 2203.5, 1408 and 1204 motors (research/motor-thermal/scaling_fit.py); all ratios are 1 at the reference
    const float dr = d / REF_D_MM, hr = h / REF_H_MM;
    const float area = (d * h + 0.5f * d * d) / (REF_D_MM * REF_H_MM + 0.5f * REF_D_MM * REF_D_MM);
    const float heatCap = sq(dr) * (h + HEAT_CAP_H_MM) / (REF_H_MM + HEAT_CAP_H_MM);          // stator, bell, magnets and bearings
    rt.invCs = 1.0f / (REF_CS * heatCap);
    rt.invCc = 1.0f / (REF_CC * heatCap);
    rt.rsc = REF_RSC / sqrtf(dr * hr);
    rt.rca = REF_RCA / sqrtf(area);
    const float gain = c->gain_percent * 0.01f;
    rt.kv = MAX(motorConfig()->kv, 100);                                                      // a zero/absurd setting must not silence the model
    // no-load loss at the same rpm barely depends on the size; ∝ electrical frequency through the magnet count
    rt.aIron = REF_A_IRON * 1e-6f * powf(dr, 0.15f) * sqrtf(sqrtf(hr)) * (MAX(motorConfig()->motorPoleCount, 2) / REF_POLES);
    const float rScale = hr * sq(dr) * sqrtf(dr);                                             // H·D^2.5, fitted on the bench motors
    rt.kCu = REF_K_CU * rScale * gain * sq(rt.kv * 0.001f);                                   // ripple loss, noise -> duty
    // measured, or R ∝ 1/(KV²·H^1.29·D^3.95): exponents from 772 motor datasheets, level from the 2203.5 reference,
    // which matches 81 published winding temperatures (rise ratio 0.95)
    rt.rLoad = c->resistance_mohm ? c->resistance_mohm * 0.001f * gain
                                  : REF_R_LL * sq(REF_KV / rt.kv) / (powf(hr, 1.29f) * powf(dr, 3.95f)) * gain;
    unsigned p = 0;
    while (d > autoProp[p].maxStatorMm) {
        p++;
    }
    const int propDinch = c->prop_diameter_dinch ? c->prop_diameter_dinch : autoProp[p].propDinch;
    const float bellRadiusM = (0.5f * d + 3.0f) * 0.001f, propDiameterM = propDinch * 0.1f * 0.0254f;
    rt.airK = (sq(2.0f * M_PIf * bellRadiusM) + sq(propDiameterM * PROP_VI_K)) / sq(60.0f);  // bell surface and prop induced speed
    rt.washK = propDiameterM * PROP_VI_K / 60.0f;

    // Parameters are recomputed on every (disarmed) profile change, the thermal state only once at boot:
    // a hot motor must not read as ambient after a profile switch on the ground.
    static bool stateInitialised = false;
    if (!stateInitialised) {
        rt.tAmb = AMBIENT_DEFAULT_C;
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            node[i].ts = node[i].tc = rt.tAmb;
        }
        rt.idleAmps = 0.0f;
        rt.ambientCounter = UPDATE_HZ;          // the first update looks at once
        rt.boardLowest = INT16_MAX;
        rt.escLowest = INT16_MAX;
        rt.armedOnce = false;
        stateInitialised = true;
    }
}

// Scheduler task at UPDATE_HZ, fed with the mixer's final motor outputs, so that every output path (normal
// mix, motor stop, crash flip) is covered. The model integrates over the real interval: a late run stays right.
void motorThermalUpdate(timeUs_t currentTimeUs)
{
    const float dt = rt.lastUs ? constrainf(cmpTimeUs(currentTimeUs, rt.lastUs) * 1e-6f, 0.0f, 0.1f) : 1.0f / UPDATE_HZ;
    rt.lastUs = currentTimeUs;
    const int motorCount = getMotorCount();

    const bool armed = ARMING_FLAG(ARMED);
    if (armed) {
        rt.armedOnce = true;
    }
    ambientUpdate();

    const float vbat = getBatteryVoltage() * 0.01f;
    const float unloadedMaxRpm = vbat * rt.kv;
    // same load derating as motorEstimateMaxRpm(); it turns negative past KV·Vbat ≈ 173k (e.g. 21000 KV on 2S)
    const float maxRpm = MAX(unloadedMaxRpm * (-5.44e-6f * unloadedMaxRpm + 0.944f), 0.0f);
    const float kCu = rt.kCu * sq(vbat);
#ifdef USE_DSHOT_TELEMETRY
    const bool haveRpm = isDshotTelemetryActive();
#endif
    // Battery current split across motors when per-motor ESC current is not available: by the prop power, rpm³,
    // with rpm telemetry, else by command². The command alone gives a stalled motor (crash, blocked prop) the
    // largest share although it takes almost none, and starves the motors that are working. The current with
    // the motors stopped (FC, VTX, receiver) is not motor current and is taken off first.
    float cmdSqSum = 0.0f, rpmCubeSum = 0.0f;
    float cmdFrac[MAX_SUPPORTED_MOTORS], rpmOf[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        cmdFrac[i] = constrainf((motorConvertToExternal(motor[i]) - 1000) * 0.001f, 0.0f, 1.0f);
        cmdSqSum += sq(cmdFrac[i]);
        rpmOf[i] = cmdFrac[i] * maxRpm;                             // no telemetry: no-load estimate
#ifdef USE_DSHOT_TELEMETRY
        if (haveRpm) {
            rpmOf[i] = getDshotRpm(i);
        }
#endif
        rpmCubeSum += power3(rpmOf[i] * 0.001f);
    }
#ifdef USE_DSHOT_TELEMETRY
    const bool splitByRpm = haveRpm && rpmCubeSum > 0.0f;
#else
    const bool splitByRpm = false;
#endif
    const float batteryAmps = getAmperage() * 0.01f;
    if (!armed && cmdSqSum == 0.0f) {       // learnt only with every motor stopped: not during a disarmed motor test
        rt.idleAmps += (batteryAmps - rt.idleAmps) * MIN(1.0f, dt / IDLE_AMPS_TAU_S);
    }
    const float motorsAmps = armed ? MAX(batteryAmps - rt.idleAmps, 0.0f) : 0.0f;
    float vCraftSq = 0.0f;
#ifdef USE_GPS
    if (STATE(GPS_FIX)) {
        vCraftSq = sq(gpsSol.groundSpeed * 0.01f);
    }
#endif
    if (isAltitudeAvailable()) {
        vCraftSq += sq(getAltitudeDerivative() * 0.01f);         // climb and descent: baro (and GPS) altitude rate, cm/s
    }
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        const float rpm = rpmOf[i];
        const float air = coreCooling(rpm, vCraftSq);
        const float wash = windingCooling(rpm);
        const float pCu = kCu * sq(motorNoiseRms(i, 0));
        const float pIron = rt.aIron * sq(rpm);
        // torque current copper loss: phase current ≈ bus current / duty, ripple energy (already in pCu) removed
        float motorAmps = 0.0f;
#ifdef USE_DSHOT_TELEMETRY
        if (haveRpm && (dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT))) {
            motorAmps = dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT];
        } else
#endif
        if (splitByRpm) {
            motorAmps = motorsAmps * power3(rpm * 0.001f) / rpmCubeSum;
        } else if (cmdSqSum > 0.0f) {
            motorAmps = motorsAmps * sq(cmdFrac[i]) / cmdSqSum;
        }
        const float iLoad = MAX(motorAmps - pCu / MAX(vbat, 1.0f), 0.0f);
        // the winding cannot lose more than the motor takes in: the /duty step must not blow up a small current
        const float pLoad = MIN(rt.rLoad * sq(iLoad / MAX(cmdFrac[i], 0.05f)), motorAmps * vbat);

        thermalNode_t *s = &node[i];
        const float q = (s->ts - s->tc) / rt.rsc;
        s->ts += (pCu + pLoad - q - (s->ts - rt.tAmb) * wash / rt.rca) * dt * rt.invCs;
        s->tc += (pIron + q - (s->tc - rt.tAmb) * air / rt.rca) * dt * rt.invCc;
        if (i < 4) {
            DEBUG_SET(DEBUG_MOTOR_THERMAL, i, lrintf(10.0f * s->ts));   //!< [index:0..3] Motor {1|2|3|4} Estimated Winding Temperature [unit:0.1degC]
        }
    }
}

// Steady winding rise from the ripple loss alone at a hover point, for the filter autotune's heat budget: the windings
// reach the air through the core (in series) and through the prop wash (in parallel), as in the time step.
float motorThermalRippleRise(float noiseRms, float rpm, float vbat)
{
    const float pCu = rt.kCu * sq(vbat * noiseRms);
    const float viaCore = rt.rsc + rt.rca / coreCooling(rpm, 0.0f);
    return pCu * viaCore / (1.0f + viaCore * windingCooling(rpm) / rt.rca);
}

float motorThermalGetTemp(int motor)
{
    return (motor >= 0 && motor < MAX_SUPPORTED_MOTORS) ? node[motor].ts : 0.0f;
}

float motorThermalGetMaxTemp(void)
{
    float m = 0.0f;
    for (int i = 0; i < getMotorCount(); i++) {
        m = MAX(m, node[i].ts);
    }
    return m;
}

bool motorThermalWarning(void)
{
    const uint8_t w = motorThermalConfig()->warning_c;
    return w && motorThermalGetMaxTemp() >= w;
}

#endif // USE_MOTOR_THERMAL
