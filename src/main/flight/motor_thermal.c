#include <math.h>

#include "platform.h"

#ifdef USE_MOTOR_THERMAL

#include "build/debug.h"
#include "common/maths.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/motor_noise.h"
#include "flight/pid.h"
#include "io/gps.h"
#include "pg/motor.h"
#include "pg/motor_thermal.h"
#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/gyro.h"

#include "motor_thermal.h"

// Bench reference: X-NOVA 2203.5 2800KV, no prop, two-node fit on random + flight-like regimes
// (research/motor-thermal, RMSE ~1 °C). The fitted model is scaled as a whole (P·s, C·s, R/s) so that
// the ripple copper loss equals the measured excess bus power (2.0 W at 35 rms command noise, 16.2 V). Units: J/K, K/W, W/krpm², W/(rpm·duty)², Ohm, m/s.
#define REF_D_MM        22.0f
#define REF_H_MM        3.5f
#define REF_KV          2800.0f
#define REF_CS          2.88f   // windings heat capacity, J/K
#define REF_CC          21.9f   // stator core heat capacity, J/K
#define REF_RSC         3.63f   // windings -> core, K/W
#define REF_RCA         11.3f   // core -> air, still air, K/W
#define REF_A_IRON      0.0449f   // iron loss per (rpm/1000)², W
#define REF_K_CU        7.94e-07f   // ripple copper loss per (KV·Vbat·duty_rms)², W
#define REF_R_LL        0.2f       // line-to-line winding resistance (load-current copper loss)
#define AIR_GAIN        1.56f   // convection: 1 + AIR_GAIN·(v/10 m/s)^1.5
#define PROP_VI_K       0.252f     // induced velocity = n[rev/s]·D[m]·sqrt(2·Ct/π), Ct≈0.1
#define UPDATE_HZ       100

typedef struct { float ts, tc; } thermalNode_t;

static thermalNode_t node[MAX_SUPPORTED_MOTORS];
static struct {
    float cs, cc, rsc, rca, aIron, kCu, rLoad, bellRadiusM, propDiameterM, tAmb, dt;
    int decimation, counter;
} rt;

static float autoPropInch(float statorDmm)
{
    if (statorDmm <= 9)  return 1.6f;
    if (statorDmm <= 12) return 2.5f;
    if (statorDmm <= 15) return 3.0f;
    if (statorDmm <= 19) return 3.5f;
    if (statorDmm <= 22) return 4.5f;
    if (statorDmm <= 23) return 5.0f;
    if (statorDmm <= 27) return 5.5f;
    if (statorDmm <= 30) return 7.0f;
    return 8.0f;
}

void motorThermalInit(void)
{
    const motorThermalConfig_t *c = motorThermalConfig();
    const float d = MAX(c->stator_diameter_mm, 5), h = MAX(c->stator_height_dmm, 10) * 0.1f;
    const float mass = (d * d * h) / (REF_D_MM * REF_D_MM * REF_H_MM);                     // volume ratio
    const float area = (d * h + 0.5f * d * d) / (REF_D_MM * REF_H_MM + 0.5f * REF_D_MM * REF_D_MM);
    rt.cs = REF_CS * mass;
    rt.cc = REF_CC * mass;
    rt.rsc = REF_RSC / ((d * h) / (REF_D_MM * REF_H_MM));
    rt.rca = REF_RCA / area;
    rt.aIron = REF_A_IRON * mass;
    const float gain = c->gain_percent * 0.01f;
    const float kv = MAX(motorConfig()->kv, 100);
    rt.kCu = REF_K_CU * (h / REF_H_MM) * gain;                                                // ripple loss ∝ KV²·H (R ∝ 1/(KV²·H))
    rt.rLoad = c->resistance_mohm ? c->resistance_mohm * 0.001f * gain
                                  : REF_R_LL * sq(REF_KV / kv) * (REF_H_MM / h) * gain;         // measured, or the same R law for the torque current
    rt.bellRadiusM = (0.5f * d + 3.0f) * 0.001f;
    const float propInch = c->prop_diameter_dinch ? c->prop_diameter_dinch * 0.1f : autoPropInch(d);
    rt.propDiameterM = propInch * 0.0254f;
    const int gyroT = gyroGetTemperature();
    rt.tAmb = (gyroT > 5 && gyroT < 45) ? gyroT : 25.0f;
    rt.decimation = MAX(1, lrintf(1.0f / (UPDATE_HZ * pidGetDT())));
    rt.dt = rt.decimation * pidGetDT();
    rt.counter = 0;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        node[i].ts = node[i].tc = rt.tAmb;
    }
}

void motorThermalUpdate(const float *motorOutput, int motorCount)
{
    if (++rt.counter < rt.decimation) {
        return;
    }
    rt.counter = 0;

    const float vbat = getBatteryVoltage() * 0.01f;
    const float kv = motorConfig()->kv;
    const float maxRpm = motorEstimateMaxRpm();
    // battery current split across motors by command² when per-motor ESC current is not available
    const float batteryAmps = getAmperage() * 0.01f;
    float cmdSqSum = 0.0f;
    float cmdFrac[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        cmdFrac[i] = constrainf((motorConvertToExternal(motorOutput[i]) - 1000) * 0.001f, 0.0f, 1.0f);
        cmdSqSum += sq(cmdFrac[i]);
    }
    float vCraft = 0.0f;
#ifdef USE_GPS
    if (STATE(GPS_FIX)) {
        vCraft = gpsSol.groundSpeed * 0.01f;
    }
#endif
#ifdef USE_DSHOT_TELEMETRY
    const bool haveRpm = isDshotTelemetryActive();
#else
    const bool haveRpm = false;
#endif
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        float rpm;
#ifdef USE_DSHOT_TELEMETRY
        if (haveRpm) {
            rpm = getDshotRpm(i);
        } else
#endif
        {
            const float frac = (motorConvertToExternal(motorOutput[i]) - 1000) * 0.001f;   // no telemetry: no-load estimate
            rpm = constrainf(frac, 0.0f, 1.0f) * maxRpm;
        }
        const float n = rpm / 60.0f;
        const float vBell = 2.0f * M_PIf * n * rt.bellRadiusM;
        const float vProp = n * rt.propDiameterM * PROP_VI_K;
        const float v = sqrtf(vBell * vBell + vProp * vProp + vCraft * vCraft);
        const float air = 1.0f + AIR_GAIN * powf(v * 0.1f, 1.5f);

        const float duty = motorNoiseRms(i, 0) * 0.001f;          // command units -> duty fraction
        const float pCu = rt.kCu * sq(kv * vbat * duty);
        const float pIron = rt.aIron * sq(rpm * 0.001f);
        // torque current copper loss: phase current ≈ bus current / duty, ripple energy (already in pCu) removed
        float motorAmps = 0.0f;
#ifdef USE_DSHOT_TELEMETRY
        if (haveRpm && (dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT))) {
            motorAmps = dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT];
        } else
#endif
        if (cmdSqSum > 0.0f) {
            motorAmps = batteryAmps * sq(cmdFrac[i]) / cmdSqSum;
        }
        const float iLoad = MAX(motorAmps - pCu / MAX(vbat, 1.0f), 0.0f);
        const float pLoad = rt.rLoad * sq(iLoad / MAX(cmdFrac[i], 0.05f));

        thermalNode_t *s = &node[i];
        const float q = (s->ts - s->tc) / rt.rsc;
        s->ts += rt.dt * (pCu + pLoad - q) / rt.cs;
        s->tc += rt.dt * (pIron + q - (s->tc - rt.tAmb) * air / rt.rca) / rt.cc;
        if (i < 4) {
            DEBUG_SET(DEBUG_MOTOR_THERMAL, i, lrintf(10.0f * s->ts));   //!< [index:0..3] Motor {1|2|3|4} Estimated Winding Temperature [unit:0.1degC]
        }
    }
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
