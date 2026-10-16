/*
 * Motor thermal estimator configuration. See research/motor-thermal/RESULTS.md (bench calibration).
 */
#pragma once

#include <stdint.h>
#include "pg/pg.h"

typedef struct motorThermalConfig_s {
    uint8_t stator_diameter_mm;    // e.g. 22 for a 2203.5 motor
    uint8_t stator_height_dmm;     // 0.1 mm units, e.g. 35 for 3.5 mm
    uint8_t prop_diameter_dinch;   // 0.1 inch units, 0 = auto from stator size
    uint8_t gain_percent;          // copper-loss gain, 100 = bench reference; raise if motors run hotter than estimated
    uint8_t warning_c;             // winding temperature warning threshold, 0 = off
    uint16_t resistance_mohm;      // line-to-line winding resistance, milliohm; 0 = derive from KV and stator size
} motorThermalConfig_t;

PG_DECLARE(motorThermalConfig_t, motorThermalConfig);
