#include "platform.h"

#ifdef USE_MOTOR_THERMAL

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor_thermal.h"

PG_REGISTER_WITH_RESET_TEMPLATE(motorThermalConfig_t, motorThermalConfig, PG_MOTOR_THERMAL_CONFIG, 0);

PG_RESET_TEMPLATE(motorThermalConfig_t, motorThermalConfig,
    // default geometry is the common 5" motor (2207); the bench reference (2203.5) is the calibration
    // anchor the constants are scaled from, not the default a user is most likely to fly
    .stator_diameter_mm = 22,
    .stator_height_dmm = 70,
    .prop_diameter_dinch = 0,
    .gain_percent = 100,
    .warning_c = 90,
    .resistance_mohm = 0
);

#endif
