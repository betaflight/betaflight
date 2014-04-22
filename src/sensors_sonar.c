#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sonar_hcsr04.h"
#include "runtime_config.h"

#include "sensors_common.h"
#include "sensors_sonar.h"

int32_t sonarAlt;              // to think about the unit

#ifdef SONAR

void Sonar_init(void)
{
    hcsr04_init(sonar_rc78);
    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void)
{
    hcsr04_get_distance(&sonarAlt);
}

#endif
