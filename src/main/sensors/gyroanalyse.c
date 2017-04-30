#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_DATA_ANALYSE

#include "arm_math.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"

#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"


void gyroDataAnalyseInit(void)
{
}

void gyroDataAnalyse(const gyroDev_t *gyroDev, const gyro_t *gyro)
{
    UNUSED(gyroDev);
    UNUSED(gyro);
}

void gyroDataAnalyseUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}
#endif // USE_GYRO_DATA_ANALYSE
