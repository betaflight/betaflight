#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_master.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"


bool isHardwareHealthy(void)
{
#ifdef MAG
	if (masterConfig.mag_hardware != MAG_NONE && !isCompassHealthy())
		return false;
#endif

	return true;
}
