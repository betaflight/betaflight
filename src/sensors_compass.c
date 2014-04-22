#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/compass_hmc5883l.h"
#include "drivers/gpio_common.h"
#include "drivers/light_led.h"

#include "flight_common.h"
#include "boardalignment.h"
#include "runtime_config.h"
#include "config.h"

#include "sensors_common.h"
#include "sensors_compass.h"

extern uint32_t currentTime; // FIXME dependency on global variable, pass it in instead.

int16_t magADC[XYZ_AXIS_COUNT];
sensor_align_e magAlign = 0;
#ifdef MAG
static uint8_t magInit = 0;

void compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    hmc5883lInit();
    LED1_OFF;
    magInit = 1;
}

int compassGetADC(int16_flightDynamicsTrims_t *magZero)
{
    static uint32_t t, tCal = 0;
    static int16_flightDynamicsTrims_t magZeroTempMin;
    static int16_flightDynamicsTrims_t magZeroTempMax;
    uint32_t axis;

    if ((int32_t)(currentTime - t) < 0)
        return 0;                 //each read is spaced by 100ms
    t = currentTime + 100000;

    // Read mag sensor
    hmc5883lRead(magADC);
    alignSensors(magADC, magADC, magAlign);

    if (f.CALIBRATE_MAG) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            magZero->raw[axis] = 0;
            magZeroTempMin.raw[axis] = magADC[axis];
            magZeroTempMax.raw[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[X] -= magZero->raw[X];
        magADC[Y] -= magZero->raw[Y];
        magADC[Z] -= magZero->raw[Z];
    }

    if (tCal != 0) {
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets
            }

            saveAndReloadCurrentProfileToCurrentProfileSlot();
        }
    }

    return 1;
}
#endif
