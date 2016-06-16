
#include "drivers/sonar_hcsr04.h"
#include "drivers/io.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/battery.h"
#include "sensors/sonar.h"

const sonarHardware_t *sonarGetTargetHardwareConfiguration(batteryConfig_t *batteryConfig)
{
    static const sonarHardware_t const sonarPWM56 = {
		.triggerIO = IO_TAG(PB8),
		.echoIO = IO_TAG(PB9),
    };
    static const sonarHardware_t sonarRC78 = {
		.triggerIO = IO_TAG(PB0),
		.echoIO = IO_TAG(PB1),
    };
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC)) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
}
