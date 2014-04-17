#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "drivers/gpio_common.h"
#include "drivers/timer_common.h"
#include "drivers/pwm_common.h"
#include "flight_mixer.h"
#include "sensors_common.h"
#include "battery.h"
#include "config.h"

#if MAX_MOTORS != MAX_SUPPORTED_MOTORS
#error Motor configuration mismatch
#endif

#if MAX_SERVOS != MAX_SUPPORTED_SERVOS
#error Servo configuration mismatch
#endif
