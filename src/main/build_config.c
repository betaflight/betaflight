#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "flight/mixer.h"

#include "build_config.h"

#if MAX_PWM_MOTORS != MAX_SUPPORTED_MOTORS
#error Motor configuration mismatch
#endif

#if MAX_PWM_SERVOS != MAX_SUPPORTED_SERVOS
#error Servo configuration mismatch
#endif
