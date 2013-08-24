#pragma once

typedef enum {
    sonar_pwm56,
    sonar_rc78,
} sonar_config_t;

void hcsr04_init(sonar_config_t config);
void hcsr04_get_distance(volatile int32_t *distance);
