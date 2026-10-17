/*
 * Motor thermal estimator: winding temperature per motor from rpm, battery voltage and the
 * motor-command noise index. Two thermal nodes (windings, stator core), convection vs air speed.
 * Calibrated on a 2203.5 / 2800 KV motor; scaled by stator size and KV.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

void motorThermalInit(void);
void motorThermalUpdate(const float *motorOutput, int motorCount);   // call at PID rate; decimates to 100 Hz
float motorThermalGetTemp(int motor);                                  // estimated winding temperature, °C
float motorThermalGetMaxTemp(void);
bool motorThermalWarning(void);
