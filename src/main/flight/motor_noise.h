/*
 * Motor command noise index: high-frequency RMS of each motor's final output command.
 * Proxy for winding/ESC heating caused by D-term and filter noise (see research/motor-thermal).
 */
#pragma once

#include <stdint.h>

#define MOTOR_NOISE_BANDS 2   // band 0: > MOTOR_NOISE_HP0_HZ, band 1: > MOTOR_NOISE_HP1_HZ

void motorNoiseInit(float looptimeUs);
void motorNoiseUpdate(const float *motorOutput, int motorCount);
float motorNoiseRms(int motor, int band);   // external (1000..2000) command units
