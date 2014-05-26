#pragma once

void autotuneReset();
void autotuneBegin(pidProfile_t *pidProfileToTune, uint8_t pidControllerInUse);
float autotune(angle_index_t angleIndex, rollAndPitchInclination_t *inclination, float errorAngle);
void autotuneEnd();

bool havePidsBeenUpdatedByAutotune(void);

