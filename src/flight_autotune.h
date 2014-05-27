#pragma once

void autotuneReset();
void autotuneBeginNextPhase(pidProfile_t *pidProfileToTune, uint8_t pidControllerInUse);
float autotune(angle_index_t angleIndex, rollAndPitchInclination_t *inclination, float errorAngle);
void autotuneEndPhase();

bool isAutotuneIdle(void);
bool hasAutotunePhaseCompleted(void);
bool havePidsBeenUpdatedByAutotune(void);

