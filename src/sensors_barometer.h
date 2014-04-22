#pragma once

#ifdef BARO
void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void baroCommon(void);
int baroUpdate(void);
int32_t baroCalculateAltitude(void);
bool isBaroCalibrationComplete(void);
void performBaroCalibrationCycle(void);
#endif
