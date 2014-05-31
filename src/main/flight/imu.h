#pragma once

extern int32_t errorAltitudeI;
extern int32_t BaroPID;
extern int16_t throttleAngleCorrection;

int getEstimatedAltitude(void);
void computeIMU(void);
