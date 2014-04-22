#pragma once

#ifdef MAG
void compassInit(void);
int compassGetADC(int16_flightDynamicsTrims_t *magZero);
#endif

extern int16_t magADC[XYZ_AXIS_COUNT];
extern sensor_align_e magAlign;
