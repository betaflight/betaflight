#pragma once

#ifdef MAG
void compassInit(void);
int compassGetADC(flightDynamicsTrims_t *magZero);
#endif

extern int16_t magADC[XYZ_AXIS_COUNT];
extern sensor_align_e magAlign;
