#pragma once

#ifdef MAG
void Mag_init(void);
int Mag_getADC(void);
#endif

extern int16_t magADC[XYZ_AXIS_COUNT];
