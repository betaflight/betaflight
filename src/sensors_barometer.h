#pragma once

extern baro_t baro;

#ifdef BARO
void Baro_Common(void);
int Baro_update(void);
#endif
