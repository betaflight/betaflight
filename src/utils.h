#pragma once

int constrain(int amt, int low, int high);
// sensor orientation
void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation);
void initBoardAlignment(void);
void productionDebug(void);
