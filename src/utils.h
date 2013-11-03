#pragma once

int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);
// sensor orientation
void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation);
void initBoardAlignment(void);
