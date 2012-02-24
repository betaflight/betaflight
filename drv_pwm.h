#pragma once

void pwmInit(bool usePPM, bool useServos, bool useDigitalServos);
void pwmWrite(uint8_t channel, uint16_t value);
uint16_t pwmRead(uint8_t channel);
