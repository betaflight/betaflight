#pragma once

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// features
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);

bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);

// failure
void failureMode(uint8_t mode);
