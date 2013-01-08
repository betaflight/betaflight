#pragma once

void i2cInit(I2C_TypeDef *I2Cx);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
uint16_t i2cGetErrorCounter(void);
