bool pca9685Initialize(void);
void pca9685setPWMOn(uint8_t servoIndex, uint16_t on);
void pca9685setPWMOff(uint8_t servoIndex, uint16_t off);
void pca9685setPWMFreq(float freq);
void pca9685setServoPulse(uint8_t servoIndex, uint16_t pulse);
void pca9685sync(uint8_t cycleIndex);
