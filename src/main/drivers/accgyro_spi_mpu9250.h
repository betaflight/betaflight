
#pragma once

#define mpu9250_CONFIG		    	0x1A

/* We should probably use these. :)
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
*/

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU9250_WHO_AM_I_CONST              (0x71)

#define MPU9250_BIT_RESET                   (0x80)

// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

void mpu9250ResetGyro(void);

bool mpu9250SpiDetect(void);

bool mpu9250SpiAccDetect(acc_t *acc);
bool mpu9250SpiGyroDetect(gyro_t *gyro);

bool mpu9250WriteRegister(uint8_t reg, uint8_t data);
bool verifympu9250WriteRegister(uint8_t reg, uint8_t data);
bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
