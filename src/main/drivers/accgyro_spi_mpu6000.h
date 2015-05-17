
#pragma once

#define MPU6000_CONFIG		    	0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU6000_WHO_AM_I_CONST              (0x68)


bool mpu6000SpiAccDetect(acc_t *acc);
bool mpu6000SpiGyroDetect(gyro_t *gyro, uint16_t lpf);

void mpu6000SpiGyroRead(int16_t *gyroADC);
void mpu6000SpiAccRead(int16_t *gyroADC);
