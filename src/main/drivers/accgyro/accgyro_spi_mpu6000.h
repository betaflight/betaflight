
#pragma once

#include "drivers/bus.h"

#define MPU6000_CONFIG              0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

uint8_t mpu6000SpiDetect(const busDevice_t *bus);

bool mpu6000SpiAccDetect(accDev_t *acc);
bool mpu6000SpiGyroDetect(gyroDev_t *gyro);
