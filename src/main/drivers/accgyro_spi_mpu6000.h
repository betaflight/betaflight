
#pragma once

#define MPU6000_CONFIG		    	0x1A

// Registers
#define MPU6000_PRODUCT_ID          0x0C
#define MPU6000_SMPLRT_DIV          0x19
#define MPU6000_GYRO_CONFIG         0x1B
#define MPU6000_ACCEL_CONFIG        0x1C
#define MPU6000_FIFO_EN             0x23
#define MPU6000_INT_PIN_CFG         0x37
#define MPU6000_INT_ENABLE          0x38
#define MPU6000_INT_STATUS          0x3A
#define MPU6000_ACCEL_XOUT_H        0x3B
#define MPU6000_ACCEL_XOUT_L        0x3C
#define MPU6000_ACCEL_YOUT_H        0x3D
#define MPU6000_ACCEL_YOUT_L        0x3E
#define MPU6000_ACCEL_ZOUT_H        0x3F
#define MPU6000_ACCEL_ZOUT_L        0x40
#define MPU6000_TEMP_OUT_H          0x41
#define MPU6000_TEMP_OUT_L          0x42
#define MPU6000_GYRO_XOUT_H         0x43
#define MPU6000_GYRO_XOUT_L         0x44
#define MPU6000_GYRO_YOUT_H         0x45
#define MPU6000_GYRO_YOUT_L         0x46
#define MPU6000_GYRO_ZOUT_H         0x47
#define MPU6000_GYRO_ZOUT_L         0x48
#define MPU6000_USER_CTRL           0x6A
#define MPU6000_SIGNAL_PATH_RESET   0x68
#define MPU6000_PWR_MGMT_1          0x6B
#define MPU6000_PWR_MGMT_2          0x6C
#define MPU6000_FIFO_COUNTH         0x72
#define MPU6000_FIFO_COUNTL         0x73
#define MPU6000_FIFO_R_W            0x74
#define MPU6000_WHOAMI              0x75

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU6000_WHO_AM_I_CONST              (0x68)

bool mpu6000SpiDetect(void);

bool mpu6000SpiAccDetect(acc_t *acc);
bool mpu6000SpiGyroDetect(gyro_t *gyro);

bool mpu6000WriteRegister(uint8_t reg, uint8_t data);
bool mpu6000ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
