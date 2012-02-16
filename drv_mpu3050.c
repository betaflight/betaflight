#include "board.h"

// MPU3050, Standard address 0x68
#define MPU3050_ADDRESS         0x68

// Registers
#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_TEMP_OUT        0x1B
#define MPU3050_GYRO_OUT        0x1D
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E

// Bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_DLPF_20HZ       0x04
#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

void mpu3050Init(void)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe
    
    ack = i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    if (!ack)
        failureMode(3);

    #if FREEFLIGHT
    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | MPU3050_DLPF_20HZ);
    #else
    // MWC more likely
    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS);
    #endif
    i2cWrite(MPU3050_ADDRESS, MPU3050_INT_CFG, 0);
    i2cWrite(MPU3050_ADDRESS, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    i2cWrite(MPU3050_ADDRESS, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
void mpu3050Read(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(MPU3050_ADDRESS, MPU3050_GYRO_OUT, 6, buf);
    gyroData[0] = buf[0] << 8 | buf[1];
    gyroData[1] = buf[2] << 8 | buf[3];
    gyroData[2] = buf[4] << 8 | buf[5];
}

int16_t mpu3050ReadTemp(void)
{
    uint8_t buf[2];
    i2cRead(MPU3050_ADDRESS, MPU3050_TEMP_OUT, 2, buf);
    
    return 35 + ((int32_t)(buf[0] << 8 | buf[1]) + 13200) / 280;
}
