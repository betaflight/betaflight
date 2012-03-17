#include "board.h"

// MPU6050, Standard address 0x68
#define MPU6050_ADDRESS         0x68
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_SMPLRT_DIV      0  //8000Hz
#define MPU6050_DLPF_CFG        0
#define MPU6050_GYRO_OUT        0x43
#define MPU6050_ACC_OUT         0x3B

static void mpu6050AccInit(void);
static void mpu6050AccRead(int16_t *accData);
static void mpu6050AccAlign(int16_t *accData);
static void mpu6050GyroInit(void);
static void mpu6050GyroRead(int16_t *gyroData);
static void mpu6050GyroAlign(int16_t *gyroData);

extern uint16_t acc_1G;

bool mpu6050Detect(sensor_t *acc, sensor_t *gyro)
{
    bool ack;
    uint8_t sig;

    delay(35); // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cRead(MPU6050_ADDRESS, MPU6050_WHO_AM_I, 1, &sig);
    if (!ack)
        return false;
    
    if (sig != 0x68)
        return false;

    acc->init = mpu6050AccInit;
    acc->read = mpu6050AccRead;
    acc->orient = mpu6050AccAlign;
    gyro->init = mpu6050GyroInit;
    gyro->read = mpu6050GyroRead;
    gyro->orient = mpu6050GyroAlign;

    return true;
}

static void mpu6050AccInit(void)
{
    i2cWrite(MPU6050_ADDRESS, 0x1C, 0x10);             // ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
    // note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
    // confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
    // seems to be not a problem on MPU6000 lol
    acc_1G = 1023;
}

static void mpu6050AccRead(int16_t *accData)
{
    uint8_t buf[6];
    
    i2cRead(MPU6050_ADDRESS, MPU6050_ACC_OUT, 6, buf);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

static void mpu6050AccAlign(int16_t *accData)
{
    int16_t temp[2];
    temp[0] = accData[0];
    temp[1] = accData[1];

    // official direction is RPY
    accData[0] = temp[1] / 8;
    accData[1] = -temp[0] / 8;
    accData[2] = accData[2] / 8;
}

static void mpu6050GyroInit(void)
{
    i2cWrite(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    i2cWrite(MPU6050_ADDRESS, 0x19, 0x00);             //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2cWrite(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWrite(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWrite(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

static void mpu6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(MPU6050_ADDRESS, MPU6050_GYRO_OUT, 6, buf);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

static void mpu6050GyroAlign(int16_t *gyroData)
{
    // official direction is RPY
    gyroData[0] = gyroData[0] / 4;
    gyroData[1] = gyroData[1] / 4;
    gyroData[2] = -gyroData[2] / 4;
}
