#include "board.h"

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 hardware
#define MPU6050_ADDRESS         0x68

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

#define MPU6050_LPF_256HZ       0
#define MPU6050_LPF_188HZ       1
#define MPU6050_LPF_98HZ        2
#define MPU6050_LPF_42HZ        3
#define MPU6050_LPF_20HZ        4
#define MPU6050_LPF_10HZ        5
#define MPU6050_LPF_5HZ         6

static uint8_t mpuLowPassFilter = MPU6050_LPF_42HZ;
static sensor_align_e gyroAlign = CW0_DEG;
static sensor_align_e accAlign = CW0_DEG;

static void mpu6050AccInit(sensor_align_e align);
static void mpu6050AccRead(int16_t *accData);
static void mpu6050GyroInit(sensor_align_e align);
static void mpu6050GyroRead(int16_t *gyroData);

extern uint16_t acc_1G;
static uint8_t mpuAccelHalf = 0;

bool mpu6050Detect(sensor_t *acc, sensor_t *gyro, uint16_t lpf, uint8_t *scale)
{
    bool ack;
    uint8_t sig, rev;
    uint8_t tmp[6];

    delay(35);                  // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cRead(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig);
    if (!ack)
        return false;

    // So like, MPU6xxx has a "WHO_AM_I" register, that is used to verify the identity of the device.
    // The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
    // The least significant bit of the MPU-60X0’s I2C address is determined by the value of the AD0 pin. (we know that already).
    // But here's the best part: The value of the AD0 pin is not reflected in this register.
    if (sig != (MPU6050_ADDRESS & 0x7e))
        return false;

    // determine product ID and accel revision
    i2cRead(MPU6050_ADDRESS, MPU_RA_XA_OFFS_H, 6, tmp);
    rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);
    if (rev) {
        /* Congrats, these parts are better. */
        if (rev == 1) {
            mpuAccelHalf = 1;
        } else if (rev == 2) {
            mpuAccelHalf = 0;
        } else {
            failureMode(5);
        }
    } else {
        i2cRead(MPU6050_ADDRESS, MPU_RA_PRODUCT_ID, 1, &sig);
        rev = sig & 0x0F;
        if (!rev) {
            failureMode(5);
        } else if (rev == 4) {
            mpuAccelHalf = 1;
        } else {
            mpuAccelHalf = 0;
        }
    }

    acc->init = mpu6050AccInit;
    acc->read = mpu6050AccRead;
    gyro->init = mpu6050GyroInit;
    gyro->read = mpu6050GyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    // give halfacc (old revision) back to system
    if (scale)
        *scale = mpuAccelHalf;

    // default lpf is 42Hz
    switch (lpf) {
        case 256:
            mpuLowPassFilter = MPU6050_LPF_256HZ;
            break;
        case 188:
            mpuLowPassFilter = MPU6050_LPF_188HZ;
            break;
        case 98:
            mpuLowPassFilter = MPU6050_LPF_98HZ;
            break;
        default:
        case 42:
            mpuLowPassFilter = MPU6050_LPF_42HZ;
            break;
        case 20:
            mpuLowPassFilter = MPU6050_LPF_20HZ;
            break;
        case 10:
            mpuLowPassFilter = MPU6050_LPF_10HZ;
            break;
        case 5:
            mpuLowPassFilter = MPU6050_LPF_5HZ;
            break;
    }

    return true;
}

static void mpu6050AccInit(sensor_align_e align)
{
    if (mpuAccelHalf)
        acc_1G = 255 * 8;
    else
        acc_1G = 512 * 8;

    if (align > 0)
        accAlign = align;
}

static void mpu6050AccRead(int16_t *accData)
{
    uint8_t buf[6];
    int16_t data[3];

    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 6, buf);
    data[0] = (int16_t)((buf[0] << 8) | buf[1]);
    data[1] = (int16_t)((buf[2] << 8) | buf[3]);
    data[2] = (int16_t)((buf[4] << 8) | buf[5]);

    alignSensors(data, accData, accAlign);
}

static void mpu6050GyroInit(sensor_align_e align)
{
    gpio_config_t gpio;

    // MPU_INT output on rev4/5 hardware (PB13, PC13)
    gpio.pin = Pin_13;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    if (hse_value == 8000000)
        gpioInit(GPIOB, &gpio);
    else if (hse_value == 12000000)
        gpioInit(GPIOC, &gpio);

    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    i2cWrite(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    i2cWrite(MPU6050_ADDRESS, MPU_RA_CONFIG, mpuLowPassFilter);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff. Moved into gyro init because the reset above would screw up accel config. Oops.
    // Accel scale 8g (4096 LSB/g)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 2 << 3);

    if (align > 0)
        gyroAlign = align;
}

static void mpu6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    int16_t data[3];

    i2cRead(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, 6, buf);
    data[0] = (int16_t)((buf[0] << 8) | buf[1]) / 4;
    data[1] = (int16_t)((buf[2] << 8) | buf[3]) / 4;
    data[2] = (int16_t)((buf[4] << 8) | buf[5]) / 4;

    alignSensors(data, gyroData, gyroAlign);
}
