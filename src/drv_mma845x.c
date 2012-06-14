#include "board.h"

// MMA8452QT, Standard address 0x1C

#define MMA8452_ADDRESS     0x1C

#define MMA8452_DEVICE_SIGNATURE    0x2A
#define MMA8451_DEVICE_SIGNATURE    0x1A

#define MMA8452_STATUS              0x00
#define MMA8452_OUT_X_MSB           0x01
#define MMA8452_WHO_AM_I            0x0D
#define MMA8452_XYZ_DATA_CFG        0x0E
#define MMA8452_HP_FILTER_CUTOFF    0x0F
#define MMA8452_CTRL_REG1           0x2A
#define MMA8452_CTRL_REG2           0x2B
#define MMA8452_CTRL_REG3           0x2C
#define MMA8452_CTRL_REG4           0x2D
#define MMA8452_CTRL_REG5           0x2E

#define MMA8452_FS_RANGE_8G         0x02
#define MMA8452_FS_RANGE_4G         0x01
#define MMA8452_FS_RANGE_2G         0x00

#define MMA8452_HPF_CUTOFF_LV1      0x00
#define MMA8452_HPF_CUTOFF_LV2      0x01
#define MMA8452_HPF_CUTOFF_LV3      0x02
#define MMA8452_HPF_CUTOFF_LV4      0x03

#define MMA8452_CTRL_REG2_B7_ST     0x80
#define MMA8452_CTRL_REG2_B6_RST    0x40
#define MMA8452_CTRL_REG2_B4_SMODS1 0x10
#define MMA8452_CTRL_REG2_B3_SMODS0 0x08
#define MMA8452_CTRL_REG2_B2_SLPE   0x04
#define MMA8452_CTRL_REG2_B1_MODS1  0x02
#define MMA8452_CTRL_REG2_B0_MODS0  0x01

#define MMA8452_CTRL_REG2_MODS_LP   0x03
#define MMA8452_CTRL_REG2_MODS_HR   0x02
#define MMA8452_CTRL_REG2_MODS_LNLP 0x01
#define MMA8452_CTRL_REG2_MODS_NOR  0x00

#define MMA8452_CTRL_REG3_IPOL          0x02
#define MMA8452_CTRL_REG4_INT_EN_DRDY   0x01

#define MMA8452_CTRL_REG1_LNOISE        0x04
#define MMA8452_CTRL_REG1_ACTIVE        0x01

extern uint16_t acc_1G;
static uint8_t device_id;

static void mma8452Init(void);
static void mma8452Read(int16_t *accelData);
static void mma8452Align(int16_t *accelData);

bool mma8452Detect(sensor_t *acc)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MMA8452_ADDRESS, MMA8452_WHO_AM_I, 1, &sig);
    if (!ack || (sig != MMA8452_DEVICE_SIGNATURE && sig != MMA8451_DEVICE_SIGNATURE))
        return false;

    acc->init = mma8452Init;
    acc->read = mma8452Read;
    acc->align = mma8452Align;
    device_id = sig;
    return true;
}

static void mma8452Init(void)
{
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, 0); // Put device in standby to configure stuff
    i2cWrite(MMA8452_ADDRESS, MMA8452_XYZ_DATA_CFG, MMA8452_FS_RANGE_8G);
    i2cWrite(MMA8452_ADDRESS, MMA8452_HP_FILTER_CUTOFF, MMA8452_HPF_CUTOFF_LV4);
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG2, MMA8452_CTRL_REG2_MODS_HR | MMA8452_CTRL_REG2_MODS_HR << 3); // High resolution measurement in both sleep and active modes
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG3, MMA8452_CTRL_REG3_IPOL); // Interrupt polarity (active HIGH)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG4, MMA8452_CTRL_REG4_INT_EN_DRDY); // Enable DRDY interrupt (unused by this driver)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG5, 0); // DRDY routed to INT2
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, MMA8452_CTRL_REG1_LNOISE | MMA8452_CTRL_REG1_ACTIVE); // Turn on measurements, low noise at max scale mode, Data Rate 800Hz. LNoise mode makes range +-4G.
    
    acc_1G = 256;
}

static void mma8452Read(int16_t *accelData)
{
    uint8_t buf[6];

    i2cRead(MMA8452_ADDRESS, MMA8452_OUT_X_MSB, 6, buf);
    accelData[0] = (buf[0] << 8) | buf[1];
    accelData[1] = (buf[2] << 8) | buf[3];
    accelData[2] = (buf[4] << 8) | buf[5];
    
    accelData[0] >>= 2;
    accelData[1] >>= 2;
    accelData[2] >>= 2;
}

static void mma8452Align(int16_t *accelData)
{
    accelData[0] = -accelData[0] / 4;
    accelData[1] = -accelData[1] / 4;
    accelData[2] = accelData[2] / 4;
}
