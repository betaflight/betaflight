#include "board.h"

// HMC5883L, default address 0x1E

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

bool hmc5883lDetect(void)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MAG_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;

    return true;
}

void hmc5883lInit(void)
{
    delay(100);
    i2cWrite(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | NormalOperation);
    delay(50);
}

void hmc5883lCal(uint8_t calibration_gain)
{
    // force positiveBias (compass should return 715 for all channels)
    i2cWrite(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | PositiveBiasConfig);
    delay(50);
    // set gains for calibration
    i2cWrite(MAG_ADDRESS, ConfigRegB, calibration_gain);
    i2cWrite(MAG_ADDRESS, ModeRegister, SingleConversion);
}

void hmc5883lFinishCal(void)
{
    // leave test mode
    i2cWrite(MAG_ADDRESS, ConfigRegA, SampleAveraging_8 << 5 | DataOutputRate_75HZ << 2 | NormalOperation);
    i2cWrite(MAG_ADDRESS, ConfigRegB, magGain);
    i2cWrite(MAG_ADDRESS, ModeRegister, ContinuousConversion);
}

void hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];

    i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);

    magData[0] = buf[0] << 8 | buf[1];
    magData[1] = buf[2] << 8 | buf[3];
    magData[2] = buf[4] << 8 | buf[5];
}
