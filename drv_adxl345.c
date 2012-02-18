#include "board.h"

// ADXL345, Alternative address mode 0x53
#define ADXL345_ADDRESS     0x53

#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_INT_ENABLE  0x2E
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATA_OUT    0x32
#define ADXL345_FIFO_CTL    0x38

#define ADXL345_BW_RATE_200 0x0B
#define ADXL345_POWER_MEAS  0x08
#define ADXL345_FULL_RANGE  0x08
#define ADXL345_RANGE_16G   0x03

bool adxl345Detect(void)
{
    bool ack = false;
    uint8_t sig = 0;
    
    ack = i2cRead(ADXL345_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xE5)
        return false;
    return true;
}

#define ADXL_RATE_100      0x0A
#define ADXL_RATE_200      0x0B
#define ADXL_RATE_400      0x0C
#define ADXL_RATE_800      0x0D
#define ADXL_RATE_1600     0x0E
#define ADXL_RATE_3200     0x0F
#define ADXL_FULL_RES      0x08
#define ADXL_RANGE_2G      0x00
#define ADXL_RANGE_4G      0x01
#define ADXL_RANGE_8G      0x02
#define ADXL_RANGE_16G     0x03

void adxl345Init(void)
{
#ifdef FREEFLIGHT
    i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_BW_RATE_200);
    i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, ADXL345_POWER_MEAS);
    i2cWrite(ADXL345_ADDRESS, ADXL345_INT_ENABLE, 0);
    i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_16G);
    i2cWrite(ADXL345_ADDRESS, ADXL345_FIFO_CTL, 0);
#else
    // MWC defaults
    i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, 1 << 3);        //  register: Power CTRL  -- value: Set measure bit 3 on
    // i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x0B);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    // i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, 0x09);  //  register: BW_RATE     -- value: rate=50hz, bw=20hz
    i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, (ADXL_RANGE_8G & 0x03) | ADXL_FULL_RES);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL_RATE_800);  //  register: BW_RATE     -- value: rate=50hz, bw=20hz


#endif
}

void adxl345Read(int16_t *accelData)
{
    uint8_t buf[6];
    
    i2cRead(ADXL345_ADDRESS, ADXL345_DATA_OUT, 6, buf);
    accelData[0] = buf[1] << 8 | buf[0];
    accelData[1] = buf[3] << 8 | buf[2];
    accelData[2] = buf[5] << 8 | buf[4];
}
