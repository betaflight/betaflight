#include "board.h"

// HMC5883L, default address 0x1E

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

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
    // force positiveBias
    i2cWrite(MAG_ADDRESS, 0x00, 0x71);      //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2cWrite(MAG_ADDRESS, 0x01, 0x60);      //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2cWrite(MAG_ADDRESS, 0x02, 0x01);      //Mode register             -- 000000 01    single Conversion Mode
    // this enters test mode
}

void hmc5883lFinishCal(void)
{
    // leave test mode
    i2cWrite(MAG_ADDRESS, 0x00, 0x70);      //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2cWrite(MAG_ADDRESS, 0x01, 0x20);      //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWrite(MAG_ADDRESS, 0x02, 0x00);      //Mode register             -- 000000 00    continuous Conversion Mode
}

void hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];
    
    i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    
    magData[0] = buf[0] << 8 | buf[1];
    magData[1] = buf[2] << 8 | buf[3];
    magData[2] = buf[4] << 8 | buf[5];
}
