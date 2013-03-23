#include "board.h"

// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)       // Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0 / 390.0)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0 / 390.0)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

bool hmc5883lDetect(void)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MAG_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;

    return true;
}

void hmc5883lInit(float *calibrationGain)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    float magGain[3];
    int16_t magADC[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator

    // PB12 - MAG_DRDY output on rev4 hardware
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    delay(50);
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
    // The new gain setting is effective from the second measurement and on.
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5); // Set the Gain
    delay(100);
    hmc5883lRead(magADC);

    for (i = 0; i < 10; i++) {  // Collect 10 samples
        i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(50);
        hmc5883lRead(magADC);       // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[0] += magADC[0];
        xyz_total[1] += magADC[1];
        xyz_total[2] += magADC[2];

        // Detect saturation.
        if (-4096 >= min(magADC[0], min(magADC[1], magADC[2]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
    }

    // Apply the negative bias. (Same gain)
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++) {
        i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(50);
        hmc5883lRead(magADC);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[0] -= magADC[0];
        xyz_total[1] -= magADC[1];
        xyz_total[2] -= magADC[2];

        // Detect saturation.
        if (-4096 >= min(magADC[0], min(magADC[1], magADC[2]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
    }

    magGain[0] = fabs(820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[0]);
    magGain[1] = fabs(820.0 * HMC58X3_Y_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[1]);
    magGain[2] = fabs(820.0 * HMC58X3_Z_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[2]);

    // leave test mode
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);   // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2cWrite(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWrite(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);

    if (!bret) {                // Something went wrong so get a best guess
        magGain[0] = 1.0;
        magGain[1] = 1.0;
        magGain[2] = 1.0;
    }

    // if parameter was passed, give calibration values back
    if (calibrationGain) {
        calibrationGain[0] = magGain[0];
        calibrationGain[1] = magGain[1];
        calibrationGain[2] = magGain[2];
    }
}

void hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];

    i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);

    magData[0] = buf[0] << 8 | buf[1];
    magData[1] = buf[2] << 8 | buf[3];
    magData[2] = buf[4] << 8 | buf[5];
}
