#include "board.h"
#include "mw.h"

int main(void)
{
    uint8_t i;
    
#if 0
    RCC->APB2ENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO; // GPIOB + AFIO
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR = 0x02000000;
    GPIOB->BRR = 0x18; // set low 4 & 3
    GPIOB->CRL = 0x44433444; // PIN 4 & 3 Output 50MHz
#endif    

    systemInit();

    readEEPROM();
    checkFirstTime();

    // configure features (TODO: pull them from eeprom)
    featureSet(FEATURE_VBAT | FEATURE_PPM);
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);

    pwmInit(feature(FEATURE_PPM), false);
    
    LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    // drop out any sensors that don't seem to work
     sensorsAutodetect();

    // Init sensors
    if (sensors(SENSOR_BARO))
        bmp085Init();
    if (sensors(SENSOR_ACC))
        adxl345Init();
    // if this fails, we get a beep + blink pattern. we're doomed.
    mpu3050Init();

    imuInit(); // Mag is initialized inside imuInit

    previousTime = micros();
    calibratingG = 400;
#if defined(POWERMETER)
    for (i = 0; i <= PMOTOR_SUM; i++)
        pMeter[i] = 0;
#endif

    // loopy    
    while (1) {
        loop();
    }
}
