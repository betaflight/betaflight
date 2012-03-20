#include "board.h"
#include "mw.h"

extern uint8_t useServo;

int main(void)
{
    uint8_t i;
    
#if 0
    // using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO; // GPIOB + AFIO
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR = 0x02000000;
    GPIOB->BRR = 0x18; // set low 4 & 3
    GPIOB->CRL = 0x44433444; // PIN 4 & 3 Output 50MHz
#endif    

    systemInit();

    readEEPROM();
    checkFirstTime(false);

    // We have these sensors
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);

    mixerInit(); // this will set useServo var depending on mixer type
    pwmInit(feature(FEATURE_PPM), useServo, feature(FEATURE_DIGITAL_SERVO));

    LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsAutodetect();
    imuInit(); // Mag is initialized inside imuInit

    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit();

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
