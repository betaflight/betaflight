#include "board.h"
#include "mw.h"

extern uint8_t useServo;
extern rcReadRawDataPtr rcReadRawFunc;

// two receiver read functions
extern uint16_t pwmReadRawRC(uint8_t chan);
extern uint16_t spektrumReadRawRC(uint8_t chan);

void throttleCalibration(void)
{
    uint8_t offset = useServo ? 2 : 0;
    uint8_t len = pwmGetNumOutputChannels() -  offset;
    uint8_t i;

    LED1_ON;

    // write maxthrottle (high)
    for (i = offset; i < len; i++)
        pwmWrite(i, cfg.maxthrottle);

    delay(3000); // 3s delay on high

    // write 1000us (low)
    for (i = offset; i < len; i++)
        pwmWrite(i, 1000);

    // blink leds to show we're calibrated and time to remove bind plug
    failureMode(4);
}

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;

#if 0
    // PC12, PA15
    // using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO; // GPIOA/C+AFIO only
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR = 0x02000000;
    GPIOA->CRH = 0x34444444; // PIN 15 Output 50MHz
    GPIOA->BRR = 0x8000; // set low 15
    GPIOC->CRH = 0x44434444; // PIN 12 Output 50MHz
    GPIOC->BRR = 0x1000; // set low 12
    
#endif


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

    serialInit(cfg.serial_baudrate);

    // We have these sensors
#ifndef FY90Q
    // AfroFlight32
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);
#else
    // FY90Q
    sensorsSet(SENSOR_ACC);
#endif

    mixerInit(); // this will set useServo var depending on mixer type
    // pwmInit returns true if throttle calibration is requested. if so, do it here. throttleCalibration() does NOT return - for safety.
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.motor_pwm_rate;
    pwm_params.servoPwmRate = cfg.servo_pwm_rate;

    if (pwmInit(&pwm_params))
        throttleCalibration(); // noreturn

    // configure PWM/CPPM read function. spektrum will override that
    rcReadRawFunc = pwmReadRawRC;

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

    if (feature(FEATURE_SPEKTRUM)) {
        spektrumInit();
        rcReadRawFunc = spektrumReadRawRC;
    } else {
        // spektrum and GPS are mutually exclusive
        // Optional GPS - available only when using PPM, otherwise required pins won't be usable
        if (feature(FEATURE_PPM))
        {
            if (feature(FEATURE_GPS))
                gpsInit(cfg.gps_baudrate);
#ifdef SONAR
            if (feature(FEATURE_SONAR))
                Sonar_init();
#endif
        }
    }

    previousTime = micros();
    if (cfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = 400;
    calibratingG = 1000;
    f.SMALL_ANGLES_25 = 1;

    // loopy
    while (1) {
        loop();
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(cfg.mincommand);
    while (1);
}
