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
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG);

    mixerInit(); // this will set useServo var depending on mixer type
    // pwmInit returns true if throttle calibration is requested. if so, do it here. throttleCalibration() does NOT return - for safety.
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
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
        if (feature(FEATURE_PPM) && feature(FEATURE_GPS))
           gpsInit(cfg.gps_baudrate);
    }

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
