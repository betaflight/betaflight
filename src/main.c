#include "board.h"
#include "mw.h"

extern uint8_t useServo;
extern rcReadRawDataPtr rcReadRawFunc;

// two receiver read functions
extern uint16_t pwmReadRawRC(uint8_t chan);
extern uint16_t spektrumReadRawRC(uint8_t chan);

static void _putc(void *p, char c)
{
    uartWrite(c);
}


int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;

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
    init_printf(NULL, _putc);

    checkFirstTime(false);
    readEEPROM();

    // configure power ADC
    if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9))
        adc_params.powerAdcChannel = mcfg.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        mcfg.power_adc_channel = 0;
    }

    adcInit(&adc_params);

    // We have these sensors
    sensorsSet(SENSORS_SET);

    mixerInit(); // this will set useServo var depending on mixer type
    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SPEKTRUM); // spektrum support uses UART too
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
    pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
    switch (mcfg.power_adc_channel) {
        case 1:
            pwm_params.adcChannel = PWM2;
            break;
        case 9:
            pwm_params.adcChannel = PWM8;
            break;
        default:
            pwm_params.adcChannel = 0;
        break;
    }

    pwmInit(&pwm_params);

    // configure PWM/CPPM read function. spektrum below will override that
    rcReadRawFunc = pwmReadRawRC;

    if (feature(FEATURE_SPEKTRUM)) {
        spektrumInit();
        rcReadRawFunc = spektrumReadRawRC;
    } else {
        // spektrum and GPS are mutually exclusive
        // Optional GPS - available in both PPM and PWM input mode, in PWM input, reduces number of available channels by 2.
        if (feature(FEATURE_GPS))
            gpsInit(mcfg.gps_baudrate);
    }
#ifdef SONAR
    // sonar stuff only works with PPM
    if (feature(FEATURE_PPM)) {
        if (feature(FEATURE_SONAR))
            Sonar_init();
    }
#endif

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

    serialInit(mcfg.serial_baudrate);

    previousTime = micros();
    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = 400;
    calibratingG = 1000;
    calibratingB = 200;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLES_25 = 1;

    // loopy
    while (1) {
        loop();
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
