#include "board.h"
#include "flight_common.h"
#include "flight_mixer.h"
#include "serial_common.h"
#include "failsafe.h"
#include "mw.h"

#include "gps_common.h"
#include "rx_common.h"
#include "drivers/serial_common.h"
#include "telemetry_common.h"
#include "boardalignment.h"
#include "config.h"
#include "config_storage.h"

#include "build_config.h"

extern rcReadRawDataPtr rcReadRawFunc;

failsafe_t *failsafe;

void initTelemetry(serialPorts_t *serialPorts);
void serialInit(serialConfig_t *initialSerialConfig);
failsafe_t* failsafeInit(failsafeConfig_t *initialFailsafeConfig, rxConfig_t *intialRxConfig);
void pwmInit(drv_pwm_config_t *init, failsafe_t *initialFailsafe);
void rxInit(rxConfig_t *rxConfig, failsafe_t *failsafe);
void buzzerInit(failsafe_t *initialFailsafe);

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t* loopbackPort1 = NULL;
    serialPort_t* loopbackPort2 = NULL;
#endif
    systemInit();
    initPrintfSupport();

    ensureEEPROMContainsValidData();
    readEEPROM();

    // configure power ADC
    if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9))
        adc_params.powerAdcChannel = mcfg.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        mcfg.power_adc_channel = 0;
    }

    adcInit(&adc_params);
    initBoardAlignment(&mcfg.boardAlignment);

    // We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
    sensorsSet(SENSORS_SET);

    mixerInit();
    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SERIALRX); // serial rx support uses UART too
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SERIALRX); // disable inputs if using spektrum
    pwm_params.useServos = isMixerUsingServos();
    pwm_params.extraServos = cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
    pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
    pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = mcfg.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
    pwm_params.servoCenterPulse = mcfg.rxConfig.midrc;
    pwm_params.failsafeThreshold = cfg.failsafeConfig.failsafe_detect_threshold;

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

    failsafe = failsafeInit(&cfg.failsafeConfig, &mcfg.rxConfig);
    buzzerInit(failsafe);
    pwmInit(&pwm_params, failsafe);

    rxInit(&mcfg.rxConfig, failsafe);

    if (feature(FEATURE_GPS) && !feature(FEATURE_SERIALRX)) {
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
        batteryInit(&mcfg.batteryConfig);

    serialInit(&mcfg.serialConfig);

#ifndef FY90Q
    if (feature(FEATURE_SOFTSERIAL)) {
        //mcfg.softserial_baudrate = 19200; // Uncomment to override config value

        setupSoftSerialPrimary(mcfg.serialConfig.softserial_baudrate, mcfg.serialConfig.softserial_1_inverted);
        setupSoftSerialSecondary(mcfg.serialConfig.softserial_2_inverted);

#ifdef SOFTSERIAL_LOOPBACK
        loopbackPort1 = (serialPort_t*)&(softSerialPorts[0]);
        serialPrint(loopbackPort1, "SOFTSERIAL 1 - LOOPBACK ENABLED\r\n");

        loopbackPort2 = (serialPort_t*)&(softSerialPorts[1]);
        serialPrint(loopbackPort2, "SOFTSERIAL 2 - LOOPBACK ENABLED\r\n");
#endif
        //core.mainport = (serialPort_t*)&(softSerialPorts[0]); // Uncomment to switch the main port to use softserial.
    }
#endif

    if (feature(FEATURE_TELEMETRY))
        initTelemetry(&serialPorts);

    previousTime = micros();
    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = CALIBRATING_ACC_CYCLES;
    calibratingG = CALIBRATING_GYRO_CYCLES;
    calibratingB = CALIBRATING_BARO_CYCLES;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLE = 1;

    // loopy
    while (1) {
        loop();
#ifdef SOFTSERIAL_LOOPBACK
        if (loopbackPort1) {
            while (serialTotalBytesWaiting(loopbackPort1)) {
                uint8_t b = serialRead(loopbackPort1);
                serialWrite(loopbackPort1, b);
                //serialWrite(core.mainport, 0x01);
                //serialWrite(core.mainport, b);
            };
        }

        if (loopbackPort2) {
            while (serialTotalBytesWaiting(loopbackPort2)) {
#ifndef OLIMEXINO // PB0/D27 and PB1/D28 internally connected so this would result in a continuous stream of data
                serialRead(loopbackPort2);
#else
                uint8_t b = serialRead(loopbackPort2);
                serialWrite(loopbackPort2, b);
                //serialWrite(core.mainport, 0x02);
                //serialWrite(core.mainport, b);
#endif // OLIMEXINO
            };
    }
#endif
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
