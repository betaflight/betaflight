#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/system_common.h"
#include "drivers/gpio_common.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart_common.h"
#include "drivers/accgyro_common.h"
#include "drivers/pwm_mapping.h"
#include "drivers/adc_common.h"

#include "flight_common.h"
#include "flight_mixer.h"

#include "serial_common.h"
#include "failsafe.h"

#include "gps_common.h"
#include "escservo.h"
#include "rc_controls.h"
#include "rx_common.h"
#include "gimbal.h"
#include "sensors_common.h"
#include "sensors_sonar.h"
#include "sensors_barometer.h"
#include "sensors_acceleration.h"
#include "sensors_gyro.h"
#include "telemetry_common.h"
#include "battery.h"
#include "boardalignment.h"
#include "runtime_config.h"
#include "config.h"
#include "config_profile.h"
#include "config_master.h"

#include "build_config.h"

extern rcReadRawDataPtr rcReadRawFunc;

extern uint32_t previousTime;

failsafe_t *failsafe;

void initPrintfSupport(void);
void timerInit(void);
void initTelemetry(void);
void serialInit(serialConfig_t *initialSerialConfig);
failsafe_t* failsafeInit(rxConfig_t *intialRxConfig);
void pwmInit(drv_pwm_config_t *init);
void rxInit(rxConfig_t *rxConfig, failsafe_t *failsafe);
void buzzerInit(failsafe_t *initialFailsafe);
void gpsInit(serialConfig_t *serialConfig, uint8_t baudrateIndex, uint8_t initialGpsProvider, gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile);
bool sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, int16_t magDeclinationFromConfig);
void imuInit(void);

void loop(void);

// FIXME bad naming - this appears to be for some new board that hasn't been made available yet.
#ifdef PROD_DEBUG
void productionDebug(void)
{
    gpio_config_t gpio;

    // remap PB6 to USART1_TX
    gpio.pin = Pin_6;
    gpio.mode = Mode_AF_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, true);
    serialInit(mcfg.serial_baudrate);
    delay(25);
    serialPrint(core.mainport, "DBG ");
    printf("%08x%08x%08x OK\n", U_ID_0, U_ID_1, U_ID_2);
    serialPrint(core.mainport, "EOF");
    delay(25);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, false);
}
#endif

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
    bool sensorsOK = false;

    initPrintfSupport();

    ensureEEPROMContainsValidData();
    readEEPROM();

    systemInit(masterConfig.emf_avoidance);

    // configure power ADC
    if (masterConfig.power_adc_channel > 0 && (masterConfig.power_adc_channel == 1 || masterConfig.power_adc_channel == 9))
        adc_params.powerAdcChannel = masterConfig.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        masterConfig.power_adc_channel = 0;
    }

    adcInit(&adc_params);

    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit(&masterConfig.batteryConfig);

    initBoardAlignment(&masterConfig.boardAlignment);

    // We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
    sensorsSet(SENSORS_SET);
    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsOK = sensorsAutodetect(&masterConfig.sensorAlignmentConfig, masterConfig.gyro_lpf, masterConfig.acc_hardware, currentProfile.mag_declination);

    // production debug output
#ifdef PROD_DEBUG
    productionDebug();
#endif

    // if gyro was not detected due to whatever reason, we give up now.
    if (!sensorsOK)
        failureMode(3);

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

    imuInit(); // Mag is initialized inside imuInit
    mixerInit(masterConfig.mixerConfiguration, masterConfig.customMixer);

    if (!canSoftwareSerialBeUsed()) {
        featureClear(FEATURE_SOFTSERIAL);
    }

#ifndef FY90Q
    timerInit();
#endif

    serialInit(&masterConfig.serialConfig);

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (masterConfig.mixerConfiguration == MULTITYPE_AIRPLANE || masterConfig.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SERIALRX); // serial rx support uses UART too
    pwm_params.useSoftSerial = canSoftwareSerialBeUsed();
    pwm_params.usePPM = feature(FEATURE_PPM);
    pwm_params.enableInput = !feature(FEATURE_SERIALRX); // disable inputs if using spektrum
    pwm_params.useServos = isMixerUsingServos();
    pwm_params.extraServos = currentProfile.gimbalConfig.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = masterConfig.motor_pwm_rate;
    pwm_params.servoPwmRate = masterConfig.servo_pwm_rate;
    pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = masterConfig.flight3DConfig.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
    pwm_params.servoCenterPulse = masterConfig.rxConfig.midrc;

    switch (masterConfig.power_adc_channel) {
        case 1:
            pwm_params.adcChannel = PWM2;
            break;
        case 9:
            pwm_params.adcChannel = PWM8;
            break;
        default:
            pwm_params.adcChannel = 0; // FIXME this is the same as PWM1
            break;
    }

    pwmInit(&pwm_params);

    failsafe = failsafeInit(&masterConfig.rxConfig);
    buzzerInit(failsafe);
    rxInit(&masterConfig.rxConfig, failsafe);

    if (feature(FEATURE_GPS)) {
        gpsInit(
            &masterConfig.serialConfig,
            masterConfig.gps_initial_baudrate_index,
            masterConfig.gps_provider,
            &currentProfile.gpsProfile,
            &currentProfile.pidProfile
        );
    }

#ifdef SONAR
    // sonar stuff only works with PPM
    if (feature(FEATURE_PPM)) {
        if (feature(FEATURE_SONAR))
            Sonar_init();
    }
#endif

    if (feature(FEATURE_TELEMETRY))
        initTelemetry();

    previousTime = micros();

    if (masterConfig.mixerConfiguration == MULTITYPE_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

    f.SMALL_ANGLE = 1;

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    serialPort_t *loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        openSoftSerial1(19200, false);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif
    
    // loopy
    while (1) {
        loop();
        
#ifdef SOFTSERIAL_LOOPBACK
        if (loopbackPort) {
            while (serialTotalBytesWaiting(loopbackPort)) {
                uint8_t b = serialRead(loopbackPort);
                serialWrite(loopbackPort, b);
            };
        }
#endif

    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(masterConfig.escAndServoConfig.mincommand);
    while (1);
}
