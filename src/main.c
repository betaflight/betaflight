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
#include "drivers/pwm_common.h"
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

//#define USE_SOFTSERIAL_FOR_MAIN_PORT

extern rcReadRawDataPtr rcReadRawFunc;

extern uint32_t previousTime;

failsafe_t *failsafe;

void timerInit(void);
void initTelemetry(serialPorts_t *serialPorts);
void serialInit(serialConfig_t *initialSerialConfig);
failsafe_t* failsafeInit(rxConfig_t *intialRxConfig);
void pwmInit(drv_pwm_config_t *init, failsafe_t *initialFailsafe);
void rxInit(rxConfig_t *rxConfig, failsafe_t *failsafe);
void buzzerInit(failsafe_t *initialFailsafe);
void gpsInit(uint8_t baudrateIndex, uint8_t initialGpsProvider, gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile);
void sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, int16_t magDeclinationFromConfig);
void imuInit(void);

void loop(void);

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t* loopbackPort1 = NULL;
    serialPort_t* loopbackPort2 = NULL;
#endif
    systemInit(masterConfig.emf_avoidance);
    initPrintfSupport();

    ensureEEPROMContainsValidData();
    readEEPROM();

    // configure power ADC
    if (masterConfig.power_adc_channel > 0 && (masterConfig.power_adc_channel == 1 || masterConfig.power_adc_channel == 9))
        adc_params.powerAdcChannel = masterConfig.power_adc_channel;
    else {
        adc_params.powerAdcChannel = 0;
        masterConfig.power_adc_channel = 0;
    }

    adcInit(&adc_params);
    initBoardAlignment(&masterConfig.boardAlignment);

    // We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
    sensorsSet(SENSORS_SET);

    mixerInit(masterConfig.mixerConfiguration, masterConfig.customMixer);
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
    pwm_params.failsafeThreshold = currentProfile.failsafeConfig.failsafe_detect_threshold;

    switch (masterConfig.power_adc_channel) {
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

    failsafe = failsafeInit(&masterConfig.rxConfig);
    buzzerInit(failsafe);
#ifndef FY90Q
    timerInit();
#endif
    pwmInit(&pwm_params, failsafe);

    rxInit(&masterConfig.rxConfig, failsafe);

    if (feature(FEATURE_GPS) && !feature(FEATURE_SERIALRX)) {
        gpsInit(
            masterConfig.gps_baudrate,
            masterConfig.gps_type,
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
    sensorsAutodetect(&masterConfig.sensorAlignmentConfig, masterConfig.gyro_lpf, masterConfig.acc_hardware, currentProfile.mag_declination);
    imuInit(); // Mag is initialized inside imuInit

    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit(&masterConfig.batteryConfig);

    serialInit(&masterConfig.serialConfig);

#ifndef FY90Q
    if (canSoftwareSerialBeUsed()) {

#if defined(USE_SOFTSERIAL_FOR_MAIN_PORT) || (0)
        masterConfig.serialConfig.softserial_baudrate = 19200;
#endif
        setupSoftSerialPrimary(masterConfig.serialConfig.softserial_baudrate, masterConfig.serialConfig.softserial_1_inverted);
        setupSoftSerialSecondary(masterConfig.serialConfig.softserial_2_inverted);

#ifdef SOFTSERIAL_LOOPBACK
        loopbackPort1 = (serialPort_t*)&(softSerialPorts[0]);
        serialPrint(loopbackPort1, "SOFTSERIAL 1 - LOOPBACK ENABLED\r\n");

        loopbackPort2 = (serialPort_t*)&(softSerialPorts[1]);
#ifndef OLIMEXINO // PB0/D27 and PB1/D28 internally connected so this would result in a continuous stream of data
        serialPrint(loopbackPort2, "SOFTSERIAL 2 - LOOPBACK ENABLED\r\n");
#endif
#endif

#ifdef USE_SOFTSERIAL_FOR_MAIN_PORT
        serialPorts.mainport = (serialPort_t*)&(softSerialPorts[0]); // Uncomment to switch the main port to use softserial.
#endif
    }
#endif

    if (feature(FEATURE_TELEMETRY))
        initTelemetry(&serialPorts);

    previousTime = micros();

    if (masterConfig.mixerConfiguration == MULTITYPE_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

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
    writeAllMotors(masterConfig.escAndServoConfig.mincommand);
    while (1);
}
