/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/inverter.h"

#include "flight/flight.h"
#include "flight/mixer.h"

#include "io/serial.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "rx/rx.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "telemetry/telemetry.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#ifdef NAZE
#include "target/NAZE/hardware_revision.h"
#endif

#include "build_config.h"

#ifdef DEBUG_SECTION_TIMES
uint32_t sectionTimes[2][4];
#endif
extern uint32_t previousTime;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

failsafe_t *failsafe;

void initPrintfSupport(void);
void timerInit(void);
void initTelemetry(void);
void serialInit(serialConfig_t *initialSerialConfig);
failsafe_t* failsafeInit(rxConfig_t *intialRxConfig);
pwmOutputConfiguration_t *pwmInit(drv_pwm_config_t *init);
void mixerInit(MultiType mixerConfiguration, motorMixer_t *customMixers);
void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration);
void rxInit(rxConfig_t *rxConfig, failsafe_t *failsafe);
void beepcodeInit(failsafe_t *initialFailsafe);
void gpsInit(serialConfig_t *serialConfig, gpsConfig_t *initialGpsConfig);
void navigationInit(gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile);
bool sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, int16_t magDeclinationFromConfig);
void imuInit(void);
void displayInit(rxConfig_t *intialRxConfig);
void ledStripInit(ledConfig_t *ledConfigsToUse, hsvColor_t *colorsToUse, failsafe_t* failsafeToUse);
void loop(void);


#ifdef STM32F303xC
// from system_stm32f30x.c
void SetSysClock(void);
#endif
#ifdef STM32F10X
// from system_stm32f10x.c
void SetSysClock(bool overclock);
#endif

void init(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;
    bool sensorsOK = false;

    initPrintfSupport();

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

#ifdef STM32F303
    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));
#endif

#ifdef STM32F303xC
    SetSysClock();
#endif
#ifdef STM32F10X
    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock(masterConfig.emf_avoidance);
#endif

#ifdef NAZE
    detectHardwareRevision();
#endif

    systemInit();

    delay(100);

    ledInit();

#ifdef BEEPER
    beeperConfig_t beeperConfig = {
        .gpioMode = Mode_Out_OD,
        .gpioPin = BEEP_PIN,
        .gpioPort = BEEP_GPIO,
        .gpioPeripheral = BEEP_PERIPHERAL,
        .isInverted = false
    };
#ifdef NAZE
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperConfig.gpioMode = Mode_Out_PP;
        beeperConfig.isInverted = true;
    }
#endif

    beeperInit(&beeperConfig);
#endif

#ifdef INVERTER
    initInverter();
#endif


#ifdef USE_SPI
    spiInit(SPI1);
    spiInit(SPI2);
#endif

#ifdef NAZE
    updateHardwareRevision();
#endif

#ifdef USE_I2C
#ifdef NAZE
    if (hardwareRevision != NAZE32_SP) {
        i2cInit(I2C_DEVICE);
    }
#else
    // Configure the rest of the stuff
    i2cInit(I2C_DEVICE);
#endif
#endif

    adc_params.enableRSSI = feature(FEATURE_RSSI_ADC);
    adc_params.enableCurrentMeter = feature(FEATURE_CURRENT_METER);
    adc_params.enableExternal1 = false;
#ifdef OLIMEXINO
    adc_params.enableExternal1 = true;
#endif
#ifdef NAZE
    // optional ADC5 input on rev.5 hardware
    adc_params.enableExternal1 = (hardwareRevision >= NAZE32_REV5);
#endif

    adcInit(&adc_params);

    initBoardAlignment(&masterConfig.boardAlignment);

#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
        displayInit(&masterConfig.rxConfig);
    }
#endif

    // We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
    sensorsSet(SENSORS_SET);
    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsOK = sensorsAutodetect(&masterConfig.sensorAlignmentConfig, masterConfig.gyro_lpf, masterConfig.acc_hardware, currentProfile->mag_declination);

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

    imuInit();
    mixerInit(masterConfig.mixerConfiguration, masterConfig.customMixer);

#ifdef MAG
    if (sensors(SENSOR_MAG))
        compassInit();
#endif

    timerInit();

    serialInit(&masterConfig.serialConfig);

    memset(&pwm_params, 0, sizeof(pwm_params));
    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (masterConfig.mixerConfiguration == MULTITYPE_AIRPLANE || masterConfig.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
#ifdef STM32F10X
    pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_USART2);
#endif
    pwm_params.useVbat = feature(FEATURE_VBAT);
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
    pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
    pwm_params.useRSSIADC = feature(FEATURE_RSSI_ADC);
    pwm_params.useCurrentMeterADC = feature(FEATURE_CURRENT_METER);
    pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
    pwm_params.usePPM = feature(FEATURE_RX_PPM);
    pwm_params.useServos = isMixerUsingServos();
    pwm_params.extraServos = currentProfile->gimbalConfig.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = masterConfig.motor_pwm_rate;
    pwm_params.servoPwmRate = masterConfig.servo_pwm_rate;
    pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
    if (feature(FEATURE_3D))
        pwm_params.idlePulse = masterConfig.flight3DConfig.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
    pwm_params.servoCenterPulse = masterConfig.rxConfig.midrc;

    pwmRxInit(masterConfig.inputFilteringMode);

    pwmOutputConfiguration_t *pwmOutputConfiguration = pwmInit(&pwm_params);

    mixerUsePWMOutputConfiguration(pwmOutputConfiguration);

    failsafe = failsafeInit(&masterConfig.rxConfig);
    beepcodeInit(failsafe);
    rxInit(&masterConfig.rxConfig, failsafe);

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsInit(
            &masterConfig.serialConfig,
            &masterConfig.gpsConfig
        );
        navigationInit(
            &currentProfile->gpsProfile,
            &currentProfile->pidProfile
        );
    }
#endif

#ifdef SONAR
    if (feature(FEATURE_SONAR)) {
        Sonar_init();
    }
#endif

#ifdef LED_STRIP
    ledStripInit(masterConfig.ledConfigs, masterConfig.colors, failsafe);

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY))
        initTelemetry();
#endif

    previousTime = micros();

    if (masterConfig.mixerConfiguration == MULTITYPE_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

    ENABLE_STATE(SMALL_ANGLE);
    DISABLE_ARMING_FLAG(PREVENT_ARMING);

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif

    // Now that everything has powered up the voltage and cell count be determined.

    // Check battery type/voltage
    if (feature(FEATURE_VBAT))
        batteryInit(&masterConfig.batteryConfig);

#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
        displayEnablePageCycling();
    }
#endif
}

#ifdef SOFTSERIAL_LOOPBACK
void processLoopback(void) {
    if (loopbackPort) {
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialTotalBytesWaiting(loopbackPort))) {
            uint8_t b = serialRead(loopbackPort);
            serialWrite(loopbackPort, b);
        };
    }
}
#else
#define processLoopback()
#endif

int main(void) {
    init();

    while (1) {
        loop();
        processLoopback();
    }
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(masterConfig.escAndServoConfig.mincommand);
    while (1);
}
