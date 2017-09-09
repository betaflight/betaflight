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
#include <string.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/assert.h"
#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "cms/cms.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/compass/compass.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/flash_m25p16.h"
#include "drivers/gpio.h"
#include "drivers/gyro_sync.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/io_pca9685.h"
#include "drivers/light_led.h"
#include "drivers/logging.h"
#include "drivers/nvic.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output.h"
#include "drivers/rx_pwm.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/vcd.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/displayport_msp.h"
#include "io/displayport_max7456.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/pwmdriver_i2c.h"
#include "io/osd.h"
#include "io/rcsplit.h"
#include "io/serial.h"

#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/initialisation.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"


#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#ifdef USE_HARDWARE_PREBOOT_SETUP
extern void initialisePreBootHardware(void);
#endif

extern uint8_t motorControlEnable;

typedef enum {
    SYSTEM_STATE_INITIALISING   = 0,
    SYSTEM_STATE_CONFIG_LOADED  = (1 << 0),
    SYSTEM_STATE_SENSORS_READY  = (1 << 1),
    SYSTEM_STATE_MOTORS_READY   = (1 << 2),
    SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3),
    SYSTEM_STATE_READY          = (1 << 7)
} systemState_e;

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void flashLedsAndBeep(void)
{
    LED1_ON;
    LED0_OFF;
    for (uint8_t i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        if (!(getPreferredBeeperOffMask() & (1 << (BEEPER_SYSTEM_INIT - 1))))
            BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;
}

void init(void)
{
#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    systemState = SYSTEM_STATE_INITIALISING;
    initBootlog();

    printfSupportInit();

    systemInit();

    // initialize IO (needed for all IO operations)
    IOInitGlobal();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

#ifdef BRUSHED_ESC_AUTODETECT
    detectBrushedESC();
#endif

    initEEPROM();
    ensureEEPROMContainsValidData();
    readEEPROM();

#ifdef USE_UNDERCLOCK
    systemClockSetup(systemConfig()->cpuUnderclock);
#endif
    
    i2cSetSpeed(systemConfig()->i2c_speed);

#ifdef USE_HARDWARE_PREBOOT_SETUP
    initialisePreBootHardware();
#endif

    addBootlogEvent2(BOOT_EVENT_CONFIG_LOADED, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    debugMode = systemConfig()->debug_mode;

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();

#ifdef ALIENFLIGHTF3
    ledInit(hardwareRevision == AFF3_REV_1 ? false : true);
#else
    ledInit(false);
#endif

#ifdef USE_EXTI
    EXTIInit();
#endif

    addBootlogEvent2(BOOT_EVENT_SYSTEM_INIT_DONE, BOOT_EVENT_FLAGS_NONE);

#ifdef SPEKTRUM_BIND
    if (feature(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spektrumBind(rxConfigMutable());
                break;
        }
    }
#endif

    delay(500);

    timerInit();  // timer must be initialized before any channel is allocated

#if defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

#ifdef USE_SERVOS
    servosInit();
    mixerUpdateStateFlags();    // This needs to be called early to allow pwm mapper to use information about FIXED_WING state
#endif

    drv_pwm_config_t pwm_params;
    memset(&pwm_params, 0, sizeof(pwm_params));

#ifdef USE_RANGEFINDER_HCSR04
    // HC-SR04 has a dedicated connection to FC and require two pins
    if (rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) {
        const rangefinderHardwarePins_t *rangefinderHardwarePins = rangefinderGetHardwarePins();
        if (rangefinderHardwarePins) {
            pwm_params.useTriggerRangefinder = true;
            pwm_params.rangefinderIOConfig.triggerTag = rangefinderHardwarePins->triggerTag;
            pwm_params.rangefinderIOConfig.echoTag = rangefinderHardwarePins->echoTag;
        }
    }
#endif

    // when using airplane/wing mixer, servo/motor outputs are remapped
    pwm_params.flyingPlatformType = getFlyingPlatformType();

#if defined(USE_UART2) && defined(STM32F10X)
    pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_USART2);
#endif
#ifdef STM32F303xC
    pwm_params.useUART3 = doesConfigurationUsePort(SERIAL_PORT_USART3);
#endif
#if defined(USE_UART2) && defined(STM32F40_41xxx)
    pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_USART2);
#endif
#if defined(USE_UART6) && defined(STM32F40_41xxx)
    pwm_params.useUART6 = doesConfigurationUsePort(SERIAL_PORT_USART6);
#endif
    pwm_params.useVbat = feature(FEATURE_VBAT);
    pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
    pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
    pwm_params.useRSSIADC = feature(FEATURE_RSSI_ADC);
    pwm_params.useCurrentMeterADC = feature(FEATURE_CURRENT_METER)
        && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC;
    pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
    pwm_params.usePPM = feature(FEATURE_RX_PPM);
    pwm_params.useSerialRx = feature(FEATURE_RX_SERIAL);

#ifdef USE_SERVOS
    pwm_params.useServoOutputs = isMixerUsingServos();
    pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
    pwm_params.servoCenterPulse = servoConfig()->servoCenterPulse;
    pwm_params.servoPwmRate = servoConfig()->servoPwmRate;
#endif

    pwm_params.pwmProtocolType = motorConfig()->motorPwmProtocol;
#ifndef BRUSHED_MOTORS
    pwm_params.useFastPwm = (motorConfig()->motorPwmProtocol == PWM_TYPE_ONESHOT125) ||
                            (motorConfig()->motorPwmProtocol == PWM_TYPE_ONESHOT42) ||
                            (motorConfig()->motorPwmProtocol == PWM_TYPE_MULTISHOT);
#endif
    pwm_params.motorPwmRate = motorConfig()->motorPwmRate;
    pwm_params.idlePulse = motorConfig()->mincommand;
    if (feature(FEATURE_3D)) {
        pwm_params.idlePulse = flight3DConfig()->neutral3d;
    }

    if (motorConfig()->motorPwmProtocol == PWM_TYPE_BRUSHED) {
        pwm_params.useFastPwm = false;
        featureClear(FEATURE_3D);
        pwm_params.idlePulse = 0; // brushed motors
    }

    pwm_params.enablePWMOutput = feature(FEATURE_PWM_OUTPUT_ENABLE);

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    pwmRxInit(systemConfig()->pwmRxInputFilteringMode);
#endif

#ifdef USE_PMW_SERVO_DRIVER
    /*
    If external PWM driver is enabled, for example PCA9685, disable internal
    servo handling mechanism, since external device will do that
    */
    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwm_params.useServoOutputs = false;
        pwm_params.useChannelForwarding = false;
    }
#endif

    // pwmInit() needs to be called as soon as possible for ESC compatibility reasons
    pwmInit(&pwm_params);

    mixerUsePWMIOConfiguration();

    if (!pwm_params.useFastPwm)
        motorControlEnable = true;

    addBootlogEvent2(BOOT_EVENT_PWM_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_MOTORS_READY;

#ifdef BEEPER
    beeperDevConfig_t beeperDevConfig = {
        .ioTag = IO_TAG(BEEPER),
#ifdef BEEPER_INVERTED
        .isOD = false,
        .isInverted = true
#else
        .isOD = true,
        .isInverted = false
#endif
    };

#if defined(NAZE) && defined(USE_HARDWARE_REVISION_DETECTION)
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperDevConfig.isOD = false;
        beeperDevConfig.isInverted = true;
    }
#endif

    beeperInit(&beeperDevConfig);
#endif

#ifdef USE_INVERTER
    initInverters();
#endif


#ifdef USE_SPI
#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1);
#endif
#ifdef USE_SPI_DEVICE_2
    spiInit(SPIDEV_2);
#endif
#ifdef USE_SPI_DEVICE_3
#ifdef ALIENFLIGHTF3
    if (hardwareRevision == AFF3_REV_2) {
        spiInit(SPIDEV_3);
    }
#else
    spiInit(SPIDEV_3);
#endif
#endif
#ifdef USE_SPI_DEVICE_4
    spiInit(SPIDEV_4);
#endif
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL1)
#if defined(FURYF3) || defined(OMNIBUS) || defined(SPRACINGF3MINI)
    if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif
#endif

#if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL2) && defined(SPRACINGF3)
    if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#ifdef USE_I2C
#if defined(I2C_DEVICE)
    #if defined(I2C_DEVICE_SHARES_UART3)
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
            i2cInit(I2C_DEVICE);
        }
    #else
        i2cInit(I2C_DEVICE);
    #endif
#endif

#if defined(I2C_DEVICE_EXT)
    #if defined(I2C_DEVICE_EXT_SHARES_UART3)
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
            i2cInit(I2C_DEVICE_EXT);
        }
    #else
        i2cInit(I2C_DEVICE_EXT);
    #endif
#endif
#endif

#ifdef USE_ADC
    drv_adc_config_t adc_params;
    memset(&adc_params, 0, sizeof(adc_params));

    // Allocate and initialize ADC channels if features are configured - can't rely on sensor detection here, it's done later
    if (feature(FEATURE_VBAT)) {
        adc_params.adcFunctionChannel[ADC_BATTERY] = adcChannelConfig()->adcFunctionChannel[ADC_BATTERY];
    }

    if (feature(FEATURE_RSSI_ADC)) {
        adc_params.adcFunctionChannel[ADC_RSSI] = adcChannelConfig()->adcFunctionChannel[ADC_RSSI];
    }

    if (feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        adc_params.adcFunctionChannel[ADC_CURRENT] =  adcChannelConfig()->adcFunctionChannel[ADC_CURRENT];
    }

#if defined(PITOT) && defined(USE_PITOT_ADC)
    if (pitotmeterConfig()->pitot_hardware == PITOT_ADC || pitotmeterConfig()->pitot_hardware == PITOT_AUTODETECT) {
        adc_params.adcFunctionChannel[ADC_AIRSPEED] = adcChannelConfig()->adcFunctionChannel[ADC_AIRSPEED];
    }
#endif

    adcInit(&adc_params);
#endif

    /* Extra 500ms delay prior to initialising hardware if board is cold-booting */
#if defined(GPS) || defined(MAG)
    if (!isMPUSoftReset()) {
        addBootlogEvent2(BOOT_EVENT_EXTRA_BOOT_DELAY, BOOT_EVENT_FLAGS_NONE);

        LED1_ON;
        LED0_OFF;

        for (int i = 0; i < 5; i++) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            delay(100);
        }

        LED0_OFF;
        LED1_OFF;
    }
#endif

    initBoardAlignment();

#ifdef CMS
    cmsInit();
#endif

#ifdef USE_DASHBOARD
    if (feature(FEATURE_DASHBOARD)) {
        dashboardInit();
    }
#endif

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsPreInit();
    }
#endif

    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }

    addBootlogEvent2(BOOT_EVENT_SENSOR_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_SENSORS_READY;

    flashLedsAndBeep();

#ifdef USE_DTERM_NOTCH
    pidInitFilters();
#endif

    imuInit();

    mspFcInit();
    mspSerialInit();

#ifdef USE_CLI
    cliInit(serialConfig());
#endif

    failsafeInit();

    rxInit();

#if (defined(OSD) || (defined(USE_MSP_DISPLAYPORT) && defined(CMS)))
    displayPort_t *osdDisplayPort = NULL;
#endif

#ifdef OSD
    if (feature(FEATURE_OSD)) {
#if defined(USE_MAX7456)
        // If there is a max7456 chip for the OSD then use it
        static vcdProfile_t vcdProfile;
        vcdProfile.video_system = osdConfig()->video_system;
        osdDisplayPort = max7456DisplayPortInit(&vcdProfile);
#elif defined(USE_OSD_OVER_MSP_DISPLAYPORT) // OSD over MSP; not supported (yet)
        osdDisplayPort = displayPortMspInit();
#endif
        // osdInit  will register with CMS by itself.
        osdInit(osdDisplayPort);
    }
#endif

#if defined(USE_MSP_DISPLAYPORT) && defined(CMS)
    // If OSD is not active, then register MSP_DISPLAYPORT as a CMS device.
    if (!osdDisplayPort) {
        cmsDisplayPortRegister(displayPortMspInit());
    }
#endif

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsInit();
        addBootlogEvent2(BOOT_EVENT_GPS_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    }
#endif

#ifdef NAV
    navigationInit();
#endif

#ifdef LED_STRIP
    ledStripInit();

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
        addBootlogEvent2(BOOT_EVENT_LEDSTRIP_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    }
#endif

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        telemetryInit();
        addBootlogEvent2(BOOT_EVENT_TELEMETRY_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    }
#endif

#ifdef USE_FLASHFS
#ifdef NAZE
    if (hardwareRevision == NAZE32_REV5) {
        m25p16_init(IOTAG_NONE);
    }
#elif defined(USE_FLASH_M25P16)
    m25p16_init(IOTAG_NONE);
#endif

    flashfsInit();
#endif

#ifdef USE_SDCARD
    bool sdcardUseDMA = false;

    sdcardInsertionDetectInit();

#ifdef SDCARD_DMA_CHANNEL_TX

#if defined(LED_STRIP) && defined(WS2811_DMA_CHANNEL)
    // Ensure the SPI Tx DMA doesn't overlap with the led strip
#if defined(STM32F4) || defined(STM32F7)
    sdcardUseDMA = !feature(FEATURE_LED_STRIP) || SDCARD_DMA_CHANNEL_TX != WS2811_DMA_STREAM;
#else
    sdcardUseDMA = !feature(FEATURE_LED_STRIP) || SDCARD_DMA_CHANNEL_TX != WS2811_DMA_CHANNEL;
#endif
#else
    sdcardUseDMA = true;
#endif

#endif

    sdcard_init(sdcardUseDMA);

    afatfs_init();
#endif

#ifdef BLACKBOX
    blackboxInit();
#endif

    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroStartCalibration();
#endif
#ifdef PITOT
    pitotStartCalibration();
#endif

    // start all timers
    // TODO - not implemented yet
    timerStart();

    // Now that everything has powered up the voltage and cell count be determined.
    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER))
        batteryInit();

#ifdef CJMCU
    LED2_ON;
#endif

#ifdef USE_PMW_SERVO_DRIVER
    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwmDriverInitialize();
    }
#endif

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    motorControlEnable = true;

    fcTasksInit();

#ifdef USE_RCSPLIT
    rcSplitInit();
#endif // USE_RCSPLIT

    addBootlogEvent2(BOOT_EVENT_SYSTEM_READY, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_READY;
}
