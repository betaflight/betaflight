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

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/rx_pwm.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/exti.h"
#include "drivers/vtx_soft_spi_rtc6705.h"

#ifdef USE_BST
#include "bus_bst.h"
#endif

#include "fc/config.h"
#include "fc/fc_init.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/cli.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/displayport_max7456.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/dashboard.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/osd.h"
#include "io/displayport_msp.h"
#include "io/vtx.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_tramp.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "sensors/esc_sensor.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build/build_config.h"
#include "build/debug.h"

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

extern uint8_t motorControlEnable;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void processLoopback(void)
{
#ifdef SOFTSERIAL_LOOPBACK
    if (loopbackPort) {
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
            uint8_t b = serialRead(loopbackPort);
            serialWrite(loopbackPort, b);
        };
    }
#endif
}

void init(void)
{
#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

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

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    //i2cSetOverclock(masterConfig.i2c_overclock);

    debugMode = masterConfig.debug_mode;

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();

#ifdef TARGET_PREINIT
    targetPreInit();
#endif

    ledInit(statusLedConfig());
    LED2_ON;

#ifdef USE_EXTI
    EXTIInit();
#endif

#if defined(BUTTONS)
#ifdef BUTTON_A_PIN
    IO_t buttonAPin = IOGetByTag(IO_TAG(BUTTON_A_PIN));
    IOInit(buttonAPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonAPin, IOCFG_IPU);
#endif

#ifdef BUTTON_B_PIN
    IO_t buttonBPin = IOGetByTag(IO_TAG(BUTTON_B_PIN));
    IOInit(buttonBPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonBPin, IOCFG_IPU);
#endif

    // Check status of bind plug and exit if not active
    delayMicroseconds(10);  // allow configuration to settle

    if (!isMPUSoftReset()) {
#if defined(BUTTON_A_PIN) && defined(BUTTON_B_PIN)
        // two buttons required
        uint8_t secondsRemaining = 5;
        bool bothButtonsHeld;
        do {
            bothButtonsHeld = !IORead(buttonAPin) && !IORead(buttonBPin);
            if (bothButtonsHeld) {
                if (--secondsRemaining == 0) {
                    resetEEPROM();
                    systemReset();
                }
                delay(1000);
                LED0_TOGGLE;
            }
        } while (bothButtonsHeld);
#endif
    }
#endif

#ifdef SPEKTRUM_BIND
    if (feature(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spektrumBind(rxConfig());
                break;
        }
    }
#endif

    delay(100);

    timerInit();  // timer must be initialized before any channel is allocated

#if defined(AVOID_UART1_FOR_PWM_PPM)
    serialInit(serialConfig(), feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART1 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(serialConfig(), feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(serialConfig(), feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(serialConfig(), feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    mixerInit(mixerConfig()->mixerMode, masterConfig.customMotorMixer);
#ifdef USE_SERVOS
    servoMixerInit(masterConfig.customServoMixer);
#endif

    uint16_t idlePulse = motorConfig()->mincommand;
    if (feature(FEATURE_3D)) {
        idlePulse = flight3DConfig()->neutral3d;
    }

    if (motorConfig()->motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureClear(FEATURE_3D);
        idlePulse = 0; // brushed motors
    }

    mixerConfigureOutput();
    motorInit(motorConfig(), idlePulse, getMotorCount());

#ifdef USE_SERVOS
    servoConfigureOutput();
    if (isMixerUsingServos()) {
        //pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
        servoInit(servoConfig());
    }
#endif

#if defined(USE_PWM) || defined(USE_PPM)
    if (feature(FEATURE_RX_PPM)) {
        ppmRxInit(ppmConfig(), motorConfig()->motorPwmProtocol);
    } else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        pwmRxInit(pwmConfig());
    }
#endif

    systemState |= SYSTEM_STATE_MOTORS_READY;

#ifdef BEEPER
    beeperInit(beeperConfig());
#endif
/* temp until PGs are implemented. */
#ifdef USE_INVERTER
    initInverters();
#endif

#ifdef USE_BST
    bstInit(BST_DEVICE);
#endif

#ifdef TARGET_BUS_INIT
    targetBusInit();
#else
    #ifdef USE_SPI
        #ifdef USE_SPI_DEVICE_1
            spiInit(SPIDEV_1);
        #endif
        #ifdef USE_SPI_DEVICE_2
            spiInit(SPIDEV_2);
        #endif
        #ifdef USE_SPI_DEVICE_3
            spiInit(SPIDEV_3);
        #endif
        #ifdef USE_SPI_DEVICE_4
            spiInit(SPIDEV_4);
        #endif
    #endif

    #ifdef USE_I2C
        i2cInit(I2C_DEVICE);
    #endif
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#ifdef VTX
    vtxInit();
#endif

#if defined(SONAR_SOFTSERIAL2_EXCLUSIVE) && defined(SONAR) && defined(USE_SOFTSERIAL2)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#if defined(SONAR_SOFTSERIAL1_EXCLUSIVE) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif

#ifdef USE_ADC
    /* these can be removed from features! */
    adcConfig()->vbat.enabled = feature(FEATURE_VBAT);
    adcConfig()->currentMeter.enabled = feature(FEATURE_CURRENT_METER);
    adcConfig()->rssi.enabled = feature(FEATURE_RSSI_ADC);
    adcInit(adcConfig());
#endif

    initBoardAlignment(boardAlignment());

#ifdef CMS
    cmsInit();
#endif

#ifdef USE_DASHBOARD
    if (feature(FEATURE_DASHBOARD)) {
        dashboardInit(rxConfig());
    }
#endif

#ifdef USE_RTC6705
    if (feature(FEATURE_VTX)) {
        rtc6705_soft_spi_init();
        current_vtx_channel = masterConfig.vtx_channel;
        rtc6705_soft_spi_set_channel(vtx_freq[current_vtx_channel]);
        rtc6705_soft_spi_set_rf_power(masterConfig.vtx_power);
    }
#endif

#ifdef OSD
    if (feature(FEATURE_OSD)) {
#ifdef USE_MAX7456
        // if there is a max7456 chip for the OSD then use it, otherwise use MSP
        displayPort_t *osdDisplayPort = max7456DisplayPortInit(vcdProfile(), displayPortProfileMax7456());
#else
        displayPort_t *osdDisplayPort = displayPortMspInit(displayPortProfileMax7456());
#endif
        osdInit(osdDisplayPort);
    }
#endif

#ifdef SONAR
    const sonarConfig_t *sonarConfig = sonarConfig();
#else
    const void *sonarConfig = NULL;
#endif
    if (!sensorsAutodetect(gyroConfig(), accelerometerConfig(), compassConfig(), barometerConfig(), sonarConfig)) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }

    systemState |= SYSTEM_STATE_SENSORS_READY;

    LED1_ON;
    LED0_OFF;
    LED2_OFF;

    for (int i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        if (!(getBeeperOffMask() & (1 << (BEEPER_SYSTEM_INIT - 1)))) BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    // gyro.targetLooptime set in sensorsAutodetect(), so we are ready to call pidSetTargetLooptime()
    pidSetTargetLooptime((gyro.targetLooptime + LOOPTIME_SUSPEND_TIME) * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(&currentProfile->pidProfile);
    pidInitConfig(&currentProfile->pidProfile);

    imuInit();

    mspFcInit();
    mspSerialInit();

#if defined(USE_MSP_DISPLAYPORT) && defined(CMS)
    cmsDisplayPortRegister(displayPortMspInit(displayPortProfileMsp()));
#endif

#ifdef USE_CLI
    cliInit(serialConfig());
#endif

    failsafeInit(rxConfig(), flight3DConfig()->deadband3d_throttle);

    rxInit(rxConfig(), modeActivationProfile()->modeActivationConditions);

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsInit(
            serialConfig(),
            gpsConfig()
        );
        navigationInit(
            gpsProfile(),
            &currentProfile->pidProfile
        );
    }
#endif

#ifdef LED_STRIP
    ledStripInit(ledStripConfig());

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        telemetryInit();
    }
#endif

#ifdef USE_ESC_SENSOR
    if (feature(FEATURE_ESC_SENSOR)) {
        escSensorInit();
    }
#endif

#ifdef USB_CABLE_DETECTION
    usbCableDetectInit();
#endif

#ifdef TRANSPONDER
    if (feature(FEATURE_TRANSPONDER)) {
        transponderInit(masterConfig.transponderData);
        transponderStartRepeating();
        systemState |= SYSTEM_STATE_TRANSPONDER_ENABLED;
    }
#endif

#ifdef USE_FLASHFS
#if defined(USE_FLASH_M25P16)
    m25p16_init(flashConfig());
#endif

    flashfsInit();
#endif

#ifdef USE_SDCARD
    if (feature(FEATURE_SDCARD)) {
        sdcardInsertionDetectInit();
        sdcard_init(sdcardConfig()->useDma);
        afatfs_init();
    }
#endif

#ifdef BLACKBOX
    initBlackbox();
#endif

    if (mixerConfig()->mixerMode == MIXER_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles();
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

#ifdef VTX_CONTROL

#ifdef VTX_SMARTAUDIO
    smartAudioInit();
#endif

#ifdef VTX_TRAMP
    trampInit();
#endif

#endif // VTX_CONTROL

    // start all timers
    // TODO - not implemented yet
    timerStart();

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

    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER))
        batteryInit(batteryConfig());

#ifdef USE_DASHBOARD
    if (feature(FEATURE_DASHBOARD)) {
#ifdef USE_OLED_GPS_DEBUG_PAGE_ONLY
        dashboardShowFixedPage(PAGE_GPS);
#else
        dashboardResetPageCycling();
        dashboardEnablePageCycling();
#endif
    }
#endif

#ifdef CJMCU
    LED2_ON;
#endif

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    motorControlEnable = true;

    fcTasksInit();
    systemState |= SYSTEM_STATE_READY;
}
