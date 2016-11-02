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

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/assert.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "drivers/logging.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_rx.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/gyro_sync.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/io_pca9685.h"

#include "fc/fc_tasks.h"
#include "fc/msp_fc.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/motors.h"
#include "io/servos.h"
#include "fc/rc_controls.h"

#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/pwmdriver_i2c.h"
#include "io/serial_cli.h"

#include "msp/msp_serial.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"
#include "sensors/sonar.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

extern uint8_t motorControlEnable;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

typedef enum {
    SYSTEM_STATE_INITIALISING   = 0,
    SYSTEM_STATE_CONFIG_LOADED  = (1 << 0),
    SYSTEM_STATE_SENSORS_READY  = (1 << 1),
    SYSTEM_STATE_MOTORS_READY   = (1 << 2),
    SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3),
    SYSTEM_STATE_READY          = (1 << 7)
} systemState_e;

static uint8_t systemState = SYSTEM_STATE_INITIALISING;

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
    initBootlog();

    printfSupportInit();

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    addBootlogEvent2(BOOT_EVENT_CONFIG_LOADED, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    systemInit();

    i2cSetOverclock(masterConfig.i2c_overclock);

    // initialize IO (needed for all IO operations)
    IOInitGlobal();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

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
        switch (masterConfig.rxConfig.serialrx_provider) {
            case SERIALRX_SPEKTRUM1024:
            case SERIALRX_SPEKTRUM2048:
                // Spektrum satellite binding if enabled on startup.
                // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                // The rest of Spektrum initialization will happen later - via spektrumInit()
                spektrumBind(&masterConfig.rxConfig);
                break;
        }
    }
#endif

    delay(500);

    timerInit();  // timer must be initialized before any channel is allocated

    dmaInit();

#if defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(&masterConfig.serialConfig, feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(&masterConfig.serialConfig, feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(&masterConfig.serialConfig, feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    mixerInit(masterConfig.mixerMode, masterConfig.customMotorMixer);
#ifdef USE_SERVOS
    servosInit(masterConfig.customServoMixer);
#endif

    drv_pwm_config_t pwm_params;
    memset(&pwm_params, 0, sizeof(pwm_params));

#ifdef SONAR
    if (feature(FEATURE_SONAR)) {
        const sonarHcsr04Hardware_t *sonarHardware = sonarGetHardwareConfiguration(masterConfig.batteryConfig.currentMeterType);
        if (sonarHardware) {
            pwm_params.useSonar = true;
            pwm_params.sonarIOConfig.triggerTag = sonarHardware->triggerTag;
            pwm_params.sonarIOConfig.echoTag = sonarHardware->echoTag;
        }
    }
#endif

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (masterConfig.mixerMode == MIXER_AIRPLANE || masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
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
        && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC;
    pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
    pwm_params.usePPM = feature(FEATURE_RX_PPM);
    pwm_params.useSerialRx = feature(FEATURE_RX_SERIAL);

#ifdef USE_SERVOS
    pwm_params.useServos = isMixerUsingServos();
    pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
    pwm_params.servoCenterPulse = masterConfig.servoConfig.servoCenterPulse;
    pwm_params.servoPwmRate = masterConfig.servoConfig.servoPwmRate;
#endif

    pwm_params.pwmProtocolType = masterConfig.motorConfig.motorPwmProtocol;
    pwm_params.useFastPwm = (masterConfig.motorConfig.motorPwmProtocol == PWM_TYPE_ONESHOT125) || 
                            (masterConfig.motorConfig.motorPwmProtocol == PWM_TYPE_ONESHOT42) ||
                            (masterConfig.motorConfig.motorPwmProtocol == PWM_TYPE_MULTISHOT);
    pwm_params.motorPwmRate = masterConfig.motorConfig.motorPwmRate;
    pwm_params.idlePulse = masterConfig.motorConfig.mincommand;
    if (feature(FEATURE_3D)) {
        pwm_params.idlePulse = masterConfig.flight3DConfig.neutral3d;
    }

    if (masterConfig.motorConfig.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        pwm_params.useFastPwm = false;
        featureClear(FEATURE_3D);
        pwm_params.idlePulse = 0; // brushed motors
    }

#ifndef SKIP_RX_PWM_PPM
    pwmRxInit(masterConfig.inputFilteringMode);
#endif

#ifdef USE_PMW_SERVO_DRIVER
    /*
    If external PWM driver is enabled, for example PCA9685, disable internal
    servo handling mechanism, since external device will do that
    */
    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwm_params.useServos = false;
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
    beeperConfig_t beeperConfig = {
        .ioTag = IO_TAG(BEEPER),
#ifdef BEEPER_INVERTED
        .isOD = false,
        .isInverted = true
#else
        .isOD = true,
        .isInverted = false
#endif
    };
#ifdef NAZE
    if (hardwareRevision >= NAZE32_REV5) {
        // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
        beeperConfig.isOD = false;
        beeperConfig.isInverted = true;
    }
#endif

    beeperInit(&beeperConfig);
#endif

#ifdef INVERTER
    initInverter();
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
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#if defined(NAZE)
    if (hardwareRevision == NAZE32_SP) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    } else  {
        serialRemovePort(SERIAL_PORT_USART3);
    }
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL1)
#if defined(FURYF3) || defined(OMNIBUS) || defined(SPRACINGF3MINI)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif
#endif

#if defined(SONAR) && defined(USE_SOFTSERIAL2) && defined(SPRACINGF3)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#ifdef USE_I2C
#if defined(NAZE)
    if (hardwareRevision != NAZE32_SP) {
        i2cInit(I2C_DEVICE);
    } else {
        if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
            i2cInit(I2C_DEVICE);
        }
    }
#elif defined(CC3D)
    if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
        i2cInit(I2C_DEVICE);
    }
#else
    i2cInit(I2C_DEVICE);
#endif
#endif

#ifdef USE_ADC
    drv_adc_config_t adc_params;

    adc_params.enableVBat = feature(FEATURE_VBAT);
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

    initBoardAlignment(&masterConfig.boardAlignment);

#ifdef DISPLAY
    if (feature(FEATURE_DISPLAY)) {
        displayInit(&masterConfig.rxConfig);
    }
#endif

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsPreInit(&masterConfig.gpsConfig);
    }
#endif

    if (!sensorsAutodetect(&masterConfig.sensorAlignmentConfig,
            masterConfig.acc_hardware,
            masterConfig.mag_hardware,
            masterConfig.baro_hardware,
            currentProfile->mag_declination,
            masterConfig.looptime,
            masterConfig.gyro_lpf,
            masterConfig.gyroSync,
            masterConfig.gyroSyncDenominator)) {

        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }

    addBootlogEvent2(BOOT_EVENT_SENSOR_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_SENSORS_READY;

    flashLedsAndBeep();

    imuInit();

    mspSerialInit(mspFcInit());

#ifdef USE_CLI
    cliInit(&masterConfig.serialConfig);
#endif

    failsafeInit(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

    rxInit(&masterConfig.rxConfig, currentProfile->modeActivationConditions);

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        gpsInit(
            &masterConfig.serialConfig,
            &masterConfig.gpsConfig
        );

        addBootlogEvent2(BOOT_EVENT_GPS_INIT_DONE, BOOT_EVENT_FLAGS_NONE);
    }
#endif

#ifdef NAV
    navigationInit(
        &masterConfig.navConfig,
        &currentProfile->pidProfile,
        &currentProfile->rcControlsConfig,
        &masterConfig.rxConfig,
        &masterConfig.flight3DConfig,
        &masterConfig.motorConfig
    );
#endif

#ifdef LED_STRIP
    ledStripInit(masterConfig.ledConfigs, masterConfig.colors, masterConfig.modeColors, &masterConfig.specialColors);

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
#ifdef STM32F4
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
    initBlackbox();
#endif

    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

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
        batteryInit(&masterConfig.batteryConfig);

#ifdef CJMCU
    LED2_ON;
#endif

#ifdef USE_PMW_SERVO_DRIVER
    pwmDriverInitialize();
#endif

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    motorControlEnable = true;

    fcTasksInit();

    addBootlogEvent2(BOOT_EVENT_SYSTEM_READY, BOOT_EVENT_FLAGS_NONE);
    systemState |= SYSTEM_STATE_READY;
}

#ifdef SOFTSERIAL_LOOPBACK
void processLoopback(void) {
    if (loopbackPort) {
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
            uint8_t b = serialRead(loopbackPort);
            serialWrite(loopbackPort, b);
        };
    }
}
#else
#define processLoopback()
#endif

int main(void)
{
    init();
    while (true) {
        scheduler();
        processLoopback();
    }
}

#ifdef DEBUG_HARDFAULTS

//from: https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
 * hard_fault_handler_c:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void hard_fault_handler_c(unsigned long *hardfault_args)
{
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}

#else

void HardFault_Handler(void)
{
    LED2_ON;

    // fall out of the sky
    const uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors) {
        stopMotors();
    }

    LED1_OFF;
    LED0_OFF;

    while (1) {
#ifdef LED2
        delay(50);
        LED2_TOGGLE;
#endif
    }
}

#endif
