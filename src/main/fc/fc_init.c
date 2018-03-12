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

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/buttons.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/flash_m25p16.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/sdcard.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/exti.h"
#include "drivers/usb_io.h"
#include "drivers/vtx_rtc6705.h"
#include "drivers/vtx_common.h"
#ifdef USE_DMA_SPI_DEVICE
#include "drivers/dma_spi.h"
#endif //USE_DMA_SPI_DEVICE

#include "fc/config.h"
#include "fc/fc_init.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "interface/cli.h"
#include "interface/msp.h"

#include "msp/msp_serial.h"

#include "pg/adc.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/flash.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/pg.h"
#include "pg/rx_pwm.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/displayport_max7456.h"
#include "io/displayport_rcdevice.h"
#include "io/displayport_srxl.h"
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
#include "io/osd_slave.h"
#include "io/piniobox.h"
#include "io/displayport_msp.h"
#include "io/vtx.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_tramp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/initialisation.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/rcdevice_cam.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build/build_config.h"
#include "build/debug.h"

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

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

#ifdef BUS_SWITCH_PIN
void busSwitchInit(void)
{
static IO_t busSwitchResetPin        = IO_NONE;

    busSwitchResetPin = IOGetByTag(IO_TAG(BUS_SWITCH_PIN));
    IOInit(busSwitchResetPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(busSwitchResetPin, IOCFG_OUT_PP);

    // ENABLE
    IOLo(busSwitchResetPin);
}
#endif

#ifdef USE_SPI
// Pre-initialize all CS pins to input with pull-up.
// It's sad that we can't do this with an initialized array,
// since we will be taking care of configurable CS pins shortly.

void spiPreInit(void)
{
#ifdef USE_ACC_SPI_MPU6000
    spiPreInitCs(IO_TAG(MPU6000_CS_PIN));
#endif
#ifdef USE_ACC_SPI_MPU6500
    spiPreInitCs(IO_TAG(MPU6500_CS_PIN));
#endif
#ifdef USE_GYRO_SPI_MPU9250
    spiPreInitCs(IO_TAG(MPU9250_CS_PIN));
#endif
#ifdef USE_GYRO_SPI_ICM20649
    spiPreInitCs(IO_TAG(ICM20649_CS_PIN));
#endif
#ifdef USE_GYRO_SPI_ICM20689
    spiPreInitCs(IO_TAG(ICM20689_CS_PIN));
#endif
#ifdef USE_ACCGYRO_BMI160
    spiPreInitCs(IO_TAG(BMI160_CS_PIN));
#endif
#ifdef USE_GYRO_L3GD20
    spiPreInitCs(IO_TAG(L3GD20_CS_PIN));
#endif
#ifdef USE_MAX7456
    spiPreInitCsOutPU(IO_TAG(MAX7456_SPI_CS_PIN)); // XXX 3.2 workaround for Kakute F4. See comment for spiPreInitCSOutPU.
#endif
#ifdef USE_SDCARD 
    spiPreInitCs(sdcardConfig()->chipSelectTag);
#endif
#ifdef USE_BARO_SPI_BMP280
    spiPreInitCs(IO_TAG(BMP280_CS_PIN));
#endif
#ifdef USE_BARO_SPI_MS5611
    spiPreInitCs(IO_TAG(MS5611_CS_PIN));
#endif
#ifdef USE_BARO_SPI_LPS
    spiPreInitCs(IO_TAG(LPS_CS_PIN));
#endif
#ifdef USE_MAG_SPI_HMC5883
    spiPreInitCs(IO_TAG(HMC5883_CS_PIN));
#endif
#ifdef USE_MAG_SPI_AK8963
    spiPreInitCs(IO_TAG(AK8963_CS_PIN));
#endif
#if defined(USE_VTX_RTC6705)
    spiPreInitCs(IO_TAG(RTC6705_CS_PIN));
#endif
#ifdef USE_FLASH_M25P16
    spiPreInitCs(IO_TAG(M25P16_CS_PIN));
#endif
#if defined(USE_RX_SPI) && !defined(USE_RX_SOFTSPI)
    spiPreInitCs(IO_TAG(RX_NSS_PIN));
#endif
}
#endif

void rebootUpdater(void)
{
    #ifdef UPT_ADDRESS
    typedef void (*pFunction)(void);
   	pFunction JumpToApplication;
	uint32_t jumpAddress;

    __disable_irq(); // disable interrupts for jump

    jumpAddress = *(__IO uint32_t*)(UPT_ADDRESS + 4);
    JumpToApplication = (pFunction)jumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(__IO uint32_t*)UPT_ADDRESS);
    JumpToApplication();
    #endif
}

void rebootMsd(void)
{
    #ifdef MSD_ADDRESS
    typedef void (*pFunction)(void);
   	pFunction JumpToApplication;
	uint32_t jumpAddress;

    __disable_irq(); // disable interrupts for jump

    jumpAddress = *(__IO uint32_t*)(MSD_ADDRESS + 4);
    JumpToApplication = (pFunction)jumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(__IO uint32_t*)MSD_ADDRESS);
    JumpToApplication();
    #endif
}
void init(void)
{
#ifdef USE_ITCM_RAM
    /* Load functions into ITCM RAM */
    extern unsigned char tcm_code_start;
    extern unsigned char tcm_code_end;
    extern unsigned char tcm_code;
    memcpy(&tcm_code_start, &tcm_code, (int)(&tcm_code_end - &tcm_code_start));
#endif

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

#ifdef USE_BRUSHED_ESC_AUTODETECT
    detectBrushedESC();
#endif

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    // !!TODO: Check to be removed when moving to generic targets
    if (strncasecmp(systemConfig()->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER))) {
        resetEEPROM();
    }

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    //i2cSetOverclock(masterConfig.i2c_overclock);

    debugMode = systemConfig()->debug_mode;

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();

#ifdef TARGET_PREINIT
    targetPreInit();
#endif

#if !defined(UNIT_TEST) && !defined(USE_FAKE_LED)
    ledInit(statusLedConfig());
#endif
    LED2_ON;

#ifdef USE_EXTI
    EXTIInit();
#endif

#if defined(USE_BUTTONS)

    buttonsInit();

    // Check status of bind plug and exit if not active
    delayMicroseconds(10);  // allow configuration to settle

    if (!isMPUSoftReset()) {
#if defined(BUTTON_A_PIN) && defined(BUTTON_B_PIN)
        // two buttons required
        uint8_t secondsRemaining = 5;
        bool bothButtonsHeld;
        do {
            bothButtonsHeld = buttonAPressed() && buttonBPressed();
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

#if defined(USE_SPEKTRUM_BIND)
    if (feature(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
        case SERIALRX_SRXL:
            // Spektrum satellite binding if enabled on startup.
            // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
            // The rest of Spektrum initialization will happen later - via spektrumInit()
            spektrumBind(rxConfigMutable());
            break;
        }
    }
#endif

#ifdef USE_OVERCLOCK
    OverclockRebootIfNecessary(systemConfig()->cpu_overclock);
#endif

    delay(100);

    timerInit();  // timer must be initialized before any channel is allocated

#ifdef BUS_SWITCH_PIN
    busSwitchInit();
#endif

#if defined(USE_UART)
    uartPinConfigure(serialPinConfig());
#endif

#if defined(AVOID_UART1_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART1 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    mixerInit(mixerConfig()->mixerMode);
    mixerConfigureOutput();

    uint16_t idlePulse = motorConfig()->mincommand;
    if (feature(FEATURE_3D)) {
        idlePulse = flight3DConfig()->neutral3d;
    }
    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureClear(FEATURE_3D);
        idlePulse = 0; // brushed motors
    }
    /* Motors needs to be initialized soon as posible because hardware initialization
     * may send spurious pulses to esc's causing their early initialization. Also ppm
     * receiver may share timer with motors so motors MUST be initialized here. */
    motorDevInit(&motorConfig()->dev, idlePulse, getMotorCount());
    systemState |= SYSTEM_STATE_MOTORS_READY;

    if (0) {}
#if defined(USE_PPM)
    else if (feature(FEATURE_RX_PPM)) {
        ppmRxInit(ppmConfig());
    }
#endif
#if defined(USE_PWM)
    else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        pwmRxInit(pwmConfig());
    }
#endif

#ifdef BEEPER
    beeperInit(beeperDevConfig());
#endif
/* temp until PGs are implemented. */
#if defined(USE_INVERTER) && !defined(SITL)
    initInverters(serialPinConfig());
#endif

#ifdef TARGET_BUS_INIT
    targetBusInit();
#else

#ifdef USE_SPI
    spiPinConfigure(spiPinConfig());

    // Initialize CS lines and keep them high
    spiPreInit();

#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1);
#endif
#ifdef USE_DMA_SPI_DEVICE
    dmaSpiInit();
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
#endif // USE_SPI

#ifdef USE_I2C
    i2cHardwareConfigure(i2cConfig());

    // Note: Unlike UARTs which are configured when client is present,
    // I2C buses are initialized unconditionally if they are configured.

#ifdef USE_I2C_DEVICE_1
    i2cInit(I2CDEV_1);
#endif
#ifdef USE_I2C_DEVICE_2
    i2cInit(I2CDEV_2);
#endif
#ifdef USE_I2C_DEVICE_3
    i2cInit(I2CDEV_3);
#endif
#ifdef USE_I2C_DEVICE_4
    i2cInit(I2CDEV_4);
#endif
#endif // USE_I2C

#endif // TARGET_BUS_INIT

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#ifdef USE_VTX_RTC6705
    rtc6705IOInit();
#endif

#ifdef USE_CAMERA_CONTROL
    cameraControlInit();
#endif

// XXX These kind of code should goto target/config.c?
// XXX And these no longer work properly as FEATURE_RANGEFINDER does control HCSR04 runtime configuration.
#if defined(RANGEFINDER_HCSR04_SOFTSERIAL2_EXCLUSIVE) && defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL2)
    if (feature(FEATURE_RANGEFINDER) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
    }
#endif

#if defined(RANGEFINDER_HCSR04_SOFTSERIAL1_EXCLUSIVE) && defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_RANGEFINDER) && feature(FEATURE_SOFTSERIAL)) {
        serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
    }
#endif

#ifdef USE_ADC
    adcConfigMutable()->vbat.enabled = (batteryConfig()->voltageMeterSource == VOLTAGE_METER_ADC);
    adcConfigMutable()->current.enabled = (batteryConfig()->currentMeterSource == CURRENT_METER_ADC);

    // The FrSky D SPI RX sends RSSI_ADC_PIN (if configured) as A2
    adcConfigMutable()->rssi.enabled = feature(FEATURE_RSSI_ADC) || (feature(FEATURE_RX_SPI) && rxConfig()->rx_spi_protocol == RX_SPI_FRSKY_D);
    adcInit(adcConfig());
#endif

    initBoardAlignment(boardAlignment());

    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, notify and don't arm.
        indicateFailure(FAILURE_MISSING_ACC, 2);
        setArmingDisabled(ARMING_DISABLED_NO_GYRO);
    }

#ifdef USE_GYRO_IMUF9001
    if(!gyroIsSane())
    {
        setArmingDisabled(ARMING_DISABLED_NO_GYRO);
    }
#endif

    systemState |= SYSTEM_STATE_SENSORS_READY;

    // gyro.targetLooptime set in sensorsAutodetect(),
    // so we are ready to call validateAndFixGyroConfig(), pidInit(), and setAccelerationFilter()
    validateAndFixGyroConfig();
    pidInit(currentPidProfile);
    accInitFilters();

#ifdef USE_SERVOS
    servosInit();
    servoConfigureOutput();
    if (isMixerUsingServos()) {
        //pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
        servoDevInit(&servoConfig()->dev);
    }
    servosFilterInit();
#endif

#ifdef USE_PINIO
    pinioInit(pinioConfig());
#endif

#ifdef USE_PINIOBOX
    pinioBoxInit(pinioBoxConfig());
#endif

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

    imuInit();

    mspInit();
    mspSerialInit();

#ifdef USE_CLI
    cliInit(serialConfig());
#endif

    failsafeInit();

    rxInit();

/*
 * CMS, display devices and OSD
 */
#ifdef USE_CMS
    cmsInit();
#endif

#if (defined(USE_OSD) || (defined(USE_MSP_DISPLAYPORT) && defined(USE_CMS)) || defined(USE_OSD_SLAVE))
    displayPort_t *osdDisplayPort = NULL;
#endif

#if defined(USE_OSD) && !defined(USE_OSD_SLAVE)
    //The OSD need to be initialised after GYRO to avoid GYRO initialisation failure on some targets

    if (feature(FEATURE_OSD)) {
#if defined(USE_MAX7456)
        // If there is a max7456 chip for the OSD then use it
        osdDisplayPort = max7456DisplayPortInit(vcdProfile());
#elif defined(USE_OSD_OVER_MSP_DISPLAYPORT) // OSD over MSP; not supported (yet)
        osdDisplayPort = displayPortMspInit();
#endif
        // osdInit  will register with CMS by itself.
        osdInit(osdDisplayPort);
    }
#endif

#if defined(USE_OSD_SLAVE) && !defined(USE_OSD)
#if defined(USE_MAX7456)
    // If there is a max7456 chip for the OSD then use it
    osdDisplayPort = max7456DisplayPortInit(vcdProfile());
    // osdInit  will register with CMS by itself.
    osdSlaveInit(osdDisplayPort);
#endif
#endif

#if defined(USE_CMS) && defined(USE_MSP_DISPLAYPORT)
    // If BFOSD is not active, then register MSP_DISPLAYPORT as a CMS device.
    if (!osdDisplayPort)
        cmsDisplayPortRegister(displayPortMspInit());
#endif

#ifdef USE_DASHBOARD
    // Dashbord will register with CMS by itself.
    if (feature(FEATURE_DASHBOARD)) {
        dashboardInit();
    }
#endif

#if defined(USE_CMS) && defined(USE_SPEKTRUM_CMS_TELEMETRY)
    // Register the srxl Textgen telemetry sensor as a displayport device
    cmsDisplayPortRegister(displayPortSrxlInit());
#endif

#ifdef USE_GPS
    if (feature(FEATURE_GPS)) {
        gpsInit();
#ifdef USE_NAV
        navigationInit();
#endif
    }
#endif

#ifdef USE_LED_STRIP
    ledStripInit();

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

#ifdef USE_TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        telemetryInit();
    }
#endif

#ifdef USE_ESC_SENSOR
    if (feature(FEATURE_ESC_SENSOR)) {
        escSensorInit();
    }
#endif

#ifdef USB_DETECT_PIN
    usbCableDetectInit();
#endif

#ifdef USE_TRANSPONDER
    if (feature(FEATURE_TRANSPONDER)) {
        transponderInit();
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
    if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
        if (sdcardConfig()->enabled) {
            sdcardInsertionDetectInit();
            sdcard_init(sdcardConfig());
            afatfs_init();
        } else {
            blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        }
    }
#endif

#ifdef USE_BLACKBOX
    blackboxInit();
#endif

    if (mixerConfig()->mixerMode == MIXER_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroStartCalibration(false);
#ifdef USE_BARO
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
#endif

#ifdef USE_VTX_CONTROL
    vtxControlInit();

#if defined(USE_VTX_COMMON)
    vtxCommonInit();
    vtxInit();
#endif

#ifdef USE_VTX_SMARTAUDIO
    vtxSmartAudioInit();
#endif

#ifdef USE_VTX_TRAMP
    vtxTrampInit();
#endif

#ifdef USE_VTX_RTC6705
#ifdef VTX_RTC6705_OPTIONAL
    if (!vtxCommonDevice()) // external VTX takes precedence when configured.
#endif
    {
        vtxRTC6705Init();
    }
#endif

#endif // VTX_CONTROL

    // start all timers
    // TODO - not implemented yet
    timerStart();

    ENABLE_STATE(SMALL_ANGLE);

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif

    batteryInit(); // always needs doing, regardless of features.

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

#ifdef USE_RCDEVICE
    rcdeviceInit();
#endif // USE_RCDEVICE

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    pwmEnableMotors();

    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    fcTasksInit();

    systemState |= SYSTEM_STATE_READY;
}
