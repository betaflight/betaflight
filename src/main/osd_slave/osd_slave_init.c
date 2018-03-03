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

#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_io.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/fc_tasks.h"
#include "fc/runtime_config.h"

#include "interface/cli.h"
#include "interface/msp.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/displayport_max7456.h"
#include "io/flashfs.h"
#include "io/ledstrip.h"
#include "io/osd_slave.h"
#include "io/serial.h"
#include "io/transponder_ir.h"

#include "osd_slave/osd_slave_init.h"

#include "pg/adc.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vcd.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build/build_config.h"
#include "build/debug.h"

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void processLoopback(void)
{
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
#ifdef USE_MAX7456
    spiPreInitCs(IO_TAG(MAX7456_SPI_CS_PIN));
#endif
}
#endif

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

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    debugMode = systemConfig()->debug_mode;

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

    delay(100);

    timerInit();  // timer must be initialized before any channel is allocated

#ifdef BUS_SWITCH_PIN
    busSwitchInit();
#endif

#if defined(USE_UART) && !defined(SITL)
    uartPinConfigure(serialPinConfig());
#endif

    serialInit(false, SERIAL_PORT_NONE);

#ifdef BEEPER
    beeperInit(beeperDevConfig());
#endif
/* temp until PGs are implemented. */
#ifdef USE_INVERTER
    initInverters();
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
#ifdef USE_SPI_DEVICE_2
    spiInit(SPIDEV_2);
#endif
#ifdef USE_SPI_DEVICE_3
    spiInit(SPIDEV_3);
#endif
#ifdef USE_SPI_DEVICE_4
    spiInit(SPIDEV_4);
#endif
#endif /* USE_SPI */

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
#endif /* USE_I2C */

#endif /* TARGET_BUS_INIT */

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#ifdef USE_ADC
    adcConfigMutable()->vbat.enabled = (batteryConfig()->voltageMeterSource == VOLTAGE_METER_ADC);
    adcConfigMutable()->current.enabled = (batteryConfig()->currentMeterSource == CURRENT_METER_ADC);

    adcConfigMutable()->rssi.enabled = feature(FEATURE_RSSI_ADC);
    adcInit(adcConfig());
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

    mspInit();
    mspSerialInit();

#ifdef USE_CLI
    cliInit(serialConfig());
#endif

    displayPort_t *osdDisplayPort = NULL;

#if defined(USE_MAX7456)
    // If there is a max7456 chip for the OSD then use it
    osdDisplayPort = max7456DisplayPortInit(vcdProfile());
    // osdInit  will register with CMS by itself.
    osdSlaveInit(osdDisplayPort);
#endif

#ifdef USE_LED_STRIP
    ledStripInit();

    if (feature(FEATURE_LED_STRIP)) {
        ledStripEnable();
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

    timerStart();

    batteryInit();

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();

    fcTasksInit();

    systemState |= SYSTEM_STATE_READY;
}
