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

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "build/atomic.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/nvic.h"

#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/flash_m25p16.h"
#include "drivers/video_textscreen.h"
#include "drivers/usb_io.h"
#include "drivers/exti.h"
#include "drivers/io.h"

#include "fc/rc_controls.h" // FIXME for throttle status, not needed by OSD.

#include "osd/osd.h"
#include "osd/osd_serial.h"

#include "io/serial.h"
#include "io/flashfs.h"

#include "osd/msp_server_osd.h"
#include "msp/msp.h"
#include "msp/msp_serial.h"
#include "io/serial_cli.h"

#include "sensors/battery.h"

#include "osd/config.h"
#include "config/config_system.h"

#include "osd/osd_tasks.h"
#include "scheduler/scheduler.h"

#ifdef STM32F303xC
// from system_stm32f30x.c
void SetSysClock(void);
#endif
#ifdef STM32F10X
// from system_stm32f10x.c
void SetSysClock(bool overclock);
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .i2c_highspeed = 0,
);

typedef enum {
    SYSTEM_STATE_INITIALISING        = 0,
    SYSTEM_STATE_CONFIG_LOADED       = (1 << 0),

    SYSTEM_STATE_READY               = (1 << 7)
} systemState_e;

static uint8_t systemState = SYSTEM_STATE_INITIALISING;

void flashLed(void)
{
    LED0_OFF;
    for (uint8_t i = 0; i < 10; i++) {
        LED0_TOGGLE;
        delay(50);
    }
    LED0_OFF;
}


void init(void)
{
    printfSupportInit();

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

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
    SetSysClock(systemConfig()->emf_avoidance);
#endif
    i2cSetOverclock(systemConfig()->i2c_highspeed);

    systemInit();

    // initialize IO (needed for all IO operations)
    IOInitGlobal();

#ifdef USE_EXTI
    EXTIInit();
#endif

    ledInit(false);

    dmaInit();

    serialInit(false);

#ifdef USE_SPI
    spiInit(SPI1);
    spiInit(SPI2);
    spiInit(SPI3);
#endif

#ifdef USE_I2C
    i2cInit(I2C_DEVICE);
#endif

    drv_adc_config_t adc_params = {
        .channelMask = ADC_CHANNEL_MASK(ADC_BATTERY) | ADC_CHANNEL_MASK(ADC_CURRENT) | ADC_CHANNEL_MASK(ADC_POWER_12V) | ADC_CHANNEL_MASK(ADC_POWER_5V)
    };

    adcInit(&adc_params);

    osdInit();

    flashLed();

    mspInit();
    mspSerialInit();


#ifdef USE_FLASHFS
#if defined(USE_FLASH_M25P16)
    m25p16_init();
#endif

    flashfsInit();
#endif

    // Now that everything has powered up the voltage and cell count be determined.

    batteryInit();

    LED1_ON;  // FIXME This is a hack to enable the bus switch.

    systemState |= SYSTEM_STATE_READY;
}

void configureScheduler(void)
{
    schedulerInit();
    setTaskEnabled(TASK_SYSTEM, true);
    setTaskEnabled(TASK_CYCLE_TIME, true);
    setTaskEnabled(TASK_MSP_SERVER, true);
    setTaskEnabled(TASK_BATTERY, true);
    setTaskEnabled(TASK_STATUS_LED, true);
    setTaskEnabled(TASK_HARDWARE_WATCHDOG, true);
    setTaskEnabled(TASK_DRAW_SCREEN, true);
    setTaskEnabled(TASK_UPDATE_FC_STATE, true);

    setTaskEnabled(TASK_TEST, true);
}

int main(void) {
    init();

	configureScheduler();

    while (true) {
        scheduler();
    }
}

void HardFault_Handler(void)
{
    while (1);
}
