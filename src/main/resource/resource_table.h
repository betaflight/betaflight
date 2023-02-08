/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "pg/pg.h"

#include "cms/cms.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/pwm_output_dshot_shared.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dma.h"
#include "drivers/flash.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/rangefinder/rangefinder_hcsr04.h"
#include "drivers/resource.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/sound_beeper.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/usb_msc.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/board.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/gyrodev.h"
#include "pg/max7456.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/pinio.h"
#include "pg/pin_pull_up_down.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/rx_spi_expresslrs.h"
#include "pg/serial_uart.h"
#include "pg/sdio.h"
#include "pg/timerio.h"
#include "pg/timerup.h"
#include "pg/usb.h"
#include "pg/vtx_table.h"

#include "rx/rx_bind.h"
#include "rx/rx_spi.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "resource/resource.h"

#if defined(USE_RESOURCE_MGMT)

#define RESOURCE_VALUE_MAX_INDEX(x) ((x) == 0 ? 1 : (x))

extern const resourceValue_t *resourceTable;

uint8_t resource_resourceTableLength(void);

#endif // USE_RESOURCE_MGMT


#ifdef USE_DMA_SPEC

#define MASK_IGNORED (0)

typedef struct dmaoptEntry_s {
    char *device;
    dmaPeripheral_e peripheral;
    pgn_t pgn;
    uint8_t stride;
    uint8_t offset;
    uint8_t maxIndex;
    uint32_t presenceMask;
} dmaoptEntry_t;

extern const dmaoptEntry_t *dmaoptEntryTable;

uint8_t resource_dmaoptEntryTableLength(void);

#endif // USE_DMA_SPEC
