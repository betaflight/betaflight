/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ESP32-S3 peripheral register definitions.
 *
 * The ESP-IDF SOC struct headers declare peripheral register structs as
 * extern symbols (e.g. "extern gpio_dev_t GPIO;").  These symbols must
 * be defined and placed at the correct memory-mapped addresses so the
 * linker can resolve them.
 *
 * We define them here as pointers cast from the documented base addresses
 * in the ESP32-S3 Technical Reference Manual (matched by reg_base.h).
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/* Prevent our platform.h macros from conflicting with ESP-IDF externs */
#include "platform.h"
#undef UART0
#undef UART1
#undef UART2
#undef I2C0
#undef I2C1
#undef SPI0
#undef SPI1

#include "soc/gpio_struct.h"
#include "soc/uart_struct.h"
#include "soc/spi_struct.h"
#include "soc/i2c_struct.h"
#include "soc/rmt_struct.h"
#include "soc/ledc_struct.h"
#if !defined(ESP32C5) && !defined(ESP32P4)
#include "soc/sens_struct.h"
#endif
#include "soc/reg_base.h"

#if defined(ESP32S3)
#include "soc/systimer_struct.h"
#include "soc/system_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#include "soc/gdma_struct.h"
#elif defined(ESP32C5) || defined(ESP32P4)
#include "soc/systimer_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#else
#include "soc/timer_group_struct.h"
#endif

#pragma GCC diagnostic pop

/* RISC-V skeleton builds: no fixed-address peripheral mapping yet. The
 * linker scripts don't carry .peripheral_* output sections, so let these
 * shims live in .bss until a real C5/P4 driver port maps them properly. */
#if defined(ESP32C5) || defined(ESP32P4)
#define PERIPH_SECTION(name)
#else
#define PERIPH_SECTION(name) __attribute__((section(name)))
#endif

/* Each peripheral symbol is placed at the base address from reg_base.h.
 * The linker resolves e.g. &GPIO -> DR_REG_GPIO_BASE (0x60004000). */
gpio_dev_t              GPIO                PERIPH_SECTION(".peripheral_gpio")         = {};
uart_dev_t              UART0               PERIPH_SECTION(".peripheral_uart0")        = {};
uart_dev_t              UART1               PERIPH_SECTION(".peripheral_uart1")        = {};
#if !defined(ESP32C5)
uart_dev_t              UART2               PERIPH_SECTION(".peripheral_uart2")        = {};
#endif
#if defined(ESP32S3) || defined(ESP32C5) || defined(ESP32P4)
spi_dev_t               GPSPI2              PERIPH_SECTION(".peripheral_spi2")         = {};
#if !defined(ESP32C5)
spi_dev_t               GPSPI3              PERIPH_SECTION(".peripheral_spi3")         = {};
#endif
#else
/* SPI1 is the internal flash SPI, SPI2=HSPI, SPI3=VSPI */
spi_dev_t               SPI1                PERIPH_SECTION(".peripheral_spi1")         = {};
spi_dev_t               SPI2                PERIPH_SECTION(".peripheral_spi2")         = {};
spi_dev_t               SPI3                PERIPH_SECTION(".peripheral_spi3")         = {};
#endif
i2c_dev_t               I2C0                PERIPH_SECTION(".peripheral_i2c0")         = {};
#if !defined(ESP32C5)
i2c_dev_t               I2C1                PERIPH_SECTION(".peripheral_i2c1")         = {};
#endif
rmt_dev_t               RMT                 PERIPH_SECTION(".peripheral_rmt")          = {};
ledc_dev_t              LEDC                PERIPH_SECTION(".peripheral_ledc")         = {};
#if !defined(ESP32C5) && !defined(ESP32P4)
sens_dev_t              SENS                PERIPH_SECTION(".peripheral_sens")         = {};
#endif

#if defined(ESP32S3)
systimer_dev_t          SYSTIMER            PERIPH_SECTION(".peripheral_systimer")     = {};
/* RMTMEM is 4 TX + 4 RX channels × 48 items each = 384 items × 4 bytes */
uint32_t                RMTMEM[384]         PERIPH_SECTION(".peripheral_rmtmem")       = {};
system_dev_t            SYSTEM              PERIPH_SECTION(".peripheral_system")       = {};
usb_serial_jtag_dev_t   USB_SERIAL_JTAG     PERIPH_SECTION(".peripheral_usb_serial")   = {};
gdma_dev_t              GDMA                PERIPH_SECTION(".peripheral_gdma")         = {};
#elif defined(ESP32C5) || defined(ESP32P4)
systimer_dev_t          SYSTIMER            = {};
usb_serial_jtag_dev_t   USB_SERIAL_JTAG     = {};
#if defined(ESP32C5)
#include "soc/io_mux_struct.h"
#include "soc/pcr_struct.h"
uart_dev_t              LP_UART             = {};
i2c_dev_t               LP_I2C              = {};
io_mux_dev_t            IO_MUX              = {};
pcr_dev_t               PCR                 = {};
/* RMT memory window — sized for a single channel since RMT isn't yet
 * wired into BF's C5 driver path; placeholder for link only. */
uint32_t                RMTMEM[48]          = {};
#endif
#if defined(ESP32P4)
#include "soc/io_mux_struct.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/usb_wrap_struct.h"
io_mux_dev_t            IO_MUX              = {};
hp_sys_clkrst_dev_t     HP_SYS_CLKRST       = {};
usb_wrap_dev_t          USB_WRAP            = {};
uart_dev_t              UART3               = {};
uart_dev_t              UART4               = {};
uart_dev_t              LP_UART             = {};
uint32_t                RMTMEM[48]          = {};
#endif
#else
timg_dev_t              TIMERG0             PERIPH_SECTION(".peripheral_timg0")        = {};
/* ESP32 has 8 bidirectional RMT channels × 64 items each = 512 items × 4 bytes */
uint32_t                RMTMEM[512]         PERIPH_SECTION(".peripheral_rmtmem")       = {};
#endif

/* Restore platform macros */
#define UART0 (&esp32UartDev0)
#define UART1 (&esp32UartDev1)
#define UART2 (&esp32UartDev2)
#define I2C0  (&esp32I2cDev0)
#define I2C1  (&esp32I2cDev1)
#define SPI0  (&esp32SpiDev0)
#define SPI1  (&esp32SpiDev1)
