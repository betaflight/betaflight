/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/dma.h"
#include "cyw43_bus_pio_spi.pio.h"
#include "cyw43.h"
#include "cyw43_internal.h"
#include "cyw43_spi.h"
#include "cyw43_debug_pins.h"
#include "pico/cyw43_driver.h"

#if CYW43_SPI_PIO

#define IRQ_SAMPLE_DELAY_NS 100

#if !CYW43_PIN_WL_DYNAMIC && PICO_PIO_USE_GPIO_BASE
// The pins should all work in the same gpio base
static_assert((CYW43_PIN_WL_DATA_OUT < 32 && CYW43_PIN_WL_DATA_IN < 32 && CYW43_PIN_WL_CLOCK < 32) ||
    (CYW43_PIN_WL_DATA_OUT >= 16 && CYW43_PIN_WL_DATA_IN >= 16 && CYW43_PIN_WL_CLOCK >= 16), "");
#endif

#ifdef CYW43_SPI_PROGRAM_NAME
#define SPI_PROGRAM_NAME CYW43_SPI_PROGRAM_NAME
#else
//#define SPI_PROGRAM_NAME spi_gap0_sample1 // for lower cpu speed
#define SPI_PROGRAM_NAME spi_gap01_sample0 // for high cpu speed
#endif
#define SPI_PROGRAM_FUNC __CONCAT(SPI_PROGRAM_NAME, _program)
#define SPI_PROGRAM_GET_DEFAULT_CONFIG_FUNC __CONCAT(SPI_PROGRAM_NAME, _program_get_default_config)
#define SPI_OFFSET_END __CONCAT(SPI_PROGRAM_NAME, _offset_end)
#define SPI_OFFSET_LP1_END __CONCAT(SPI_PROGRAM_NAME, _offset_lp1_end)

#if !CYW43_PIO_CLOCK_DIV_DYNAMIC
#define cyw43_pio_clock_div_int CYW43_PIO_CLOCK_DIV_INT
#define cyw43_pio_clock_div_frac CYW43_PIO_CLOCK_DIV_FRAC
#else
static uint16_t cyw43_pio_clock_div_int = CYW43_PIO_CLOCK_DIV_INT;
static uint8_t cyw43_pio_clock_div_frac = CYW43_PIO_CLOCK_DIV_FRAC;

void cyw43_set_pio_clock_divisor(uint16_t clock_div_int, uint8_t clock_div_frac) {
    cyw43_pio_clock_div_int = clock_div_int;
    cyw43_pio_clock_div_frac = clock_div_frac;
}
#endif

#define PADS_DRIVE_STRENGTH PADS_BANK0_GPIO0_DRIVE_VALUE_12MA

#if !CYW43_USE_SPI
#error CYW43_USE_SPI should be true
#endif

#ifndef NDEBUG
//#define ENABLE_SPI_DUMPING 1
#endif

// Set to 1 to enable
#if ENABLE_SPI_DUMPING //NDEBUG
#if 0
#define DUMP_SPI_TRANSACTIONS(A) A
#else
static bool enable_spi_packet_dumping; // set to true to dump
#define DUMP_SPI_TRANSACTIONS(A) if (enable_spi_packet_dumping) {A}
#endif

static uint32_t counter = 0;
#else
#define DUMP_SPI_TRANSACTIONS(A)
#endif

//#define SWAP32(A) ((((A) & 0xff000000U) >> 8) | (((A) & 0xff0000U) << 8) | (((A) & 0xff00U) >> 8) | (((A) & 0xffU) << 8))
__force_inline static uint32_t __swap16x2(uint32_t a) {
    pico_default_asm ("rev16 %0, %0" : "+l" (a) : : );
    return a;
}
#define SWAP32(a) __swap16x2(a)

typedef struct {
    PIO pio;
    uint pio_offset;
    uint pio_sm;
    int8_t dma_out;
    int8_t dma_in;
} bus_data_t;

static bus_data_t bus_data_instance;

int cyw43_spi_init(cyw43_int_t *self) {
    // Only does something if CYW43_LOGIC_DEBUG=1
    logic_debug_init();

    assert(!self->bus_data);
    self->bus_data = &bus_data_instance;
    bus_data_t *bus_data = (bus_data_t *)self->bus_data;
    bus_data->pio = NULL;
    bus_data->dma_in = -1;
    bus_data->dma_out = -1;

    const uint min_gpio = MIN(CYW43_PIN_WL_CLOCK, MIN(CYW43_PIN_WL_DATA_IN, CYW43_PIN_WL_DATA_OUT));
    const uint max_gpio = MAX(CYW43_PIN_WL_CLOCK, MAX(CYW43_PIN_WL_DATA_IN, CYW43_PIN_WL_DATA_OUT));
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&SPI_PROGRAM_FUNC, &bus_data->pio, &bus_data->pio_sm, &bus_data->pio_offset, min_gpio, max_gpio - min_gpio + 1, true)) {
        cyw43_spi_deinit(self);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }
    pio_sm_config config = SPI_PROGRAM_GET_DEFAULT_CONFIG_FUNC(bus_data->pio_offset);

    sm_config_set_clkdiv_int_frac(&config, cyw43_pio_clock_div_int, cyw43_pio_clock_div_frac);
    hw_write_masked(&pads_bank0_hw->io[CYW43_PIN_WL_CLOCK],
                    (uint)PADS_DRIVE_STRENGTH << PADS_BANK0_GPIO0_DRIVE_LSB,
                    PADS_BANK0_GPIO0_DRIVE_BITS
    );
    hw_write_masked(&pads_bank0_hw->io[CYW43_PIN_WL_CLOCK],
                    (uint)1 << PADS_BANK0_GPIO0_SLEWFAST_LSB,
                    PADS_BANK0_GPIO0_SLEWFAST_BITS
    );

    sm_config_set_out_pins(&config, CYW43_PIN_WL_DATA_OUT, 1);
    sm_config_set_in_pins(&config, CYW43_PIN_WL_DATA_IN);
    sm_config_set_set_pins(&config, CYW43_PIN_WL_DATA_OUT, 1);
    sm_config_set_sideset(&config, 1, false, false);
    sm_config_set_sideset_pins(&config, CYW43_PIN_WL_CLOCK);
    sm_config_set_in_shift(&config, false, true, 32);
    sm_config_set_out_shift(&config, false, true, 32);
    hw_set_bits(&bus_data->pio->input_sync_bypass, 1u << (CYW43_PIN_WL_DATA_IN - pio_get_gpio_base(bus_data->pio)));
    pio_sm_set_config(bus_data->pio, bus_data->pio_sm, &config);
    pio_sm_set_consecutive_pindirs(bus_data->pio, bus_data->pio_sm, CYW43_PIN_WL_CLOCK, 1, true);
    gpio_set_function(CYW43_PIN_WL_DATA_OUT, pio_get_funcsel(bus_data->pio));

    // Set data pin to pull down and schmitt
    gpio_set_pulls(CYW43_PIN_WL_DATA_IN, false, true);
    gpio_set_input_hysteresis_enabled(CYW43_PIN_WL_DATA_IN, true);

    pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_set(pio_pins, 1));

    bus_data->dma_out = (int8_t) dma_claim_unused_channel(false);
    bus_data->dma_in = (int8_t) dma_claim_unused_channel(false);
    if (bus_data->dma_out < 0 || bus_data->dma_in < 0) {
        cyw43_spi_deinit(self);
        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
    }
    return 0;
}

void cyw43_spi_deinit(cyw43_int_t *self) {
    if (self->bus_data) {
        bus_data_t *bus_data = (bus_data_t *)self->bus_data;
        if (bus_data->pio) {
            pio_remove_program_and_unclaim_sm(&SPI_PROGRAM_FUNC, bus_data->pio, bus_data->pio_sm, bus_data->pio_offset);
            bus_data->pio = NULL;
        }
        if (bus_data->dma_out >= 0) {
            dma_channel_cleanup(bus_data->dma_out);
            dma_channel_unclaim(bus_data->dma_out);
            bus_data->dma_out = -1;
        }
        if (bus_data->dma_in >= 0) {
            dma_channel_cleanup(bus_data->dma_in);
            dma_channel_unclaim(bus_data->dma_in);
            bus_data->dma_in = -1;
        }
        self->bus_data = NULL;
    }
}

static void cs_set(bool value) {
    gpio_put(CYW43_PIN_WL_CS, value);
}

static __noinline void ns_delay(uint32_t ns) {
    // cycles = ns * clk_sys_hz / 1,000,000,000
    uint32_t cycles = ns * (clock_get_hz(clk_sys) >> 16u) / (1000000000u >> 16u);
    busy_wait_at_least_cycles(cycles);
}

static void start_spi_comms(cyw43_int_t *self) {
    bus_data_t *bus_data = (bus_data_t *)self->bus_data;
    gpio_set_function(CYW43_PIN_WL_DATA_OUT, pio_get_funcsel(bus_data->pio));
    gpio_set_function(CYW43_PIN_WL_CLOCK, pio_get_funcsel(bus_data->pio));
    gpio_pull_down(CYW43_PIN_WL_CLOCK);
    // Pull CS low
    cs_set(false);
}

// we need to atomically de-assert CS and enable IRQ
static void stop_spi_comms(void) {
    // from this point a positive edge will cause an IRQ to be pending
    cs_set(true);

    // we need to wait a bit in case the irq line is incorrectly high
    ns_delay(IRQ_SAMPLE_DELAY_NS);
}

#if ENABLE_SPI_DUMPING
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
    unsigned int i = 0;

    for (i = 0; i < len;) {
        if ((i & 0x0f) == 0) {
            printf("\n");
        } else if ((i & 0x07) == 0) {
            printf(" ");
        }
        printf("%02x ", bptr[i++]);
    }
    printf("\n");
}
#endif

int cyw43_spi_transfer(cyw43_int_t *self, const uint8_t *tx, size_t tx_length, uint8_t *rx,
                       size_t rx_length) {

    if ((tx == NULL) && (rx == NULL)) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    bus_data_t *bus_data = (bus_data_t *)self->bus_data;
    start_spi_comms(self);
    if (rx != NULL) {
        if (tx == NULL) {
            tx = rx;
            assert(tx_length && tx_length < rx_length);
        }
        DUMP_SPI_TRANSACTIONS(
                printf("[%lu] bus TX/RX %u bytes rx %u:", counter++, tx_length, rx_length);
                dump_bytes(tx, tx_length);
        )
        assert(!(tx_length & 3));
        assert(!(((uintptr_t)tx) & 3));
        assert(!(((uintptr_t)rx) & 3));
        assert(!(rx_length & 3));

        pio_sm_set_enabled(bus_data->pio, bus_data->pio_sm, false);
        pio_sm_set_wrap(bus_data->pio, bus_data->pio_sm, bus_data->pio_offset, bus_data->pio_offset + SPI_OFFSET_END - 1);
        pio_sm_clear_fifos(bus_data->pio, bus_data->pio_sm);
        pio_sm_set_pindirs_with_mask(bus_data->pio, bus_data->pio_sm,
            1u << (CYW43_PIN_WL_DATA_OUT - pio_get_gpio_base(bus_data->pio)),
            1u << (CYW43_PIN_WL_DATA_OUT - pio_get_gpio_base(bus_data->pio)));
        pio_sm_restart(bus_data->pio, bus_data->pio_sm);
        pio_sm_clkdiv_restart(bus_data->pio, bus_data->pio_sm);
        pio_sm_put(bus_data->pio, bus_data->pio_sm, tx_length * 8 - 1);
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_out(pio_x, 32));
        pio_sm_put(bus_data->pio, bus_data->pio_sm, (rx_length - tx_length) * 8 - 1);
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_out(pio_y, 32));
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_jmp(bus_data->pio_offset));
        dma_channel_abort(bus_data->dma_out);
        dma_channel_abort(bus_data->dma_in);

        dma_channel_config out_config = dma_channel_get_default_config(bus_data->dma_out);
        channel_config_set_bswap(&out_config, true);
        channel_config_set_dreq(&out_config, pio_get_dreq(bus_data->pio, bus_data->pio_sm, true));

        dma_channel_configure(bus_data->dma_out, &out_config, &bus_data->pio->txf[bus_data->pio_sm], tx, tx_length / 4, true);

        dma_channel_config in_config = dma_channel_get_default_config(bus_data->dma_in);
        channel_config_set_bswap(&in_config, true);
        channel_config_set_dreq(&in_config, pio_get_dreq(bus_data->pio, bus_data->pio_sm, false));
        channel_config_set_write_increment(&in_config, true);
        channel_config_set_read_increment(&in_config, false);
        dma_channel_configure(bus_data->dma_in, &in_config, rx + tx_length, &bus_data->pio->rxf[bus_data->pio_sm], rx_length / 4 - tx_length / 4, true);

        pio_sm_set_enabled(bus_data->pio, bus_data->pio_sm, true);
        __compiler_memory_barrier();

        dma_channel_wait_for_finish_blocking(bus_data->dma_out);
        dma_channel_wait_for_finish_blocking(bus_data->dma_in);

        __compiler_memory_barrier();
        memset(rx, 0, tx_length); // make sure we don't have garbage in what would have been returned data if using real SPI
    } else if (tx != NULL) {
        DUMP_SPI_TRANSACTIONS(
                printf("[%lu] bus TX only %u bytes:", counter++, tx_length);
                dump_bytes(tx, tx_length);
        )
        assert(!(((uintptr_t)tx) & 3));
        assert(!(tx_length & 3));
        pio_sm_set_enabled(bus_data->pio, bus_data->pio_sm, false);
        pio_sm_set_wrap(bus_data->pio, bus_data->pio_sm, bus_data->pio_offset, bus_data->pio_offset + SPI_OFFSET_LP1_END - 1);
        pio_sm_clear_fifos(bus_data->pio, bus_data->pio_sm);
        pio_sm_set_pindirs_with_mask(bus_data->pio, bus_data->pio_sm,
            1u << (CYW43_PIN_WL_DATA_OUT - pio_get_gpio_base(bus_data->pio)),
            1u << (CYW43_PIN_WL_DATA_OUT - pio_get_gpio_base(bus_data->pio)));
        pio_sm_restart(bus_data->pio, bus_data->pio_sm);
        pio_sm_clkdiv_restart(bus_data->pio, bus_data->pio_sm);
        pio_sm_put(bus_data->pio, bus_data->pio_sm, tx_length * 8 - 1);
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_out(pio_x, 32));
        pio_sm_put(bus_data->pio, bus_data->pio_sm, 0);
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_out(pio_y, 32));
        pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_jmp(bus_data->pio_offset));
        dma_channel_abort(bus_data->dma_out);

        dma_channel_config out_config = dma_channel_get_default_config(bus_data->dma_out);
        channel_config_set_bswap(&out_config, true);
        channel_config_set_dreq(&out_config, pio_get_dreq(bus_data->pio, bus_data->pio_sm, true));

        dma_channel_configure(bus_data->dma_out, &out_config, &bus_data->pio->txf[bus_data->pio_sm], tx, tx_length / 4, true);

        uint32_t fdebug_tx_stall = 1u << (PIO_FDEBUG_TXSTALL_LSB + bus_data->pio_sm);
        bus_data->pio->fdebug = fdebug_tx_stall;
        pio_sm_set_enabled(bus_data->pio, bus_data->pio_sm, true);
        while (!(bus_data->pio->fdebug & fdebug_tx_stall)) {
            tight_loop_contents(); // todo timeout
        }
        __compiler_memory_barrier();
        pio_sm_set_enabled(bus_data->pio, bus_data->pio_sm, false);
        pio_sm_set_consecutive_pindirs(bus_data->pio, bus_data->pio_sm, CYW43_PIN_WL_DATA_IN, 1, false);
    } else if (rx != NULL) { /* currently do one at a time */
        DUMP_SPI_TRANSACTIONS(
                printf("[%lu] bus TX %u bytes:", counter++, rx_length);
                dump_bytes(rx, rx_length);
        )
        panic_unsupported();
    }
    pio_sm_exec(bus_data->pio, bus_data->pio_sm, pio_encode_mov(pio_pins, pio_null)); // for next time we turn output on

    stop_spi_comms();
    DUMP_SPI_TRANSACTIONS(
            printf("RXed:");
            dump_bytes(rx, rx_length);
            printf("\n");
    )

    return 0;
}

// Initialise our gpios
void cyw43_spi_gpio_setup(void) {
    // Setup CYW43_PIN_WL_REG_ON (23)
    gpio_init(CYW43_PIN_WL_REG_ON);
    gpio_set_dir(CYW43_PIN_WL_REG_ON, GPIO_OUT);
    gpio_pull_up(CYW43_PIN_WL_REG_ON);

    // Setup DO, DI and IRQ (24)
    gpio_init(CYW43_PIN_WL_DATA_OUT);
    gpio_set_dir(CYW43_PIN_WL_DATA_OUT, GPIO_OUT);
    gpio_put(CYW43_PIN_WL_DATA_OUT, false);

    // Setup CS (25)
    gpio_init(CYW43_PIN_WL_CS);
    gpio_set_dir(CYW43_PIN_WL_CS, GPIO_OUT);
    gpio_put(CYW43_PIN_WL_CS, true);
}

// Reset wifi chip
void cyw43_spi_reset(void) {
    gpio_put(CYW43_PIN_WL_REG_ON, false); // off
    sleep_ms(20);
    gpio_put(CYW43_PIN_WL_REG_ON, true); // on
    sleep_ms(250);

    // Setup IRQ (24) - also used for DO, DI
    gpio_init(CYW43_PIN_WL_HOST_WAKE);
    gpio_set_dir(CYW43_PIN_WL_HOST_WAKE, GPIO_IN);
}

static inline uint32_t make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz) {
    return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

#if CYW43_VERBOSE_DEBUG
static const char *func_name(int fn) {
    switch (fn)
    {
        case BUS_FUNCTION:
            return "BUS_FUNCTION";
        case BACKPLANE_FUNCTION:
            return "BACKPLANE_FUNCTION";
        case WLAN_FUNCTION:
            return "WLAN_FUNCTION";
        default:
            return "UNKNOWN";
    }
}
#endif

uint32_t read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    uint32_t buf[2] = {0};
    assert(fn != BACKPLANE_FUNCTION);
    buf[0] = SWAP32(make_cmd(false, true, fn, reg, 4));
    int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)buf, 8);
    if (ret != 0) {
        return ret;
    }
    return SWAP32(buf[1]);
}

static inline uint32_t _cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint size) {
    // Padding plus max read size of 32 bits + another 4?
    static_assert(CYW43_BACKPLANE_READ_PAD_LEN_BYTES % 4 == 0, "");
    int index = (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4) + 1 + 1;
    uint32_t buf32[index];
    uint8_t *buf = (uint8_t *)buf32;
    const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
    buf32[0] = make_cmd(false, true, fn, reg, size);

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_READ, 1);
    }
    int ret = cyw43_spi_transfer(self, NULL, 4, buf, 8 + padding);
    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_READ, 0);
    }

    if (ret != 0) {
        return ret;
    }
    uint32_t result = buf32[padding > 0 ? index - 1 : 1];
    CYW43_VDEBUG("cyw43_read_reg_u%d %s 0x%lx=0x%lx\n", size * 8, func_name(fn), reg, result);
    return result;
}

uint32_t cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 4);
}

int cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 2);
}

int cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 1);
}

// This is only used to switch the word order on boot
int write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    uint32_t buf[2];
    // Boots up in little endian so command needs swapping too
    buf[0] = SWAP32(make_cmd(true, true, fn, reg, 4));
    buf[1] = SWAP32(val);
    int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);
    CYW43_VDEBUG("write_reg_u32_swap %s 0x%lx=0x%lx\n", func_name(fn), reg, val);
    return ret;
}

static inline int _cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val, uint size) {
    uint32_t buf[2];
    buf[0] = make_cmd(true, true, fn, reg, size);
    buf[1] = val;
    if (fn == BACKPLANE_FUNCTION) {
        // In case of f1 overflow
        self->last_size = 8;
        self->last_header[0] = buf[0];
        self->last_header[1] = buf[1];
        self->last_backplane_window = self->cur_backplane_window;
    }

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_WRITE, 1);
    }

    int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_WRITE, 0);
    }

    CYW43_VDEBUG("cyw43_write_reg_u%d %s 0x%lx=0x%lx\n", size * 8, func_name(fn), reg, val);
    return ret;
}

int cyw43_write_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 4);
}

int cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 2);
}

int cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 1);
}

#if CYW43_BUS_MAX_BLOCK_SIZE > 0x7f8
#error Block size is wrong for SPI
#endif

int cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, uint8_t *buf) {
    assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
    const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
    size_t aligned_len = (len + 3) & ~3;
    assert(aligned_len > 0 && aligned_len <= 0x7f8);
    assert(buf == self->spid_buf || buf < self->spid_buf || buf >= (self->spid_buf + sizeof(self->spid_buf)));
    self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(false, true, fn, addr, len);
    if (fn == WLAN_FUNCTION) {
        logic_debug_set(pin_WIFI_RX, 1);
    }
    int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)&self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4 + padding);
    if (fn == WLAN_FUNCTION) {
        logic_debug_set(pin_WIFI_RX, 0);
    }
    if (ret != 0) {
        CYW43_PRINTF("cyw43_read_bytes error %d", ret);
        return ret;
    }
    if (buf != self->spid_buf) { // avoid a copy in the usual case just to add the header
        memcpy(buf, self->spid_buf, len);
    }
    return 0;
}

// See whd_bus_spi_transfer_bytes
// Note, uses spid_buf if src isn't using it already
// Apart from firmware download this appears to only be used for wlan functions?
int cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, const uint8_t *src) {
    assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
    const size_t aligned_len = (len + 3) & ~3u;
    assert(aligned_len > 0 && aligned_len <= 0x7f8);
    if (fn == WLAN_FUNCTION) {
        // Wait for FIFO to be ready to accept data
        int f2_ready_attempts = 1000;
        while (f2_ready_attempts-- > 0) {
            uint32_t bus_status = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
            if (bus_status & STATUS_F2_RX_READY) {
                logic_debug_set(pin_F2_RX_READY_WAIT, 0);
                break;
            } else {
                logic_debug_set(pin_F2_RX_READY_WAIT, 1);
            }
        }
        if (f2_ready_attempts <= 0) {
            CYW43_PRINTF("F2 not ready\n");
            return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
        }
    }
    if (src == self->spid_buf) { // avoid a copy in the usual case just to add the header
        self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);
        logic_debug_set(pin_WIFI_TX, 1);
        int res = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);
        logic_debug_set(pin_WIFI_TX, 0);
        return res;
    } else {
        // todo: would be nice to get rid of this. Only used for firmware download?
        assert(src < self->spid_buf || src >= (self->spid_buf + sizeof(self->spid_buf)));
        self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);
        memcpy(self->spid_buf, src, len);
        return cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);
    }
}
#endif

#if CYW43_PIN_WL_DYNAMIC

// storage for cyw43 pins
static uint cyw43_pin_array[CYW43_PIN_INDEX_WL_COUNT] = {
    CYW43_DEFAULT_PIN_WL_REG_ON,
    CYW43_DEFAULT_PIN_WL_DATA_OUT,
    CYW43_DEFAULT_PIN_WL_DATA_IN,
    CYW43_DEFAULT_PIN_WL_HOST_WAKE,
    CYW43_DEFAULT_PIN_WL_CLOCK,
    CYW43_DEFAULT_PIN_WL_CS
};

// Check the cyw43 gpio pin array is valid
static bool cyw43_pins_valid(uint pins[CYW43_PIN_INDEX_WL_COUNT]) {
    // check the gpios are valid
    for(int i = 0; i < CYW43_PIN_INDEX_WL_COUNT; i++) {
        if (pins[i] >= NUM_BANK0_GPIOS) {
            return false;
        }
    }
#if PICO_PIO_USE_GPIO_BASE
    // These pins should use the same gpio base
    return (pins[CYW43_PIN_INDEX_WL_DATA_OUT] < 32 && pins[CYW43_PIN_INDEX_WL_DATA_IN] < 32 && pins[CYW43_PIN_INDEX_WL_CLOCK] < 32) ||
        (pins[CYW43_PIN_INDEX_WL_DATA_OUT] >= 16 && pins[CYW43_PIN_INDEX_WL_DATA_IN] >= 16 && pins[CYW43_PIN_INDEX_WL_CLOCK] >= 16);
#else
    return true;
#endif
}

// Set the gpio pin array
int cyw43_set_pins_wl(uint pins[CYW43_PIN_INDEX_WL_COUNT]) {
    assert(!bus_data_instance.pio);
    if (bus_data_instance.pio) {
        return PICO_ERROR_RESOURCE_IN_USE;
    }
    assert(cyw43_pins_valid(pins));
    if (!cyw43_pins_valid(pins)) {
        return PICO_ERROR_INVALID_ARG;
    }
    memcpy(cyw43_pin_array, pins, sizeof(cyw43_pin_array));
    return PICO_OK;
}

// Get a gpio pin
uint cyw43_get_pin_wl(cyw43_pin_index_t pin_id) {
    assert(pin_id < CYW43_PIN_INDEX_WL_COUNT);
    assert(cyw43_pin_array[pin_id] < NUM_BANK0_GPIOS);
    return cyw43_pin_array[pin_id];
}
#endif // CYW43_PIN_WL_DYNAMIC