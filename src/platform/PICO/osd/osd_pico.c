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

#include "platform.h"

#if ENABLE_FB_OSD

#if !(defined OSD_W_PIN && defined OSD_EN_PIN && defined OSD_SYNC_PIN)
#error This PICO OSD requires OSD_W_PIN, OSD_EN_PIN and OSD_SYNC_PIN to be defined
#endif

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "common/maths.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/osd.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "osd/osd.h"
#include "pg/vcd.h"

// pico sdk
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

// local
#include "osd_pico.h"
#include "osd_pico_internal.h"
#include "osd_tx.pio.h"

static const PIO osdPio = PIO_INSTANCE(PIO_OSD_INDEX);
static const uint osdPioIrq = PIO_IRQ_NUM(osdPio, 0);

const int fb_nx = PICO_OSD_BUF_WIDTH * 4;
const int charsPerLine = 30;

// PAL / NTSC, require initialisation
int fb_ny;
int charLines = VIDEO_LINES_PAL; // Variable, default to 16 (PAL)
int numChars;
int fb_words;

// PIO program offset and state machine.
static int osd_tx_offset;
static int osd_tx_sm;

// GPIOs for white/black, write enable, sync-detect.
static int osd_w_gpio;
static int osd_en_gpio;
static int osd_sync_gpio;
static int osdPioBase;

__attribute__((aligned(4))) static uint32_t osdBufferBackgroundW[PICO_OSD_BUF_LENGTH/4];
__attribute__((aligned(4))) static uint32_t osdBuffer1W[PICO_OSD_BUF_LENGTH/4];
__attribute__((aligned(4))) static uint32_t osdBuffer2W[PICO_OSD_BUF_LENGTH/4];
uint8_t *osdBufferBackground = (uint8_t *)osdBufferBackgroundW;
uint8_t *osdBufferA = (uint8_t *)osdBuffer1W;
static uint8_t *osdBufferB = (uint8_t *)osdBuffer2W;

static const uint32_t zero;
//static const uint32_t zero = 0xaaaaaaaa;
//static const uint32_t zero = 0x22222222;
//static const uint32_t zero = 0x88888888;
//static const uint32_t zero = 0xf2f2f2f2;

static volatile bool dmaClearBackgroundBuffer = true; // set true to enforce clear of background buffer followed by rendering of background items
static int dma_chan_bg_to_bufA;
static int dma_chan_bufB_to_fifo;

// buffer update control (avoid tearing etc.)
static volatile bool in_safe_zone;
static uint32_t safe_zone_start_us;
bool transferredSinceVsync;

#ifdef OSD_FB_PICO_DEBUG
// trace / debugging
uint32_t startVsyncCycles;
uint32_t startVsyncCyclesPrev;
int tus;
int tusr;
uint32_t maxcycles;
int nisz;
int dmb;
uint32_t maxAHI;
uint32_t renderTot;
uint32_t drawBGTot;
uint32_t drawFGTot;
uint32_t renderStartCycles;
uint32_t renderEndCycles;
uint32_t renderStartCyclesMax;
uint32_t renderEndCyclesMax;
uint32_t dd1,dd2,dd3,dd4,dd5,dd6,dd7,dd8;

static int badX;
static int badY;
static int badC = -1;
static int ddc=0;

#endif

__attribute__((aligned(4))) uint8_t osdCharBuffer[OSD_CHAR_BUFFER_LENGTH];

uint8_t osdCharLineInUse[OSD_SD_ROWS];

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c);
void osdPioWrite(uint8_t x, uint8_t y, const char *text);

static bool init_gpios(void)
{
    static bool did;
    if (!did) {
        // Insist on osd_w_gpio -> osd_en_gpio -> osd_sync_gpio being consecutive.
        osd_w_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_W_PIN));
        osd_en_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_EN_PIN));
        if (osd_en_gpio != osd_w_gpio + 1) {
            bprintf("*** OSD_EN_GPIO must be next pin up from OSD_W_GPIO (%d vs %d)", osd_en_gpio, osd_w_gpio);
            return false;
        }
        
        osd_sync_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_SYNC_PIN));
        if (osd_sync_gpio != osd_en_gpio + 1) {
            // might relax this... wait GPIO vs wait PINS if single SM, or just separate SMs
            bprintf("*** OSD_SYNC_GPIO must be next pin up from OSD_EN_GPIO (%d vs %d)", osd_sync_gpio, osd_en_gpio);
            return false;
        }

        osdPioBase = osd_sync_gpio < 32 ? 0 : 16; // Need the higher range if the highest gpio is not in the low range 0..31.
        bprintf("osd_w gpio %d, osd_en gpio %d, osd_sync gpio %d osdPioBase %d", osd_w_gpio, osd_en_gpio, osd_sync_gpio, osdPioBase);
        did = true;
    }

    return true;
}

void osdPioClearCharBuffer(void)
{
    DEBUG_INC(dd6);
    DEBUG_COUNTER_INST(c1);
    memset(osdCharBuffer, 0x20, OSD_CHAR_BUFFER_LENGTH);
    memset(osdCharLineInUse, 0, OSD_SD_ROWS);
    DEBUG_COUNTER_DIFF(dd8,c1);
}

int64_t safe_zone_callback(alarm_id_t id, void * user_data)
{
    UNUSED(id);
    UNUSED(user_data);
    in_safe_zone = false;
    return 0; // don't automatically reschedule
}

bool osdPioBufferAvailable(void)
{
    if (transferredSinceVsync) {
        // We have completed a draw / render pass since the last vsync, don't start a new one.
        return false;
    }

    if (!in_safe_zone) {
        DEBUG_INC(nisz);
        return false;
    }

    if (dma_channel_is_busy(dma_chan_bg_to_bufA)) {
        // Busy preparing osdBufferA for rendering (copying in background buffer),
        // don't allow rendering into osdBufferA until that is complete.
        DEBUG_INC(dmb);
        return false;
    }

    return true;
}

bool plotToBackground;

void selectBackgroundBuffer(void)
{
    plotToBackground = true;
}

void selectForegroundBuffer(void)
{
    plotToBackground = false;
}


static void vsync_callback(void);

static dma_channel_config config_zero_to_bg;
static dma_channel_config config_bg_to_bufA;

static bool osd_init_device(bool isPAL, int displayLines, int transferWords)
{
    fb_words = transferWords;

    // PAL field period = 20000us, NTSC ~= 16833us.
    // Disallow TRANSFER operations (render to osdBufferA) during final 1000us or so.
    safe_zone_start_us = isPAL ? 19000 : 15800;
    in_safe_zone = true;

    bprintf("OSD osd_init_device lines %d words %d", displayLines, fb_words);
    bprintf("pbw %d, pbh %d, bpl %d", PICO_OSD_BUF_WIDTH, PICO_OSD_BUF_HEIGHT_MAX, PICO_OSD_BUF_LENGTH);
    bprintf("osdBuffer1: %p osdBuffer2: %p", osdBuffer1W, osdBuffer2W);
    bprintf("nx %d, ny %d", fb_nx, fb_ny);
    memset(osdBufferA, 0, PICO_OSD_BUF_LENGTH);
    osdPioClearCharBuffer();

    if (!init_gpios()) {
        return false;
    }

    pio_set_gpio_base(osdPio, osdPioBase);
    osd_tx_offset = pio_add_program(osdPio, isPAL ? &osd_tx_pal_program : &osd_tx_ntsc_program);
    osd_tx_sm = pio_claim_unused_sm(osdPio, false);
    if (osd_tx_sm < 0) {
        bprintf("*** pico osd tx failed to claim state machine");
        return false;
    }

    // Set up for outputs from PIO
    gpio_put(osd_w_gpio, false);
    gpio_put(osd_en_gpio, false);
    pio_gpio_init(osdPio, osd_w_gpio);
    pio_gpio_init(osdPio, osd_en_gpio);

    // Default config with wrap set    
    pio_sm_config config = isPAL ? osd_tx_pal_program_get_default_config(osd_tx_offset)
        : osd_tx_ntsc_program_get_default_config(osd_tx_offset);

    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_w_gpio, 2, true /* output */);
    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_sync_gpio, 1, false /* input */);
    sm_config_set_in_pin_base(&config, osd_sync_gpio); // in PIN set SYNC (for WAIT)
    sm_config_set_in_pin_count(&config, 1);
    sm_config_set_jmp_pin(&config, osd_sync_gpio);     // jmp PIN is SYNC
    sm_config_set_set_pins(&config, osd_w_gpio, 2);    // set PIN set W, EN
    sm_config_set_out_pins(&config, osd_w_gpio, 2);    // out PIN set W, EN

    sm_config_set_out_shift(&config, true, false, 32); // No autopull
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    int pioclock = (int)75e6;
    float div = (float)SystemCoreClock / pioclock;
    bprintf("OSD device clock div = %f", (double)div);
    sm_config_set_clkdiv(&config, div);
    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);

    // prepare value for vert pixel loop
    pio_sm_put(osdPio, osd_tx_sm, displayLines - 1);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_pull(false, false));
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_mov(pio_isr, pio_osr));

    pio_set_irq0_source_enabled(osdPio, pis_interrupt0, true); // enable state machine IRQ 0 => system irq PIO_thisone_IRQ_0
    irq_set_exclusive_handler(osdPioIrq, vsync_callback);
    irq_set_enabled(osdPioIrq, true);

    dma_chan_bg_to_bufA = dma_claim_unused_channel(false);
    if (-1 == dma_chan_bg_to_bufA) {
        bprintf("**** failed to claim dma channel (bg to bufA) for osd pico");
        return false;
    }

    dma_chan_bufB_to_fifo = dma_claim_unused_channel(false);
    if (-1 == dma_chan_bufB_to_fifo) {
        bprintf("**** failed to claim dma channel (buf2 to fifo) for osd pico");
        dma_channel_unclaim(dma_chan_bg_to_bufA);
        return false;
    }

    // There is no irq to handle on dma completion, so we don't call dmaSetHandler,
    // but we do want to register ownership (shows up in cli on "dma").
    if (!dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(dma_chan_bg_to_bufA), OWNER_OSD, 1) ||
        !dmaAllocate(DMA_CHANNEL_TO_IDENTIFIER(dma_chan_bufB_to_fifo), OWNER_OSD, 2)) {
        // Unexpected
        bprintf("*** dmaAllocate failed in osd_init_device ***");
        dma_channel_unclaim(dma_chan_bg_to_bufA);
        dma_channel_unclaim(dma_chan_bufB_to_fifo);
       return false;
    }

    dma_channel_config c = dma_channel_get_default_config(dma_chan_bufB_to_fifo);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(osdPio, osd_tx_sm, true));

    dma_channel_configure(
        dma_chan_bufB_to_fifo,
        &c,
        &osdPio->txf[osd_tx_sm], // Write address (fixed PIO TX FIFO)
        NULL,                    // Read address (reset each time)
        fb_words,        // Number of transfers
        false                    // Don't start immediately
    );

    config_zero_to_bg = dma_channel_get_default_config(dma_chan_bg_to_bufA);
    channel_config_set_transfer_data_size(&config_zero_to_bg, DMA_SIZE_32);
    channel_config_set_read_increment(&config_zero_to_bg, false); // no increment when copying from the Zero word.
    channel_config_set_write_increment(&config_zero_to_bg, true);
    channel_config_set_chain_to(&config_zero_to_bg, dma_chan_bufB_to_fifo); // DMA to PIO fifo starts immediately on completion of clearing buf1

    config_bg_to_bufA = config_zero_to_bg; // Copy bg to bufA, also chains to bufB->fifo.
    channel_config_set_read_increment(&config_bg_to_bufA, true); // increment when copying from the background buffer.
    bprintf("OSD config_bg_to_bufA %08x, config_zero_to_bg %08x", config_bg_to_bufA, config_zero_to_bg);

    return true;
}

int osdPioRowsCount(void)
{
    return charLines;
}

static void clearCountProgram(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_remove_program_and_unclaim_sm(&osd_count_sync_program, osdPio, osd_tx_sm, osd_tx_offset);
}

bool osdPioStartNTSC(void)
{
    fb_ny = PICO_OSD_BUF_HEIGHT_NTSC;
    charLines = VIDEO_LINES_NTSC;
    numChars = charsPerLine * charLines;
    bprintf("OSD set NTSC buf height %d char lines %d numChars %d", fb_ny, charLines, numChars);
    clearCountProgram();
    if (!osd_init_device(false, PICO_OSD_BUF_HEIGHT_NTSC, PICO_OSD_DISPLAY_WORDS_NTSC)) {
        return false;
    }

    osdPioEnableDevice();
    return true;
}

bool osdPioStartPAL(void)
{
    fb_ny = PICO_OSD_BUF_HEIGHT_PAL;
    charLines = VIDEO_LINES_PAL;
    numChars = charsPerLine * charLines;
    bprintf("OSD set PAL buf height %d char lines %d numChars %d", fb_ny, charLines, numChars);
    clearCountProgram();
    if (!osd_init_device(true, PICO_OSD_BUF_HEIGHT_PAL, PICO_OSD_DISPLAY_WORDS_PAL)) {
        return false;
    }

    osdPioEnableDevice();
    return true;
}

static const int initLines = 1000;

bool osdPioStartDetection(void)
{
    if (!init_gpios()) {
        return false;
    }

    pio_set_gpio_base(osdPio, osdPioBase);
    osd_tx_offset = pio_add_program(osdPio, &osd_count_sync_program);
    osd_tx_sm = pio_claim_unused_sm(osdPio, false);
    pio_sm_config config = osd_count_sync_program_get_default_config(osd_tx_offset);

    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_sync_gpio, 1, false /* input */);
    sm_config_set_in_pin_base(&config, osd_sync_gpio); // in PIN set SYNC (for WAIT)
    sm_config_set_in_pin_count(&config, 1);
    sm_config_set_jmp_pin(&config, osd_sync_gpio);     // jmp PIN is SYNC

    sm_config_set_out_shift(&config, true, false, 32); // no autopull
    sm_config_set_in_shift(&config, true, false, 32); // no autopush

    int pioclock = (int)75e6;
    float div = (float)SystemCoreClock / pioclock;
    bprintf("OSD Detect pio clock div = %f", (double)div);
    sm_config_set_clkdiv(&config, div);

    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);
    // Prepare OSR with the initial hsync counter value for the decrementing loop.
    pio_sm_put(osdPio, osd_tx_sm, initLines);

    // Start counting...
    pio_sm_set_enabled(osdPio, osd_tx_sm, true);

    return true;
}

int osdPioCountHSyncs(void)
{
    // Non-blocking, understand return of 0 as invalid / not ready.
    int hsyncs = 0;
    int pc = pio_sm_get_pc(osdPio, osd_tx_sm);
    if (pc - osd_tx_offset == osd_count_sync_offset_ready) {
        // Program has reached "pull block". Extract from ISR, then restart by sending to TX fifo.
        pio_sm_clear_fifos(osdPio, osd_tx_sm);
        pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_push(false, false)); // ISR -> RX fifo.
        hsyncs = initLines - pio_sm_get(osdPio, osd_tx_sm); // read down-counter from fifo, subtract from initial value.
        pio_sm_put(osdPio, osd_tx_sm, initLines); // write initial value to fifo to restart the program.
    }

    return hsyncs;
}


#ifdef OSD_FB_PICO_DEBUG

static void vsync_callback_debug(void)
{
    ++ddc;

// #define NN 250
#define NN 50

    static uint32_t vmax = 0;
    static uint32_t szo;
    uint32_t q;
    static uint32_t qtot;
    uint32_t szn = getCycleCounter();
    int cm = ddc % NN;
    if (ddc>20) {
        q = szn-szo;
        if (cm != 1) {
            vmax = q > vmax ? q : vmax;
        }
        qtot += q;
    }

    static uint32_t n_to_c;

    dd3 = MAX(dd3,dd2); dd2 = 0;

    if (ddc % NN == 0) {
//        bprintf("%d vsync_callback busy %d %d (previous tainted n to c %d)",c, business, busybuf, n_to_c);
#if 0
        bprintf("%d vsync_callback busy %d %d nisz %d dmb %d (previous tainted n to c %d)",
                ddc, business, busybuf, nisz, dmb, n_to_c);
#endif
        nisz = 0; dmb = 0;
        // NB ave wraps quickly (~1000 vsyncs)
//        bprintf("max time between callbacks: %d, last: %d, ave: %.1f",vmax/150, q/150, (double)(((float)qtot)/c/150));
//        bprintf("tus %d, tusr %d, ave %.1f calls per VS, %.1f rds per VS, %.1f calls/rd",
//                tus, tusr,
//                (double)tus/NN, (double)tusr/NN, (double)tus/tusr);
//        bprintf("max (per rd) us per call (ave over rds) %.1f, for which painted (ave over rds) %.1f",
//                (double)maxcycles/150.0/tusr, (double)paintedmaxcycles/tusr);
#if 0
        bprintf("max us per render call (last set of vsyncs had %d complete rds) %d", tusr, maxcycles/150);
#endif
        static int printq = 0;
        if (++printq == 3) {
            printq = 0;
            bprintf("%d completed %d, ave us (duty cycle) per vsync render %d (%.1f), "
                    "start ave %.1f max %.1f, end ave %.1f max %.1f "
                    "[%d %d %d %d %d %d]",
                    ddc, tusr,
                    renderTot/(NN*150), ((double)renderTot)/(NN*(150*20000/100)),
                    ((double)renderStartCycles)/(NN*150), ((double)renderStartCyclesMax)/(150),
                    ((double)renderEndCycles)/(NN*150), ((double)renderEndCyclesMax)/(150),
                    dd1/150,dd2/150,dd3/150,dd6,dd7,dd8
                   );
        }

        dd1 = dd2 = dd3 = dd4 = dd5 = dd6 = dd7 = dd8 = 0;

#if 0
                bprintf(", fg %d (%.1f), bg %d (%.1f), fg+bg %d (%.1f)",
                drawBGTot/(NN*150), ((double)drawBGTot)/(NN*150*20000/100),
                drawFGTot/(NN*150), ((double)drawFGTot)/(NN*150*20000/100),
                (drawFGTot + drawBGTot)/(NN*150), ((double)(drawFGTot + drawBGTot))/(NN*150*20000/100));
#endif
        if (badC != -1) {
            bprintf("*** detected out of range plot, last was %d, %d, %d", badX, badY, badC);
            badC = -1;
        }

        renderStartCycles = 0; renderEndCycles = 0;
        renderStartCyclesMax = 0; renderEndCyclesMax = 0;
//        bprintf("max ah cache cycles %d", maxAHI);
        renderTot = 0; drawFGTot = 0; drawBGTot = 0;
        maxcycles = 0;
        tus = 0; tusr = 0;
        vmax = 0;
        n_to_c = getCycleCounter() - szn;
        UNUSED(n_to_c);
    }

    szo = szn;
}
#endif

static void vsync_callback(void)
{
#ifdef OSD_FB_PICO_DEBUG
    startVsyncCyclesPrev = startVsyncCycles;
    startVsyncCycles=getCycleCounter();
#endif

    // static int fieldOddEven;
    // fieldOddEven = fieldOddEven ^ 0x1;  // odd or even field (we can't tell which is which), alternate 0, 1

    // If for some reason the complete draw and render sequence takes longer than fits into the vsync period,
    // don't flip the buffers (display will be jerky but complete, no tearing / flicker)
    bool flipThisVSync = transferredSinceVsync;

    if (flipThisVSync) {
        // We have completed rendering into bufferA, rename so that's now bufferB and will be dma-d to screen.
        // Then (below) get bufferA ready for more rendering (dma background buffer into the new bufferA).
        uint8_t * tptr = osdBufferA;
        osdBufferA = osdBufferB;
        osdBufferB = tptr;
        transferredSinceVsync = false;
    }

    // Need to clear the IRQ flag state from the PIO.
    // This just writes a 1 to a register, doesn't mess with SM execution    
//    pio_interrupt_clear(osdPio, 0);

    // start new dma as soon as possible, or at any rate before doing significant update work

    // * stop any dma in progress
    // * flip buffer (or alternate buffers)
    // * reset the read address and transfer count on the channel
    // * start dma
    // * do any work to update the buffer
    // * clear pio tx fifo

//#define testdmaabort

    static int business;
    static int busybuf;



#if 0
    // Note on dma channel aborts, workaround for RP2350 erratum, in the case that dma has a handler
    // disable the channel on IRQ0
    dma_channel_set_irq0_enabled(channel, false);
    // abort the channel
    dma_channel_abort(channel);
    // clear the spurious IRQ (if there was one)
    dma_channel_acknowledge_irq0(channel);
    // re-enable the channel on IRQ0
    dma_channel_set_irq0_enabled(channel, true);
#endif


    if (dma_channel_is_busy(dma_chan_bg_to_bufA)) {
        // Unexpected, PIO shouldn't get back to vsync IRQ unless dma buf2->fifo has started
        // unless sync signals are rather mixed up (detected vsync pulse but no hsync pulses on any line)
        busybuf++;
        dma_channel_abort(dma_chan_bg_to_bufA);
    }

    // Ensure that DMA (buf2 to PIO FIFO) is not in progress, and that the PIO FIFO is empty.
    // (Can happen if some lines were skipped due to not detecting hsync pulse)
    if (dma_channel_is_busy(dma_chan_bufB_to_fifo)) {
        ++business;
        dma_channel_abort(dma_chan_bufB_to_fifo);
    }

    pio_sm_clear_fifos(osdPio, osd_tx_sm);

    // Reset the incrementing addresses
    dma_channel_set_read_addr(dma_chan_bufB_to_fifo, osdBufferB, false);

    if (dmaClearBackgroundBuffer) {
        dma_channel_configure(
            dma_chan_bg_to_bufA,    // Take over this dma channel for purpose of clearing the background buffer
            &config_zero_to_bg,     // Config (don't increment read address)
            osdBufferBackground,    // Write address
            &zero,                  // Read address
            fb_words,               // Number of transfers
            false                   // Don't start immediately
        );
        setBackgroundItemsPending();
    } else if (flipThisVSync) {
        dma_channel_configure(
            dma_chan_bg_to_bufA,
            &config_bg_to_bufA,     // Config (increment read address)
            osdBufferA,             // Write address
            osdBufferBackground,    // Read address
            fb_words,               // Number of transfers
            false                   // Don't start immediately
        );
    }
    
    if (dmaClearBackgroundBuffer || flipThisVSync) {
        // Start DMA for bg->osdBufferA (effectively clears screen buffer)
        // which chains to DMA for osdBufferB -> screen
        dma_channel_start(dma_chan_bg_to_bufA);
    } else {
        // If not updating this time, just repeat the dma copy from bufB to fifo.
        dma_channel_start(dma_chan_bufB_to_fifo);
    }

    // Reset the background clear request flag if it was set.
    dmaClearBackgroundBuffer = false;

    // Probably best clear the interrupt here at the end, just in case there are re-trigger issues if cleared earlier...
    pio_interrupt_clear(osdPio, 0);

    // Protect against starting a render operation just before a vsync callback.
    static alarm_id_t aid = -1 ;
    if (aid != -1) {
        cancel_alarm(aid);
    }
    aid = add_alarm_in_us(safe_zone_start_us, safe_zone_callback, 0, true);
    in_safe_zone = true;
    
#ifdef OSD_FB_PICO_DEBUG
    vsync_callback_debug();
#endif
}

static void enable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
    pio_gpio_init(osdPio, osd_w_gpio);
    pio_gpio_init(osdPio, osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}

// maybe don't need...
static void disable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_set(pio_pins, 0));
    gpio_init(osd_w_gpio);
    gpio_init(osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}

void osdPioEnableDevice(void) {
    enable();
}

void osdPioDisableDevice(void) {
    disable();
}

void osdPioRedrawBackground(void)
{
    bprintf("OSD osdPioRedrawBackground setting dmaClearBackgroundBuffer to true");
    dmaClearBackgroundBuffer = true;
}


void plot(int x, int y, int c)
{
    // c =  0 -> transparent (no overlay)   W=any EN=0
    // c =  1 -> black                      W=0   EN=1
    // c =  2 -> white                      W=1   EN=1

    uint8_t *plotBuffer = plotToBackground ? osdBufferBackground : osdBufferA;

    if (x<0 || y<0 || x>=fb_nx || y>=fb_ny) {
#ifdef OSD_FB_PICO_DEBUG
        badX = x;
        badY = y;
        badC = c;
#endif
        return;
    }

    uint8_t * pByte = plotBuffer + PICO_OSD_BUF_WIDTH * y;
    pByte += (int)(x/4); // 4 pixels per byte
#if 0
    if (pByte<plotBuffer || pByte>=plotBuffer + PICO_OSD_BUF_LENGTH) {
        bprintf("huh %p (%p) %d, %d, %d",pByte,plotBuffer, x,y,c);
    }
#endif
    static uint8_t masks[4] = {0b00000011, 0b00001100, 0b00110000, 0b11000000};
    static uint8_t  cols[4] = {0b00000000, 0b10101010, 0b11111111, 0b00000000};
    uint8_t mask = masks[x%4];
    uint8_t col = cols[c];
    *pByte = ((*pByte) &(~mask)) | (mask&col);
}

void hLine(int x, int y, int count, int col)
{
    for (int i=0; i<count; ++i) {
        plot(x++, y, col);
    }
}

void dhLine(int x, int y, int count)
{
    for (int i=x; i < x + count; i++) {
        plot(i, y, 2);
        plot(i, y+1, 1);
    }
}

void dvLine(int x, int y, int count)
{
    for (int i=y; i < y + count; i++) {
        plot(x, i, 2);
        plot(x+1, i, 1);
    }
}

    
// WARNING iter line functions are designed to be called iteratively, but only from one source at a time
// (only keeps track of one pixel counter, delta etc.)

typedef struct {
    int count;
    int maxCount;
    float delta;
    bool shallow;
    int ic;
    float fc;
} iterLineData_t;

static void iterLineDataInit(iterLineData_t *data, int x1, int y1, int x2, int y2)
{
    data->count = 0;
    int dx = x2 - x1;
    int dy = y2 - y1;
    bool shallow = ABS(dx) > ABS(dy);
    data->shallow = shallow;
    if (shallow) {
        data->delta = (float)dy / dx;
        if (x1 < x2) {
            data->ic = x1;
            data->fc = (float)y1;
            data->maxCount = x2 - x1 + 1;
        } else {
            data->ic = x2;
            data->fc = (float)y2;
            data->maxCount = x1 - x2 + 1;
        }
    } else {
        data->delta = dy == 0 ? 0.0f : (float)dx / dy; // cope with case of a single point.
        if (y1 < y2) {
            data->fc = (float)x1;
            data->ic = y1;
            data->maxCount = y2 - y1 + 1;
        } else {
            data->fc = (float)x2;
            data->ic = y2;
            data->maxCount = y1 - y2 + 1;
        }
    }
}

// iterLineData shared amongst all of the iter...Line functions.
static iterLineData_t iterLineData;

void iterLineInit(int x1, int y1, int x2, int y2)
{
    iterLineDataInit(&iterLineData, x1, y1, x2, y2);
}

bool iterDLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if (iterLineData.shallow) {
        plot(iterLineData.ic, iterLineData.fc, 2);
        plot(iterLineData.ic, iterLineData.fc + 1, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    } else {
        plot(iterLineData.fc, iterLineData.ic, 2);
        plot(iterLineData.fc + 1, iterLineData.ic, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    }

    iterLineData.count++;
    return false;
}

bool iterQLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if (iterLineData.shallow) {
        plot(iterLineData.ic, iterLineData.fc, 2);
        plot(iterLineData.ic, iterLineData.fc + 1, 2);
        plot(iterLineData.ic, iterLineData.fc + 2, 1);
        plot(iterLineData.ic, iterLineData.fc - 1, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    } else {
        plot(iterLineData.fc, iterLineData.ic, 2);
        plot(iterLineData.fc + 1, iterLineData.ic, 2);
        plot(iterLineData.fc + 2, iterLineData.ic, 1);
        plot(iterLineData.fc - 1, iterLineData.ic, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    }

    iterLineData.count++;
    return false;
}

bool iterDashedDLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if ((iterLineData.count % 16) < 9) {
        if (iterLineData.shallow) {
            plot(iterLineData.ic, iterLineData.fc, 2);
            plot(iterLineData.ic, iterLineData.fc + 1, 1);
        }
        else {
            plot(iterLineData.fc, iterLineData.ic, 2);
            plot(iterLineData.fc + 1, iterLineData.ic, 1);
        }
    }

    iterLineData.ic++;
    iterLineData.fc += iterLineData.delta;
    iterLineData.count++;
    return false;
}

bool iterDashedQLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if ((iterLineData.count % 16) < 9) {
        if (iterLineData.shallow) {
            plot(iterLineData.ic, iterLineData.fc, 2);
            plot(iterLineData.ic, iterLineData.fc + 1, 2);
            plot(iterLineData.ic, iterLineData.fc + 2, 1);
            plot(iterLineData.ic, iterLineData.fc - 1, 1);
        }
        else {
            plot(iterLineData.fc, iterLineData.ic, 2);
            plot(iterLineData.fc + 1, iterLineData.ic, 2);
            plot(iterLineData.fc + 2, iterLineData.ic, 1);
            plot(iterLineData.fc - 1, iterLineData.ic, 1);
        }
    }

    iterLineData.ic++;
    iterLineData.fc += iterLineData.delta;
    iterLineData.count++;
    return false;
}

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    if (x < charsPerLine && y < charLines) {
        osdCharBuffer[y*charsPerLine + x] = c;
        osdCharLineInUse[y] = 1;
    }
}

void osdPioWrite(uint8_t x, uint8_t y, const char *text)
{
    if (y < charLines) {
        osdCharLineInUse[y] = 1;
        uint8_t *p = osdCharBuffer + y * charsPerLine;
        int i=0;
        while (text[i] && x < charsPerLine) {
            p[x++] = text[i++];
        }
    }
}

#endif // ENABLE_FB_OSD
