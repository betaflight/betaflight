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
 * WS2811 LED strip on FlexPWM and eDMA: each 1.25 us reload raises DMAEN[VALDE] and the
 * eDMA writes the next width into VAL3 or VAL5; its acknowledge sets MCTRL[LDOK] and
 * clears STS[RF] (RM i.MX RT1060 Rev. 4, 55.8.48 and 55.8.19).
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"
#include "common/maths.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/nvic.h"

#include "platform/dma.h"

#include "flexpwm_imxrt.h"
#include "flexpwm_pins_imxrt.h"
#include "iomux_nxp.h"

// The high time of each bit, from the WS2812 data sheet.
#define WS2811_T0H_NS               350
#define WS2811_T1H_NS               700

// Sized for GRBW, the widest format.
#define WS2811_BITS_PER_LED_MAX     32

// Low entries after the last bit: 52.5 us, past the 50 us latch time.
#define WS2811_DELAY_BUFFER_LENGTH  42

#define WS2811_DMA_BUFFER_SIZE      (WS2811_DATA_BUFFER_SIZE * WS2811_BITS_PER_LED_MAX + WS2811_DELAY_BUFFER_LENGTH)

// One pulse width per bit in counter ticks, read by the eDMA.
static DMA_DATA_ZERO_INIT uint16_t ledStripDmaBuffer[WS2811_DMA_BUFFER_SIZE];

static ioTag_t ledStripIoTag;
static ledStripFormatRGB_e ledStripFormat;

// Kept apart from the pad so the DMA handler reads no flash table.
static PWM_Type *ledStripPwm;
static uint8_t ledStripSubmodule;
static volatile uint16_t *ledStripPulse;
static dmaIdentifier_e ledStripDma = DMA_NONE;
static uint16_t bitCompare0;
static uint16_t bitCompare1;

// The value write eDMA request of each submodule.
static const uint16_t ledStripDmaRequest[FLEXPWM_INSTANCE_COUNT][PWM_SM_COUNT] = {
    { kDmaRequestMuxFlexPWM1ValueSub0, kDmaRequestMuxFlexPWM1ValueSub1,
      kDmaRequestMuxFlexPWM1ValueSub2, kDmaRequestMuxFlexPWM1ValueSub3 },
    { kDmaRequestMuxFlexPWM2ValueSub0, kDmaRequestMuxFlexPWM2ValueSub1,
      kDmaRequestMuxFlexPWM2ValueSub2, kDmaRequestMuxFlexPWM2ValueSub3 },
    { kDmaRequestMuxFlexPWM3ValueSub0, kDmaRequestMuxFlexPWM3ValueSub1,
      kDmaRequestMuxFlexPWM3ValueSub2, kDmaRequestMuxFlexPWM3ValueSub3 },
    { kDmaRequestMuxFlexPWM4ValueSub0, kDmaRequestMuxFlexPWM4ValueSub1,
      kDmaRequestMuxFlexPWM4ValueSub2, kDmaRequestMuxFlexPWM4ValueSub3 },
};

// Nanoseconds as counter ticks, rounded to the nearest.
static uint16_t ws2811Ticks(uint32_t tickHz, uint32_t ns)
{
    return (uint16_t)(((uint64_t)tickHz * ns + 500000000u) / 1000000000u);
}

static volatile uint16_t *ws2811PulseRegister(PWM_Type *pwm, const imxrtFlexpwmPin_t *pad)
{
    return flexpwmPadIsChannelB(pad) ? &pwm->SM[pad->submodule].VAL5
                                     : &pwm->SM[pad->submodule].VAL3;
}

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat)
{
    ledStripFormat = ledFormat;
    memset(ledStripDmaBuffer, 0, sizeof(ledStripDmaBuffer));
    ledStripIoTag = ioTag;
}

void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex)
{
    unsigned bitsPerLed;
    uint32_t packedColour;

    switch (ledStripFormat) {
    case LED_RGB:       // WS2811 drivers use RGB order
        packedColour = (color->rgb.r << 16) | (color->rgb.g << 8) | color->rgb.b;
        bitsPerLed = 24;
        break;

    case LED_GRBW: {    // SK6812 drivers take a fourth, white channel
        const uint8_t white = MIN(MIN(color->rgb.r, color->rgb.g), color->rgb.b);
        packedColour = (color->rgb.g << 24) | (color->rgb.r << 16) | (color->rgb.b << 8) | white;
        bitsPerLed = 32;
        break;
    }

    case LED_GRB:       // WS2812 drivers use GRB order
    default:
        packedColour = (color->rgb.g << 16) | (color->rgb.r << 8) | color->rgb.b;
        bitsPerLed = 24;
        break;
    }

    // Most significant bit first.
    uint16_t *entry = &ledStripDmaBuffer[ledIndex * bitsPerLed];
    for (unsigned bit = bitsPerLed; bit-- > 0; ) {
        *entry++ = (packedColour & (1U << bit)) ? bitCompare1 : bitCompare0;
    }
}

static FAST_IRQ_HANDLER void ws2811DmaHandler(dmaChannelDescriptor_t *descriptor)
{
    UNUSED(descriptor);

    PWM_Type *pwm = ledStripPwm;
    const uint8_t sm = ledStripSubmodule;

    // Drop the request before stopping the submodule so no reload re-arms it.
    dmaDisableRequest(ledStripDma);
    pwm->SM[sm].DMAEN = 0;

    // The last entries were the zeroes, so the pin is already low.
    flexpwmStop(pwm, 1u << sm);

    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(void)
{
    const IO_t io = IOGetByTag(ledStripIoTag);
    const imxrtFlexpwmPin_t *pad = flexpwmFindPad(ledStripIoTag);

    if (!io || !pad || !IOIsFreeOrPreinit(io)) {
        return false;
    }

    const uint32_t clockHz = flexpwmClockHz();

    uint8_t prescaler;
    uint16_t modulo;
    if (!flexpwmDeriveTiming(clockHz, WS2811_CARRIER_HZ, &prescaler, &modulo)) {
        return false;
    }

    const uint32_t tickHz = flexpwmTickRate(clockHz, prescaler);

    PWM_Type *pwm = flexpwmPadReg(pad);

    flexpwmInstanceInit(pwm);

    if (!flexpwmSubmoduleInit(pwm, pad->submodule, prescaler, modulo)) {
        return false;
    }

    const dmaIdentifier_e identifier = dmaGetFreeIdentifier();

    if (identifier == DMA_NONE || !dmaAllocate(identifier, OWNER_LED_STRIP, 0)) {
        return false;
    }

    dmaEnable(identifier);
    dmaMuxEnable(identifier, ledStripDmaRequest[pad->instance - 1][pad->submodule]);
    dmaSetHandler(identifier, ws2811DmaHandler, NVIC_PRIO_WS2811_DMA, 0);

    IOInit(io, OWNER_LED_STRIP, 0);
    flexpwmOutputEnable(pwm, pad->submodule, flexpwmPadIsChannelB(pad));
    nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP);

    // Leaves the pin low: submodule setup staged a zero pulse.
    flexpwmLoadOk(pwm, 1u << pad->submodule);

    ledStripPwm = pwm;
    ledStripSubmodule = pad->submodule;
    ledStripPulse = ws2811PulseRegister(pwm, pad);
    ledStripDma = identifier;
    bitCompare0 = ws2811Ticks(tickHz, WS2811_T0H_NS);
    bitCompare1 = ws2811Ticks(tickHz, WS2811_T1H_NS);

    return true;
}

void ws2811LedStripStartTransfer(void)
{
    if (!ledStripPwm) {
        ws2811LedDataTransferInProgress = false;
        return;
    }

    PWM_Type *pwm = ledStripPwm;
    const uint8_t sm = ledStripSubmodule;

    const dmaTransfer_t transfer = {
        .src = ledStripDmaBuffer,
        .dst = (void *)ledStripPulse,
        .srcOffset = sizeof(ledStripDmaBuffer[0]),
        .dstOffset = 0,
        .srcSize = DMA_XFER_SIZE_2B,
        .dstSize = DMA_XFER_SIZE_2B,
        .minorBytes = sizeof(ledStripDmaBuffer[0]),
        .majorCount = WS2811_DMA_BUFFER_SIZE,
        .srcLastAdjust = 0,
        .dstLastAdjust = 0,
    };

    // The channel stops at the end of the frame (DREQ) and interrupts.
    if (!dmaSetTransfer(ledStripDma, &transfer, true)) {
        ws2811LedDataTransferInProgress = false;
        return;
    }

    // STS[RF] can only be cleared while the value request is off.
    pwm->SM[sm].STS = PWM_STS_RF_MASK;

    dmaEnableRequest(ledStripDma);

    // Starting the counter raises the reload flag, fetching the first entry.
    pwm->SM[sm].DMAEN = PWM_DMAEN_VALDE_MASK;
    flexpwmRun(pwm, 1u << sm);
}

#endif // USE_LED_STRIP
