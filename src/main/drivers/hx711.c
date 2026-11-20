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

#include "platform.h"

#ifdef USE_HX711

#include "drivers/hx711.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "pg/hx711.h"

static IO_t sck, dout;
static bool configured, saturated;
static int32_t raw;
static uint32_t sequence, lastSampleMs, lastSampleUs;

void hx711Init(void)
{
    const hx711Config_t *config = hx711Config();
    sck = IOGetByTag(config->sckTag);
    dout = IOGetByTag(config->doutTag);
    if (!sck || !dout || sck == dout || !IOIsFreeOrPreinit(sck) || !IOIsFreeOrPreinit(dout)) {
        return;
    }
    IOInit(sck, OWNER_HX711_SCK, 0);
    IOInit(dout, OWNER_HX711_DOUT, 0);
    IOLo(sck);
    IOConfigGPIO(sck, IOCFG_OUT_PP);
    IOConfigGPIO(dout, IOCFG_IPU);
    configured = true;
}

void hx711Update(void)
{
    // Do not wait for a conversion; only clock a completed sample.
    if (!configured || IORead(dout)) {
        return;
    }
    uint32_t bits = 0;
    for (unsigned i = 0; i < 25; i++) {
        // Protect only each high pulse. HX711 powers down if SCK is high
        // for >60us. Interrupts remain enabled during conversion waiting.
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        IOHi(sck);
        delayMicroseconds(1);
        if (i < 24) {
            bits = (bits << 1) | (IORead(dout) ? 1 : 0);
        }
        IOLo(sck);
        __set_PRIMASK(primask);
        delayMicroseconds(1);
    }
    // DOUT is released high after the 25th pulse. A line stuck low, for example
    // a short or a disconnected module, must not clock out zeros and be
    // reported as a fresh sample; keep the previous reading to age out instead.
    if (!IORead(dout)) {
        return;
    }
    raw = (int32_t)(bits & 0x7fffff) - (int32_t)(bits & 0x800000);
    saturated = raw == -8388608 || raw == 8388607;
    lastSampleUs = micros();
    lastSampleMs = millis();
    sequence++;
}

void hx711GetSample(hx711Sample_t *sample)
{
    sample->ageMs = sequence ? (uint32_t)(millis() - lastSampleMs) : UINT32_MAX;
    sample->flags = (configured ? HX711_CONFIGURED : 0)
        | (sequence && sample->ageMs <= 250 ? HX711_FRESH : 0)
        | (saturated ? HX711_SATURATED : 0);
    sample->raw = raw;
    sample->sequence = sequence;
    sample->timeUs = lastSampleUs;
}
#endif
