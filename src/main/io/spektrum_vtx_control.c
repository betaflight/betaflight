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
#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)

#include <string.h>

#include "config/config.h"
#include "drivers/vtx_common.h"
#include "io/vtx.h"

#include "io/spektrum_vtx_control.h"

// We can not use the common set/get-frequncy API.
// Some VTX devices do not support it.
//#define USE_VTX_COMMON_FREQ_API

#ifdef USE_VTX_COMMON_FREQ_API
const uint16_t SpektrumVtxfrequencyTable[SPEKTRUM_VTX_BAND_COUNT][SPEKTRUM_VTX_CHAN_COUNT] =
    {
        { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
        { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
        { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
        { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
        { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
    };
#else
// Translation table, Spektrum bands to BF internal vtx_common bands
const uint8_t spek2commonBand[SPEKTRUM_VTX_BAND_COUNT]= {
    VTX_COMMON_BAND_FS,
    VTX_COMMON_BAND_RACE,
    VTX_COMMON_BAND_E,
    VTX_COMMON_BAND_B,
    VTX_COMMON_BAND_A,
};
#endif

// RF Power Index translation tables. No generic power API available.....

#ifdef USE_VTX_TRAMP
// Tramp "---", 25, 200, 400. 600 mW
const uint8_t vtxTrampPi[SPEKTRUM_VTX_POWER_COUNT] = {
                                       // Spektrum Spec    Tx menu  Tx sends   To VTX    Watt
    VTX_TRAMP_POWER_OFF,               //         Off      INHIBIT         0        0     -
    VTX_TRAMP_POWER_OFF,               //   1 -  14mW            -         -        -     -
    VTX_TRAMP_POWER_25,                //  15 -  25mW   15 -  25mW         2        1    25mW
    VTX_TRAMP_POWER_100,               //  26 -  99mW   26 -  99mW         3        2   100mW Slightly outside range
    VTX_TRAMP_POWER_200,               // 100 - 299mW  100 - 200mW         4        3   200mW
    VTX_TRAMP_POWER_400,               // 300 - 600mW  300 - 600mW         5        4   400mW
    VTX_TRAMP_POWER_600,               // 601 - max    601+ mW             6        5   600mW Slightly outside range
    VTX_TRAMP_POWER_200                // Manual               -           -        -     -
};
#endif // USE_VTX_TRAMP
//todo: enable pit mode where appropriate, for all protcols
#ifdef USE_VTX_RTC6705
// RTC6705 "---", 25 or 200 mW
const uint8_t vtxRTC6705Pi[SPEKTRUM_VTX_POWER_COUNT] = {
    VTX_6705_POWER_25,                // Off
    VTX_6705_POWER_25,                //   1 -  14mW
    VTX_6705_POWER_25,                 //  15 -  25mW
    VTX_6705_POWER_25,                 //  26 -  99mW
    VTX_6705_POWER_200,                // 100 - 299mW
    VTX_6705_POWER_200,                // 300 - 600mW
    VTX_6705_POWER_200,                // 601 - max
    VTX_6705_POWER_200                 // Manual
};
#endif //USE_VTX_RTC6705

#ifdef USE_VTX_SMARTAUDIO
// SmartAudio "---", 25, 200, 500. 800 mW
const uint8_t vtxSaPi[SPEKTRUM_VTX_POWER_COUNT] = {
    VTX_SA_POWER_OFF,                  // Off
    VTX_SA_POWER_OFF,                  //   1 -  14mW
    VTX_SA_POWER_25,                   //  15 -  25mW
    VTX_SA_POWER_25,                   //  26 -  99mW
    VTX_SA_POWER_200,                  // 100 - 299mW
    VTX_SA_POWER_500,                  // 300 - 600mW
    VTX_SA_POWER_800,                  // 601 - max
    VTX_SA_POWER_200                   // Manual
};
#endif // USE_VTX_SMARTAUDIO

uint8_t convertSpektrumVtxPowerIndex(uint8_t sPower)
{
    uint8_t devicePower = 0;

    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    switch (vtxCommonGetDeviceType(vtxDevice)) {
#ifdef USE_VTX_RTC6705
    case VTXDEV_RTC6705:
        devicePower = vtxRTC6705Pi[sPower];
        break;
#endif // USE_VTX_RTC6705

#ifdef USE_VTX_SMARTAUDIO
    case VTXDEV_SMARTAUDIO:
        devicePower = vtxSaPi[sPower];
        break;
#endif // USE_VTX_SMARTAUDIO

#ifdef USE_VTX_TRAMP
    case VTXDEV_TRAMP:
        devicePower = vtxTrampPi[sPower];
        break;
#endif // USE_VTX_TRAMP

    case VTXDEV_UNKNOWN:
    case VTXDEV_UNSUPPORTED:
    default:
        break;

    }
    return devicePower;
}

#ifdef USE_SPEKTRUM_REGION_CODES
// Just a global SpektrumRegion for now, To save VTX ctrl input to VTX tm output.
// Would need a PG item to survive power cycle. Not really used so let it be as is.
uint8_t SpektrumRegion = SPEKTRUM_VTX_REGION_NONE;
#endif

// Mark an inital invalid VTX ctrl frame to force first VTX settings cheange to actually come from Tx/Rx.
static uint32_t vtxControl_ipc = ~(SPEKTRUM_VTX_CONTROL_FRAME);

// ############ RX task ######################
void spektrumHandleVtxControl(uint32_t vtxCntrl)
{
  vtxControl_ipc = vtxCntrl;
}
// ###########################################

// ############ VTX_CONTROL task #############
void spektrumVtxControl(void)
{
    static uint32_t prevVtxControl =0;
    uint32_t vtxControl;

    // Check for invalid VTX ctrl frames
    if ((vtxControl_ipc & SPEKTRUM_VTX_CONTROL_FRAME_MASK) != SPEKTRUM_VTX_CONTROL_FRAME) return;

    vtxControl = vtxControl_ipc;
    vtxControl_ipc = 0;

    if (prevVtxControl == vtxControl) return;
    prevVtxControl = vtxControl;

    spektrumVtx_t vtx = {
        .pitMode = (vtxControl & SPEKTRUM_VTX_PIT_MODE_MASK) >> SPEKTRUM_VTX_PIT_MODE_SHIFT,
        .region  = (vtxControl & SPEKTRUM_VTX_REGION_MASK)   >> SPEKTRUM_VTX_REGION_SHIFT,
        .power   = (vtxControl & SPEKTRUM_VTX_POWER_MASK)    >> SPEKTRUM_VTX_POWER_SHIFT,
        .band    = (vtxControl & SPEKTRUM_VTX_BAND_MASK)     >> SPEKTRUM_VTX_BAND_SHIFT,
        .channel = (vtxControl & SPEKTRUM_VTX_CHANNEL_MASK)  >> SPEKTRUM_VTX_CHANNEL_SHIFT,
#ifdef USE_SPEKTRUM_REGION_CODES
        .region  = (vtxControl & SPEKTRUM_VTX_REGION_MASK)   >> SPEKTRUM_VTX_REGION_SHIFT;
#endif
    };

    const vtxSettingsConfig_t prevSettings = {
        .band    = vtxSettingsConfig()->band,
        .channel = vtxSettingsConfig()->channel,
        .freq    = vtxSettingsConfig()->freq,
        .power   = vtxSettingsConfig()->power,
        .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
    };
    vtxSettingsConfig_t newSettings = prevSettings;

    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
#ifdef USE_VTX_COMMON_FREQ_API
        uint16_t freq = SpektrumVtxfrequencyTable[vtx.band][vtx.channel];
        if (prevSettings.freq != freq) {
            newSettings.band    = VTX_COMMON_BAND_USER;
            newSettings.channel = vtx.channel;
            newSettings.freq    = freq;
        }
#else
        // Convert to the internal Common Band index
        const uint8_t band    = spek2commonBand[vtx.band];
        const uint8_t channel = vtx.channel +1; // 0 based to 1 based
        if ((prevSettings.band != band) || (prevSettings.channel != channel)) {
            newSettings.band    = band;
            newSettings.channel = channel;
            newSettings.freq    = vtxCommonLookupFrequency(vtxDevice, band, channel);
        }
#endif
        // Seems to be no unified internal VTX API standard for power levels/indexes, VTX device brand specific.
        const uint8_t power = convertSpektrumVtxPowerIndex(vtx.power);
        if (prevSettings.power != power) {
            newSettings.power   = power;
        }
        // Everyone seems to agree on what PIT ON/OFF means
        unsigned vtxCurrentStatus;
        if (vtxCommonGetStatus(vtxDevice, &vtxCurrentStatus)) {
            if ((vtxCurrentStatus & VTX_STATUS_PIT_MODE) != vtx.pitMode) {
                vtxCommonSetPitMode(vtxDevice, vtx.pitMode);
            }
        }
    }

    if (memcmp(&prevSettings,&newSettings,sizeof(vtxSettingsConfig_t))) {
        vtxSettingsConfigMutable()->band = newSettings.band;
        vtxSettingsConfigMutable()->channel = newSettings.channel;
        vtxSettingsConfigMutable()->power   = newSettings.power;
        vtxSettingsConfigMutable()->freq    = newSettings.freq;
        saveConfigAndNotify();
    }
#ifdef USE_SPEKTRUM_REGION_CODES
    // Save region code
    SpektrumRegion = vtx.region;
#endif

}

#endif // USE_SPEKTRUM_VTX_CONTROL && USE_VTX_COMMON
