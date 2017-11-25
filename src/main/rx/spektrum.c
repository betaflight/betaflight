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
#include "string.h"
#include "platform.h"
#include "common/maths.h"

#ifdef USE_SERIAL_RX

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_string.h"

#include "fc/config.h"
#include "fc/fc_dispatch.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "config/feature.h"

// driver for spektrum satellite receiver / sbus

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7

#define SPEKTRUM_NEEDED_FRAME_INTERVAL  5000
#define SPEKTRUM_TELEMETRY_FRAME_DELAY  1000   // Gap between received Rc frame and transmited TM frame, uS

#define SPEKTRUM_BAUDRATE               115200

#define SPEKTRUM_MAX_FADE_PER_SEC       40
#define SPEKTRUM_FADE_REPORTS_PER_SEC   2

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;
static bool srxlEnabled = false;
static int32_t resolution;

#ifdef USE_SPEKTRUM_FAKE_RSSI
// Spektrum Rx type. Determined by bind method.
static bool spektrumSatInternal = true; // Assume internal,bound by BF.

// Variables used for calculating a signal strength from satellite fade.
//  This is time-variant and computed every second based on the fade
//  count over the last second.
static uint32_t spek_fade_last_sec = 0; // Stores the timestamp of the last second.
static uint16_t spek_fade_last_sec_count = 0; // Stores the fade count at the last second.
#endif

static uint8_t rssi_channel; // Stores the RX RSSI channel.

static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

static rxRuntimeConfig_t *rxRuntimeConfigPtr;
static serialPort_t *serialPort;

static uint8_t telemetryBuf[SRXL_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;

void srxlRxSendTelemetryDataDispatch(dispatchEntry_t *self);

// Linear mapping and interpolation function
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION

// Conversion table from dBm to a percentage scale aproximating a more linear RSSI vs Distance curve.

static const stru_dbm_table dbmTable[] = {
  {SPEKTRUM_RSSI_MAX, 101},
  {-49,	100},
  {-56, 98},
  {-61,	95},
  {-66,	89},
  {-69,	83},
  {-71,	78},
  {-73,	72},
  {-74,	69},
  {-75,	66},
  {-76,	63},
  {-77,	60},
#if 0 // Linear part of the table, can be interpolated
  {-78,	56},
  {-79,	52},
  {-80,	48},
  {-81,	44},
  {-82,	40},
  {-83,	36},
  {-84,	32},
  {-85,	28},
  {-86,	24},
  {-87,	20}, // Beta Flight default RSSI % alatm point
  {-88,	16},
  {-89,	12},
  {-90,	 8}, // Failsafe usually hits here
  {-91,	 4},
#endif // Linear part of the table, end
  {SPEKTRUM_RSSI_MIN, 0}};

#define SIZEOF_dbmTable  (sizeof(dbmTable)/sizeof(dbmTable[0]))

// Convert dBm to Range %
static int8_t dBm2range (int8_t dBm)
{
  int8_t  retval = dbmTable[0].reportAs;
  uint8_t i = 1;

  while (i < SIZEOF_dbmTable) {
    if (dBm >= dbmTable[i].dBm) {
      // Linear interpolation between table points.
      retval = map(dBm, dbmTable[i-1].dBm, dbmTable[i].dBm, dbmTable[i-1].reportAs, dbmTable[i].reportAs);
      i = SIZEOF_dbmTable;
    }
    i++;
  }

  if (retval < 0) {
    retval = 0;
  }
  else if (retval > 100) {
    retval = 100;
  }
  return (retval);
}
#endif

#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(VTX_COMMON)

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
    const uint8_t spek2commonBand[]= {
      VTX_COMMON_BAND_FS,
      VTX_COMMON_BAND_RACE,
      VTX_COMMON_BAND_E,
      VTX_COMMON_BAND_B,
      VTX_COMMON_BAND_A,
    };
#endif

    // RF Power Index translation tables. No generic power API available.....

    // Tramp "---", 25, 200, 400. 600 mW
const uint8_t vtxTrampPi[] = {           // Spektrum Spec    Tx menu  Tx sends   To VTX    Wand
      VTX_TRAMP_POWER_OFF,               //         Off      INHIBIT         0        0     -
      VTX_TRAMP_POWER_OFF,               //   1 -  14mW            -         -        -     -
      VTX_TRAMP_POWER_25,                //  15 -  25mW   15 -  25mW         2        1    25mW
      VTX_TRAMP_POWER_100,               //  26 -  99mW   26 -  99mW         3        2   100mW Slightly outside range
      VTX_TRAMP_POWER_200,               // 100 - 299mW  100 - 200mW         4        3   200mW
      VTX_TRAMP_POWER_400,               // 300 - 600mW  300 - 600mW         5        4   400mW
      VTX_TRAMP_POWER_600,               // 601 - max    601+ mW             6        5   600mW Slightly outside range
      VTX_TRAMP_POWER_200                // Manual               -           -        -     -
    };

    // RTC6705 "---", 25 or 200 mW
    const uint8_t vtxRTC6705Pi[] = {
      VTX_6705_POWER_OFF,                // Off
      VTX_6705_POWER_OFF,                //   1 -  14mW
      VTX_6705_POWER_25,                 //  15 -  25mW
      VTX_6705_POWER_25,                 //  26 -  99mW
      VTX_6705_POWER_200,                // 100 - 299mW
      VTX_6705_POWER_200,                // 300 - 600mW
      VTX_6705_POWER_200,                // 601 - max
      VTX_6705_POWER_200                 // Manual
    };

    // SmartAudio "---", 25, 200, 500. 800 mW
    const uint8_t vtxSaPi[] = {
      VTX_SA_POWER_OFF,                  // Off
      VTX_SA_POWER_OFF,                  //   1 -  14mW
      VTX_SA_POWER_25,                   //  15 -  25mW
      VTX_SA_POWER_25,                   //  26 -  99mW
      VTX_SA_POWER_200,                  // 100 - 299mW
      VTX_SA_POWER_500,                  // 300 - 600mW
      VTX_SA_POWER_800,                  // 601 - max
      VTX_SA_POWER_200                   // Manual
    };

    uint8_t convertSpektrumVtxPowerIndex(uint8_t sPower)
    {
      uint8_t devicePower = 0;

      switch (vtxCommonGetDeviceType()) {
      case VTXDEV_RTC6705:
        devicePower = vtxRTC6705Pi[sPower];
        break;

      case VTXDEV_SMARTAUDIO:
        devicePower = vtxSaPi[sPower];
        break;

      case VTXDEV_TRAMP:
        devicePower = vtxTrampPi[sPower];
        break;

      case VTXDEV_UNKNOWN:
      case VTXDEV_UNSUPPORTED:
      default:
        break;

      }
      return devicePower;
    }

    void handleSpektrumVtxControl(uint32_t vtxControl)
    {
      stru_vtx vtx;

      vtx.pitMode = (vtxControl & SPEKTRUM_VTX_PIT_MODE_MASK) >> SPEKTRUM_VTX_PIT_MODE_SHIFT;;
      vtx.region  = (vtxControl & SPEKTRUM_VTX_REGION_MASK)   >> SPEKTRUM_VTX_REGION_SHIFT;
      vtx.power   = (vtxControl & SPEKTRUM_VTX_POWER_MASK)    >> SPEKTRUM_VTX_POWER_SHIFT;
      vtx.band    = (vtxControl & SPEKTRUM_VTX_BAND_MASK)     >> SPEKTRUM_VTX_BAND_SHIFT;
      vtx.channel = (vtxControl & SPEKTRUM_VTX_CHANNEL_MASK)  >> SPEKTRUM_VTX_CHANNEL_SHIFT;

      const vtxSettingsConfig_t prevSettings = {
        .band = vtxSettingsConfig()->band,
        .channel = vtxSettingsConfig()->channel,
        .freq = vtxSettingsConfig()->freq,
        .power = vtxSettingsConfig()->power,
        .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
      };
      vtxSettingsConfig_t newSettings = prevSettings;

#ifdef USE_VTX_COMMON_FREQ_API
      uint16_t freq = SpektrumVtxfrequencyTable[vtx.band][vtx.channel];
      if (vtxCommonDeviceRegistered()) {
        if (prevSettings.freq != freq) {
          newSettings.band = VTX_COMMON_BAND_USER;
          newSettings.channel = vtx.channel;
          newSettings.freq = freq;
        }
      }

#else
      // Convert to the internal Common Band index
      uint8_t band    = spek2commonBand[vtx.band];
      uint8_t channel = vtx.channel +1; // 0 based to 1 based
      if (vtxCommonDeviceRegistered()) {
        if ((prevSettings.band != band) || (prevSettings.channel != channel)) {
          newSettings.band = band;
          newSettings.channel = channel;
          newSettings.freq = vtx58_Bandchan2Freq(band, channel);
        }
      }
#endif

      // Seems to be no unified internal VTX API std for popwer levels/indexes, VTX device brand specific.
      uint8_t power = convertSpektrumVtxPowerIndex(vtx.power);
      if (vtxCommonDeviceRegistered()) {
        if (prevSettings.power != power) {
          newSettings.power = power;
        }
      }

      // Everyone seems to agree on what PIT ON/OFF means
      uint8_t currentPitMode = 0;
      if (vtxCommonDeviceRegistered()) {
        vtxCommonGetPitMode(&currentPitMode);
        if (currentPitMode != vtx.pitMode) {
            vtxCommonSetPitMode(vtx.pitMode);
        }
      }

      if(memcmp(&prevSettings,&newSettings,sizeof(vtxSettingsConfig_t))) {
          vtxSettingsConfigMutable()->band = newSettings.band;
          vtxSettingsConfigMutable()->channel = newSettings.channel;
          vtxSettingsConfigMutable()->power = newSettings.power;
          vtxSettingsConfigMutable()->freq = newSettings.freq;
          saveConfigAndNotify();
      }
    }

#endif // USE_SPEKTRUM_VTX_CONTROL && VTX_COMMON


// Receive ISR callback
static void spektrumDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint32_t spekTime, spekTimeInterval;
    static uint32_t spekTimeLast = 0;
    static uint8_t spekFramePosition = 0;

    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;

    if (spekTimeInterval > SPEKTRUM_NEEDED_FRAME_INTERVAL) {
        spekFramePosition = 0;
    }

    if (spekFramePosition < SPEK_FRAME_SIZE) {
        spekFrame[spekFramePosition++] = (uint8_t)c;
        if (spekFramePosition < SPEK_FRAME_SIZE) {
            rcFrameComplete = false;
        } else {
            rcFrameComplete = true;
        }
    }
}

static uint32_t spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];
static dispatchEntry_t srxlTelemetryDispatch = { .dispatch = srxlRxSendTelemetryDataDispatch};

static uint8_t spektrumFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (!rcFrameComplete) {
        return RX_FRAME_PENDING;
    }

    rcFrameComplete = false;

#ifdef USE_SPEKTRUM_REAL_RSSI
    static int8_t spek_last_rssi = SPEKTRUM_RSSI_MAX;

    // Fetch RSSI
    if (srxlEnabled) {
       // Real RSSI reported omly by SRXL Telemetry Rx, in dBm.
      int8_t rssi = spekFrame[0];

      if (rssi <= SPEKTRUM_RSSI_FADE_LIMIT ) {
        // If Rx reports -100 dBm or less, it is a fade out and frame loss.
        // If it is a temporary fade, real RSSI will come back in the next frame, in that case.
        // we should not report 0% back as OSD keeps a "minimum RSSI" value. Instead keep last good report
        // If it is a total link loss, failsafe will kick in.
        // We could count the fades here, but currentlly to no use

        // Ignore report and Keep last known good value
        rssi = spek_last_rssi;
      }

      if(rssi_channel != 0) {
#ifdef USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
        // Do an dBm to percent conversion with an approxatelly linear distance
        // and map the percentage to RSSI RC channel range
        spekChannelData[rssi_channel] = (uint16_t)(map(dBm2range (rssi),
                                                       0, 100,
                                                       0,resolution));
#else
        // Do a direkt dBm to percent mapping, keeping the non-linear dBm logarithmic curve.
        spekChannelData[rssi_channel] = (uint16_t)(map(rssi),
                                                       SPEKTRUM_RSSI_MIN, SPEKTRUM_RSSI_MAX,
                                                       0,resolution));
#endif
      }
      spek_last_rssi = rssi;
    }

#ifdef USE_SPEKTRUM_FAKE_RSSI
    else
#endif
#endif // USE_SPEKTRUM_REAL_RSSI

#ifdef USE_SPEKTRUM_FAKE_RSSI
    {
      // Fake RSSI value computed from fades

      const uint32_t current_secs = micros() / 1000 / (1000 / SPEKTRUM_FADE_REPORTS_PER_SEC);
      uint16_t fade;
      uint8_t system;

      // Get fade count, different format depending on Rx rype and how Rx is bound. Initially assumed Internal
      if (spektrumSatInternal) {
        // Internal Rx, bind values 3, 5, 7, 9
        fade = (uint16_t) spekFrame[0];
        system = spekFrame[1];

        // Try to detect system type by assuming Internal until we find ANY frame telling otherwise.
        if ( !( (system == SPEKTRUM_DSM2_22) |
                (system == SPEKTRUM_DSM2_11) |
                (system == SPEKTRUM_DSMX_22) |
                (system == SPEKTRUM_DSMX_11) ) ){
          spektrumSatInternal =false; // Nope, this is an externally bound Sat Rx
        }
      }

      if (!spektrumSatInternal) {
        // External Rx, bind values 4, 6, 8, 10
        fade = ((spekFrame[0] << 8) + spekFrame[1]);
      }

      if (spek_fade_last_sec == 0) {
        // This is the first frame status received.
        spek_fade_last_sec_count = fade;
        spek_fade_last_sec = current_secs;
      } else if (spek_fade_last_sec != current_secs) {
        // If the difference is > 1, then we missed several seconds worth of frames and
        // should just throw out the fade calc (as it's likely a full signal loss).
        if ((current_secs - spek_fade_last_sec) == 1) {
          if (rssi_channel != 0) {
            spekChannelData[rssi_channel] = (uint16_t)(map(fade - spek_fade_last_sec_count,
                                                           SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC, 0,
                                                           0, resolution));
          }
        }
        spek_fade_last_sec_count = fade;
        spek_fade_last_sec = current_secs;
      }
    }
#endif

    // Get the RC control channel inputs
    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2) {
        const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
        if (spekChannel < rxRuntimeConfigPtr->channelCount && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT) {
            if (rssi_channel == 0 || spekChannel != rssi_channel) {
                spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
            }
        }
    }

#if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(VTX_COMMON)

    // Get the VTX control bytes in a frame
    uint32_t vtxControl = ((spekFrame[SPEKTRUM_VTX_CONTROL_1] << 24) |
                           (spekFrame[SPEKTRUM_VTX_CONTROL_2] << 16) |
                           (spekFrame[SPEKTRUM_VTX_CONTROL_3] <<  8) |
                           (spekFrame[SPEKTRUM_VTX_CONTROL_4] <<  0) );

    // Handle VTX control frame.
    if ( (vtxControl & SPEKTRUM_VTX_CONTROL_FRAME_MASK) == SPEKTRUM_VTX_CONTROL_FRAME ) {
      handleSpektrumVtxControl(vtxControl);
    }

#endif // USE_SPEKTRUM_VTX_CONTROL && VTX_COMMON

    /* only process if srxl enabled, some data in buffer AND servos in phase 0 */
    if (srxlEnabled && telemetryBufLen && (spekFrame[2] & 0x80)) {
        dispatchAdd(&srxlTelemetryDispatch, SPEKTRUM_TELEMETRY_FRAME_DELAY);
    }
    return RX_FRAME_COMPLETE;
}

static uint16_t spektrumReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    if (spekHiRes)
        data = 988 + (spekChannelData[chan] >> 1);   // 2048 mode
    else
        data = 988 + spekChannelData[chan];          // 1024 mode

    return data;
}

#ifdef USE_SPEKTRUM_BIND

bool spekShouldBind(uint8_t spektrum_sat_bind)
{
#ifdef USE_SPEKTRUM_BIND_PLUG
    IO_t BindPlug = IOGetByTag(rxConfig()->spektrum_bind_plug_ioTag);

    if (BindPlug) {
        IOInit(BindPlug, OWNER_RX_BIND, 0);
        IOConfigGPIO(BindPlug, IOCFG_IPU);

        // Check status of bind plug and exit if not active
        delayMicroseconds(10);  // allow configuration to settle
        if (IORead(BindPlug)) {
            return false;
        }
    }
#endif

    return !(
        isMPUSoftReset() ||
        spektrum_sat_bind == SPEKTRUM_SAT_BIND_DISABLED ||
        spektrum_sat_bind > SPEKTRUM_SAT_BIND_MAX
    );
}

/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
void spektrumBind(rxConfig_t *rxConfig)
{
    if (!spekShouldBind(rxConfig->spektrum_sat_bind)) {
        return;
    }

    // Determine a pin to use
    ioTag_t bindPin = IO_TAG_NONE;

    if (rxConfig->spektrum_bind_pin_override_ioTag) {
        bindPin = rxConfig->spektrum_bind_pin_override_ioTag;
    } else {
        const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
        if (!portConfig) {
            return;
        }

        int index = SERIAL_PORT_IDENTIFIER_TO_INDEX(portConfig->identifier);
        ioTag_t txPin = serialPinConfig()->ioTagTx[index];
        ioTag_t rxPin = serialPinConfig()->ioTagRx[index];

        // Take care half-duplex case
        switch (rxConfig->serialrx_provider) {
        case SERIALRX_SRXL:
#ifdef USE_TELEMETRY
            if (feature(FEATURE_TELEMETRY) && !telemetryCheckRxPortShared(portConfig)) {
                bindPin = txPin;
            }
            break;
#endif
        default:
            bindPin = rxPin;
        }

        if (!bindPin) {
            return;
        }
    }

    IO_t bindIO = IOGetByTag(bindPin);

    IOInit(bindIO, OWNER_RX_BIND, 0);
    IOConfigGPIO(bindIO, IOCFG_OUT_PP);

    LED1_ON;

    // RX line, set high
    IOWrite(bindIO, true);

    // Bind window is around 20-140ms after powerup
    delay(60);
    LED1_OFF;

    for (int i = 0; i < rxConfig->spektrum_sat_bind; i++) {
        LED0_OFF;
        LED2_OFF;
        // RX line, drive low for 120us
        IOWrite(bindIO, false);
        delayMicroseconds(120);

        LED0_ON;
        LED2_ON;
        // RX line, drive high for 120us
        IOWrite(bindIO, true);
        delayMicroseconds(120);

    }


    // Release the bind pin to avoid interference with an actual rx pin,
    // when rxConfig->spektrum_bind_pin_override_ioTag is used.
    // This happens when the bind pin is connected in parallel to the rx pin.

    if (rxConfig->spektrum_bind_pin_override_ioTag) {
        delay(50); // Keep it high for 50msec
        IOConfigGPIO(bindIO, IOCFG_IN_FLOATING);
    }

    // If we came here as a result of hard  reset (power up, with spektrum_sat_bind set), then reset it back to zero and write config
    // Don't reset if hardware bind plug is present
    // Reset only when autoreset is enabled

    if (!rxConfig->spektrum_bind_plug_ioTag && rxConfig->spektrum_sat_bind_autoreset == 1 && !isMPUSoftReset()) {
        rxConfig->spektrum_sat_bind = 0;
        saveConfigAndNotify();
    }
}
#endif // USE_SPEKTRUM_BIND

bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfigPtr = rxRuntimeConfig;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    srxlEnabled = false;
#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    switch (rxConfig->serialrx_provider) {
    case SERIALRX_SRXL:
#ifdef USE_TELEMETRY
        srxlEnabled = (feature(FEATURE_TELEMETRY) && !portShared);
#endif
    case SERIALRX_SPEKTRUM2048:
        // 11 bit frames
        spek_chan_shift = 3;
        spek_chan_mask = 0x07;
        spekHiRes = true;
        resolution = 2048;
        rxRuntimeConfig->channelCount = SPEKTRUM_2048_CHANNEL_COUNT;
        rxRuntimeConfig->rxRefreshRate = 11000;
        break;
    case SERIALRX_SPEKTRUM1024:
        // 10 bit frames
        spek_chan_shift = 2;
        spek_chan_mask = 0x03;
        spekHiRes = false;
        resolution = 1024;
        rxRuntimeConfig->channelCount = SPEKTRUM_1024_CHANNEL_COUNT;
        rxRuntimeConfig->rxRefreshRate = 22000;
        break;
    }

    rxRuntimeConfig->rcReadRawFn = spektrumReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = spektrumFrameStatus;

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        spektrumDataReceive,
        NULL,
        SPEKTRUM_BAUDRATE,
        portShared || srxlEnabled ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | ((srxlEnabled || rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = serialPort;
    }
#endif

    rssi_channel = rxConfig->rssi_channel - 1; // -1 because rxConfig->rssi_channel is 1-based and rssi_channel is 0-based.
    if (rssi_channel >= rxRuntimeConfig->channelCount) {
        rssi_channel = 0;
    }

    if (serialPort && srxlEnabled) {
        dispatchEnable();
    }
    return serialPort != NULL;
}

void srxlRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void srxlRxSendTelemetryDataDispatch(dispatchEntry_t* self)
{
    UNUSED(self);
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool srxlRxIsActive(void)
{
    return serialPort != NULL;
}

#endif // SERIAL_RX
