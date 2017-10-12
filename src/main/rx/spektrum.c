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

#ifdef SERIAL_RX

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "fc/config.h"
#include "fc/fc_dispatch.h"

#ifdef TELEMETRY
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


// Receive ISR callback
static void spektrumDataReceive(uint16_t c)
{
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

static uint8_t spektrumFrameStatus(void)
{
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

    /* only process if srxl enabled, some data in buffer AND servos in phase 0 */
    if (srxlEnabled && telemetryBufLen && (spekFrame[2] & 0x80)) {
        dispatchAdd(&srxlTelemetryDispatch, 100);
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
#ifdef TELEMETRY
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
#ifdef TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    switch (rxConfig->serialrx_provider) {
    case SERIALRX_SRXL:
#ifdef TELEMETRY
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
        SPEKTRUM_BAUDRATE,
        portShared || srxlEnabled ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | ((srxlEnabled || rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );

#ifdef TELEMETRY
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
