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

#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "rx.h"

#include "config/config_reset.h"

#include "drivers/io.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "rx/rx_spi.h"

#ifndef SERIALRX_PROVIDER

#if defined(USE_SERIALRX_CRSF)
#define SERIALRX_PROVIDER SERIALRX_CRSF
#elif defined(USE_SERIALRX_GHST)
#define SERIALRX_PROVIDER SERIALRX_GHST
#elif defined(USE_SERIALRX_IBUS)
#define SERIALRX_PROVIDER SERIALRX_IBUS
#elif defined(USE_SERIALRX_SBUS)
#define SERIALRX_PROVIDER SERIALRX_SBUS
#elif defined(USE_SERIALRX_SPEKTRUM)
#if defined(USE_TELEMETRY_SRXL)
#define SERIALRX_PROVIDER SERIALRX_SRXL
#else
#define SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
#endif
#elif defined(USE_SERIALRX_FPORT)
#define SERIALRX_PROVIDER SERIALRX_FPORT
#elif defined(USE_SERIALRX_XBUS)
#define SERIALRX_PROVIDER SERIALRX_XBUS_MODE_B
#elif defined(USE_SERIALRX_SRXL2)
#define SERIALRX_PROVIDER SERIALRX_SRXL2
#elif defined(USE_SERIALRX_JETIEXBUS)
#define SERIALRX_PROVIDER SERIALRX_JETIEXBUS
#elif defined(USE_SERIALRX_SUMD)
#define SERIALRX_PROVIDER SERIALRX_SUMD
#elif defined(USE_SERIALRX_SUMH)
#define SERIALRX_PROVIDER SERIALRX_SUMH
#else
#define SERIALRX_PROVIDER SERIALRX_NONE
#endif

#endif

#ifndef SERIALRX_HALFDUPLEX
#if (defined(USE_SERIALRX_FPORT) || defined(USE_SERIALRX_SRXL2)) && !(defined(USE_SERIALRX_CRSF) && defined(USE_SERIALRX_GHST) && defined(USE_SERIALRX_IBUS) && defined(USE_SERIALRX_SBUS) && defined(USE_SERIALRX_SPEKTRUM) && defined(USE_SERIALRX_XBUS))
#define SERIALRX_HALFDUPLEX 1
#else
#define SERIALRX_HALFDUPLEX 0
#endif
#endif

PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 4);
void pgResetFn_rxConfig(rxConfig_t *rxConfig)
{
    RESET_CONFIG_2(rxConfig_t, rxConfig,
        .halfDuplex = SERIALRX_HALFDUPLEX,
        .serialrx_provider = SERIALRX_PROVIDER,
        .serialrx_inverted = 0,
        .spektrum_bind_pin_override_ioTag = IO_TAG(SPEKTRUM_BIND_PIN),
        .spektrum_bind_plug_ioTag = IO_TAG(BINDPLUG_PIN),
        .spektrum_sat_bind = 0,
        .spektrum_sat_bind_autoreset = 1,
        .midrc = RX_MID_USEC,
        .mincheck = 1050,
        .maxcheck = 1900,
        .rx_min_usec = RX_MIN_USEC,          // any of first 4 channels below this value will trigger rx loss detection
        .rx_max_usec = RX_MAX_USEC,         // any of first 4 channels above this value will trigger rx loss detection
        .rssi_src_frame_errors = false,
        .rssi_channel = 0,
        .rssi_scale = RSSI_SCALE_DEFAULT,
        .rssi_offset = 0,
        .rssi_invert = 0,
        .rssi_src_frame_lpf_period = 30,
        .rssi_smoothing = 125,
        .fpvCamAngleDegrees = 0,
        .airModeActivateThreshold = 25,
        .max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT,
        .rc_smoothing_mode = 1,
        .rc_smoothing_setpoint_cutoff = 0,
        .rc_smoothing_feedforward_cutoff = 0,
        .rc_smoothing_throttle_cutoff = 0,
        .rc_smoothing_debug_axis = ROLL,
        .rc_smoothing_auto_factor_rpy = 30,
        .rc_smoothing_auto_factor_throttle = 30,
        .srxl2_unit_id = 1,
        .srxl2_baud_fast = true,
        .sbus_baud_fast = false,
        .msp_override_channels_mask = 0,
        .crsf_use_negotiated_baud = false,
        .serialtx_inverted = 0,
    );

#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", rxConfig);
#else
    parseRcChannels("AETR1234", rxConfig);
#endif
}

#endif
