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

#pragma once

#include "common/time.h"

#include "pg/pg.h"

#include "drivers/io_types.h"

#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)
#define MAX_RXFAIL_RANGE_STEP ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25)

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
} rxFrameState_e;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_SUMH = 4,
    SERIALRX_XBUS_MODE_B = 5,
    SERIALRX_XBUS_MODE_B_RJ01 = 6,
    SERIALRX_IBUS = 7,
    SERIALRX_JETIEXBUS = 8,
    SERIALRX_CRSF = 9,
    SERIALRX_SRXL = 10,
    SERIALRX_TARGET_CUSTOM = 11,
    SERIALRX_FPORT = 12,
} SerialRXType;

#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#endif

extern const char rcChannelLetters[];

extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

#define RX_MAPPABLE_CHANNEL_COUNT 8

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255
#define RSSI_SCALE_DEFAULT (4095.0f / 100.0f + 0.5f) // 100% @ 4095

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID
} rxFailsafeChannelMode_e;

#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2

typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfig_t;

PG_DECLARE_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs);

typedef struct rxChannelRangeConfig_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfig_t;

PG_DECLARE_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs);

typedef struct rxConfig_s {
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t serialrx_inverted;              // invert the serial RX protocol compared to it's default setting
    uint8_t halfDuplex;                     // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
    uint8_t rx_spi_protocol;                // type of SPI RX protocol
                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;
    ioTag_t spektrum_bind_pin_override_ioTag;
    ioTag_t spektrum_bind_plug_ioTag;
    uint8_t spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
    uint8_t spektrum_sat_bind_autoreset;    // whenever we will reset (exit) binding mode after hard reboot
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t rcInterpolation;
    uint8_t rcInterpolationChannels;
    uint8_t rcInterpolationInterval;
    uint8_t fpvCamAngleDegrees;             // Camera angle to be scaled into rc commands
    uint8_t airModeActivateThreshold;       // Throttle setpoint percent where airmode gets activated

    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
} rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);

struct rxRuntimeConfig_s;
typedef uint16_t (*rcReadRawDataFnPtr)(const struct rxRuntimeConfig_s *rxRuntimeConfig, uint8_t chan); // used by receiver driver to return channel data
typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeConfig_s *rxRuntimeConfig);
typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeConfig_s *rxRuntimeConfig);

typedef struct rxRuntimeConfig_s {
    uint8_t             channelCount; // number of RC channels as reported by current input driver
    uint16_t            rxRefreshRate;
    rcReadRawDataFnPtr  rcReadRawFn;
    rcFrameStatusFnPtr  rcFrameStatusFn;
    rcProcessFrameFnPtr rcProcessFrameFn;
    uint16_t            *channelData;
    void                *frameData;
} rxRuntimeConfig_t;

typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_ADC,
    RSSI_SOURCE_RX_CHANNEL,
    RSSI_SOURCE_RX_PROTOCOL,
    RSSI_SOURCE_MSP,
} rssiSource_e;

extern rssiSource_e rssiSource;

extern rxRuntimeConfig_t rxRuntimeConfig; //!!TODO remove this extern, only needed once for channelCount

void rxInit(void);
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
bool rxIsReceivingSignal(void);
bool rxAreFlightChannelsValid(void);
bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs);

void parseRcChannels(const char *input, rxConfig_t *rxConfig);

void setRssiFiltered(uint16_t newRssi, rssiSource_e source);
void setRssiUnfiltered(uint16_t rssiValue, rssiSource_e source);
void setRssiMsp(uint8_t newMspRssi);
void updateRSSI(timeUs_t currentTimeUs);
uint16_t getRssi(void);

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig);

void suspendRxSignal(void);
void resumeRxSignal(void);

uint16_t rxGetRefreshRate(void);

