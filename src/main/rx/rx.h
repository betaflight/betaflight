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

#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_ZERO 0 // FIXME should all usages of this be changed to use PWM_RANGE_MIN?
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
#define DEFAULT_SERVO_MIN_ANGLE 90
#define DEFAULT_SERVO_MAX_ANGLE 90

typedef enum {
    SERIAL_RX_FRAME_PENDING = 0,
    SERIAL_RX_FRAME_COMPLETE = (1 << 0),
    SERIAL_RX_FRAME_FAILSAFE = (1 << 1)
} serialrxFrameState_t;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_SUMH = 4,
    SERIALRX_XBUS_MODE_B = 5,
    SERIALRX_XBUS_MODE_B_RJ01 = 6,
    SERIALRX_PROVIDER_MAX = SERIALRX_XBUS_MODE_B_RJ01
} SerialRXType;

#define SERIALRX_PROVIDER_COUNT (SERIALRX_PROVIDER_MAX + 1)

#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT 12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT 8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT (18)

#define NON_AUX_CHANNEL_COUNT 4
#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)



#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#endif

extern const char rcChannelLetters[];

extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

#define MAX_MAPPABLE_RX_INPUTS 8

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255
#define RSSI_SCALE_DEFAULT 30

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID,
} rxFailsafeChannelMode_e;

#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX,
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2

typedef struct rxFailsafeChannelConfiguration_s {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfiguration_t;

typedef struct rxChannelRangeConfiguration_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfiguration_t;

typedef struct rxConfig_s {
    uint8_t rcmap[MAX_MAPPABLE_RX_INPUTS];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_ppm_invert;
    uint8_t rcSmoothing;                    // Enable/Disable RC filtering
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end

    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    rxFailsafeChannelConfiguration_t failsafe_channel_configurations[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    rxChannelRangeConfiguration_t channelRanges[NON_AUX_CHANNEL_COUNT];
} rxConfig_t;

#define REMAPPABLE_CHANNEL_COUNT (sizeof(((rxConfig_t *)0)->rcmap) / sizeof(((rxConfig_t *)0)->rcmap[0]))

typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;                  // number of rc channels as reported by current input driver
    uint8_t auxChannelCount;
} rxRuntimeConfig_t;

extern rxRuntimeConfig_t rxRuntimeConfig;

void useRxConfig(rxConfig_t *rxConfigToUse);

typedef uint16_t (*rcReadRawDataPtr)(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);        // used by receiver driver to return channel data

void updateRx(uint32_t currentTime);
bool rxIsReceivingSignal(void);
bool rxAreFlightChannelsValid(void);
bool shouldProcessRx(uint32_t currentTime);
void calculateRxChannelsAndUpdateFailsafe(uint32_t currentTime);

void parseRcChannels(const char *input, rxConfig_t *rxConfig);
uint8_t serialRxFrameStatus(rxConfig_t *rxConfig);

void updateRSSI(uint32_t currentTime);
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *rxChannelRangeConfiguration);

void suspendRxSignal(void);
void resumeRxSignal(void);

void initRxRefreshRate(uint16_t *rxRefreshRatePtr);
