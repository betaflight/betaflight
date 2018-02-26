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

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/fport.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/crsf.h"
#include "rx/rx_spi.h"
#include "rx/targetcustomserial.h"


//#define DEBUG_RX_SIGNAL_LOSS

const char rcChannelLetters[] = "AERT12345678abcdefgh";

static uint16_t rssi = 0;                  // range: [0;1023]
static timeUs_t lastMspRssiUpdateUs = 0;

#define MSP_RSSI_TIMEOUT_US 1500000   // 1.5 sec

rssiSource_e rssiSource;

static bool rxDataProcessingRequired = false;
static bool auxiliaryProcessingRequired = false;

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static bool rxIsInFailsafeMode = true;
static uint8_t rxChannelCount;

static timeUs_t rxNextUpdateAtUs = 0;
static uint32_t needRxSignalBefore = 0;
static uint32_t needRxSignalMaxDelayUs;
static uint32_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

static int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULS_TIME    300
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_50_HZ (1000000 / 50)
#define DELAY_10_HZ (1000000 / 10)
#define DELAY_5_HZ (1000000 / 5)
#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeConfig_t rxRuntimeConfig;
static uint8_t rcSampleIndex = 0;

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

#ifndef SPEKTRUM_BIND_PIN
#define SPEKTRUM_BIND_PIN NONE
#endif

#ifndef BINDPLUG_PIN
#define BINDPLUG_PIN NONE
#endif

PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 2);
void pgResetFn_rxConfig(rxConfig_t *rxConfig)
{
    RESET_CONFIG_2(rxConfig_t, rxConfig,
        .halfDuplex = 0,
        .serialrx_provider = SERIALRX_PROVIDER,
        .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
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
        .rssi_channel = 0,
        .rssi_scale = RSSI_SCALE_DEFAULT,
        .rssi_invert = 0,
        .rcInterpolation = RC_SMOOTHING_AUTO,
        .rcInterpolationChannels = 0,
        .rcInterpolationInterval = 19,
        .fpvCamAngleDegrees = 0,
        .airModeActivateThreshold = 32,
        .max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT
    );

#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", rxConfig);
#else
    parseRcChannels("AETR1234", rxConfig);
#endif
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig) {
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    return true;
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SRXL:
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
    case SERIALRX_TARGET_CUSTOM:
        enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
    rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeConfig.rcProcessFrameFn = nullProcessFrame;
    rcSampleIndex = 0;
    needRxSignalMaxDelayUs = DELAY_10_HZ;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

#ifdef USE_SERIAL_RX
    if (feature(FEATURE_RX_SERIAL)) {
        const bool enabled = serialRxInit(rxConfig(), &rxRuntimeConfig);
        if (!enabled) {
            featureClear(FEATURE_RX_SERIAL);
            rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
            rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
        }
    }
#endif

#ifdef USE_RX_MSP
    if (feature(FEATURE_RX_MSP)) {
        rxMspInit(rxConfig(), &rxRuntimeConfig);
        needRxSignalMaxDelayUs = DELAY_5_HZ;
    }
#endif

#ifdef USE_RX_SPI
    if (feature(FEATURE_RX_SPI)) {
        const bool enabled = rxSpiInit(rxConfig(), &rxRuntimeConfig);
        if (!enabled) {
            featureClear(FEATURE_RX_SPI);
            rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
            rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
        }
    }
#endif

#if defined(USE_PWM) || defined(USE_PPM)
    if (feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM)) {
        rxPwmInit(rxConfig(), &rxRuntimeConfig);
    }
#endif

#if defined(USE_ADC)
    if (feature(FEATURE_RSSI_ADC)) {
        rssiSource = RSSI_SOURCE_ADC;
    } else
#endif
    if (rxConfig()->rssi_channel > 0) {
        rssiSource = RSSI_SOURCE_RX_CHANNEL;
    }

    rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeConfig.channelCount);
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxSignal(void)
{
    suspendRxSignalUntil = micros() + SKIP_RC_ON_SUSPEND_PERIOD;
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    failsafeOnRxSuspend(SKIP_RC_ON_SUSPEND_PERIOD);
}

void resumeRxSignal(void)
{
    suspendRxSignalUntil = micros();
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    failsafeOnRxResume();
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    if (rxSignalReceived) {
        if (currentTimeUs >= needRxSignalBefore) {
            rxSignalReceived = false;
        }
    }

#if defined(USE_PWM) || defined(USE_PPM)
    if (feature(FEATURE_RX_PPM)) {
        if (isPPMDataBeingReceived()) {
            rxDataProcessingRequired = true;
            rxSignalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            resetPPMDataReceivedState();
        }
    } else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        if (isPWMDataBeingReceived()) {
            rxDataProcessingRequired = true;
            rxSignalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    } else
#endif
    {
        const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig);
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataProcessingRequired = true;
            rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }

        if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
            auxiliaryProcessingRequired = true;
        }
    }

    if (cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

    return rxDataProcessingRequired || auxiliaryProcessingRequired; // data driven or 50Hz
}

static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

    // update the recent samples and compute the average of them
    rcSamples[chan][currentSampleIndex] = sample;

    // avoid returning an incorrect average which would otherwise occur before enough samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    rcDataMean[chan] = 0;
    for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
        rcDataMean[chan] += rcSamples[chan][sampleIndex];
    }
    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}

static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);

    switch (channelFailsafeConfig->mode) {
    case RX_FAILSAFE_MODE_AUTO:
        switch (channel) {
        case ROLL:
        case PITCH:
        case YAW:
            return rxConfig()->midrc;
        case THROTTLE:
            if (feature(FEATURE_3D))
                return rxConfig()->midrc;
            else
                return rxConfig()->rx_min_usec;
        }
        /* no break */

    default:
    case RX_FAILSAFE_MODE_INVALID:
    case RX_FAILSAFE_MODE_HOLD:
        return rcData[channel];

    case RX_FAILSAFE_MODE_SET:
        return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
    }
}

STATIC_UNIT_TESTED uint16_t applyRxChannelRangeConfiguraton(int sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = constrain(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;

        // sample the channel
        uint16_t sample = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

static void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();

    const bool useValueFromRx = rxSignalReceived && !rxIsInFailsafeMode;

#ifdef DEBUG_RX_SIGNAL_LOSS
    debug[0] = rxSignalReceived;
    debug[1] = rxIsInFailsafeMode;
    debug[2] = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 0);
#endif

    rxFlightChannelsValid = true;
    for (int channel = 0; channel < rxChannelCount; channel++) {
        uint16_t sample = rcRaw[channel];

        const bool validPulse = useValueFromRx && isPulseValid(sample);

        if (validPulse) {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        } else {
            if (cmp32(currentTimeMs, rcInvalidPulsPeriod[channel]) < 0) {
                continue;           // skip to next channel to hold channel value MAX_INVALID_PULS_TIME
            } else {
                sample = getRxfailValue(channel);   // after that apply rxfail value
                if (channel < NON_AUX_CHANNEL_COUNT) {
                    rxFlightChannelsValid = false;
                }
            }
        }
        if (feature(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM)) {
            // smooth output for PWM and PPM
            rcData[channel] = calculateChannelMovingAverage(channel, sample);
        } else {
            rcData[channel] = sample;
        }
    }

    if (rxFlightChannelsValid && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = true;
        failsafeOnValidDataFailed();
        for (int channel = 0; channel < rxChannelCount; channel++) {
            rcData[channel] = getRxfailValue(channel);
        }
    }

#ifdef DEBUG_RX_SIGNAL_LOSS
    debug[3] = rcData[THROTTLE];
#endif
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeConfig.rcProcessFrameFn(&rxRuntimeConfig);
    }

    if (!rxDataProcessingRequired) {
        return false;
    }

    rxDataProcessingRequired = false;
    rxNextUpdateAtUs = currentTimeUs + DELAY_50_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    readRxChannelsApplyRanges();
    detectAndApplySignalLossBehaviour();

    rcSampleIndex++;

    return true;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

void setRssiFiltered(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssi = newRssi;
}

#define RSSI_SAMPLE_COUNT 16
#define RSSI_MAX_VALUE 1023

void setRssiUnfiltered(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    static uint16_t rssiSamples[RSSI_SAMPLE_COUNT];
    static uint8_t rssiSampleIndex = 0;
    static unsigned sum = 0;

    sum = sum + rssiValue;
    sum = sum - rssiSamples[rssiSampleIndex];
    rssiSamples[rssiSampleIndex] = rssiValue;
    rssiSampleIndex = (rssiSampleIndex + 1) % RSSI_SAMPLE_COUNT;

    int16_t rssiMean = sum / RSSI_SAMPLE_COUNT;

    rssi = rssiMean;
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if (rssiSource == RSSI_SOURCE_MSP) {
        rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIPWM(void)
{
    // Read value of AUX channel as rssi
    int16_t pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        pwmRssi = ((2000 - pwmRssi) + 1000);
    }

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiFiltered(constrain((uint16_t)(((pwmRssi - 1000) / 1000.0f) * 1024.0f), 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

static void updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt) < 0) {
        return;
    }
    rssiUpdateAt = currentTimeUs + DELAY_50_HZ;

    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint16_t rssiValue = (uint16_t)((1024.0f * adcRssiSample) / (rxConfig()->rssi_scale * 100.0f));
    rssiValue = constrain(rssiValue, 0, RSSI_MAX_VALUE);

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    setRssiUnfiltered((uint16_t)rssiValue, RSSI_SOURCE_ADC);
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{
    switch (rssiSource) {
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIPWM();
        break;
    case RSSI_SOURCE_ADC:
        updateRSSIADC(currentTimeUs);
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > MSP_RSSI_TIMEOUT_US) {
            rssi = 0;
        }
        break;
    default:
        break;
    }
}

uint16_t getRssi(void)
{
    return rssi;
}

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}
