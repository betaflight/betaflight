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

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"


#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/rx_spi.h"
#include "rx/crsf.h"
#include "rx/eleres.h"


//#define DEBUG_RX_SIGNAL_LOSS

const char rcChannelLetters[] = "AERT12345678abcdefgh";

uint16_t rssi = 0;                  // range: [0;1023]

static bool rxDataReceived = false;
static bool rxSignalReceived = false;
static bool rxSignalReceivedNotDataDriven = false;
static bool rxFlightChannelsValid = false;
static bool rxIsInFailsafeMode = true;
static bool rxIsInFailsafeModeNotDataDriven = true;

static timeUs_t rxUpdateAt = 0;
static timeUs_t needRxSignalBefore = 0;
static timeDelta_t needRxSignalMaxDelayUs = 0;
static timeUs_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
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

PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 1);

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#define RX_MIDRC 1500
#define RX_MIN_USEX 885
PG_RESET_TEMPLATE(rxConfig_t, rxConfig,
    .halfDuplex = 0,
    .serialrx_provider = SERIALRX_PROVIDER,
    .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
    .spektrum_sat_bind = 0,
    .sbus_inversion = 1,
    .midrc = RX_MIDRC,
    .mincheck = 1100,
    .maxcheck = 1900,
    .rx_min_usec = RX_MIN_USEX,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = 2115,         // any of first 4 channels above this value will trigger rx loss detection
    .rssi_channel = 0,
    .rssi_scale = RSSI_SCALE_DEFAULT,
    .rssiInvert = 0,
    .rcSmoothing = 1,
);

void resetAllRxChannelRangeConfigurations(void)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigsMutable(i)->min = PWM_RANGE_MIN;
        rxChannelRangeConfigsMutable(i)->max = PWM_RANGE_MAX;
    }
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

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(void)
{
    return RX_FRAME_PENDING;
}

#define REQUIRED_CHANNEL_MASK 0x0F // first 4 channels

static uint8_t validFlightChannelMask;

STATIC_UNIT_TESTED void rxResetFlightChannelStatus(void) {
    validFlightChannelMask = REQUIRED_CHANNEL_MASK;
}

STATIC_UNIT_TESTED bool rxHaveValidFlightChannels(void)
{
    return (validFlightChannelMask == REQUIRED_CHANNEL_MASK);
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

// pulse duration is in micro seconds (usec)
STATIC_UNIT_TESTED void rxUpdateFlightChannelStatus(uint8_t channel, bool valid)
{
    if (channel < NON_AUX_CHANNEL_COUNT && !valid) {
        // if signal is invalid - mark channel as BAD
        validFlightChannelMask &= ~(1 << channel);
    }
}

#ifdef SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
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
    rcSampleIndex = 0;
    needRxSignalMaxDelayUs = DELAY_10_HZ;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        if (modeActivationConditions(i)->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationConditions(i)->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationConditions(i)->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

#ifdef SERIAL_RX
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

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM)) {
        rxPwmInit(rxConfig(), &rxRuntimeConfig);
    }
#endif
}

static uint8_t calculateChannelRemapping(const uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
{
    if (channelToRemap < channelMapEntryCount) {
        return channelMap[channelToRemap];
    }
    return channelToRemap;
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

static bool isRxDataDriven(void)
{
    return !(feature(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM));
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
        if (((int32_t)(currentTimeUs - needRxSignalBefore) >= 0)) {
            rxSignalReceived = false;
            rxSignalReceivedNotDataDriven = false;
        }
    }

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (feature(FEATURE_RX_PPM)) {
        if (isPPMDataBeingReceived()) {
            rxSignalReceivedNotDataDriven = true;
            rxIsInFailsafeModeNotDataDriven = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            resetPPMDataReceivedState();
        }
    } else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        if (isPWMDataBeingReceived()) {
            rxSignalReceivedNotDataDriven = true;
            rxIsInFailsafeModeNotDataDriven = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    } else
#endif
    {
        rxDataReceived = false;
        const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn();
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataReceived = true;
            rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    }

    return rxDataReceived || ((int32_t)(currentTimeUs - rxUpdateAt) >= 0); // data driven or 50Hz
}

static uint16_t calculateNonDataDrivenChannel(uint8_t chan, uint16_t sample)
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

STATIC_UNIT_TESTED uint16_t applyRxChannelRangeConfiguraton(int sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);

    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {

        const uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

static void detectSignalLoss(void)
{
    const bool rxIsDataDriven = isRxDataDriven();
    const timeMs_t currentMilliTime = millis();

    if (!rxIsDataDriven) {
        rxSignalReceived = rxSignalReceivedNotDataDriven;
        rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven;
    }

#ifdef DEBUG_RX_SIGNAL_LOSS
    debug[0] = rxSignalReceived;
    debug[1] = rxIsInFailsafeMode;
    debug[2] = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 0);
#endif

    rxResetFlightChannelStatus();

    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
        uint16_t sample = rcRaw[channel];
        bool validPulse = isPulseValid(sample);

        if (!validPulse) {
            sample = rcData[channel];   // hold channel
            if (currentMilliTime > rcInvalidPulsPeriod[channel]) {
                rxUpdateFlightChannelStatus(channel, validPulse);
            }
        } else {
            rcInvalidPulsPeriod[channel] = currentMilliTime + MAX_INVALID_PULS_TIME;
        }

        if (rxIsDataDriven) {
            rcData[channel] = sample;
        } else {
            rcData[channel] = calculateNonDataDrivenChannel(channel, sample);
        }
    }

    rxFlightChannelsValid = rxHaveValidFlightChannels();

    if (rxFlightChannelsValid && !IS_RC_MODE_ACTIVE(BOXFAILSAFE) && rxSignalReceived && !rxIsInFailsafeMode) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven = true;
        failsafeOnValidDataFailed();
    }
}

void calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    rxUpdateAt = currentTimeUs + DELAY_50_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }
        return;
    }

    readRxChannelsApplyRanges();
    detectSignalLoss();

    rcSampleIndex++;
}

void parseRcChannels(const char *input)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
            rxConfigMutable()->rcmap[s - rcChannelLetters] = c - input;
    }
}

#ifdef USE_RX_ELERES
void updateRSSIeleres(uint32_t currentTime)
{
    UNUSED(currentTime);
    rssi = eleresRssi();
}
#endif // USE_RX_ELERES

static void updateRSSIPWM(void)
{
    int16_t pwmRssi = 0;
    // Read value of AUX channel as rssi
    pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    rssi = (uint16_t)((constrain(pwmRssi - 1000, 0, 1000) / 1000.0f) * 1023.0f);
}

#define RSSI_ADC_SAMPLE_COUNT 16
//#define RSSI_SCALE (0xFFF / 100.0f)

static void updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint8_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
    static uint8_t adcRssiSampleIndex = 0;
    static timeUs_t rssiUpdateAtUs = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAtUs) < 0) {
        return;
    }
    rssiUpdateAtUs = currentTimeUs + DELAY_50_HZ;

    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    const uint8_t rssiPercentage = adcRssiSample / rxConfig()->rssi_scale;

    adcRssiSampleIndex = (adcRssiSampleIndex + 1) % RSSI_ADC_SAMPLE_COUNT;

    adcRssiSamples[adcRssiSampleIndex] = rssiPercentage;

    int16_t adcRssiMean = 0;
    for (int sampleIndex = 0; sampleIndex < RSSI_ADC_SAMPLE_COUNT; sampleIndex++) {
        adcRssiMean += adcRssiSamples[sampleIndex];
    }

    adcRssiMean = adcRssiMean / RSSI_ADC_SAMPLE_COUNT;

    rssi = (uint16_t)((constrain(adcRssiMean, 0, 100) / 100.0f) * 1023.0f);
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{
    // Read RSSI
    if (rxConfig()->rssi_channel > 0) {
        updateRSSIPWM();
    } else if (feature(FEATURE_RSSI_ADC)) {
        updateRSSIADC(currentTimeUs);
#ifdef USE_RX_ELERES
    } else if (feature(FEATURE_RX_SPI) && rxConfig()->rx_spi_protocol == RFM22_ELERES) {
        updateRSSIeleres(currentTimeUs);
#endif
    }

    // Apply RSSI inversion
    if (rxConfig()->rssiInvert) {
        rssi = 1023 - rssi;
    }

}

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}
