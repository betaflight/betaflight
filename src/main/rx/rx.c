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

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/pwm_rx.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "fc/rc_controls.h"

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
static const rxConfig_t *rxConfig;
static uint8_t rcSampleIndex = 0;

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

void serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void useRxConfig(const rxConfig_t *rxConfigToUse)
{
    rxConfig = rxConfigToUse;
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
    return  pulseDuration >= rxConfig->rx_min_usec &&
            pulseDuration <= rxConfig->rx_max_usec;
}

// pulse duration is in micro seconds (usec)
STATIC_UNIT_TESTED void rxUpdateFlightChannelStatus(uint8_t channel, bool valid)
{
    if (channel < NON_AUX_CHANNEL_COUNT && !valid) {
        // if signal is invalid - mark channel as BAD
        validFlightChannelMask &= ~(1 << channel);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *rxChannelRangeConfiguration) {
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfiguration->min = PWM_RANGE_MIN;
        rxChannelRangeConfiguration->max = PWM_RANGE_MAX;
        rxChannelRangeConfiguration++;
    }
}

void rxInit(const rxConfig_t *rxConfig, const modeActivationCondition_t *modeActivationConditions)
{
    uint8_t i;
    uint16_t value;

    useRxConfig(rxConfig);
    rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
    rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
    rcSampleIndex = 0;

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig->midrc : rxConfig->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[i];
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // ARM switch is defined, determine an OFF value
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

#ifdef SERIAL_RX
    if (feature(FEATURE_RX_SERIAL)) {
        serialRxInit(rxConfig, &rxRuntimeConfig);
    }
#endif

#ifndef SKIP_RX_MSP
    if (feature(FEATURE_RX_MSP)) {
        rxMspInit(rxConfig, &rxRuntimeConfig);
    }
#endif

#ifdef USE_RX_SPI
    if (feature(FEATURE_RX_SPI)) {
        const bool enabled = rxSpiInit(rxConfig, &rxRuntimeConfig);
        if (!enabled) {
            featureClear(FEATURE_RX_SPI);
            rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
        }
    }
#endif

#ifndef SKIP_RX_PWM_PPM
    if (feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeConfig.rxRefreshRate = 20000;
        rxPwmInit(rxConfig, &rxRuntimeConfig);
    }
#endif
}

#ifdef SERIAL_RX
void serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SPEKTRUM1024:
        enabled = spektrumInit(rxConfig, rxRuntimeConfig);
        break;
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
    default:
        enabled = false;
        break;
    }

    if (!enabled) {
        featureClear(FEATURE_RX_SERIAL);
        rxRuntimeConfig->rcReadRawFn = nullReadRawRC;
    }
}

static uint8_t serialRxFrameStatus(const rxConfig_t *rxConfig)
{
    /**
     * FIXME: Each of the xxxxFrameStatus() methods MUST be able to survive being called without the
     * corresponding xxxInit() method having been called first.
     *
     * This situation arises when the cli or the msp changes the value of rxConfig->serialrx_provider
     *
     * A solution is for the ___Init() to configure the serialRxFrameStatus function pointer which
     * should be used instead of the switch statement below.
     */
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        return spektrumFrameStatus();
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        return sbusFrameStatus();
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        return sumdFrameStatus();
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        return sumhFrameStatus();
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        return xBusFrameStatus();
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        return ibusFrameStatus();
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        return jetiExBusFrameStatus();
#endif
    default:
        return RX_FRAME_PENDING;
    }
    return RX_FRAME_PENDING;
}
#endif

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
static bool isRxDataDriven(void) {
    return !(feature(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM));
}

static void resetRxSignalReceivedFlagIfNeeded(timeUs_t currentTimeUs)
{
    if (!rxSignalReceived) {
        return;
    }

    if (((int32_t)(currentTimeUs - needRxSignalBefore) >= 0)) {
        rxSignalReceived = false;
        rxSignalReceivedNotDataDriven = false;
    }
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

bool updateRx(timeUs_t currentTimeUs)
{
    resetRxSignalReceivedFlagIfNeeded(currentTimeUs);

    if (isRxDataDriven()) {
        rxDataReceived = false;
    }


#ifdef SERIAL_RX
    if (feature(FEATURE_RX_SERIAL)) {
        const uint8_t frameStatus = serialRxFrameStatus(rxConfig);
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataReceived = true;
            rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + DELAY_10_HZ;
        }
    }
#endif

#ifdef USE_RX_SPI
    if (feature(FEATURE_RX_SPI)) {
        const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn();
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataReceived = true;
            rxIsInFailsafeMode = false;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + DELAY_5_HZ;
        }
    }
#endif

#ifndef SKIP_RX_MSP
    if (feature(FEATURE_RX_MSP)) {
        const uint8_t frameStatus = rxMspFrameStatus();
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataReceived = true;
            rxIsInFailsafeMode = false;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + DELAY_5_HZ;
        }
    }
#endif

#ifndef SKIP_RX_PWM_PPM
    if (feature(FEATURE_RX_PPM)) {
        if (isPPMDataBeingReceived()) {
            rxSignalReceivedNotDataDriven = true;
            rxIsInFailsafeModeNotDataDriven = false;
            needRxSignalBefore = currentTimeUs + DELAY_10_HZ;
            resetPPMDataReceivedState();
        }
    }

    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        if (isPWMDataBeingReceived()) {
            rxSignalReceivedNotDataDriven = true;
            rxIsInFailsafeModeNotDataDriven = false;
            needRxSignalBefore = currentTimeUs + DELAY_10_HZ;
        }
    }
#endif
    return rxDataReceived || ((int32_t)(currentTimeUs - rxUpdateAt) >= 0); // data driven or 50Hz
}

static uint16_t calculateNonDataDrivenChannel(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

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

    uint8_t sampleIndex;
    for (sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++)
        rcDataMean[chan] += rcSamples[chan][sampleIndex];

    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}

static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig->failsafe_channel_configurations[channel];
    uint8_t mode = channelFailsafeConfiguration->mode;

    switch(mode) {
        case RX_FAILSAFE_MODE_AUTO:
            switch (channel) {
                case ROLL:
                case PITCH:
                case YAW:
                    return rxConfig->midrc;

                case THROTTLE:
                    if (feature(FEATURE_3D))
                        return rxConfig->midrc;
                    else
                        return rxConfig->rx_min_usec;
            }
            /* no break */

        default:
        case RX_FAILSAFE_MODE_INVALID:
        case RX_FAILSAFE_MODE_HOLD:
            return rcData[channel];

        case RX_FAILSAFE_MODE_SET:
            return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step);
    }
}

STATIC_UNIT_TESTED uint16_t applyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRange(sample, range.min, range.max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);

    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {

        const uint8_t rawChannel = calculateChannelRemapping(rxConfig->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxConfig->channelRanges[channel]);
        }

        rcRaw[channel] = sample;
    }
}

static void detectAndApplySignalLossBehaviour(void)
{
    int channel;
    uint16_t sample;
    bool useValueFromRx = true;
    bool rxIsDataDriven = isRxDataDriven();
    uint32_t currentMilliTime = millis();

    if (!rxIsDataDriven) {
        rxSignalReceived = rxSignalReceivedNotDataDriven;
        rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven;
    }

    if (!rxSignalReceived || rxIsInFailsafeMode) {
        useValueFromRx = false;
    }

#ifdef DEBUG_RX_SIGNAL_LOSS
    debug[0] = rxSignalReceived;
    debug[1] = rxIsInFailsafeMode;
    debug[2] = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, 0);
#endif

    rxResetFlightChannelStatus();

    for (channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {

        sample = (useValueFromRx) ? rcRaw[channel] : PPM_RCVR_TIMEOUT;

        bool validPulse = isPulseValid(sample);

        if (!validPulse) {
            if (currentMilliTime < rcInvalidPulsPeriod[channel]) {
                sample = rcData[channel];           // hold channel for MAX_INVALID_PULS_TIME
            } else {
                sample = getRxfailValue(channel);   // after that apply rxfail value
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

    if ((rxFlightChannelsValid) && !(IS_RC_MODE_ACTIVE(BOXFAILSAFE) && feature(FEATURE_FAILSAFE))) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven = true;
        failsafeOnValidDataFailed();

        for (channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
            rcData[channel] = getRxfailValue(channel);
        }
    }

#ifdef DEBUG_RX_SIGNAL_LOSS
    debug[3] = rcData[THROTTLE];
#endif
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
    detectAndApplySignalLossBehaviour();

    rcSampleIndex++;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
    }
}

void updateRSSIPWM(void)
{
    int16_t pwmRssi = 0;
    // Read value of AUX channel as rssi
    pwmRssi = rcData[rxConfig->rssi_channel - 1];

    // RSSI_Invert option
    if (rxConfig->rssi_ppm_invert) {
        pwmRssi = ((2000 - pwmRssi) + 1000);
    }

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    rssi = (uint16_t)((constrain(pwmRssi - 1000, 0, 1000) / 1000.0f) * 1023.0f);
}

#define RSSI_ADC_SAMPLE_COUNT 16
//#define RSSI_SCALE (0xFFF / 100.0f)

void updateRSSIADC(timeUs_t currentTimeUs)
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

    int16_t adcRssiMean = 0;
    uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint8_t rssiPercentage = adcRssiSample / rxConfig->rssi_scale;

    adcRssiSampleIndex = (adcRssiSampleIndex + 1) % RSSI_ADC_SAMPLE_COUNT;

    adcRssiSamples[adcRssiSampleIndex] = rssiPercentage;

    uint8_t sampleIndex;

    for (sampleIndex = 0; sampleIndex < RSSI_ADC_SAMPLE_COUNT; sampleIndex++) {
        adcRssiMean += adcRssiSamples[sampleIndex];
    }

    adcRssiMean = adcRssiMean / RSSI_ADC_SAMPLE_COUNT;

    rssi = (uint16_t)((constrain(adcRssiMean, 0, 100) / 100.0f) * 1023.0f);
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{

    if (rxConfig->rssi_channel > 0) {
        updateRSSIPWM();
    } else if (feature(FEATURE_RSSI_ADC)) {
        updateRSSIADC(currentTimeUs);
    }
}

uint16_t rxRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}
