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
#include "build_config.h"
#include "debug.h"


#include "common/maths.h"

#include "config/config.h"

#include "drivers/serial.h"
#include "drivers/adc.h"
#include "io/serial.h"
#include "io/rc_controls.h"

#include "flight/failsafe.h"

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/system.h"
#include "rx/pwm.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"

#include "rx/rx.h"


//#define DEBUG_RX_SIGNAL_LOSS

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);

bool sbusInit(rxConfig_t *initialRxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
bool sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
bool sumhInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);

void rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);

const char rcChannelLetters[] = "AERT12345678abcdefgh";

uint16_t rssi = 0;                  // range: [0;1023]

static bool rxDataReceived = false;
static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;

static uint32_t rxUpdateAt = 0;
static uint32_t needRxSignalBefore = 0;
static uint32_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_50_HZ (1000000 / 50)
#define DELAY_10_HZ (1000000 / 10)
#define DELAY_5_HZ (1000000 / 5)
#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeConfig_t rxRuntimeConfig;
static rxConfig_t *rxConfig;
static uint8_t rcSampleIndex = 0;

static uint16_t nullReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel) {
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static rcReadRawDataPtr rcReadRawFunc = nullReadRawRC;
static uint16_t rxRefreshRate;

void serialRxInit(rxConfig_t *rxConfig);

void useRxConfig(rxConfig_t *rxConfigToUse)
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

void rxInit(rxConfig_t *rxConfig)
{
    uint8_t i;

    useRxConfig(rxConfig);
    rcSampleIndex = 0;

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig->midrc;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig->midrc : rxConfig->rx_min_usec;

#ifdef SERIAL_RX
    if (feature(FEATURE_RX_SERIAL)) {
        serialRxInit(rxConfig);
    }
#endif

    if (feature(FEATURE_RX_MSP)) {
        rxMspInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
    }

    if (feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM)) {
        rxRefreshRate = 20000;
        rxPwmInit(&rxRuntimeConfig, &rcReadRawFunc);
    }

    rxRuntimeConfig.auxChannelCount = rxRuntimeConfig.channelCount - STICK_CHANNEL_COUNT;
}

#ifdef SERIAL_RX
void serialRxInit(rxConfig_t *rxConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
        case SERIALRX_SPEKTRUM1024:
            rxRefreshRate = 22000;
            enabled = spektrumInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SPEKTRUM2048:
            rxRefreshRate = 11000;
            enabled = spektrumInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SBUS:
            rxRefreshRate = 11000;
            enabled = sbusInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SUMD:
            rxRefreshRate = 11000;
            enabled = sumdInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SUMH:
            rxRefreshRate = 11000;
            enabled = sumhInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_XBUS_MODE_B:
        case SERIALRX_XBUS_MODE_B_RJ01:
            rxRefreshRate = 11000;
            enabled = xBusInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
    }

    if (!enabled) {
        featureClear(FEATURE_RX_SERIAL);
        rcReadRawFunc = nullReadRawRC;
    }
}

uint8_t serialRxFrameStatus(rxConfig_t *rxConfig)
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
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
            return spektrumFrameStatus();
        case SERIALRX_SBUS:
            return sbusFrameStatus();
        case SERIALRX_SUMD:
            return sumdFrameStatus();
        case SERIALRX_SUMH:
            return sumhFrameStatus();
        case SERIALRX_XBUS_MODE_B:
        case SERIALRX_XBUS_MODE_B_RJ01:
            return xBusFrameStatus();
    }
    return SERIAL_RX_FRAME_PENDING;
}
#endif

uint8_t calculateChannelRemapping(uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
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

static void resetRxSignalReceivedFlagIfNeeded(uint32_t currentTime)
{
    if (!rxSignalReceived) {
        return;
    }

    if (((int32_t)(currentTime - needRxSignalBefore) >= 0)) {
        rxSignalReceived = false;
#ifdef DEBUG_RX_SIGNAL_LOSS
        debug[0]++;
#endif
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

void updateRx(uint32_t currentTime)
{
    resetRxSignalReceivedFlagIfNeeded(currentTime);

    if (isRxDataDriven()) {
        rxDataReceived = false;
    }


#ifdef SERIAL_RX
    if (feature(FEATURE_RX_SERIAL)) {
        uint8_t frameStatus = serialRxFrameStatus(rxConfig);

        if (frameStatus & SERIAL_RX_FRAME_COMPLETE) {
            rxDataReceived = true;
            rxSignalReceived = (frameStatus & SERIAL_RX_FRAME_FAILSAFE) == 0;
            needRxSignalBefore = currentTime + DELAY_10_HZ;
        }
    }
#endif

    if (feature(FEATURE_RX_MSP)) {
        rxDataReceived = rxMspFrameComplete();

        if (rxDataReceived) {
            rxSignalReceived = true;
            needRxSignalBefore = currentTime + DELAY_5_HZ;
        }
    }

    if (feature(FEATURE_RX_PPM)) {
        if (isPPMDataBeingReceived()) {
            rxSignalReceived = true;
            needRxSignalBefore = currentTime + DELAY_10_HZ;
            resetPPMDataReceivedState();
        }
    }

    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        if (isPWMDataBeingReceived()) {
            rxSignalReceived = true;
            needRxSignalBefore = currentTime + DELAY_10_HZ;
        }
    }

}

bool shouldProcessRx(uint32_t currentTime)
{
    return rxDataReceived || ((int32_t)(currentTime - rxUpdateAt) >= 0); // data driven or 50Hz
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
    rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig->failsafe_channel_configurations[channel];

    switch(channelFailsafeConfiguration->mode) {
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
    uint8_t channel;

    for (channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {

        uint8_t rawChannel = calculateChannelRemapping(rxConfig->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = rcReadRawFunc(&rxRuntimeConfig, rawChannel);

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

    if (!rxSignalReceived) {
        if (rxIsDataDriven && rxDataReceived) {
            // use the values from the RX
        } else {
            useValueFromRx = false;
        }
    }

    rxResetFlightChannelStatus();

    for (channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {

        sample = (useValueFromRx) ? rcRaw[channel] : PPM_RCVR_TIMEOUT;

        bool validPulse = isPulseValid(sample);

        if (!validPulse) {
            sample = getRxfailValue(channel);
        }

        rxUpdateFlightChannelStatus(channel, validPulse);

        if (rxIsDataDriven) {
            rcData[channel] = sample;
        } else {
            rcData[channel] = calculateNonDataDrivenChannel(channel, sample);
        }
    }

    rxFlightChannelsValid = rxHaveValidFlightChannels();

    if ((rxFlightChannelsValid) && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        failsafeOnValidDataReceived();
    } else {
        rxSignalReceived = false;
        failsafeOnValidDataFailed();

        for (channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
            rcData[channel] = getRxfailValue(channel);
        }
    }

}

void calculateRxChannelsAndUpdateFailsafe(uint32_t currentTime)
{
    rxUpdateAt = currentTime + DELAY_50_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples) {
        if (currentTime > suspendRxSignalUntil) {
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

void updateRSSIADC(uint32_t currentTime)
{
#ifndef USE_ADC
    UNUSED(currentTime);
#else
    static uint8_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
    static uint8_t adcRssiSampleIndex = 0;
    static uint32_t rssiUpdateAt = 0;

    if ((int32_t)(currentTime - rssiUpdateAt) < 0) {
        return;
    }
    rssiUpdateAt = currentTime + DELAY_50_HZ;

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

void updateRSSI(uint32_t currentTime)
{

    if (rxConfig->rssi_channel > 0) {
        updateRSSIPWM();
    } else if (feature(FEATURE_RSSI_ADC)) {
        updateRSSIADC(currentTime);
    }
}

void initRxRefreshRate(uint16_t *rxRefreshRatePtr) {
    *rxRefreshRatePtr = rxRefreshRate;
}

