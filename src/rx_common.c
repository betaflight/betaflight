#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "config.h"

#include "failsafe.h"

#include "rx_pwm.h"
#include "rx_sbus.h"
#include "rx_spektrum.h"
#include "rx_sumd.h"
#include "rx_msp.h"

#include "rx_common.h"

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
void sbusInit(rxConfig_t *initialRxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
void spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
void sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
void rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);

const char rcChannelLetters[] = "AERT1234";

int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

#define PPM_AND_PWM_SAMPLE_COUNT 4

#define PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   2250      // maximum PWM pulse width which is considered valid


static rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

rxRuntimeConfig_t rxRuntimeConfig;

void serialRxInit(rxConfig_t *rxConfig);

static failsafe_t *failsafe;

void rxInit(rxConfig_t *rxConfig, failsafe_t *initialFailsafe)
{
    uint8_t i;

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig->midrc;
    }

    failsafe = initialFailsafe;

    if (feature(FEATURE_SERIALRX)) {
        serialRxInit(rxConfig);
    } else {
        rxPwmInit(&rxRuntimeConfig, &rcReadRawFunc);
    }
}

void serialRxInit(rxConfig_t *rxConfig)
{
    switch (rxConfig->serialrx_type) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
            spektrumInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SBUS:
            sbusInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_SUMD:
            sumdInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
        case SERIALRX_MSP:
            rxMspInit(rxConfig, &rxRuntimeConfig, &rcReadRawFunc);
            break;
    }
}

bool isSerialRxFrameComplete(rxConfig_t *rxConfig)
{
    switch (rxConfig->serialrx_type) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
            return spektrumFrameComplete();
        case SERIALRX_SBUS:
            return sbusFrameComplete();
        case SERIALRX_SUMD:
            return sumdFrameComplete();
        case SERIALRX_MSP:
            return rxMspFrameComplete();
    }
    return false;
}

uint8_t calculateChannelRemapping(uint8_t *rcmap, uint8_t channelToRemap) {
    return rcmap[channelToRemap];
}

void computeRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    uint8_t chan;
    static int16_t rcSamples[MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT], rcDataMean[MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT];
    static uint8_t rcSampleIndex = 0;
    uint8_t currentSampleIndex = 0;

    if (feature(FEATURE_FAILSAFE)) {
        failsafe->vTable->incrementCounter();
    }

    if (!feature(FEATURE_SERIALRX)) {
        rcSampleIndex++;
        currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;
    }

    for (chan = 0; chan < MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT; chan++) {
        uint8_t rawChannel = calculateChannelRemapping(rxConfig->rcmap, chan);

        // sample the channel
        uint16_t sample = rcReadRawFunc(rxRuntimeConfig, rawChannel);

        if (feature(FEATURE_FAILSAFE)) {
            failsafe->vTable->checkPulse(rawChannel, sample);
        }

        // validate the range
        if (sample < PULSE_MIN || sample > PULSE_MAX)
            sample = rxConfig->midrc;

        if (feature(FEATURE_SERIALRX)) {
            rcData[chan] = sample;
            continue;
        }

        // update the recent samples and compute the average of them
        rcSamples[chan][currentSampleIndex] = sample;
        rcDataMean[chan] = 0;

        uint8_t sampleIndex;
        for (sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++)
            rcDataMean[chan] += rcSamples[chan][sampleIndex];

        rcData[chan] = rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
    }
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s)
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
    }
}

