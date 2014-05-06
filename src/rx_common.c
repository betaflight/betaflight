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

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *failsafe, rcReadRawDataPtr *callback);
void sbusInit(rxConfig_t *initialRxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback);
void spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback);
void sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback);
void rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback);

const char rcChannelLetters[] = "AERT1234";

int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

#define PPM_AND_PWM_SAMPLE_COUNT 4

rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

rxRuntimeConfig_t rxRuntimeConfig;

void serialRxInit(rxConfig_t *rxConfig, failsafe_t *failsafe);

void rxInit(rxConfig_t *rxConfig, failsafe_t *failsafe)
{
    uint8_t i;

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig->midrc;
    }

    if (feature(FEATURE_SERIALRX)) {
        serialRxInit(rxConfig, failsafe);
    } else {
        rxPwmInit(&rxRuntimeConfig, failsafe, &rcReadRawFunc);
    }
}

void serialRxInit(rxConfig_t *rxConfig, failsafe_t *failsafe)
{
    switch (rxConfig->serialrx_type) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
            spektrumInit(rxConfig, &rxRuntimeConfig, failsafe, &rcReadRawFunc);
            break;
        case SERIALRX_SBUS:
            sbusInit(rxConfig, &rxRuntimeConfig, failsafe, &rcReadRawFunc);
            break;
        case SERIALRX_SUMD:
            sumdInit(rxConfig, &rxRuntimeConfig, failsafe, &rcReadRawFunc);
            break;
        case SERIALRX_MSP:
            rxMspInit(rxConfig, &rxRuntimeConfig, failsafe, &rcReadRawFunc);
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

void computeRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    uint8_t chan;

    if (feature(FEATURE_SERIALRX)) {
        for (chan = 0; chan < MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT; chan++)
            rcData[chan] = rcReadRawFunc(rxConfig, rxRuntimeConfig, chan);
    } else {
        static int16_t rcSamples[MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT], rcDataMean[MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT];
        static uint8_t rcSampleIndex = 0;
        uint8_t a;

        rcSampleIndex++;
        uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

        for (chan = 0; chan < MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT; chan++) {

            // sample the channel
            rcSamples[chan][currentSampleIndex] = rcReadRawFunc(rxConfig, rxRuntimeConfig, chan);

            // compute the average of recent samples
            rcDataMean[chan] = 0;
            for (a = 0; a < PPM_AND_PWM_SAMPLE_COUNT; a++)
                rcDataMean[chan] += rcSamples[chan][a];

            rcDataMean[chan] = (rcDataMean[chan] + 2) / PPM_AND_PWM_SAMPLE_COUNT;


            if (rcDataMean[chan] < rcData[chan] - 3)
                rcData[chan] = rcDataMean[chan] + 2;
            if (rcDataMean[chan] > rcData[chan] + 3)
                rcData[chan] = rcDataMean[chan] - 2;
        }
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

