#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "rx_common.h"
#include "config.h"

#include "drivers/pwm_common.h"


int16_t rcData[RC_CHANS];       // interval [1000;2000]

rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

uint16_t pwmReadRawRC(rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t data;

    data = pwmRead(rxConfig->rcmap[chan]);
    if (data < 750 || data > 2250)
        data = rxConfig->midrc;

    return data;
}

void computeRC(rxConfig_t *rxConfig)
{
    uint8_t chan;

    if (feature(FEATURE_SERIALRX)) {
        for (chan = 0; chan < 8; chan++)
            rcData[chan] = rcReadRawFunc(rxConfig, chan);
    } else {
        static int16_t rcData4Values[8][4], rcDataMean[8];
        static uint8_t rc4ValuesIndex = 0;
        uint8_t a;

        rc4ValuesIndex++;
        for (chan = 0; chan < 8; chan++) {
            rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(rxConfig, chan);
            rcDataMean[chan] = 0;
            for (a = 0; a < 4; a++)
                rcDataMean[chan] += rcData4Values[chan][a];

            rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
            if (rcDataMean[chan] < rcData[chan] - 3)
                rcData[chan] = rcDataMean[chan] + 2;
            if (rcDataMean[chan] > rcData[chan] + 3)
                rcData[chan] = rcDataMean[chan] - 2;
        }
    }
}
