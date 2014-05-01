#pragma once

#define PWM_RANGE_ZERO 0 // FIXME should all usages of this be changed to use PWM_RANGE_MIN?
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000

#define DEFAULT_SERVO_MIN 1020
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_MSP = 4,
    SERIALRX_PROVIDER_MAX = SERIALRX_MSP
} SerialRXType;

#define MAX_SUPPORTED_RC_PPM_AND_PWM_CHANNEL_COUNT 8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT (18)

extern const char rcChannelLetters[];

extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

typedef struct rxConfig_s {
    uint8_t rcmap[8];                       // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_type;                  // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_SERIALRX first.
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
} rxConfig_t;

typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;                  // number of rc channels as reported by current input driver
} rxRuntimeConfig_t;

extern rxRuntimeConfig_t rxRuntimeConfig;

typedef uint16_t (* rcReadRawDataPtr)(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);        // used by receiver driver to return channel data

void computeRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void parseRcChannels(const char *input, rxConfig_t *rxConfig);
bool isSerialRxFrameComplete(rxConfig_t *rxConfig);
