#pragma once

#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,          // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_HEX6H = 18,
    MULTITYPE_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MULTITYPE_DUALCOPTER = 20,
    MULTITYPE_SINGLECOPTER = 21,
    MULTITYPE_CUSTOM = 22,          // no current GUI displays this
    MULTITYPE_LAST = 23
} MultiType;

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

#define CHANNEL_FORWARDING_DISABLED 0xFF

typedef struct servoParam_t {
    int16_t min;                            // servo min
    int16_t max;                            // servo max
    int16_t middle;                         // servo middle
    int8_t rate;                            // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
    int8_t forwardFromChannel;              // RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
} servoParam_t;

bool isMixerUsingServos(void);
