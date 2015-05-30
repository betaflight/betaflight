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

#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 10
#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500


// Note: this is called MultiType/MULTITYPE_* in baseflight.
typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BI = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_CUSTOM = 23
} mixerMode_e;

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct mixerConfig_s {
    uint8_t pid_at_min_throttle;            // when enabled pids are used at minimum throttle
    int8_t yaw_direction;
    uint16_t yaw_jump_prevention_limit;      // make limit configurable (original fixed value was 100)
#ifdef USE_SERVOS
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    int16_t servo_lowpass_freq;             // lowpass servo filter frequency selection; 1/1000ths of loop freq
    int8_t servo_lowpass_enable;            // enable/disable lowpass filter
#endif
} mixerConfig_t;

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
} flight3DConfig_t;

typedef struct airplaneConfig_t {
    uint8_t flaps_speed;                    // airplane mode flaps, 0 = no flaps, > 0 = flap speed, larger = faster
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign
} airplaneConfig_t;

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

#ifdef USE_SERVOS
typedef struct servoParam_t {
    int16_t min;                            // servo min
    int16_t max;                            // servo max
    int16_t middle;                         // servo middle
    int8_t rate;                            // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
    int8_t forwardFromChannel;              // RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
} servoParam_t;

struct gimbalConfig_s;
struct escAndServoConfig_s;
struct rxConfig_s;

extern int16_t servo[MAX_SUPPORTED_SERVOS];
bool isMixerUsingServos(void);
void writeServos(void);
void filterServos(void);
#endif

extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

void mixerUseConfigs(
#ifdef USE_SERVOS
        servoParam_t *servoConfToUse,
        struct gimbalConfig_s *gimbalConfigToUse,
#endif
        flight3DConfig_t *flight3DConfigToUse,
		struct escAndServoConfig_s *escAndServoConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
		struct rxConfig_s *rxConfigToUse);

void writeAllMotors(int16_t mc);
void mixerLoadMix(int index, motorMixer_t *customMixers);
void mixerResetMotors(void);
void mixTable(void);
void writeMotors(void);
void stopMotors(void);
void StopPwmAllMotors(void);
