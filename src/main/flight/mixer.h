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

#define QUAD_MOTOR_COUNT 4

/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings
  0 = stop
  1-5: beep
  6: ESC info request (FW Version and SN sent over the tlm wire)
  7: spin direction 1
  8: spin direction 2
  9: 3d mode off
  10: 3d mode on
  11: ESC settings request (saved settings over the TLM wire)
  12: save Settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> positive direction
  1048 (low) - 2047 (high) -> negative direction
*/

// Digital protocol has fixed values
#define DSHOT_DISARM_COMMAND      0
#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_DEADBAND_LOW  1047
#define DSHOT_3D_DEADBAND_HIGH 1048

// Note: this is called MultiType/MULTITYPE_* in baseflight.
typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
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
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25,
    MIXER_QUADX_1234 = 26
} mixerMode_e;

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_s {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct mixerConfig_s {
    uint8_t mixerMode;
    int8_t yaw_motor_direction;
} mixerConfig_t;

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
} flight3DConfig_t;

typedef struct airplaneConfig_s {
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign
} airplaneConfig_t;

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

extern const mixer_t mixers[];
extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
struct motorConfig_s;
struct rxConfig_s;

uint8_t getMotorCount();

void mixerUseConfigs(
        flight3DConfig_t *flight3DConfigToUse,
        struct motorConfig_s *motorConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
        struct rxConfig_s *rxConfigToUse);

void mixerLoadMix(int index, motorMixer_t *customMixers);
void mixerInit(mixerMode_e mixerMode, motorMixer_t *customMotorMixers);

void mixerConfigureOutput(void);

void mixerResetDisarmedMotors(void);
struct pidProfile_s;
void mixTable(struct pidProfile_s *pidProfile);
void syncMotors(bool enabled);
void writeMotors(void);
void stopMotors(void);
void stopPwmAllMotors(void);

bool isMotorProtocolDshot(void);
uint16_t convertExternalToMotor(uint16_t externalValue);
uint16_t convertMotorToExternal(uint16_t motorValue);
