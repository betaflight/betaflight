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

typedef enum {
    RC_INTERP_RP = 0, // roll and pitch
    RC_INTERP_RPY = 1, //roll, pitch, and yaw
    RC_INTERP_RPYT = 2 //roll, pitch, yaw, and throttle
} rcInterpChannels_e;

#ifdef USE_GYRO_IMUF9001
extern volatile bool isSetpointNew;
#endif
void processRcCommand(void);
float getSetpointRate(int axis);
uint32_t getSetpointRateInt(int axis);
float getRcDeflection(int axis);
float getRcDeflectionAbs(int axis);
float getThrottlePIDAttenuation(void);
void updateRcCommands(void);
void resetYawAxis(void);
void initRcProcessing(void);
bool isMotorsReversed(void);

#if defined(USE_TPA_CURVES)
float getThrottlePIDAttenuationKp(void);
float getThrottlePIDAttenuationKi(void);
float getThrottlePIDAttenuationKd(void);
#endif
