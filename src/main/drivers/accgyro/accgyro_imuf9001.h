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

#include "drivers/bus.h"

uint8_t imuf9001SpiDetect(const gyroDev_t *bus);

bool imufSpiAccDetect(accDev_t *acc);
bool imufSpiGyroDetect(gyroDev_t *gyro);

void imufSpiGyroInit(gyroDev_t *gyro);
void imufSpiAccInit(accDev_t *acc);

void imufStartCalibration(void);

#ifndef IMUF_DEFAULT_PITCH_Q
#define IMUF_DEFAULT_PITCH_Q  1500
#endif
#ifndef IMUF_DEFAULT_ROLL_Q
#define IMUF_DEFAULT_ROLL_Q  1500
#endif
#ifndef IMUF_DEFAULT_YAW_Q
#define IMUF_DEFAULT_YAW_Q  1500
#endif
#ifndef IMUF_DEFAULT_PITCH_W
#define IMUF_DEFAULT_PITCH_W  10
#endif
#ifndef IMUF_DEFAULT_ROLL_W
#define IMUF_DEFAULT_ROLL_W  10
#endif
#ifndef IMUF_DEFAULT_YAW_W
#define IMUF_DEFAULT_YAW_W  10
#endif



volatile uint32_t isImufCalibrating;

typedef struct imufVersion
{   
    uint32_t hardware;
    uint32_t firmware;
    uint32_t bootloader;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} __attribute__((__packed__)) imufVersion_t;

typedef struct imufCommand {
   uint32_t command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   uint32_t param9;
   uint32_t param10;
   uint32_t crc;
   uint32_t tail;
} __attribute__ ((__packed__)) imufCommand_t;

typedef enum gyroCommands
{
    IMUF_COMMAND_NONE            = 0,
    IMUF_COMMAND_CALIBRATE       = 99,
    IMUF_COMMAND_LISTENING       = 108,
    IMUF_COMMAND_REPORT_INFO     = 121,
    IMUF_COMMAND_SETUP           = 122,
    IMUF_COMMAND_RESTART         = 127,
} gyroCommands_t;

typedef struct gyroFrame
{
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;
    float temp;
} __attribute__((__packed__)) gyroFrame_t;

typedef struct imuFrame
{
    float w;
    float x;
    float y;
    float z;
} __attribute__((__packed__)) imuFrame_t;

typedef struct imuCommFrame
{
    gyroFrame_t gyroFrame;
    imuFrame_t  imuFrame;
    uint32_t    tail;
} __attribute__((__packed__)) imuCommFrame_t;

typedef enum imufLoopHz
{
    IMUF_32000 = 0,
    IMUF_16000 = 1,
    IMUF_8000  = 2,
    IMUF_4000  = 3,
    IMUF_2000  = 4,
    IMUF_1000  = 5,
    IMUF_500   = 6,
    IMUF_250   = 7,
} imufLoopHz_t;

typedef enum imufOutput
{
    IMUF_GYRO_OUTPUT = 1 << 0,
    IMUF_TEMP_OUTPUT = 1 << 1,
    IMUF_ACC_OUTPUT  = 1 << 2,
    IMUF_QUAT_OUTPUT = 1 << 3,
} imufOutput_t;

typedef enum imufOreintation
{
    IMU_CW0       = 0,
    IMU_CW90      = 1,
    IMU_CW180     = 2,
    IMU_CW270     = 3,
    IMU_CW0_INV   = 4,
    IMU_CW90_INV  = 5,
    IMU_CW180_INV = 6,
    IMU_CW270_INV = 7,
    IMU_CW45      = 8,
    IMU_CW135     = 9,
    IMU_CW225     = 10,
    IMU_CW315     = 11,
    IMU_CW45_INV  = 12,
    IMU_CW135_INV = 13,
    IMU_CW225_INV = 14,
    IMU_CW315_INV = 15,
    IMU_CUSTOM    = 16,
} imufOrientation_t;

typedef struct imufMode
{
    uint8_t  command;       //output Hz
    uint8_t  hz;            //output Hz
    uint8_t  dataOut;       //what data to send
    uint8_t  filterLevelX;  //what filter level, 0% to 100% as a uint8_t
    uint8_t  filterLevelY;  //what filter level, 0% to 100% as a uint8_t
    uint8_t  filterLevelZ;  //what filter level, 0% to 100% as a uint8_t
    uint8_t  orientation;   //what orienetation is the IMU? 0 gives raw output, if you want to use quats this must be set right
    uint16_t rotationX;     //custom orientation X, used when orientation is set to IMU_CUSTOM
    uint16_t rotationY;     //custom orientation Y, used when orientation is set to IMU_CUSTOM
    uint16_t rotationZ;     //custom orientation Z, used when orientation is set to IMU_CUSTOM
    uint8_t  param4;         //future parameters
    uint8_t  param5;         //future parameters
    uint8_t  param6;         //future parameters
    uint8_t  param7;         //future parameters
    uint8_t  param8;         //future parameters
} __attribute__((__packed__)) imufMode_t;

typedef enum gyroToBoardCommMode
{
    GTBCM_SETUP                  = 53, //setup
    GTBCM_GYRO_ONLY_PASSTHRU     = 6,  //no crc, gyro, 3*2 bytes
    GTBCM_GYRO_ACC_PASSTHRU      = 14, //no crc, acc, temp, gyro, 3*2, 1*2, 3*2 bytes
    GTBCM_GYRO_ONLY_FILTER_F     = 20, //gyro filtered, 3*4 bytes, 4 bytes crc
    GTBCM_GYRO_ACC_FILTER_F      = 32, //gyro filtered, acc filtered, temp, crc
    GTBCM_GYRO_ACC_QUAT_FILTER_F = 48, //gyro filtered, acc filtered, temp, quaternions filtered, crc
    GTBCM_DEFAULT                = GTBCM_GYRO_ACC_FILTER_F, //default mode
} gyroToBoardCommMode_t;

typedef enum imufCalibrationSteps
{
    IMUF_NOT_CALIBRATING   = 0,
    IMUF_CALIBRATION_STEP1 = 1,
    IMUF_CALIBRATION_STEP2 = 2,

} imufCalibrationSteps_t;

extern uint32_t getCrcImuf9001(uint32_t* data, uint32_t size);