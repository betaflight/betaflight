/*
 * This file is part of Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * =================================================================
 *
 * Most of the functionality of this library is based on the VL53L1X
 * API provided by ST (STSW-IMG009).
 *
 * The following applies to source code reproduced or derived from
 * the API:
 *
 * -----------------------------------------------------------------
 *
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 * 
 *  This file : part of VL53L1 Core and : dual licensed,
 *  either 'STMicroelectronics
 *  Proprietary license'
 *  or 'BSD 3-clause "New" or "Revised" License' , at your option.
 * 
 * *******************************************************************************
 * 
 *  'STMicroelectronics Proprietary license'
 * 
 * *******************************************************************************
 * 
 *  License terms: STMicroelectronics Proprietary in accordance with licensing
 *  terms at www.st.com/sla0081
 * 
 *  STMicroelectronics confidential
 *  Reproduction and Communication of this document : strictly prohibited unless
 *  specifically authorized in writing by STMicroelectronics.
 * 
 * 
 * *******************************************************************************
 * 
 *  Alternatively, VL53L1 Core may be distributed under the terms of
 *  'BSD 3-clause "New" or "Revised" License', in which case the following
 *  provisions apply instead of the ones mentioned above :
 * 
 * *******************************************************************************
 * 
 *  License terms: BSD 3-clause "New" or "Revised" License.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software
 *  without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_VL53L1X)

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/bus.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_vl53l1x.h"

#include "build/debug.h"

#define VL53L1X_I2C_ADDR    0x29

#define VL53L1X_MAX_RANGE_CM                                (300)
#define VL53L1X_DETECTION_CONE_DECIDEGREES                  (270)
#define VL53L1X_TIMING_BUDGET                               (33)

#define VL53L1X_BOOT_TIMEOUT_MS     (500)

#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       4
#define VL53L1X_IMPLEMENTATION_VER_SUB         0
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

typedef int8_t VL53L1X_ERROR;

#define SOFT_RESET                                          0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                    0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS      0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS  0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS  0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                  0x001E
#define MM_CONFIG__INNER_OFFSET_MM                          0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                          0x0022
#define GPIO_HV_MUX__CTRL                                   0x0030
#define GPIO__TIO_HV_STATUS                                 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                       0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                     0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                   0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                        0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                        0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                   0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                   0x0062
#define RANGE_CONFIG__SIGMA_THRESH                          0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS         0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                      0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD              0x006C
#define SYSTEM__THRESH_HIGH                                 0x0072
#define SYSTEM__THRESH_LOW                                  0x0074
#define SD_CONFIG__WOI_SD0                                  0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                        0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                    0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE       0x0080
#define SYSTEM__SEQUENCE_CONFIG                             0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD               0x0082
#define SYSTEM__INTERRUPT_CLEAR                             0x0086
#define SYSTEM__MODE_START                                  0x0087
#define VL53L1_RESULT__RANGE_STATUS                         0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0       0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                  0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0               0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0  0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                    0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD             0x013E

#define ALGO__PART_TO_PART_RANGE_OFFSET_MM  0x001E
#define MM_CONFIG__INNER_OFFSET_MM          0x0020
#define MM_CONFIG__OUTER_OFFSET_MM          0x0022



#define VL53L1_ERROR_NONE                              ((VL53L1X_ERROR)  0)
#define VL53L1_ERROR_CALIBRATION_WARNING               ((VL53L1X_ERROR) - 1)
    /*!< Warning invalid calibration data may be in used
        \a  VL53L1_InitData()
        \a VL53L1_GetOffsetCalibrationData
        \a VL53L1_SetOffsetCalibrationData */
#define VL53L1_ERROR_MIN_CLIPPED                       ((VL53L1X_ERROR) - 2)
    /*!< Warning parameter passed was clipped to min before to be applied */

#define VL53L1_ERROR_UNDEFINED                         ((VL53L1X_ERROR) - 3)
    /*!< Unqualified error */
#define VL53L1_ERROR_INVALID_PARAMS                    ((VL53L1X_ERROR) - 4)
    /*!< Parameter passed is invalid or out of range */
#define VL53L1_ERROR_NOT_SUPPORTED                     ((VL53L1X_ERROR) - 5)
    /*!< Function is not supported in current mode or configuration */
#define VL53L1_ERROR_RANGE_ERROR                       ((VL53L1X_ERROR) - 6)
    /*!< Device report a ranging error interrupt status */
#define VL53L1_ERROR_TIME_OUT                          ((VL53L1X_ERROR) - 7)
    /*!< Aborted due to time out */
#define VL53L1_ERROR_MODE_NOT_SUPPORTED                ((VL53L1X_ERROR) - 8)
    /*!< Asked mode is not supported by the device */
#define VL53L1_ERROR_BUFFER_TOO_SMALL                  ((VL53L1X_ERROR) - 9)
    /*!< ... */
#define VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL            ((VL53L1X_ERROR) - 10)
    /*!< Supplied buffer is larger than I2C supports */
#define VL53L1_ERROR_GPIO_NOT_EXISTING                 ((VL53L1X_ERROR) - 11)
    /*!< User tried to setup a non-existing GPIO pin */
#define VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  ((VL53L1X_ERROR) - 12)
    /*!< unsupported GPIO functionality */
#define VL53L1_ERROR_CONTROL_INTERFACE                 ((VL53L1X_ERROR) - 13)
    /*!< error reported from IO functions */
#define VL53L1_ERROR_INVALID_COMMAND                   ((VL53L1X_ERROR) - 14)
    /*!< The command is not allowed in the current device state
     *  (power down) */
#define VL53L1_ERROR_DIVISION_BY_ZERO                  ((VL53L1X_ERROR) - 15)
    /*!< In the function a division by zero occurs */
#define VL53L1_ERROR_REF_SPAD_INIT                     ((VL53L1X_ERROR) - 16)
    /*!< Error during reference SPAD initialization */
#define VL53L1_ERROR_GPH_SYNC_CHECK_FAIL               ((VL53L1X_ERROR) - 17)
    /*!<  GPH sync interrupt check fail - API out of sync with device*/
#define VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL           ((VL53L1X_ERROR) - 18)
    /*!<  Stream count check fail - API out of sync with device */
#define VL53L1_ERROR_GPH_ID_CHECK_FAIL                 ((VL53L1X_ERROR) - 19)
    /*!<  GPH ID check fail - API out of sync with device */
#define VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL      ((VL53L1X_ERROR) - 20)
    /*!<  Zone dynamic config stream count check failed - API out of sync */
#define VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL            ((VL53L1X_ERROR) - 21)
    /*!<  Zone dynamic config GPH ID check failed - API out of sync */

#define VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL   ((VL53L1X_ERROR) - 22)
    /*!<  Thrown when run_xtalk_extraction fn has 0 successful samples
     * when using the full array to sample the xtalk. In this case there is
     * not enough information to generate new Xtalk parm info. The function
     * will exit and leave the current xtalk parameters unaltered */
#define VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL ((VL53L1X_ERROR) - 23)
    /*!<  Thrown when run_xtalk_extraction fn has found that the
     * avg sigma estimate of the full array xtalk sample is > than the
     * maximal limit allowed. In this case the xtalk sample is too noisy for
     * measurement. The function will exit and leave the current xtalk parameters
     * unaltered. */


#define VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL           ((VL53L1X_ERROR) - 24)
    /*!<  Thrown if there one of stages has no valid offset calibration
     *    samples. A fatal error calibration not valid */
#define VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL    ((VL53L1X_ERROR) - 25)
    /*!<  Thrown if there one of stages has zero effective SPADS
     *    Traps the case when MM1 SPADs is zero.
     *    A fatal error calibration not valid */
#define VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL             ((VL53L1X_ERROR) - 26)
    /*!<  Thrown if then some of the zones have no valid samples
     *    A fatal error calibration not valid */

#define VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH             ((VL53L1X_ERROR) - 27)
    /*!<  Thrown if the tuning file key table version does not match with
     * expected value. The driver expects the key table version to match
     * the compiled default version number in the define
     * #VL53L1_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT
     * */

#define VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS   ((VL53L1X_ERROR) - 28)
    /*!<  Thrown if there are less than 5 good SPADs are available. */
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH      ((VL53L1X_ERROR) - 29)
    /*!<  Thrown if the final reference rate is greater than
          the upper reference rate limit - default is 40 Mcps.
          Implies a minimum Q3 (x10) SPAD (5) selected */
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW       ((VL53L1X_ERROR) - 30)
    /*!<  Thrown if the final reference rate is less than
          the lower reference rate limit - default is 10 Mcps.
          Implies maximum Q1 (x1) SPADs selected */


#define VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES       ((VL53L1X_ERROR) - 31)
    /*!<  Thrown if there is less than the requested number of
     *    valid samples. */
#define VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH        ((VL53L1X_ERROR) - 32)
    /*!<  Thrown if the offset calibration range sigma estimate is greater
     *    than 8.0 mm. This is the recommended min value to yield a stable
     *    offset measurement */
#define VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH         ((VL53L1X_ERROR) - 33)
    /*!< Thrown when VL53L1_run_offset_calibration()  peak rate is greater
         than that 50.0Mcps. This is the recommended  max rate to avoid
         pile-up influencing the offset measurement */
#define VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW    ((VL53L1X_ERROR) - 34)
    /*!< Thrown when VL53L1_run_offset_calibration() when one of stages
         range has less that 5.0 effective SPADS. This is the recommended
         min value to yield a stable offset */


#define VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES       ((VL53L1X_ERROR) - 35)
    /*!<  Thrown if one of more of the zones have less than
          the requested number of valid samples */
#define VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH        ((VL53L1X_ERROR) - 36)
    /*!<  Thrown if one or more zones have sigma estimate value greater
     *    than 8.0 mm. This is the recommended min value to yield a stable
     *    offset measurement */
#define VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH         ((VL53L1X_ERROR) - 37)
    /*!< Thrown if one of more zones have  peak rate higher than
          that 50.0Mcps. This is the recommended  max rate to avoid
         pile-up influencing the offset measurement */


#define VL53L1_WARNING_XTALK_MISSING_SAMPLES             ((VL53L1X_ERROR) - 38)
    /*!< Thrown to notify that some of the xtalk samples did not yield
     * valid ranging pulse data while attempting to measure
     * the xtalk signal in vl53l1_run_xtalk_extract(). This can signify any of
     * the zones are missing samples, for further debug information the
     * xtalk_results struct should be referred to. This warning is for
     * notification only, the xtalk pulse and shape have still been generated
     */
#define VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT     ((VL53L1X_ERROR) - 39)
    /*!< Thrown to notify that some of teh xtalk samples used for gradient
     * generation did not yield valid ranging pulse data while attempting to
     * measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
     * that any one of the zones 0-3 yielded no successful samples. The
     * xtalk_results struct should be referred to for further debug info.
     * This warning is for notification only, the xtalk pulse and shape
     * have still been generated.
     */
#define VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT    ((VL53L1X_ERROR) - 40)
/*!< Thrown to notify that some of the xtalk samples used for gradient
     * generation did not pass the sigma limit check  while attempting to
     * measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
     * that any one of the zones 0-3 yielded an avg sigma_mm value > the limit.
     * The xtalk_results struct should be referred to for further debug info.
     * This warning is for notification only, the xtalk pulse and shape
     * have still been generated.
     */

#define VL53L1_ERROR_NOT_IMPLEMENTED                   ((VL53L1X_ERROR) - 41)
    /*!< Tells requested functionality has not been implemented yet or
     * not compatible with the device */
#define VL53L1_ERROR_PLATFORM_SPECIFIC_START           ((VL53L1X_ERROR) - 60)
    /*!< Tells the starting code for platform */
/** @} VL53L1_define_Error_group */

/****************************************
 * PRIVATE define do not edit
 ****************************************/

/**
 *  @brief defines SW Version
 */
typedef struct {
    uint8_t      major;    /*!< major number */
    uint8_t      minor;    /*!< minor number */
    uint8_t      build;    /*!< build number */
    uint32_t     revision; /*!< revision number */
} VL53L1X_Version_t;

/**
 *  @brief defines packed reading results type
 */
typedef struct {
    uint8_t Status;     /*!< ResultStatus */
    uint16_t Distance;  /*!< ResultDistance */
    uint16_t Ambient;   /*!< ResultAmbient */
    uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
    uint16_t NumSPADs;  /*!< ResultNumSPADs */
} VL53L1X_Result_t;

/**
 * @brief This function returns the SW driver version
 */
// VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion);

/**
 * @brief This function sets the sensor I2C address used in case multiple devices application, default address 0x52
 */
// // VL53L1X_ERROR VL53L1X_SetI2CAddress(busDevice_t * dev, uint8_t new_address);

// /**
//  * @brief This function loads the 135 bytes default values to initialize the sensor.
//  * @param dev Device address
//  * @return 0:success, != 0:failed
//  */
VL53L1X_ERROR VL53L1X_SensorInit(const extDevice_t *dev);

/**
 * @brief This function clears the interrupt, to be called after a ranging data reading
 * to arm the interrupt for the next data ready event.
 */
static VL53L1X_ERROR VL53L1X_ClearInterrupt(const extDevice_t *dev);

/**
 * @brief This function programs the interrupt polarity\n
 * 1=active high (default), 0=active low
 */
// VL53L1X_ERROR VL53L1X_SetInterruptPolarity(busDevice_t * dev, uint8_t IntPol);

/**
 * @brief This function returns the current interrupt polarity\n
 * 1=active high (default), 0=active low
 */
static VL53L1X_ERROR VL53L1X_GetInterruptPolarity(const extDevice_t *dev, uint8_t *pIntPol);

/**
 * @brief This function starts the ranging distance operation\n
 * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
 * 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
 */
static VL53L1X_ERROR VL53L1X_StartRanging(const extDevice_t *dev);

/**
 * @brief This function stops the ranging.
 */
static VL53L1X_ERROR VL53L1X_StopRanging(const extDevice_t *dev);

/**
 * @brief This function checks if the new ranging data is available by polling the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
static VL53L1X_ERROR VL53L1X_CheckForDataReady(const extDevice_t *dev, uint8_t *isDataReady);

/**
 * @brief This function programs the timing budget in ms.
 * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
 */
static VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(const extDevice_t *dev, uint16_t TimingBudgetInMs);

/**
 * @brief This function returns the current timing budget in ms.
 */
static VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(const extDevice_t *dev, uint16_t *pTimingBudgetInMs);

/**
 * @brief This function programs the distance mode (1=short, 2=long(default)).
 * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
 * Long mode can range up to 4 m in the dark with 200 ms timing budget.
 */
static VL53L1X_ERROR VL53L1X_SetDistanceMode(const extDevice_t *dev, uint16_t DistanceMode);

/**
 * @brief This function returns the current distance mode (1=short, 2=long).
 */
static VL53L1X_ERROR VL53L1X_GetDistanceMode(const extDevice_t *dev, uint16_t *pDistanceMode);

/**
 * @brief This function programs the Intermeasurement period in ms\n
 * Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
 * the customer has the duty to check the condition. Default = 100 ms
 */
static VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(const extDevice_t *dev,
                     uint32_t InterMeasurementInMs);

/**
 * @brief This function returns the Intermeasurement period in ms.
 */
// VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(const extDevice_t *dev, uint16_t * pIM);

/**
 * @brief This function returns the boot state of the device (1:booted, 0:not booted)
 */
// VL53L1X_ERROR VL53L1X_BootState(const extDevice_t *dev, uint8_t *state);

/**
 * @brief This function returns the sensor id, sensor Id must be 0xEEAC
 */
// VL53L1X_ERROR VL53L1X_GetSensorId(const extDevice_t *dev, uint16_t *id);

/**
 * @brief This function returns the distance measured by the sensor in mm
 */
static VL53L1X_ERROR VL53L1X_GetDistance(const extDevice_t *dev, uint16_t *distance);

/**
 * @brief This function returns the returned signal per SPAD in kcps/SPAD.
 * With kcps stands for Kilo Count Per Second
 */
// VL53L1X_ERROR VL53L1X_GetSignalPerSpad(const extDevice_t *dev, uint16_t *signalPerSp);

/**
 * @brief This function returns the ambient per SPAD in kcps/SPAD
 */
// VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(const extDevice_t *dev, uint16_t *amb);

/**
 * @brief This function returns the returned signal in kcps.
 */
// static VL53L1X_ERROR VL53L1X_GetSignalRate(const extDevice_t *dev, uint16_t *signalRate);

/**
 * @brief This function returns the current number of enabled SPADs
 */
// static VL53L1X_ERROR VL53L1X_GetSpadNb(const extDevice_t *dev, uint16_t *spNb);

/**
 * @brief This function returns the ambient rate in kcps
 */
// VL53L1X_ERROR VL53L1X_GetAmbientRate(const extDevice_t *dev, uint16_t *ambRate);

/**
 * @brief This function returns the ranging status error \n
 * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
 */
// VL53L1X_ERROR VL53L1X_GetRangeStatus(const extDevice_t *dev, uint8_t *rangeStatus);

/**
 * @brief This function returns measurements and the range status in a single read access
 */
// VL53L1X_ERROR VL53L1X_GetResult(const extDevice_t *dev, VL53L1X_Result_t *pResult);

/**
 * @brief This function programs the offset correction in mm
 * @param OffsetValue:the offset correction value to program in mm
 */
// VL53L1X_ERROR VL53L1X_SetOffset(const extDevice_t *dev, int16_t OffsetValue);

/**
 * @brief This function returns the programmed offset correction value in mm
 */
// VL53L1X_ERROR VL53L1X_GetOffset(const extDevice_t *dev, int16_t *Offset);

/**
 * @brief This function programs the xtalk correction value in cps (Count Per Second).\n
 * This is the number of photons reflected back from the cover glass in cps.
 */
// VL53L1X_ERROR VL53L1X_SetXtalk(const extDevice_t *dev, uint16_t XtalkValue);

/**
 * @brief This function returns the current programmed xtalk correction value in cps
 */
// VL53L1X_ERROR VL53L1X_GetXtalk(const extDevice_t *dev, uint16_t *Xtalk);

/**
 * @brief This function programs the threshold detection mode\n
 * Example:\n
 * VL53L1X_SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,2,1): Out of window \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,3,1): In window \n
 * @param   dev : device address
 * @param   ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
 * @param   ThreshHigh(in mm) :  the threshold above which one the device raises an interrupt if Window = 1
 * @param   Window detection mode : 0=below, 1=above, 2=out, 3=in
 * @param   IntOnNoTarget = 0 (No longer used - just use 0)
 */
// VL53L1X_ERROR VL53L1X_SetDistanceThreshold(const extDevice_t *dev, uint16_t ThreshLow,
//                   uint16_t ThreshHigh, uint8_t Window,
//                   uint8_t IntOnNoTarget);

/**
 * @brief This function returns the window detection mode (0=below; 1=above; 2=out; 3=in)
 */
// VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(const extDevice_t *dev, uint16_t *window);

/**
 * @brief This function returns the low threshold in mm
 */
// VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(const extDevice_t *dev, uint16_t *low);

/**
 * @brief This function returns the high threshold in mm
 */
// VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(const extDevice_t *dev, uint16_t *high);

/**
 * @brief This function programs the ROI (Region of Interest)\n
 * The ROI position is centered, only the ROI size can be reprogrammed.\n
 * The smallest acceptable ROI size = 4\n
 * @param X:ROI Width; Y=ROI Height
 */
// VL53L1X_ERROR VL53L1X_SetROI(const extDevice_t *dev, uint16_t X, uint16_t Y);

/**
 *@brief This function returns width X and height Y
 */
// VL53L1X_ERROR VL53L1X_GetROI_XY(const extDevice_t *dev, uint16_t *ROI_X, uint16_t *ROI_Y);

/**
 *@brief This function programs the new user ROI center, please to be aware that there is no check in this function.
 *if the ROI center vs ROI size is out of border the ranging function return error #13
 */
// VL53L1X_ERROR VL53L1X_SetROICenter(const extDevice_t *dev, uint8_t ROICenter);

/**
 *@brief This function returns the current user ROI center
 */
// VL53L1X_ERROR VL53L1X_GetROICenter(const extDevice_t *dev, uint8_t *ROICenter);

/**
 * @brief This function programs a new signal threshold in kcps (default=1024 kcps\n
 */
// VL53L1X_ERROR VL53L1X_SetSignalThreshold(const extDevice_t *dev, uint16_t signal);

/**
 * @brief This function returns the current signal threshold in kcps
 */
// VL53L1X_ERROR VL53L1X_GetSignalThreshold(const extDevice_t *dev, uint16_t *signal);

/**
 * @brief This function programs a new sigma threshold in mm (default=15 mm)
 */
// VL53L1X_ERROR VL53L1X_SetSigmaThreshold(const extDevice_t *dev, uint16_t sigma);

/**
 * @brief This function returns the current sigma threshold in mm
 */
// VL53L1X_ERROR VL53L1X_GetSigmaThreshold(const extDevice_t *dev, uint16_t *signal);

/**
 * @brief This function performs the temperature calibration.
 * It is recommended to call this function any time the temperature might have changed by more than 8 deg C
 * without sensor ranging activity for an extended period.
 */
// VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(const extDevice_t *dev);

/**
 * @brief This function performs the offset calibration.\n
 * The function returns the offset value found and programs the offset compensation into the device.
 * @param TargetDistInMm target distance in mm, ST recommended 100 mm
 * Target reflectance = grey17%
 * @return 0:success, !=0: failed
 * @return offset pointer contains the offset found in mm
 */
// int8_t VL53L1X_CalibrateOffset(const extDevice_t *dev, uint16_t TargetDistInMm, int16_t *offset);

/**
 * @brief This function performs the xtalk calibration.\n
 * The function returns the xtalk value found and programs the xtalk compensation to the device
 * @param TargetDistInMm target distance in mm\n
 * The target distance : the distance where the sensor start to "under range"\n
 * due to the influence of the photons reflected back from the cover glass becoming strong\n
 * It's also called inflection point\n
 * Target reflectance = grey 17%
 * @return 0: success, !=0: failed
 * @return xtalk pointer contains the xtalk value found in cps (number of photons in count per second)
 */
// int8_t VL53L1X_CalibrateXtalk(const extDevice_t *dev, uint16_t TargetDistInMm, uint16_t *xtalk);



const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xcc, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

// static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
//     255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
//     255, 255, 11, 12
// };

static uint8_t _I2CBuffer[256];

static int32_t lastMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static bool lastMeasurementIsNew = false;
static bool isInitialized = false;

#define _I2CWrite(dev, data, size) \
    (busWriteRegisterBuffer(dev, 0xFF, data, size) ? 0 : -1)

#define _I2CRead(dev, data, size) \
    (busReadRegisterBuffer(dev, 0xFF, data, size) ? 0 : -1)

VL53L1X_ERROR VL53L1_WriteMulti(const extDevice_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    status_int = _I2CWrite(dev, _I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1X_ERROR VL53L1_ReadMulti(const extDevice_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    status_int = _I2CWrite(dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(dev, pdata, count);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    return Status;
}

VL53L1X_ERROR VL53L1_WrByte(const extDevice_t *dev, uint16_t index, uint8_t data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    status_int = _I2CWrite(dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L1X_ERROR VL53L1_WrWord(const extDevice_t *dev, uint16_t index, uint16_t data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    status_int = _I2CWrite(dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L1X_ERROR VL53L1_WrDWord(const extDevice_t *dev, uint16_t index, uint32_t data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8)  & 0xFF;
    _I2CBuffer[5] = (data >> 0 ) & 0xFF;
    status_int = _I2CWrite(dev, _I2CBuffer, 6);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L1X_ERROR VL53L1_RdByte(const extDevice_t *dev, uint16_t index, uint8_t *data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    status_int = _I2CWrite(dev, _I2CBuffer, 2);
    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(dev, data, 1);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    return Status;
}

VL53L1X_ERROR VL53L1_UpdateByte(const extDevice_t *dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(dev, index, data);
done:
    return Status;
}

VL53L1X_ERROR VL53L1_RdWord(const extDevice_t *dev, uint16_t index, uint16_t *data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    status_int = _I2CWrite(dev, _I2CBuffer, 2);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    return Status;
}

VL53L1X_ERROR VL53L1_RdDWord(const extDevice_t *dev, uint16_t index, uint32_t *data) {
    VL53L1X_ERROR Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    status_int = _I2CWrite(dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    return Status;
}

// VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion)
// {
//     VL53L1X_ERROR Status = 0;

//     pVersion->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
//     pVersion->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
//     pVersion->build = VL53L1X_IMPLEMENTATION_VER_SUB;
//     pVersion->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
//     return Status;
// }

// VL53L1X_ERROR VL53L1X_SetI2CAddress(busDevice_t * dev, uint8_t new_address)
// {
//     VL53L1X_ERROR status = 0;

//     status = VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
//     return status;
// }

VL53L1X_ERROR VL53L1X_SensorInit(const extDevice_t *dev)
{
    VL53L1X_ERROR status = 0;
    uint8_t Addr = 0x00, tmp;

    for (Addr = 0x2D; Addr <= 0x87; Addr++){
        status = VL53L1_WrByte(dev, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
        if (status != VL53L1_ERROR_NONE) {
            return status;
        }
    }
    status = VL53L1X_StartRanging(dev);
    if (status != VL53L1_ERROR_NONE) {
        return status;
    }
    tmp  = 0;
    const timeMs_t deadlineMs = millis() + VL53L1X_BOOT_TIMEOUT_MS;
    while (tmp == 0) {
        status = VL53L1X_CheckForDataReady(dev, &tmp);
        if (status != VL53L1_ERROR_NONE) {
            return status;
        }
        if (cmpTimeMs(millis(), deadlineMs) > 0) {
            return VL53L1_ERROR_TIME_OUT;
        }
    }
    status = VL53L1X_ClearInterrupt(dev);
    status = VL53L1X_StopRanging(dev);
    status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
    status = VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
    return status;
}

static VL53L1X_ERROR VL53L1X_ClearInterrupt(const extDevice_t *dev)
{
    VL53L1X_ERROR status = 0;

    status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    return status;
}

// VL53L1X_ERROR VL53L1X_SetInterruptPolarity(busDevice_t * dev, uint8_t NewPolarity)
// {
//     uint8_t Temp;
//     VL53L1X_ERROR status = 0;

//     status = VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
//     Temp = Temp & 0xEF;
//     status = VL53L1_WrByte(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
//     return status;
// }

static VL53L1X_ERROR VL53L1X_GetInterruptPolarity(const extDevice_t *dev, uint8_t *pInterruptPolarity)
{
    uint8_t Temp;
    VL53L1X_ERROR status = 0;

    status = VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
    Temp = Temp & 0x10;
    *pInterruptPolarity = !(Temp>>4);
    return status;
}

static VL53L1X_ERROR VL53L1X_StartRanging(const extDevice_t *dev)
{
    VL53L1X_ERROR status = 0;

    status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);  /* Enable VL53L1X */
    return status;
}

static VL53L1X_ERROR VL53L1X_StopRanging(const extDevice_t *dev)
{
    VL53L1X_ERROR status = 0;

    status = VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00);  /* Disable VL53L1X */
    return status;
}

static VL53L1X_ERROR VL53L1X_CheckForDataReady(const extDevice_t *dev, uint8_t *isDataReady)
{
    uint8_t Temp;
    uint8_t IntPol;
    VL53L1X_ERROR status = 0;

    status = VL53L1X_GetInterruptPolarity(dev, &IntPol);
    status = VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
    /* Read in the register to check if a new value is available */
    if (status == 0){
        if ((Temp & 1) == IntPol)
            *isDataReady = 1;
        else
            *isDataReady = 0;
    }
    return status;
}

static VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(const extDevice_t *dev, uint16_t TimingBudgetInMs)
{
    uint16_t DM;
    VL53L1X_ERROR  status=0;

    status = VL53L1X_GetDistanceMode(dev, &DM);
    if (DM == 0)
        return 1;
    else if (DM == 1) { /* Short DistanceMode */
        switch (TimingBudgetInMs) {
        case 15: /* only available in short distance mode */
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x01D);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x0027);
            break;
        case 20:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x0051);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x006E);
            break;
        case 33:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x00D6);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x006E);
            break;
        case 50:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x1AE);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x01E8);
            break;
        case 100:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x02E1);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x0388);
            break;
        case 200:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x03E1);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x0496);
            break;
        case 500:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x0591);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x05C1);
            break;
        default:
            status = 1;
            break;
        }
    } else {
        switch (TimingBudgetInMs) {
        case 20:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x001E);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x0022);
            break;
        case 33:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x0060);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x006E);
            break;
        case 50:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x00AD);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x00C6);
            break;
        case 100:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x01CC);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x01EA);
            break;
        case 200:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x02D9);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x02F8);
            break;
        case 500:
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    0x048F);
            VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    0x04A4);
            break;
        default:
            status = 1;
            break;
        }
    }
    return status;
}

static VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(const extDevice_t *dev, uint16_t *pTimingBudget)
{
    uint16_t Temp;
    VL53L1X_ERROR status = 0;

    status = VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
    switch (Temp) {
        case 0x001D :
            *pTimingBudget = 15;
            break;
        case 0x0051 :
        case 0x001E :
            *pTimingBudget = 20;
            break;
        case 0x00D6 :
        case 0x0060 :
            *pTimingBudget = 33;
            break;
        case 0x1AE :
        case 0x00AD :
            *pTimingBudget = 50;
            break;
        case 0x02E1 :
        case 0x01CC :
            *pTimingBudget = 100;
            break;
        case 0x03E1 :
        case 0x02D9 :
            *pTimingBudget = 200;
            break;
        case 0x0591 :
        case 0x048F :
            *pTimingBudget = 500;
            break;
        default:
            status = 1;
            *pTimingBudget = 0;
    }
    return status;
}

static VL53L1X_ERROR VL53L1X_SetDistanceMode(const extDevice_t *dev, uint16_t DM)
{
    uint16_t TB;
    VL53L1X_ERROR status = 0;

    status = VL53L1X_GetTimingBudgetInMs(dev, &TB);
    if (status != 0)
        return 1;
    switch (DM) {
    case 1:
        status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
        status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0705);
        status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
        break;
    case 2:
        status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
        status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
        status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
        break;
    default:
        status = 1;
        break;
    }

    if (status == 0)
        status = VL53L1X_SetTimingBudgetInMs(dev, TB);
    return status;
}

static VL53L1X_ERROR VL53L1X_GetDistanceMode(const extDevice_t *dev, uint16_t *DM)
{
    uint8_t TempDM, status=0;

    status = VL53L1_RdByte(dev,PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
    if (TempDM == 0x14)
        *DM=1;
    if(TempDM == 0x0A)
        *DM=2;
    return status;
}

static VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(const extDevice_t *dev, uint32_t InterMeasMs)
{
    uint16_t ClockPLL;
    VL53L1X_ERROR status = 0;

    status = VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
    ClockPLL = ClockPLL&0x3FF;
    VL53L1_WrDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
            (uint32_t)(ClockPLL * InterMeasMs * 1.075));
    return status;

}

// VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(const extDevice_t *dev, uint16_t *pIM)
// {
//     uint16_t ClockPLL;
//     VL53L1X_ERROR status = 0;
//     uint32_t tmp;

//     status = VL53L1_RdDWord(dev,VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
//     *pIM = (uint16_t)tmp;
//     status = VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
//     ClockPLL = ClockPLL&0x3FF;
//     *pIM= (uint16_t)(*pIM/(ClockPLL*1.065));
//     return status;
// }

// VL53L1X_ERROR VL53L1X_BootState(const extDevice_t *dev, uint8_t *state)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t tmp = 0;

//     status = VL53L1_RdByte(dev,VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
//     *state = tmp;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetSensorId(const extDevice_t *devv, uint16_t *sensorId)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp = 0;

//     status = VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
//     *sensorId = tmp;
//     return status;
// }

static VL53L1X_ERROR VL53L1X_GetDistance(const extDevice_t *dev, uint16_t *distance)
{
    VL53L1X_ERROR status = 0;
    uint16_t tmp;

    status = (VL53L1_RdWord(dev,
            VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
    *distance = tmp;
    return status;
}

// VL53L1X_ERROR VL53L1X_GetSignalPerSpad(const extDevice_t *dev, uint16_t *signalRate)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t SpNb=1, signal;

//     status = VL53L1_RdWord(dev,
//         VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
//     status = VL53L1_RdWord(dev,
//         VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
//     *signalRate = (uint16_t) (200.0*signal/SpNb);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(const extDevice_t *dev, uint16_t *ambPerSp)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t AmbientRate, SpNb = 1;

//     status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
//     status = VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
//     *ambPerSp=(uint16_t) (200.0 * AmbientRate / SpNb);
//     return status;
// }

// static VL53L1X_ERROR VL53L1X_GetSignalRate(const extDevice_t *dev, uint16_t *signal)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,
//         VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
//     *signal = tmp*8;
//     return status;
// }

// static VL53L1X_ERROR VL53L1X_GetSpadNb(const extDevice_t *dev, uint16_t *spNb)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,
//                   VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
//     *spNb = tmp >> 8;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetAmbientRate(const extDevice_t *dev, uint16_t *ambRate)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
//     *ambRate = tmp*8;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetRangeStatus(const extDevice_t *dev, uint8_t *rangeStatus)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t RgSt;

//     *rangeStatus = 255;
//     status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
//     RgSt = RgSt & 0x1F;
//     if (RgSt < 24)
//         *rangeStatus = status_rtn[RgSt];
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetResult(const extDevice_t *dev, VL53L1X_Result_t *pResult)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t Temp[17];
//     uint8_t RgSt = 255;

//     status = VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, Temp, 17);
//     RgSt = Temp[0] & 0x1F;
//     if (RgSt < 24)
//         RgSt = status_rtn[RgSt];
//     pResult->Status = RgSt;
//     pResult->Ambient = (Temp[7] << 8 | Temp[8]) * 8;
//     pResult->NumSPADs = Temp[3];
//     pResult->SigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
//     pResult->Distance = Temp[13] << 8 | Temp[14];

//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetOffset(const extDevice_t *dev, int16_t OffsetValue)
// {
//     VL53L1X_ERROR status = 0;
//     int16_t Temp;

//     Temp = (OffsetValue*4);
//     VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
//             (uint16_t)Temp);
//     VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
//     VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
//     return status;
// }

// VL53L1X_ERROR  VL53L1X_GetOffset(const extDevice_t *dev, int16_t *offset)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t Temp;

//     status = VL53L1_RdWord(dev,ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
//     Temp = Temp<<3;
//     Temp = Temp>>5;
//     *offset = (int16_t)(Temp);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetXtalk(const extDevice_t *dev, uint16_t XtalkValue)
// {
// /* XTalkValue in count per second to avoid float type */
//     VL53L1X_ERROR status = 0;

//     status = VL53L1_WrWord(dev,
//             ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
//             0x0000);
//     status = VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
//             0x0000);
//     status = VL53L1_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
//             (XtalkValue<<9)/1000); /* * << 9 (7.9 format) and /1000 to convert cps to kpcs */
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetXtalk(const extDevice_t *dev, uint16_t *xtalk )
// {
//     VL53L1X_ERROR status = 0;

//     status = VL53L1_RdWord(dev,ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalk);
//     *xtalk = (uint16_t)((*xtalk*1000)>>9); /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetDistanceThreshold(const extDevice_t *dev, uint16_t ThreshLow,
//                   uint16_t ThreshHigh, uint8_t Window,
//                   uint8_t IntOnNoTarget)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t Temp = 0;

//     status = VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
//     Temp = Temp & 0x47;
//     if (IntOnNoTarget == 0) {
//         status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO,
//                    (Temp | (Window & 0x07)));
//     } else {
//         status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO,
//                    ((Temp | (Window & 0x07)) | 0x40));
//     }
//     status = VL53L1_WrWord(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
//     status = VL53L1_WrWord(dev, SYSTEM__THRESH_LOW, ThreshLow);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(const extDevice_t *dev, uint16_t *window)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t tmp;
//     status = VL53L1_RdByte(dev,SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
//     *window = (uint16_t)(tmp & 0x7);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(const extDevice_t *dev, uint16_t *low)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,SYSTEM__THRESH_LOW, &tmp);
//     *low = tmp;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(const extDevice_t *dev, uint16_t *high)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,SYSTEM__THRESH_HIGH, &tmp);
//     *high = tmp;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetROICenter(const extDevice_t *dev, uint8_t ROICenter)
// {
//     VL53L1X_ERROR status = 0;
//     status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetROICenter(const extDevice_t *dev, uint8_t *ROICenter)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t tmp;
//     status = VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, &tmp);
//     *ROICenter = tmp;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetROI(const extDevice_t *dev, uint16_t X, uint16_t Y)
// {
//     uint8_t OpticalCenter;
//     VL53L1X_ERROR status = 0;

//     status =VL53L1_RdByte(dev, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
//     if (X > 16)
//         X = 16;
//     if (Y > 16)
//         Y = 16;
//     if (X > 10 || Y > 10){
//         OpticalCenter = 199;
//     }
//     status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
//     status = VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
//                (Y - 1) << 4 | (X - 1));
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetROI_XY(const extDevice_t *dev, uint16_t *ROI_X, uint16_t *ROI_Y)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t tmp;

//     status = VL53L1_RdByte(dev,ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
//     *ROI_X = ((uint16_t)tmp & 0x0F) + 1;
//     *ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetSignalThreshold(const extDevice_t *dev, uint16_t Signal)
// {
//     VL53L1X_ERROR status = 0;

//     VL53L1_WrWord(dev,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,Signal>>3);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetSignalThreshold(const extDevice_t *dev, uint16_t *signal)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,
//                 RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
//     *signal = tmp <<3;
//     return status;
// }

// VL53L1X_ERROR VL53L1X_SetSigmaThreshold(const extDevice_t *dev, uint16_t Sigma)
// {
//     VL53L1X_ERROR status = 0;

//     if(Sigma>(0xFFFF>>2)){
//         return 1;
//     }
//     /* 16 bits register 14.2 format */
//     status = VL53L1_WrWord(dev,RANGE_CONFIG__SIGMA_THRESH,Sigma<<2);
//     return status;
// }

// VL53L1X_ERROR VL53L1X_GetSigmaThreshold(const extDevice_t *dev, uint16_t *sigma)
// {
//     VL53L1X_ERROR status = 0;
//     uint16_t tmp;

//     status = VL53L1_RdWord(dev,RANGE_CONFIG__SIGMA_THRESH, &tmp);
//     *sigma = tmp >> 2;
//     return status;

// }

// VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(const extDevice_t *dev)
// {
//     VL53L1X_ERROR status = 0;
//     uint8_t tmp=0;

//     status = VL53L1_WrByte(dev,VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x81); /* full VHV */
//     status = VL53L1_WrByte(dev,0x0B,0x92);
//     status = VL53L1X_StartRanging(dev);
//     while(tmp==0){
//         status = VL53L1X_CheckForDataReady(dev, &tmp);
//     }
//     tmp  = 0;
//     status = VL53L1X_ClearInterrupt(dev);
//     status = VL53L1X_StopRanging(dev);
//     status = VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
//     status = VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
//     return status;
// }


// int8_t VL53L1X_CalibrateOffset(const extDevice_t *dev, uint16_t TargetDistInMm, int16_t *offset)
// {
//     uint8_t i, tmp;
//     int16_t AverageDistance = 0;
//     uint16_t distance;
//     VL53L1X_ERROR status = 0;

//     status = VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
//     status = VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
//     status = VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
//     status = VL53L1X_StartRanging(dev); /* Enable VL53L1X sensor */
//     for (i = 0; i < 50; i++) {
//         tmp = 0;
//         while (tmp == 0){
//             status = VL53L1X_CheckForDataReady(dev, &tmp);
//         }
//         status = VL53L1X_GetDistance(dev, &distance);
//         status = VL53L1X_ClearInterrupt(dev);
//         AverageDistance = AverageDistance + distance;
//     }
//     status = VL53L1X_StopRanging(dev);
//     AverageDistance = AverageDistance / 50;
//     *offset = TargetDistInMm - AverageDistance;
//     status = VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);
//     return status;
// }

// int8_t VL53L1X_CalibrateXtalk(const extDevice_t *dev, uint16_t TargetDistInMm, uint16_t *xtalk)
// {
//     uint8_t i, tmp;
//     float AverageSignalRate = 0;
//     float AverageDistance = 0;
//     float AverageSpadNb = 0;
//     uint16_t distance = 0, spadNum;
//     uint16_t sr;
//     VL53L1X_ERROR status = 0;
//     uint32_t calXtalk;

//     status = VL53L1_WrWord(dev, 0x0016,0);
//     status = VL53L1X_StartRanging(dev);
//     for (i = 0; i < 50; i++) {
//         tmp = 0;
//         while (tmp == 0){
//             status = VL53L1X_CheckForDataReady(dev, &tmp);
//         }
//         status= VL53L1X_GetSignalRate(dev, &sr);
//         status= VL53L1X_GetDistance(dev, &distance);
//         status = VL53L1X_ClearInterrupt(dev);
//         AverageDistance = AverageDistance + distance;
//         status = VL53L1X_GetSpadNb(dev, &spadNum);
//         AverageSpadNb = AverageSpadNb + spadNum;
//         AverageSignalRate =
//             AverageSignalRate + sr;
//     }
//     status = VL53L1X_StopRanging(dev);
//     AverageDistance = AverageDistance / 50;
//     AverageSpadNb = AverageSpadNb / 50;
//     AverageSignalRate = AverageSignalRate / 50;
//     /* Calculate Xtalk value */
//     calXtalk = (uint16_t)(512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb);
//     *xtalk = (uint16_t)((calXtalk*1000)>>9);
//     status = VL53L1_WrWord(dev, 0x0016, (uint16_t)calXtalk);
//     return status;
// }

static void vl53l1x_Init(rangefinderDev_t * rangefinder)
{
    VL53L1X_ERROR status = VL53L1_ERROR_NONE;
    isInitialized = false;
    status = VL53L1X_SensorInit(&rangefinder->dev);
    if (status == VL53L1_ERROR_NONE) {
        VL53L1X_SetDistanceMode(&rangefinder->dev, 2); /* 1=short, 2=long */
        VL53L1X_SetTimingBudgetInMs(&rangefinder->dev, 33); /* in ms possible values [20, 50, 100, 200, 500] */
        VL53L1X_SetInterMeasurementInMs(&rangefinder->dev, RANGEFINDER_VL53L1X_TASK_PERIOD_MS); /* in ms, IM must be > = TB */
        status = VL53L1X_StartRanging(&rangefinder->dev);
    }
    isInitialized = (status == VL53L1_ERROR_NONE);
}

void vl53l1x_Update(rangefinderDev_t * rangefinder)
{
    uint16_t Distance;
    uint8_t dataReady = 0;

    if (!isInitialized) {
        return;
    }

    VL53L1X_CheckForDataReady(&rangefinder->dev, &dataReady);
    if (dataReady != 0) {
        if (VL53L1X_GetDistance(&rangefinder->dev, &Distance) == VL53L1_ERROR_NONE) {
            lastMeasurementCm = Distance / 10;
            lastMeasurementIsNew = true;
        }
    }

    VL53L1X_ClearInterrupt(&rangefinder->dev);
}

int32_t vl53l1x_GetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (isInitialized) {
        if (lastMeasurementIsNew) {
            lastMeasurementIsNew = false;
            return (lastMeasurementCm < VL53L1X_MAX_RANGE_CM) ? lastMeasurementCm : RANGEFINDER_OUT_OF_RANGE;
        }
        else {
            return RANGEFINDER_NO_NEW_DATA;
        }
    }
    else {
        return RANGEFINDER_HARDWARE_FAILURE;
    }
}

static bool deviceDetect(const extDevice_t *dev)
{
    for (int retry = 0; retry < 5; retry++) {
        uint8_t model_id;

        delay(150);

        VL53L1X_ERROR err = VL53L1_RdByte(dev, 0x010F, &model_id);
        if (err == 0 && model_id == 0xEA) {
            return true;
        }
    };

    return false;
}

bool vl53l1xDetect(rangefinderDev_t * rangefinder)
{
    extDevice_t *dev = &rangefinder->dev;
    bool defaultAddressApplied = false;

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for VL53L1X
        dev->busType_u.i2c.address = VL53L1X_I2C_ADDR;
        defaultAddressApplied = true;
    }

    if (!deviceDetect(dev)) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    rangefinder->delayMs = RANGEFINDER_VL53L1X_TASK_PERIOD_MS;
    rangefinder->maxRangeCm = VL53L1X_MAX_RANGE_CM;
    rangefinder->detectionConeDeciDegrees = VL53L1X_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = VL53L1X_DETECTION_CONE_DECIDEGREES;

    rangefinder->init = &vl53l1x_Init;
    rangefinder->update = &vl53l1x_Update;
    rangefinder->read = &vl53l1x_GetDistance;

    return true;
}

#endif
