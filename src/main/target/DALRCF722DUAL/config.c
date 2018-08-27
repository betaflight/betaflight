/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h" 


#ifdef TARGET_VALIDATECONFIG

#include "fc/config.h"

#include "sensors/gyro.h"

void targetValidateConfiguration(void)
{
	if(gyroConfigMutable()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) {
		if((gyroConfigMutable()->gyroMovementCalibrationThreshold ==48) && (gyroConfigMutable()->gyroMovementCalibrationThreshold !=148)) {
            gyroConfigMutable()->gyroMovementCalibrationThreshold = 148; //for cli set
		}
    } else {
		if((gyroConfigMutable()->gyroMovementCalibrationThreshold ==148) && (gyroConfigMutable()->gyroMovementCalibrationThreshold !=48)) {
			gyroConfigMutable()->gyroMovementCalibrationThreshold = 48; //for cli set
		}
    }
}

#endif
