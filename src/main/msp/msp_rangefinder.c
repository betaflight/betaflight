/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_MSP
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidarmt.h"
#include "sensors/rangefinder.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_rangefinder.h"

static uint8_t rangefinderMSP_id = 0;

void setRangefinderMSP(uint8_t rangefinder_id)
{
    rangefinderMSP_id = rangefinder_id;
}

void mspRangefinderReceiveNewData(uint8_t * bufferPtr)
{   
    switch (rangefinderMSP_id)
    {
        case RANGEFINDER_MTF01:
        case RANGEFINDER_MTF02:
        case RANGEFINDER_MTF01P:
        case RANGEFINDER_MTF02P:
        case RANGEFINDER_MT01P:
            mtRangefinderReceiveNewData(bufferPtr);
            break;
        
        default:
            break;
    }
}
#endif