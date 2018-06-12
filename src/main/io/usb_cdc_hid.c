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

#include "platform.h"

#ifdef USE_USB_CDC_HID

#include "common/maths.h"

#include "rx/rx.h"

//TODO: Make it platform independent in the future
#ifdef STM32F4
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
#elif defined(STM32F7)
#include "usbd_cdc_interface.h"
#include "usbd_hid.h"
#endif

#define USB_CDC_HID_NUM_AXES 8

// In the windows joystick driver, the axes are defined as shown in the third column.

const uint8_t hidChannelMapping[] = {
    0, // X,          ROLL
    1, // Y,          PITCH
    6, // ,           AUX3
    3, // Y Rotation, THROTTLE
    4, // Z Rotation, AUX1
    2, // X Rotation, YAW
    7, // ,           AUX4
    5, // Wheel,      AUX2
};

void sendRcDataToHid(void)
{
    int8_t report[8];
    for (unsigned i = 0; i < USB_CDC_HID_NUM_AXES; i++) {
        const uint8_t channel = getMappedChannel(hidChannelMapping[i]);
            report[i] = scaleRange(constrain(rcData[channel], 1000, 2000), 1000, 2000, -127, 127);
            if (i == 1 || i == 2) {
                report[i] = -report[i];
            }
    }
#ifdef STM32F4
    USBD_HID_SendReport(&USB_OTG_dev, (uint8_t*)report, sizeof(report));
#elif defined(STM32F7)
    extern USBD_HandleTypeDef  USBD_Device;
    USBD_HID_SendReport(&USBD_Device, (uint8_t*)report, sizeof(report));
#endif
}
#endif
