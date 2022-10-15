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


#include <stdbool.h>

#include "platform.h"

#ifdef USE_USB_CDC_HID

#include "common/maths.h"

#include "fc/rc_controls.h"

#include "rx/rx.h"

#include "pg/usb.h"

#include "sensors/battery.h"

//TODO: Make it platform independent in the future
#if defined(STM32F4)
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#include "drivers/serial_usb_vcp.h"
#include "usbd_hid.h"
#include "vcp_hal/usbd_cdc_interface.h"
#endif

#define USB_CDC_HID_NUM_AXES 8
#define USB_CDC_HID_NUM_BUTTONS 8

#define USB_CDC_HID_RANGE_MIN -127
#define USB_CDC_HID_RANGE_MAX 127

// In the windows joystick driver, the axes are defined as shown in the second column.

const uint8_t hidChannelMapping[] = {
    ROLL,     // X
    PITCH,    // Y
    AUX3,     // Z
    YAW,      // X Rotation
    AUX1,     // Z Rotation
    THROTTLE, // Y Rotation
    AUX4,     // Slider
    AUX2,     // Dial
    AUX5,     // Button 1
    AUX6,     // Button 2
    AUX7,     // Button 3
    AUX8,     // Button 4
    AUX9,     // Button 5
    AUX10,    // Button 6
    AUX11,    // Button 7
    AUX12     // Button 8
};

void sendRcDataToHid(void)
{
    int8_t report[9];
     // Axes
    for (unsigned i = 0; i < USB_CDC_HID_NUM_AXES; i++) {
        const uint8_t channel = hidChannelMapping[i];
        report[i] = scaleRange(constrain(rcData[channel], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, USB_CDC_HID_RANGE_MIN, USB_CDC_HID_RANGE_MAX);
        if (channel == PITCH) {
            // PITCH is inverted in Windows
            report[i] = -report[i];
        }
    }

    // Buttons
    // Each bit in one byte represents one button so we have 8 buttons in one-byte-data
    report[8] = 0;
    for (unsigned i = 0; i < USB_CDC_HID_NUM_BUTTONS; i++) {
        const uint8_t channel = hidChannelMapping[i + USB_CDC_HID_NUM_AXES];
        if (scaleRange(constrain(rcData[channel], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, USB_CDC_HID_RANGE_MIN, USB_CDC_HID_RANGE_MAX) > 0) {
            report[8] |= (1 << i);
        }
    }
#if defined(STM32F4)
    USBD_HID_SendReport(&USB_OTG_dev, (uint8_t*)report, sizeof(report));
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    USBD_HID_SendReport(&USBD_Device, (uint8_t*)report, sizeof(report));
#else
# error "MCU does not support USB HID."
#endif
}

bool cdcDeviceIsMayBeActive(void)
{
    return usbDevConfig()->type == COMPOSITE && usbIsConnected() && (getBatteryState() == BATTERY_NOT_PRESENT || batteryConfig()->voltageMeterSource == VOLTAGE_METER_NONE);
}
#endif
