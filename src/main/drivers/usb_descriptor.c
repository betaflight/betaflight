/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/version.h"
#include "common/printf.h"
#include "fc/board_info.h"
#include "pg/board.h"

#include "usb_descriptor.h"

#define USB_DESCRIPTOR_MSC_SUFFIX   " MSC"

#define USB_DESCRIPTOR_PRODUCT_SIZE (sizeof(FC_FIRMWARE_NAME) + MAX_BOARD_NAME_LENGTH \
                                     + sizeof(" ()" USB_DESCRIPTOR_MSC_SUFFIX))

static const char *usbDescriptorName(void)
{
#if defined(USE_BOARD_INFO)
    const char *boardName = getBoardName();
    if (strlen(boardName) > 0) {
        return boardName;
    }
#endif

    return targetName;
}

static const char *usbDescriptorCompose(char *productString, const char *suffix)
{
    if (strlen(productString) == 0) {
        char name[MAX_BOARD_NAME_LENGTH + 1];
        strncpy(name, usbDescriptorName(), MAX_BOARD_NAME_LENGTH);
        name[MAX_BOARD_NAME_LENGTH] = '\0';

        tfp_sprintf(productString, "%s (%s)%s", FC_FIRMWARE_NAME, name, suffix);
    }

    return productString;
}

const char *usbDescriptorProductString(void)
{
    static char productString[USB_DESCRIPTOR_PRODUCT_SIZE];

    return usbDescriptorCompose(productString, "");
}

const char *usbDescriptorMscProductString(void)
{
    static char mscProductString[USB_DESCRIPTOR_PRODUCT_SIZE];

    return usbDescriptorCompose(mscProductString, USB_DESCRIPTOR_MSC_SUFFIX);
}
