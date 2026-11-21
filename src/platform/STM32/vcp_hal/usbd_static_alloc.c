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

/*
 * Static replacement for USBD_malloc/USBD_free (see usbd_conf.h).
 *
 * The ST USB device classes allocate their handle in the class Init
 * callback and free it in DeInit. Using the C library heap for this pulls
 * malloc/free into the image and puts USB state in an unbounded heap.
 * Instead, each class handle gets a fixed slot sized for it.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "usbd_conf.h"
#include "usbd_cdc.h"
#ifdef USE_USB_CDC_HID
#include "usbd_hid.h"
#endif

#define USBD_WORDS(type) ((sizeof(type) + sizeof(uint32_t) - 1) / sizeof(uint32_t))

static uint32_t cdcHandle[USBD_WORDS(USBD_CDC_HandleTypeDef)];
#ifdef USE_USB_CDC_HID
static uint32_t hidHandle[USBD_WORDS(USBD_HID_HandleTypeDef)];
#endif

typedef struct usbdStaticSlot_s {
    uint32_t *mem;
    size_t size;
    bool used;
} usbdStaticSlot_t;

static usbdStaticSlot_t slots[] = {
#ifdef USE_USB_CDC_HID
    // Smallest first, so a request takes the tightest slot that fits
    { hidHandle, sizeof(hidHandle), false },
#endif
    { cdcHandle, sizeof(cdcHandle), false },
};

void *USBD_static_malloc(uint32_t size)
{
    for (unsigned i = 0; i < ARRAYLEN(slots); i++) {
        if (!slots[i].used && size <= slots[i].size) {
            slots[i].used = true;
            return slots[i].mem;
        }
    }
    // Class Init handles NULL and reports a memory error
    return NULL;
}

void USBD_static_free(void *p)
{
    // Pointers not from a slot (e.g. the MSC class's static handle) are ignored
    for (unsigned i = 0; i < ARRAYLEN(slots); i++) {
        if (slots[i].mem == p) {
            slots[i].used = false;
            return;
        }
    }
}
