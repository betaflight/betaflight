/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */
#ifdef __RTTHREAD__
#include <tusb.h>
#include <stdint.h>

#include <board.h>
#include <rtdevice.h>

static bool ejected = false;
static rt_device_t flash_device;
static struct rt_device_blk_geometry blk_geom;

#ifdef __CC_ARM
uint16_t __builtin_bswap16(uint16_t x)
{
    return (x << 8) | (x >> 8);
}
#endif

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;

    const char vid[] = PKG_TINYUSB_DEVICE_MSC_VID;
    const char pid[] = PKG_TINYUSB_DEVICE_MSC_PID;
    const char rev[] = PKG_TINYUSB_DEVICE_MSC_REV;

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void) lun;

    if (ejected)
    {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
        return false;
    }

    if (flash_device == NULL)
    {
        flash_device = rt_device_find(PKG_TINYUSB_DEVICE_MSC_NAME);
    }
    if (flash_device != NULL)
    {
        static uint8_t open_flg = 0;
        if (!open_flg)
        {
            open_flg = 1;
            rt_device_open(flash_device, 0);
        }

        rt_device_control(flash_device, RT_DEVICE_CTRL_BLK_GETGEOME, &blk_geom);
        return true;
    }

    return false;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void) lun;

    *block_count = blk_geom.sector_count;
    *block_size  = blk_geom.bytes_per_sector;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void) lun;
    (void) power_condition;

    if (load_eject)
    {
        if (start)
        {
            ejected = false;
        } else {
            // unload disk storage
            ejected = true;
        }
    }

    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;
    (void) bufsize;

    return (int32_t) rt_device_read(flash_device, (rt_off_t) lba, buffer, 1) * blk_geom.bytes_per_sector;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;
    (void) bufsize;

    return (int32_t) rt_device_write(flash_device, (rt_off_t) lba, buffer, 1) * blk_geom.bytes_per_sector;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    void const *response = NULL;
    uint16_t resplen = 0;

    // most scsi handled is input
    bool in_xfer = true;

    switch (scsi_cmd[0])
    {
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
            // Host is about to read/write etc ... better not to disconnect disk
            resplen = 0;
            break;

        default:
            // Set Sense = Invalid Command Operation
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

            // negative means error -> tinyusb could stall and/or response with failed status
            resplen = -1;
            break;
    }

    // return resplen must not larger than bufsize
    if (resplen > bufsize) resplen = bufsize;

    if (response && (resplen > 0))
    {
        if (in_xfer)
        {
            memcpy(buffer, response, resplen);
        } else {
            // SCSI output
        }
    }

    return resplen;
}
#endif /*__RTTHREAD__*/
