/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// IO routines from usb_msc_at32F435.c
int8_t msc_disk_capacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size);
int8_t msc_disk_read(uint8_t lun, uint32_t blk_addr, uint8_t *buf, uint16_t blk_len);
int8_t msc_disk_write(uint8_t lun, uint32_t blk_addr, uint8_t *buf, uint16_t blk_len);
uint8_t *get_inquiry(uint8_t lun);
uint8_t msc_get_readonly(uint8_t lun);

