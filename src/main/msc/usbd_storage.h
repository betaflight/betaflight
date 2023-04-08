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
 *
 * Author: Chris Hockuba (https://github.com/conkerkh)
 */

#pragma once

#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
#else
#include "usbd_msc_mem.h"
#ifndef AT32F435
#include "usbd_msc_core.h"
#endif
#endif

#include "common/time.h"

#ifdef USE_HAL_DRIVER
extern USBD_StorageTypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD_SDIO
extern USBD_StorageTypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_SDCARD_SPI
extern USBD_StorageTypeDef USBD_MSC_MICRO_SD_SPI_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_StorageTypeDef USBD_MSC_EMFAT_fops;
#endif
#else // USE_HAL_DRIVER
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD_SDIO
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_SDCARD_SPI
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SD_SPI_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_STORAGE_cb_TypeDef USBD_MSC_EMFAT_fops;
#endif
#endif // USE_HAL_DRIVER
