/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 */

#pragma once

#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
#else
#include "usbd_msc_mem.h"
#include "usbd_msc_core.h"
#endif

#ifdef USE_HAL_DRIVER
extern USBD_StorageTypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD
extern USBD_StorageTypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_StorageTypeDef USBD_MSC_EMFAT_fops;
#endif
#else
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
#ifdef USE_SDCARD
extern USBD_STORAGE_cb_TypeDef USBD_MSC_MICRO_SDIO_fops;
#endif
#ifdef USE_FLASHFS
extern USBD_STORAGE_cb_TypeDef USBD_MSC_EMFAT_fops;
#endif
#endif
