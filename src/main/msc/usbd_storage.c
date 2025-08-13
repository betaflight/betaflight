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

/*
 * Author: jflyper (https://github.com/jflyper)
 */

#include "platform.h"
#include "common/time.h"

#if !(defined(PICO))
#ifdef USE_HAL_DRIVER
#include "usbd_msc.h"
#else
#include "usbd_msc_mem.h"
#ifndef AT32F435
#include "usbd_msc_core.h"
#endif
#endif
#endif

#include "usbd_storage.h"

#if !(defined(PICO))
#ifdef USE_HAL_DRIVER
USBD_StorageTypeDef *USBD_STORAGE_fops;
#else
USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;
#endif
#endif
