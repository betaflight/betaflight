/*
 * @Author:  
 * @Date: 2026-03-31 15:48:58
 * @LastEditors:  
 * @LastEditTime: 2026-04-16 15:20:21
 * @FilePath: \betaflight\src\main\drivers\accgyro\accgyro_spi_scs3302.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#pragma once

#include <stdbool.h>
//#include <stdint.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"

// Discovery functions
uint8_t scs3302SpiDetect(const extDevice_t *dev);
bool scs3302SpiAccDetect(accDev_t *acc);
bool scs3302SpiGyroDetect(gyroDev_t *gyro);
