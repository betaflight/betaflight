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

#pragma once

struct accDev_s;
extern struct accDev_s *virtualAccDev;
bool virtualAccDetect(struct accDev_s *acc);
void virtualAccSet(struct accDev_s *acc, int16_t x, int16_t y, int16_t z);

struct gyroDev_s;
extern struct gyroDev_s *virtualGyroDev;
bool virtualGyroDetect(struct gyroDev_s *gyro);
void virtualGyroSet(struct gyroDev_s *gyro, int16_t x, int16_t y, int16_t z);
