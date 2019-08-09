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

#define SDIO_CFG_TO_DEV(x) ((x) - 1)
#define SDIO_DEV_TO_CFG(x) ((x) + 1)

typedef enum {
    SDIOINVALID = -1,
    SDIODEV_1 = 0,
    SDIODEV_2,
} SDIODevice;

#define SDIODEV_COUNT 2

#if defined(STM32H7)
void sdioPinConfigure();
void SDIO_GPIO_Init(void);
#endif
