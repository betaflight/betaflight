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

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/serial_resource.h"

#define UARTDEV_CONFIG_MAX (RESOURCE_UART_COUNT + RESOURCE_LPUART_COUNT)

typedef struct serialUartConfig_s {
    int8_t txDmaopt;
    int8_t rxDmaopt;
} serialUartConfig_t;

PG_DECLARE_ARRAY(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig);
