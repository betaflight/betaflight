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

#pragma once

#if defined(INVERTER_PIN_USART1) || defined(INVERTER_PIN_USART2) || defined(INVERTER_PIN_USART3) || defined(INVERTER_PIN_USART4) || defined(INVERTER_PIN_USART5) || defined(INVERTER_PIN_USART6)
#define USE_INVERTER
#endif

void initInverters(void);

void enableInverter(USART_TypeDef *USARTx, bool on);
