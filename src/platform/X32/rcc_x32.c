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

#include "platform.h"
#include "platform/rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
    case RCC_AHB1_1:
        RCC_EnableAHB1PeriphClk1(mask, NewState);
        break;
    case RCC_AHB1_2:
        RCC_EnableAHB1PeriphClk2(mask, NewState);
        break;
    case RCC_AHB1_3:
        RCC_EnableAHB1PeriphClk3(mask, NewState);
        break;
    case RCC_AHB1_4:
        RCC_EnableAHB1PeriphClk4(mask, NewState);
        break;
    case RCC_AHB2_1:
        RCC_EnableAHB2PeriphClk1(mask, NewState);
        break;
    case RCC_AHB2_2:
        RCC_EnableAHB2PeriphClk2(mask, NewState);
        break;
    case RCC_AHB5_1:
        RCC_EnableAHB5PeriphClk1( mask, NewState);
        break;
    case RCC_AHB5_2:
        RCC_EnableAHB5PeriphClk2(mask, NewState);
        break;
    case RCC_AHB9_1:
        RCC_EnableAHB9PeriphClk1(mask, NewState);
        break;
    case RCC_APB1_1:
        RCC_EnableAPB1PeriphClk1(mask, NewState);
        break;
    case RCC_APB1_2:
        RCC_EnableAPB1PeriphClk2(mask, NewState);
        break;
    case RCC_APB1_3:
        RCC_EnableAPB1PeriphClk3(mask, NewState);
        break;
    case RCC_APB1_4:
        RCC_EnableAPB1PeriphClk4(mask, NewState);
        break;
    case RCC_APB1_5:
        RCC_EnableAPB1PeriphClk5(mask, NewState);
        break;
    case RCC_APB2_1:
        RCC_EnableAPB2PeriphClk1(mask, NewState);
        break;
    case RCC_APB2_2:
        RCC_EnableAPB2PeriphClk2(mask, NewState);
        break;
    case RCC_APB2_3:
        RCC_EnableAPB2PeriphClk3(mask, NewState);
        break;
    case RCC_APB2_4:
        RCC_EnableAPB2PeriphClk4(mask, NewState);
        break;
    case RCC_APB5_1:
        RCC_EnableAPB5PeriphClk1(mask, NewState);
        break;
    case RCC_APB5_2:
        RCC_EnableAPB5PeriphClk2(mask, NewState);
        break;
    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    if (NewState != ENABLE) {
        return;
    }

    switch (tag) {
    case RCC_AHB1_1:
        RCC_EnableAHB1PeriphReset1(mask);
        break;
    case RCC_AHB1_2:
        RCC_EnableAHB1PeriphReset2(mask);
        break;
    case RCC_AHB1_3:
        RCC_EnableAHB1PeriphReset3(mask);
        break;
    case RCC_AHB1_4:
        RCC_EnableAHB1PeriphReset4(mask);
        break;
    case RCC_AHB2_1:
        RCC_EnableAHB2PeriphReset1(mask);
        break;
    case RCC_AHB2_2:
        RCC_EnableAHB2PeriphReset1(mask);
        break;
    case RCC_AHB5_1:
        RCC_EnableAHB5PeriphReset1(mask);
        break;
    case RCC_AHB5_2:
        RCC_EnableAHB5PeriphReset2(mask);
        break;
    case RCC_APB1_1:
        RCC_EnableAPB1PeriphReset1(mask);
        break;
    case RCC_APB1_2:
        RCC_EnableAPB1PeriphReset2(mask);
        break;
    case RCC_APB1_3:
        RCC_EnableAPB1PeriphReset3(mask);
        break;
    case RCC_APB1_4:
        RCC_EnableAPB1PeriphReset4(mask);
        break;
    case RCC_APB1_5:
        RCC_EnableAPB1PeriphReset5(mask);
        break;
    case RCC_APB2_1:
        RCC_EnableAPB2PeriphReset1(mask);
        break;
    case RCC_APB2_2:
        RCC_EnableAPB2PeriphReset2(mask);
        break;
    case RCC_APB2_3:
        RCC_EnableAPB2PeriphReset3(mask);
        break;
    case RCC_APB2_4:
        RCC_EnableAPB2PeriphReset4(mask);
        break;
    case RCC_APB5_1:
        RCC_EnableAPB5PeriphReset1(mask);
        break;
    case RCC_APB5_2:
        RCC_EnableAPB5PeriphReset2(mask);
        break;
    }
}
