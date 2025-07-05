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

static void rcu_ahb1_periph_clk_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB1EN |= mask;
    } else {
        RCU_AHB1EN &= ~mask;
    }
}

static void rcu_ahb2_periph_clk_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB2EN |= mask;
    } else {
        RCU_AHB2EN &= ~mask;
    }
}

static void rcu_ahb3_periph_clk_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB3EN |= mask;
    } else {
        RCU_AHB3EN &= ~mask;
    }
}

static void rcu_apb1_periph_clk_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_APB1EN |= mask;
    } else {
        RCU_APB1EN &= ~mask;
    }
}

static void rcu_apb2_periph_clk_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_APB2EN |= mask;
    } else {
        RCU_APB2EN &= ~mask;
    }
}

static void rcu_ahb1_periph_rst_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB1RST |= mask;
    } else {
        RCU_AHB1RST &= ~mask;
    }
}

static void rcu_ahb2_periph_rst_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB2RST |= mask;
    } else {
        RCU_AHB2RST &= ~mask;
    }
}

static void rcu_ahb3_periph_rst_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_AHB3RST |= mask;
    } else {
        RCU_AHB3RST &= ~mask;
    }
}

static void rcu_apb1_periph_rst_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_APB1RST |= mask;
    } else {
        RCU_APB1RST &= ~mask;
    }
}

static void rcu_apb2_periph_rst_config(uint32_t mask, FunctionalState NewState) 
{
    if (NewState == ENABLE) {
        RCU_APB2RST |= mask;
    } else {
        RCU_APB2RST &= ~mask;
    }
}


void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
    case RCC_AHB1:
        rcu_ahb1_periph_clk_config(mask, NewState);
        break;
    case RCC_AHB2:
        rcu_ahb2_periph_clk_config(mask, NewState);
        break;
    case RCC_AHB3:
        rcu_ahb3_periph_clk_config(mask, NewState);
        break;
    case RCC_APB1:
        rcu_apb1_periph_clk_config(mask, NewState);
        break;
    case RCC_APB2:
        rcu_apb2_periph_clk_config(mask, NewState);
        break;
    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
    case RCC_AHB1:
        rcu_ahb1_periph_rst_config(mask, NewState);
        break;
    case RCC_AHB2:
        rcu_ahb2_periph_rst_config(mask, NewState);
        break;
    case RCC_AHB3:
        rcu_ahb3_periph_rst_config(mask, NewState);
        break;
    case RCC_APB2:
        rcu_apb1_periph_rst_config(mask, NewState);
        break;
    case RCC_APB1:
        rcu_apb2_periph_rst_config(mask, NewState);
        break;
    }
}
