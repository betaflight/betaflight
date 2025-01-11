/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "syscfg/syscfg.h"
#include "mcu/da1469x_hal.h"
#include "mcu/da1469x_clock.h"

static inline bool
da1469x_clock_is_xtal32m_settled(void)
{
    return ((*(uint32_t *)0x5001001c & 0xff00) == 0) &&
           ((*(uint32_t *)0x50010054 & 0x000f) != 0xb);
}

void
da1469x_clock_sys_xtal32m_init(void)
{
    uint32_t reg;
    int xtalrdy_cnt;

    /* Number of lp_clk cycles (~30.5us) */
    xtalrdy_cnt = MYNEWT_VAL(MCU_CLOCK_XTAL32M_SETTLE_TIME_US) * 10 / 305;

    reg = CRG_XTAL->XTALRDY_CTRL_REG;
    reg &= ~(CRG_XTAL_XTALRDY_CTRL_REG_XTALRDY_CLK_SEL_Msk |
             CRG_XTAL_XTALRDY_CTRL_REG_XTALRDY_CNT_Msk);
    reg |= xtalrdy_cnt;
    CRG_XTAL->XTALRDY_CTRL_REG = reg;
}

void
da1469x_clock_sys_xtal32m_enable(void)
{
  PDC->PDC_CTRL0_REG = (2 << PDC_PDC_CTRL0_REG_TRIG_SELECT_Pos) |
                       (15 << PDC_PDC_CTRL0_REG_TRIG_ID_Pos) |
                       (1 << PDC_PDC_CTRL0_REG_PDC_MASTER_Pos) |
                       (1 << PDC_PDC_CTRL0_REG_EN_XTAL_Pos);

  PDC->PDC_SET_PENDING_REG = 0;
  PDC->PDC_ACKNOWLEDGE_REG = 0;
}

void
da1469x_clock_sys_xtal32m_switch(void)
{
    if (CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_RC32M_Msk) {
        CRG_TOP->CLK_SWITCH2XTAL_REG = CRG_TOP_CLK_SWITCH2XTAL_REG_SWITCH2XTAL_Msk;
    } else {
        CRG_TOP->CLK_CTRL_REG &= ~CRG_TOP_CLK_CTRL_REG_SYS_CLK_SEL_Msk;
    }

    while (!(CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_XTAL32M_Msk));
}

void
da1469x_clock_sys_xtal32m_wait_to_settle(void)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    NVIC_ClearPendingIRQ(XTAL32M_RDY_IRQn);

    if (!da1469x_clock_is_xtal32m_settled()) {
        NVIC_EnableIRQ(XTAL32M_RDY_IRQn);
        while (!NVIC_GetPendingIRQ(XTAL32M_RDY_IRQn)) {
            __WFI();
        }
        NVIC_DisableIRQ(XTAL32M_RDY_IRQn);
    }

    __HAL_ENABLE_INTERRUPTS(primask);
}

void
da1469x_clock_sys_xtal32m_switch_safe(void)
{
    da1469x_clock_sys_xtal32m_wait_to_settle();

    da1469x_clock_sys_xtal32m_switch();
}

void
da1469x_clock_sys_rc32m_disable(void)
{
    CRG_TOP->CLK_RC32M_REG &= ~CRG_TOP_CLK_RC32M_REG_RC32M_ENABLE_Msk;
}

void
da1469x_clock_lp_xtal32k_enable(void)
{
    CRG_TOP->CLK_XTAL32K_REG |= CRG_TOP_CLK_XTAL32K_REG_XTAL32K_ENABLE_Msk;
}

void
da1469x_clock_lp_xtal32k_switch(void)
{
    CRG_TOP->CLK_CTRL_REG = (CRG_TOP->CLK_CTRL_REG &
                             ~CRG_TOP_CLK_CTRL_REG_LP_CLK_SEL_Msk) |
                            (2 << CRG_TOP_CLK_CTRL_REG_LP_CLK_SEL_Pos);
}

void
da1469x_clock_pll_disable(void)
{
    while (CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_PLL96M_Msk) {
        CRG_TOP->CLK_SWITCH2XTAL_REG = CRG_TOP_CLK_SWITCH2XTAL_REG_SWITCH2XTAL_Msk;
    }

    CRG_XTAL->PLL_SYS_CTRL1_REG &= ~CRG_XTAL_PLL_SYS_CTRL1_REG_PLL_EN_Msk;
}

void
da1469x_clock_pll_wait_to_lock(void)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    NVIC_ClearPendingIRQ(PLL_LOCK_IRQn);

    if (!da1469x_clock_is_pll_locked()) {
        NVIC_EnableIRQ(PLL_LOCK_IRQn);
        while (!NVIC_GetPendingIRQ(PLL_LOCK_IRQn)) {
            __WFI();
        }
        NVIC_DisableIRQ(PLL_LOCK_IRQn);
    }

    __HAL_ENABLE_INTERRUPTS(primask);
}

void
da1469x_clock_sys_pll_switch(void)
{
    /* CLK_SEL_Msk == 3 means PLL */
    CRG_TOP->CLK_CTRL_REG |= CRG_TOP_CLK_CTRL_REG_SYS_CLK_SEL_Msk;

    while (!(CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_PLL96M_Msk));
}
