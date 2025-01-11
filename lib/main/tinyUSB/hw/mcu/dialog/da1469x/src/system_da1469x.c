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

#include "mcu/mcu.h"
#include <mcu/da1469x_clock.h>

extern uint8_t __StackLimit;

uint32_t SystemCoreClock = 32000000;

void
SystemInit(void)
{
  /* Enable FPU when using hard-float */
#if (__FPU_USED == 1)
  SCB->CPACR |= (3UL << 20) | (3UL << 22);
  __DSB();
  __ISB();
#endif

  /* Freeze watchdog */
  GPREG->SET_FREEZE_REG |= GPREG_SET_FREEZE_REG_FRZ_SYS_WDOG_Msk;
  /* Initialize power domains (disable radio only) */
  CRG_TOP->PMU_CTRL_REG = CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Msk;

  CRG_TOP->P0_SET_PAD_LATCH_REG = CRG_TOP_P0_PAD_LATCH_REG_P0_LATCH_EN_Msk;
  CRG_TOP->P1_SET_PAD_LATCH_REG = CRG_TOP_P1_PAD_LATCH_REG_P1_LATCH_EN_Msk;

  /* Reset clock dividers to 0 */
  CRG_TOP->CLK_AMBA_REG &= ~(CRG_TOP_CLK_AMBA_REG_HCLK_DIV_Msk | CRG_TOP_CLK_AMBA_REG_PCLK_DIV_Msk);

  /* PD_TIM is already started in SystemInit */

  da1469x_clock_sys_xtal32m_init();
  da1469x_clock_sys_xtal32m_enable();
  da1469x_clock_sys_pll_enable();
  da1469x_clock_pll_wait_to_lock();
  /* Switch to XTAL32M and disable RC32M */
  da1469x_clock_sys_xtal32m_switch_safe();
  da1469x_clock_sys_rc32m_disable();
}

void _init(void)
{
}
