
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
#include <stddef.h>
#include "syscfg/syscfg.h"
#include "mcu/da1469x_hal.h"
#include <mcu/mcu.h>
#include "hal/hal_gpio.h"

/* GPIO interrupts */
#define HAL_GPIO_MAX_IRQ        MYNEWT_VAL(MCU_GPIO_MAX_IRQ)

#define GPIO_REG(name) ((__IO uint32_t *)(GPIO_BASE + offsetof(GPIO_Type, name)))
#define WAKEUP_REG(name) ((__IO uint32_t *)(WAKEUP_BASE + offsetof(WAKEUP_Type, name)))
#define CRG_TOP_REG(name) ((__IO uint32_t *)(CRG_TOP_BASE + offsetof(CRG_TOP_Type, name)))

#ifndef MCU_GPIO_PORT0_PIN_COUNT
#define MCU_GPIO_PORT0_PIN_COUNT 32
#endif

#if (MCU_GPIO_PORT0_PIN_COUNT) == 32
#define GPIO_PORT(pin)          (((unsigned)(pin)) >> 5U)
#define GPIO_PORT_PIN(pin)      (((unsigned)(pin)) & 31U)
#else
#define GPIO_PORT(pin)          (((unsigned)(pin)) < MCU_GPIO_PORT0_PIN_COUNT ? 0 : 1)
#define GPIO_PORT_PIN(pin)      ((unsigned)(pin) < MCU_GPIO_PORT0_PIN_COUNT ? \
                                (pin) : (pin) - MCU_GPIO_PORT0_PIN_COUNT)
#endif

#define GPIO_PIN_BIT(pin)       (1 << GPIO_PORT_PIN(pin))

#define GPIO_PIN_DATA_REG_ADDR(pin)        (GPIO_REG(P0_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_DATA_REG(pin)             *GPIO_PIN_DATA_REG_ADDR(pin)
#define GPIO_PIN_SET_DATA_REG_ADDR(pin)    (GPIO_REG(P0_SET_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_SET_DATA_REG(pin)         *GPIO_PIN_SET_DATA_REG_ADDR(pin)
#define GPIO_PIN_RESET_DATA_REG_ADDR(pin)  (GPIO_REG(P0_RESET_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_RESET_DATA_REG(pin)       *GPIO_PIN_RESET_DATA_REG_ADDR(pin)
#define GPIO_PIN_MODE_REG_ADDR(pin)        (GPIO_REG(P0_00_MODE_REG) + (pin))
#define GPIO_PIN_MODE_REG(pin)             *GPIO_PIN_MODE_REG_ADDR(pin)
#define GPIO_PIN_PADPWR_CTRL_REG_ADDR(pin) (GPIO_REG(P0_PADPWR_CTRL_REG) + GPIO_PORT(pin))
#define GPIO_PIN_PADPWR_CTRL_REG(pin)      *GPIO_PIN_PADPWR_CTRL_REG_ADDR(pin)
#define GPIO_PIN_UNLATCH_ADDR(pin)         (CRG_TOP_REG(P0_SET_PAD_LATCH_REG) + GPIO_PORT(pin) * 3)
#define GPIO_PIN_LATCH_ADDR(pin)           (CRG_TOP_REG(P0_RESET_PAD_LATCH_REG) + GPIO_PORT(pin) * 3)

#define WKUP_CTRL_REG_ADDR              (WAKEUP_REG(WKUP_CTRL_REG))
#define WKUP_RESET_IRQ_REG_ADDR         (WAKEUP_REG(WKUP_RESET_IRQ_REG))
#define WKUP_SELECT_PX_REG_ADDR(pin)    (WAKEUP_REG(WKUP_SELECT_P0_REG) + GPIO_PORT(pin))
#define WKUP_SELECT_PX_REG(pin)         *(WKUP_SELECT_PX_REG_ADDR(pin))
#define WKUP_POL_PX_REG_ADDR(pin)       (WAKEUP_REG(WKUP_POL_P0_REG) + GPIO_PORT(pin))
#define WKUP_POL_PX_SET_FALLING(pin)    do { *(WKUP_POL_PX_REG_ADDR(pin)) |= GPIO_PIN_BIT(pin); } while (0)
#define WKUP_POL_PX_SET_RISING(pin)     do { *(WKUP_POL_PX_REG_ADDR(pin)) &= ~GPIO_PIN_BIT(pin); } while (0)
#define WKUP_STAT_PX_REG_ADDR(pin)      (WAKEUP_REG(WKUP_STATUS_P0_REG) + GPIO_PORT(pin))
#define WKUP_STAT(pin)                  ((*(WKUP_STAT_PX_REG_ADDR(pin)) >> GPIO_PORT_PIN(pin)) & 1)
#define WKUP_CLEAR_PX_REG_ADDR(pin)     (WAKEUP_REG(WKUP_CLEAR_P0_REG) + GPIO_PORT(pin))
#define WKUP_CLEAR_PX(pin)              do { (*(WKUP_CLEAR_PX_REG_ADDR(pin)) = GPIO_PIN_BIT(pin)); } while (0)
#define WKUP_SEL_GPIO_PX_REG_ADDR(pin)  (WAKEUP_REG(WKUP_SEL_GPIO_P0_REG) + GPIO_PORT(pin))
#define WKUP_SEL_GPIO_PX_REG(pin)       *(WKUP_SEL_GPIO_PX_REG_ADDR(pin))

/* Storage for GPIO callbacks. */
struct hal_gpio_irq {
    int pin;
    hal_gpio_irq_handler_t func;
    void *arg;
};

static struct hal_gpio_irq hal_gpio_irqs[HAL_GPIO_MAX_IRQ];

#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
static uint32_t g_mcu_gpio_latch_state[2];
static uint8_t g_mcu_gpio_retained_num;
static struct da1469x_retreg g_mcu_gpio_retained[MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM)];
#endif

/*
 * We assume that any latched pin has default configuration, i.e. was either
 * not configured or was deinited. Any unlatched pin is considered to be used
 * by someone.
 *
 * By default, all pins are assumed to have default configuration and are
 * latched. This allows PD_COM to be disabled (if no other peripheral needs
 * it) since we do not need GPIO mux to be active.
 *
 * Configuration of any pin shall be done as follows, with interrupts disabled:
 * 1. call mcu_gpio_unlatch_prepare() to enable PD_COM if needed
 * 2. configure pin
 * 3. call mcu_gpio_unlatch() to actually unlatch pin
 *
 * Once pin is restored to default configuration it shall be latched again by
 * calling mcu_gpio_latch().
 */

#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
static void
mcu_gpio_retained_add_port(uint32_t latch_val, volatile uint32_t *base_reg)
{
    struct da1469x_retreg *retreg;
    int pin;

    retreg = &g_mcu_gpio_retained[g_mcu_gpio_retained_num];

    while (latch_val) {
        assert(g_mcu_gpio_retained_num < MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM));

        pin = __builtin_ctz(latch_val);
        latch_val &= ~(1 << pin);

        da1469x_retreg_assign(retreg, &base_reg[pin]);

        g_mcu_gpio_retained_num++;
        retreg++;
    }
}
#endif

static void
mcu_gpio_retained_refresh(void)
{
#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
    g_mcu_gpio_retained_num = 0;

    mcu_gpio_retained_add_port(CRG_TOP->P0_PAD_LATCH_REG, &GPIO->P0_00_MODE_REG);
    mcu_gpio_retained_add_port(CRG_TOP->P1_PAD_LATCH_REG, &GPIO->P1_00_MODE_REG);
#endif
}

static inline void
mcu_gpio_unlatch_prepare(int pin)
{
    __HAL_ASSERT_CRITICAL();
    (void)pin;

    /* Acquire PD_COM if first pin will be unlatched */
//    if ((CRG_TOP->P0_PAD_LATCH_REG | CRG_TOP->P1_PAD_LATCH_REG) == 0) {
//        da1469x_pd_acquire(MCU_PD_DOMAIN_COM);
//    }
}

static inline void
mcu_gpio_unlatch(int pin)
{
    __HAL_ASSERT_CRITICAL();

    *GPIO_PIN_UNLATCH_ADDR(pin) = GPIO_PIN_BIT(pin);
    mcu_gpio_retained_refresh();
}

static inline void
mcu_gpio_latch(int pin)
{
    (void)pin;
//    uint32_t primask;
//    uint32_t latch_pre;
//    uint32_t latch_post;
//
//    __HAL_DISABLE_INTERRUPTS(primask);
//
//    latch_pre = CRG_TOP->P0_PAD_LATCH_REG | CRG_TOP->P1_PAD_LATCH_REG;
//
//    *GPIO_PIN_LATCH_ADDR(pin) = GPIO_PIN_BIT(pin);
//    mcu_gpio_retained_refresh();
//
//    latch_post = CRG_TOP->P0_PAD_LATCH_REG | CRG_TOP->P1_PAD_LATCH_REG;
//
//    /* Release PD_COM if last pin was latched */
//    if (latch_pre && !latch_post) {
//        da1469x_pd_release(MCU_PD_DOMAIN_COM);
//    }
//
//    __HAL_ENABLE_INTERRUPTS(primask);
}

int
hal_gpio_init_in(int pin, hal_gpio_pull_t pull)
{
    volatile uint32_t *px_xx_mod_reg = GPIO_PIN_MODE_REG_ADDR(pin);
    uint32_t regval;
    uint32_t primask;

    switch (pull) {
    case HAL_GPIO_PULL_UP:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT_PULLUP;
        break;
    case HAL_GPIO_PULL_DOWN:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT_PULLDOWN;
        break;
    case HAL_GPIO_PULL_NONE:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT;
        break;
    default:
        return -1;
    }

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(pin);

    *px_xx_mod_reg = regval;

    mcu_gpio_unlatch(pin);

    __HAL_ENABLE_INTERRUPTS(primask);

    return 0;
}

int
hal_gpio_init_out(int pin, int val)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(pin);

    GPIO_PIN_MODE_REG(pin) = MCU_GPIO_MODE_OUTPUT;

    if (val) {
        GPIO_PIN_SET_DATA_REG(pin) = GPIO_PIN_BIT(pin);
    } else {
        GPIO_PIN_RESET_DATA_REG(pin) = GPIO_PIN_BIT(pin);
    }

    mcu_gpio_unlatch(pin);

    __HAL_ENABLE_INTERRUPTS(primask);

    return 0;
}

int
hal_gpio_deinit(int pin)
{
    /* Reset mode to default value and latch pin */
    GPIO_PIN_MODE_REG(pin) = 0x200;
    GPIO_PIN_RESET_DATA_REG(pin) = GPIO_PIN_BIT(pin);

    mcu_gpio_latch(pin);

    return 0;
}

void
hal_gpio_write(int pin, int val)
{
    if (val) {
        GPIO_PIN_SET_DATA_REG(pin) = GPIO_PIN_BIT(pin);
    } else {
        GPIO_PIN_RESET_DATA_REG(pin) = GPIO_PIN_BIT(pin);
    }
}

int
hal_gpio_read(int pin)
{
    return (GPIO_PIN_DATA_REG(pin) >> GPIO_PORT_PIN(pin)) & 1;
}

int
hal_gpio_toggle(int pin)
{
    int new_value = hal_gpio_read(pin) == 0;

    hal_gpio_write(pin, new_value);

    return new_value;
}

static void
hal_gpio_irq_handler(void)
{
    struct hal_gpio_irq *irq;
    uint32_t stat;
    int i;

    *WKUP_RESET_IRQ_REG_ADDR = 1;
    NVIC_ClearPendingIRQ(KEY_WKUP_GPIO_IRQn);

    for (i = 0; i < HAL_GPIO_MAX_IRQ; i++) {
        irq = &hal_gpio_irqs[i];

        /* Read latched status value from relevant GPIO port */
        stat = WKUP_STAT(irq->pin);

        if (irq->func && stat) {
            irq->func(irq->arg);
        }

        WKUP_CLEAR_PX(irq->pin);
    }
}

static void
hal_gpio_irq_setup(void)
{
    static uint8_t irq_setup;
    int sr;

    if (!irq_setup) {
        __HAL_DISABLE_INTERRUPTS(sr);

        irq_setup = 1;

        NVIC_ClearPendingIRQ(GPIO_P0_IRQn);
        NVIC_ClearPendingIRQ(GPIO_P1_IRQn);
        NVIC_SetVector(GPIO_P0_IRQn, (uint32_t)hal_gpio_irq_handler);
        NVIC_SetVector(GPIO_P1_IRQn, (uint32_t)hal_gpio_irq_handler);
        WAKEUP->WKUP_CTRL_REG = 0;
        WAKEUP->WKUP_CLEAR_P0_REG = 0xFFFFFFFF;
        WAKEUP->WKUP_CLEAR_P1_REG = 0x007FFFFF;
        WAKEUP->WKUP_SELECT_P0_REG = 0;
        WAKEUP->WKUP_SELECT_P1_REG = 0;
        WAKEUP->WKUP_SEL_GPIO_P0_REG = 0;
        WAKEUP->WKUP_SEL_GPIO_P1_REG = 0;
        WAKEUP->WKUP_RESET_IRQ_REG = 0;

        CRG_TOP->CLK_TMR_REG |= CRG_TOP_CLK_TMR_REG_WAKEUPCT_ENABLE_Msk;

        __HAL_ENABLE_INTERRUPTS(sr);
        NVIC_EnableIRQ(GPIO_P0_IRQn);
        NVIC_EnableIRQ(GPIO_P1_IRQn);
    }
}

static int
hal_gpio_find_empty_slot(void)
{
    int i;

    for (i = 0; i < HAL_GPIO_MAX_IRQ; i++) {
        if (hal_gpio_irqs[i].func == NULL) {
            return i;
        }
    }

    return -1;
}

int
hal_gpio_irq_init(int pin, hal_gpio_irq_handler_t handler, void *arg,
                  hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull)
{
    int i;

    hal_gpio_irq_setup();

    i = hal_gpio_find_empty_slot();
    /* If assert failed increase syscfg value MCU_GPIO_MAX_IRQ */
    assert(i >= 0);
    if (i < 0) {
        return -1;
    }

    hal_gpio_init_in(pin, pull);

    switch (trig) {
    case HAL_GPIO_TRIG_RISING:
        WKUP_POL_PX_SET_RISING(pin);
        break;
    case HAL_GPIO_TRIG_FALLING:
        WKUP_POL_PX_SET_FALLING(pin);
        break;
    case HAL_GPIO_TRIG_BOTH:
        /* Not supported */
    default:
        return -1;
    }

    hal_gpio_irqs[i].pin = pin;
    hal_gpio_irqs[i].func = handler;
    hal_gpio_irqs[i].arg = arg;

    return 0;
}

void
hal_gpio_irq_release(int pin)
{
    int i;

    hal_gpio_irq_disable(pin);

    for (i = 0; i < HAL_GPIO_MAX_IRQ; i++) {
        if (hal_gpio_irqs[i].pin == pin && hal_gpio_irqs[i].func) {
            hal_gpio_irqs[i].pin = -1;
            hal_gpio_irqs[i].arg = NULL;
            hal_gpio_irqs[i].func = NULL;
        }
    }
}

void
hal_gpio_irq_enable(int pin)
{
    WKUP_SEL_GPIO_PX_REG(pin) |= GPIO_PIN_BIT(pin);
}

void
hal_gpio_irq_disable(int pin)
{
    WKUP_SEL_GPIO_PX_REG(pin) &= ~GPIO_PIN_BIT(pin);
    WKUP_CLEAR_PX(pin);
}

void
mcu_gpio_set_pin_function(int pin, int mode, mcu_gpio_func func)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(pin);

    GPIO_PIN_MODE_REG(pin) = (func & GPIO_P0_00_MODE_REG_PID_Msk) |
        (mode & (GPIO_P0_00_MODE_REG_PUPD_Msk | GPIO_P0_00_MODE_REG_PPOD_Msk));

    mcu_gpio_unlatch(pin);

    __HAL_ENABLE_INTERRUPTS(primask);
}

void
mcu_gpio_enter_sleep(void)
{
#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
    if (g_mcu_gpio_retained_num == 0) {
        return;
    }

    g_mcu_gpio_latch_state[0] = CRG_TOP->P0_PAD_LATCH_REG;
    g_mcu_gpio_latch_state[1] = CRG_TOP->P1_PAD_LATCH_REG;

    da1469x_retreg_update(g_mcu_gpio_retained, g_mcu_gpio_retained_num);

    CRG_TOP->P0_RESET_PAD_LATCH_REG = CRG_TOP_P0_PAD_LATCH_REG_P0_LATCH_EN_Msk;
    CRG_TOP->P1_RESET_PAD_LATCH_REG = CRG_TOP_P1_PAD_LATCH_REG_P1_LATCH_EN_Msk;

    da1469x_pd_release(MCU_PD_DOMAIN_COM);
#endif
}

void
mcu_gpio_exit_sleep(void)
{
#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
    if (g_mcu_gpio_retained_num == 0) {
        return;
    }

    da1469x_pd_acquire(MCU_PD_DOMAIN_COM);

    da1469x_retreg_restore(g_mcu_gpio_retained, g_mcu_gpio_retained_num);

    /* Set pins states to their latched values */
    GPIO->P0_DATA_REG = GPIO->P0_DATA_REG;
    GPIO->P1_DATA_REG = GPIO->P1_DATA_REG;

    CRG_TOP->P0_PAD_LATCH_REG = g_mcu_gpio_latch_state[0];
    CRG_TOP->P1_PAD_LATCH_REG = g_mcu_gpio_latch_state[1];
#endif
}
