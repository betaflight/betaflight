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

// External interrupt (EXTI) implementation using HPM GPIO pin
// interrupts, including software emulation of both-edge triggering.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_EXTI
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/io_impl.h"
#include "drivers/exti.h"
#include "nvic_hpmicro.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
typedef enum {
    HPM_GPIOA = 0,
    HPM_GPIOB,
    HPM_GPIOC,
    HPM_GPIOD,
    HPM_GPIOE,
    HPM_GPIOF,
    HPM_GPIO_RSV0,
    HPM_GPIO_RSV1,
    HPM_GPIO_RSV2,
    HPM_GPIO_RSV3,
    HPM_GPIO_RSV4,
    HPM_GPIO_RSV5,
    HPM_GPIO_RSV6,
    HPM_GPIOX = 13,
    HPM_GPIOY,
    HPM_GPIOZ,
} gpioPortIdx_t;
// Each port has 32 independent interrupt-capable pins (unlike STM32 where 16
// EXTI lines are muxed across ports via SYSCFG).
typedef struct extiChannelRec_s {
    extiCallbackRec_t *handler[32];
    uint32_t bothEdgeMask;
} extiChannelRec_t;

#define EXTI_IRQ_GROUPS 16
// Sentinel value for extiGroupIRQn[]: no interrupt line exists for this port
// on this SoC (e.g. GPIOE/F on HPM6360, or unused port slots 6-12).
#define EXTI_IRQN_NONE 255

FAST_DATA_ZERO_INIT extiChannelRec_t extiChannelRecs[EXTI_IRQ_GROUPS];
static uint32_t extiGroupPriority[EXTI_IRQ_GROUPS];

static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    IRQn_GPIO0_A,               //0
    IRQn_GPIO0_B,               //1
    IRQn_GPIO0_C,               //2
#ifdef HPM6750
    IRQn_GPIO0_D,               //3
    IRQn_GPIO0_E,               //4
    IRQn_GPIO0_F,               //5
#elif defined(HPM6360)
    EXTI_IRQN_NONE,             //3  GPIOD has no data port or ISR on HPM6360
    EXTI_IRQN_NONE,             //4  GPIOE has no IRQ line on HPM6360
    EXTI_IRQN_NONE,             //5  GPIOF has no IRQ line on HPM6360
#else
#error "Unsupported HPMicro SoC"
#endif
    EXTI_IRQN_NONE,             //6
    EXTI_IRQN_NONE,             //7
    EXTI_IRQN_NONE,             //8
    EXTI_IRQN_NONE,             //9
    EXTI_IRQN_NONE,             //10
    EXTI_IRQN_NONE,             //11
    EXTI_IRQN_NONE,             //12
    IRQn_GPIO0_X,               //13
    IRQn_GPIO0_Y,               //14
    IRQn_GPIO0_Z,               //15
};

static const gpio_interrupt_trigger_t triggerLookupTable[] = {
    [BETAFLIGHT_EXTI_TRIGGER_RISING] = gpio_interrupt_trigger_edge_rising,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING] = gpio_interrupt_trigger_edge_falling
};

void EXTIInit(void)
{
    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
    memset(extiGroupPriority, 0, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);

    if (portIdx < 0 || portIdx >= EXTI_IRQ_GROUPS) {
        return;
    }
    if (extiGroupIRQn[portIdx] == EXTI_IRQN_NONE) {
        // No interrupt line for this port on this SoC (e.g. GPIOE/F on HPM6360)
        return;
    }
    if (chIdx < 0 || chIdx > 31) {
        return;
    }
    if ((unsigned) trigger > BETAFLIGHT_EXTI_TRIGGER_BOTH) {
        return;
    }
    extiChannelRec_t *rec = &extiChannelRecs[portIdx];
    rec->handler[chIdx] = cb;

    EXTIDisable(io);

    IOConfigGPIO(io, config);

    const uint32_t pinMask = 1U << chIdx;
    gpio_interrupt_trigger_t trigger1;
    if (trigger == BETAFLIGHT_EXTI_TRIGGER_BOTH) {
        // HPM6300/HPM6700 have no native both-edge mode.  Arm the edge
        // opposite to the current level and update it after every interrupt.
        rec->bothEdgeMask |= pinMask;
        trigger1 = (gpio_read_pin(HPM_GPIO0, portIdx, chIdx))
                   ? gpio_interrupt_trigger_edge_falling : gpio_interrupt_trigger_edge_rising;
    } else {
        rec->bothEdgeMask &= ~pinMask;
        trigger1 = triggerLookupTable[trigger];
    }

    gpio_config_pin_interrupt(HPM_GPIO0, portIdx, chIdx, trigger1);

    // Clear any stale interrupt flag that may have been set by IOConfigGPIO
    // toggling the pin pad configuration, so we don't fire a spurious ISR
    // the moment the interrupt is enabled.
    gpio_clear_pin_interrupt_flag(HPM_GPIO0, portIdx, chIdx);

    // Callers pass STM32-encoded NVIC priorities (lower value = more urgent);
    // the PLIC uses the opposite convention (0 disables, larger = more urgent),
    // so decode the encoded priority onto 1..7 before comparing
    const uint32_t plicPriority = hpmPlicPriorityFromNvic(irqPriority);
    if (extiGroupPriority[portIdx] == 0) {
        extiGroupPriority[portIdx] = plicPriority;
        intc_m_enable_irq_with_priority(extiGroupIRQn[portIdx], plicPriority);
    } else if (extiGroupPriority[portIdx] < plicPriority) {
        extiGroupPriority[portIdx] = plicPriority;
        intc_set_irq_priority(extiGroupIRQn[portIdx], plicPriority);
    }
    EXTIEnable(io);
}

void EXTIRelease(IO_t io)
{
    EXTIDisable(io);

    const int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);

    if (chIdx < 0 || portIdx < 0 || portIdx >= EXTI_IRQ_GROUPS) {
        return;
    }

    extiChannelRec_t *rec = &extiChannelRecs[portIdx];
    rec->bothEdgeMask &= ~(1U << chIdx);
    rec->handler[chIdx] = NULL;
}

FAST_CODE void EXTIEnable(IO_t io)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);

    if (chIdx < 0 || portIdx < 0 || portIdx >= EXTI_IRQ_GROUPS) {
        return;
    }

    gpio_enable_pin_interrupt(HPM_GPIO0, portIdx, chIdx);
}

FAST_CODE void EXTIDisable(IO_t io)
{
    int chIdx = IO_GPIOPinIdx(io);
    int portIdx = IO_GPIOPortIdx(io);

    if (chIdx < 0 || portIdx < 0 || portIdx >= EXTI_IRQ_GROUPS) {
        return;
    }

    gpio_disable_pin_interrupt(HPM_GPIO0, portIdx, chIdx);
    gpio_clear_pin_interrupt_flag(HPM_GPIO0, portIdx, chIdx);
}

static inline unsigned extiLowestSetBitIndex(uint32_t value)
{
    return __builtin_ctz(value);
}

FAST_CODE static void gpio_interrupt_handler(GPIO_Type *base, unsigned char port_index, unsigned char group)
{
    uint32_t flag = gpio_get_port_interrupt_flags(base, port_index);
    while (flag != 0) {
        const unsigned i = extiLowestSetBitIndex(flag);
        const uint32_t pinMask = 1U << i;
        flag &= flag - 1;

        gpio_clear_pin_interrupt_flag(base, port_index, i);
        if (extiChannelRecs[group].bothEdgeMask & pinMask) {
            const gpio_interrupt_trigger_t nextTrigger =
                (gpio_read_pin(base, port_index, i)) ? gpio_interrupt_trigger_edge_falling
                                                     : gpio_interrupt_trigger_edge_rising;
            gpio_config_pin_interrupt(base, port_index, i, nextTrigger);
        }
        if (extiChannelRecs[group].handler[i] && extiChannelRecs[group].handler[i]->fn != NULL) {
            extiChannelRecs[group].handler[i]->fn(extiChannelRecs[group].handler[i]);
        }
    }
}

#if defined(IRQn_GPIO0_A) && defined(GPIO_DI_GPIOA)
void gpio_porta_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOA, HPM_GPIOA);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_A, gpio_porta_isr)
#endif
#if defined(IRQn_GPIO0_B) && defined(GPIO_DI_GPIOB)
void gpio_portb_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOB, HPM_GPIOB);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B, gpio_portb_isr)
#endif
#if defined(IRQn_GPIO0_C) && defined(GPIO_DI_GPIOC)
void gpio_portc_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOC, HPM_GPIOC);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_C, gpio_portc_isr)
#endif
#if defined(IRQn_GPIO0_D) && defined(GPIO_DI_GPIOD)
void gpio_portd_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOD, HPM_GPIOD);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_D, gpio_portd_isr)
#endif
#if defined(IRQn_GPIO0_E) && defined(GPIO_DI_GPIOE)
void gpio_porte_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOE, HPM_GPIOE);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_E, gpio_porte_isr)
#endif
#if defined(IRQn_GPIO0_F) && defined(GPIO_DI_GPIOF)
void gpio_portf_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOF, HPM_GPIOF);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_F, gpio_portf_isr)
#endif
#if defined(IRQn_GPIO0_X) && defined(GPIO_DI_GPIOX)
void gpio_portx_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOX, HPM_GPIOX);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_X, gpio_portx_isr)
#endif
#if defined(IRQn_GPIO0_Y) && defined(GPIO_DI_GPIOY)
void gpio_porty_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOY, HPM_GPIOY);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_porty_isr)
#endif
#if defined(IRQn_GPIO0_Z) && defined(GPIO_DI_GPIOZ)
void gpio_portz_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOZ, HPM_GPIOZ);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Z, gpio_portz_isr)
#endif
#endif
