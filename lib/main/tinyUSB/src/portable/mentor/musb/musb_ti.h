/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024, Brent Kowal (Analog Devices, Inc)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef TUSB_MUSB_TI_H_
#define TUSB_MUSB_TI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#if CFG_TUSB_MCU == OPT_MCU_TM4C123
  #include "TM4C123.h"
  #define FIFO0_WORD FIFO0
  #define FIFO1_WORD FIFO1
//#elif CFG_TUSB_MCU == OPT_MCU_TM4C129
#elif CFG_TUSB_MCU == OPT_MCU_MSP432E4
  #include "msp.h"
#else
  #error "Unsupported MCUs"
#endif

#define MUSB_CFG_SHARED_FIFO       0
#define MUSB_CFG_DYNAMIC_FIFO      1
#define MUSB_CFG_DYNAMIC_FIFO_SIZE 4096

const uintptr_t MUSB_BASES[] = { USB0_BASE };

// Header supports both device and host modes. Only include what's necessary
#if CFG_TUD_ENABLED

// Mapping of IRQ numbers to port. Currently just 1.
static const IRQn_Type  musb_irqs[] = {
    USB0_IRQn
};

static inline void musb_dcd_phy_init(uint8_t rhport){
  (void)rhport;
  //Nothing to do for this part
}

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_enable(uint8_t rhport) {
  NVIC_EnableIRQ(musb_irqs[rhport]);
}

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_disable(uint8_t rhport) {
  NVIC_DisableIRQ(musb_irqs[rhport]);
}

TU_ATTR_ALWAYS_INLINE static inline unsigned musb_dcd_get_int_enable(uint8_t rhport) {
  return NVIC_GetEnableIRQ(musb_irqs[rhport]);
}

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_clear(uint8_t rhport) {
  NVIC_ClearPendingIRQ(musb_irqs[rhport]);
}

static inline void musb_dcd_int_handler_enter(uint8_t rhport) {
  (void)rhport;
  //Nothing to do for this part
}

#endif // CFG_TUD_ENABLED

#ifdef __cplusplus
 }
#endif

#endif // TUSB_MUSB_TI_H_
