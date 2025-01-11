/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Ha Thach (tinyusb.org)
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


#ifndef TUSB_DWC2_ESP32_H_
#define TUSB_DWC2_ESP32_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_intr_alloc.h"
#include "soc/periph_defs.h"
#include "soc/usb_wrap_struct.h"

#if TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)
#define DWC2_FS_REG_BASE   0x60080000UL
#define DWC2_EP_MAX        7

static const dwc2_controller_t _dwc2_controller[] = {
  { .reg_base = DWC2_FS_REG_BASE, .irqnum = ETS_USB_INTR_SOURCE, .ep_count = 7, .ep_in_count = 5, .ep_fifo_size = 1024 }
};

#elif TU_CHECK_MCU(OPT_MCU_ESP32P4)
#define DWC2_FS_REG_BASE   0x50040000UL
#define DWC2_HS_REG_BASE   0x50000000UL
#define DWC2_EP_MAX        16

// On ESP32 for consistency we associate
// - Port0 to OTG_FS, and Port1 to OTG_HS
static const dwc2_controller_t _dwc2_controller[] = {
{ .reg_base = DWC2_FS_REG_BASE, .irqnum = ETS_USB_OTG11_CH0_INTR_SOURCE, .ep_count = 7, .ep_in_count = 5, .ep_fifo_size = 1024 },
{ .reg_base = DWC2_HS_REG_BASE, .irqnum = ETS_USB_OTG_INTR_SOURCE, .ep_count = 16, .ep_in_count = 8, .ep_fifo_size = 4096 }
};
#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
static intr_handle_t usb_ih[TU_ARRAY_SIZE(_dwc2_controller)];

static void dwc2_int_handler_wrap(void* arg) {
  const uint8_t rhport = tu_u16_low((uint16_t)(uintptr_t)arg);
  const tusb_role_t role = (tusb_role_t) tu_u16_high((uint16_t)(uintptr_t)arg);
#if CFG_TUD_ENABLED
  if (role == TUSB_ROLE_DEVICE) {
    dcd_int_handler(rhport);
  }
#endif
#if CFG_TUH_ENABLED && !CFG_TUH_MAX3421
  if (role == TUSB_ROLE_HOST) {
    hcd_int_handler(rhport, true);
  }
#endif
}

TU_ATTR_ALWAYS_INLINE static inline void dwc2_int_set(uint8_t rhport, tusb_role_t role, bool enabled) {
  if (enabled) {
    esp_intr_alloc(_dwc2_controller[rhport].irqnum, ESP_INTR_FLAG_LOWMED,
                   dwc2_int_handler_wrap, (void*)(uintptr_t)tu_u16(role, rhport), &usb_ih[rhport]);
  } else {
    esp_intr_free(usb_ih[rhport]);
  }
}

#define dwc2_dcd_int_enable(_rhport)  dwc2_int_set(_rhport, TUSB_ROLE_DEVICE, true)
#define dwc2_dcd_int_disable(_rhport) dwc2_int_set(_rhport, TUSB_ROLE_DEVICE, false)

TU_ATTR_ALWAYS_INLINE static inline void dwc2_remote_wakeup_delay(void) {
  vTaskDelay(pdMS_TO_TICKS(1));
}

// MCU specific PHY init, called BEFORE core reset
TU_ATTR_ALWAYS_INLINE static inline void dwc2_phy_init(dwc2_regs_t* dwc2, uint8_t hs_phy_type) {
  (void)dwc2;
  (void)hs_phy_type;
  // maybe usb_utmi_hal_init()

}

// MCU specific PHY update, it is called AFTER init() and core reset
TU_ATTR_ALWAYS_INLINE static inline void dwc2_phy_update(dwc2_regs_t* dwc2, uint8_t hs_phy_type) {
  (void)dwc2;
  (void)hs_phy_type;
  // maybe usb_utmi_hal_disable()
}

//--------------------------------------------------------------------+
// Data Cache
//--------------------------------------------------------------------+
#if CFG_TUD_DWC2_DMA_ENABLE || CFG_TUH_DWC2_DMA_ENABLE
#if defined(SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE) && SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#include "esp_cache.h"

#if CFG_TUD_MEM_DCACHE_LINE_SIZE != CONFIG_CACHE_L1_CACHE_LINE_SIZE || \
    CFG_TUH_MEM_DCACHE_LINE_SIZE != CONFIG_CACHE_L1_CACHE_LINE_SIZE
#error "CFG_TUD/TUH_MEM_DCACHE_LINE_SIZE must match CONFIG_CACHE_L1_CACHE_LINE_SIZE"
#endif

TU_ATTR_ALWAYS_INLINE static inline uint32_t round_up_to_cache_line_size(uint32_t size) {
  if (size & (CONFIG_CACHE_L1_CACHE_LINE_SIZE-1)) {
    size = (size & ~(CONFIG_CACHE_L1_CACHE_LINE_SIZE-1)) + CONFIG_CACHE_L1_CACHE_LINE_SIZE;
  }
  return size;
}

TU_ATTR_ALWAYS_INLINE static inline bool dwc2_dcache_clean(const void* addr, uint32_t data_size) {
  const int flag = ESP_CACHE_MSYNC_FLAG_TYPE_DATA | ESP_CACHE_MSYNC_FLAG_DIR_C2M;
  data_size = round_up_to_cache_line_size(data_size);
  return ESP_OK == esp_cache_msync((void*)addr, data_size, flag);
}

TU_ATTR_ALWAYS_INLINE static inline bool dwc2_dcache_invalidate(const void* addr, uint32_t data_size) {
  const int flag = ESP_CACHE_MSYNC_FLAG_TYPE_DATA | ESP_CACHE_MSYNC_FLAG_DIR_M2C;
  data_size = round_up_to_cache_line_size(data_size);
  return ESP_OK == esp_cache_msync((void*)addr, data_size, flag);
}

TU_ATTR_ALWAYS_INLINE static inline bool dwc2_dcache_clean_invalidate(const void* addr, uint32_t data_size) {
  const int flag = ESP_CACHE_MSYNC_FLAG_TYPE_DATA | ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_DIR_M2C;
  data_size = round_up_to_cache_line_size(data_size);
  return ESP_OK == esp_cache_msync((void*)addr, data_size, flag);
}

#endif
#endif

#ifdef __cplusplus
}
#endif

#endif
