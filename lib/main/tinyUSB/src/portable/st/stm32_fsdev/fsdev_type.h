/*
 * The MIT License (MIT)
 *
 * Copyright(c) N Conrad
 * Copyright(c) 2024, hathach (tinyusb.org)
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

#ifndef TUSB_FSDEV_TYPE_H
#define TUSB_FSDEV_TYPE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

// If sharing with CAN, one can set this to be non-zero to give CAN space where it wants it
// Both of these MUST be a multiple of 2, and are in byte units.
#ifndef FSDEV_BTABLE_BASE
#define FSDEV_BTABLE_BASE 0U
#endif

TU_VERIFY_STATIC(FSDEV_BTABLE_BASE % 8 == 0, "BTABLE base must be aligned to 8 bytes");

// FSDEV_PMA_SIZE is PMA buffer size in bytes.
// - 512-byte devices, access with a stride of two words (use every other 16-bit address)
// - 1024-byte devices, access with a stride of one word (use every 16-bit address)
// - 2048-byte devices, access with 32-bit address

// For purposes of accessing the packet
#if FSDEV_PMA_SIZE == 512
  // 1x16 bit / word access scheme
  #define FSDEV_PMA_STRIDE  2
  #define pma_access_scheme TU_ATTR_ALIGNED(4)
#elif FSDEV_PMA_SIZE == 1024
  // 2x16 bit / word access scheme
  #define FSDEV_PMA_STRIDE  1
  #define pma_access_scheme
#elif FSDEV_PMA_SIZE == 2048
  // 32 bit access scheme
  #define FSDEV_BUS_32BIT
  #define FSDEV_PMA_STRIDE  1
  #define pma_access_scheme
#endif

// The fsdev_bus_t type can be used for both register and PMA access necessities
#ifdef FSDEV_BUS_32BIT
  typedef uint32_t fsdev_bus_t;
  #define fsdevbus_unaligned_read(_addr)           tu_unaligned_read32(_addr)
  #define fsdevbus_unaligned_write(_addr, _value)  tu_unaligned_write32(_addr, _value)
#else
  typedef uint16_t fsdev_bus_t;
  #define fsdevbus_unaligned_read(_addr)           tu_unaligned_read16(_addr)
  #define fsdevbus_unaligned_write(_addr, _value)  tu_unaligned_write16(_addr, _value)
#endif

enum {
  FSDEV_BUS_SIZE = sizeof(fsdev_bus_t),
};

//--------------------------------------------------------------------+
// BTable Typedef
//--------------------------------------------------------------------+
enum {
  BTABLE_BUF_TX = 0,
  BTABLE_BUF_RX = 1
};

// hardware limit endpoint
#define FSDEV_EP_COUNT 8

// Buffer Table is located in Packet Memory Area (PMA) and therefore its address access is forced to either
// 16-bit or 32-bit depending on FSDEV_BUS_32BIT.
// 0: TX (IN), 1: RX (OUT)
typedef union {
  // data is strictly 16-bit access (address could be 32-bit aligned)
  struct {
    volatile pma_access_scheme uint16_t addr;
    volatile pma_access_scheme uint16_t count;
  } ep16[FSDEV_EP_COUNT][2];

  // strictly 32-bit access
  struct {
    volatile uint32_t count_addr;
  } ep32[FSDEV_EP_COUNT][2];
} fsdev_btable_t;

TU_VERIFY_STATIC(sizeof(fsdev_btable_t) == FSDEV_EP_COUNT*8*FSDEV_PMA_STRIDE, "size is not correct");
TU_VERIFY_STATIC(FSDEV_BTABLE_BASE + FSDEV_EP_COUNT*8 <= FSDEV_PMA_SIZE, "BTABLE does not fit in PMA RAM");

#define FSDEV_BTABLE ((volatile fsdev_btable_t*) (FSDEV_PMA_BASE + FSDEV_PMA_STRIDE*(FSDEV_BTABLE_BASE)))

typedef struct {
  volatile pma_access_scheme fsdev_bus_t value;
} fsdev_pma_buf_t;

#define PMA_BUF_AT(_addr) ((fsdev_pma_buf_t*) (FSDEV_PMA_BASE + FSDEV_PMA_STRIDE*(_addr)))

//--------------------------------------------------------------------+
// Registers Typedef
//--------------------------------------------------------------------+

// volatile 32-bit aligned
#define _va32     volatile TU_ATTR_ALIGNED(4)

typedef struct {
  struct {
    _va32 fsdev_bus_t reg;
  }ep[FSDEV_EP_COUNT];

  _va32 uint32_t RESERVED7[8];       // Reserved
  _va32 fsdev_bus_t CNTR;            // 40: Control register
  _va32 fsdev_bus_t ISTR;            // 44: Interrupt status register
  _va32 fsdev_bus_t FNR;             // 48: Frame number register
  _va32 fsdev_bus_t DADDR;           // 4C: Device address register
  _va32 fsdev_bus_t BTABLE;          // 50: Buffer Table address register (16-bit only)
  _va32 fsdev_bus_t LPMCSR;          // 54: LPM Control and Status Register (32-bit only)
  _va32 fsdev_bus_t BCDR;            // 58: Battery Charging Detector Register (32-bit only)
} fsdev_regs_t;

TU_VERIFY_STATIC(offsetof(fsdev_regs_t, CNTR) == 0x40, "Wrong offset");
TU_VERIFY_STATIC(sizeof(fsdev_regs_t) == 0x5C, "Size is not correct");

#define FSDEV_REG ((fsdev_regs_t*) FSDEV_REG_BASE)


#ifndef USB_EPTX_STAT
#define USB_EPTX_STAT 0x0030U
#endif

#ifndef USB_EPRX_STAT
#define USB_EPRX_STAT 0x3000U
#endif

#ifndef USB_EPTX_STAT_Pos
#define USB_EPTX_STAT_Pos    4u
#endif

#ifndef USB_EP_DTOG_TX_Pos
#define USB_EP_DTOG_TX_Pos   6u
#endif

#ifndef USB_EP_CTR_TX_Pos
#define USB_EP_CTR_TX_Pos    7u
#endif

typedef enum {
  EP_STAT_DISABLED = 0,
  EP_STAT_STALL = 1,
  EP_STAT_NAK = 2,
  EP_STAT_VALID = 3
}ep_stat_t;

#define EP_STAT_MASK(_dir)  (3u << (USB_EPTX_STAT_Pos + ((_dir) == TUSB_DIR_IN ? 0 : 8)))
#define EP_DTOG_MASK(_dir)  (1u << (USB_EP_DTOG_TX_Pos + ((_dir) == TUSB_DIR_IN ? 0 : 8)))

//--------------------------------------------------------------------+
// Endpoint Helper
// - CTR is write 0 to clear
// - DTOG and STAT are write 1 to toggle
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline uint32_t ep_read(uint32_t ep_id) {
  return FSDEV_REG->ep[ep_id].reg;
}

TU_ATTR_ALWAYS_INLINE static inline void ep_write(uint32_t ep_id, uint32_t value, bool need_exclusive) {
  if (need_exclusive) {
    dcd_int_disable(0);
  }

  FSDEV_REG->ep[ep_id].reg = (fsdev_bus_t) value;

  if (need_exclusive) {
    dcd_int_enable(0);
  }
}

TU_ATTR_ALWAYS_INLINE static inline void ep_write_clear_ctr(uint32_t ep_id, tusb_dir_t dir) {
  uint32_t reg = FSDEV_REG->ep[ep_id].reg;
  reg |= USB_EP_CTR_TX | USB_EP_CTR_RX;
  reg &= USB_EPREG_MASK;
  reg &= ~(1 << (USB_EP_CTR_TX_Pos + (dir == TUSB_DIR_IN ? 0 : 8)));
  ep_write(ep_id, reg, false);
}

TU_ATTR_ALWAYS_INLINE static inline void ep_change_status(uint32_t* reg, tusb_dir_t dir, ep_stat_t state) {
  *reg ^= (state << (USB_EPTX_STAT_Pos + (dir == TUSB_DIR_IN ? 0 : 8)));
}

TU_ATTR_ALWAYS_INLINE static inline void ep_change_dtog(uint32_t* reg, tusb_dir_t dir, uint8_t state) {
  *reg ^= (state << (USB_EP_DTOG_TX_Pos + (dir == TUSB_DIR_IN ? 0 : 8)));
}

TU_ATTR_ALWAYS_INLINE static inline bool ep_is_iso(uint32_t reg) {
  return (reg & USB_EP_TYPE_MASK) == USB_EP_ISOCHRONOUS;
}

//--------------------------------------------------------------------+
// BTable Helper
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline uint32_t btable_get_addr(uint32_t ep_id, uint8_t buf_id) {
#ifdef FSDEV_BUS_32BIT
  return FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr & 0x0000FFFFu;
#else
  return FSDEV_BTABLE->ep16[ep_id][buf_id].addr;
#endif
}

TU_ATTR_ALWAYS_INLINE static inline void btable_set_addr(uint32_t ep_id, uint8_t buf_id, uint16_t addr) {
#ifdef FSDEV_BUS_32BIT
  uint32_t count_addr = FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr;
  count_addr = (count_addr & 0xFFFF0000u) | (addr & 0x0000FFFCu);
  FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr = count_addr;
#else
  FSDEV_BTABLE->ep16[ep_id][buf_id].addr = addr;
#endif
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t btable_get_count(uint32_t ep_id, uint8_t buf_id) {
  uint16_t count;
#ifdef FSDEV_BUS_32BIT
  count = (FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr >> 16);
#else
  count = FSDEV_BTABLE->ep16[ep_id][buf_id].count;
#endif
  return count & 0x3FFU;
}

TU_ATTR_ALWAYS_INLINE static inline void btable_set_count(uint32_t ep_id, uint8_t buf_id, uint16_t byte_count) {
#ifdef FSDEV_BUS_32BIT
  uint32_t count_addr = FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr;
  count_addr = (count_addr & ~0x03FF0000u) | ((byte_count & 0x3FFu) << 16);
  FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr = count_addr;
#else
  uint16_t cnt = FSDEV_BTABLE->ep16[ep_id][buf_id].count;
  cnt = (cnt & ~0x3FFU) | (byte_count & 0x3FFU);
  FSDEV_BTABLE->ep16[ep_id][buf_id].count = cnt;
#endif
}

/* Aligned buffer size according to hardware */
TU_ATTR_ALWAYS_INLINE static inline uint16_t pma_align_buffer_size(uint16_t size, uint8_t* blsize, uint8_t* num_block) {
  /* The STM32 full speed USB peripheral supports only a limited set of
   * buffer sizes given by the RX buffer entry format in the USB_BTABLE. */
  uint16_t block_in_bytes;
  if (size > 62) {
    block_in_bytes = 32;
    *blsize = 1;
    *num_block = tu_div_ceil(size, 32);
  } else {
    block_in_bytes = 2;
    *blsize = 0;
    *num_block = tu_div_ceil(size, 2);
  }

  return (*num_block) * block_in_bytes;
}

TU_ATTR_ALWAYS_INLINE static inline void btable_set_rx_bufsize(uint32_t ep_id, uint8_t buf_id, uint16_t wCount) {
  uint8_t blsize, num_block;
  (void) pma_align_buffer_size(wCount, &blsize, &num_block);

  /* Encode into register. When BLSIZE==1, we need to subtract 1 block count */
  uint16_t bl_nb = (blsize << 15) | ((num_block - blsize) << 10);
  if (bl_nb == 0) {
    // zlp but 0 is invalid value, set blsize to 1 (32 bytes)
    // Note: lower value can cause PMAOVR on setup with ch32v203
    bl_nb = 1 << 15;
  }

#ifdef FSDEV_BUS_32BIT
  uint32_t count_addr = FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr;
  count_addr = (bl_nb << 16) | (count_addr & 0x0000FFFFu);
  FSDEV_BTABLE->ep32[ep_id][buf_id].count_addr = count_addr;
#else
  FSDEV_BTABLE->ep16[ep_id][buf_id].count = bl_nb;
#endif

}

#ifdef __cplusplus
 }
#endif

#endif
