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
#include <stdint.h>
#include "mcu/mcu.h"
#include "mcu/da1469x_hal.h"
#include <flash_map/flash_map.h>
#include <mcu/da1469x_clock.h>
#if MCUBOOT_MYNEWT
#include "bootutil/bootutil.h"
#include "bootutil/image.h"
#include "bootutil/bootutil_log.h"
#include "mcu/da1469x_dma.h"
#include "mcu/da1469x_otp.h"
#endif

#if MYNEWT_VAL(BOOT_CUSTOM_START) && MCUBOOT_MYNEWT
sec_text_ram_core
#endif
void __attribute__((naked))
hal_system_start(void *img_start)
{
    uint32_t img_data_addr;
    uint32_t *img_data;

    img_data_addr = MCU_MEM_QSPIF_M_START_ADDRESS + (uint32_t)img_start;

    assert(img_data_addr < MCU_MEM_QSPIF_M_END_ADDRESS);

    img_data = (uint32_t *)img_data_addr;

    asm volatile (".syntax unified        \n"
                  /* 1st word is stack pointer */
                  "    msr  msp, %0       \n"
                  /* 2nd word is a reset handler (image entry) */
                  "    bx   %1            \n"
                  : /* no output */
                  : "r" (img_data[0]), "r" (img_data[1]));
}

void
hal_system_restart(void *img_start)
{
    uint32_t primask __attribute__((unused));
    int i;

    /*
     * Disable interrupts, and leave them disabled.
     * They get re-enabled when system starts coming back again.
     */
    __HAL_DISABLE_INTERRUPTS(primask);

    for (i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
        NVIC->ICER[i] = 0xffffffff;
    }

    hal_system_start(img_start);
}

#if MYNEWT_VAL(BOOT_CUSTOM_START) && MCUBOOT_MYNEWT
#define IMAGE_TLV_AES_NONCE   0x50
#define IMAGE_TLV_SECRET_ID   0x60

sec_text_ram_core void
boot_custom_start(uintptr_t flash_base, struct boot_rsp *rsp)
{
    int rc;
    struct image_tlv_iter it;
    const struct flash_area *fap;
    uint32_t off;
    uint16_t len;
    uint16_t type;
    uint8_t buf[8];
    uint8_t key;
    uint32_t nonce[2];
    bool has_aes_nonce;
    bool has_secret_id;
    DMA_Type *dma_regs = DMA;
    uint32_t  jump_offset = rsp->br_image_off + rsp->br_hdr->ih_hdr_size;

    BOOT_LOG_INF("Custom initialization");

    /* skip to booting if we are running nonsecure mode */
    if (!(CRG_TOP->SECURE_BOOT_REG & 0x1)) {
        hal_system_start((void *)(flash_base + jump_offset));
    }

    rc = flash_area_open(flash_area_id_from_image_slot(0), &fap);
    assert(rc == 0);

    rc = bootutil_tlv_iter_begin(&it, rsp->br_hdr, fap, IMAGE_TLV_ANY, true);
    assert(rc == 0);

    has_aes_nonce = has_secret_id = false;
    while (true) {
        rc = bootutil_tlv_iter_next(&it, &off, &len, &type);
        assert(rc >= 0);

        if (rc > 0) {
            break;
        }

        if (type == IMAGE_TLV_AES_NONCE) {
            assert(len == 8);

            rc = flash_area_read(fap, off, buf, len);
            assert(rc == 0);

            nonce[0] = __builtin_bswap32(*(uint32_t *)buf);
            nonce[1] = __builtin_bswap32(*(uint32_t *)(buf + 4));
            has_aes_nonce = true;
        } else if (type == IMAGE_TLV_SECRET_ID) {
            assert(len == 4);

            rc = flash_area_read(fap, off, buf, len);
            assert(rc == 0);

            key = buf[0];
            has_secret_id = true;
        }
    }

    assert(has_aes_nonce && has_secret_id && key <= 7);

    /* enable OTP clock and set in read mode */
    da1469x_clock_amba_enable(CRG_TOP_CLK_AMBA_REG_OTP_ENABLE_Msk);
    da1469x_otp_set_mode(OTPC_MODE_READ);

    /* disable decrypt on the fly and program start and end addresses */
    QSPIC->QSPIC_CTR_CTRL_REG = 0;
    QSPIC->QSPIC_CTR_SADDR_REG = jump_offset;
    QSPIC->QSPIC_CTR_EADDR_REG = QSPIC->QSPIC_CTR_SADDR_REG +
                                 rsp->br_hdr->ih_img_size - 1;

    /* securely DMA hardware key from secret storage to QSPI decrypt engine */
    dma_regs->DMA_REQ_MUX_REG |= 0xf000;
    dma_regs->DMA7_LEN_REG = 8;
    dma_regs->DMA7_A_START_REG = MCU_OTPM_BASE + OTP_SEGMENT_QSPI_FW_KEYS +
                                 (32 * key);
    dma_regs->DMA7_B_START_REG = (uint32_t)&QSPIC->QSPIC_CTR_KEY_0_3_REG;
    dma_regs->DMA7_CTRL_REG = DMA_DMA7_CTRL_REG_AINC_Msk |
                              DMA_DMA7_CTRL_REG_BINC_Msk |
                              (MCU_DMA_BUS_WIDTH_4B << DMA_DMA7_CTRL_REG_BW_Pos) |
                              DMA_DMA7_CTRL_REG_DMA_ON_Msk;
    while (dma_regs->DMA7_IDX_REG != 8);

    /* program NONCE */
    QSPIC->QSPIC_CTR_NONCE_0_3_REG = nonce[0];
    QSPIC->QSPIC_CTR_NONCE_4_7_REG = nonce[1];

    /* turn back on decrypt on the fly */
    QSPIC->QSPIC_CTR_CTRL_REG = 1;

    /* set OTP to standby and turn off clock */
    da1469x_otp_set_mode(OTPC_MODE_STBY);
    da1469x_clock_amba_disable(CRG_TOP_CLK_AMBA_REG_OTP_ENABLE_Msk);

    hal_system_start((void *)(flash_base + jump_offset));
}
#endif
