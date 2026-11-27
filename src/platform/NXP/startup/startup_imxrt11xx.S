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

/*
 * Vector table and reset handler for the i.MXRT 11xx family, CM7 side; the CM4
 * stays in reset. The reset handler makes the FlexRAM split first, because the
 * initial stack pointer lies in the DTCM it creates, then branches to
 * nxpResetTail in startup_nxp_common.S.
 */

/*
 * FlexRAM split: 16 banks of 32 KB, two bits per bank (01 OCRAM, 10 DTCM,
 * 11 ITCM), banks 0..7 in GPR17 and 8..15 in GPR18. 0xffaa in both gives four
 * DTCM and four ITCM banks each, 256 KB of each. GPR16 selects the GPR17/18
 * bank layout over the fuses; the TCM sizes follow the bank counts.
 */
    .equ IOMUXC_GPR_BASE,         0x400e4000
    .equ IOMUXC_GPR_GPR16_OFFSET, 0x40
    .equ IOMUXC_GPR_GPR17_OFFSET, 0x44
    .equ IOMUXC_GPR_GPR18_OFFSET, 0x48

    .equ FLEXRAM_BANK_CFG_LOW,    0x0000ffaa
    .equ FLEXRAM_BANK_CFG_HIGH,   0x0000ffaa
    .equ GPR16_FLEXRAM_BANK_CFG_SEL, (1 << 2)

/* The TCM sizes in bytes, which the linker script checks its regions against. */
    .equ FLEXRAM_BANK_SIZE,       (32 * 1024)
    .equ FLEXRAM_ITCM_BANKS,      8
    .equ FLEXRAM_DTCM_BANKS,      8

    .global __flexram_itcm_size
    .set    __flexram_itcm_size, FLEXRAM_ITCM_BANKS * FLEXRAM_BANK_SIZE
    .global __flexram_dtcm_size
    .set    __flexram_dtcm_size, FLEXRAM_DTCM_BANKS * FLEXRAM_BANK_SIZE

/* The bank pattern must hold as many ITCM and DTCM banks as the bank counts give. */
    .set .Litcm_banks, 0
    .set .Ldtcm_banks, 0
    .irp bank, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
    .if (((FLEXRAM_BANK_CFG_LOW | (FLEXRAM_BANK_CFG_HIGH << 16)) >> (2 * \bank)) & 3) == 3
    .set .Litcm_banks, .Litcm_banks + 1
    .elseif (((FLEXRAM_BANK_CFG_LOW | (FLEXRAM_BANK_CFG_HIGH << 16)) >> (2 * \bank)) & 3) == 2
    .set .Ldtcm_banks, .Ldtcm_banks + 1
    .endif
    .endr
    .if .Litcm_banks != FLEXRAM_ITCM_BANKS || .Ldtcm_banks != FLEXRAM_DTCM_BANKS
    .error "FLEXRAM_BANK_CFG_LOW and _HIGH do not match FLEXRAM_ITCM_BANKS and _DTCM_BANKS"
    .endif

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

/* 16 core vectors and 218 external interrupts (NUMBER_OF_INT_VECTORS), in
   IRQn_Type order; the linker script checks the table against this count. */
    .global __vector_table_entries
    .set    __vector_table_entries, 234

    .section .isr_vector, "a", %progbits
    .align 2
    .globl  __Vectors
    .type   __Vectors, %object
__Vectors:
    .word   __stack_top
    .word   Reset_Handler
    .word   NMI_Handler
    .word   HardFault_Handler
    .word   MemManage_Handler
    .word   BusFault_Handler
    .word   UsageFault_Handler
    .word   0
    .word   0
    .word   0
    .word   0
    .word   SVC_Handler
    .word   DebugMon_Handler
    .word   0
    .word   PendSV_Handler
    .word   SysTick_Handler
    .word   DMA0_DMA16_IRQHandler
    .word   DMA1_DMA17_IRQHandler
    .word   DMA2_DMA18_IRQHandler
    .word   DMA3_DMA19_IRQHandler
    .word   DMA4_DMA20_IRQHandler
    .word   DMA5_DMA21_IRQHandler
    .word   DMA6_DMA22_IRQHandler
    .word   DMA7_DMA23_IRQHandler
    .word   DMA8_DMA24_IRQHandler
    .word   DMA9_DMA25_IRQHandler
    .word   DMA10_DMA26_IRQHandler
    .word   DMA11_DMA27_IRQHandler
    .word   DMA12_DMA28_IRQHandler
    .word   DMA13_DMA29_IRQHandler
    .word   DMA14_DMA30_IRQHandler
    .word   DMA15_DMA31_IRQHandler
    .word   DMA_ERROR_IRQHandler
    .word   CTI_TRIGGER_OUT0_IRQHandler
    .word   CTI_TRIGGER_OUT1_IRQHandler
    .word   CORE_IRQHandler
    .word   LPUART1_IRQHandler
    .word   LPUART2_IRQHandler
    .word   LPUART3_IRQHandler
    .word   LPUART4_IRQHandler
    .word   LPUART5_IRQHandler
    .word   LPUART6_IRQHandler
    .word   LPUART7_IRQHandler
    .word   LPUART8_IRQHandler
    .word   LPUART9_IRQHandler
    .word   LPUART10_IRQHandler
    .word   LPUART11_IRQHandler
    .word   LPUART12_IRQHandler
    .word   LPI2C1_IRQHandler
    .word   LPI2C2_IRQHandler
    .word   LPI2C3_IRQHandler
    .word   LPI2C4_IRQHandler
    .word   LPI2C5_IRQHandler
    .word   LPI2C6_IRQHandler
    .word   LPSPI1_IRQHandler
    .word   LPSPI2_IRQHandler
    .word   LPSPI3_IRQHandler
    .word   LPSPI4_IRQHandler
    .word   LPSPI5_IRQHandler
    .word   LPSPI6_IRQHandler
    .word   CAN1_IRQHandler
    .word   CAN1_ERROR_IRQHandler
    .word   CAN2_IRQHandler
    .word   CAN2_ERROR_IRQHandler
    .word   CAN3_IRQHandler
    .word   CAN3_ERROR_IRQHandler
    .word   FLEXRAM_IRQHandler
    .word   KPP_IRQHandler
    .word   Reserved68_IRQHandler
    .word   GPR_IRQ_IRQHandler
    .word   eLCDIF_IRQHandler
    .word   LCDIFv2_IRQHandler
    .word   CSI_IRQHandler
    .word   PXP_IRQHandler
    .word   MIPI_CSI_IRQHandler
    .word   MIPI_DSI_IRQHandler
    .word   GPU2D_IRQHandler
    .word   GPIO6_Combined_0_15_IRQHandler
    .word   GPIO6_Combined_16_31_IRQHandler
    .word   DAC_IRQHandler
    .word   KEY_MANAGER_IRQHandler
    .word   WDOG2_IRQHandler
    .word   SNVS_HP_NON_TZ_IRQHandler
    .word   SNVS_HP_TZ_IRQHandler
    .word   SNVS_PULSE_EVENT_IRQHandler
    .word   CAAM_IRQ0_IRQHandler
    .word   CAAM_IRQ1_IRQHandler
    .word   CAAM_IRQ2_IRQHandler
    .word   CAAM_IRQ3_IRQHandler
    .word   CAAM_RECORVE_ERRPR_IRQHandler
    .word   CAAM_RTIC_IRQHandler
    .word   CDOG_IRQHandler
    .word   SAI1_IRQHandler
    .word   SAI2_IRQHandler
    .word   SAI3_RX_IRQHandler
    .word   SAI3_TX_IRQHandler
    .word   SAI4_RX_IRQHandler
    .word   SAI4_TX_IRQHandler
    .word   SPDIF_IRQHandler
    .word   TMPSNS_INT_IRQHandler
    .word   TMPSNS_LOW_HIGH_IRQHandler
    .word   TMPSNS_PANIC_IRQHandler
    .word   LPSR_LP8_BROWNOUT_IRQHandler
    .word   Reserved103_IRQHandler
    .word   ADC1_IRQHandler
    .word   ADC2_IRQHandler
    .word   USBPHY1_IRQHandler
    .word   USBPHY2_IRQHandler
    .word   RDC_IRQHandler
    .word   GPIO13_Combined_0_31_IRQHandler
    .word   Reserved110_IRQHandler
    .word   DCIC1_IRQHandler
    .word   DCIC2_IRQHandler
    .word   ASRC_IRQHandler
    .word   FLEXRAM_ECC_IRQHandler
    .word   CM7_GPIO2_3_IRQHandler
    .word   GPIO1_Combined_0_15_IRQHandler
    .word   GPIO1_Combined_16_31_IRQHandler
    .word   GPIO2_Combined_0_15_IRQHandler
    .word   GPIO2_Combined_16_31_IRQHandler
    .word   GPIO3_Combined_0_15_IRQHandler
    .word   GPIO3_Combined_16_31_IRQHandler
    .word   GPIO4_Combined_0_15_IRQHandler
    .word   GPIO4_Combined_16_31_IRQHandler
    .word   GPIO5_Combined_0_15_IRQHandler
    .word   GPIO5_Combined_16_31_IRQHandler
    .word   FLEXIO1_IRQHandler
    .word   FLEXIO2_IRQHandler
    .word   WDOG1_IRQHandler
    .word   RTWDOG3_IRQHandler
    .word   EWM_IRQHandler
    .word   OCOTP_READ_FUSE_ERROR_IRQHandler
    .word   OCOTP_READ_DONE_ERROR_IRQHandler
    .word   GPC_IRQHandler
    .word   MUA_IRQHandler
    .word   GPT1_IRQHandler
    .word   GPT2_IRQHandler
    .word   GPT3_IRQHandler
    .word   GPT4_IRQHandler
    .word   GPT5_IRQHandler
    .word   GPT6_IRQHandler
    .word   PWM1_0_IRQHandler
    .word   PWM1_1_IRQHandler
    .word   PWM1_2_IRQHandler
    .word   PWM1_3_IRQHandler
    .word   PWM1_FAULT_IRQHandler
    .word   FLEXSPI1_IRQHandler
    .word   FLEXSPI2_IRQHandler
    .word   SEMC_IRQHandler
    .word   USDHC1_IRQHandler
    .word   USDHC2_IRQHandler
    .word   USB_OTG2_IRQHandler
    .word   USB_OTG1_IRQHandler
    .word   ENET_IRQHandler
    .word   ENET_1588_Timer_IRQHandler
    .word   ENET_1G_MAC0_Tx_Rx_1_IRQHandler
    .word   ENET_1G_MAC0_Tx_Rx_2_IRQHandler
    .word   ENET_1G_IRQHandler
    .word   ENET_1G_1588_Timer_IRQHandler
    .word   XBAR1_IRQ_0_1_IRQHandler
    .word   XBAR1_IRQ_2_3_IRQHandler
    .word   ADC_ETC_IRQ0_IRQHandler
    .word   ADC_ETC_IRQ1_IRQHandler
    .word   ADC_ETC_IRQ2_IRQHandler
    .word   ADC_ETC_IRQ3_IRQHandler
    .word   ADC_ETC_ERROR_IRQ_IRQHandler
    .word   Reserved166_IRQHandler
    .word   Reserved167_IRQHandler
    .word   Reserved168_IRQHandler
    .word   Reserved169_IRQHandler
    .word   Reserved170_IRQHandler
    .word   PIT1_IRQHandler
    .word   PIT2_IRQHandler
    .word   ACMP1_IRQHandler
    .word   ACMP2_IRQHandler
    .word   ACMP3_IRQHandler
    .word   ACMP4_IRQHandler
    .word   Reserved177_IRQHandler
    .word   Reserved178_IRQHandler
    .word   Reserved179_IRQHandler
    .word   Reserved180_IRQHandler
    .word   ENC1_IRQHandler
    .word   ENC2_IRQHandler
    .word   ENC3_IRQHandler
    .word   ENC4_IRQHandler
    .word   Reserved185_IRQHandler
    .word   Reserved186_IRQHandler
    .word   TMR1_IRQHandler
    .word   TMR2_IRQHandler
    .word   TMR3_IRQHandler
    .word   TMR4_IRQHandler
    .word   SEMA4_CP0_IRQHandler
    .word   SEMA4_CP1_IRQHandler
    .word   PWM2_0_IRQHandler
    .word   PWM2_1_IRQHandler
    .word   PWM2_2_IRQHandler
    .word   PWM2_3_IRQHandler
    .word   PWM2_FAULT_IRQHandler
    .word   PWM3_0_IRQHandler
    .word   PWM3_1_IRQHandler
    .word   PWM3_2_IRQHandler
    .word   PWM3_3_IRQHandler
    .word   PWM3_FAULT_IRQHandler
    .word   PWM4_0_IRQHandler
    .word   PWM4_1_IRQHandler
    .word   PWM4_2_IRQHandler
    .word   PWM4_3_IRQHandler
    .word   PWM4_FAULT_IRQHandler
    .word   Reserved208_IRQHandler
    .word   Reserved209_IRQHandler
    .word   Reserved210_IRQHandler
    .word   Reserved211_IRQHandler
    .word   Reserved212_IRQHandler
    .word   Reserved213_IRQHandler
    .word   Reserved214_IRQHandler
    .word   Reserved215_IRQHandler
    .word   PDM_HWVAD_EVENT_IRQHandler
    .word   PDM_HWVAD_ERROR_IRQHandler
    .word   PDM_EVENT_IRQHandler
    .word   PDM_ERROR_IRQHandler
    .word   EMVSIM1_IRQHandler
    .word   EMVSIM2_IRQHandler
    .word   MECC1_INT_IRQHandler
    .word   MECC1_FATAL_INT_IRQHandler
    .word   MECC2_INT_IRQHandler
    .word   MECC2_FATAL_INT_IRQHandler
    .word   XECC_FLEXSPI1_INT_IRQHandler
    .word   XECC_FLEXSPI1_FATAL_INT_IRQHandler
    .word   XECC_FLEXSPI2_INT_IRQHandler
    .word   XECC_FLEXSPI2_FATAL_INT_IRQHandler
    .word   XECC_SEMC_INT_IRQHandler
    .word   XECC_SEMC_FATAL_INT_IRQHandler
    .word   ENET_QOS_IRQHandler
    .word   ENET_QOS_PMT_IRQHandler
    .size   __Vectors, . - __Vectors

    .section .text.Reset_Handler, "ax", %progbits
    .align 2
    .thumb_func
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    cpsid   i

    /* Split FlexRAM before anything touches RAM; no stack exists yet. */
    ldr     r0, =IOMUXC_GPR_BASE

    ldr     r1, =FLEXRAM_BANK_CFG_LOW
    str     r1, [r0, #IOMUXC_GPR_GPR17_OFFSET]
    ldr     r1, =FLEXRAM_BANK_CFG_HIGH
    str     r1, [r0, #IOMUXC_GPR_GPR18_OFFSET]

    ldr     r1, [r0, #IOMUXC_GPR_GPR16_OFFSET]
    orr     r1, r1, #GPR16_FLEXRAM_BANK_CFG_SEL
    str     r1, [r0, #IOMUXC_GPR_GPR16_OFFSET]
    dsb
    isb

    b       nxpResetTail
    .size   Reset_Handler, . - Reset_Handler

/*
 * Weak default handlers branch to systemFaultAction(). They live in ITCM so a fault during
 * a flash write does not fetch from flash; the section name must not start with .text.
 */
    .section .itcm_text.defaultHandler, "ax", %progbits
    .align 2
    .thumb_func
    .weak   defaultHandler
    .type   defaultHandler, %function
defaultHandler:
    b       systemFaultAction
    .size   defaultHandler, . - defaultHandler

    .macro def_default_handler handler_name
    .weak   \handler_name
    .set    \handler_name, defaultHandler
    .endm

    def_default_handler NMI_Handler
    def_default_handler HardFault_Handler
    def_default_handler MemManage_Handler
    def_default_handler BusFault_Handler
    def_default_handler UsageFault_Handler
    def_default_handler SVC_Handler
    def_default_handler DebugMon_Handler
    def_default_handler PendSV_Handler
    def_default_handler SysTick_Handler

    def_default_handler DMA0_DMA16_IRQHandler
    def_default_handler DMA1_DMA17_IRQHandler
    def_default_handler DMA2_DMA18_IRQHandler
    def_default_handler DMA3_DMA19_IRQHandler
    def_default_handler DMA4_DMA20_IRQHandler
    def_default_handler DMA5_DMA21_IRQHandler
    def_default_handler DMA6_DMA22_IRQHandler
    def_default_handler DMA7_DMA23_IRQHandler
    def_default_handler DMA8_DMA24_IRQHandler
    def_default_handler DMA9_DMA25_IRQHandler
    def_default_handler DMA10_DMA26_IRQHandler
    def_default_handler DMA11_DMA27_IRQHandler
    def_default_handler DMA12_DMA28_IRQHandler
    def_default_handler DMA13_DMA29_IRQHandler
    def_default_handler DMA14_DMA30_IRQHandler
    def_default_handler DMA15_DMA31_IRQHandler
    def_default_handler DMA_ERROR_IRQHandler
    def_default_handler CTI_TRIGGER_OUT0_IRQHandler
    def_default_handler CTI_TRIGGER_OUT1_IRQHandler
    def_default_handler CORE_IRQHandler
    def_default_handler LPUART1_IRQHandler
    def_default_handler LPUART2_IRQHandler
    def_default_handler LPUART3_IRQHandler
    def_default_handler LPUART4_IRQHandler
    def_default_handler LPUART5_IRQHandler
    def_default_handler LPUART6_IRQHandler
    def_default_handler LPUART7_IRQHandler
    def_default_handler LPUART8_IRQHandler
    def_default_handler LPUART9_IRQHandler
    def_default_handler LPUART10_IRQHandler
    def_default_handler LPUART11_IRQHandler
    def_default_handler LPUART12_IRQHandler
    def_default_handler LPI2C1_IRQHandler
    def_default_handler LPI2C2_IRQHandler
    def_default_handler LPI2C3_IRQHandler
    def_default_handler LPI2C4_IRQHandler
    def_default_handler LPI2C5_IRQHandler
    def_default_handler LPI2C6_IRQHandler
    def_default_handler LPSPI1_IRQHandler
    def_default_handler LPSPI2_IRQHandler
    def_default_handler LPSPI3_IRQHandler
    def_default_handler LPSPI4_IRQHandler
    def_default_handler LPSPI5_IRQHandler
    def_default_handler LPSPI6_IRQHandler
    def_default_handler CAN1_IRQHandler
    def_default_handler CAN1_ERROR_IRQHandler
    def_default_handler CAN2_IRQHandler
    def_default_handler CAN2_ERROR_IRQHandler
    def_default_handler CAN3_IRQHandler
    def_default_handler CAN3_ERROR_IRQHandler
    def_default_handler FLEXRAM_IRQHandler
    def_default_handler KPP_IRQHandler
    def_default_handler Reserved68_IRQHandler
    def_default_handler GPR_IRQ_IRQHandler
    def_default_handler eLCDIF_IRQHandler
    def_default_handler LCDIFv2_IRQHandler
    def_default_handler CSI_IRQHandler
    def_default_handler PXP_IRQHandler
    def_default_handler MIPI_CSI_IRQHandler
    def_default_handler MIPI_DSI_IRQHandler
    def_default_handler GPU2D_IRQHandler
    def_default_handler GPIO6_Combined_0_15_IRQHandler
    def_default_handler GPIO6_Combined_16_31_IRQHandler
    def_default_handler DAC_IRQHandler
    def_default_handler KEY_MANAGER_IRQHandler
    def_default_handler WDOG2_IRQHandler
    def_default_handler SNVS_HP_NON_TZ_IRQHandler
    def_default_handler SNVS_HP_TZ_IRQHandler
    def_default_handler SNVS_PULSE_EVENT_IRQHandler
    def_default_handler CAAM_IRQ0_IRQHandler
    def_default_handler CAAM_IRQ1_IRQHandler
    def_default_handler CAAM_IRQ2_IRQHandler
    def_default_handler CAAM_IRQ3_IRQHandler
    def_default_handler CAAM_RECORVE_ERRPR_IRQHandler
    def_default_handler CAAM_RTIC_IRQHandler
    def_default_handler CDOG_IRQHandler
    def_default_handler SAI1_IRQHandler
    def_default_handler SAI2_IRQHandler
    def_default_handler SAI3_RX_IRQHandler
    def_default_handler SAI3_TX_IRQHandler
    def_default_handler SAI4_RX_IRQHandler
    def_default_handler SAI4_TX_IRQHandler
    def_default_handler SPDIF_IRQHandler
    def_default_handler TMPSNS_INT_IRQHandler
    def_default_handler TMPSNS_LOW_HIGH_IRQHandler
    def_default_handler TMPSNS_PANIC_IRQHandler
    def_default_handler LPSR_LP8_BROWNOUT_IRQHandler
    def_default_handler Reserved103_IRQHandler
    def_default_handler ADC1_IRQHandler
    def_default_handler ADC2_IRQHandler
    def_default_handler USBPHY1_IRQHandler
    def_default_handler USBPHY2_IRQHandler
    def_default_handler RDC_IRQHandler
    def_default_handler GPIO13_Combined_0_31_IRQHandler
    def_default_handler Reserved110_IRQHandler
    def_default_handler DCIC1_IRQHandler
    def_default_handler DCIC2_IRQHandler
    def_default_handler ASRC_IRQHandler
    def_default_handler FLEXRAM_ECC_IRQHandler
    def_default_handler CM7_GPIO2_3_IRQHandler
    def_default_handler GPIO1_Combined_0_15_IRQHandler
    def_default_handler GPIO1_Combined_16_31_IRQHandler
    def_default_handler GPIO2_Combined_0_15_IRQHandler
    def_default_handler GPIO2_Combined_16_31_IRQHandler
    def_default_handler GPIO3_Combined_0_15_IRQHandler
    def_default_handler GPIO3_Combined_16_31_IRQHandler
    def_default_handler GPIO4_Combined_0_15_IRQHandler
    def_default_handler GPIO4_Combined_16_31_IRQHandler
    def_default_handler GPIO5_Combined_0_15_IRQHandler
    def_default_handler GPIO5_Combined_16_31_IRQHandler
    def_default_handler FLEXIO1_IRQHandler
    def_default_handler FLEXIO2_IRQHandler
    def_default_handler WDOG1_IRQHandler
    def_default_handler RTWDOG3_IRQHandler
    def_default_handler EWM_IRQHandler
    def_default_handler OCOTP_READ_FUSE_ERROR_IRQHandler
    def_default_handler OCOTP_READ_DONE_ERROR_IRQHandler
    def_default_handler GPC_IRQHandler
    def_default_handler MUA_IRQHandler
    def_default_handler GPT1_IRQHandler
    def_default_handler GPT2_IRQHandler
    def_default_handler GPT3_IRQHandler
    def_default_handler GPT4_IRQHandler
    def_default_handler GPT5_IRQHandler
    def_default_handler GPT6_IRQHandler
    def_default_handler PWM1_0_IRQHandler
    def_default_handler PWM1_1_IRQHandler
    def_default_handler PWM1_2_IRQHandler
    def_default_handler PWM1_3_IRQHandler
    def_default_handler PWM1_FAULT_IRQHandler
    def_default_handler FLEXSPI1_IRQHandler
    def_default_handler FLEXSPI2_IRQHandler
    def_default_handler SEMC_IRQHandler
    def_default_handler USDHC1_IRQHandler
    def_default_handler USDHC2_IRQHandler
    def_default_handler USB_OTG2_IRQHandler
    def_default_handler USB_OTG1_IRQHandler
    def_default_handler ENET_IRQHandler
    def_default_handler ENET_1588_Timer_IRQHandler
    def_default_handler ENET_1G_MAC0_Tx_Rx_1_IRQHandler
    def_default_handler ENET_1G_MAC0_Tx_Rx_2_IRQHandler
    def_default_handler ENET_1G_IRQHandler
    def_default_handler ENET_1G_1588_Timer_IRQHandler
    def_default_handler XBAR1_IRQ_0_1_IRQHandler
    def_default_handler XBAR1_IRQ_2_3_IRQHandler
    def_default_handler ADC_ETC_IRQ0_IRQHandler
    def_default_handler ADC_ETC_IRQ1_IRQHandler
    def_default_handler ADC_ETC_IRQ2_IRQHandler
    def_default_handler ADC_ETC_IRQ3_IRQHandler
    def_default_handler ADC_ETC_ERROR_IRQ_IRQHandler
    def_default_handler Reserved166_IRQHandler
    def_default_handler Reserved167_IRQHandler
    def_default_handler Reserved168_IRQHandler
    def_default_handler Reserved169_IRQHandler
    def_default_handler Reserved170_IRQHandler
    def_default_handler PIT1_IRQHandler
    def_default_handler PIT2_IRQHandler
    def_default_handler ACMP1_IRQHandler
    def_default_handler ACMP2_IRQHandler
    def_default_handler ACMP3_IRQHandler
    def_default_handler ACMP4_IRQHandler
    def_default_handler Reserved177_IRQHandler
    def_default_handler Reserved178_IRQHandler
    def_default_handler Reserved179_IRQHandler
    def_default_handler Reserved180_IRQHandler
    def_default_handler ENC1_IRQHandler
    def_default_handler ENC2_IRQHandler
    def_default_handler ENC3_IRQHandler
    def_default_handler ENC4_IRQHandler
    def_default_handler Reserved185_IRQHandler
    def_default_handler Reserved186_IRQHandler
    def_default_handler TMR1_IRQHandler
    def_default_handler TMR2_IRQHandler
    def_default_handler TMR3_IRQHandler
    def_default_handler TMR4_IRQHandler
    def_default_handler SEMA4_CP0_IRQHandler
    def_default_handler SEMA4_CP1_IRQHandler
    def_default_handler PWM2_0_IRQHandler
    def_default_handler PWM2_1_IRQHandler
    def_default_handler PWM2_2_IRQHandler
    def_default_handler PWM2_3_IRQHandler
    def_default_handler PWM2_FAULT_IRQHandler
    def_default_handler PWM3_0_IRQHandler
    def_default_handler PWM3_1_IRQHandler
    def_default_handler PWM3_2_IRQHandler
    def_default_handler PWM3_3_IRQHandler
    def_default_handler PWM3_FAULT_IRQHandler
    def_default_handler PWM4_0_IRQHandler
    def_default_handler PWM4_1_IRQHandler
    def_default_handler PWM4_2_IRQHandler
    def_default_handler PWM4_3_IRQHandler
    def_default_handler PWM4_FAULT_IRQHandler
    def_default_handler Reserved208_IRQHandler
    def_default_handler Reserved209_IRQHandler
    def_default_handler Reserved210_IRQHandler
    def_default_handler Reserved211_IRQHandler
    def_default_handler Reserved212_IRQHandler
    def_default_handler Reserved213_IRQHandler
    def_default_handler Reserved214_IRQHandler
    def_default_handler Reserved215_IRQHandler
    def_default_handler PDM_HWVAD_EVENT_IRQHandler
    def_default_handler PDM_HWVAD_ERROR_IRQHandler
    def_default_handler PDM_EVENT_IRQHandler
    def_default_handler PDM_ERROR_IRQHandler
    def_default_handler EMVSIM1_IRQHandler
    def_default_handler EMVSIM2_IRQHandler
    def_default_handler MECC1_INT_IRQHandler
    def_default_handler MECC1_FATAL_INT_IRQHandler
    def_default_handler MECC2_INT_IRQHandler
    def_default_handler MECC2_FATAL_INT_IRQHandler
    def_default_handler XECC_FLEXSPI1_INT_IRQHandler
    def_default_handler XECC_FLEXSPI1_FATAL_INT_IRQHandler
    def_default_handler XECC_FLEXSPI2_INT_IRQHandler
    def_default_handler XECC_FLEXSPI2_FATAL_INT_IRQHandler
    def_default_handler XECC_SEMC_INT_IRQHandler
    def_default_handler XECC_SEMC_FATAL_INT_IRQHandler
    def_default_handler ENET_QOS_IRQHandler
    def_default_handler ENET_QOS_PMT_IRQHandler
