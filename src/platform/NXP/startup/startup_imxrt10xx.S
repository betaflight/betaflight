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
 * Vector table and reset handler for the i.MXRT 10xx family. The reset handler
 * makes the FlexRAM split first, because the initial stack pointer lies in the
 * DTCM it creates, then branches to nxpResetTail in startup_nxp_common.S.
 */

/*
 * FlexRAM split: 16 banks of 32 KB. IOMUXC_GPR17 holds two bits per bank
 * (01 OCRAM, 10 DTCM, 11 ITCM); banks 0..7 are ITCM and 8..15 DTCM, so
 * 0xaaaaffff. GPR14 sizes are log2(size in KB) + 1 (256 KB is 9) in bits
 * 19:16 (ITCM) and 23:20 (DTCM). GPR16 enables the TCMs and selects the GPR17
 * bank layout over the fuses.
 */
    .equ IOMUXC_GPR_BASE,        0x400ac000
    .equ IOMUXC_GPR_GPR14_OFFSET, 0x38
    .equ IOMUXC_GPR_GPR16_OFFSET, 0x40
    .equ IOMUXC_GPR_GPR17_OFFSET, 0x44

    .equ FLEXRAM_BANK_CFG,       0xaaaaffff
    .equ FLEXRAM_TCM_SIZE_256KB, 9
    .equ GPR14_TCM_SIZES,        (FLEXRAM_TCM_SIZE_256KB << 16) | (FLEXRAM_TCM_SIZE_256KB << 20)
    .equ GPR16_INIT_ITCM_EN,     (1 << 0)
    .equ GPR16_INIT_DTCM_EN,     (1 << 1)
    .equ GPR16_FLEXRAM_BANK_CFG_SEL, (1 << 2)

/* The TCM sizes in bytes, which the linker script checks its regions against. */
    .global __flexram_itcm_size
    .set    __flexram_itcm_size, 1 << (FLEXRAM_TCM_SIZE_256KB - 1 + 10)
    .global __flexram_dtcm_size
    .set    __flexram_dtcm_size, 1 << (FLEXRAM_TCM_SIZE_256KB - 1 + 10)

/* The bank pattern must hold as many ITCM and DTCM banks as the size codes give. */
    .set .Litcm_banks, 0
    .set .Ldtcm_banks, 0
    .irp bank, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
    .if ((FLEXRAM_BANK_CFG >> (2 * \bank)) & 3) == 3
    .set .Litcm_banks, .Litcm_banks + 1
    .elseif ((FLEXRAM_BANK_CFG >> (2 * \bank)) & 3) == 2
    .set .Ldtcm_banks, .Ldtcm_banks + 1
    .endif
    .endr
    .if .Litcm_banks * 32768 != __flexram_itcm_size || .Ldtcm_banks * 32768 != __flexram_dtcm_size
    .error "FLEXRAM_BANK_CFG does not match FLEXRAM_TCM_SIZE_256KB"
    .endif

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

/* 16 core vectors and 158 external interrupts (NUMBER_OF_INT_VECTORS), in
   IRQn_Type order; the linker script checks the table against this count. */
    .global __vector_table_entries
    .set    __vector_table_entries, 174

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
    .word   CTI0_ERROR_IRQHandler
    .word   CTI1_ERROR_IRQHandler
    .word   CORE_IRQHandler
    .word   LPUART1_IRQHandler
    .word   LPUART2_IRQHandler
    .word   LPUART3_IRQHandler
    .word   LPUART4_IRQHandler
    .word   LPUART5_IRQHandler
    .word   LPUART6_IRQHandler
    .word   LPUART7_IRQHandler
    .word   LPUART8_IRQHandler
    .word   LPI2C1_IRQHandler
    .word   LPI2C2_IRQHandler
    .word   LPI2C3_IRQHandler
    .word   LPI2C4_IRQHandler
    .word   LPSPI1_IRQHandler
    .word   LPSPI2_IRQHandler
    .word   LPSPI3_IRQHandler
    .word   LPSPI4_IRQHandler
    .word   CAN1_IRQHandler
    .word   CAN2_IRQHandler
    .word   FLEXRAM_IRQHandler
    .word   KPP_IRQHandler
    .word   TSC_DIG_IRQHandler
    .word   GPR_IRQ_IRQHandler
    .word   LCDIF_IRQHandler
    .word   CSI_IRQHandler
    .word   PXP_IRQHandler
    .word   WDOG2_IRQHandler
    .word   SNVS_HP_WRAPPER_IRQHandler
    .word   SNVS_HP_WRAPPER_TZ_IRQHandler
    .word   SNVS_LP_WRAPPER_IRQHandler
    .word   CSU_IRQHandler
    .word   DCP_IRQHandler
    .word   DCP_VMI_IRQHandler
    .word   Reserved68_IRQHandler
    .word   TRNG_IRQHandler
    .word   SJC_IRQHandler
    .word   BEE_IRQHandler
    .word   SAI1_IRQHandler
    .word   SAI2_IRQHandler
    .word   SAI3_RX_IRQHandler
    .word   SAI3_TX_IRQHandler
    .word   SPDIF_IRQHandler
    .word   PMU_EVENT_IRQHandler
    .word   Reserved78_IRQHandler
    .word   TEMP_LOW_HIGH_IRQHandler
    .word   TEMP_PANIC_IRQHandler
    .word   USB_PHY1_IRQHandler
    .word   USB_PHY2_IRQHandler
    .word   ADC1_IRQHandler
    .word   ADC2_IRQHandler
    .word   DCDC_IRQHandler
    .word   Reserved86_IRQHandler
    .word   GPIO10_Combined_0_31_IRQHandler
    .word   GPIO1_INT0_IRQHandler
    .word   GPIO1_INT1_IRQHandler
    .word   GPIO1_INT2_IRQHandler
    .word   GPIO1_INT3_IRQHandler
    .word   GPIO1_INT4_IRQHandler
    .word   GPIO1_INT5_IRQHandler
    .word   GPIO1_INT6_IRQHandler
    .word   GPIO1_INT7_IRQHandler
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
    .word   RTWDOG_IRQHandler
    .word   EWM_IRQHandler
    .word   CCM_1_IRQHandler
    .word   CCM_2_IRQHandler
    .word   GPC_IRQHandler
    .word   SRC_IRQHandler
    .word   Reserved115_IRQHandler
    .word   GPT1_IRQHandler
    .word   GPT2_IRQHandler
    .word   PWM1_0_IRQHandler
    .word   PWM1_1_IRQHandler
    .word   PWM1_2_IRQHandler
    .word   PWM1_3_IRQHandler
    .word   PWM1_FAULT_IRQHandler
    .word   FLEXSPI2_IRQHandler
    .word   FLEXSPI_IRQHandler
    .word   SEMC_IRQHandler
    .word   USDHC1_IRQHandler
    .word   USDHC2_IRQHandler
    .word   USB_OTG2_IRQHandler
    .word   USB_OTG1_IRQHandler
    .word   ENET_IRQHandler
    .word   ENET_1588_Timer_IRQHandler
    .word   XBAR1_IRQ_0_1_IRQHandler
    .word   XBAR1_IRQ_2_3_IRQHandler
    .word   ADC_ETC_IRQ0_IRQHandler
    .word   ADC_ETC_IRQ1_IRQHandler
    .word   ADC_ETC_IRQ2_IRQHandler
    .word   ADC_ETC_ERROR_IRQ_IRQHandler
    .word   PIT_IRQHandler
    .word   ACMP1_IRQHandler
    .word   ACMP2_IRQHandler
    .word   ACMP3_IRQHandler
    .word   ACMP4_IRQHandler
    .word   Reserved143_IRQHandler
    .word   Reserved144_IRQHandler
    .word   ENC1_IRQHandler
    .word   ENC2_IRQHandler
    .word   ENC3_IRQHandler
    .word   ENC4_IRQHandler
    .word   TMR1_IRQHandler
    .word   TMR2_IRQHandler
    .word   TMR3_IRQHandler
    .word   TMR4_IRQHandler
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
    .word   ENET2_IRQHandler
    .word   ENET2_1588_Timer_IRQHandler
    .word   CAN3_IRQHandler
    .word   Reserved171_IRQHandler
    .word   FLEXIO3_IRQHandler
    .word   GPIO6_7_8_9_IRQHandler
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

    ldr     r1, =FLEXRAM_BANK_CFG
    str     r1, [r0, #IOMUXC_GPR_GPR17_OFFSET]

    ldr     r1, [r0, #IOMUXC_GPR_GPR14_OFFSET]
    bic     r1, r1, #0x00ff0000             /* clear both TCM size fields */
    ldr     r2, =GPR14_TCM_SIZES
    orr     r1, r1, r2
    str     r1, [r0, #IOMUXC_GPR_GPR14_OFFSET]

    ldr     r1, [r0, #IOMUXC_GPR_GPR16_OFFSET]
    orr     r1, r1, #(GPR16_INIT_ITCM_EN | GPR16_INIT_DTCM_EN | GPR16_FLEXRAM_BANK_CFG_SEL)
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
    def_default_handler CTI0_ERROR_IRQHandler
    def_default_handler CTI1_ERROR_IRQHandler
    def_default_handler CORE_IRQHandler
    def_default_handler LPUART1_IRQHandler
    def_default_handler LPUART2_IRQHandler
    def_default_handler LPUART3_IRQHandler
    def_default_handler LPUART4_IRQHandler
    def_default_handler LPUART5_IRQHandler
    def_default_handler LPUART6_IRQHandler
    def_default_handler LPUART7_IRQHandler
    def_default_handler LPUART8_IRQHandler
    def_default_handler LPI2C1_IRQHandler
    def_default_handler LPI2C2_IRQHandler
    def_default_handler LPI2C3_IRQHandler
    def_default_handler LPI2C4_IRQHandler
    def_default_handler LPSPI1_IRQHandler
    def_default_handler LPSPI2_IRQHandler
    def_default_handler LPSPI3_IRQHandler
    def_default_handler LPSPI4_IRQHandler
    def_default_handler CAN1_IRQHandler
    def_default_handler CAN2_IRQHandler
    def_default_handler FLEXRAM_IRQHandler
    def_default_handler KPP_IRQHandler
    def_default_handler TSC_DIG_IRQHandler
    def_default_handler GPR_IRQ_IRQHandler
    def_default_handler LCDIF_IRQHandler
    def_default_handler CSI_IRQHandler
    def_default_handler PXP_IRQHandler
    def_default_handler WDOG2_IRQHandler
    def_default_handler SNVS_HP_WRAPPER_IRQHandler
    def_default_handler SNVS_HP_WRAPPER_TZ_IRQHandler
    def_default_handler SNVS_LP_WRAPPER_IRQHandler
    def_default_handler CSU_IRQHandler
    def_default_handler DCP_IRQHandler
    def_default_handler DCP_VMI_IRQHandler
    def_default_handler Reserved68_IRQHandler
    def_default_handler TRNG_IRQHandler
    def_default_handler SJC_IRQHandler
    def_default_handler BEE_IRQHandler
    def_default_handler SAI1_IRQHandler
    def_default_handler SAI2_IRQHandler
    def_default_handler SAI3_RX_IRQHandler
    def_default_handler SAI3_TX_IRQHandler
    def_default_handler SPDIF_IRQHandler
    def_default_handler PMU_EVENT_IRQHandler
    def_default_handler Reserved78_IRQHandler
    def_default_handler TEMP_LOW_HIGH_IRQHandler
    def_default_handler TEMP_PANIC_IRQHandler
    def_default_handler USB_PHY1_IRQHandler
    def_default_handler USB_PHY2_IRQHandler
    def_default_handler ADC1_IRQHandler
    def_default_handler ADC2_IRQHandler
    def_default_handler DCDC_IRQHandler
    def_default_handler Reserved86_IRQHandler
    def_default_handler GPIO10_Combined_0_31_IRQHandler
    def_default_handler GPIO1_INT0_IRQHandler
    def_default_handler GPIO1_INT1_IRQHandler
    def_default_handler GPIO1_INT2_IRQHandler
    def_default_handler GPIO1_INT3_IRQHandler
    def_default_handler GPIO1_INT4_IRQHandler
    def_default_handler GPIO1_INT5_IRQHandler
    def_default_handler GPIO1_INT6_IRQHandler
    def_default_handler GPIO1_INT7_IRQHandler
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
    def_default_handler RTWDOG_IRQHandler
    def_default_handler EWM_IRQHandler
    def_default_handler CCM_1_IRQHandler
    def_default_handler CCM_2_IRQHandler
    def_default_handler GPC_IRQHandler
    def_default_handler SRC_IRQHandler
    def_default_handler Reserved115_IRQHandler
    def_default_handler GPT1_IRQHandler
    def_default_handler GPT2_IRQHandler
    def_default_handler PWM1_0_IRQHandler
    def_default_handler PWM1_1_IRQHandler
    def_default_handler PWM1_2_IRQHandler
    def_default_handler PWM1_3_IRQHandler
    def_default_handler PWM1_FAULT_IRQHandler
    def_default_handler FLEXSPI2_IRQHandler
    def_default_handler FLEXSPI_IRQHandler
    def_default_handler SEMC_IRQHandler
    def_default_handler USDHC1_IRQHandler
    def_default_handler USDHC2_IRQHandler
    def_default_handler USB_OTG2_IRQHandler
    def_default_handler USB_OTG1_IRQHandler
    def_default_handler ENET_IRQHandler
    def_default_handler ENET_1588_Timer_IRQHandler
    def_default_handler XBAR1_IRQ_0_1_IRQHandler
    def_default_handler XBAR1_IRQ_2_3_IRQHandler
    def_default_handler ADC_ETC_IRQ0_IRQHandler
    def_default_handler ADC_ETC_IRQ1_IRQHandler
    def_default_handler ADC_ETC_IRQ2_IRQHandler
    def_default_handler ADC_ETC_ERROR_IRQ_IRQHandler
    def_default_handler PIT_IRQHandler
    def_default_handler ACMP1_IRQHandler
    def_default_handler ACMP2_IRQHandler
    def_default_handler ACMP3_IRQHandler
    def_default_handler ACMP4_IRQHandler
    def_default_handler Reserved143_IRQHandler
    def_default_handler Reserved144_IRQHandler
    def_default_handler ENC1_IRQHandler
    def_default_handler ENC2_IRQHandler
    def_default_handler ENC3_IRQHandler
    def_default_handler ENC4_IRQHandler
    def_default_handler TMR1_IRQHandler
    def_default_handler TMR2_IRQHandler
    def_default_handler TMR3_IRQHandler
    def_default_handler TMR4_IRQHandler
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
    def_default_handler ENET2_IRQHandler
    def_default_handler ENET2_1588_Timer_IRQHandler
    def_default_handler CAN3_IRQHandler
    def_default_handler Reserved171_IRQHandler
    def_default_handler FLEXIO3_IRQHandler
    def_default_handler GPIO6_7_8_9_IRQHandler
