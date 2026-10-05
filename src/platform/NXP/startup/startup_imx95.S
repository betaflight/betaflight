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
 * Vector table and reset handler for the i.MX95 Cortex-M7. mcuboot enters with its VTOR, MPU and caches
 * still set up, so the reset handler moves VTOR to flash, turns caches, MPU and tick off, stops owned
 * eDMA and motor outputs, masks every interrupt and seeds both TCMs for ECC before nxpResetTail.
 */

    .equ CM7_VTOR,              0xe000ed08
    .equ CM7_MPU_CTRL,          0xe000ed94
    .equ CM7_CCR,               0xe000ed14
    .equ CM7_CCR_CACHES,        (1 << 16) | (1 << 17)
    .equ CM7_SYST_CSR,          0xe000e010
    .equ CM7_NVIC_ICER0,        0xe000e180
    .equ CM7_NVIC_ICPR0,        0xe000e280
    .equ CM7_NVIC_REGISTERS,    8

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

/* 16 core vectors and 240 external interrupts; the HAL's NUMBER_OF_INT_VECTORS also counts IRQSTEER inputs. */
    .global __vector_table_entries
    .set    __vector_table_entries, 256

/* mcuboot starts the image from the first two words; the core vectors serve until the full table is in ITCM. */
    .section .boot_vector, "a", %progbits
    .align 2
    .globl  __bootVectors
    .type   __bootVectors, %object
__bootVectors:
    .word   __stack_top
    .word   Reset_Handler
    .rept   14
    .word   earlyFault
    .endr
    .size   __bootVectors, . - __bootVectors

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
    .word   Reserved16_IRQHandler
    .word   Reserved17_IRQHandler
    .word   Reserved18_IRQHandler
    .word   Reserved19_IRQHandler
    .word   Reserved20_IRQHandler
    .word   Reserved21_IRQHandler
    .word   Reserved22_IRQHandler
    .word   Reserved23_IRQHandler
    .word   CAN1_IRQHandler
    .word   CAN1_ERROR_IRQHandler
    .word   GPIO1_0_IRQHandler
    .word   GPIO1_1_IRQHandler
    .word   I3C1_IRQHandler
    .word   LPI2C1_IRQHandler
    .word   LPI2C2_IRQHandler
    .word   LPIT1_IRQHandler
    .word   LPSPI1_IRQHandler
    .word   LPSPI2_IRQHandler
    .word   LPTMR1_IRQHandler
    .word   LPUART1_IRQHandler
    .word   LPUART2_IRQHandler
    .word   Reserved37_IRQHandler
    .word   Reserved38_IRQHandler
    .word   Reserved39_IRQHandler
    .word   Reserved40_IRQHandler
    .word   Reserved41_IRQHandler
    .word   Reserved42_IRQHandler
    .word   V2X_FH_APCH0_IRQHandler
    .word   V2X_FH_APHSM1_IRQHandler
    .word   TPM1_IRQHandler
    .word   TPM2_IRQHandler
    .word   WDOG1_IRQHandler
    .word   WDOG2_IRQHandler
    .word   TRDC_MGR_A_IRQHandler
    .word   SAI1_IRQHandler
    .word   Reserved51_IRQHandler
    .word   Reserved52_IRQHandler
    .word   Reserved53_IRQHandler
    .word   CAN2_IRQHandler
    .word   CAN2_ERROR_IRQHandler
    .word   CAN3_IRQHandler
    .word   CAN3_ERROR_IRQHandler
    .word   CAN4_IRQHandler
    .word   CAN4_ERROR_IRQHandler
    .word   CAN5_IRQHandler
    .word   CAN5_ERROR_IRQHandler
    .word   FLEXIO1_IRQHandler
    .word   FLEXIO2_IRQHandler
    .word   FlexSPI1_IRQHandler
    .word   GPIO2_0_IRQHandler
    .word   GPIO2_1_IRQHandler
    .word   GPIO3_0_IRQHandler
    .word   GPIO3_1_IRQHandler
    .word   GPIO4_0_IRQHandler
    .word   GPIO4_1_IRQHandler
    .word   GPIO5_0_IRQHandler
    .word   GPIO5_1_IRQHandler
    .word   I3C2_IRQHandler
    .word   LPI2C3_IRQHandler
    .word   LPI2C4_IRQHandler
    .word   LPIT2_IRQHandler
    .word   LPSPI3_IRQHandler
    .word   LPSPI4_IRQHandler
    .word   LPTMR2_IRQHandler
    .word   LPUART3_IRQHandler
    .word   LPUART4_IRQHandler
    .word   LPUART5_IRQHandler
    .word   LPUART6_IRQHandler
    .word   LPUART7_IRQHandler
    .word   LPUART8_IRQHandler
    .word   Reserved86_IRQHandler
    .word   Reserved87_IRQHandler
    .word   Reserved88_IRQHandler
    .word   TPM3_IRQHandler
    .word   TPM4_IRQHandler
    .word   TPM5_IRQHandler
    .word   TPM6_IRQHandler
    .word   WDOG3_IRQHandler
    .word   WDOG4_IRQHandler
    .word   WDOG5_IRQHandler
    .word   TMPSNS1_THR1_IRQHandler
    .word   TMPSNS1_THR2_IRQHandler
    .word   TMPSNS1_DRDY_IRQHandler
    .word   TMPSNS2_THR1_IRQHandler
    .word   TMPSNS2_THR2_IRQHandler
    .word   TMPSNS2_DRDY_IRQHandler
    .word   uSDHC1_IRQHandler
    .word   uSDHC2_IRQHandler
    .word   Reserved104_IRQHandler
    .word   Reserved105_IRQHandler
    .word   Reserved106_IRQHandler
    .word   Reserved107_IRQHandler
    .word   Reserved108_IRQHandler
    .word   Reserved109_IRQHandler
    .word   Reserved110_IRQHandler
    .word   DMA3_ERROR_IRQHandler
    .word   DMA3_0_IRQHandler
    .word   DMA3_1_IRQHandler
    .word   DMA3_2_IRQHandler
    .word   DMA3_3_IRQHandler
    .word   DMA3_4_IRQHandler
    .word   DMA3_5_IRQHandler
    .word   DMA3_6_IRQHandler
    .word   DMA3_7_IRQHandler
    .word   DMA3_8_IRQHandler
    .word   DMA3_9_IRQHandler
    .word   DMA3_10_IRQHandler
    .word   DMA3_11_IRQHandler
    .word   DMA3_12_IRQHandler
    .word   DMA3_13_IRQHandler
    .word   DMA3_14_IRQHandler
    .word   DMA3_15_IRQHandler
    .word   DMA3_16_IRQHandler
    .word   DMA3_17_IRQHandler
    .word   DMA3_18_IRQHandler
    .word   DMA3_19_IRQHandler
    .word   DMA3_20_IRQHandler
    .word   DMA3_21_IRQHandler
    .word   DMA3_22_IRQHandler
    .word   DMA3_23_IRQHandler
    .word   DMA3_24_IRQHandler
    .word   DMA3_25_IRQHandler
    .word   DMA3_26_IRQHandler
    .word   DMA3_27_IRQHandler
    .word   DMA3_28_IRQHandler
    .word   DMA3_29_IRQHandler
    .word   DMA3_30_IRQHandler
    .word   DMA5_2_ERROR_IRQHandler
    .word   DMA5_2_0_1_IRQHandler
    .word   DMA5_2_2_3_IRQHandler
    .word   DMA5_2_4_5_IRQHandler
    .word   DMA5_2_6_7_IRQHandler
    .word   DMA5_2_8_9_IRQHandler
    .word   DMA5_2_10_11_IRQHandler
    .word   DMA5_2_12_13_IRQHandler
    .word   DMA5_2_14_15_IRQHandler
    .word   DMA5_2_16_17_IRQHandler
    .word   DMA5_2_18_19_IRQHandler
    .word   DMA5_2_20_21_IRQHandler
    .word   DMA5_2_22_23_IRQHandler
    .word   DMA5_2_24_25_IRQHandler
    .word   DMA5_2_26_27_IRQHandler
    .word   DMA5_2_28_29_IRQHandler
    .word   DMA5_2_30_31_IRQHandler
    .word   DMA5_2_32_33_IRQHandler
    .word   DMA5_2_34_35_IRQHandler
    .word   DMA5_2_36_37_IRQHandler
    .word   DMA5_2_38_39_IRQHandler
    .word   DMA5_2_40_41_IRQHandler
    .word   DMA5_2_42_43_IRQHandler
    .word   DMA5_2_44_45_IRQHandler
    .word   DMA5_2_46_47_IRQHandler
    .word   DMA5_2_48_49_IRQHandler
    .word   DMA5_2_50_51_IRQHandler
    .word   DMA5_2_52_53_IRQHandler
    .word   DMA5_2_54_55_IRQHandler
    .word   DMA5_2_56_57_IRQHandler
    .word   DMA5_2_58_59_IRQHandler
    .word   DMA5_2_60_61_IRQHandler
    .word   DMA5_2_62_63_IRQHandler
    .word   Reserved176_IRQHandler
    .word   Reserved177_IRQHandler
    .word   Reserved178_IRQHandler
    .word   Reserved179_IRQHandler
    .word   Reserved180_IRQHandler
    .word   Reserved181_IRQHandler
    .word   Reserved182_IRQHandler
    .word   Reserved183_IRQHandler
    .word   Reserved184_IRQHandler
    .word   SAI2_IRQHandler
    .word   SAI3_IRQHandler
    .word   SAI4_IRQHandler
    .word   SAI5_IRQHandler
    .word   Reserved189_IRQHandler
    .word   Reserved190_IRQHandler
    .word   USB1_IRQHandler
    .word   USB2_IRQHandler
    .word   LPSPI5_IRQHandler
    .word   LPSPI6_IRQHandler
    .word   LPSPI7_IRQHandler
    .word   LPSPI8_IRQHandler
    .word   LPI2C5_IRQHandler
    .word   LPI2C6_IRQHandler
    .word   LPI2C7_IRQHandler
    .word   LPI2C8_IRQHandler
    .word   PDM_HWVAD_ERROR_IRQHandler
    .word   PDM_HWVAD_EVENT_IRQHandler
    .word   PDM_ERROR_IRQHandler
    .word   PDM_EVENT_IRQHandler
    .word   Reserved205_IRQHandler
    .word   Reserved206_IRQHandler
    .word   uSDHC3_IRQHandler
    .word   Reserved208_IRQHandler
    .word   Reserved209_IRQHandler
    .word   Reserved210_IRQHandler
    .word   Reserved211_IRQHandler
    .word   Reserved212_IRQHandler
    .word   Reserved213_IRQHandler
    .word   Reserved214_IRQHandler
    .word   ADC_ER_IRQHandler
    .word   ADC_WD_IRQHandler
    .word   ADC_EOC_IRQHandler
    .word   Reserved218_IRQHandler
    .word   Reserved219_IRQHandler
    .word   Reserved220_IRQHandler
    .word   MU5_A_IRQHandler
    .word   MU6_A_IRQHandler
    .word   MU7_B_IRQHandler
    .word   MU8_B_IRQHandler
    .word   Reserved225_IRQHandler
    .word   Reserved226_IRQHandler
    .word   Reserved227_IRQHandler
    .word   Reserved228_IRQHandler
    .word   Reserved229_IRQHandler
    .word   DISP_IRQSTEER0_IRQHandler
    .word   DISP_IRQSTEER1_IRQHandler
    .word   DISP_IRQSTEER2_IRQHandler
    .word   DISP_IRQSTEER3_IRQHandler
    .word   DISP_IRQSTEER4_IRQHandler
    .word   DISP_IRQSTEER7_IRQHandler
    .word   Reserved236_IRQHandler
    .word   ISI_IRQHandler
    .word   Reserved238_IRQHandler
    .word   Reserved239_IRQHandler
    .word   IRQSTEER_0_IRQHandler
    .word   IRQSTEER_1_IRQHandler
    .word   IRQSTEER_2_IRQHandler
    .word   IRQSTEER_3_IRQHandler
    .word   IRQSTEER_4_IRQHandler
    .word   IRQSTEER_5_IRQHandler
    .word   IRQSTEER_6_IRQHandler
    .word   IRQSTEER_7_IRQHandler
    .word   IRQSTEER_8_IRQHandler
    .word   IRQSTEER_9_IRQHandler
    .word   MU1_A_IRQHandler
    .word   MU1_B_IRQHandler
    .word   MU2_A_IRQHandler
    .word   MU2_B_IRQHandler
    .word   MU3_A_IRQHandler
    .word   MU3_B_IRQHandler
    .size   __Vectors, . - __Vectors

    .section .text.Reset_Handler, "ax", %progbits
    .align 2
    .thumb_func
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    cpsid   i

    ldr     r0, =CM7_VTOR
    ldr     r1, =__bootVectors
    str     r1, [r0]

    /* Caches off without a clean; nxpMpuInit invalidates them. */
    ldr     r0, =CM7_CCR
    ldr     r1, [r0]
    bic     r1, r1, #CM7_CCR_CACHES
    str     r1, [r0]
    dsb
    isb
    ldr     r0, =CM7_MPU_CTRL
    movs    r1, #0
    str     r1, [r0]
    ldr     r0, =CM7_SYST_CSR
    str     r1, [r0]
    dsb
    isb

    /* A stale channel could still write into the TCMs about to be seeded. */
    bl      imx95DmaStopOwnedChannels
    /* Stop motor outputs a previous image left running. */
    bl      imx95StopOwnedPeripherals

    ldr     r0, =CM7_NVIC_ICER0
    ldr     r2, =CM7_NVIC_ICPR0
    mvn     r1, #0
    movs    r3, #CM7_NVIC_REGISTERS
1:  str     r1, [r0], #4
    str     r1, [r2], #4
    subs    r3, r3, #1
    bne     1b
    dsb
    isb

    /* ITCM ECC words are 64 bits wide, so it is seeded with strd. */
    movs    r2, #0
    movs    r3, #0
    ldr     r0, =__itcm_region_start
    ldr     r1, =__itcm_region_end
1:  strd    r2, r3, [r0], #8
    cmp     r0, r1
    blo     1b

    /* DTCM ECC words are 32 bits wide. */
    ldr     r0, =__dtcm_region_start
    ldr     r1, =__dtcm_region_end
1:  str     r2, [r0], #4
    cmp     r0, r1
    blo     1b

    /* OCRAM_CODE and OCRAM_RODATA: flash -> OCRAM. */
    ldr     r0, =__ocram_text_load
    ldr     r1, =__ocram_text_start
    ldr     r2, =__ocram_text_end
    bl      copyRange

    /* Zero SLOW_DATA_ZERO_INIT in OCRAM, which may be empty. */
    movs    r2, #0
    ldr     r0, =__ocram2_start
    ldr     r1, =__ocram2_end
1:  cmp     r0, r1
    bhs     2f
    str     r2, [r0], #4
    b       1b
2:
    dsb
    isb

    b       nxpResetTail
    .size   Reset_Handler, . - Reset_Handler

/* A fault before the full table is in place stops here for a debugger. */
    .thumb_func
    .type   earlyFault, %function
earlyFault:
    b       earlyFault
    .size   earlyFault, . - earlyFault

/* Weak default handlers branch to systemFaultAction(), which never returns. */
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

    def_default_handler Reserved16_IRQHandler
    def_default_handler Reserved17_IRQHandler
    def_default_handler Reserved18_IRQHandler
    def_default_handler Reserved19_IRQHandler
    def_default_handler Reserved20_IRQHandler
    def_default_handler Reserved21_IRQHandler
    def_default_handler Reserved22_IRQHandler
    def_default_handler Reserved23_IRQHandler
    def_default_handler CAN1_IRQHandler
    def_default_handler CAN1_ERROR_IRQHandler
    def_default_handler GPIO1_0_IRQHandler
    def_default_handler GPIO1_1_IRQHandler
    def_default_handler I3C1_IRQHandler
    def_default_handler LPI2C1_IRQHandler
    def_default_handler LPI2C2_IRQHandler
    def_default_handler LPIT1_IRQHandler
    def_default_handler LPSPI1_IRQHandler
    def_default_handler LPSPI2_IRQHandler
    def_default_handler LPTMR1_IRQHandler
    def_default_handler LPUART1_IRQHandler
    def_default_handler LPUART2_IRQHandler
    def_default_handler Reserved37_IRQHandler
    def_default_handler Reserved38_IRQHandler
    def_default_handler Reserved39_IRQHandler
    def_default_handler Reserved40_IRQHandler
    def_default_handler Reserved41_IRQHandler
    def_default_handler Reserved42_IRQHandler
    def_default_handler V2X_FH_APCH0_IRQHandler
    def_default_handler V2X_FH_APHSM1_IRQHandler
    def_default_handler TPM1_IRQHandler
    def_default_handler TPM2_IRQHandler
    def_default_handler WDOG1_IRQHandler
    def_default_handler WDOG2_IRQHandler
    def_default_handler TRDC_MGR_A_IRQHandler
    def_default_handler SAI1_IRQHandler
    def_default_handler Reserved51_IRQHandler
    def_default_handler Reserved52_IRQHandler
    def_default_handler Reserved53_IRQHandler
    def_default_handler CAN2_IRQHandler
    def_default_handler CAN2_ERROR_IRQHandler
    def_default_handler CAN3_IRQHandler
    def_default_handler CAN3_ERROR_IRQHandler
    def_default_handler CAN4_IRQHandler
    def_default_handler CAN4_ERROR_IRQHandler
    def_default_handler CAN5_IRQHandler
    def_default_handler CAN5_ERROR_IRQHandler
    def_default_handler FLEXIO1_IRQHandler
    def_default_handler FLEXIO2_IRQHandler
    def_default_handler FlexSPI1_IRQHandler
    def_default_handler GPIO2_0_IRQHandler
    def_default_handler GPIO2_1_IRQHandler
    def_default_handler GPIO3_0_IRQHandler
    def_default_handler GPIO3_1_IRQHandler
    def_default_handler GPIO4_0_IRQHandler
    def_default_handler GPIO4_1_IRQHandler
    def_default_handler GPIO5_0_IRQHandler
    def_default_handler GPIO5_1_IRQHandler
    def_default_handler I3C2_IRQHandler
    def_default_handler LPI2C3_IRQHandler
    def_default_handler LPI2C4_IRQHandler
    def_default_handler LPIT2_IRQHandler
    def_default_handler LPSPI3_IRQHandler
    def_default_handler LPSPI4_IRQHandler
    def_default_handler LPTMR2_IRQHandler
    def_default_handler LPUART3_IRQHandler
    def_default_handler LPUART4_IRQHandler
    def_default_handler LPUART5_IRQHandler
    def_default_handler LPUART6_IRQHandler
    def_default_handler LPUART7_IRQHandler
    def_default_handler LPUART8_IRQHandler
    def_default_handler Reserved86_IRQHandler
    def_default_handler Reserved87_IRQHandler
    def_default_handler Reserved88_IRQHandler
    def_default_handler TPM3_IRQHandler
    def_default_handler TPM4_IRQHandler
    def_default_handler TPM5_IRQHandler
    def_default_handler TPM6_IRQHandler
    def_default_handler WDOG3_IRQHandler
    def_default_handler WDOG4_IRQHandler
    def_default_handler WDOG5_IRQHandler
    def_default_handler TMPSNS1_THR1_IRQHandler
    def_default_handler TMPSNS1_THR2_IRQHandler
    def_default_handler TMPSNS1_DRDY_IRQHandler
    def_default_handler TMPSNS2_THR1_IRQHandler
    def_default_handler TMPSNS2_THR2_IRQHandler
    def_default_handler TMPSNS2_DRDY_IRQHandler
    def_default_handler uSDHC1_IRQHandler
    def_default_handler uSDHC2_IRQHandler
    def_default_handler Reserved104_IRQHandler
    def_default_handler Reserved105_IRQHandler
    def_default_handler Reserved106_IRQHandler
    def_default_handler Reserved107_IRQHandler
    def_default_handler Reserved108_IRQHandler
    def_default_handler Reserved109_IRQHandler
    def_default_handler Reserved110_IRQHandler
    def_default_handler DMA3_ERROR_IRQHandler
    def_default_handler DMA3_0_IRQHandler
    def_default_handler DMA3_1_IRQHandler
    def_default_handler DMA3_2_IRQHandler
    def_default_handler DMA3_3_IRQHandler
    def_default_handler DMA3_4_IRQHandler
    def_default_handler DMA3_5_IRQHandler
    def_default_handler DMA3_6_IRQHandler
    def_default_handler DMA3_7_IRQHandler
    def_default_handler DMA3_8_IRQHandler
    def_default_handler DMA3_9_IRQHandler
    def_default_handler DMA3_10_IRQHandler
    def_default_handler DMA3_11_IRQHandler
    def_default_handler DMA3_12_IRQHandler
    def_default_handler DMA3_13_IRQHandler
    def_default_handler DMA3_14_IRQHandler
    def_default_handler DMA3_15_IRQHandler
    def_default_handler DMA3_16_IRQHandler
    def_default_handler DMA3_17_IRQHandler
    def_default_handler DMA3_18_IRQHandler
    def_default_handler DMA3_19_IRQHandler
    def_default_handler DMA3_20_IRQHandler
    def_default_handler DMA3_21_IRQHandler
    def_default_handler DMA3_22_IRQHandler
    def_default_handler DMA3_23_IRQHandler
    def_default_handler DMA3_24_IRQHandler
    def_default_handler DMA3_25_IRQHandler
    def_default_handler DMA3_26_IRQHandler
    def_default_handler DMA3_27_IRQHandler
    def_default_handler DMA3_28_IRQHandler
    def_default_handler DMA3_29_IRQHandler
    def_default_handler DMA3_30_IRQHandler
    def_default_handler DMA5_2_ERROR_IRQHandler
    def_default_handler DMA5_2_0_1_IRQHandler
    def_default_handler DMA5_2_2_3_IRQHandler
    def_default_handler DMA5_2_4_5_IRQHandler
    def_default_handler DMA5_2_6_7_IRQHandler
    def_default_handler DMA5_2_8_9_IRQHandler
    def_default_handler DMA5_2_10_11_IRQHandler
    def_default_handler DMA5_2_12_13_IRQHandler
    def_default_handler DMA5_2_14_15_IRQHandler
    def_default_handler DMA5_2_16_17_IRQHandler
    def_default_handler DMA5_2_18_19_IRQHandler
    def_default_handler DMA5_2_20_21_IRQHandler
    def_default_handler DMA5_2_22_23_IRQHandler
    def_default_handler DMA5_2_24_25_IRQHandler
    def_default_handler DMA5_2_26_27_IRQHandler
    def_default_handler DMA5_2_28_29_IRQHandler
    def_default_handler DMA5_2_30_31_IRQHandler
    def_default_handler DMA5_2_32_33_IRQHandler
    def_default_handler DMA5_2_34_35_IRQHandler
    def_default_handler DMA5_2_36_37_IRQHandler
    def_default_handler DMA5_2_38_39_IRQHandler
    def_default_handler DMA5_2_40_41_IRQHandler
    def_default_handler DMA5_2_42_43_IRQHandler
    def_default_handler DMA5_2_44_45_IRQHandler
    def_default_handler DMA5_2_46_47_IRQHandler
    def_default_handler DMA5_2_48_49_IRQHandler
    def_default_handler DMA5_2_50_51_IRQHandler
    def_default_handler DMA5_2_52_53_IRQHandler
    def_default_handler DMA5_2_54_55_IRQHandler
    def_default_handler DMA5_2_56_57_IRQHandler
    def_default_handler DMA5_2_58_59_IRQHandler
    def_default_handler DMA5_2_60_61_IRQHandler
    def_default_handler DMA5_2_62_63_IRQHandler
    def_default_handler Reserved176_IRQHandler
    def_default_handler Reserved177_IRQHandler
    def_default_handler Reserved178_IRQHandler
    def_default_handler Reserved179_IRQHandler
    def_default_handler Reserved180_IRQHandler
    def_default_handler Reserved181_IRQHandler
    def_default_handler Reserved182_IRQHandler
    def_default_handler Reserved183_IRQHandler
    def_default_handler Reserved184_IRQHandler
    def_default_handler SAI2_IRQHandler
    def_default_handler SAI3_IRQHandler
    def_default_handler SAI4_IRQHandler
    def_default_handler SAI5_IRQHandler
    def_default_handler Reserved189_IRQHandler
    def_default_handler Reserved190_IRQHandler
    def_default_handler USB1_IRQHandler
    def_default_handler USB2_IRQHandler
    def_default_handler LPSPI5_IRQHandler
    def_default_handler LPSPI6_IRQHandler
    def_default_handler LPSPI7_IRQHandler
    def_default_handler LPSPI8_IRQHandler
    def_default_handler LPI2C5_IRQHandler
    def_default_handler LPI2C6_IRQHandler
    def_default_handler LPI2C7_IRQHandler
    def_default_handler LPI2C8_IRQHandler
    def_default_handler PDM_HWVAD_ERROR_IRQHandler
    def_default_handler PDM_HWVAD_EVENT_IRQHandler
    def_default_handler PDM_ERROR_IRQHandler
    def_default_handler PDM_EVENT_IRQHandler
    def_default_handler Reserved205_IRQHandler
    def_default_handler Reserved206_IRQHandler
    def_default_handler uSDHC3_IRQHandler
    def_default_handler Reserved208_IRQHandler
    def_default_handler Reserved209_IRQHandler
    def_default_handler Reserved210_IRQHandler
    def_default_handler Reserved211_IRQHandler
    def_default_handler Reserved212_IRQHandler
    def_default_handler Reserved213_IRQHandler
    def_default_handler Reserved214_IRQHandler
    def_default_handler ADC_ER_IRQHandler
    def_default_handler ADC_WD_IRQHandler
    def_default_handler ADC_EOC_IRQHandler
    def_default_handler Reserved218_IRQHandler
    def_default_handler Reserved219_IRQHandler
    def_default_handler Reserved220_IRQHandler
    def_default_handler MU5_A_IRQHandler
    def_default_handler MU6_A_IRQHandler
    def_default_handler MU7_B_IRQHandler
    def_default_handler MU8_B_IRQHandler
    def_default_handler Reserved225_IRQHandler
    def_default_handler Reserved226_IRQHandler
    def_default_handler Reserved227_IRQHandler
    def_default_handler Reserved228_IRQHandler
    def_default_handler Reserved229_IRQHandler
    def_default_handler DISP_IRQSTEER0_IRQHandler
    def_default_handler DISP_IRQSTEER1_IRQHandler
    def_default_handler DISP_IRQSTEER2_IRQHandler
    def_default_handler DISP_IRQSTEER3_IRQHandler
    def_default_handler DISP_IRQSTEER4_IRQHandler
    def_default_handler DISP_IRQSTEER7_IRQHandler
    def_default_handler Reserved236_IRQHandler
    def_default_handler ISI_IRQHandler
    def_default_handler Reserved238_IRQHandler
    def_default_handler Reserved239_IRQHandler
    def_default_handler IRQSTEER_0_IRQHandler
    def_default_handler IRQSTEER_1_IRQHandler
    def_default_handler IRQSTEER_2_IRQHandler
    def_default_handler IRQSTEER_3_IRQHandler
    def_default_handler IRQSTEER_4_IRQHandler
    def_default_handler IRQSTEER_5_IRQHandler
    def_default_handler IRQSTEER_6_IRQHandler
    def_default_handler IRQSTEER_7_IRQHandler
    def_default_handler IRQSTEER_8_IRQHandler
    def_default_handler IRQSTEER_9_IRQHandler
    def_default_handler MU1_A_IRQHandler
    def_default_handler MU1_B_IRQHandler
    def_default_handler MU2_A_IRQHandler
    def_default_handler MU2_B_IRQHandler
    def_default_handler MU3_A_IRQHandler
    def_default_handler MU3_B_IRQHandler
