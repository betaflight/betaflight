/**
 ******************************************************************************
 * @file      startup_stm32n657xx.s
 * @author    GPM Application Team
 * @brief     STM32N657XX device vector table for GCC toolchain.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the initial PC == Reset_Handler,
 *                - Set the vector table entries with the exceptions ISR address
 *                - Copy firmware code from XSPI flash to RAM (LRUN)
 *                - Branches to main in the C library (which eventually
 *                  calls main()).
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

.syntax unified
.arch armv8.1-m.main
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* Linker symbols used by Reset_Handler */
.word _sitext
.word _stext
.word _etext
.word _sbss
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 *
 *          This handler executes from XSPI flash (XIP) and copies all
 *          firmware code and data to RAM before jumping to main().
 *
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  /* Mask interrupts and clear every NVIC IRQ enable + pending bit before
     any C code runs. SWD-load doesn't reset the NVIC, so any IRQ left
     enabled by a previous BF run (notably USB1_OTG_HS) stays pending and
     fires while .bss is still being zeroed — the handler's globals are
     NULL and we BusFault before main(). */
  cpsid i
  movs  r0, #0
  ldr   r1, =0xFFFFFFFF
  ldr   r2, =0xE000E180   /* NVIC_ICER0 */
  movs  r3, #16
1:
  str   r1, [r2]          /* clear ISER[i] */
  adds  r2, r2, #4
  subs  r3, r3, #1
  bne   1b
  ldr   r2, =0xE000E280   /* NVIC_ICPR0 */
  movs  r3, #16
2:
  str   r1, [r2]          /* clear ISPR[i] */
  adds  r2, r2, #4
  subs  r3, r3, #1
  bne   2b
  /* SysTick off as well */
  ldr   r2, =0xE000E010   /* SysTick_CTRL */
  str   r0, [r2]
  ldr   r2, =0xE000E018   /* SysTick_VAL */
  str   r0, [r2]
  dsb
  isb

  ldr   r0, =_sstack
  msr   MSPLIM, r0
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* BF↔OBL bring-up debug — earliest markers. AXISRAM2 NS at the
 * @DBGRAM buffer base. OBL exposes 0x24100000..0x241001FF. Writes
 * at MULTIPLE offsets so we can see which (if any) survive — the OLD
 * BF's write of 0xBFD00001 to offset 0 produced 0x00000001 in the
 * buffer, suggesting either RIFSC NS-access truncation or the boot
 * ROM clobbering offset 0 specifically.
 *
 *   0x24100000  CABA0001  Reset_Handler entered (offset 0 — clobber test)
 *   0x24100080  CABA0001  ditto, mid-buffer
 *   0x24100100  CABA0001  ditto, second half (proven OBL-write area)
 *   0x24100104  CABA0002  after AXISRAM clock enable
 *   0x24100108  CABA0003  after .text copy
 *   0x2410010C  CABA0004  after .bss zero
 *   0x24100110  CABA0005  after __libc_init_array, just before main()
 *   0x24100120  bf_pc     literal address of Reset_Handler (=PC self)
 */
  ldr   r3, =0xCABA0001
  ldr   r2, =0x24100000
  str   r3, [r2]
  ldr   r2, =0x24100080
  str   r3, [r2]
  ldr   r2, =0x24100100
  str   r3, [r2]
  /* Self-PC: load PC into r2 then store. Tells us where BF actually
   * is executing from when this runs. */
  mov   r2, pc
  ldr   r3, =0x24100120
  str   r2, [r3]
  dsb
  isb

/* Enable AXISRAM1..6 clocks before any AXISRAM1+ store. The boot ROM only
 * clocks AXISRAM2 when it hands off to a signed FSBL; .bss, the stack and
 * the DMA-capable regions live in AXISRAM1 / AXISRAM3+ and the next loop
 * (.bss zero) faults if the RAM is unclocked. Idempotent for the FSBL-stub
 * handoff and SWD-load paths where AXISRAM1 is already clocked. Writes
 * RCC->MEMENSR via the NS alias (0x4602_8A4C); the register itself is
 * shared regardless of alias. */
  ldr   r2, =0x46028A4C
  ldr   r3, =0x0000018F   /* AXISRAM3..6 (b3:0) | AXISRAM1 (b7) | AXISRAM2 (b8) */
  str   r3, [r2]
  dsb
  isb

/* Marker: post-AXISRAM-clock-enable. */
  ldr   r2, =0x24100104
  ldr   r3, =0xCABA0002
  str   r3, [r2]

/* Copy firmware code from XSPI flash to RAM (.text, .rodata, .pg_registry, etc.) */
  ldr r0, =_stext
  ldr r1, =_etext
  ldr r2, =_sitext
  movs r3, #0
  b LoopCopyTextInit

CopyTextInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyTextInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyTextInit

/* Marker: post-.text-copy. */
  ldr   r2, =0x24100108
  ldr   r3, =0xCABA0003
  str   r3, [r2]

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Marker: post-.bss-zero. */
  ldr   r2, =0x2410010C
  ldr   r3, =0xCABA0004
  str   r3, [r2]

/* Call static constructors */
  bl __libc_init_array

/* Marker: post-libc-init, pre-main. */
  ldr   r2, =0x24100110
  ldr   r3, =0xCABA0005
  str   r3, [r2]

/* Call the application's entry point.*/
  bl main

LoopForever:
  b LoopForever

  .size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The STM32N657XX vector table. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
* External-interrupt slot order matches the CMSIS IRQn_Type enum exactly
* (stm32n657xx.h). Reserved IRQ numbers emit `.word 0` so that every
* IRQn=N maps to vector offset 16+N words.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word SecureFault_Handler
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  /* External interrupts (CMSIS IRQn order) */
  .word PVD_PVM_IRQHandler                      /*   0  PVD/PVM through EXTI Line detection          */
  .word 0                                       /*   1  Reserved                                     */
  .word DTS_IRQHandler                          /*   2  Digital Temperature Sensor                   */
  .word RCC_IRQHandler                          /*   3  RCC global                                   */
  .word LOCKUP_IRQHandler                       /*   4  Cortex-M55 LOCKUP                            */
  .word CACHE_ECC_IRQHandler                    /*   5  Cache ECC                                    */
  .word TCM_ECC_IRQHandler                      /*   6  TCM ECC                                      */
  .word BKP_ECC_IRQHandler                      /*   7  Backup-RAM ECC                               */
  .word FPU_IRQHandler                          /*   8  FPU                                          */
  .word 0                                       /*   9  Reserved                                     */
  .word RTC_S_IRQHandler                        /*  10  RTC secure                                   */
  .word TAMP_IRQHandler                         /*  11  Tamper                                       */
  .word RIFSC_TAMPER_IRQHandler                 /*  12  RIFSC tamper                                 */
  .word IAC_IRQHandler                          /*  13  IAC                                          */
  .word RCC_S_IRQHandler                        /*  14  RCC secure                                   */
  .word 0                                       /*  15  Reserved                                     */
  .word RTC_IRQHandler                          /*  16  RTC non-secure                               */
  .word 0                                       /*  17  Reserved                                     */
  .word IWDG_IRQHandler                         /*  18  Internal Watchdog                            */
  .word WWDG_IRQHandler                         /*  19  Window Watchdog                              */
  .word EXTI0_IRQHandler                        /*  20  EXTI Line 0                                  */
  .word EXTI1_IRQHandler                        /*  21  EXTI Line 1                                  */
  .word EXTI2_IRQHandler                        /*  22  EXTI Line 2                                  */
  .word EXTI3_IRQHandler                        /*  23  EXTI Line 3                                  */
  .word EXTI4_IRQHandler                        /*  24  EXTI Line 4                                  */
  .word EXTI5_IRQHandler                        /*  25  EXTI Line 5                                  */
  .word EXTI6_IRQHandler                        /*  26  EXTI Line 6                                  */
  .word EXTI7_IRQHandler                        /*  27  EXTI Line 7                                  */
  .word EXTI8_IRQHandler                        /*  28  EXTI Line 8                                  */
  .word EXTI9_IRQHandler                        /*  29  EXTI Line 9                                  */
  .word EXTI10_IRQHandler                       /*  30  EXTI Line 10                                 */
  .word EXTI11_IRQHandler                       /*  31  EXTI Line 11                                 */
  .word EXTI12_IRQHandler                       /*  32  EXTI Line 12                                 */
  .word EXTI13_IRQHandler                       /*  33  EXTI Line 13                                 */
  .word EXTI14_IRQHandler                       /*  34  EXTI Line 14                                 */
  .word EXTI15_IRQHandler                       /*  35  EXTI Line 15                                 */
  .word SAES_IRQHandler                         /*  36  SAES                                         */
  .word CRYP_IRQHandler                         /*  37  CRYP                                         */
  .word PKA_IRQHandler                          /*  38  PKA                                          */
  .word HASH_IRQHandler                         /*  39  HASH                                         */
  .word RNG_IRQHandler                          /*  40  RNG                                          */
  .word 0                                       /*  41  Reserved                                     */
  .word MCE1_IRQHandler                         /*  42  MCE1                                         */
  .word MCE2_IRQHandler                         /*  43  MCE2                                         */
  .word MCE3_IRQHandler                         /*  44  MCE3                                         */
  .word MCE4_IRQHandler                         /*  45  MCE4                                         */
  .word ADC1_2_IRQHandler                       /*  46  ADC1/ADC2                                    */
  .word CSI_IRQHandler                          /*  47  CSI                                          */
  .word DCMIPP_IRQHandler                       /*  48  DCMIPP                                       */
  .word 0                                       /*  49  Reserved                                     */
  .word 0                                       /*  50  Reserved                                     */
  .word 0                                       /*  51  Reserved                                     */
  .word PAHB_ERR_IRQHandler                     /*  52  PAHB error                                   */
  .word NPU0_IRQHandler                         /*  53  NPU master ints[0]                           */
  .word NPU1_IRQHandler                         /*  54  NPU master ints[1]                           */
  .word NPU2_IRQHandler                         /*  55  NPU master ints[2]                           */
  .word NPU3_IRQHandler                         /*  56  NPU master ints[3]                           */
  .word CACHEAXI_IRQHandler                     /*  57  CACHEAXI                                     */
  .word LTDC_LO_IRQHandler                      /*  58  LTDC low-layer                               */
  .word LTDC_LO_ERR_IRQHandler                  /*  59  LTDC low-layer error                         */
  .word DMA2D_IRQHandler                        /*  60  DMA2D                                        */
  .word JPEG_IRQHandler                         /*  61  JPEG                                         */
  .word VENC_IRQHandler                         /*  62  VENC                                         */
  .word GFXMMU_IRQHandler                       /*  63  GFXMMU                                       */
  .word GFXTIM_IRQHandler                       /*  64  GFXTIM                                       */
  .word GPU2D_IRQHandler                        /*  65  GPU2D global                                 */
  .word GPU2D_ER_IRQHandler                     /*  66  GPU2D error                                  */
  .word ICACHE_IRQHandler                       /*  67  ICACHE                                       */
  .word HPDMA1_Channel0_IRQHandler              /*  68  HPDMA1 Channel 0                             */
  .word HPDMA1_Channel1_IRQHandler              /*  69  HPDMA1 Channel 1                             */
  .word HPDMA1_Channel2_IRQHandler              /*  70  HPDMA1 Channel 2                             */
  .word HPDMA1_Channel3_IRQHandler              /*  71  HPDMA1 Channel 3                             */
  .word HPDMA1_Channel4_IRQHandler              /*  72  HPDMA1 Channel 4                             */
  .word HPDMA1_Channel5_IRQHandler              /*  73  HPDMA1 Channel 5                             */
  .word HPDMA1_Channel6_IRQHandler              /*  74  HPDMA1 Channel 6                             */
  .word HPDMA1_Channel7_IRQHandler              /*  75  HPDMA1 Channel 7                             */
  .word HPDMA1_Channel8_IRQHandler              /*  76  HPDMA1 Channel 8                             */
  .word HPDMA1_Channel9_IRQHandler              /*  77  HPDMA1 Channel 9                             */
  .word HPDMA1_Channel10_IRQHandler             /*  78  HPDMA1 Channel 10                            */
  .word HPDMA1_Channel11_IRQHandler             /*  79  HPDMA1 Channel 11                            */
  .word HPDMA1_Channel12_IRQHandler             /*  80  HPDMA1 Channel 12                            */
  .word HPDMA1_Channel13_IRQHandler             /*  81  HPDMA1 Channel 13                            */
  .word HPDMA1_Channel14_IRQHandler             /*  82  HPDMA1 Channel 14                            */
  .word HPDMA1_Channel15_IRQHandler             /*  83  HPDMA1 Channel 15                            */
  .word GPDMA1_Channel0_IRQHandler              /*  84  GPDMA1 Channel 0                             */
  .word GPDMA1_Channel1_IRQHandler              /*  85  GPDMA1 Channel 1                             */
  .word GPDMA1_Channel2_IRQHandler              /*  86  GPDMA1 Channel 2                             */
  .word GPDMA1_Channel3_IRQHandler              /*  87  GPDMA1 Channel 3                             */
  .word GPDMA1_Channel4_IRQHandler              /*  88  GPDMA1 Channel 4                             */
  .word GPDMA1_Channel5_IRQHandler              /*  89  GPDMA1 Channel 5                             */
  .word GPDMA1_Channel6_IRQHandler              /*  90  GPDMA1 Channel 6                             */
  .word GPDMA1_Channel7_IRQHandler              /*  91  GPDMA1 Channel 7                             */
  .word GPDMA1_Channel8_IRQHandler              /*  92  GPDMA1 Channel 8                             */
  .word GPDMA1_Channel9_IRQHandler              /*  93  GPDMA1 Channel 9                             */
  .word GPDMA1_Channel10_IRQHandler             /*  94  GPDMA1 Channel 10                            */
  .word GPDMA1_Channel11_IRQHandler             /*  95  GPDMA1 Channel 11                            */
  .word GPDMA1_Channel12_IRQHandler             /*  96  GPDMA1 Channel 12                            */
  .word GPDMA1_Channel13_IRQHandler             /*  97  GPDMA1 Channel 13                            */
  .word GPDMA1_Channel14_IRQHandler             /*  98  GPDMA1 Channel 14                            */
  .word GPDMA1_Channel15_IRQHandler             /*  99  GPDMA1 Channel 15                            */
  .word I2C1_EV_IRQHandler                      /* 100  I2C1 Event                                   */
  .word I2C1_ER_IRQHandler                      /* 101  I2C1 Error                                   */
  .word I2C2_EV_IRQHandler                      /* 102  I2C2 Event                                   */
  .word I2C2_ER_IRQHandler                      /* 103  I2C2 Error                                   */
  .word I2C3_EV_IRQHandler                      /* 104  I2C3 Event                                   */
  .word I2C3_ER_IRQHandler                      /* 105  I2C3 Error                                   */
  .word I2C4_EV_IRQHandler                      /* 106  I2C4 Event                                   */
  .word I2C4_ER_IRQHandler                      /* 107  I2C4 Error                                   */
  .word I3C1_EV_IRQHandler                      /* 108  I3C1 Event                                   */
  .word I3C1_ER_IRQHandler                      /* 109  I3C1 Error                                   */
  .word I3C2_EV_IRQHandler                      /* 110  I3C2 Event                                   */
  .word I3C2_ER_IRQHandler                      /* 111  I3C2 Error                                   */
  .word TIM1_BRK_IRQHandler                     /* 112  TIM1 Break                                   */
  .word TIM1_UP_IRQHandler                      /* 113  TIM1 Update                                  */
  .word TIM1_TRG_COM_IRQHandler                 /* 114  TIM1 Trigger and Commutation                 */
  .word TIM1_CC_IRQHandler                      /* 115  TIM1 Capture Compare                         */
  .word TIM2_IRQHandler                         /* 116  TIM2                                         */
  .word TIM3_IRQHandler                         /* 117  TIM3                                         */
  .word TIM4_IRQHandler                         /* 118  TIM4                                         */
  .word TIM5_IRQHandler                         /* 119  TIM5                                         */
  .word TIM6_IRQHandler                         /* 120  TIM6                                         */
  .word TIM7_IRQHandler                         /* 121  TIM7                                         */
  .word TIM8_BRK_IRQHandler                     /* 122  TIM8 Break                                   */
  .word TIM8_UP_IRQHandler                      /* 123  TIM8 Update                                  */
  .word TIM8_TRG_COM_IRQHandler                 /* 124  TIM8 Trigger and Commutation                 */
  .word TIM8_CC_IRQHandler                      /* 125  TIM8 Capture Compare                         */
  .word TIM9_IRQHandler                         /* 126  TIM9                                         */
  .word TIM10_IRQHandler                        /* 127  TIM10                                        */
  .word TIM11_IRQHandler                        /* 128  TIM11                                        */
  .word TIM12_IRQHandler                        /* 129  TIM12                                        */
  .word TIM13_IRQHandler                        /* 130  TIM13                                        */
  .word TIM14_IRQHandler                        /* 131  TIM14                                        */
  .word TIM15_IRQHandler                        /* 132  TIM15                                        */
  .word TIM16_IRQHandler                        /* 133  TIM16                                        */
  .word TIM17_IRQHandler                        /* 134  TIM17                                        */
  .word TIM18_IRQHandler                        /* 135  TIM18                                        */
  .word LPTIM1_IRQHandler                       /* 136  LPTIM1                                       */
  .word LPTIM2_IRQHandler                       /* 137  LPTIM2                                       */
  .word LPTIM3_IRQHandler                       /* 138  LPTIM3                                       */
  .word LPTIM4_IRQHandler                       /* 139  LPTIM4                                       */
  .word LPTIM5_IRQHandler                       /* 140  LPTIM5                                       */
  .word ADF1_FLT0_IRQHandler                    /* 141  ADF1 Filter 0                                */
  .word MDF1_FLT0_IRQHandler                    /* 142  MDF1 Filter 0                                */
  .word MDF1_FLT1_IRQHandler                    /* 143  MDF1 Filter 1                                */
  .word MDF1_FLT2_IRQHandler                    /* 144  MDF1 Filter 2                                */
  .word MDF1_FLT3_IRQHandler                    /* 145  MDF1 Filter 3                                */
  .word MDF1_FLT4_IRQHandler                    /* 146  MDF1 Filter 4                                */
  .word MDF1_FLT5_IRQHandler                    /* 147  MDF1 Filter 5                                */
  .word SAI1_A_IRQHandler                       /* 148  SAI1 Block A                                 */
  .word SAI1_B_IRQHandler                       /* 149  SAI1 Block B                                 */
  .word SAI2_A_IRQHandler                       /* 150  SAI2 Block A                                 */
  .word SAI2_B_IRQHandler                       /* 151  SAI2 Block B                                 */
  .word SPDIFRX1_IRQHandler                     /* 152  SPDIFRX1                                     */
  .word SPI1_IRQHandler                         /* 153  SPI1                                         */
  .word SPI2_IRQHandler                         /* 154  SPI2                                         */
  .word SPI3_IRQHandler                         /* 155  SPI3                                         */
  .word SPI4_IRQHandler                         /* 156  SPI4                                         */
  .word SPI5_IRQHandler                         /* 157  SPI5                                         */
  .word SPI6_IRQHandler                         /* 158  SPI6                                         */
  .word USART1_IRQHandler                       /* 159  USART1                                       */
  .word USART2_IRQHandler                       /* 160  USART2                                       */
  .word USART3_IRQHandler                       /* 161  USART3                                       */
  .word UART4_IRQHandler                        /* 162  UART4                                        */
  .word UART5_IRQHandler                        /* 163  UART5                                        */
  .word USART6_IRQHandler                       /* 164  USART6                                       */
  .word UART7_IRQHandler                        /* 165  UART7                                        */
  .word UART8_IRQHandler                        /* 166  UART8                                        */
  .word UART9_IRQHandler                        /* 167  UART9                                        */
  .word USART10_IRQHandler                      /* 168  USART10                                      */
  .word LPUART1_IRQHandler                      /* 169  LPUART1                                      */
  .word XSPI1_IRQHandler                        /* 170  XSPI1                                        */
  .word XSPI2_IRQHandler                        /* 171  XSPI2                                        */
  .word XSPI3_IRQHandler                        /* 172  XSPI3                                        */
  .word FMC_IRQHandler                          /* 173  FMC                                          */
  .word SDMMC1_IRQHandler                       /* 174  SDMMC1                                       */
  .word SDMMC2_IRQHandler                       /* 175  SDMMC2                                       */
  .word UCPD1_IRQHandler                        /* 176  UCPD1                                        */
  .word USB1_OTG_HS_IRQHandler                  /* 177  USB1 OTG HS                                  */
  .word USB2_OTG_HS_IRQHandler                  /* 178  USB2 OTG HS                                  */
  .word ETH1_IRQHandler                         /* 179  Ethernet                                     */
  .word FDCAN1_IT0_IRQHandler                   /* 180  FDCAN1 IT0                                   */
  .word FDCAN1_IT1_IRQHandler                   /* 181  FDCAN1 IT1                                   */
  .word FDCAN2_IT0_IRQHandler                   /* 182  FDCAN2 IT0                                   */
  .word FDCAN2_IT1_IRQHandler                   /* 183  FDCAN2 IT1                                   */
  .word FDCAN3_IT0_IRQHandler                   /* 184  FDCAN3 IT0                                   */
  .word FDCAN3_IT1_IRQHandler                   /* 185  FDCAN3 IT1                                   */
  .word FDCAN_CU_IRQHandler                     /* 186  FDCAN clock unit                             */
  .word MDIOS_IRQHandler                        /* 187  MDIOS                                        */
  .word DCMI_PSSI_IRQHandler                    /* 188  DCMI/PSSI                                    */
  .word WAKEUP_PIN_IRQHandler                   /* 189  Wake-up pins                                 */
  .word CTI_INT0_IRQHandler                     /* 190  CTI INT0                                     */
  .word CTI_INT1_IRQHandler                     /* 191  CTI INT1                                     */
  .word 0                                       /* 192  Reserved                                     */
  .word LTDC_UP_IRQHandler                      /* 193  LTDC up-layer                                */
  .word LTDC_UP_ERR_IRQHandler                  /* 194  LTDC up-layer error                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak SecureFault_Handler
  .thumb_set SecureFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler,Default_Handler

  .weak DTS_IRQHandler
  .thumb_set DTS_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak LOCKUP_IRQHandler
  .thumb_set LOCKUP_IRQHandler,Default_Handler

  .weak CACHE_ECC_IRQHandler
  .thumb_set CACHE_ECC_IRQHandler,Default_Handler

  .weak TCM_ECC_IRQHandler
  .thumb_set TCM_ECC_IRQHandler,Default_Handler

  .weak BKP_ECC_IRQHandler
  .thumb_set BKP_ECC_IRQHandler,Default_Handler

  .weak FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

  .weak RTC_S_IRQHandler
  .thumb_set RTC_S_IRQHandler,Default_Handler

  .weak TAMP_IRQHandler
  .thumb_set TAMP_IRQHandler,Default_Handler

  .weak RIFSC_TAMPER_IRQHandler
  .thumb_set RIFSC_TAMPER_IRQHandler,Default_Handler

  .weak IAC_IRQHandler
  .thumb_set IAC_IRQHandler,Default_Handler

  .weak RCC_S_IRQHandler
  .thumb_set RCC_S_IRQHandler,Default_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak IWDG_IRQHandler
  .thumb_set IWDG_IRQHandler,Default_Handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak EXTI5_IRQHandler
  .thumb_set EXTI5_IRQHandler,Default_Handler

  .weak EXTI6_IRQHandler
  .thumb_set EXTI6_IRQHandler,Default_Handler

  .weak EXTI7_IRQHandler
  .thumb_set EXTI7_IRQHandler,Default_Handler

  .weak EXTI8_IRQHandler
  .thumb_set EXTI8_IRQHandler,Default_Handler

  .weak EXTI9_IRQHandler
  .thumb_set EXTI9_IRQHandler,Default_Handler

  .weak EXTI10_IRQHandler
  .thumb_set EXTI10_IRQHandler,Default_Handler

  .weak EXTI11_IRQHandler
  .thumb_set EXTI11_IRQHandler,Default_Handler

  .weak EXTI12_IRQHandler
  .thumb_set EXTI12_IRQHandler,Default_Handler

  .weak EXTI13_IRQHandler
  .thumb_set EXTI13_IRQHandler,Default_Handler

  .weak EXTI14_IRQHandler
  .thumb_set EXTI14_IRQHandler,Default_Handler

  .weak EXTI15_IRQHandler
  .thumb_set EXTI15_IRQHandler,Default_Handler

  .weak SAES_IRQHandler
  .thumb_set SAES_IRQHandler,Default_Handler

  .weak CRYP_IRQHandler
  .thumb_set CRYP_IRQHandler,Default_Handler

  .weak PKA_IRQHandler
  .thumb_set PKA_IRQHandler,Default_Handler

  .weak HASH_IRQHandler
  .thumb_set HASH_IRQHandler,Default_Handler

  .weak RNG_IRQHandler
  .thumb_set RNG_IRQHandler,Default_Handler

  .weak MCE1_IRQHandler
  .thumb_set MCE1_IRQHandler,Default_Handler

  .weak MCE2_IRQHandler
  .thumb_set MCE2_IRQHandler,Default_Handler

  .weak MCE3_IRQHandler
  .thumb_set MCE3_IRQHandler,Default_Handler

  .weak MCE4_IRQHandler
  .thumb_set MCE4_IRQHandler,Default_Handler

  .weak ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,Default_Handler

  .weak CSI_IRQHandler
  .thumb_set CSI_IRQHandler,Default_Handler

  .weak DCMIPP_IRQHandler
  .thumb_set DCMIPP_IRQHandler,Default_Handler

  .weak PAHB_ERR_IRQHandler
  .thumb_set PAHB_ERR_IRQHandler,Default_Handler

  .weak NPU0_IRQHandler
  .thumb_set NPU0_IRQHandler,Default_Handler

  .weak NPU1_IRQHandler
  .thumb_set NPU1_IRQHandler,Default_Handler

  .weak NPU2_IRQHandler
  .thumb_set NPU2_IRQHandler,Default_Handler

  .weak NPU3_IRQHandler
  .thumb_set NPU3_IRQHandler,Default_Handler

  .weak CACHEAXI_IRQHandler
  .thumb_set CACHEAXI_IRQHandler,Default_Handler

  .weak LTDC_LO_IRQHandler
  .thumb_set LTDC_LO_IRQHandler,Default_Handler

  .weak LTDC_LO_ERR_IRQHandler
  .thumb_set LTDC_LO_ERR_IRQHandler,Default_Handler

  .weak DMA2D_IRQHandler
  .thumb_set DMA2D_IRQHandler,Default_Handler

  .weak JPEG_IRQHandler
  .thumb_set JPEG_IRQHandler,Default_Handler

  .weak VENC_IRQHandler
  .thumb_set VENC_IRQHandler,Default_Handler

  .weak GFXMMU_IRQHandler
  .thumb_set GFXMMU_IRQHandler,Default_Handler

  .weak GFXTIM_IRQHandler
  .thumb_set GFXTIM_IRQHandler,Default_Handler

  .weak GPU2D_IRQHandler
  .thumb_set GPU2D_IRQHandler,Default_Handler

  .weak GPU2D_ER_IRQHandler
  .thumb_set GPU2D_ER_IRQHandler,Default_Handler

  .weak ICACHE_IRQHandler
  .thumb_set ICACHE_IRQHandler,Default_Handler

  .weak HPDMA1_Channel0_IRQHandler
  .thumb_set HPDMA1_Channel0_IRQHandler,Default_Handler

  .weak HPDMA1_Channel1_IRQHandler
  .thumb_set HPDMA1_Channel1_IRQHandler,Default_Handler

  .weak HPDMA1_Channel2_IRQHandler
  .thumb_set HPDMA1_Channel2_IRQHandler,Default_Handler

  .weak HPDMA1_Channel3_IRQHandler
  .thumb_set HPDMA1_Channel3_IRQHandler,Default_Handler

  .weak HPDMA1_Channel4_IRQHandler
  .thumb_set HPDMA1_Channel4_IRQHandler,Default_Handler

  .weak HPDMA1_Channel5_IRQHandler
  .thumb_set HPDMA1_Channel5_IRQHandler,Default_Handler

  .weak HPDMA1_Channel6_IRQHandler
  .thumb_set HPDMA1_Channel6_IRQHandler,Default_Handler

  .weak HPDMA1_Channel7_IRQHandler
  .thumb_set HPDMA1_Channel7_IRQHandler,Default_Handler

  .weak HPDMA1_Channel8_IRQHandler
  .thumb_set HPDMA1_Channel8_IRQHandler,Default_Handler

  .weak HPDMA1_Channel9_IRQHandler
  .thumb_set HPDMA1_Channel9_IRQHandler,Default_Handler

  .weak HPDMA1_Channel10_IRQHandler
  .thumb_set HPDMA1_Channel10_IRQHandler,Default_Handler

  .weak HPDMA1_Channel11_IRQHandler
  .thumb_set HPDMA1_Channel11_IRQHandler,Default_Handler

  .weak HPDMA1_Channel12_IRQHandler
  .thumb_set HPDMA1_Channel12_IRQHandler,Default_Handler

  .weak HPDMA1_Channel13_IRQHandler
  .thumb_set HPDMA1_Channel13_IRQHandler,Default_Handler

  .weak HPDMA1_Channel14_IRQHandler
  .thumb_set HPDMA1_Channel14_IRQHandler,Default_Handler

  .weak HPDMA1_Channel15_IRQHandler
  .thumb_set HPDMA1_Channel15_IRQHandler,Default_Handler

  .weak GPDMA1_Channel0_IRQHandler
  .thumb_set GPDMA1_Channel0_IRQHandler,Default_Handler

  .weak GPDMA1_Channel1_IRQHandler
  .thumb_set GPDMA1_Channel1_IRQHandler,Default_Handler

  .weak GPDMA1_Channel2_IRQHandler
  .thumb_set GPDMA1_Channel2_IRQHandler,Default_Handler

  .weak GPDMA1_Channel3_IRQHandler
  .thumb_set GPDMA1_Channel3_IRQHandler,Default_Handler

  .weak GPDMA1_Channel4_IRQHandler
  .thumb_set GPDMA1_Channel4_IRQHandler,Default_Handler

  .weak GPDMA1_Channel5_IRQHandler
  .thumb_set GPDMA1_Channel5_IRQHandler,Default_Handler

  .weak GPDMA1_Channel6_IRQHandler
  .thumb_set GPDMA1_Channel6_IRQHandler,Default_Handler

  .weak GPDMA1_Channel7_IRQHandler
  .thumb_set GPDMA1_Channel7_IRQHandler,Default_Handler

  .weak GPDMA1_Channel8_IRQHandler
  .thumb_set GPDMA1_Channel8_IRQHandler,Default_Handler

  .weak GPDMA1_Channel9_IRQHandler
  .thumb_set GPDMA1_Channel9_IRQHandler,Default_Handler

  .weak GPDMA1_Channel10_IRQHandler
  .thumb_set GPDMA1_Channel10_IRQHandler,Default_Handler

  .weak GPDMA1_Channel11_IRQHandler
  .thumb_set GPDMA1_Channel11_IRQHandler,Default_Handler

  .weak GPDMA1_Channel12_IRQHandler
  .thumb_set GPDMA1_Channel12_IRQHandler,Default_Handler

  .weak GPDMA1_Channel13_IRQHandler
  .thumb_set GPDMA1_Channel13_IRQHandler,Default_Handler

  .weak GPDMA1_Channel14_IRQHandler
  .thumb_set GPDMA1_Channel14_IRQHandler,Default_Handler

  .weak GPDMA1_Channel15_IRQHandler
  .thumb_set GPDMA1_Channel15_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak I2C4_EV_IRQHandler
  .thumb_set I2C4_EV_IRQHandler,Default_Handler

  .weak I2C4_ER_IRQHandler
  .thumb_set I2C4_ER_IRQHandler,Default_Handler

  .weak I3C1_EV_IRQHandler
  .thumb_set I3C1_EV_IRQHandler,Default_Handler

  .weak I3C1_ER_IRQHandler
  .thumb_set I3C1_ER_IRQHandler,Default_Handler

  .weak I3C2_EV_IRQHandler
  .thumb_set I3C2_EV_IRQHandler,Default_Handler

  .weak I3C2_ER_IRQHandler
  .thumb_set I3C2_ER_IRQHandler,Default_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler,Default_Handler

  .weak TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler,Default_Handler

  .weak TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak TIM9_IRQHandler
  .thumb_set TIM9_IRQHandler,Default_Handler

  .weak TIM10_IRQHandler
  .thumb_set TIM10_IRQHandler,Default_Handler

  .weak TIM11_IRQHandler
  .thumb_set TIM11_IRQHandler,Default_Handler

  .weak TIM12_IRQHandler
  .thumb_set TIM12_IRQHandler,Default_Handler

  .weak TIM13_IRQHandler
  .thumb_set TIM13_IRQHandler,Default_Handler

  .weak TIM14_IRQHandler
  .thumb_set TIM14_IRQHandler,Default_Handler

  .weak TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler,Default_Handler

  .weak TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak TIM18_IRQHandler
  .thumb_set TIM18_IRQHandler,Default_Handler

  .weak LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler,Default_Handler

  .weak LPTIM3_IRQHandler
  .thumb_set LPTIM3_IRQHandler,Default_Handler

  .weak LPTIM4_IRQHandler
  .thumb_set LPTIM4_IRQHandler,Default_Handler

  .weak LPTIM5_IRQHandler
  .thumb_set LPTIM5_IRQHandler,Default_Handler

  .weak ADF1_FLT0_IRQHandler
  .thumb_set ADF1_FLT0_IRQHandler,Default_Handler

  .weak MDF1_FLT0_IRQHandler
  .thumb_set MDF1_FLT0_IRQHandler,Default_Handler

  .weak MDF1_FLT1_IRQHandler
  .thumb_set MDF1_FLT1_IRQHandler,Default_Handler

  .weak MDF1_FLT2_IRQHandler
  .thumb_set MDF1_FLT2_IRQHandler,Default_Handler

  .weak MDF1_FLT3_IRQHandler
  .thumb_set MDF1_FLT3_IRQHandler,Default_Handler

  .weak MDF1_FLT4_IRQHandler
  .thumb_set MDF1_FLT4_IRQHandler,Default_Handler

  .weak MDF1_FLT5_IRQHandler
  .thumb_set MDF1_FLT5_IRQHandler,Default_Handler

  .weak SAI1_A_IRQHandler
  .thumb_set SAI1_A_IRQHandler,Default_Handler

  .weak SAI1_B_IRQHandler
  .thumb_set SAI1_B_IRQHandler,Default_Handler

  .weak SAI2_A_IRQHandler
  .thumb_set SAI2_A_IRQHandler,Default_Handler

  .weak SAI2_B_IRQHandler
  .thumb_set SAI2_B_IRQHandler,Default_Handler

  .weak SPDIFRX1_IRQHandler
  .thumb_set SPDIFRX1_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler,Default_Handler

  .weak SPI5_IRQHandler
  .thumb_set SPI5_IRQHandler,Default_Handler

  .weak SPI6_IRQHandler
  .thumb_set SPI6_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler,Default_Handler

  .weak USART6_IRQHandler
  .thumb_set USART6_IRQHandler,Default_Handler

  .weak UART7_IRQHandler
  .thumb_set UART7_IRQHandler,Default_Handler

  .weak UART8_IRQHandler
  .thumb_set UART8_IRQHandler,Default_Handler

  .weak UART9_IRQHandler
  .thumb_set UART9_IRQHandler,Default_Handler

  .weak USART10_IRQHandler
  .thumb_set USART10_IRQHandler,Default_Handler

  .weak LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler,Default_Handler

  .weak XSPI1_IRQHandler
  .thumb_set XSPI1_IRQHandler,Default_Handler

  .weak XSPI2_IRQHandler
  .thumb_set XSPI2_IRQHandler,Default_Handler

  .weak XSPI3_IRQHandler
  .thumb_set XSPI3_IRQHandler,Default_Handler

  .weak FMC_IRQHandler
  .thumb_set FMC_IRQHandler,Default_Handler

  .weak SDMMC1_IRQHandler
  .thumb_set SDMMC1_IRQHandler,Default_Handler

  .weak SDMMC2_IRQHandler
  .thumb_set SDMMC2_IRQHandler,Default_Handler

  .weak UCPD1_IRQHandler
  .thumb_set UCPD1_IRQHandler,Default_Handler

  .weak USB1_OTG_HS_IRQHandler
  .thumb_set USB1_OTG_HS_IRQHandler,Default_Handler

  .weak USB2_OTG_HS_IRQHandler
  .thumb_set USB2_OTG_HS_IRQHandler,Default_Handler

  .weak ETH1_IRQHandler
  .thumb_set ETH1_IRQHandler,Default_Handler

  .weak FDCAN1_IT0_IRQHandler
  .thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

  .weak FDCAN1_IT1_IRQHandler
  .thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

  .weak FDCAN2_IT0_IRQHandler
  .thumb_set FDCAN2_IT0_IRQHandler,Default_Handler

  .weak FDCAN2_IT1_IRQHandler
  .thumb_set FDCAN2_IT1_IRQHandler,Default_Handler

  .weak FDCAN3_IT0_IRQHandler
  .thumb_set FDCAN3_IT0_IRQHandler,Default_Handler

  .weak FDCAN3_IT1_IRQHandler
  .thumb_set FDCAN3_IT1_IRQHandler,Default_Handler

  .weak FDCAN_CU_IRQHandler
  .thumb_set FDCAN_CU_IRQHandler,Default_Handler

  .weak MDIOS_IRQHandler
  .thumb_set MDIOS_IRQHandler,Default_Handler

  .weak DCMI_PSSI_IRQHandler
  .thumb_set DCMI_PSSI_IRQHandler,Default_Handler

  .weak WAKEUP_PIN_IRQHandler
  .thumb_set WAKEUP_PIN_IRQHandler,Default_Handler

  .weak CTI_INT0_IRQHandler
  .thumb_set CTI_INT0_IRQHandler,Default_Handler

  .weak CTI_INT1_IRQHandler
  .thumb_set CTI_INT1_IRQHandler,Default_Handler

  .weak LTDC_UP_IRQHandler
  .thumb_set LTDC_UP_IRQHandler,Default_Handler

  .weak LTDC_UP_ERR_IRQHandler
  .thumb_set LTDC_UP_ERR_IRQHandler,Default_Handler
