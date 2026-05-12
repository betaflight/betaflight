/*
 * X32M78x XIP-to-TCM Startup (GCC, Cortex-M7 Thumb-2)
 *
 * Boot flow (modelled after u-boot):
 *
 *   ENTRY(_start)          -- vectors.S   : vector table + branch to reset
 *        |
 *        v
 *      reset               -- start.S     : disable IRQ, set temp SP
 *        |
 *        +-- bl lowlevel_init             : SystemInit, cache  (C)
 *        |
 *        +-- bl _main                     : copy code to TCM, jump    (C)
 *
 * All boot-stage code runs XIP from NOR-Flash (0x1500_0000).
 * After _main relocates one contiguous runtime image into ITCM and
 * sets VTOR/MSP, execution continues at main() from ITCM at bus speed.
 */

    .syntax unified
    .arch   armv7e-m
    .fpu    fpv5-d16
    .thumb

.extern _app_estack
.extern main

/* ======================================================================
 * Section: .isr_vector_boot  (placed at FLASH base by linker script)
 *
 * Cortex-M boot vector: only two entries are needed
 *   [0] Initial SP  -- temporary stack in AHB_SRAM
 *   [1] Reset entry -- _start (== Reset_Handler)
 *
 * This is the equivalent of u-boot's vectors.S .vectors section.
 * ====================================================================== */

    .section .isr_vector_boot, "a", %progbits
    .type   _boot_vectors, %object
_boot_vectors:
    .word   _estack             /* [0] Initial Stack Pointer (AHB_SRAM top) */
    .word   _start              /* [1] Reset_Handler = _start               */
    .size   _boot_vectors, . - _boot_vectors

/* ======================================================================
 * Section: .text.reset  (boot code, runs from FLASH)
 *
 * _start -> reset -> lowlevel_init -> _main
 *
 * Equivalent to u-boot:
 *   vectors.S:  _start: b reset
 *   start.S:    reset: ... bl cpu_init_crit(->lowlevel_init) ... bl _main
 * ====================================================================== */

    .section .text.reset, "ax", %progbits
    .global _start
    .type   _start, %function
    .thumb_func

_start:
    /*
     * Cortex-M hardware has already:
     *   1. Loaded MSP from vector[0] (_estack -> AHB_SRAM top)
     *   2. Jumped to vector[1] (here)
     *
     * Unlike ARMv7-A (u-boot), no need to set SVC mode or clear CP15.
     * We go straight to reset, just like "b reset" in vectors.S.
     */
    b       reset

    .size   _start, . - _start

/* ---------------------------------------------------------------------- */

    .global reset
    .type   reset, %function
    .thumb_func

reset:
    /* ---- Step 1: Disable interrupts ----
     * Only PRIMASK is needed on Cortex-M.  Do NOT set FAULTMASK here:
     * FAULTMASK (cpsid f) masks ALL exceptions including configurable-
     * priority IRQs, and __enable_irq() (cpsie i) does NOT clear it.
     * Leaving FAULTMASK=1 would silently block every interrupt later. */
    cpsid   i

    /* ---- Step 2: Ensure SP is set (already done by HW from vector[0]) ---- */
    ldr     r0, =_estack
    msr     msp, r0

    /*
     * ---- Step 3: bl lowlevel_init ----
     *
     * In u-boot this is called via cpu_init_crit -> lowlevel_init.
     * Our lowlevel_init (in xcore/soc.c) does:
     *   - SystemInit()          : FPU enable, VTOR, TCM size config, PWR
     *   - cache_init()          : Invalidate + enable I-Cache & D-Cache
     *
     * Clock setup is deferred to main() after relocation.
     */
    bl      lowlevel_init

    /*
     * ---- Step 4: bl _main ----
     *
     * In u-boot, _main sets up the C runtime and calls board_init_r.
     * Our _main (in xcore/soc.c) does:
     *   - Copy one contiguous runtime image from FLASH -> ITCM
     *   - Copy .data          from FLASH -> AHB_SRAM
     *   - Copy .app_data      from FLASH -> DTCM
     *   - Zero .bss, .app_bss
     *   - Set VTOR to ITCM vector table
     *   - Set MSP to DTCM stack top (_app_estack)
     *   - Branch to main() running from ITCM
     */
    bl      _main

    /* Should never return. Hang if it does. */
_hang:
    b       _hang

    .size   reset, . - reset

/* ======================================================================
 * Default_Handler -- catch-all for unhandled interrupts
 *
 * Placed in .text.Default_Handler so it stays in FLASH (boot region)
 * and is available before TCM copy.
 * ====================================================================== */

    .section .text.Default_Handler, "ax", %progbits
    .global Default_Handler
    .type   Default_Handler, %function
    .thumb_func

Default_Handler:
    b       Default_Handler

    .size   Default_Handler, . - Default_Handler

/* =====================================================================
 * Vector table  (placed in .tcm_vector by the linker script)
 * ===================================================================== */

.section .tcm_vector, "a", %progbits
.align 2
.global g_pfnVectors_runtime
.type   g_pfnVectors_runtime, %object

g_pfnVectors_runtime:
  .word  _app_estack                    /* Initial stack pointer (DTCM top)                 */
  .word  main                           /* Reset handler equivalent: jump to main()         */
  .word  NMI_Handler                    /* NMI Handler                                      */
  .word  HardFault_Handler              /* Hard Fault Handler                               */
  .word  MemManage_Handler              /* MPU Fault Handler                                */
  .word  BusFault_Handler               /* Bus Fault Handler                                */
  .word  UsageFault_Handler             /* Usage Fault Handler                              */
  .word  0                              /* Reserved                                         */
  .word  0                              /* Reserved                                         */
  .word  0                              /* Reserved                                         */
  .word  0                              /* Reserved                                         */
  .word  SVC_Handler                    /* SVCall Handler                                   */
  .word  DebugMon_Handler               /* Debug Monitor Handler                            */
  .word  0                              /* Reserved                                         */
  .word  PendSV_Handler                 /* PendSV Handler                                   */
  .word  X32SysTick_Handler             /* X32 system tick                                  */
  /* External Interrupts */
  .word  WWDG1_IRQHandler              /* Window1 Watchdog interrupt                        */
  .word  PVD_IRQHandler                /* PVD through EXTI Line16 detection interrupt       */
  .word  RTC_TAMPER_IRQHandler         /* RTC Tamper/Timestamp/Overflow or LSE-CSS          */
  .word  RTC_WKUP_IRQHandler           /* RTC Wakeup timer through EXTI line 19 interrupt   */
  .word  RCC_IRQHandler                /* RCC interrupt                                     */
  .word  EXTI0_IRQHandler              /* EXTI Line 0 interrupt                             */
  .word  EXTI1_IRQHandler              /* EXTI Line 1 interrupt                             */
  .word  EXTI2_IRQHandler              /* EXTI Line 2 interrupt                             */
  .word  EXTI3_IRQHandler              /* EXTI Line 3 interrupt                             */
  .word  EXTI4_IRQHandler              /* EXTI Line 4 interrupt                             */
  .word  EXTI9_5_IRQHandler            /* EXTI Line[9:5] interrupt                          */
  .word  EXTI15_10_IRQHandler          /* EXTI Line[15:10] interrupt                        */
  .word  DMA1_Channel0_IRQHandler      /* DMA1 Channel 0                                    */
  .word  DMA1_Channel1_IRQHandler      /* DMA1 Channel 1                                    */
  .word  DMA1_Channel2_IRQHandler      /* DMA1 Channel 2                                    */
  .word  DMA1_Channel3_IRQHandler      /* DMA1 Channel 3                                    */
  .word  DMA1_Channel4_IRQHandler      /* DMA1 Channel 4                                    */
  .word  DMA1_Channel5_IRQHandler      /* DMA1 Channel 5                                    */
  .word  DMA1_Channel6_IRQHandler      /* DMA1 Channel 6                                    */
  .word  DMA1_Channel7_IRQHandler      /* DMA1 Channel 7                                    */
  .word  DMA2_Channel0_IRQHandler      /* DMA2 Channel 0                                    */
  .word  DMA2_Channel1_IRQHandler      /* DMA2 Channel 1                                    */
  .word  DMA2_Channel2_IRQHandler      /* DMA2 Channel 2                                    */
  .word  DMA2_Channel3_IRQHandler      /* DMA2 Channel 3                                    */
  .word  DMA2_Channel4_IRQHandler      /* DMA2 Channel 4                                    */
  .word  DMA2_Channel5_IRQHandler      /* DMA2 Channel 5                                    */
  .word  DMA2_Channel6_IRQHandler      /* DMA2 Channel 6                                    */
  .word  DMA2_Channel7_IRQHandler      /* DMA2 Channel 7                                    */
  .word  DMA3_Channel0_IRQHandler      /* DMA3 Channel 0                                    */
  .word  DMA3_Channel1_IRQHandler      /* DMA3 Channel 1                                    */
  .word  DMA3_Channel2_IRQHandler      /* DMA3 Channel 2                                    */
  .word  DMA3_Channel3_IRQHandler      /* DMA3 Channel 3                                    */
  .word  DMA3_Channel4_IRQHandler      /* DMA3 Channel 4                                    */
  .word  DMA3_Channel5_IRQHandler      /* DMA3 Channel 5                                    */
  .word  DMA3_Channel6_IRQHandler      /* DMA3 Channel 6                                    */
  .word  DMA3_Channel7_IRQHandler      /* DMA3 Channel 7                                    */
  .word  MDMA_Channel0_IRQHandler      /* MDMA Channel 0                                    */
  .word  MDMA_Channel1_IRQHandler      /* MDMA Channel 1                                    */
  .word  MDMA_Channel2_IRQHandler      /* MDMA Channel 2                                    */
  .word  MDMA_Channel3_IRQHandler      /* MDMA Channel 3                                    */
  .word  MDMA_Channel4_IRQHandler      /* MDMA Channel 4                                    */
  .word  MDMA_Channel5_IRQHandler      /* MDMA Channel 5                                    */
  .word  MDMA_Channel6_IRQHandler      /* MDMA Channel 6                                    */
  .word  MDMA_Channel7_IRQHandler      /* MDMA Channel 7                                    */
  .word  MDMA_Channel8_IRQHandler      /* MDMA Channel 8                                    */
  .word  MDMA_Channel9_IRQHandler      /* MDMA Channel 9                                    */
  .word  MDMA_Channel10_IRQHandler     /* MDMA Channel 10                                   */
  .word  MDMA_Channel11_IRQHandler     /* MDMA Channel 11                                   */
  .word  MDMA_Channel12_IRQHandler     /* MDMA Channel 12                                   */
  .word  MDMA_Channel13_IRQHandler     /* MDMA Channel 13                                   */
  .word  MDMA_Channel14_IRQHandler     /* MDMA Channel 14                                   */
  .word  MDMA_Channel15_IRQHandler     /* MDMA Channel 15                                   */
  .word  SDPU_IRQHandler               /* SDPU global interrupt                             */
  .word  0                             /* Reserved                                          */
  .word  0                             /* Reserved                                          */
  .word  FPU_CPU1_IRQHandler           /* FPU_CM7 global interrupt                          */
  .word  ECCMON_IRQHandler             /* ECCMON global interrupt                           */
  .word  RTC_ALARM_IRQHandler          /* RTC Alarm via EXTI17 interrupt                    */
  .word  I2C1_EV_IRQHandler            /* I2C1 event interrupt                              */
  .word  I2C1_ER_IRQHandler            /* I2C1 error interrupt                              */
  .word  I2C2_EV_IRQHandler            /* I2C2 event interrupt                              */
  .word  I2C2_ER_IRQHandler            /* I2C2 error interrupt                              */
  .word  I2C3_EV_IRQHandler            /* I2C3 event interrupt                              */
  .word  I2C3_ER_IRQHandler            /* I2C3 error interrupt                              */
  .word  I2C4_EV_IRQHandler            /* I2C4 event interrupt                              */
  .word  I2C4_ER_IRQHandler            /* I2C4 error interrupt                              */
  .word  I2C5_EV_IRQHandler            /* I2C5 event interrupt                              */
  .word  I2C5_ER_IRQHandler            /* I2C5 error interrupt                              */
  .word  I2C6_EV_IRQHandler            /* I2C6 event interrupt                              */
  .word  I2C6_ER_IRQHandler            /* I2C6 error interrupt                              */
  .word  I2C7_EV_IRQHandler            /* I2C7 event interrupt                              */
  .word  I2C7_ER_IRQHandler            /* I2C7 error interrupt                              */
  .word  I2C8_EV_IRQHandler            /* I2C8 event interrupt                              */
  .word  I2C8_ER_IRQHandler            /* I2C8 error interrupt                              */
  .word  I2C9_EV_IRQHandler            /* I2C9 event interrupt                              */
  .word  I2C9_ER_IRQHandler            /* I2C9 error interrupt                              */
  .word  I2C10_EV_IRQHandler           /* I2C10 event interrupt                             */
  .word  I2C10_ER_IRQHandler           /* I2C10 error interrupt                             */
  .word  I2S1_IRQHandler               /* I2S1 global interrupt                             */
  .word  I2S2_IRQHandler               /* I2S1 global interrupt                             */
  .word  I2S3_IRQHandler               /* I2S1 global interrupt                             */
  .word  I2S4_IRQHandler             /* I2S1 global interrupt                             */
  .word  0                             /* Reserved (SDK startup: slot before xSPI2)         */
  .word  xSPI2_IRQHandler             /* xSPI2 global interrupt                            */
  .word  SPI1_IRQHandler               /* SPI1 global interrupt                             */
  .word  SPI2_IRQHandler               /* SPI2 global interrupt                             */
  .word  SPI3_IRQHandler               /* SPI3 global interrupt                             */
  .word  SPI4_IRQHandler               /* SPI4 global interrupt                             */
  .word  SPI5_IRQHandler               /* SPI5 global interrupt                             */
  .word  SPI6_IRQHandler               /* SPI6 global interrupt                             */
  .word  SPI7_IRQHandler               /* SPI7 global interrupt                             */
  .word  LCD_EV_IRQHandler             /* TFT LCD Controller event interrupt                */
  .word  LCD_ER_IRQHandler             /* TFT LCD Controller error interrupt                */
  .word  DVP1_IRQHandler               /* DVP1 global interrupt                             */
  .word  DVP2_IRQHandler               /* DVP2 global interrupt                             */
  .word  DMAMUX2_IRQHandler            /* DMAMUX2 (MDMA MUX) global interrupt               */
  .word  USB1_HS_EPx_OUT_IRQHandler    /* USB1_HS endpoint OUT global interrupt             */
  .word  USB1_HS_EPx_IN_IRQHandler     /* USB1_HS endpoint IN global interrupt              */
  .word  USB1_HS_WKUP_IRQHandler       /* USB1_HS WKUP through EXTI line 62                 */
  .word  USB1_HS_IRQHandler            /* USB1_HS global interrupt                          */
  .word  USB2_HS_EPx_OUT_IRQHandler    /* USB2_HS endpoint OUT global interrupt             */
  .word  USB2_HS_EPx_IN_IRQHandler     /* USB2_HS endpoint IN global interrupt              */
  .word  USB2_HS_WKUP_IRQHandler       /* USB2_HS WKUP through EXTI line 63                 */
  .word  USB2_HS_IRQHandler            /* USB2_HS global interrupt                          */
  .word  ETH1_IRQHandler               /* Ethernet 1 global interrupt                       */
  .word  ETH1_PMT_LPI_IRQHandler       /* Ethernet1 PMT/LPI through EXTI line 83            */
  .word  ETH2_IRQHandler               /* Ethernet 2 global interrupt                       */
  .word  ETH2_PMT_LPI_IRQHandler       /* Ethernet2 PMT/LPI through EXTI line 84            */
  .word  FDCAN1_INT0_IRQHandler        /* FDCAN1 global interrupt line 0                    */
  .word  FDCAN2_INT0_IRQHandler        /* FDCAN2 global interrupt line 0                    */
  .word  FDCAN3_INT0_IRQHandler        /* FDCAN3 global interrupt line 0                    */
  .word  FDCAN4_INT0_IRQHandler        /* FDCAN4 global interrupt line 0                    */
  .word  FDCAN1_INT1_IRQHandler        /* FDCAN1 global interrupt line 1                    */
  .word  FDCAN2_INT1_IRQHandler        /* FDCAN2 global interrupt line 1                    */
  .word  FDCAN3_INT1_IRQHandler        /* FDCAN3 global interrupt line 1                    */
  .word  FDCAN4_INT1_IRQHandler        /* FDCAN4 global interrupt line 1                    */
  .word  USART1_IRQHandler             /* USART1 global interrupt                           */
  .word  USART2_IRQHandler             /* USART2 global interrupt                           */
  .word  USART3_IRQHandler             /* USART3 global interrupt                           */
  .word  USART4_IRQHandler             /* USART4 global interrupt                           */
  .word  USART5_IRQHandler             /* USART5 global interrupt                           */
  .word  USART6_IRQHandler             /* USART6 global interrupt                           */
  .word  USART7_IRQHandler             /* USART7 global interrupt                           */
  .word  USART8_IRQHandler             /* USART8 global interrupt                           */
  .word  UART9_IRQHandler              /* UART9 global interrupt                            */
  .word  UART10_IRQHandler             /* UART10 global interrupt                           */
  .word  UART11_IRQHandler             /* UART11 global interrupt                           */
  .word  UART12_IRQHandler             /* UART12 global interrupt                           */
  .word  UART13_IRQHandler             /* UART13 global interrupt                           */
  .word  UART14_IRQHandler             /* UART14 global interrupt                           */
  .word  UART15_IRQHandler             /* UART15 global interrupt                           */
  .word  LPUART1_IRQHandler            /* LPUART1 global interrupt + wakeup through EXTI 49 */
  .word  LPUART2_IRQHandler            /* LPUART2 global interrupt + wakeup through EXTI 52 */
  .word  GPU_IRQHandler                /* GPU global interrupt                              */
  .word  0                             /* Reserved                                          */
  .word  SDMMC1_IRQHandler             /* SDMMC1_IRQ + WKUP through EXTI line 24            */
  .word  SDMMC2_IRQHandler             /* SDMMC2_IRQ + WKUP through EXTI line 25            */
  .word  ADC1_IRQHandler               /* ADC1 global interrupt                             */
  .word  ADC2_IRQHandler               /* ADC2 global interrupt                             */
  .word  ADC3_IRQHandler               /* ADC3 global interrupt                             */
  .word  COMP1_2_IRQHandler            /* COMP1 and COMP2 through EXTI line 20 and 21       */
  .word  COMP3_4_IRQHandler            /* COMP3 and COMP4 through EXTI line 22 and 23       */
  .word  SHRTIM1_INT1_IRQHandler       /* High Resolution timer 1 interrupt 1               */
  .word  SHRTIM1_INT2_IRQHandler       /* High Resolution timer 1 interrupt 2               */
  .word  SHRTIM1_INT3_IRQHandler       /* High Resolution timer 1 interrupt 3               */
  .word  SHRTIM1_INT4_IRQHandler       /* High Resolution timer 1 interrupt 4               */
  .word  SHRTIM1_INT5_IRQHandler       /* High Resolution timer 1 interrupt 5               */
  .word  SHRTIM1_INT6_IRQHandler       /* High Resolution timer 1 interrupt 6               */
  .word  SHRTIM1_INT7_IRQHandler       /* High Resolution timer 1 interrupt 7               */
  .word  SHRTIM1_INT8_IRQHandler       /* High Resolution timer 1 interrupt 8               */
  .word  SHRTIM2_INT1_IRQHandler       /* High Resolution timer 2 interrupt 1               */
  .word  SHRTIM2_INT2_IRQHandler       /* High Resolution timer 2 interrupt 2               */
  .word  SHRTIM2_INT3_IRQHandler       /* High Resolution timer 2 interrupt 3               */
  .word  SHRTIM2_INT4_IRQHandler       /* High Resolution timer 2 interrupt 4               */
  .word  SHRTIM2_INT5_IRQHandler       /* High Resolution timer 2 interrupt 5               */
  .word  SHRTIM2_INT6_IRQHandler       /* High Resolution timer 2 interrupt 6               */
  .word  SHRTIM2_INT7_IRQHandler       /* High Resolution timer 2 interrupt 7               */
  .word  SHRTIM2_INT8_IRQHandler       /* High Resolution timer 2 interrupt 8               */
  .word  FDCAN5_INT0_IRQHandler        /* FDCAN5 global interrupt line 0                    */
  .word  FDCAN6_INT0_IRQHandler        /* FDCAN6 global interrupt line 0                    */
  .word  FDCAN7_INT0_IRQHandler        /* FDCAN7 global interrupt line 0                    */
  .word  FDCAN8_INT0_IRQHandler        /* FDCAN8 global interrupt line 0                    */
  .word  FDCAN5_INT1_IRQHandler        /* FDCAN5 global interrupt line 1                    */
  .word  FDCAN6_INT1_IRQHandler        /* FDCAN6 global interrupt line 1                    */
  .word  FDCAN7_INT1_IRQHandler        /* FDCAN7 global interrupt line 1                    */
  .word  FDCAN8_INT1_IRQHandler        /* FDCAN8 global interrupt line 1                    */
  .word  DSI_IRQHandler                /* MIPI DSI interrupt through EXTI line 87             */
  .word  0                             /* Reserved                                          */
  .word  LPTIM5_WKUP_IRQHandler        /* LPTIM5 wakeup through EXTI 86                     */
  .word  JPEG_SGDMA_H2P_IRQHandler     /* JPEG SGDMA Host to Peripheral interrupt            */
  .word  JPEG_SGDMA_P2H_IRQHandler     /* JPEG SGDMA Peripheral to Host interrupt           */
  .word  WAKEUP_IO_IRQHandler          /* 6 WAKEUP IOs through EXTI line 70-75              */
  .word  SEMA4_INT1_IRQHandler         /* SEMA4 interrupt1                                  */
  .word  0                             /* Reserved                                          */
  .word  WWDG2_RST_IRQHandler          /* WWDG2 reset interrupt through EXTI line 82        */
  .word  OTPC_IRQHandler               /* OTPC interrupt                                    */
  .word  FEMC_IRQHandler               /* FEMC interrupt                                    */
  .word  DCMUB_IRQHandler              /* DCMUB interrupt                                   */
  .word  DAC1_IRQHandler               /* DAC1 interrupt                                    */
  .word  DAC2_IRQHandler               /* DAC2 interrupt                                    */
  .word  MDMA_AHBS_ER_IRQHandler       /* MDMA HABS ERROR through EXTI line 55-56           */
  .word  CM7_CATCH_READ_ER_IRQHandler  /* CM7 error on cache read through EXTI line 64-65   */
  .word  DAC3_IRQHandler               /* DAC3 interrupt                                    */
  .word  DAC4_IRQHandler               /* DAC4 interrupt                                    */
  .word  EMC_IRQHandler                /* EMC event interrupt through EXTI line 88-89       */
  .word  DAC5_IRQHandler               /* DAC5 interrupt                                    */
  .word  DAC6_IRQHandler               /* DAC6 interrupt                                    */
  .word  ESC_OPB_IRQHandler            /* ETHERCAT OPB interrupt                            */
  .word  ESC_SYNC0_IRQHandler          /* ETHERCAT SYNC0 interrupt                          */
  .word  ESC_SYNC1_IRQHandler          /* ETHERCAT SYNC1 interrupt                          */
  .word  ESC_WRP_IRQHandler            /* ETHERCAT WRAPPER interrupt                        */
  .word  0                             /* Reserved                                          */
  .word  ATIM1_BRK_IRQHandler          /* Advanced timer 1 break interrupt                  */
  .word  ATIM1_TRG_COM_IRQHandler      /* Advanced timer 1 trigger/commutation interrupt    */
  .word  ATIM1_CC_IRQHandler           /* Advanced timer 1 capture/compare interrupt        */
  .word  ATIM1_UP_IRQHandler           /* Advanced timer 1 update interrupt                 */
  .word  ATIM2_BRK_IRQHandler          /* Advanced timer 2 break interrupt                  */
  .word  ATIM2_TRG_COM_IRQHandler      /* advanced timer 2 trigger and commutation IRQs   */
  .word  ATIM2_CC_IRQHandler           /* Advanced timer 2 capture/compare interrupt        */
  .word  ATIM2_UP_IRQHandler           /* Advanced timer 2 update interrupt                 */
  .word  ATIM3_BRK_IRQHandler          /* Advanced timer 3 break interrupt                  */
  .word  ATIM3_TRG_COM_IRQHandler      /* Advanced timer 3 trigger and commutation IRQs     */
  .word  ATIM3_CC_IRQHandler           /* Advanced timer 3 capture/compare interrupt        */
  .word  ATIM3_UP_IRQHandler           /* Advanced timer 3 update interrupt                 */
  .word  ATIM4_BRK_IRQHandler          /* Advanced timer 4 break interrupt                  */
  .word  ATIM4_TRG_COM_IRQHandler      /* Advanced timer 4 trigger and commutation IRQs     */
  .word  ATIM4_CC_IRQHandler           /* Advanced timer 4 capture/compare interrupt        */
  .word  ATIM4_UP_IRQHandler           /* Advanced timer 4 update interrupt                 */
  .word  GTIMA1_IRQHandler             /* General timer A1 global interrupt                 */
  .word  GTIMA2_IRQHandler             /* General timer A2 global interrupt                 */
  .word  GTIMA3_IRQHandler             /* General timer A3 global interrupt                 */
  .word  GTIMA4_IRQHandler             /* General timer A4 global interrupt                 */
  .word  GTIMA5_IRQHandler             /* General timer A5 global interrupt                 */
  .word  GTIMA6_IRQHandler             /* General timer A6 global interrupt                 */
  .word  GTIMA7_IRQHandler             /* General timer A7 global interrupt                 */
  .word  GTIMB1_IRQHandler             /* General timer B1 global interrupt                 */
  .word  GTIMB2_IRQHandler             /* General timer B2 global interrupt                 */
  .word  GTIMB3_IRQHandler             /* General timer B3 global interrupt                 */
  .word  BTIM1_IRQHandler              /* Base timer 1 global interrupt                     */
  .word  BTIM2_IRQHandler              /* Base timer 2 global interrupt                     */
  .word  BTIM3_IRQHandler              /* Base timer 3 global interrupt                     */
  .word  BTIM4_IRQHandler              /* Base timer 4 global interrupt                     */
  .word  LPTIM1_WKUP_IRQHandler        /* LPTIM1 wakeup interrupt                           */
  .word  LPTIM2_WKUP_IRQHandler        /* LPTIM2 wakeup interrupt                           */
  .word  LPTIM3_WKUP_IRQHandler        /* LPTIM3 wakeup interrupt                           */
  .word  LPTIM4_WKUP_IRQHandler        /* LPTIM4 wakeup interrupt                           */
  .word  DSMU_FLT0_IRQHandler          /* DSMU Filter interrupt 0                           */
  .word  DSMU_FLT1_IRQHandler          /* DSMU Filter interrupt 1                           */
  .word  DSMU_FLT2_IRQHandler          /* DSMU Filter interrupt 2                           */
  .word  DSMU_FLT3_IRQHandler          /* DSMU Filter interrupt 3                           */
  .word  FMAC_IRQHandler               /* FMAC global interrupt                             */
  .word  CORDIC_IRQHandler             /* Cordic global interrupt                           */
  .word  DMAMUX1_IRQHandler            /* DMAMUX1 interrupt                                 */
  .word  MMU_IRQHandler                /* MMU interrupt                                     */
  .word  SysTick_Handler               /* SysTick Handler                                   */

  /* need check to user manual of exti chapter   */

  .size  g_pfnVectors_runtime, . - g_pfnVectors_runtime

/* =====================================================================
 * Weak default handlers
 *
 * Each handler is a real function in its own section (.text.<name>).
 * The vector table .word generates a relocation to the handler SYMBOL
 * (not to Default_Handler), preserving the link-time override path
 * and working correctly with --gc-sections.
 *
 * NOTE: Do NOT use .thumb_set here.  .thumb_set creates an alias
 *       resolved at assembly time, causing the .word to reference
 *       Default_Handler directly -- breaking weak override.
 * ===================================================================== */

.macro WEAK_HANDLER handler
    .section .text.\handler, "ax", %progbits
    .weak   \handler
    .type   \handler, %function
    .thumb_func
\handler:
    b       Default_Handler
    .size   \handler, . - \handler
.endm

  WEAK_HANDLER  NMI_Handler
  WEAK_HANDLER  HardFault_Handler
  WEAK_HANDLER  MemManage_Handler
  WEAK_HANDLER  BusFault_Handler
  WEAK_HANDLER  UsageFault_Handler
  WEAK_HANDLER  SVC_Handler
  WEAK_HANDLER  DebugMon_Handler
  WEAK_HANDLER  PendSV_Handler
  WEAK_HANDLER  X32SysTick_Handler
  WEAK_HANDLER  WWDG1_IRQHandler
  WEAK_HANDLER  PVD_IRQHandler
  WEAK_HANDLER  RTC_TAMPER_IRQHandler
  WEAK_HANDLER  RTC_WKUP_IRQHandler
  WEAK_HANDLER  RCC_IRQHandler
  WEAK_HANDLER  EXTI0_IRQHandler
  WEAK_HANDLER  EXTI1_IRQHandler
  WEAK_HANDLER  EXTI2_IRQHandler
  WEAK_HANDLER  EXTI3_IRQHandler
  WEAK_HANDLER  EXTI4_IRQHandler
  WEAK_HANDLER  EXTI9_5_IRQHandler
  WEAK_HANDLER  EXTI15_10_IRQHandler
  WEAK_HANDLER  DMA1_Channel0_IRQHandler
  WEAK_HANDLER  DMA1_Channel1_IRQHandler
  WEAK_HANDLER  DMA1_Channel2_IRQHandler
  WEAK_HANDLER  DMA1_Channel3_IRQHandler
  WEAK_HANDLER  DMA1_Channel4_IRQHandler
  WEAK_HANDLER  DMA1_Channel5_IRQHandler
  WEAK_HANDLER  DMA1_Channel6_IRQHandler
  WEAK_HANDLER  DMA1_Channel7_IRQHandler
  WEAK_HANDLER  DMA2_Channel0_IRQHandler
  WEAK_HANDLER  DMA2_Channel1_IRQHandler
  WEAK_HANDLER  DMA2_Channel2_IRQHandler
  WEAK_HANDLER  DMA2_Channel3_IRQHandler
  WEAK_HANDLER  DMA2_Channel4_IRQHandler
  WEAK_HANDLER  DMA2_Channel5_IRQHandler
  WEAK_HANDLER  DMA2_Channel6_IRQHandler
  WEAK_HANDLER  DMA2_Channel7_IRQHandler
  WEAK_HANDLER  DMA3_Channel0_IRQHandler
  WEAK_HANDLER  DMA3_Channel1_IRQHandler
  WEAK_HANDLER  DMA3_Channel2_IRQHandler
  WEAK_HANDLER  DMA3_Channel3_IRQHandler
  WEAK_HANDLER  DMA3_Channel4_IRQHandler
  WEAK_HANDLER  DMA3_Channel5_IRQHandler
  WEAK_HANDLER  DMA3_Channel6_IRQHandler
  WEAK_HANDLER  DMA3_Channel7_IRQHandler
  WEAK_HANDLER  MDMA_Channel0_IRQHandler
  WEAK_HANDLER  MDMA_Channel1_IRQHandler
  WEAK_HANDLER  MDMA_Channel2_IRQHandler
  WEAK_HANDLER  MDMA_Channel3_IRQHandler
  WEAK_HANDLER  MDMA_Channel4_IRQHandler
  WEAK_HANDLER  MDMA_Channel5_IRQHandler
  WEAK_HANDLER  MDMA_Channel6_IRQHandler
  WEAK_HANDLER  MDMA_Channel7_IRQHandler
  WEAK_HANDLER  MDMA_Channel8_IRQHandler
  WEAK_HANDLER  MDMA_Channel9_IRQHandler
  WEAK_HANDLER  MDMA_Channel10_IRQHandler
  WEAK_HANDLER  MDMA_Channel11_IRQHandler
  WEAK_HANDLER  MDMA_Channel12_IRQHandler
  WEAK_HANDLER  MDMA_Channel13_IRQHandler
  WEAK_HANDLER  MDMA_Channel14_IRQHandler
  WEAK_HANDLER  MDMA_Channel15_IRQHandler
  WEAK_HANDLER  SDPU_IRQHandler
  WEAK_HANDLER  FPU_CPU1_IRQHandler
  WEAK_HANDLER  ECCMON_IRQHandler
  WEAK_HANDLER  RTC_ALARM_IRQHandler
  WEAK_HANDLER  I2C1_EV_IRQHandler
  WEAK_HANDLER  I2C1_ER_IRQHandler
  WEAK_HANDLER  I2C2_EV_IRQHandler
  WEAK_HANDLER  I2C2_ER_IRQHandler
  WEAK_HANDLER  I2C3_EV_IRQHandler
  WEAK_HANDLER  I2C3_ER_IRQHandler
  WEAK_HANDLER  I2C4_EV_IRQHandler
  WEAK_HANDLER  I2C4_ER_IRQHandler
  WEAK_HANDLER  I2C5_EV_IRQHandler
  WEAK_HANDLER  I2C5_ER_IRQHandler
  WEAK_HANDLER  I2C6_EV_IRQHandler
  WEAK_HANDLER  I2C6_ER_IRQHandler
  WEAK_HANDLER  I2C7_EV_IRQHandler
  WEAK_HANDLER  I2C7_ER_IRQHandler
  WEAK_HANDLER  I2C8_EV_IRQHandler
  WEAK_HANDLER  I2C8_ER_IRQHandler
  WEAK_HANDLER  I2C9_EV_IRQHandler
  WEAK_HANDLER  I2C9_ER_IRQHandler
  WEAK_HANDLER  I2C10_EV_IRQHandler
  WEAK_HANDLER  I2C10_ER_IRQHandler
  WEAK_HANDLER  I2S1_IRQHandler
  WEAK_HANDLER  I2S2_IRQHandler
  WEAK_HANDLER  I2S3_IRQHandler
  WEAK_HANDLER  I2S4_IRQHandler
  WEAK_HANDLER  xSPI1_IRQHandler
  WEAK_HANDLER  xSPI2_IRQHandler
  WEAK_HANDLER  SPI1_IRQHandler
  WEAK_HANDLER  SPI2_IRQHandler
  WEAK_HANDLER  SPI3_IRQHandler
  WEAK_HANDLER  SPI4_IRQHandler
  WEAK_HANDLER  SPI5_IRQHandler
  WEAK_HANDLER  SPI6_IRQHandler
  WEAK_HANDLER  SPI7_IRQHandler
  WEAK_HANDLER  LCD_EV_IRQHandler
  WEAK_HANDLER  LCD_ER_IRQHandler
  WEAK_HANDLER  DVP1_IRQHandler
  WEAK_HANDLER  DVP2_IRQHandler
  WEAK_HANDLER  DMAMUX2_IRQHandler
  WEAK_HANDLER  USB1_HS_EPx_OUT_IRQHandler
  WEAK_HANDLER  USB1_HS_EPx_IN_IRQHandler
  WEAK_HANDLER  USB1_HS_WKUP_IRQHandler
  WEAK_HANDLER  USB1_HS_IRQHandler
  WEAK_HANDLER  USB2_HS_EPx_OUT_IRQHandler
  WEAK_HANDLER  USB2_HS_EPx_IN_IRQHandler
  WEAK_HANDLER  USB2_HS_WKUP_IRQHandler
  WEAK_HANDLER  USB2_HS_IRQHandler
  WEAK_HANDLER  ETH1_IRQHandler
  WEAK_HANDLER  ETH1_PMT_LPI_IRQHandler
  WEAK_HANDLER  ETH2_IRQHandler
  WEAK_HANDLER  ETH2_PMT_LPI_IRQHandler
  WEAK_HANDLER  FDCAN1_INT0_IRQHandler
  WEAK_HANDLER  FDCAN2_INT0_IRQHandler
  WEAK_HANDLER  FDCAN3_INT0_IRQHandler
  WEAK_HANDLER  FDCAN4_INT0_IRQHandler
  WEAK_HANDLER  FDCAN1_INT1_IRQHandler
  WEAK_HANDLER  FDCAN2_INT1_IRQHandler
  WEAK_HANDLER  FDCAN3_INT1_IRQHandler
  WEAK_HANDLER  FDCAN4_INT1_IRQHandler
  WEAK_HANDLER  USART1_IRQHandler
  WEAK_HANDLER  USART2_IRQHandler
  WEAK_HANDLER  USART3_IRQHandler
  WEAK_HANDLER  USART4_IRQHandler
  WEAK_HANDLER  USART5_IRQHandler
  WEAK_HANDLER  USART6_IRQHandler
  WEAK_HANDLER  USART7_IRQHandler
  WEAK_HANDLER  USART8_IRQHandler
  WEAK_HANDLER  UART9_IRQHandler
  WEAK_HANDLER  UART10_IRQHandler
  WEAK_HANDLER  UART11_IRQHandler
  WEAK_HANDLER  UART12_IRQHandler
  WEAK_HANDLER  UART13_IRQHandler
  WEAK_HANDLER  UART14_IRQHandler
  WEAK_HANDLER  UART15_IRQHandler
  WEAK_HANDLER  LPUART1_IRQHandler
  WEAK_HANDLER  LPUART2_IRQHandler
  WEAK_HANDLER  GPU_IRQHandler
  WEAK_HANDLER  SDMMC1_IRQHandler
  WEAK_HANDLER  SDMMC2_IRQHandler
  WEAK_HANDLER  ADC1_IRQHandler
  WEAK_HANDLER  ADC2_IRQHandler
  WEAK_HANDLER  ADC3_IRQHandler
  WEAK_HANDLER  COMP1_2_IRQHandler
  WEAK_HANDLER  COMP3_4_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT1_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT2_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT3_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT4_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT5_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT6_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT7_IRQHandler
  WEAK_HANDLER  SHRTIM1_INT8_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT1_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT2_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT3_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT4_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT5_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT6_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT7_IRQHandler
  WEAK_HANDLER  SHRTIM2_INT8_IRQHandler
  WEAK_HANDLER  FDCAN5_INT0_IRQHandler
  WEAK_HANDLER  FDCAN6_INT0_IRQHandler
  WEAK_HANDLER  FDCAN7_INT0_IRQHandler
  WEAK_HANDLER  FDCAN8_INT0_IRQHandler
  WEAK_HANDLER  FDCAN5_INT1_IRQHandler
  WEAK_HANDLER  FDCAN6_INT1_IRQHandler
  WEAK_HANDLER  FDCAN7_INT1_IRQHandler
  WEAK_HANDLER  FDCAN8_INT1_IRQHandler
  WEAK_HANDLER  DSI_IRQHandler
  WEAK_HANDLER  LPTIM5_WKUP_IRQHandler
  WEAK_HANDLER  JPEG_SGDMA_H2P_IRQHandler
  WEAK_HANDLER  JPEG_SGDMA_P2H_IRQHandler
  WEAK_HANDLER  WAKEUP_IO_IRQHandler
  WEAK_HANDLER  SEMA4_INT1_IRQHandler
  WEAK_HANDLER  WWDG2_RST_IRQHandler
  WEAK_HANDLER  OTPC_IRQHandler
  WEAK_HANDLER  FEMC_IRQHandler
  WEAK_HANDLER  DCMUB_IRQHandler
  WEAK_HANDLER  DAC1_IRQHandler
  WEAK_HANDLER  DAC2_IRQHandler
  WEAK_HANDLER  MDMA_AHBS_ER_IRQHandler
  WEAK_HANDLER  CM7_CATCH_READ_ER_IRQHandler
  WEAK_HANDLER  DAC3_IRQHandler
  WEAK_HANDLER  DAC4_IRQHandler
  WEAK_HANDLER  EMC_IRQHandler
  WEAK_HANDLER  DAC5_IRQHandler
  WEAK_HANDLER  DAC6_IRQHandler
  WEAK_HANDLER  ESC_OPB_IRQHandler
  WEAK_HANDLER  ESC_SYNC0_IRQHandler
  WEAK_HANDLER  ESC_SYNC1_IRQHandler
  WEAK_HANDLER  ESC_WRP_IRQHandler
  WEAK_HANDLER  ATIM1_BRK_IRQHandler
  WEAK_HANDLER  ATIM1_TRG_COM_IRQHandler
  WEAK_HANDLER  ATIM1_CC_IRQHandler
  WEAK_HANDLER  ATIM1_UP_IRQHandler
  WEAK_HANDLER  ATIM2_BRK_IRQHandler
  WEAK_HANDLER  ATIM2_TRG_COM_IRQHandler
  WEAK_HANDLER  ATIM2_CC_IRQHandler
  WEAK_HANDLER  ATIM2_UP_IRQHandler
  WEAK_HANDLER  ATIM3_BRK_IRQHandler
  WEAK_HANDLER  ATIM3_TRG_COM_IRQHandler
  WEAK_HANDLER  ATIM3_CC_IRQHandler
  WEAK_HANDLER  ATIM3_UP_IRQHandler
  WEAK_HANDLER  ATIM4_BRK_IRQHandler
  WEAK_HANDLER  ATIM4_TRG_COM_IRQHandler
  WEAK_HANDLER  ATIM4_CC_IRQHandler
  WEAK_HANDLER  ATIM4_UP_IRQHandler
  WEAK_HANDLER  GTIMA1_IRQHandler
  WEAK_HANDLER  GTIMA2_IRQHandler
  WEAK_HANDLER  GTIMA3_IRQHandler
  WEAK_HANDLER  GTIMA4_IRQHandler
  WEAK_HANDLER  GTIMA5_IRQHandler
  WEAK_HANDLER  GTIMA6_IRQHandler
  WEAK_HANDLER  GTIMA7_IRQHandler
  WEAK_HANDLER  GTIMB1_IRQHandler
  WEAK_HANDLER  GTIMB2_IRQHandler
  WEAK_HANDLER  GTIMB3_IRQHandler
  WEAK_HANDLER  BTIM1_IRQHandler
  WEAK_HANDLER  BTIM2_IRQHandler
  WEAK_HANDLER  BTIM3_IRQHandler
  WEAK_HANDLER  BTIM4_IRQHandler
  WEAK_HANDLER  LPTIM1_WKUP_IRQHandler
  WEAK_HANDLER  LPTIM2_WKUP_IRQHandler
  WEAK_HANDLER  LPTIM3_WKUP_IRQHandler
  WEAK_HANDLER  LPTIM4_WKUP_IRQHandler
  WEAK_HANDLER  DSMU_FLT0_IRQHandler
  WEAK_HANDLER  DSMU_FLT1_IRQHandler
  WEAK_HANDLER  DSMU_FLT2_IRQHandler
  WEAK_HANDLER  DSMU_FLT3_IRQHandler
  WEAK_HANDLER  FMAC_IRQHandler
  WEAK_HANDLER  CORDIC_IRQHandler
  WEAK_HANDLER  DMAMUX1_IRQHandler
  WEAK_HANDLER  MMU_IRQHandler
  WEAK_HANDLER  SysTick_Handler

.end
