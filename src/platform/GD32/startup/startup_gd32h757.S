
  .syntax unified
  .cpu cortex-m7
  .fpu fpv5-sp-d16
  .thumb

.global  Default_Handler

/* necessary symbols defined in linker script to initialize data */
.word  _sidata
.word  _sdata
.word  _edata
.word  _sbss
.word  _ebss
.word  _ssram2
.word  _esram2
.word  __fastram_bss_start__
.word  __fastram_bss_end__

  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function

/* reset Handler */
Reset_Handler:
    LDR     r1, =0x24000000
    LDR     r2, =0x80000  /* 512K AXI SRAM */
    MOV     r0, #0x00
SRAM_INIT:
	STM     r1!, {r0}
	SUBS    r2, r2, #4
	CMP     r2, #0x00
	BNE     SRAM_INIT

  movs r1, #0
  b DataInit

CopyData:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

DataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyData
  ldr r2, =_sbss
  b Zerobss

FillZerobss:
  movs r3, #0
  str r3, [r2], #4

Zerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* SRAM2 section */
  ldr  r2, =_ssram2
  b  Zerosram2
/* Zero fill the sram2 segment. */
FillZerosram2:
  movs  r3, #0
  str  r3, [r2], #4

Zerosram2:
  ldr  r3, = _esram2
  cmp  r2, r3
  bcc  FillZerosram2

  /* Fast RAM BSS section */
  ldr  r2, =__fastram_bss_start__
  b  Zerofastram_bss

FillZerofastram_bss:
  movs  r3, #0
  str  r3, [r2], #4

Zerofastram_bss:
  ldr  r3, = __fastram_bss_end__
  cmp  r2, r3
  bcc  FillZerofastram_bss


/* Call SystemInit function */
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/*Call the main function */
  bl main
  bx lr

.size Reset_Handler, .-Reset_Handler

    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

   .section  .vectors,"a",%progbits
   .global __gVectors



__gVectors:
                    .word _estack                                 /* Top of Stack */
                    .word Reset_Handler                           /* Reset Handler */
                    .word NMI_Handler                             /* NMI Handler */
                    .word HardFault_Handler                       /* Hard Fault Handler */
                    .word MemManage_Handler                       /* MPU Fault Handler */
                    .word BusFault_Handler                        /* Bus Fault Handler */
                    .word UsageFault_Handler                      /* Usage Fault Handler */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word SVC_Handler                             /* SVCall Handler */
                    .word DebugMon_Handler                        /* Debug Monitor Handler */
                    .word 0                                       /* Reserved */
                    .word PendSV_Handler                          /* PendSV Handler */
                    .word SysTick_Handler                         /* SysTick Handler */

                    /* External interrupts handler */
                    .word WWDGT_IRQHandler                        /* Vector Number 16,Window Watchdog Timer */
                    .word AVD_LVD_OVD_IRQHandler                  /* Vector Number 17,AVD/LVD/OVD through EXTI Line detect */
                    .word TAMPER_STAMP_LXTAL_IRQHandler           /* Vector Number 18,RTC Tamper and TimeStamp through EXTI Line detect, LXTAL clock security system interrupt */
                    .word RTC_WKUP_IRQHandler                     /* Vector Number 19,RTC Wakeup from EXTI interrupt */
                    .word FMC_IRQHandler                          /* Vector Number 20,FMC global interrupt */
                    .word RCU_IRQHandler                          /* Vector Number 21,RCU global interrupt */
                    .word EXTI0_IRQHandler                        /* Vector Number 22,EXTI Line 0 */
                    .word EXTI1_IRQHandler                        /* Vector Number 23,EXTI Line 1 */
                    .word EXTI2_IRQHandler                        /* Vector Number 24,EXTI Line 2 */
                    .word EXTI3_IRQHandler                        /* Vector Number 25,EXTI Line 3 */
                    .word EXTI4_IRQHandler                        /* Vector Number 26,EXTI Line 4 */
                    .word DMA0_Channel0_IRQHandler                /* Vector Number 27,DMA0 Channel 0 */
                    .word DMA0_Channel1_IRQHandler                /* Vector Number 28,DMA0 Channel 1 */
                    .word DMA0_Channel2_IRQHandler                /* Vector Number 29,DMA0 Channel 2 */
                    .word DMA0_Channel3_IRQHandler                /* Vector Number 30,DMA0 Channel 3 */
                    .word DMA0_Channel4_IRQHandler                /* Vector Number 31,DMA0 Channel 4 */
                    .word DMA0_Channel5_IRQHandler                /* Vector Number 32,DMA0 Channel 5 */
                    .word DMA0_Channel6_IRQHandler                /* Vector Number 33,DMA0 Channel 6 */
                    .word ADC0_1_IRQHandler                       /* Vector Number 34,ADC0 and ADC1 interrupt */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word EXTI5_9_IRQHandler                      /* Vector Number 39,EXTI5 to EXTI9 */
                    .word TIMER0_BRK_IRQHandler                   /* Vector Number 40,TIMER0 Break */
                    .word TIMER0_UP_IRQHandler                    /* Vector Number 41,TIMER0 Update */
                    .word TIMER0_TRG_CMT_IRQHandler               /* Vector Number 42,TIMER0 Trigger and Commutation */
                    .word TIMER0_Channel_IRQHandler               /* Vector Number 43,TIMER0 Capture Compare */
                    .word TIMER1_IRQHandler                       /* Vector Number 44,TIMER1 */
                    .word TIMER2_IRQHandler                       /* Vector Number 45,TIMER2 */
                    .word TIMER3_IRQHandler                       /* Vector Number 46,TIMER3 */
                    .word I2C0_EV_IRQHandler                      /* Vector Number 47,I2C0 Event */
                    .word I2C0_ER_IRQHandler                      /* Vector Number 48,I2C0 Error */
                    .word I2C1_EV_IRQHandler                      /* Vector Number 49,I2C1 Event */
                    .word I2C1_ER_IRQHandler                      /* Vector Number 50,I2C1 Error */
                    .word SPI0_IRQHandler                         /* Vector Number 51,SPI0 */
                    .word SPI1_IRQHandler                         /* Vector Number 52,SPI1 */
                    .word USART0_IRQHandler                       /* Vector Number 53,USART0 global and wakeup */
                    .word USART1_IRQHandler                       /* Vector Number 54,USART1 global and wakeup */
                    .word USART2_IRQHandler                       /* Vector Number 55,USART2 global and wakeup */
                    .word EXTI10_15_IRQHandler                    /* Vector Number 56,EXTI10 to EXTI15 */
                    .word RTC_Alarm_IRQHandler                    /* Vector Number 57,RTC Alarm */
                    .word 0                                       /* Reserved */
                    .word TIMER7_BRK_IRQHandler                   /* Vector Number 59,TIMER7 Break */
                    .word TIMER7_UP_IRQHandler                    /* Vector Number 60,TIMER7 Update */
                    .word TIMER7_TRG_CMT_IRQHandler               /* Vector Number 61,TIMER7 Trigger and Commutation */
                    .word TIMER7_Channel_IRQHandler               /* Vector Number 62,TIMER7 Channel Capture Compare */
                    .word DMA0_Channel7_IRQHandler                /* Vector Number 63,DMA0 Channel 7 */
                    .word EXMC_IRQHandler                         /* Vector Number 64,EXMC */
                    .word SDIO0_IRQHandler                        /* Vector Number 65,SDIO0 */
                    .word TIMER4_IRQHandler                       /* Vector Number 66,TIMER4 */
                    .word SPI2_IRQHandler                         /* Vector Number 67,SPI2 */
                    .word UART3_IRQHandler                        /* Vector Number 68,UART3 */
                    .word UART4_IRQHandler                        /* Vector Number 69,UART4 */
                    .word TIMER5_DAC_UDR_IRQHandler               /* Vector Number 70,TIMER5 global interrupt and DAC1/DAC0 underrun error */
                    .word TIMER6_IRQHandler                       /* Vector Number 71,TIMER6 */
                    .word DMA1_Channel0_IRQHandler                /* Vector Number 72,DMA1 Channel0 */
                    .word DMA1_Channel1_IRQHandler                /* Vector Number 73,DMA1 Channel1 */
                    .word DMA1_Channel2_IRQHandler                /* Vector Number 74,DMA1 Channel2 */
                    .word DMA1_Channel3_IRQHandler                /* Vector Number 75,DMA1 Channel3 */
                    .word DMA1_Channel4_IRQHandler                /* Vector Number 76,DMA1 Channel4 */
                    .word ENET0_IRQHandler                        /* Vector Number 77,Ethernet0 */
                    .word ENET0_WKUP_IRQHandler                   /* Vector Number 78,Ethernet0 Wakeup through EXTI Line */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word DMA1_Channel5_IRQHandler                /* Vector Number 84,DMA1 Channel5 */
                    .word DMA1_Channel6_IRQHandler                /* Vector Number 85,DMA1 Channel6 */
                    .word DMA1_Channel7_IRQHandler                /* Vector Number 86,DMA1 Channel7 */
                    .word USART5_IRQHandler                       /* Vector Number 87,USART5 global and wakeup */
                    .word I2C2_EV_IRQHandler                      /* Vector Number 88,I2C2 Event */
                    .word I2C2_ER_IRQHandler                      /* Vector Number 89,I2C2 Error */
                    .word USBHS0_EP1_OUT_IRQHandler               /* Vector Number 90,USBHS0 Endpoint 1 Out */
                    .word USBHS0_EP1_IN_IRQHandler                /* Vector Number 91,USBHS0 Endpoint 1 in */
                    .word USBHS0_WKUP_IRQHandler                  /* Vector Number 92,USBHS0 Wakeup through EXTI Line */
                    .word USBHS0_IRQHandler                       /* Vector Number 93,USBHS0 */
                    .word DCI_IRQHandler                          /* Vector Number 94,DCI */
                    .word CAU_IRQHandler                          /* Vector Number 95,CAU */
                    .word HAU_TRNG_IRQHandler                     /* Vector Number 96,HAU and TRNG */
                    .word FPU_IRQHandler                          /* Vector Number 97,FPU */
                    .word UART6_IRQHandler                        /* Vector Number 98,UART6 */
                    .word UART7_IRQHandler                        /* Vector Number 99,UART7 */
                    .word SPI3_IRQHandler                         /* Vector Number 100,SPI3 */
                    .word SPI4_IRQHandler                         /* Vector Number 101,SPI4 */
                    .word SPI5_IRQHandler                         /* Vector Number 102,SPI5 */
                    .word SAI0_IRQHandler                         /* Vector Number 103,SAI0 */
                    .word TLI_IRQHandler                          /* Vector Number 104,TLI */
                    .word TLI_ER_IRQHandler                       /* Vector Number 105,TLI Error */
                    .word IPA_IRQHandler                          /* Vector Number 106,IPA */
                    .word SAI1_IRQHandler                         /* Vector Number 107,SAI1 */
                    .word OSPI0_IRQHandler                        /* Vector Number 108,OSPI0 */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word I2C3_EV_IRQHandler                      /* Vector Number 111,I2C3 Event */
                    .word I2C3_ER_IRQHandler                      /* Vector Number 112,I2C3 Error */
                    .word RSPDIF_IRQHandler                       /* Vector Number 113,RSPDIF */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word DMAMUX_OVR_IRQHandler                   /* Vector Number 118,DMAMUX Overrun interrupt */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word HPDF_INT0_IRQHandler                    /* Vector Number 126,HPDF global interrupt 0 */
                    .word HPDF_INT1_IRQHandler                    /* Vector Number 127,HPDF global interrupt 1 */
                    .word HPDF_INT2_IRQHandler                    /* Vector Number 128,HPDF global interrupt 2 */
                    .word HPDF_INT3_IRQHandler                    /* Vector Number 129,HPDF global interrupt 3 */
                    .word SAI2_IRQHandler                         /* Vector Number 130,SAI2 global interrupt */
                    .word 0                                       /* Reserved */
                    .word TIMER14_IRQHandler                      /* Vector Number 132,TIMER14 */
                    .word TIMER15_IRQHandler                      /* Vector Number 133,TIMER15 */
                    .word TIMER16_IRQHandler                      /* Vector Number 134,TIMER16 */
                    .word 0                                       /* Reserved */
                    .word MDIO_IRQHandler                         /* Vector Number 136,MDIO */
                    .word 0                                       /* Reserved */
                    .word MDMA_IRQHandler                         /* Vector Number 138,MDMA */
                    .word 0                                       /* Reserved */
                    .word SDIO1_IRQHandler                        /* Vector Number 140,SDIO1 */
                    .word HWSEM_IRQHandler                        /* Vector Number 141,HWSEM */
                    .word 0                                       /* Reserved */
                    .word ADC2_IRQHandler                         /* Vector Number 143,ADC2 */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word CMP0_1_IRQHandler                       /* Vector Number 153,CMP0 and CMP1 */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word CTC_IRQHandler                          /* Vector Number 160,Clock Recovery System */
                    .word RAMECCMU_IRQHandler                     /* Vector Number 161,RAMECCMU */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word OSPI1_IRQHandler                        /* Vector Number 166,OSPI1 */
                    .word RTDEC0_IRQHandler                       /* Vector Number 167,RTDEC0 */
                    .word RTDEC1_IRQHandler                       /* Vector Number 168,RTDEC1 */
                    .word FAC_IRQHandler                          /* Vector Number 169,FAC */
                    .word TMU_IRQHandler                          /* Vector Number 170,TMU */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word 0                                       /* Reserved */
                    .word TIMER22_IRQHandler                      /* Vector Number 177,TIMER22 */
                    .word TIMER23_IRQHandler                      /* Vector Number 178,TIMER23 */
                    .word TIMER30_IRQHandler                      /* Vector Number 179,TIMER30 */
                    .word TIMER31_IRQHandler                      /* Vector Number 180,TIMER31 */
                    .word TIMER40_IRQHandler                      /* Vector Number 181,TIMER40 */
                    .word TIMER41_IRQHandler                      /* Vector Number 182,TIMER41 */
                    .word TIMER42_IRQHandler                      /* Vector Number 183,TIMER42 */
                    .word TIMER43_IRQHandler                      /* Vector Number 184,TIMER43 */
                    .word TIMER44_IRQHandler                      /* Vector Number 185,TIMER44 */
                    .word TIMER50_IRQHandler                      /* Vector Number 186,TIMER50 */
                    .word TIMER51_IRQHandler                      /* Vector Number 187,TIMER51 */
                    .word USBHS1_EP1_OUT_IRQHandler               /* Vector Number 188,USBHS1 endpoint 1 out */
                    .word USBHS1_EP1_IN_IRQHandler                /* Vector Number 189,USBHS1 endpoint 1 in */
                    .word USBHS1_WKUP_IRQHandler                  /* Vector Number 190,USBHS1 wakeup */
                    .word USBHS1_IRQHandler                       /* Vector Number 191,USBHS1 */
                    .word ENET1_IRQHandler                        /* Vector Number 192,Ethernet1 */
                    .word ENET1_WKUP_IRQHandler                   /* Vector Number 193,Ethernet1 wakeup */
                    .word 0                                       /* Reserved */
                    .word CAN0_WKUP_IRQHandler                    /* Vector Number 195,CAN0 wakeup */
                    .word CAN0_Message_IRQHandler                 /* Vector Number 196,CAN0 interrupt for message buffer */
                    .word CAN0_Busoff_IRQHandler                  /* Vector Number 197,CAN0 interrupt for Bus off / Bus off done */
                    .word CAN0_Error_IRQHandler                   /* Vector Number 198,CAN0 interrupt for Error */
                    .word CAN0_FastError_IRQHandler               /* Vector Number 199,CAN0 interrupt for Error in fast transmission */
                    .word CAN0_TEC_IRQHandler                     /* Vector Number 200,CAN0 interrupt for Transmit warning */
                    .word CAN0_REC_IRQHandler                     /* Vector Number 201,CAN0 interrupt for Receive warning */
                    .word CAN1_WKUP_IRQHandler                    /* Vector Number 202,CAN1 wakeup */
                    .word CAN1_Message_IRQHandler                 /* Vector Number 203,CAN1 interrupt for message buffer */
                    .word CAN1_Busoff_IRQHandler                  /* Vector Number 204,CAN1 interrupt for Bus off / Bus off done */
                    .word CAN1_Error_IRQHandler                   /* Vector Number 205,CAN1 interrupt for Error */
                    .word CAN1_FastError_IRQHandler               /* Vector Number 206,CAN1 interrupt for Error in fast transmission */
                    .word CAN1_TEC_IRQHandler                     /* Vector Number 207,CAN1 interrupt for Transmit warning */
                    .word CAN1_REC_IRQHandler                     /* Vector Number 208,CAN1 interrupt for Receive warning */
                    .word CAN2_WKUP_IRQHandler                    /* Vector Number 209,CAN2 wakeup */
                    .word CAN2_Message_IRQHandler                 /* Vector Number 210,CAN2 interrupt for message buffer */
                    .word CAN2_Busoff_IRQHandler                  /* Vector Number 211,CAN2 interrupt for Bus off / Bus off done */
                    .word CAN2_Error_IRQHandler                   /* Vector Number 212,CAN2 interrupt for Error */
                    .word CAN2_FastError_IRQHandler               /* Vector Number 213,CAN2 interrupt for Error in fast transmission */
                    .word CAN2_TEC_IRQHandler                     /* Vector Number 214,CAN2 interrupt for Transmit warning */
                    .word CAN2_REC_IRQHandler                     /* Vector Number 215,CAN2 interrupt for Receive warning */
                    .word EFUSE_IRQHandler                        /* Vector Number 216,EFUSE */
                    .word I2C0_WKUP_IRQHandler                    /* Vector Number 217,I2C0 wakeup */
                    .word I2C1_WKUP_IRQHandler                    /* Vector Number 218,I2C1 wakeup */
                    .word I2C2_WKUP_IRQHandler                    /* Vector Number 219,I2C2 wakeup */
                    .word I2C3_WKUP_IRQHandler                    /* Vector Number 220,I2C3 wakeup */
                    .word LPDTS_IRQHandler                        /* Vector Number 221,LPDTS */
                    .word LPDTS_WKUP_IRQHandler                   /* Vector Number 222,LPDTS wakeup */
                    .word TIMER0_DEC_IRQHandler                   /* Vector Number 223,TIMER0 DEC */
                    .word TIMER7_DEC_IRQHandler                   /* Vector Number 224,TIMER7 DEC */
                    .word TIMER1_DEC_IRQHandler                   /* Vector Number 225,TIMER1 DEC */
                    .word TIMER2_DEC_IRQHandler                   /* Vector Number 226,TIMER2 DEC */
                    .word TIMER3_DEC_IRQHandler                   /* Vector Number 227,TIMER3 DEC */
                    .word TIMER4_DEC_IRQHandler                   /* Vector Number 228,TIMER4 DEC */
                    .word TIMER22_DEC_IRQHandler                  /* Vector Number 229,TIMER22 DEC */
                    .word TIMER23_DEC_IRQHandler                  /* Vector Number 230,TIMER23 DEC */
                    .word TIMER30_DEC_IRQHandler                  /* Vector Number 231,TIMER30 DEC */
                    .word TIMER31_DEC_IRQHandler                  /* Vector Number 232,TIMER31 DEC */

  .size   __gVectors, . - __gVectors

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

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak WWDGT_IRQHandler
  .thumb_set WWDGT_IRQHandler,Default_Handler

  .weak AVD_LVD_OVD_IRQHandler
  .thumb_set AVD_LVD_OVD_IRQHandler,Default_Handler

  .weak TAMPER_STAMP_LXTAL_IRQHandler
  .thumb_set TAMPER_STAMP_LXTAL_IRQHandler,Default_Handler

  .weak RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler,Default_Handler

  .weak FMC_IRQHandler
  .thumb_set FMC_IRQHandler,Default_Handler

  .weak RCU_IRQHandler
  .thumb_set RCU_IRQHandler,Default_Handler

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

  .weak DMA0_Channel0_IRQHandler
  .thumb_set DMA0_Channel0_IRQHandler,Default_Handler

  .weak DMA0_Channel1_IRQHandler
  .thumb_set DMA0_Channel1_IRQHandler,Default_Handler

  .weak DMA0_Channel2_IRQHandler
  .thumb_set DMA0_Channel2_IRQHandler,Default_Handler

  .weak DMA0_Channel3_IRQHandler
  .thumb_set DMA0_Channel3_IRQHandler,Default_Handler

  .weak DMA0_Channel4_IRQHandler
  .thumb_set DMA0_Channel4_IRQHandler,Default_Handler

  .weak DMA0_Channel5_IRQHandler
  .thumb_set DMA0_Channel5_IRQHandler,Default_Handler

  .weak DMA0_Channel6_IRQHandler
  .thumb_set DMA0_Channel6_IRQHandler,Default_Handler

  .weak ADC0_1_IRQHandler
  .thumb_set ADC0_1_IRQHandler,Default_Handler

  .weak EXTI5_9_IRQHandler
  .thumb_set EXTI5_9_IRQHandler,Default_Handler

  .weak TIMER0_BRK_IRQHandler
  .thumb_set TIMER0_BRK_IRQHandler,Default_Handler

  .weak TIMER0_UP_IRQHandler
  .thumb_set TIMER0_UP_IRQHandler,Default_Handler

  .weak TIMER0_TRG_CMT_IRQHandler
  .thumb_set TIMER0_TRG_CMT_IRQHandler,Default_Handler

  .weak TIMER0_Channel_IRQHandler
  .thumb_set TIMER0_Channel_IRQHandler,Default_Handler

  .weak TIMER1_IRQHandler
  .thumb_set TIMER1_IRQHandler,Default_Handler

  .weak TIMER2_IRQHandler
  .thumb_set TIMER2_IRQHandler,Default_Handler

  .weak TIMER3_IRQHandler
  .thumb_set TIMER3_IRQHandler,Default_Handler

  .weak I2C0_EV_IRQHandler
  .thumb_set I2C0_EV_IRQHandler,Default_Handler

  .weak I2C0_ER_IRQHandler
  .thumb_set I2C0_ER_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak SPI0_IRQHandler
  .thumb_set SPI0_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak USART0_IRQHandler
  .thumb_set USART0_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak EXTI10_15_IRQHandler
  .thumb_set EXTI10_15_IRQHandler,Default_Handler

  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak TIMER7_BRK_IRQHandler
  .thumb_set TIMER7_BRK_IRQHandler,Default_Handler

  .weak TIMER7_UP_IRQHandler
  .thumb_set TIMER7_UP_IRQHandler,Default_Handler

  .weak TIMER7_TRG_CMT_IRQHandler
  .thumb_set TIMER7_TRG_CMT_IRQHandler,Default_Handler

  .weak TIMER7_Channel_IRQHandler
  .thumb_set TIMER7_Channel_IRQHandler,Default_Handler

  .weak DMA0_Channel7_IRQHandler
  .thumb_set DMA0_Channel7_IRQHandler,Default_Handler

  .weak EXMC_IRQHandler
  .thumb_set EXMC_IRQHandler,Default_Handler

  .weak SDIO0_IRQHandler
  .thumb_set SDIO0_IRQHandler,Default_Handler

  .weak TIMER4_IRQHandler
  .thumb_set TIMER4_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak UART3_IRQHandler
  .thumb_set UART3_IRQHandler,Default_Handler

  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak TIMER5_DAC_UDR_IRQHandler
  .thumb_set TIMER5_DAC_UDR_IRQHandler,Default_Handler

  .weak TIMER6_IRQHandler
  .thumb_set TIMER6_IRQHandler,Default_Handler

  .weak DMA1_Channel0_IRQHandler
  .thumb_set DMA1_Channel0_IRQHandler,Default_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak ENET0_IRQHandler
  .thumb_set ENET0_IRQHandler,Default_Handler

  .weak ENET0_WKUP_IRQHandler
  .thumb_set ENET0_WKUP_IRQHandler,Default_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

  .weak USART5_IRQHandler
  .thumb_set USART5_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak USBHS0_EP1_OUT_IRQHandler
  .thumb_set USBHS0_EP1_OUT_IRQHandler,Default_Handler

  .weak USBHS0_EP1_IN_IRQHandler
  .thumb_set USBHS0_EP1_IN_IRQHandler,Default_Handler

  .weak USBHS0_WKUP_IRQHandler
  .thumb_set USBHS0_WKUP_IRQHandler,Default_Handler

  .weak USBHS0_IRQHandler
  .thumb_set USBHS0_IRQHandler,Default_Handler

  .weak DCI_IRQHandler
  .thumb_set DCI_IRQHandler,Default_Handler

  .weak CAU_IRQHandler
  .thumb_set CAU_IRQHandler,Default_Handler

  .weak HAU_TRNG_IRQHandler
  .thumb_set HAU_TRNG_IRQHandler,Default_Handler

  .weak FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

  .weak UART6_IRQHandler
  .thumb_set UART6_IRQHandler,Default_Handler

  .weak UART7_IRQHandler
  .thumb_set UART7_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler,Default_Handler

  .weak SPI5_IRQHandler
  .thumb_set SPI5_IRQHandler,Default_Handler

  .weak SAI0_IRQHandler
  .thumb_set SAI0_IRQHandler,Default_Handler

  .weak TLI_IRQHandler
  .thumb_set TLI_IRQHandler,Default_Handler

  .weak TLI_ER_IRQHandler
  .thumb_set TLI_ER_IRQHandler,Default_Handler

  .weak IPA_IRQHandler
  .thumb_set IPA_IRQHandler,Default_Handler

  .weak SAI1_IRQHandler
  .thumb_set SAI1_IRQHandler,Default_Handler

  .weak OSPI0_IRQHandler
  .thumb_set OSPI0_IRQHandler,Default_Handler

  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak RSPDIF_IRQHandler
  .thumb_set RSPDIF_IRQHandler,Default_Handler

  .weak DMAMUX_OVR_IRQHandler
  .thumb_set DMAMUX_OVR_IRQHandler,Default_Handler

  .weak HPDF_INT0_IRQHandler
  .thumb_set HPDF_INT0_IRQHandler,Default_Handler

  .weak HPDF_INT1_IRQHandler
  .thumb_set HPDF_INT1_IRQHandler,Default_Handler

  .weak HPDF_INT2_IRQHandler
  .thumb_set HPDF_INT2_IRQHandler,Default_Handler

  .weak HPDF_INT3_IRQHandler
  .thumb_set HPDF_INT3_IRQHandler,Default_Handler

  .weak SAI2_IRQHandler
  .thumb_set SAI2_IRQHandler,Default_Handler

  .weak TIMER14_IRQHandler
  .thumb_set TIMER14_IRQHandler,Default_Handler

  .weak TIMER15_IRQHandler
  .thumb_set TIMER15_IRQHandler,Default_Handler

  .weak TIMER16_IRQHandler
  .thumb_set TIMER16_IRQHandler,Default_Handler

  .weak MDIO_IRQHandler
  .thumb_set MDIO_IRQHandler,Default_Handler

  .weak MDMA_IRQHandler
  .thumb_set MDMA_IRQHandler,Default_Handler

  .weak SDIO1_IRQHandler
  .thumb_set SDIO1_IRQHandler,Default_Handler

  .weak HWSEM_IRQHandler
  .thumb_set HWSEM_IRQHandler,Default_Handler

  .weak ADC2_IRQHandler
  .thumb_set ADC2_IRQHandler,Default_Handler

  .weak CMP0_1_IRQHandler
  .thumb_set CMP0_1_IRQHandler,Default_Handler

  .weak CTC_IRQHandler
  .thumb_set CTC_IRQHandler,Default_Handler

  .weak RAMECCMU_IRQHandler
  .thumb_set RAMECCMU_IRQHandler,Default_Handler

  .weak OSPI1_IRQHandler
  .thumb_set OSPI1_IRQHandler,Default_Handler

  .weak RTDEC0_IRQHandler
  .thumb_set RTDEC0_IRQHandler,Default_Handler

  .weak RTDEC1_IRQHandler
  .thumb_set RTDEC1_IRQHandler,Default_Handler

  .weak FAC_IRQHandler
  .thumb_set FAC_IRQHandler,Default_Handler

  .weak TMU_IRQHandler
  .thumb_set TMU_IRQHandler,Default_Handler

  .weak TIMER22_IRQHandler
  .thumb_set TIMER22_IRQHandler,Default_Handler

  .weak TIMER23_IRQHandler
  .thumb_set TIMER23_IRQHandler,Default_Handler

  .weak TIMER30_IRQHandler
  .thumb_set TIMER30_IRQHandler,Default_Handler

  .weak TIMER31_IRQHandler
  .thumb_set TIMER31_IRQHandler,Default_Handler

  .weak TIMER40_IRQHandler
  .thumb_set TIMER40_IRQHandler,Default_Handler

  .weak TIMER41_IRQHandler
  .thumb_set TIMER41_IRQHandler,Default_Handler

  .weak TIMER42_IRQHandler
  .thumb_set TIMER42_IRQHandler,Default_Handler

  .weak TIMER43_IRQHandler
  .thumb_set TIMER43_IRQHandler,Default_Handler

  .weak TIMER44_IRQHandler
  .thumb_set TIMER44_IRQHandler,Default_Handler

  .weak TIMER50_IRQHandler
  .thumb_set TIMER50_IRQHandler,Default_Handler

  .weak TIMER51_IRQHandler
  .thumb_set TIMER51_IRQHandler,Default_Handler

  .weak USBHS1_EP1_OUT_IRQHandler
  .thumb_set USBHS1_EP1_OUT_IRQHandler,Default_Handler

  .weak USBHS1_EP1_IN_IRQHandler
  .thumb_set USBHS1_EP1_IN_IRQHandler,Default_Handler

  .weak USBHS1_WKUP_IRQHandler
  .thumb_set USBHS1_WKUP_IRQHandler,Default_Handler

  .weak USBHS1_IRQHandler
  .thumb_set USBHS1_IRQHandler,Default_Handler

  .weak ENET1_IRQHandler
  .thumb_set ENET1_IRQHandler,Default_Handler

  .weak ENET1_WKUP_IRQHandler
  .thumb_set ENET1_WKUP_IRQHandler,Default_Handler

  .weak CAN0_WKUP_IRQHandler
  .thumb_set CAN0_WKUP_IRQHandler,Default_Handler

  .weak CAN0_Message_IRQHandler
  .thumb_set CAN0_Message_IRQHandler,Default_Handler

  .weak CAN0_Busoff_IRQHandler
  .thumb_set CAN0_Busoff_IRQHandler,Default_Handler

  .weak CAN0_Error_IRQHandler
  .thumb_set CAN0_Error_IRQHandler,Default_Handler

  .weak CAN0_FastError_IRQHandler
  .thumb_set CAN0_FastError_IRQHandler,Default_Handler

  .weak CAN0_TEC_IRQHandler
  .thumb_set CAN0_TEC_IRQHandler,Default_Handler

  .weak CAN0_REC_IRQHandler
  .thumb_set CAN0_REC_IRQHandler,Default_Handler

  .weak CAN1_WKUP_IRQHandler
  .thumb_set CAN1_WKUP_IRQHandler,Default_Handler

  .weak CAN1_Message_IRQHandler
  .thumb_set CAN1_Message_IRQHandler,Default_Handler

  .weak CAN1_Busoff_IRQHandler
  .thumb_set CAN1_Busoff_IRQHandler,Default_Handler

  .weak CAN1_Error_IRQHandler
  .thumb_set CAN1_Error_IRQHandler,Default_Handler

  .weak CAN1_FastError_IRQHandler
  .thumb_set CAN1_FastError_IRQHandler,Default_Handler

  .weak CAN1_TEC_IRQHandler
  .thumb_set CAN1_TEC_IRQHandler,Default_Handler

  .weak CAN1_REC_IRQHandler
  .thumb_set CAN1_REC_IRQHandler,Default_Handler

  .weak CAN2_WKUP_IRQHandler
  .thumb_set CAN2_WKUP_IRQHandler,Default_Handler

  .weak CAN2_Message_IRQHandler
  .thumb_set CAN2_Message_IRQHandler,Default_Handler

  .weak CAN2_Busoff_IRQHandler
  .thumb_set CAN2_Busoff_IRQHandler,Default_Handler

  .weak CAN2_Error_IRQHandler
  .thumb_set CAN2_Error_IRQHandler,Default_Handler

  .weak CAN2_FastError_IRQHandler
  .thumb_set CAN2_FastError_IRQHandler,Default_Handler

  .weak CAN2_TEC_IRQHandler
  .thumb_set CAN2_TEC_IRQHandler,Default_Handler

  .weak CAN2_REC_IRQHandler
  .thumb_set CAN2_REC_IRQHandler,Default_Handler

  .weak EFUSE_IRQHandler
  .thumb_set EFUSE_IRQHandler,Default_Handler

  .weak I2C0_WKUP_IRQHandler
  .thumb_set I2C0_WKUP_IRQHandler,Default_Handler

  .weak I2C1_WKUP_IRQHandler
  .thumb_set I2C1_WKUP_IRQHandler,Default_Handler

  .weak I2C2_WKUP_IRQHandler
  .thumb_set I2C2_WKUP_IRQHandler,Default_Handler

  .weak I2C3_WKUP_IRQHandler
  .thumb_set I2C3_WKUP_IRQHandler,Default_Handler

  .weak LPDTS_IRQHandler
  .thumb_set LPDTS_IRQHandler,Default_Handler

  .weak LPDTS_WKUP_IRQHandler
  .thumb_set LPDTS_WKUP_IRQHandler,Default_Handler

  .weak TIMER0_DEC_IRQHandler
  .thumb_set TIMER0_DEC_IRQHandler,Default_Handler

  .weak TIMER7_DEC_IRQHandler
  .thumb_set TIMER7_DEC_IRQHandler,Default_Handler

  .weak TIMER1_DEC_IRQHandler
  .thumb_set TIMER1_DEC_IRQHandler,Default_Handler

  .weak TIMER2_DEC_IRQHandler
  .thumb_set TIMER2_DEC_IRQHandler,Default_Handler

  .weak TIMER3_DEC_IRQHandler
  .thumb_set TIMER3_DEC_IRQHandler,Default_Handler

  .weak TIMER4_DEC_IRQHandler
  .thumb_set TIMER4_DEC_IRQHandler,Default_Handler

  .weak TIMER22_DEC_IRQHandler
  .thumb_set TIMER22_DEC_IRQHandler,Default_Handler

  .weak TIMER23_DEC_IRQHandler
  .thumb_set TIMER23_DEC_IRQHandler,Default_Handler

  .weak TIMER30_DEC_IRQHandler
  .thumb_set TIMER30_DEC_IRQHandler,Default_Handler

  .weak TIMER31_DEC_IRQHandler
  .thumb_set TIMER31_DEC_IRQHandler,Default_Handler
