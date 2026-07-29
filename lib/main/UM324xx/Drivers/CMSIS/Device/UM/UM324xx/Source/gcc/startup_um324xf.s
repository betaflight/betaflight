/**
  ******************************************************************************
  * @file      startup_um324xf.s
  * @author    MCD Application Team
  * @version   V1.0.0
  * @date      13-Feb-2025
  * @brief    
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the clock system and the external SRAM mounted on 
  *                  STM324xG-EVAL board to be used as data memory (optional, 
  *                  to be enabled by user)
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
    
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_um32Vectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
    .weak  Reset_Handler
    .type  Reset_Handler, %function
Reset_Handler:  
  ldr  r0, =_estack 
  mov  sp, r0

/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit   
/* Call static constructors */
/*    bl __libc_init_array */
/* Call the application's entry point.*/
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an 
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None     
 * @retval None       
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M4. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
    .section  .isr_vector,"a",%progbits
    .type  g_um32Vectors, %object
    .size  g_um32Vectors, .-g_um32Vectors
    
    
g_um32Vectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
/* External Interrupts */
  .word  WWDT_IRQHandler           			/*  0:  WWDT_IRQHandler */
  .word  LVD_IRQHandler            			/*  1:  LVD_IRQHandler */
  .word  TAMPSTAMP_IRQHandler      			/*  2:  TAMPSTAMP_IRQHandler */
  .word  0       							/*  3:  0 */
  .word  EFC_IRQHandler            			/*  4:  EFC_IRQHandler */
  .word  RCM_IRQHandler            			/*  5:  RCM_IRQHandler */
  .word  EXTI0_IRQHandler          			/*  6:  EXTI0_IRQHandler */					
  .word  EXTI1_IRQHandler          			/*  7:  EXTI1_IRQHandler */					
  .word  EXTI2_IRQHandler          			/*  8:  EXTI2_IRQHandler */
  .word  EXTI3_IRQHandler          			/*  9:  EXTI3_IRQHandler */
  .word  EXTI4_IRQHandler          			/*  10: EXTI4_IRQHandler */		
  .word  DMAC0CH0_IRQHandler       			/* 11: DMAC0CH0_IRQHandler */
  .word  DMAC0CH1_IRQHandler       			/* 12: DMAC0CH1_IRQHandler */
  .word  DMAC0CH2_IRQHandler       			/* 13: DMAC0CH2_IRQHandler */
  .word  DMAC0CH3_IRQHandler       			/* 14: DMAC0CH3_IRQHandler */
  .word  DMAC0CH4_IRQHandler       			/* 15: DMAC0CH4_IRQHandler */
  .word  DMAC0CH5_IRQHandler       			/* 16: DMAC0CH5_IRQHandler */
  .word  DMAC0CH6_IRQHandler       			/* 17: DMAC0CH6_IRQHandler */
  .word  DMAC0CH7_IRQHandler       			/* 18: DMAC0CH7_IRQHandler */
  .word  ADC0_IRQHandler           			/* 19: ADC0_IRQHandler */
  .word  ADC1_IRQHandler           			/* 20: ADC1_IRQHandler */
  .word  CAN0_IRQHandler           			/* 21: CAN0_IRQHandler */
  .word  CAN1_IRQHandler           			/* 22: CAN1_IRQHandler */
  .word  EXTI5TO9_IRQHandler       			/* 23: EXTI5TO9_IRQHandler */
  .word  TIM0_BRK_TIM8_IRQHandler  			/* 24: TIM0_BRK_TIM8_IRQHandler */
  .word  TIM0_UP_TIM9_IRQHandler   			/* 25: TIM0_UP_TIM9_IRQHandler */
  .word  TIM0_TRG_COM_TIM10_IRQHandler       /* 26: TIM0_TRG_COM_TIM10_IRQHandler */
  .word  TIM0_CC_IRQHandler       			/* 27: TIM0_CC_IRQHandler */
  .word  TIM1_IRQHandler       	 			/* 28: TIM1_IRQHandler */
  .word  TIM2_IRQHandler          			/* 29: TIM2_IRQHandler */
  .word  TIM3_IRQHandler          			/* 30: TIM3_IRQHandler */
  .word  I2C0_IRQHandler          			/* 31: I2C0_IRQHandler */
  .word  I2C1_IRQHandler          			/* 32: I2C1_IRQHandler */
  .word  0        							/* 33: 0 */
  .word  0					       			/* 34: 0 */
  .word  SPI0_IRQHandler          			/* 35: SPI0_IRQHandler */
  .word  SPI1_IRQHandler          			/* 36: SPI1_IRQHandler */
  .word  UART0_IRQHandler         			/* 37: UART0_IRQHandler */
  .word  UART1_IRQHandler         			/* 38: UART1_IRQHandler */
  .word  UART2_IRQHandler         			/* 39: UART2_IRQHandler */
  .word  EXTI10TO15_IRQHandler    			/* 40: EXTI10TO15_IRQHandler */							
  .word  RTC_ALARM_IRQHandler     			/* 41: RTC_ALARM_IRQHandler */
  .word  0    					 			/* 42: 0 */
  .word  TIM7_BRK_TIM11_IRQHandler    		/* 43: TIM7_BRK_TIM11_IRQHandler */
  .word  TIM7_UP_TIM12_IRQHandler     		/* 44: TIM7_UP_TIM12_IRQHandler */
  .word  TIM7_TRG_COM_TIM13_IRQHandler     	/* 45: TIM7_TRG_COM_TIM13_IRQHandler */
  .word  TIM7_CC_IRQHandler     				/* 46: TIM7_CC_IRQHandler */
  .word  SDIO_IRQHandler     				/* 47: SDIO_IRQHandler */
  .word  0     								/* 48: 0 */
  .word  0     								/* 49: 0 */
  .word  TIM4_IRQHandler     				/* 50: TIM4_IRQHandler */
  .word  SPI2_IRQHandler     				/* 51: SPI2_IRQHandler */
  .word  UART3_IRQHandler     				/* 52: UART3_IRQHandler */
  .word  UART4_IRQHandler     				/* 53: UART4_IRQHandler */
  .word  TIM5_IRQHandler     				/* 54: TIM5_IRQHandler */
  .word  TIM6_IRQHandler     				/* 55: TIM6_IRQHandler */
  .word  DMAC1CH0_IRQHandler         		/* 56: DMAC1CH0_IRQHandler */
  .word  DMAC1CH1_IRQHandler         		/* 57: DMAC1CH1_IRQHandler */
  .word  DMAC1CH2_IRQHandler         		/* 58: DMAC1CH2_IRQHandler */
  .word  DMAC1CH3_IRQHandler         		/* 59: DMAC1CH3_IRQHandler */
  .word  DMAC1CH4_IRQHandler         		/* 60: DMAC1CH4_IRQHandler */
  .word  EMAC_IRQHandler         			/* 61: EMAC_IRQHandler		 */				
  .word  SPI3_IRQHandler        				/* 62: SPI3_IRQHandler */
  .word  0        							/* 63: 0 */
  .word  TS_IRQHandler        				/* 64: TS_IRQHandler */
  .word  OPA0_IRQHandler        				/* 65: OPA0_IRQHandler */
  .word  OPA1_IRQHandler        				/* 66: OPA1_IRQHandler */				
  .word  DAC_IRQHandler             			/* 67: DAC_IRQHandler */
  .word  DMAC1CH5_IRQHandler         		/* 68: DMAC1CH5_IRQHandler */
  .word  DMAC1CH6_IRQHandler         		/* 69: DMAC1CH6_IRQHandler */
  .word  DMAC1CH7_IRQHandler         		/* 70: DMAC1CH7_IRQHandler */
  .word  UART5_IRQHandler         			/* 71: UART5_IRQHandler */
  .word  I2C2_IRQHandler         			/* 72: I2C2_IRQHandler */
  .word  USB0_IRQHandler         			/* 73: USB0_IRQHandler */
  .word  0         							/* 74: 0 */
  .word  0         							/* 75: 0 */
  .word  USART6_IRQHandler              		/* 76: USART6_IRQHandler */
  .word  USART7_IRQHandler              		/* 77: USART7_IRQHandler */
  .word  DCMI_IRQHandler             		/* 78: DCMI_IRQHandler */
  .word  AES_IRQHandler              		/* 79: AES_IRQHandler */
  .word  SHA_IRQHandler              		/* 80: SHA_IRQHandler */
  .word  FPU_IRQHandler              		/* 81: FPU_IRQHandler */
  .word  ACMP0_IRQHandler            		/* 82: ACMP0_IRQHandler */
  .word  ACMP1_IRQHandler           			/* 83: ACMP1_IRQHandler */
  .word  ACMP2_IRQHandler             		/* 84: ACMP2_IRQHandler */
  .word  I2S0_IRQHandler             		/* 85: I2S0_IRQHandler */
  .word  I2S1_IRQHandler             		/* 86: I2S1_IRQHandler */
  .word  0			             			/* 87: 0 */
  .word  0           						/* 88: 0 */
  .word  0           						/* 89: 0 */
  .word  QSPI_IRQHandler             		/* 90: QSPI_IRQHandler */
  .word  0          							/* 91: 0 */
  .word  0 									/* 92: 0 */
  .word  IWDT_IRQHandler           			/* 93: IWDT_IRQHandler */
  .word  0      								/* 94: 0 */
  .word  0      	 							/* 95: 0 */
  .word  0      	 							/* 96: 0 */
  .word  0      	 							/* 97: 0 */			
  .word  0      	 							/* 98: 0 */
  .word  0      	 							/* 99: 0 */
  .word  0      	 							/* 100: 0 */
  .word  0      	 							/* 101: 0 */
  .word  0     	 							/* 102: 0 */
  .word  0      					 			/* 103: 0 */
  .word  0      					 			/* 104: 0 */
  .word  0      					 			/* 105: 0 */
  .word  0      					 			/* 106: 0 */
  .word  LPUART_IRQHandler       			/* 107: LPUART_IRQHandler */
  .word  0       	                		/* 108: 0 */
  .word  LPTIMER0_IRQHandler       			/* 109: LPTIMER0_IRQHandler */
  .word  LPTIMER1_IRQHandler       			/* 110: LPTIMER1_IRQHandler */
  .word  0       							/* 111: 0 */
  .word  0       							/* 112: 0 */
  .word  0       							/* 113: 0 */
  .word  0       							/* 114: 0 */ 
  .word  0       							/* 115: 0 */
  .word  0       							/* 116: 0 */
  .word  0       							/* 117: 0 */
  .word  0       							/* 118: 0 */
                        
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
* 
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler
  
   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler
  
   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler
  
   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler              
  
   .weak      WWDT_IRQHandler                   
   .thumb_set WWDT_IRQHandler,Default_Handler      
                  
   .weak      LVD_IRQHandler      
   .thumb_set LVD_IRQHandler,Default_Handler
               
   .weak      TAMPSTAMP_IRQHandler            
   .thumb_set TAMPSTAMP_IRQHandler,Default_Handler
            
   .weak      EFC_IRQHandler                  
   .thumb_set EFC_IRQHandler,Default_Handler

   .weak      RCM_IRQHandler                  
   .thumb_set RCM_IRQHandler,Default_Handler
                  
   .weak      EXTI0_IRQHandler         
   .thumb_set EXTI0_IRQHandler,Default_Handler
                  
   .weak      EXTI1_IRQHandler         
   .thumb_set EXTI1_IRQHandler,Default_Handler
                     
   .weak      EXTI2_IRQHandler         
   .thumb_set EXTI2_IRQHandler,Default_Handler 
                 
   .weak      EXTI3_IRQHandler         
   .thumb_set EXTI3_IRQHandler,Default_Handler
                        
   .weak      EXTI4_IRQHandler         
   .thumb_set EXTI4_IRQHandler,Default_Handler
                  
   .weak      DMAC0CH0_IRQHandler               
   .thumb_set DMAC0CH0_IRQHandler,Default_Handler
         
   .weak      DMAC0CH1_IRQHandler               
   .thumb_set DMAC0CH1_IRQHandler,Default_Handler
                  
   .weak      DMAC0CH2_IRQHandler               
   .thumb_set DMAC0CH2_IRQHandler,Default_Handler
                  
   .weak      DMAC0CH3_IRQHandler               
   .thumb_set DMAC0CH3_IRQHandler,Default_Handler 
                 
   .weak      DMAC0CH4_IRQHandler              
   .thumb_set DMAC0CH4_IRQHandler,Default_Handler
                  
   .weak      DMAC0CH5_IRQHandler               
   .thumb_set DMAC0CH5_IRQHandler,Default_Handler
                  
   .weak      DMAC0CH6_IRQHandler               
   .thumb_set DMAC0CH6_IRQHandler,Default_Handler

   .weak      DMAC0CH7_IRQHandler               
   .thumb_set DMAC0CH7_IRQHandler,Default_Handler
                  
   .weak      ADC0_IRQHandler      
   .thumb_set ADC0_IRQHandler,Default_Handler

   .weak      ADC1_IRQHandler      
   .thumb_set ADC1_IRQHandler,Default_Handler
               
   .weak      CAN0_IRQHandler   
   .thumb_set CAN0_IRQHandler,Default_Handler
            
   .weak      CAN1_IRQHandler                  
   .thumb_set CAN1_IRQHandler,Default_Handler
            
   .weak      EXTI5TO9_IRQHandler   
   .thumb_set EXTI5TO9_IRQHandler,Default_Handler
            
   .weak      TIM0_BRK_TIM8_IRQHandler            
   .thumb_set TIM0_BRK_TIM8_IRQHandler,Default_Handler
            
   .weak      TIM0_UP_TIM9_IRQHandler            
   .thumb_set TIM0_UP_TIM9_IRQHandler,Default_Handler
      
   .weak      TIM0_TRG_COM_TIM10_IRQHandler      
   .thumb_set TIM0_TRG_COM_TIM10_IRQHandler,Default_Handler
      
   .weak      TIM0_CC_IRQHandler   
   .thumb_set TIM0_CC_IRQHandler,Default_Handler

   .weak      TIM1_IRQHandler            
   .thumb_set TIM1_IRQHandler,Default_Handler

   .weak      TIM2_IRQHandler            
   .thumb_set TIM2_IRQHandler,Default_Handler
                  
   .weak      TIM3_IRQHandler            
   .thumb_set TIM3_IRQHandler,Default_Handler
                  
   .weak      I2C0_IRQHandler   
   .thumb_set I2C0_IRQHandler,Default_Handler
                     
   .weak      I2C1_IRQHandler   
   .thumb_set I2C1_IRQHandler,Default_Handler
                           
   .weak      SPI0_IRQHandler            
   .thumb_set SPI0_IRQHandler,Default_Handler
                        
   .weak      SPI1_IRQHandler            
   .thumb_set SPI1_IRQHandler,Default_Handler
                  
   .weak      UART0_IRQHandler      
   .thumb_set UART0_IRQHandler,Default_Handler
                     
   .weak      UART1_IRQHandler      
   .thumb_set UART1_IRQHandler,Default_Handler
                     
   .weak      UART2_IRQHandler      
   .thumb_set UART2_IRQHandler,Default_Handler
                  
   .weak      EXTI10TO15_IRQHandler               
   .thumb_set EXTI10TO15_IRQHandler,Default_Handler
               
   .weak      RTC_ALARM_IRQHandler               
   .thumb_set RTC_ALARM_IRQHandler,Default_Handler
            
   .weak      TIM7_BRK_TIM11_IRQHandler         
   .thumb_set TIM7_BRK_TIM11_IRQHandler,Default_Handler
            
   .weak      TIM7_UP_TIM12_IRQHandler         
   .thumb_set TIM7_UP_TIM12_IRQHandler,Default_Handler
         
   .weak      TIM7_TRG_COM_TIM13_IRQHandler            
   .thumb_set TIM7_TRG_COM_TIM13_IRQHandler,Default_Handler
      
   .weak      TIM7_CC_IRQHandler   
   .thumb_set TIM7_CC_IRQHandler,Default_Handler
                     
   .weak      SDIO_IRQHandler            
   .thumb_set SDIO_IRQHandler,Default_Handler
                     
   .weak      TIM4_IRQHandler            
   .thumb_set TIM4_IRQHandler,Default_Handler
                     
   .weak      SPI2_IRQHandler            
   .thumb_set SPI2_IRQHandler,Default_Handler
                     
   .weak      UART3_IRQHandler         
   .thumb_set UART3_IRQHandler,Default_Handler
                  
   .weak      UART4_IRQHandler         
   .thumb_set UART4_IRQHandler,Default_Handler
                  
   .weak      TIM5_IRQHandler                  
   .thumb_set TIM5_IRQHandler,Default_Handler
               
   .weak      TIM6_IRQHandler            
   .thumb_set TIM6_IRQHandler,Default_Handler
         
   .weak      DMAC1CH0_IRQHandler               
   .thumb_set DMAC1CH0_IRQHandler,Default_Handler
               
   .weak      DMAC1CH1_IRQHandler               
   .thumb_set DMAC1CH1_IRQHandler,Default_Handler
                  
   .weak      DMAC1CH2_IRQHandler               
   .thumb_set DMAC1CH2_IRQHandler,Default_Handler
            
   .weak      DMAC1CH3_IRQHandler               
   .thumb_set DMAC1CH3_IRQHandler,Default_Handler
            
   .weak      DMAC1CH4_IRQHandler               
   .thumb_set DMAC1CH4_IRQHandler,Default_Handler
            
   .weak      EMAC_IRQHandler      
   .thumb_set EMAC_IRQHandler,Default_Handler
                  
   .weak      SPI3_IRQHandler                  
   .thumb_set SPI3_IRQHandler,Default_Handler
            
   .weak      TS_IRQHandler   
   .thumb_set TS_IRQHandler,Default_Handler
                           
   .weak      OPA0_IRQHandler                  
   .thumb_set OPA0_IRQHandler,Default_Handler
                           
   .weak      OPA1_IRQHandler                  
   .thumb_set OPA1_IRQHandler,Default_Handler
                           
   .weak      DAC_IRQHandler                  
   .thumb_set DAC_IRQHandler,Default_Handler
                           
   .weak      DMAC1CH5_IRQHandler      
   .thumb_set DMAC1CH5_IRQHandler,Default_Handler
                     
   .weak      DMAC1CH6_IRQHandler               
   .thumb_set DMAC1CH6_IRQHandler,Default_Handler
                  
   .weak      DMAC1CH7_IRQHandler               
   .thumb_set DMAC1CH7_IRQHandler,Default_Handler
                  
   .weak      UART5_IRQHandler               
   .thumb_set UART5_IRQHandler,Default_Handler
                  
   .weak      I2C2_IRQHandler      
   .thumb_set I2C2_IRQHandler,Default_Handler
                        
   .weak      USB0_IRQHandler   
   .thumb_set USB0_IRQHandler,Default_Handler
                        
   .weak      USART6_IRQHandler   
   .thumb_set USART6_IRQHandler,Default_Handler
                        
   .weak      USART7_IRQHandler         
   .thumb_set USART7_IRQHandler,Default_Handler
               
   .weak      DCMI_IRQHandler            
   .thumb_set DCMI_IRQHandler,Default_Handler
               
   .weak      AES_IRQHandler         
   .thumb_set AES_IRQHandler,Default_Handler
            
   .weak      SHA_IRQHandler      
   .thumb_set SHA_IRQHandler,Default_Handler
                  
   .weak      FPU_IRQHandler            
   .thumb_set FPU_IRQHandler,Default_Handler
                     
   .weak      ACMP0_IRQHandler            
   .thumb_set ACMP0_IRQHandler,Default_Handler
               
   .weak      ACMP1_IRQHandler                  
   .thumb_set ACMP1_IRQHandler,Default_Handler   

   .weak      ACMP2_IRQHandler                  
   .thumb_set ACMP2_IRQHandler,Default_Handler  
   
   .weak      I2S0_IRQHandler                  
   .thumb_set I2S0_IRQHandler,Default_Handler 

   .weak      I2S1_IRQHandler                  
   .thumb_set I2S1_IRQHandler,Default_Handler 

   .weak      QSPI_IRQHandler                  
   .thumb_set QSPI_IRQHandler,Default_Handler

   .weak      IWDT_IRQHandler                  
   .thumb_set IWDT_IRQHandler,Default_Handler

   .weak      LPUART_IRQHandler                  
   .thumb_set LPUART_IRQHandler,Default_Handler

   .weak      LPTIMER0_IRQHandler                  
   .thumb_set LPTIMER0_IRQHandler,Default_Handler

   .weak      LPTIMER1_IRQHandler                  
   .thumb_set LPTIMER1_IRQHandler,Default_Handler
