/********************************** (C) COPYRIGHT *******************************
* File Name          : startup_ch32h417_v5f.s
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : CH32H417 vector table t0 V5F for eclipse toolchain.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

	.section	.init,"ax",@progbits
	.global	_start
	.align	2
_start:
	# j	handle_reset
    lui ra, %Hi(handle_reset)
    jr  %Lo(handle_reset)(ra)
    .section    .vector,"ax",@progbits
    .align  1
_vector_base:
    .option norvc;
    .word   _start
    .word   0
    .word   NMI_Handler              
    .word   HardFault_Handler         
    .word   0
    .word   Ecall_M_Mode_Handler      
    .word   0
    .word   0
    .word   Ecall_U_Mode_Handler      
    .word   Break_Point_Handler      
    .word   0
    .word   0
    .word   SysTick0_Handler          
    .word   SysTick1_Handler           
    .word   SW_Handler
    .word   0
    .word   IPC_CH0_Handler          
    .word   IPC_CH1_Handler          
    .word   IPC_CH2_Handler          
    .word   IPC_CH3_Handler          
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   HSEM_Handler
    .word   0
    .word   0
    .word   0
    /* External Interrupts */
    .word   WWDG_IRQHandler            
    .word   EXTI15_8_IRQHandler         
    .word   FLASH_IRQHandler         
    .word   RCC_IRQHandler   
    .word   EXTI7_0_IRQHandler          
    .word   SPI1_IRQHandler           
    .word   DMA1_Channel2_IRQHandler   
    .word   DMA1_Channel3_IRQHandler   
    .word   DMA1_Channel4_IRQHandler   
    .word   DMA1_Channel5_IRQHandler  
    .word   DMA1_Channel6_IRQHandler   
    .word   DMA1_Channel7_IRQHandler   
    .word   DMA1_Channel8_IRQHandler  
    .word   USART2_IRQHandler          
    .word   I2C1_EV_IRQHandler        
    .word   I2C1_ER_IRQHandler         
    .word   USART1_IRQHandler          
    .word   SPI2_IRQHandler           
    .word   SPI3_IRQHandler            
    .word   SPI4_IRQHandler           
    .word   I2C2_EV_IRQHandler      
    .word   I2C2_ER_IRQHandler         
    .word   USBPD_IRQHandler        
    .word   USBPDWakeUp_IRQHandler         
    .word   USBHS_IRQHandler           
    .word   DMA1_Channel1_IRQHandler   
    .word   CAN1_SCE_IRQHandler       
    .word   CAN1_TX_IRQHandler      
    .word   CAN1_RX0_IRQHandler       
    .word   0       
    .word   USBSS_IRQHandler          
    .word   USBSS_LINK_IRQHandler           
    .word   USBHSWakeup_IRQHandler     
    .word   USBSSWakeup_IRQHandler    
    .word   RTCAlarm_IRQHandler        
    .word   USBFS_IRQHandler          
    .word   USBFSWakeup_IRQHandler     
    .word   ADC1_2_IRQHandler                                
    .word   TIM1_BRK_IRQHandler              
    .word   TIM1_UP_IRQHandler               
    .word   TIM1_TRG_COM_IRQHandler         
    .word   TIM1_CC_IRQHandler             
    .word   TIM2_IRQHandler                  
    .word   TIM3_IRQHandler                  
    .word   TIM4_IRQHandler                 
    .word   TIM5_IRQHandler                 
    .word   I2C3_EV_IRQHandler               
    .word   I2C3_ER_IRQHandler               
    .word   I2C4_EV_IRQHandler               
    .word   I2C4_ER_IRQHandler              
    .word   QSPI1_IRQHandler              
    .word   SERDES_IRQHandler               
    .word   USART3_IRQHandler              
    .word   USART4_IRQHandler             
    .word   TIM8_BRK_IRQHandler              
    .word   TIM8_UP_IRQHandler               
    .word   TIM8_TRG_COM_IRQHandler          
    .word   TIM8_CC_IRQHandler              
    .word   TIM9_IRQHandler                
    .word   TIM10_IRQHandler               
    .word   TIM11_IRQHandler                
    .word   TIM12_IRQHandler               
    .word   FMC_IRQHandler                  
    .word   SDMMC_IRQHandler                
    .word   LPTIM1_IRQHandler              
    .word   LPTIM2_IRQHandler               
    .word   USART5_IRQHandler               
    .word   USART6_IRQHandler              
    .word   TIM6_IRQHandler                 
    .word   TIM7_IRQHandler                 
    .word   DMA2_Channel1_IRQHandler         
    .word   DMA2_Channel2_IRQHandler         
    .word   DMA2_Channel3_IRQHandler         
    .word   DMA2_Channel4_IRQHandler         
    .word   DMA2_Channel5_IRQHandler        
    .word   DMA2_Channel6_IRQHandler       
    .word   DMA2_Channel7_IRQHandler        
    .word   DMA2_Channel8_IRQHandler         
    .word   ETH_IRQHandler                  
    .word   ETH_WKUP_IRQHandler 
    .word   CAN2_SCE_IRQHandler       
    .word   CAN2_TX_IRQHandler      
    .word   CAN2_RX0_IRQHandler       
    .word   0                  
    .word   USART7_IRQHandler               
    .word   USART8_IRQHandler             
    .word   I3C_EV_IRQHandler                
    .word   I3C_ER_IRQHandler                
    .word   DVP_IRQHandler                   
    .word   ECDC_IRQHandler                  
    .word   PIOC_IRQHandler               
    .word   SAI_IRQHandler
    .word   LTDC_IRQHandler               
    .word   GPHA_IRQHandler 
    .word   0              
    .word   DFSDM0_IRQHandler              
    .word   DFSDM1_IRQHandler  
    .word   0  
    .word   0             
    .word   SWPMI_IRQHandler   
    .word   0  
    .word   0                   
    .word   QSPI2_IRQHandler              
    .word   SWPMI_WKUP_IRQHandler          
    .word   CAN3_SCE_IRQHandler       
    .word   CAN3_TX_IRQHandler      
    .word   CAN3_RX0_IRQHandler       
    .word   0              
    .word   LPTIM2_WKUP_IRQHandler         
    .word   LPTIM1_WKUP_IRQHandler          
    .word   I3C_WKUP_IRQHandler              
    .word   RTC_IRQHandler                    
    .word   HSADC_IRQHandler                  
    .word   UHSIF_IRQHandler                 
    .word   RNG_IRQHandler                    
    .word   SDIO_IRQHandler 
    .word   USART_WKUP_IRQHandler          

    .option rvc;
    .section    .text.vector_handler, "ax", @progbits
    .weak   NMI_Handler              
    .weak   HardFault_Handler         
    .weak   Ecall_M_Mode_Handler      
    .weak   Ecall_U_Mode_Handler      
    .weak   Break_Point_Handler      
    .weak   SysTick0_Handler          
    .weak   SysTick1_Handler  
    .weak   SW_Handler         
    .weak   IPC_CH0_Handler          
    .weak   IPC_CH1_Handler          
    .weak   IPC_CH2_Handler          
    .weak   IPC_CH3_Handler  
    .weak   HSEM_Handler        
    .weak   WWDG_IRQHandler            
    .weak   EXTI15_8_IRQHandler         
    .weak   FLASH_IRQHandler         
    .weak   RCC_IRQHandler 
    .weak   EXTI7_0_IRQHandler          
    .weak   SPI1_IRQHandler           
    .weak   DMA1_Channel2_IRQHandler   
    .weak   DMA1_Channel3_IRQHandler   
    .weak   DMA1_Channel4_IRQHandler   
    .weak   DMA1_Channel5_IRQHandler  
    .weak   DMA1_Channel6_IRQHandler   
    .weak   DMA1_Channel7_IRQHandler   
    .weak   DMA1_Channel8_IRQHandler  
    .weak   USART2_IRQHandler          
    .weak   I2C1_EV_IRQHandler        
    .weak   I2C1_ER_IRQHandler         
    .weak   USART1_IRQHandler          
    .weak   SPI2_IRQHandler           
    .weak   SPI3_IRQHandler            
    .weak   SPI4_IRQHandler           
    .weak   I2C2_EV_IRQHandler      
    .weak   I2C2_ER_IRQHandler         
    .weak   USBPD_IRQHandler        
    .weak   USBPDWakeUp_IRQHandler         
    .weak   USBHS_IRQHandler           
    .weak   DMA1_Channel1_IRQHandler   
    .weak   CAN1_SCE_IRQHandler       
    .weak   CAN1_TX_IRQHandler      
    .weak   CAN1_RX0_IRQHandler           
    .weak   USBSS_IRQHandler          
    .weak   USBSS_LINK_IRQHandler           
    .weak   USBHSWakeup_IRQHandler     
    .weak   USBSSWakeup_IRQHandler    
    .weak   RTCAlarm_IRQHandler        
    .weak   USBFS_IRQHandler          
    .weak   USBFSWakeup_IRQHandler     
    .weak   ADC1_2_IRQHandler                                
    .weak   TIM1_BRK_IRQHandler              
    .weak   TIM1_UP_IRQHandler               
    .weak   TIM1_TRG_COM_IRQHandler         
    .weak   TIM1_CC_IRQHandler             
    .weak   TIM2_IRQHandler                  
    .weak   TIM3_IRQHandler                  
    .weak   TIM4_IRQHandler                 
    .weak   TIM5_IRQHandler                 
    .weak   I2C3_EV_IRQHandler               
    .weak   I2C3_ER_IRQHandler               
    .weak   I2C4_EV_IRQHandler               
    .weak   I2C4_ER_IRQHandler              
    .weak   QSPI1_IRQHandler              
    .weak   SERDES_IRQHandler               
    .weak   USART3_IRQHandler              
    .weak   USART4_IRQHandler             
    .weak   TIM8_BRK_IRQHandler              
    .weak   TIM8_UP_IRQHandler               
    .weak   TIM8_TRG_COM_IRQHandler          
    .weak   TIM8_CC_IRQHandler              
    .weak   TIM9_IRQHandler                
    .weak   TIM10_IRQHandler               
    .weak   TIM11_IRQHandler                
    .weak   TIM12_IRQHandler               
    .weak   FMC_IRQHandler                  
    .weak   SDMMC_IRQHandler                
    .weak   LPTIM1_IRQHandler              
    .weak   LPTIM2_IRQHandler               
    .weak   USART5_IRQHandler               
    .weak   USART6_IRQHandler              
    .weak   TIM6_IRQHandler                 
    .weak   TIM7_IRQHandler                 
    .weak   DMA2_Channel1_IRQHandler         
    .weak   DMA2_Channel2_IRQHandler         
    .weak   DMA2_Channel3_IRQHandler         
    .weak   DMA2_Channel4_IRQHandler         
    .weak   DMA2_Channel5_IRQHandler        
    .weak   DMA2_Channel6_IRQHandler       
    .weak   DMA2_Channel7_IRQHandler        
    .weak   DMA2_Channel8_IRQHandler         
    .weak   ETH_IRQHandler                  
    .weak   ETH_WKUP_IRQHandler  
    .weak   CAN2_SCE_IRQHandler       
    .weak   CAN2_TX_IRQHandler      
    .weak   CAN2_RX0_IRQHandler                
    .weak   USART7_IRQHandler               
    .weak   USART8_IRQHandler             
    .weak   I3C_EV_IRQHandler                
    .weak   I3C_ER_IRQHandler                
    .weak   DVP_IRQHandler                   
    .weak   ECDC_IRQHandler                                   
    .weak   PIOC_IRQHandler               
    .weak   SAI_IRQHandler
    .weak   LTDC_IRQHandler               
    .weak   GPHA_IRQHandler               
    .weak   DFSDM0_IRQHandler              
    .weak   DFSDM1_IRQHandler              
    .weak   SWPMI_IRQHandler                 
    .weak   QSPI2_IRQHandler              
    .weak   SWPMI_WKUP_IRQHandler          
    .weak   CAN3_SCE_IRQHandler           
    .weak   CAN3_TX_IRQHandler             
    .weak   CAN3_RX0_IRQHandler                    
    .weak   LPTIM2_WKUP_IRQHandler         
    .weak   LPTIM1_WKUP_IRQHandler          
    .weak   I3C_WKUP_IRQHandler              
    .weak   RTC_IRQHandler                    
    .weak   HSADC_IRQHandler                  
    .weak   UHSIF_IRQHandler                 
    .weak   RNG_IRQHandler                    
    .weak   SDIO_IRQHandler 
    .weak   USART_WKUP_IRQHandler   

NMI_Handler:              
HardFault_Handler:         
Ecall_M_Mode_Handler:      
Ecall_U_Mode_Handler:      
Break_Point_Handler:      
SysTick0_Handler:          
SysTick1_Handler: 
SW_Handler:          
IPC_CH0_Handler:          
IPC_CH1_Handler:          
IPC_CH2_Handler:          
IPC_CH3_Handler: 
HSEM_Handler:         
WWDG_IRQHandler:            
EXTI15_8_IRQHandler:         
FLASH_IRQHandler:         
RCC_IRQHandler:   
EXTI7_0_IRQHandler:          
SPI1_IRQHandler:           
DMA1_Channel2_IRQHandler:   
DMA1_Channel3_IRQHandler:   
DMA1_Channel4_IRQHandler:   
DMA1_Channel5_IRQHandler:  
DMA1_Channel6_IRQHandler:   
DMA1_Channel7_IRQHandler:   
DMA1_Channel8_IRQHandler:  
USART2_IRQHandler:          
I2C1_EV_IRQHandler:        
I2C1_ER_IRQHandler:         
USART1_IRQHandler:          
SPI2_IRQHandler:           
SPI3_IRQHandler:            
SPI4_IRQHandler:           
I2C2_EV_IRQHandler:      
I2C2_ER_IRQHandler:         
USBPD_IRQHandler:        
USBPDWakeUp_IRQHandler:         
USBHS_IRQHandler:           
DMA1_Channel1_IRQHandler:   
CAN1_SCE_IRQHandler:       
CAN1_TX_IRQHandler:      
CAN1_RX0_IRQHandler:             
USBSS_IRQHandler:          
USBSS_LINK_IRQHandler:           
USBHSWakeup_IRQHandler:     
USBSSWakeup_IRQHandler:    
RTCAlarm_IRQHandler:        
USBFS_IRQHandler:          
USBFSWakeup_IRQHandler:     
ADC1_2_IRQHandler:                                
TIM1_BRK_IRQHandler:              
TIM1_UP_IRQHandler:               
TIM1_TRG_COM_IRQHandler:         
TIM1_CC_IRQHandler:             
TIM2_IRQHandler:                  
TIM3_IRQHandler:                  
TIM4_IRQHandler:                 
TIM5_IRQHandler:                 
I2C3_EV_IRQHandler:               
I2C3_ER_IRQHandler:               
I2C4_EV_IRQHandler:               
I2C4_ER_IRQHandler:              
QSPI1_IRQHandler:              
SERDES_IRQHandler:               
USART3_IRQHandler:              
USART4_IRQHandler:             
TIM8_BRK_IRQHandler:              
TIM8_UP_IRQHandler:               
TIM8_TRG_COM_IRQHandler:          
TIM8_CC_IRQHandler:              
TIM9_IRQHandler:                
TIM10_IRQHandler:               
TIM11_IRQHandler:                
TIM12_IRQHandler:               
FMC_IRQHandler:                  
SDMMC_IRQHandler:                
LPTIM1_IRQHandler:              
LPTIM2_IRQHandler:               
USART5_IRQHandler:               
USART6_IRQHandler:              
TIM6_IRQHandler:                 
TIM7_IRQHandler:                 
DMA2_Channel1_IRQHandler:         
DMA2_Channel2_IRQHandler:         
DMA2_Channel3_IRQHandler:         
DMA2_Channel4_IRQHandler:         
DMA2_Channel5_IRQHandler:        
DMA2_Channel6_IRQHandler:       
DMA2_Channel7_IRQHandler:        
DMA2_Channel8_IRQHandler:         
ETH_IRQHandler:                  
ETH_WKUP_IRQHandler:   
CAN2_SCE_IRQHandler:       
CAN2_TX_IRQHandler:      
CAN2_RX0_IRQHandler:           
USART7_IRQHandler:               
USART8_IRQHandler:             
I3C_EV_IRQHandler:                
I3C_ER_IRQHandler:                
DVP_IRQHandler:                   
ECDC_IRQHandler:                  
PIOC_IRQHandler:               
SAI_IRQHandler:
LTDC_IRQHandler:               
GPHA_IRQHandler:               
DFSDM0_IRQHandler:              
DFSDM1_IRQHandler:              
SWPMI_IRQHandler:                 
QSPI2_IRQHandler:              
SWPMI_WKUP_IRQHandler:          
CAN3_SCE_IRQHandler:           
CAN3_TX_IRQHandler:             
CAN3_RX0_IRQHandler:                       
LPTIM2_WKUP_IRQHandler:         
LPTIM1_WKUP_IRQHandler:          
I3C_WKUP_IRQHandler:              
RTC_IRQHandler:                    
HSADC_IRQHandler:                  
UHSIF_IRQHandler:                 
RNG_IRQHandler:                    
SDIO_IRQHandler: 
USART_WKUP_IRQHandler:
1:
	j 1b

	.section	.handle_reset,"ax",@progbits
	.weak	handle_reset
	.align	1
handle_reset:
.option push 
.option	norelax 
	la gp, __global_pointer$
.option	pop 

	la sp, _eusrstack 

    /*Enable enhance read and continue read*/

    # lui a0, 0x40022
    # li a1,0x45670123
    # sw a1,4(a0)
    # li a2,0xCDEF89AB
    # sw a2,4(a0)

    # lw a2 ,0(a0)
    # li a1,0x880
    # or a2 ,a2,a1
    # sw a2,0(a0)
    # fence

/* Load TCM code  section from flash to TCM */
    la a0, tcm_code
    la a1, tcm_code_start
    la a2, tcm_code_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b

/* Load SRAM code  section from flash to RAM */
2:
    la a0, ram_code
    la a1, ram_code_start
    la a2, ram_code_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b

    
/* Load data section from flash to RAM */
2:
    /*close enhance read and continue read*/
    # lui a0, 0x40022
    # lw a1,0(a0)
    # li a2,0x3
    # and a1,a1,a2
    # sw a1,0(a0)
    # li a1 ,0x408080
    # sw a1,16(a0)


/* load fastram data */
	la a0, _sfastram_idata
	la a1, _sfastram_data
	la a2, _efastram_data
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b

/* load data */
2:
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b

    /* clear fastram bss */
2:
	la a0, _sfastram_bss
	la a1, _efastram_bss
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b

2:
/* Clear bss section */
	la a0, _sbss
	la a1, _ebss
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:
/* config hardware push/pop addr */
    li t0, 0x200E0000
    csrw 0xBC4, t0
/* Configure Prefetch */
    # li t0, 0x123733E0
    li t0, 0x345933E0
    # li t0, 0x233833E0
    csrw 0xBC0, t0


    csrwi 0xBD0, 0x4
	fence.i
	li t0, (1<<1 | 3<<24)
	csrc 0xBC2, t0
	fence.i

	/* config pmp3 */
	li a0, (0x804FFFF)  /* 0x20100000 --512KB */
	csrw pmpaddr3, a0
	li a0, 0x7F000000  
	csrw pmpcfg0, a0

    li t0, (1<<12)
    csrw 0xbc3, t0



    /* config dma and cpu priority */
    # li t0, (3<<18)
    # csrc 0xbc5, t0    /* DMA C0 > C1 */

    # li t0, 0x0A0A0A0A
    # csrw 0xbc6, t0


/* Configure nesting level */
	li t0, 0x03
	csrw 0xBC1, t0
/* Enable interrupt nesting and hardware stack */
    li t0, 0x0B
    csrw 0x804, t0
/* Enable floating point and global interrupt, configure privileged mode */
   	li t0, 0x7888           
   	csrw mstatus, t0

/* Configure the interrupt vector table recognition mode and entry address mode */
 	la t0, _vector_base
    ori t0, t0, 3           
	csrw mtvec, t0

    la a0, __libc_fini_array
    call atexit
    call __libc_init_array

	la t0, main
	csrw mepc, t0
	mret


