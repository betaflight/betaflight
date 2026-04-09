/********************************** (C) COPYRIGHT  *******************************
* File Name          : core_riscv.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : RISC-V V3F_V5F Core Peripheral Access Layer Header File for CH32H417
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CORE_RISCV_H__
#define __CORE_RISCV_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* IO definitions */
#ifdef __cplusplus
  #define     __I     volatile                /* defines 'read only' permissions    */
#else
  #define     __I     volatile const          /* defines 'read only' permissions    */
#endif
#define       __O     volatile                /* defines 'write only' permissions   */
#define       __IO    volatile                /* defines 'read / write' permissions */

/* Standard Peripheral Library old types (maintained for legacy purpose) */
typedef __I uint64_t vuc64;  /* Read Only */
typedef __I uint32_t vuc32;  /* Read Only */
typedef __I uint16_t vuc16;  /* Read Only */
typedef __I uint8_t  vuc8;   /* Read Only */

typedef const uint64_t uc64;  /* Read Only */
typedef const uint32_t uc32;  /* Read Only */
typedef const uint16_t uc16;  /* Read Only */
typedef const uint8_t  uc8;   /* Read Only */

typedef __I int64_t vsc64;  /* Read Only */
typedef __I int32_t vsc32;  /* Read Only */
typedef __I int16_t vsc16;  /* Read Only */
typedef __I int8_t  vsc8;   /* Read Only */

typedef const int64_t sc64;  /* Read Only */
typedef const int32_t sc32;  /* Read Only */
typedef const int16_t sc16;  /* Read Only */
typedef const int8_t  sc8;   /* Read Only */

typedef __IO uint64_t  vu64;
typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;

typedef uint64_t  u64;
typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

typedef __IO int64_t  vs64;
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef int64_t  s64;
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;

typedef enum {NoREADY = 0, READY = !NoREADY} ErrorStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#define RV_STATIC_INLINE  static inline

/* memory mapped structure for HSEM */
typedef struct
{
  __IO uint32_t RX[32];
  uint8_t RESERVED0[0x80];
  __I  uint32_t RLRX[32]; 
  uint8_t RESERVED1[0x80];
  __IO uint32_t LSE;
  uint8_t RESERVED2[4];
  __IO uint32_t CLR;
  __IO uint32_t KEY;
  uint8_t RESERVED3[0xF0];
  __IO uint32_t IER;
  uint8_t RESERVED4[4];
  __IO uint32_t ISR;
  uint8_t RESERVE5[4];
  __I  uint32_t ISM;
  uint8_t RESERVED6[4];
  __I  uint32_t LSM;
}HSEM_Type;

/* memory mapped structure for IPC */
typedef struct
{
  __IO uint32_t CTLR;
  __I  uint32_t ISR;
  __I  uint32_t ISM;
  uint32_t RESERVED0;
  __IO uint32_t ENA;
  __IO uint32_t STS;
  __O uint32_t SET;
  __O uint32_t CLR;
  __IO uint32_t MSG[4];
}IPC_Type;

/* memory mapped structure for Program Fast Interrupt Controller (PFIC) */
typedef struct{
  __I  uint32_t ISR[8];
  __I  uint32_t IPR[8];
  __IO uint32_t ITHRESDR;
  uint32_t RESERVED;
  __IO uint32_t CFGR;
  __I  uint32_t GISR;
  __IO uint8_t VTFIDR[4];
  uint8_t RESERVED0[12];
  __IO uint32_t VTFADDR[4];
  uint8_t RESERVED1[0x90];
  __O  uint32_t IENR[8];
  uint8_t RESERVED2[0x60];
  __O  uint32_t IRER[8];
  uint8_t RESERVED3[0x60];
  __O  uint32_t IPSR[8];
  uint8_t RESERVED4[0x60];
  __O  uint32_t IPRR[8];
  uint8_t RESERVED5[0x60];
  __IO uint32_t IACTR[8];
  uint8_t RESERVED6[0xE0];
  __IO uint8_t IPRIOR[256];
  uint8_t RESERVED7[0x100]; 
  __IO uint8_t IALLOCR[256];
  __I  uint32_t IAUTR[8];
  __IO uint32_t WAKEIP[2];
  uint8_t RESERVED8[0x58]; 
  __I  uint32_t CSTAR[2];
  uint8_t RESERVED9[0x4F8]; 
  __IO uint32_t EENR;
  __IO uint32_t EPR;
  __IO uint32_t EWUPR;
  uint8_t RESERVED10[0x84]; 
  __IO uint32_t SCTLR;
}PFIC_Type;

/* memory mapped structure for SysTick */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t ISR;  //only for SysTick0
  __IO uint32_t CNT;
  uint32_t RESERVED0;
  __IO uint32_t CMP;
}SysTick_Type;

#define HSEM          ((HSEM_Type *) 0xE000C000 )
#define IPC           ((IPC_Type *) 0xE000D000 )
#define PFIC          ((PFIC_Type *) 0xE000E000 )
#define NVIC          PFIC
#define	NVIC_KEY3		  ((uint32_t)0xBEEF0000)

#define SysTick0      ((SysTick_Type *) 0xE000F000)
#define SysTick1      ((SysTick_Type *) 0xE000F080)

/* Core_ID */
#define Core_ID_V3F   ((uint8_t)0x00)
#define Core_ID_V5F   ((uint8_t)0x01)

/*********************************************************************
 * @fn      __enable_irq
 *
 * @brief   Enable Global Interrupt
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __enable_irq(void)
{
  __asm volatile ("csrs 0x800, %0" : : "r" (0x88) );
}

/*********************************************************************
 * @fn      __disable_irq
 *
 * @brief   Disable Global Interrupt
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __disable_irq(void)
{
  __asm volatile ("csrc 0x800, %0" : : "r" (0x88) );
  __asm volatile ("fence.i");
}

/*********************************************************************
 * @fn      __NOP
 *
 * @brief   nop
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __NOP(void)
{
  __asm volatile ("nop");
}

/*********************************************************************
 * @fn      NVIC_EnableIRQ
 *
 * @brief   Enable Interrupt
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  NVIC->IENR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      NVIC_DisableIRQ
 *
 * @brief   Disable Interrupt
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  NVIC->IRER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
  __asm volatile ("fence.i");
}

/*********************************************************************
 * @fn      NVIC_GetStatusIRQ
 *
 * @brief   Get Interrupt Enable State
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  1 - Interrupt Enable
 *          0 - Interrupt Disable
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetStatusIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((NVIC->ISR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      NVIC_GetPendingIRQ
 *
 * @brief   Get Interrupt Pending State
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  1 - Interrupt Pending Enable
 *          0 - Interrupt Pending Disable
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((NVIC->IPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      NVIC_SetPendingIRQ
 *
 * @brief   Set Interrupt Pending
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  NVIC->IPSR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      NVIC_ClearPendingIRQ
 *
 * @brief   Clear Interrupt Pending
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  NVIC->IPRR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      NVIC_GetActive
 *
 * @brief   Get Interrupt Active State
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  1 - Interrupt Active
 *          0 - Interrupt No Active
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((NVIC->IACTR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      NVIC_SetPriority
 *
 * @brief   Set Interrupt Priority
 *
 * @param   IRQn - Interrupt Numbers
 *          V5F:
 *            interrupt nesting enable- 5~8 Level(CSR-0x804 bit1 = 1 bit[3:2] = 3)
 *              priority - bit[7:5] - Preemption Priority
 *                         bit[4] - Sub priority
 *                         bit[3:0] - Reserve
 *            interrupt nesting enable- 3~4 Level(CSR-0x804 bit1 = 1 bit[3:2] = 2)
 *              priority - bit[7:6] - Preemption Priority
 *                         bit[5:4] - Sub priority
 *                         bit[3:0] - Reserve
 *            interrupt nesting enable-2 Level(CSR-0x804 bit1 = 1 bit[3:2] = 1)
 *              priority - bit[7] - Preemption Priority
 *                         bit[6:4] - Sub priority
 *                         bit[3:0] - Reserve
 *            interrupt nesting disable(CSR-0x804 bit1 = 0)
 *             priority - bit[7:4] - Sub priority
 *                         bit[3:0] - Reserve
 *          V3F:
 *            interrupt nesting enable-2 Level(CSR-0x804 bit1 = 1 bit[3:2] = 1)
 *              priority - bit[7] - Preemption Priority
 *                         bit[6:4] - Sub priority
 *                         bit[3:0] - Reserve
 *            interrupt nesting disable(CSR-0x804 bit1 = 0)
 *              priority - bit[7:4] - Sub priority
 *                         bit[3:0] - Reserve
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_SetPriority(IRQn_Type IRQn, uint8_t priority)
{
  NVIC->IPRIOR[(uint32_t)(IRQn)] = priority;
}

/*********************************************************************
 * @fn      NVIC_GetAllocateIRQ
 *
 * @brief   Get Interrupt Allocate 
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  1 - Allocate to V5F
 *          0 - Allocate to V3F
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetAllocateIRQ(IRQn_Type IRQn)
{
  return((uint32_t)((NVIC->IALLOCR[(uint32_t)(IRQn)] & (1 << 0))?1:0));
}

/*********************************************************************
 * @fn      NVIC_SetAllocateIRQ
 *
 * @brief   Set Interrupt Allocate
 *
 * @param   IRQn - Interrupt Numbers ( >31 )
 *          Core_ID - Core ID
 *            Core_ID_V5F - V5F
 *            Core_ID_V3F - V3F
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_SetAllocateIRQ(IRQn_Type IRQn, uint8_t Core_ID)
{
  if(IRQn > 31)  return ;
  NVIC->IALLOCR[(uint32_t)(IRQn)] = Core_ID;
}

/*********************************************************************
 * @fn      NVIC_OwnCoreGetAllocateIRQ
 *
 * @brief   Own Core Get Interrupt Allocate 
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  1 - Allocate to own core
 *          0 - No allocate to own core
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_OwnCoreGetAllocateIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((NVIC->IAUTR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      __WFI
 *
 * @brief   Wait for Interrupt
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __WFI(void)
{
  NVIC->SCTLR &= ~(1<<3);	// wfi
  asm volatile ("wfi");
}

/*********************************************************************
 * @fn      _SEV
 *
 * @brief   Set Event
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void _SEV(void)
{
  uint32_t t;

  t = NVIC->SCTLR;
  NVIC->SCTLR |= (1<<3)|(1<<5);
  NVIC->SCTLR = (NVIC->SCTLR & ~(1<<5)) | ( t & (1<<5));
}

/*********************************************************************
 * @fn      _WFE
 *
 * @brief   Wait for Events
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void _WFE(void)
{
  NVIC->SCTLR |= (1<<3);
  asm volatile ("wfi");
}

/*********************************************************************
 * @fn      __WFE
 *
 * @brief   Wait for Events
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __WFE(void)
{
  NVIC->EPR = 0xFFFFFFFF;
  NVIC->SCTLR |= (1<<3);
  asm volatile ("wfi");
}

/*********************************************************************
 * @fn      SetVTFIRQ
 *
 * @brief   Set VTF Interrupt
 *
 * @param   addr - VTF interrupt service function base address.
 *          IRQn -Interrupt Numbers
 *          num - VTF Interrupt Numbers
 *          NewState - DISABLE or ENABLE
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void SetVTFIRQ(uint32_t addr, IRQn_Type IRQn, uint8_t num, FunctionalState NewState)
{
  if(num > 3)  return ;

  if (NewState != DISABLE)
  {
    NVIC->VTFIDR[num] = IRQn;
    NVIC->VTFADDR[num] = ((addr&0xFFFFFFFE)|0x1);
  }
  else
  {
    NVIC->VTFIDR[num] = IRQn;
    NVIC->VTFADDR[num] = ((addr&0xFFFFFFFE)&(~0x1));
  }
}

/*********************************************************************
 * @fn      NVIC_SystemReset
 *
 * @brief   Initiate a system reset request
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_SystemReset(void)
{
  NVIC->CFGR = NVIC_KEY3|(1<<7);
}

/*********************************************************************
 * @fn      NVIC_WakeUp_V3F
 *
 * @brief   Wake up V3F and set address for PC
 *
 * @param   addr - V3F wake up address for PC(addr%1024 == 0).
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_WakeUp_V3F(uint32_t addr)
{
  addr &= ~0x3FF; 
  NVIC->WAKEIP[0] = addr;
  NVIC->SCTLR |= (1<<5);
}

/*********************************************************************
 * @fn      NVIC_WakeUp_V5F
 *
 * @brief   Wake up V5F and set address for PC
 *
 * @param   addr - V5F wake up address for PC(addr%1024 == 0).
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_WakeUp_V5F(uint32_t addr)
{
  addr &= ~0x3FF; 
  NVIC->WAKEIP[1] = addr;
  NVIC->SCTLR |= (1<<5);
}

/*********************************************************************
 * @fn      NVIC_GetCurrentCoreID
 *
 * @brief   Get Current Core ID
 *
 * @return  1 - V5F
 *          0 - V3F
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetCurrentCoreID(void)
{
  return((uint32_t)(NVIC->SCTLR & (1<<16))?1:0);
}

/*********************************************************************
 * @fn      NVIC_EventWakeUPCmd
 *
 * @brief   Enables or disables the event wake up.
 *
 * @param   IRQn -Interrupt Numbers ( <=31 )
 *          NewState - DISABLE or ENABLE
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_EventWakeUPCmd(IRQn_Type IRQn, FunctionalState NewState)
{
  if(IRQn > 31)  return ;

  if (NewState != DISABLE)
  {
    NVIC->EENR |= IRQn;
  }
  else
  {
    NVIC->EENR &= (~IRQn);
  }
}

/*********************************************************************
 * @fn      NVIC_ClearPendingWakeuUpEvent
 *
 * @brief   Clear wake up event pending 
 *
 * @param   IRQn - Interrupt Numbers (from 8 to 31)
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_ClearPendingWakeuUpEvent(IRQn_Type IRQn)
{
  if((IRQn > 31) && (IRQn < 8))  return ;
  NVIC->EPR = IRQn;
}

/*********************************************************************
 * @fn      NVIC_GetPendingWakeuUpEvent
 *
 * @brief   Get wake up event pending 
 *
 * @param   IRQn - Interrupt Numbers( <=31)
 *
 * @return  1 - Event pending
 *          0 - Event no pending
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetPendingWakeuUpEvent(IRQn_Type IRQn)
{
  return((uint32_t)((NVIC->EPR & (1 << IRQn))?1:0));
}

/*********************************************************************
 * @fn      NVIC_GetWakeuUpEvent
 *
 * @brief   Get wake up event 
 *
 * @param   IRQn - Interrupt Numbers( <=31)
 *
 * @return  1 - The event wake up core 
 *          0 - The event do not wake up core 
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t NVIC_GetWakeuUpEvent(IRQn_Type IRQn)
{
  return((uint32_t)((NVIC->EWUPR & (1 << IRQn))?1:0));
}

/*********************************************************************
 * @fn      ASM_MCPY
 *
 * @brief   Implement the assembly instruction function of mcpy, copy
 *        the continuous data from the starting address of SrcAddrStart
 *        to the ending address of SrcAddrEnd to the starting address
 *        of DstAddrStart.(Only for V3F)
 *
 * @param   SA - Copy the start address of the source region.
 *          EA - Copy the end address of the source region.
 *          DA - Copy the start address of the destination region.
 *
 * @return  none
 */
static inline void ASM_MCPY(uint8_t* DA,uint8_t* SA,uint8_t* EA)
{
    __asm__ volatile("mcpy %2, %0, %1" :"+r"(SA) , "+r"(DA):"r"(EA):"memory");
}

/*********************************************************************
 * @fn      __AMOADD_W
 *   
 * @brief   Atomic Add with 32bit value
 *          Atomically ADD 32bit value with value in memory using amoadd.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be ADDed
 *
 * @return  return memory value + add value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOADD_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amoadd.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOAND_W
 *
 * @brief   Atomic And with 32bit value
 *          Atomically AND 32bit value with value in memory using amoand.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be ANDed
 *
 * @return  return memory value & and value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOAND_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amoand.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOMAX_W
 *
 * @brief   Atomic signed MAX with 32bit value
 *          Atomically signed max compare 32bit value with value in memory using amomax.d.
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be compared
 *
 * @return  return the bigger value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOMAX_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amomax.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOMAXU_W
 *
 * @brief   Atomic unsigned MAX with 32bit value
 *          Atomically unsigned max compare 32bit value with value in memory using amomaxu.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be compared
 *             
 * @return  return the bigger value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t __AMOMAXU_W(volatile uint32_t *addr, uint32_t value)
{
    uint32_t result;

    __asm volatile ("amomaxu.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOMIN_W
 *
 * @brief   Atomic signed MIN with 32bit value
 *          Atomically signed min compare 32bit value with value in memory using amomin.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be compared
 *
 * @return  return the smaller value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOMIN_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amomin.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOMINU_W
 *
 * @brief   Atomic unsigned MIN with 32bit value
 *          Atomically unsigned min compare 32bit value with value in memory using amominu.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be compared
 *
 * @return  return the smaller value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t __AMOMINU_W(volatile uint32_t *addr, uint32_t value)
{
    uint32_t result;

    __asm volatile ("amominu.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOOR_W
 *  
 * @brief   Atomic OR with 32bit value
 *          Atomically OR 32bit value with value in memory using amoor.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be ORed
 * 
 * @return  return memory value | and value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOOR_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amoor.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/*********************************************************************
 * @fn      __AMOSWAP_W
 *
 * @brief   Atomically swap new 32bit value into memory using amoswap.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          newval - New value to be stored into the address
 *
 * @return  return the original value in memory
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE uint32_t __AMOSWAP_W(volatile uint32_t *addr, uint32_t newval)
{
    uint32_t result;

    __asm volatile ("amoswap.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(newval) : "memory");
    return result;
}

/*********************************************************************
 * @fn      __AMOXOR_W     
 *
 * @brief   Atomic XOR with 32bit value
 *          Atomically XOR 32bit value with value in memory using amoxor.d.
 *
 * @param   addr - Address pointer to data, address need to be 4byte aligned
 *          value - value to be XORed
 *
 * @return  return memory value ^ and value
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE int32_t __AMOXOR_W(volatile int32_t *addr, int32_t value)
{
    int32_t result;

    __asm volatile ("amoxor.w %0, %2, %1" : \
            "=r"(result), "+A"(*addr) : "r"(value) : "memory");
    return *addr;
}

/* Core_Exported_Functions */  
extern uint32_t __get_FFLAGS(void);
extern void __set_FFLAGS(uint32_t value);
extern uint32_t __get_FRM(void);
extern void __set_FRM(uint32_t value);
extern uint32_t __get_FCSR(void);
extern void __set_FCSR(uint32_t value);
extern uint32_t __get_MSTATUS(void);
extern void __set_MSTATUS(uint32_t value);
extern uint32_t __get_MISA(void);
extern void __set_MISA(uint32_t value);
extern uint32_t __get_MTVEC(void);
extern void __set_MTVEC(uint32_t value);
extern uint32_t __get_MSCRATCH(void);
extern void __set_MSCRATCH(uint32_t value);
extern uint32_t __get_MEPC(void);
extern void __set_MEPC(uint32_t value);
extern uint32_t __get_MCAUSE(void);
extern void __set_MCAUSE(uint32_t value);
extern uint32_t __get_MTVAL(void);
extern void __set_MTVAL(uint32_t value);
extern uint32_t __get_MVENDORID(void);
extern uint32_t __get_MARCHID(void);
extern uint32_t __get_MIMPID(void);
extern uint32_t __get_MHARTID(void);
extern uint32_t __get_SP(void);
void __set_MCOUNT_INHIBIT(uint32_t value);
uint32_t __get_MCOUNT_INHIBIT(void);
void __set_MCYCLE(uint32_t value);
uint32_t __get_MCYCLE(void);
void __set_MINSTRET(uint32_t value);
uint32_t __get_MINSTRET(void);
void __set_UCYCLE(uint32_t value);
uint32_t __get_UCYCLE(void);
void __set_UINSTRET(uint32_t value);
uint32_t __get_UINSTRET(void);

#ifdef __cplusplus
}
#endif

#endif





