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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "x32m7xx.h"

#if !defined  (HSE_VALUE)
#define HSE_VALUE    ((uint32_t)25000000UL) /* Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (CSI_VALUE)
  #define CSI_VALUE           ((uint32_t)4000000UL) /* Value of the Internal oscillator in Hz*/
#endif /* CSI_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE           ((uint32_t)64000000UL) /* Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

#define VECT_TAB_OFFSET       ((uint32_t)0x00000000UL) /* Vector Table base offset field.   */

#define FLASH_BANK1_BASE      (0x15000000U)

#define FLASH_BANK2_BASE      (0x15080000U)

uint32_t SystemCoreClock = 600000000;

#ifndef TCM_SIZE_VALUE
#define TCM_SIZE_VALUE        (0x1E) /*TCM_SIZE=0x20:256K ITCM;256K DTCM;512K AXI_SRAM2/3*/
#endif

/** Private_Functions  */

/* defaule power supply is extern LDO. User can change the way of power supply*/
//#define PWR_SUPPLY_SELECTION       (PWR_LDO_SUPPLY)  /* External LDO Supply  */
#define PWR_SUPPLY_SELECTION       (PWR_DIRECT_SMPS_SUPPLY)  /* DCDC Supply  */
//#define PWR_SUPPLY_SELECTION       (PWR_EXTERNAL_SOURCE_SUPPLY)  /* VCAP Supply  */

/**
 *\*\name   ConfigTcmSize.
 *\*\fun    Config TCM_SIZE
 *\*\param  tcmSizeValue 
 *\*\    for X32M76x and X32M78x (The input parameters must be the following values):
 *\*\           0x00 :1024KB ITCM,  0   KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x01 :896 KB ITCM,  128 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x02 :768 KB ITCM,  256 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x03 :640 KB ITCM,  384 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x04 :512 KB ITCM,  512 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x05 :384 KB ITCM,  640 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1 
 *\*\           0x06 :256 KB ITCM,  768 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x07 :128 KB ITCM,  896 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x08 :0   KB ITCM,  1024KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x09 :896 KB ITCM,  0   KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0A :768 KB ITCM,  128 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0B :640 KB ITCM,  256 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0C :512 KB ITCM,  384 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0D :384 KB ITCM,  512 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0E :256 KB ITCM,  640 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x0F :128 KB ITCM,  768 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x10 :0   KB ITCM,  896 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x11 :768 KB ITCM,  0   KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x12 :640 KB ITCM,  128 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x13 :512 KB ITCM,  256 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x14 :384 KB ITCM,  384 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x15 :256 KB ITCM,  512 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x16 :128 KB ITCM,  640 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x17 :0   KB ITCM,  768 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x18 :640 KB ITCM,  0   KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x19 :512 KB ITCM,  128 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1A :384 KB ITCM,  256 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1B :256 KB ITCM,  384 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1C :128 KB ITCM,  512 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1D :0   KB ITCM,  640 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1E :512 KB ITCM,  0   KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1F :384 KB ITCM,  128 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x20 :256 KB ITCM,  256 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x21 :128 KB ITCM,  384 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x22 :0   KB ITCM,  512 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x23 :384 KB ITCM,  0   KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x24 :256 KB ITCM,  128 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x25 :128 KB ITCM,  256 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x26 :0   KB ITCM,  384 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x27 :256 KB ITCM,  0   KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x28 :128 KB ITCM,  128 KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x29 :0   KB ITCM,  256 KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2A :128 KB ITCM,  0   KB DTCM,  896 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2B :0   KB ITCM,  128 KB DTCM,  896 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2C~2F :0KB ITCM,  0   KB DTCM,  1024KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\    for X32M73x (The input parameters must be the following values):
 *\*\           0x1E :512 KB ITCM,  0   KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x1F :384 KB ITCM,  128 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x20 :256 KB ITCM,  256 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x21 :128 KB ITCM,  384 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x22 :0   KB ITCM,  512 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x23 :384 KB ITCM,  0   KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x24 :256 KB ITCM,  128 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x25 :128 KB ITCM,  256 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x26 :0   KB ITCM,  384 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x27 :256 KB ITCM,  0   KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x28 :128 KB ITCM,  128 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x29 :0   KB ITCM,  256 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2A :128 KB ITCM,  0   KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2B :0   KB ITCM,  128 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\           0x2C~2F :0KB ITCM,  0   KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRMA1
 *\*\return none  
 */               
void __attribute__((noinline, section(".text.reset"))) ConfigTcmSize(uint32_t tcmSizeValue)
{
    __IO uint32_t currValue = (*(uint32_t(*)(void))0x1ff00f01)();
    if((currValue == 0x2c) && (currValue != tcmSizeValue))
    {
        *(__IO uint32_t*)0x51105280 = tcmSizeValue;
        NVIC_SystemReset();
    }
}
#ifdef CORE_CM7
/**
*\*\name    PWR_ConfigSupply.
*\*\fun     Configure the PWR supply.
*\*\param   SupplySource (The input parameters must be the following values):
*\*\          - PWR_LDO_SUPPLY               :External LDO  SUPPLY 
*\*\          - PWR_DIRECT_SMPS_SUPPLY       :DCDC SUPPLY 
*\*\          - PWR_EXTERNAL_SOURCE_SUPPLY   :External VCAP SUPPLY 
*\*\return  none
**/
static void __attribute__((noinline, section(".text.reset"))) PWR_ConfigSupply(uint32_t SupplySource)
{
    __IO uint32_t tempreg;
    /* Get the old register value */
    tempreg = PWR->SYSCTRL4;
    /* Clear the old  value */
    tempreg &= (~PWR_SUPPLY_MODE_MASK);
    /* Set the new values */
    tempreg |= SupplySource;
    /* Set the power supply configuration */
    PWR->SYSCTRL4 = tempreg;
    
    /* Config NRST Filter Function */
    tempreg = PWR->SYSCTRL1;
    tempreg &= ~(PWR_RST_AGFBPEN_MAST | PWR_RST_DGF_CNT_MAST | PWR_RST_DGFBPEN_MAST);
    tempreg |= PWR_RST_DGF_CNT_DEFAULT;
    PWR->SYSCTRL1 = tempreg;
}
#endif
/**
 *\*\name   SystemInit.
 *\*\fun    Setup the microcontroller system.Initialize the FPU setting, vector table location and External memory configuration.
 *\*\param  none
 *\*\return none
 */
void __attribute__((section(".text.reset"))) SystemInit (void)
{    

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
  #endif

    /*SEVONPEND enabled so that an interrupt coming from the CPU(n) interrupt signal is
     detectable by the CPU after a WFI/WFE instruction.*/ 
 SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

#ifdef CORE_CM7 
  
#endif /* CORE_CM7*/

#ifdef CORE_CM4

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = 0x30000000; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BANK2_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif  

#else
#ifdef CORE_CM7

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = 0x24000000;       /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BANK1_BASE | VECT_TAB_OFFSET;       /* Vector Table Relocation in Internal FLASH */
#endif

   /*defaule TCM_SIZE=0x2F,All TCMSRAM are AXI_SRAM,if you want to use ITCM/DTCM, define INIT_TCM_SIZE*/
#ifdef USING_TCM
    ConfigTcmSize(TCM_SIZE_VALUE);
#endif
    /*User can change the way of power supply */
    PWR_ConfigSupply(PWR_SUPPLY_SELECTION);

#else
#error Please #define CORE_CM4 or CORE_CM7
#endif                       
#endif

}


/**
 *\*\name   X32SysTick_Handler.
 *\*\fun    This function handles X32SysTick Handler.
 *\*\param  none
 *\*\return none
 */
void X32SysTick_Handler(void)
{

}

