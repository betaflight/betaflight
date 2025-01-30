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

/// @todo [Project-H5] this files should be checked correctly, I cannot continue to do Cargo cult programming

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00U /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

void SystemInit(void)
{
  uint32_t reg_opsr;

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
   SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
  #endif

  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR = RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR1 = 0U;
  RCC->CFGR2 = 0U;

  /* Reset HSEON, HSECSSON, HSEBYP, HSEEXT, HSIDIV, HSIKERON, CSION, CSIKERON, HSI48 and PLLxON bits */
#if defined(RCC_CR_PLL3ON)
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIDIV | RCC_CR_HSIKERON | \
               RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
#else
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIDIV | RCC_CR_HSIKERON | \
               RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON);
#endif

  /* Reset PLLxCFGR register */
  RCC->PLL1CFGR = 0U;
  RCC->PLL2CFGR = 0U;
#if defined(RCC_CR_PLL3ON)
  RCC->PLL3CFGR = 0U;
#endif /* RCC_CR_PLL3ON */

  /* Reset PLL1DIVR register */
  RCC->PLL1DIVR = 0x01010280U;
  /* Reset PLL1FRACR register */
  RCC->PLL1FRACR = 0x00000000U;
  /* Reset PLL2DIVR register */
  RCC->PLL2DIVR = 0x01010280U;
  /* Reset PLL2FRACR register */
  RCC->PLL2FRACR = 0x00000000U;
#if defined(RCC_CR_PLL3ON)
  /* Reset PLL3DIVR register */
  RCC->PLL3DIVR = 0x01010280U;
  /* Reset PLL3FRACR register */
  RCC->PLL3FRACR = 0x00000000U;
#endif /* RCC_CR_PLL3ON */

  /* Reset HSEBYP bit */
  RCC->CR &= ~(RCC_CR_HSEBYP);

  /* Disable all interrupts */
  RCC->CIER = 0U;

  /* Configure the Vector Table location add offset address ------------------*/
  #ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM1_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
  #else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
  #endif /* VECT_TAB_SRAM */

  /* Check OPSR register to verify if there is an ongoing swap or option bytes update interrupted by a reset */
  reg_opsr = FLASH->OPSR & FLASH_OPSR_CODE_OP;
  if ((reg_opsr == FLASH_OPSR_CODE_OP) || (reg_opsr == (FLASH_OPSR_CODE_OP_2 | FLASH_OPSR_CODE_OP_1)))
  {
    /* Check FLASH Option Control Register access */
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != 0U)
    {
      /* Authorizes the Option Byte registers programming */
      FLASH->OPTKEYR = 0x08192A3BU;
      FLASH->OPTKEYR = 0x4C5D6E7FU;
    }
    /* Launch the option bytes change operation */
    FLASH->OPTCR |= FLASH_OPTCR_OPTSTART;

    /* Lock the FLASH Option Control Register access */
    FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
  }
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x1fff0000)
#define SYSMEMBOOT_LOADER       ((uint32_t *)0x1fff0000)

typedef void *(*bootJumpPtr)(void);

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void systemJumpToBootloader(void)
{
    /// @todo Implement systemJumpToBootloader
    // source from G4 does not work
}
