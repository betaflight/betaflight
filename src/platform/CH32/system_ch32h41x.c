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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"
#include "ch32_debug.h"


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

// typedef void resetHandler_t(void);

// typedef struct isrVector_s {
//     __I uint32_t    stackEnd;
//     resetHandler_t *resetHandler;
// } isrVector_t;

static void checkForBootLoaderRequest(void)
{
    volatile uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);



    RCC_ClearFlag( );
    
    FLASH_Unlock();

    FLASH->BOOT_MODEKEYR = 0x45670123;
    FLASH->BOOT_MODEKEYR = 0xCDEF89AB;

    FLASH->STATR &= ~(1<<14);
    FLASH->STATR |= (1<<14);  //switch to bootloader area (0x1FFF0000)

    FLASH_Lock( );
    NVIC_SystemReset( );

    while (1);
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    //enable all needed periph
    RCC_HBPeriphClockCmd(    \
        RCC_HBPeriph_DMA1  |  \
        RCC_HBPeriph_DMA2  |  \
        RCC_HBPeriph_USBHS |  \
        RCC_HBPeriph_OTG_FS|  \
        RCC_HBPeriph_PIOC  |  \
        0, ENABLE             \
    );
    RCC_HB1PeriphClockCmd(    \
        RCC_HB1Periph_TIM2 |  \
        RCC_HB1Periph_TIM3 |  \
        RCC_HB1Periph_TIM4 |  \
        RCC_HB1Periph_TIM5 |  \
        RCC_HB1Periph_TIM6 |  \
        RCC_HB1Periph_TIM7 |  \
        RCC_HB1Periph_USART6| \
        RCC_HB1Periph_USART7| \
        RCC_HB1Periph_USART8| \
        RCC_HB1Periph_LPTIM1| \
        RCC_HB1Periph_LPTIM2| \
        RCC_HB1Periph_WWDG  | \
        RCC_HB1Periph_SPI2  | \
        RCC_HB1Periph_SPI3  | \
        RCC_HB1Periph_SPI4  | \
        RCC_HB1Periph_USART2| \
        RCC_HB1Periph_USART3| \
        RCC_HB1Periph_USART4| \
        RCC_HB1Periph_USART5| \
        RCC_HB1Periph_I2C1  | \
        RCC_HB1Periph_I2C2  | \
        RCC_HB1Periph_CAN3  | \
        RCC_HB1Periph_CAN1  | \
        RCC_HB1Periph_CAN2  | \
        RCC_HB1Periph_BKP   | \
        RCC_HB1Periph_PWR   | \
        RCC_HB1Periph_DAC   | \
        RCC_HB1Periph_I2C3  | \
        RCC_HB1Periph_SWPMI | \
        0, ENABLE             \
    );
    RCC_HB2PeriphClockCmd(    \
        RCC_HB2Periph_AFIO  | \
        RCC_HB2Periph_GPIOA | \
        RCC_HB2Periph_GPIOB | \
        RCC_HB2Periph_GPIOC | \
        RCC_HB2Periph_GPIOD | \
        RCC_HB2Periph_GPIOE | \
        RCC_HB2Periph_GPIOF | \
        RCC_HB2Periph_ADC1  | \
        RCC_HB2Periph_ADC2  | \
        RCC_HB2Periph_TIM1  | \
        RCC_HB2Periph_SPI1  | \
        RCC_HB2Periph_TIM8  | \
        RCC_HB2Periph_USART1| \
        RCC_HB2Periph_I2C4  | \
        RCC_HB2Periph_SDIO  | \
        RCC_HB2Periph_TIM9  | \
        RCC_HB2Periph_TIM10 | \
        RCC_HB2Periph_TIM11 | \
        RCC_HB2Periph_TIM12 | \
        0, ENABLE             \
    );
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_SFTRSTF)
        return true;
    else
        return false;
}

uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > 0xFFFFFFFF)
  {
    return (1UL);                                                   /* Reload value impossible */
  }
  SysTick1->ISR  &= ~(1<<1);                                        /* clear flag */
  SysTick1->CMP  = (uint32_t)(ticks - 1UL);                         /* set reload register */
  SysTick1->CNT  = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick1->CTLR = (1<<6)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0);
  NVIC_SetPriority (SysTick1_IRQn, 0xF0);                            /* set Priority for Systick Interrupt */
  NVIC_EnableIRQ(SysTick1_IRQn);
  return (0UL);                                                     /* Function successful */
}

// void ipcInterruptInit(void)
// {
//     IPC_InitTypeDef  IPC_InitStructure = {0};
//     IPC_InitStructure.IPC_CH = IPC_CH0;
//     IPC_InitStructure.TxCID = IPC_TxCID0;
//     IPC_InitStructure.RxCID = IPC_RxCID1;   //V5 receive
//     IPC_InitStructure.TxIER = DISABLE;
//     IPC_InitStructure.RxIER = ENABLE;
//     IPC_InitStructure.AutoEN = ENABLE;
// 	IPC_Init(&IPC_InitStructure);     
// 	IPC_CH0_Lock();
//     IPC->CLR = 0x1; //clear flag;
//     NVIC_SetPriority(IPC_CH0_IRQn, 0x10); 
// 	NVIC_EnableIRQ(IPC_CH0_IRQn);
// }


void systemInit(void)
{
    SystemAndCoreClockUpdate();

    persistentObjectInit();

    checkForBootLoaderRequest();

    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->RSTSCKR;

    RCC_HBPeriphClockCmd(    \
        RCC_HBPeriph_USBHS |  \
        RCC_HBPeriph_OTG_FS|  \
        0, DISABLE            \
    );

    
    RCC_ClearFlag();
    enableGPIOPowerUsageAndNoiseReductions();
    // Init cycle counter
    cycleCounterInit();
    // SysTick
    SysTick_Config(HCLKClock / 1000);

    SWPMI->OR |= 1;  //disable SWPMI, enable GPIO input 


    // ipcInterruptInit( );
}

// void IPC_CH0_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// void IPC_CH0_Handler(void)
// {
//     if (IPC_GetITStatus(IPC_CH0,IPC_CH_Sta_Bit0) != RESET) 
// 	{

//         __disable_irq( );
//         while(1);
// 	}
// }
