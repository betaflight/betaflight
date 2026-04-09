/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT_BITBANG

#include "build/atomic.h"
#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "platform/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;

    bbPort->gpioModeMask |= (0x03 << (pinIndex * 2));
    bbPort->gpioModeInput |= (DIR_IN << (pinIndex * 2));
    bbPort->gpioModeOutput |= (DIR_OUT << (pinIndex * 2));

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbPort->gpioIdleBSRR |= (1 << pinIndex);         // BS (lower half)
    } else
#endif
    {
        bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        IOWrite(bbMotor->io, 1);
    } else
#endif
    {
        IOWrite(bbMotor->io, 0);
    }
         // is this needed here?
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    TIM_Cmd(bbPort->timhw->tim, DISABLE);

    timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);

    timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        TIM_CtrlPWMOutputs(timhw->tim, ENABLE);
    }
#endif

    // Enable and keep it running

    TIM_Cmd(bbPort->timhw->tim, ENABLE);
}

#ifdef USE_DMA_REGISTER_CACHE

static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->CFGR = dmaRegCache->CFGR;
    ((DMA_ARCH_TYPE *)dmaResource)->CNTR = dmaRegCache->CNTR;
    ((DMA_ARCH_TYPE *)dmaResource)->PADDR = dmaRegCache->PADDR;
    ((DMA_ARCH_TYPE *)dmaResource)->MADDR = dmaRegCache->MADDR;
    ((DMA_ARCH_TYPE *)dmaResource)->M1ADDR = dmaRegCache->M1ADDR;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->CFGR = ((DMA_ARCH_TYPE *)dmaResource)->CFGR;
    dmaRegCache->CNTR = ((DMA_ARCH_TYPE *)dmaResource)->CNTR;
    dmaRegCache->PADDR = ((DMA_ARCH_TYPE *)dmaResource)->PADDR;
    dmaRegCache->MADDR = ((DMA_ARCH_TYPE *)dmaResource)->MADDR;
    dmaRegCache->M1ADDR = ((DMA_ARCH_TYPE *)dmaResource)->M1ADDR;
}
#endif


void bbSwitchToOutput(bbPort_t * bbPort)
{
    dbgPinHi(1);
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->BSHR, bbPort->gpioIdleBSRR);

    // Set GPIO to output
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) 
    {
#if 1       
        // if(bbPort->portIndex >= 8)
        // {
        //     bbPort->gpio->CFGHR &= ~(0xF<< ((bbPort->portIndex - 8)*4));
        //     bbPort->gpio->CFGHR |= 0x3<<((bbPort->portIndex - 8)*4);      //outpp
        // }
        // else
        // {
        //     bbPort->gpio->CFGLR &= ~(0xF<< ((bbPort->portIndex)*4));
        //     bbPort->gpio->CFGLR |= 0x3<<((bbPort->portIndex)*4);       //outpp
        // }

        uint32_t expandedMaskLow = 0, outputlow = 0;
        uint32_t expandedMaskHigh = 0, outputhigh = 0;

        for (int i = 0; i < 16; i++) 
        {
            uint8_t mask2bit = (bbPort->gpioModeMask >> (i * 2)) & 0x3;
            
            if (mask2bit != 0) 
            {
                uint32_t mapped4bit = 0xF;  
                if (i < 8)
                {
                    expandedMaskLow |= (mapped4bit << (i * 4));
                } 
                else 
                {
                    expandedMaskHigh |= (mapped4bit << ((i - 8) * 4));
                }
            }

            uint8_t out2bit = (bbPort->gpioModeOutput >> (i * 2)) & 0x3;
             uint32_t mappedout4bit = 0;  
            if (out2bit != 0) 
            {
               mappedout4bit = 0x3;
            }
            // else
            // {
            //     mappedout4bit = 0x8;
            // }

                if (i < 8)
                {
                    outputlow |= (mappedout4bit << (i * 4));
                } 
                else 
                {
                    outputhigh |= (mappedout4bit << ((i - 8) * 4));
                }

        }
        MODIFY_REG(bbPort->gpio->CFGLR, expandedMaskLow, outputlow);
        MODIFY_REG(bbPort->gpio->CFGHR, expandedMaskHigh, outputhigh);
#else
        // MODIFY_REG(bbPort->gpio->cfgr, bbPort->gpioModeMask, bbPort->gpioModeOutput);
        GPIOE->CFGLR &= ~(0x0FFFF000);
        GPIOE->CFGLR |=  (0x03333000);  //
#endif
    }

    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    // xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for output

    ((TIM_TypeDef *)bbPort->timhw->tim)->ATRLR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set GPIO to input
    WRITE_REG(bbPort->gpio->BSHR, bbPort->gpioIdleBSRR);
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {

#if 1       
        // if(bbPort->portIndex >= 8)
        // {
        //     bbPort->gpio->CFGHR &= ~(0xF<< ((bbPort->portIndex - 8)*4));
        //     bbPort->gpio->CFGHR |= 0x4<<((bbPort->portIndex - 8)*4);      //float in
        // }
        // else
        // {
        //     bbPort->gpio->CFGLR &= ~(0xF<< ((bbPort->portIndex)*4));
        //     bbPort->gpio->CFGLR |= 0x4<<((bbPort->portIndex)*4);       //float in
        // }
        // MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeInput);
    

        uint32_t expandedMaskLow = 0, outputlow = 0;
        uint32_t expandedMaskHigh = 0, outputhigh = 0;

        for (int i = 0; i < 16; i++) 
        {
            uint8_t mask2bit = (bbPort->gpioModeMask >> (i * 2)) & 0x3;
            
            if (mask2bit != 0) 
            {
                uint32_t mapped4bit = 0xF;  
                if (i < 8)
                {
                    expandedMaskLow |= (mapped4bit << (i * 4));
                } 
                else 
                {
                    expandedMaskHigh |= (mapped4bit << ((i - 8) * 4));
                }
            }

            uint8_t out2bit = (bbPort->gpioModeOutput >> (i * 2)) & 0x3;
             uint32_t mappedout4bit = 0;  
            if (out2bit != 0) 
            {
               mappedout4bit = 0x8;  //
            }
            // else
            // {
            //     mappedout4bit = 0x;
            // }

                if (i < 8)
                {
                    outputlow |= (mappedout4bit << (i * 4));
                } 
                else 
                {
                    outputhigh |= (mappedout4bit << ((i - 8) * 4));
                }

        }
        MODIFY_REG(bbPort->gpio->CFGLR, expandedMaskLow, outputlow);
        MODIFY_REG(bbPort->gpio->CFGHR, expandedMaskHigh, outputhigh);
#else
        GPIOE->CFGLR &= ~(0x0FFFF000);
        GPIOE->CFGLR |=  (0x08888000);  //
#endif
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    // xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for input

    ((TIM_TypeDef *)bbPort->timhw->tim)->CNT = 0;
    ((TIM_TypeDef *)bbPort->timhw->tim)->ATRLR = bbPort->inputARR;

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    dbgPinLo(1);
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DMA_StructInit(dmainit);

    // dmainit->DMA_Mode = DMA_Mode_Normal;
    // dmainit->DMA_Channel = bbPort->dmaChannel;
    // dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;
    // dmainit->DMA_FIFOMode = DMA_FIFOMode_Enable ;
    // dmainit->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    // dmainit->DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    // dmainit->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    dmainit->DMA_Mode = DMA_Mode_Normal;
    dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;



    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralDST;
        dmainit->DMA_BufferSize = bbPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSHR;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        dmainit->DMA_M2M = DMA_M2M_Disable;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralSRC;
        dmainit->DMA_BufferSize = bbPort->portInputCount;

        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->INDR;

        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portInputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        dmainit->DMA_M2M = DMA_M2M_Disable;
#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    TIM_TimeBaseInitTypeDef *init = &bbPort->timeBaseInit;

    init->TIM_Prescaler = 0; // Feed raw timerClock
    init->TIM_ClockDivision = TIM_CKD_DIV1;
    init->TIM_CounterMode = TIM_CounterMode_Up;
    init->TIM_Period = period;
    TIM_TimeBaseInit(bbPort->timhw->tim, init);
    TIM_ARRPreloadConfig(bbPort->timhw->tim, ENABLE);
}

void bbTIM_DMACmd(void* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    TIM_DMACmd((TIM_TypeDef *)TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TC, ENABLE);
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    xDMA_Cmd(bbPort->dmaResource, NewState);
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}

#endif // USE_DSHOT_BB
