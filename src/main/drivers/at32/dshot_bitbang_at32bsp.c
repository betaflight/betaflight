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
#include "dma_atbsp.h"
#include "dma_reqmap_mcu.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h" // XXX for pwmOutputPort_t motors[]; should go away with refactoring
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;
    /* 
     *初始化对应pin的 modemask、input模式值，output模式寄存器值，供下面switchtoinput、switchtooutput函数用
     * 好处： 直接寄存器操作对应的pin 在输入和输出模式之间快速翻转
     * 原因：dshot 是fc 和esc 之间1根线通信，需要在输入和输出之间快速切换模式
     * 移植思路：
     * f3\f4之后芯片， 输入输出mode分离为多个寄存器，但是在f1、at32f403a仍然使用CRL CRH 两个寄存器进行初始设置，并且ipd\ipu 模式，
     * 需要单独写一下BSRR\BRR
     * 因此需要考虑参考GPIO_Init 函数的写法，简化
     * GPIO_CRL_MODE0 +PullUP PULLDown 设计一个输入模式，一个输出模式，不能直接用GPIO_Mode_input\output
     * 按照针脚进行移位 ，输入模式、输出模式按照针脚模式进行设置
     * bbGpioSetup中先设置对应pin的 mask位，然后在模式切换函数中，直接WRITE_REG操作
     * 先写CRL CRH 寄存器，然后判断 是否有IPD、IPU，然后单独写BSRR\BRR
     *
     * port by EMSR(shanggl@wo.cn)
     */

    bbPort->gpioModeMask |=  (0x3 << (pinIndex * 2));// mask  GPIO_MODER_0 = b11
    bbPort->gpioModeInput |= (GPIO_MODE_INPUT << (pinIndex * 2)); //input  b00
    bbPort->gpioModeOutput |= (GPIO_MODE_OUTPUT << (pinIndex * 2));//output b01


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

    IOConfigGPIO(bbMotor->io, IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL , bbPuPdMode));
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    tmr_output_config_type  TIM_OCStruct;
	tmr_output_default_para_init(&TIM_OCStruct);
	TIM_OCStruct.oc_mode= TMR_OUTPUT_CONTROL_PWM_MODE_A;//when count up  pwm1 eq pwma pwm2 =pwmb
    TIM_OCStruct.oc_idle_state=TRUE;
    TIM_OCStruct.oc_output_state=TRUE;
	TIM_OCStruct.oc_polarity=TMR_OUTPUT_ACTIVE_LOW;
    tmr_channel_value_set(timhw->tim, (timhw->channel-1)*2, 10);

    tmr_counter_enable(bbPort->timhw->tim, FALSE);
    tmr_output_channel_config(timhw->tim,(timhw->channel-1)*2, &TIM_OCStruct);
    tmr_channel_enable(timhw->tim, ((timhw->channel-1)*2),TRUE);
    tmr_output_channel_buffer_enable(timhw->tim, ((timhw->channel-1)*2),TRUE);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        tmr_output_enable(timhw->tim,TRUE);
    }
#endif

    // Enable and keep it running
    tmr_counter_enable(bbPort->timhw->tim,TRUE);
}

#ifdef USE_DMA_REGISTER_CACHE

void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->ctrl = dmaRegCache->CCR;	//ctrl info
    ((DMA_ARCH_TYPE *)dmaResource)->dtcnt = dmaRegCache->CNDTR; // dtcnt data count
    ((DMA_ARCH_TYPE *)dmaResource)->paddr = dmaRegCache->CPAR;  //pheriph address
    ((DMA_ARCH_TYPE *)dmaResource)->maddr = dmaRegCache->CMAR;  //Memory address
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
	dmaRegCache->CCR=((DMA_ARCH_TYPE *)dmaResource)->ctrl;
	dmaRegCache->CNDTR=((DMA_ARCH_TYPE *)dmaResource)->dtcnt;
	dmaRegCache->CPAR=((DMA_ARCH_TYPE *)dmaResource)->paddr ;
	dmaRegCache->CMAR=((DMA_ARCH_TYPE *)dmaResource)->maddr ;

}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    dbgPinHi(1);
    // Output idle level before switching to output
    // stm32f4 Use BSRR register for this
    // Normal: Use BR (higher half)  at32f43x maps to scr
    // Inverted: Use BS (lower half) at32f43x maps to clr
    //BSRR bit set register
    WRITE_REG(bbPort->gpio->scr, bbPort->gpioIdleBSRR);
    // Set GPIO to output
    ATOMIC_BLOCK( NVIC_PRIO_TIMER ) {
        MODIFY_REG(bbPort->gpio->cfgr, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output
    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TCIF, TRUE);
#endif

    // Reinitialize pacer timer for output

    bbPort->timhw->tim->pr = bbPort->outputARR;
    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set GPIO to input
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->cfgr, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }
    // Reinitialize port group DMA for input
    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TCIF, TRUE);
#endif

    // Reinitialize pacer timer for input
    bbPort->timhw->tim->cval = 0;
    bbPort->timhw->tim->pr = bbPort->inputARR;

    bbDMA_CmdEnable(bbPort, TRUE);
    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;
    dbgPinLo(1);
}
#endif


void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{

	dma_init_type * dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

	dma_default_para_init(dmainit);

	dmainit->loop_mode_enable = FALSE;
	dmainit->peripheral_inc_enable =FALSE;
	dmainit->memory_inc_enable = TRUE;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->priority = DMA_PRIORITY_VERY_HIGH;
        dmainit->direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dmainit->buffer_size = bbPort->portOutputCount;
        dmainit->peripheral_base_addr = (uint32_t)&bbPort->gpio->scr; //MAP BSRRL TO SCR
        dmainit->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;// use half word ? just low part is usefull
        dmainit->memory_base_addr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->priority = DMA_PRIORITY_VERY_HIGH;
        dmainit->direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dmainit->buffer_size = bbPort->portInputCount;

        dmainit->peripheral_base_addr = (uint32_t)&bbPort->gpio->idt;
        dmainit->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dmainit->memory_base_addr = (uint32_t)bbPort->portInputBuffer;
        dmainit->memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    tmr_base_init(bbPort->timhw->tim, period,0);
    tmr_clock_source_div_set(bbPort->timhw->tim,TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(bbPort->timhw->tim,TMR_COUNT_UP);
    tmr_period_buffer_enable(bbPort->timhw->tim,TRUE);
}

void bbTIM_DMACmd(tmr_type * TIMx, uint16_t TIM_DMASource, confirm_state NewState)
{
	tmr_dma_request_enable(TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TCIF, TRUE);
}

void bbDMA_CmdEnable(bbPort_t *bbPort, confirm_state NewState)
{
    xDMA_Cmd(bbPort->dmaResource, NewState);
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}

#endif // USE_DSHOT_BB
