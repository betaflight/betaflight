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

#include "platform.h"
#include <stdint.h>

// 崩溃信息结构体,用于在复位后保留调试信息
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t cfsr;      // Configurable Fault Status Register
    uint32_t hfsr;      // HardFault Status Register
    uint32_t dfsr;      // Debug Fault Status Register
    uint32_t mmfar;     // MemManage Fault Address Register
    uint32_t bfar;      // BusFault Address Register
    uint32_t afsr;      // Auxiliary Fault Status Register
} crash_info_t;

// 将崩溃信息放在特殊的内存区域,以便复位后仍可读取
__attribute__((section(".noinit"))) crash_info_t crashInfo;
__attribute__((section(".noinit"))) uint32_t crashInfoValid;

#define CRASH_INFO_MAGIC 0xDEADBEEF

/**
 * @brief  打印寄存器状态(通过串口或 SWO)
 * @note   这是一个弱符号,可以被更强的实现覆盖
 */
__attribute__((weak)) void hardFaultPrintRegisters(const crash_info_t* info)
{
    // 如果系统有串口,可以在这里输出调试信息
    // 目前仅作为占位符,实际使用时需要根据具体平台实现
    (void)info;
}

/**
 * @brief  HardFault 异常处理程序
 * @note   此函数捕获所有未处理的硬件异常,保存现场并进入无限循环
 *         方便调试器连接和分析
 */
void HardFault_Handler(void)
{
    __attribute__((unused)) volatile uint32_t dummy;
    
    // 获取异常栈帧指针
    // 当异常发生时,CPU 会自动将寄存器压入栈中
    // 栈帧布局取决于异常发生时使用的栈(MSP 或 PSP)
    uint32_t* stackFrame;
    
    // 检查 EXC_RETURN 的 bit 2 来判断使用的是 MSP 还是 PSP
    // 由于我们在 HardFault 中,通常使用 MSP
    asm volatile("mrs %0, msp" : "=r"(stackFrame));
    
    // 保存通用寄存器到 crashInfo
    crashInfo.r0  = stackFrame[0];
    crashInfo.r1  = stackFrame[1];
    crashInfo.r2  = stackFrame[2];
    crashInfo.r3  = stackFrame[3];
    crashInfo.r12 = stackFrame[4];
    crashInfo.lr  = stackFrame[5];
    crashInfo.pc  = stackFrame[6];
    crashInfo.psr = stackFrame[7];
    
    // 读取故障状态寄存器
    crashInfo.cfsr = SCB->CFSR;
    crashInfo.hfsr = SCB->HFSR;
    crashInfo.dfsr = SCB->DFSR;
    crashInfo.mmfar = SCB->MMFAR;
    crashInfo.bfar = SCB->BFAR;
    crashInfo.afsr = SCB->AFSR;
    
    // 标记崩溃信息有效
    crashInfoValid = CRASH_INFO_MAGIC;
    
    // 尝试输出调试信息(如果串口已初始化)
    hardFaultPrintRegisters(&crashInfo);
    
    // 清除 HardFault 标志(如果需要继续运行,但通常不建议)
    // SCB->HFSR |= SCB_HFSR_VECTTBL_Msk;
    
    // 进入无限循环,等待调试器连接
    // 使用 breakpoint 指令让调试器能够捕获
    #if defined(__GNUC__)
        __asm volatile("bkpt #0");
    #endif
    
    while(1) {
        // 看门狗喂狗(如果启用),防止复位丢失调试信息
        // watchdogRefresh();
        
        // 可以添加 LED 闪烁指示错误状态
        // ledFlashError();
    }
}

/**
 * @brief  MemManage 异常处理程序
 * @note   内存保护违规(如 MPU 违例)
 */
void MemManage_Handler(void)
{
    // MemManage 通常也会升级到 HardFault
    HardFault_Handler();
}

/**
 * @brief  BusFault 异常处理程序
 * @note   总线访问错误(如访问不存在的内存地址)
 */
void BusFault_Handler(void)
{
    // BusFault 通常也会升级到 HardFault
    HardFault_Handler();
}

/**
 * @brief  UsageFault 异常处理程序
 * @note   用法错误(如除以零、未对齐访问、无效指令等)
 */
void UsageFault_Handler(void)
{
    // UsageFault 通常也会升级到 HardFault
    HardFault_Handler();
}

/**
 * @brief  默认中断处理程序
 * @note   所有未实现的中断都会跳转到这里
 */
void Default_Handler(void)
{
    // 进入无限循环
    while(1) {
        #if defined(__GNUC__)
            __asm volatile("nop");
        #endif
    }
}
