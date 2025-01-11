/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "board_api.h"

//--------------------------------------------------------------------+
// newlib read()/write() retarget
//--------------------------------------------------------------------+
#ifdef __ICCARM__
  #define sys_write   __write
  #define sys_read    __read
#elif defined(__MSP430__) || defined(__RX__)
  #define sys_write   write
  #define sys_read    read
#else
  #define sys_write   _write
  #define sys_read    _read
#endif

int sys_write(int fhdl, const char *buf, size_t count) TU_ATTR_USED;
int sys_read(int fhdl, char *buf, size_t count) TU_ATTR_USED;

#if defined(LOGGER_RTT)
// Logging with RTT

// If using SES IDE, use the Syscalls/SEGGER_RTT_Syscalls_SES.c instead
#if !(defined __SES_ARM) && !(defined __SES_RISCV) && !(defined __CROSSWORKS_ARM)
#include "SEGGER_RTT.h"

int sys_write(int fhdl, const char *buf, size_t count) {
  (void) fhdl;
  SEGGER_RTT_Write(0, (const char *) buf, (int) count);
  return (int) count;
}

int sys_read(int fhdl, char *buf, size_t count) {
  (void) fhdl;
  int rd = (int) SEGGER_RTT_Read(0, buf, count);
  return (rd > 0) ? rd : -1;
}

#endif

#elif defined(LOGGER_SWO)
// Logging with SWO for ARM Cortex
#include "board_mcu.h"

int sys_write (int fhdl, const char *buf, size_t count) {
  (void) fhdl;
  uint8_t const* buf8 = (uint8_t const*) buf;

  for(size_t i=0; i<count; i++) {
    ITM_SendChar(buf8[i]);
  }

  return (int) count;
}

int sys_read (int fhdl, char *buf, size_t count) {
  (void) fhdl;
  (void) buf;
  (void) count;
  return 0;
}

#else

// Default logging with on-board UART
int sys_write (int fhdl, const char *buf, size_t count) {
  (void) fhdl;
  return board_uart_write(buf, (int) count);
}

int sys_read (int fhdl, char *buf, size_t count) {
  (void) fhdl;
  int rd = board_uart_read((uint8_t*) buf, (int) count);
  return (rd > 0) ? rd : -1;
}

#endif

//int _close(int fhdl) {
//  (void) fhdl;
//  return 0;
//}

//int _fstat(int file, struct stat *st) {
//  memset(st, 0, sizeof(*st));
//  st->st_mode = S_IFCHR;
//}

// Clang use picolibc
#if defined(__clang__)
static int cl_putc(char c, FILE *f) {
  (void) f;
  return sys_write(0, &c, 1);
}

static int cl_getc(FILE* f) {
  (void) f;
  char c;
  return sys_read(0, &c, 1) > 0 ? c : -1;
}

static FILE __stdio = FDEV_SETUP_STREAM(cl_putc, cl_getc, NULL, _FDEV_SETUP_RW);
FILE *const stdin = &__stdio;
__strong_reference(stdin, stdout);
__strong_reference(stdin, stderr);
#endif

//--------------------------------------------------------------------+
// Board API
//--------------------------------------------------------------------+
int board_getchar(void) {
  char c;
  return (sys_read(0, &c, 1) > 0) ? (int) c : (-1);
}


uint32_t tusb_time_millis_api(void) {
  return board_millis();
}

//--------------------------------------------------------------------
// FreeRTOS hooks
//--------------------------------------------------------------------
#if CFG_TUSB_OS == OPT_OS_FREERTOS && !TUSB_MCU_VENDOR_ESPRESSIF
#include "FreeRTOS.h"
#include "task.h"

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  TU_ASSERT(false, );
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, char *pcTaskName) {
  (void) pxTask;
  (void) pcTaskName;

  taskDISABLE_INTERRUPTS();
  TU_ASSERT(false, );
}

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize ) {
  /* If the buffers to be provided to the Idle task are declared inside this
   * function then they must be declared static - otherwise they will be allocated on
   * the stack and so not exists after this function exits. */
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

  /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize ) {
  /* If the buffers to be provided to the Timer task are declared inside this
   * function then they must be declared static - otherwise they will be allocated on
   * the stack and so not exists after this function exits. */
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

  /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  /* Pass out the array that will be used as the Timer task's stack. */
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#if CFG_TUSB_MCU == OPT_MCU_RX63X || CFG_TUSB_MCU == OPT_MCU_RX65X
#include "iodefine.h"
void vApplicationSetupTimerInterrupt(void) {
  /* Enable CMT0 */
  unsigned short oldPRCR = SYSTEM.PRCR.WORD;
  SYSTEM.PRCR.WORD = (0xA5u<<8) | TU_BIT(1);
  MSTP(CMT0)       = 0;
  SYSTEM.PRCR.WORD = (0xA5u<<8) | oldPRCR;

  CMT0.CMCNT      = 0;
  CMT0.CMCOR      = (unsigned short)(((configPERIPHERAL_CLOCK_HZ/configTICK_RATE_HZ)-1)/128);
  CMT0.CMCR.WORD  = TU_BIT(6) | 2;
  IR(CMT0, CMI0)  = 0;
  IPR(CMT0, CMI0) = configKERNEL_INTERRUPT_PRIORITY;
  IEN(CMT0, CMI0) = 1;
  CMT.CMSTR0.BIT.STR0 = 1;
}
#endif


#endif
