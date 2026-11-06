#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the production UM32 BIDIR state machine against mock registers on a Linux host.

Usage: python3 src/test/um32_uart_bidir_test.py (requires Clang and sanitizers).
This does not emulate UART or LPTIM electrical timing.
"""
from pathlib import Path
import subprocess
import os
import shlex
import shutil
import tempfile

root=Path(__file__).resolve().parents[2]
temporary=tempfile.TemporaryDirectory(prefix='bf-um32-bidir-')
out=Path(temporary.name)
includes=['build/atomic.h','io/serial.h','drivers/io.h','drivers/nvic.h','drivers/serial.h','drivers/serial_impl.h','drivers/time.h','drivers/serial_uart.h','drivers/serial_uart_impl.h']
for name in includes:
 p=out/name;p.parent.mkdir(exist_ok=True,parents=True);p.write_text('')
# Preserve the production option parser verbatim; resolve its local headers
# against the register harness instead of the complete firmware type system.
for name in ['serial.h', 'serial_impl.h']:
 (out/name).write_text('')
(out/'serial_options.c').write_bytes((root/'src/main/drivers/serial_impl.c').read_bytes())
(out/'platform.h').write_text(r'''
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define USE_UART
#define USE_UART7
#define USE_UART8
#define UART1_BIDIR_RX_PIN 7
#define UART2_BIDIR_RX_PIN 17
#define UART8_BIDIR_RX_PIN 27
#define IO_TAG(x) (x)
#define IO_TAG_NONE 0
#define SERIAL_PORT_UART1 1
#define SERIAL_PORT_UART2 2
#define SERIAL_PORT_UART8 8
#define SERIAL_PORT_UART7 7
#define SERIAL_BIDIR 8
#define SERIAL_PARITY_EVEN 4
#define SERIAL_STOPBITS_2 2
#define SERIAL_INVERTED 1
#define SERIAL_BIDIR_PP 16
#define SERIAL_PULL_NONE 32
#define SERIAL_PULL_SMARTAUDIO 64
#define IOCFG_IN_FLOATING 0
#define IOCFG_AF_PP 1
#define IOCFG_IPU 32
#define IOCFG_IPD 64
#define IOCFG_AF_PP_UP 33
#define IOCFG_AF_PP_PD 65
#define UARTHARDWARE_MAX_PINS 2
#define UART_EX_LSR_TEMT 64
#define UART_EX_LSR_OE 2
#define UART_EX_LSR_PE 4
#define UART_EX_LSR_FE 8
#define UART_EX_LSR_RFE 128
#define UART_EX_USR_RFNE 8
#define USART_CSR_TXEMPTY 512
#define USART_CSR_RXRDY 1
#define LPTIM_CTRL_LPTEN 1
#define LPTIM_IF_OVIF 2
#define LPTIM_IE_OVIE 2
#define LPTIM_CFG_CLKSEL_1 512
#define LPTIM_CFG_DIVSEL 7168
#define LPTIM_CFG_MODE 4
#define NVIC_PRIO_SERIALUART1 80
#define NVIC_PRIORITY_BASE(x) 1
#define NVIC_PRIORITY_SUB(x) 1
#define LPTIMER1_IRQn 110
#define ATOMIC_BLOCK(prio) for(int once=1;once;once=0)
#define __HAL_PMU_LPTIM1_CLK_ENABLE() ((void)0)
#define __HAL_PMU_LPTIM1_FORCE_RESET() ((void)0)
#define __HAL_PMU_LPTIM1_RELEASE_RESET() ((void)0)
typedef unsigned ioTag_t;
typedef unsigned IO_t;
typedef unsigned ioConfig_t;
typedef unsigned portOptions_e;
typedef enum {serialPullNone, serialPullDown, serialPullUp} serialPullMode_t;
typedef int serialPortIdentifier_e;
typedef struct {volatile uint32_t LSR,USR,RBR;} UART_TypeDef;
typedef struct {volatile uint32_t CSR,RHR;} USART_TypeDef;
typedef struct {volatile uint32_t CTRL,CFG,TARGET,IF,IE;} MockTimer;
static MockTimer timer;
#define LPTIM1 (&timer)
typedef struct {int identifier;unsigned options,baudRate; volatile unsigned txBufferHead,txBufferTail;} serialPort_t;
typedef struct {serialPort_t port;void *USARTx;} uartPort_t;
typedef struct {unsigned pin;} Pin;
typedef struct {unsigned af;Pin rxPins[2];} Hardware;
typedef struct {Pin tx,rx;Hardware *hardware;} uartDevice_t;
static Hardware hw[8]; static uartDevice_t devs[8];
static unsigned pinMode[64],pinPull[64],now,clockHz=288000000;
static inline uartDevice_t *uartDeviceFromIdentifier(int id){return &devs[id-1];}
static inline IO_t IOGetByTag(ioTag_t t){return t;}
static inline void IOConfigGPIO(IO_t io,int mode){pinMode[io]=mode&1;pinPull[io]=mode&96;}
static inline void IOConfigGPIOAF(IO_t io,int mode,int af){(void)af;IOConfigGPIO(io,mode);}
static inline void IOInit(IO_t io,int owner,int index){(void)io;(void)owner;(void)index;}
static inline int serialOwnerTxRx(int id){return id;}
static inline int serialOwnerIndex(int id){return id;}
static inline void HAL_NVIC_SetPriority(int irq,int a,int b){(void)irq;(void)a;(void)b;}
static inline void HAL_NVIC_EnableIRQ(int irq){(void)irq;}
static inline unsigned HAL_RCM_GetPCLK0Freq(void){return clockHz;}
static inline unsigned micros(void){return now;}
''')
(out/'test.c').write_text(r'''
#include <assert.h>
#include <stdio.h>
#include "platform.h"
#include "@SERIAL_OPTIONS_SOURCE@"
#include "@BIDIR_SOURCE@"
static UART_TypeDef regs[8];
static uartPort_t setup(int id,unsigned baud) {
 unsigned i=id-1; hw[i]=(Hardware){.af=7,.rxPins={{id==8?27:(id==1?7:17)},{0}}};
 devs[i]=(uartDevice_t){.tx={id==8?26:(id==1?6:16)},.rx={id==1?10:hw[i].rxPins[0].pin},.hardware=&hw[i]};
 return (uartPort_t){.port={.identifier=id,.options=SERIAL_BIDIR,.baudRate=baud},.USARTx=&regs[i]};
}
int main(void) {
 uartPort_t a=setup(1,4800),b=setup(2,115200);
 uartBidirConfigure(&a); assert(pinMode[6]==0 && pinMode[7]==1 && pinMode[10]==0);
 assert(uartBidirStart(&a)); assert(pinMode[6]==1 && pinMode[7]==0);
 uartBidirDrain(&a); assert(timer.CTRL && ports[0].waiting);
 now=100; LPTIMER1_IRQHandler(); assert(pinMode[6]==1); // FIFO empty is insufficient.
 regs[0].LSR=UART_EX_LSR_OE; LPTIMER1_IRQHandler();assert(uartBidirTakeErrors(&a)==UART_EX_LSR_OE);
 regs[0].LSR=UART_EX_LSR_TEMT;LPTIMER1_IRQHandler();assert(pinMode[6]==0 && pinMode[7]==1 && !timer.IE);
 // New data arriving between FIFO-empty IRQ and timer IRQ must cancel release.
 regs[0].LSR=0;uartBidirStart(&a);uartBidirDrain(&a);a.port.txBufferHead=1;
 regs[0].LSR=UART_EX_LSR_TEMT;LPTIMER1_IRQHandler();assert(pinMode[6]==1 && !ports[0].waiting);
 uartBidirStart(&a);a.port.txBufferTail=1;uartBidirDrain(&a);assert(pinMode[6]==0);
 // New transmission cancels an already armed callback, even with an empty SW queue.
 regs[0].LSR=0;uartBidirStart(&a);uartBidirDrain(&a);uartBidirStart(&a);
 regs[0].LSR=UART_EX_LSR_TEMT;LPTIMER1_IRQHandler();assert(pinMode[6]==1);
 uartBidirDrain(&a);assert(pinMode[6]==0);
 // Two ports at different baud rates must use elapsed time, not callback count.
 uartBidirConfigure(&b);regs[0].LSR=0;uartBidirStart(&a);uartBidirDrain(&a);uartBidirStart(&b);uartBidirDrain(&b);
 for(unsigned i=0;i<20;i++){now+=10;LPTIMER1_IRQHandler();}assert(!ports[0].fault && !ports[1].fault);
 regs[1].LSR=UART_EX_LSR_TEMT;LPTIMER1_IRQHandler();assert(pinMode[16]==0 && pinMode[6]==1);
 // Stuck hardware releases the shared line and refuses another TX until reopen.
 now=ports[0].deadline;LPTIMER1_IRQHandler();assert(ports[0].fault && pinMode[6]==0 && !uartBidirStart(&a));
 uartBidirConfigure(&a);assert(uartBidirStart(&a));
 // Deadline wrap-around, and a port reopened as ordinary full duplex.
 now=0xfffffff0;regs[0].LSR=0;uartBidirDrain(&a);now=20;LPTIMER1_IRQHandler();assert(!ports[0].fault);
 a.port.options=0;uartBidirConfigure(&a);assert(!ports[0].uart);LPTIMER1_IRQHandler();assert(!timer.IE);
 // An invalid alternate RX pad must not bind a UART.
 b.port.options=SERIAL_BIDIR;hw[1].rxPins[0].pin=3;uartBidirConfigure(&b);assert(!ports[1].uart);
 // SAM-style USART uses TXEMPTY, not the UART_EX register layout.
 uartPort_t c=setup(8,57600);USART_TypeDef sam={0};c.USARTx=&sam;uartBidirConfigure(&c);uartBidirStart(&c);uartBidirDrain(&c);
 sam.CSR=USART_CSR_TXEMPTY;LPTIMER1_IRQHandler();assert(pinMode[26]==0 && pinMode[27]==1);
 // Timer rate follows the real clock at both supported CPU frequencies.
 for(unsigned i=0;i<2;i++){clockHz=i?336000000:288000000;uartBidirStart(&c);sam.CSR=0;uartBidirDrain(&c);assert(timer.TARGET>0 && timer.TARGET<65536);sam.CSR=512;LPTIMER1_IRQHandler();}
 // Tramp keeps its ordinary 9600/8N1 options, including on a pull-up board.
 a=setup(1,9600);uartBidirConfigure(&a);
 assert(pinPull[6]==32 && pinPull[7]==32);
 assert(a.port.options==SERIAL_BIDIR && a.port.baudRate==9600);
 for(unsigned i=0;i<3;i++) {
  regs[0].LSR=0;uartBidirStart(&a);
  assert(pinMode[6]==1 && pinPull[6]==32 && pinMode[7]==0 && pinPull[7]==0);
  uartBidirDrain(&a);LPTIMER1_IRQHandler();assert(pinMode[6]==1);
  regs[0].LSR=64;LPTIMER1_IRQHandler();
  assert(pinMode[6]==0 && pinMode[7]==1 && pinPull[6]==32 && pinPull[7]==32);
 }
 // SmartAudio follows the board's declared bias; other ports keep API defaults.
 a.port.options=SERIAL_BIDIR|SERIAL_PULL_SMARTAUDIO|SERIAL_STOPBITS_2;
 const unsigned saPull =
#ifdef UART1_BIDIR_PULLUP
 32;
#elif defined(USE_SMARTAUDIO_NOPULLDOWN)
 0;
#else
 64;
#endif
 a.port.baudRate=4800;uartBidirConfigure(&a);assert(pinPull[6]==saPull && pinPull[7]==saPull);
 uartBidirStart(&a);regs[0].LSR=64;uartBidirDrain(&a);assert(pinPull[6]==saPull && pinPull[7]==saPull);
 assert(pinMode[6]==0 && pinMode[7]==1);
 b=setup(2,4800);b.port.options=SERIAL_BIDIR|SERIAL_PULL_SMARTAUDIO|SERIAL_STOPBITS_2;
 uartBidirConfigure(&b);
#ifdef USE_SMARTAUDIO_NOPULLDOWN
 assert(pinPull[16]==0 && pinPull[17]==0);
#else
 assert(pinPull[16]==64 && pinPull[17]==64);
#endif
 // A board bias does not override explicit no-pull or inverted-port options.
 a.port.options=SERIAL_BIDIR|SERIAL_PULL_NONE;uartBidirConfigure(&a);
 assert(pinPull[6]==0 && pinPull[7]==0);
#ifdef UART1_BIDIR_PULLUP
 a.port.options|=SERIAL_PULL_SMARTAUDIO;uartBidirConfigure(&a);
 assert(pinPull[6]==0 && pinPull[7]==0);
#endif
 a.port.options=SERIAL_BIDIR|SERIAL_INVERTED;uartBidirConfigure(&a);
 assert(pinPull[6]==64 && pinPull[7]==64);
 a.port.options=SERIAL_PULL_SMARTAUDIO;uartBidirConfigure(&a);
 assert(!ports[0].uart); // No board override on a full-duplex port.
 puts("PASS: TEMT, repeated Tramp TX/RX, board bias, per-port isolation, no-pull, inversion and recovery");
}
'''.replace('@BIDIR_SOURCE@', (root/'src/platform/UM32/serial_uart_bidir.c').as_posix())
   .replace('@SERIAL_OPTIONS_SOURCE@', (out/'serial_options.c').as_posix()))
compiler=shlex.split(os.environ.get('CC', shutil.which('clang-18') or 'clang'))
cmd=compiler+['-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(out),'-I'+str(root/'src/platform/UM32/include'),str(out/'test.c'),'-o',str(out/'test')]
for defines in [[], ['-DUSE_SMARTAUDIO_NOPULLDOWN'], ['-DUART1_BIDIR_PULLUP'], ['-DUART1_BIDIR_PULLUP', '-DUSE_SMARTAUDIO_NOPULLDOWN']]:
 subprocess.run(cmd+defines,check=True)
 subprocess.run([str(out/'test')],check=True)
