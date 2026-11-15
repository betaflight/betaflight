# This file is part of Betaflight.
#
# Betaflight is free software. You can redistribute this software
# and/or modify this software under the terms of the GNU General
# Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later
# version.
#
# Betaflight is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public
# License along with this software.
#
# If not, see <http://www.gnu.org/licenses/>.

from pathlib import Path
import subprocess
import sys
import tempfile
sys.dont_write_bytecode = True
root=Path(__file__).resolve().parents[3]
temporary_directory=tempfile.TemporaryDirectory(prefix='ch32h415-regression-')
out=Path(temporary_directory.name)
common='#include <stdint.h>\n#include <stdbool.h>\n#include <stdio.h>\n#include <string.h>\n#include <stdlib.h>\n'
def fn(rel,marker):
 p=root/rel;s=p.read_text();start=s.index(marker);b=s.index('{',start);n=1;i=b+1
 while n:
  if s[i]=='{':n+=1
  if s[i]=='}':n-=1
  i+=1
 return '#line %d "%s"\n'%(s[:start].count('\n')+1,p)+s[start:i]+'\n#line 1 "audit_harness"\n'
def run(name,src,timeout=2,extra_flags=()):
 p=out/(name+'.c');p.write_text(common+src)
 cmd=['gcc','-std=gnu11','-O1','-g','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-Wno-pointer-to-int-cast',*extra_flags,str(p),'-o',str(out/name)]
 c=subprocess.run(cmd,capture_output=True,text=True)
 if c.returncode:
  raise RuntimeError(f'{name}: build failed\n{c.stderr}')
 try:
  r=subprocess.run([str(out/name)],capture_output=True,text=True,timeout=timeout,check=True)
  print(name,'PASS',(r.stdout+r.stderr).strip())
 except subprocess.TimeoutExpired as e:
  raise RuntimeError(f'{name}: timed out\n{(e.stdout or b"").decode()}')
 except subprocess.CalledProcessError as e:
  raise RuntimeError(f'{name}: failed with exit code {e.returncode}\n{e.stdout}{e.stderr}')

pf='src/platform/CH32/include/platform/platform.h'
run('micros_global_irq_disabled', '''
#define CH32H4
typedef struct { uint32_t GISR; } pfic_t;
typedef struct { uint32_t CNT; } systick_t;
static pfic_t pfic;
static systick_t tick;
#define PFIC (&pfic)
#define SysTick1 (&tick)
static volatile uint32_t sysTickUptime, sysTickValStamp;
static uint32_t usTicks=350;
static bool irqEnabled;
static uint32_t __get_BASEPRI(void) { return 0; }
static bool ch32GlobalInterruptsEnabled(void) { return irqEnabled; }
static uint32_t microsISR(void) { return 1234; }
''' + fn('src/platform/CH32/system_ch32h41x.c','uint32_t micros(void)') + '''
int main(void) {
    tick.CNT=200;
    if (micros() != 1234) return 1;
    irqEnabled=true;
    sysTickValStamp=349999;
    tick.CNT=175000;
    if (micros() != 500) return 2;
    puts("global masking uses the non-spinning path; first tick returns normally");
}
''')
run('ch32_adc_temperature', '''
static uint32_t fakeCalibration;
#define CH32_TS_CAL_ADDRESS ((uintptr_t)&fakeCalibration)
''' + fn('src/platform/CH32/adc_ch32h41x.c','int16_t adcInternalComputeTemperature(') + '''
int main(void) {
    fakeCalibration=(30u << 16) | 1000u;
    if (adcInternalComputeTemperature(1000, 4096) != 30) return 1;
    if (adcInternalComputeTemperature(997, 4096) != 31) return 2;
    if (adcInternalComputeTemperature(1003, 4096) != 29) return 3;
    if (adcInternalComputeTemperature(998, 4096) != 30) return 4;
    if (adcInternalComputeTemperature(1002, 4096) != 30) return 5;
    (void)adcInternalComputeTemperature(UINT16_MAX, UINT16_MAX);
    puts("factory reference and symmetric rounding above and below calibration");
}
''')
run('exti_unregistered', '''
typedef struct extiCallbackRec_s extiCallbackRec_t;
struct extiCallbackRec_s { void (*fn)(extiCallbackRec_t *); };
static struct { extiCallbackRec_t *handler; } extiChannelRecs[16];
static uint32_t imr, pr;
#define EXTI_REG_IMR imr
#define EXTI_REG_PR pr
static int calls;
static void callback(extiCallbackRec_t *self) { (void)self; calls++; }
''' + fn('src/platform/CH32/exti_ch32.c','static void EXTI_IRQHandler(') + '''
int main(void) {
    extiCallbackRec_t valid={callback}, empty={NULL};
    extiChannelRecs[3].handler=&valid;
    extiChannelRecs[2].handler=&empty;
    imr=pr=0x0f;
    EXTI_IRQHandler(0x07);
    if (calls != 0 || pr != 7) return 1;
    pr=0x0f;
    EXTI_IRQHandler(0x0f);
    if (calls != 1 || pr != 15) return 2;
    puts("missing callbacks are skipped; registered callbacks still run");
}
''')
host_include=out/'include'
host_include.mkdir(exist_ok=True)
(host_include/'platform.h').write_text('/* The host tests supply simulated peripherals. */\n')
run('priority_saturation', '''
#include "platform/priority.h"
int main(void) {
    for (unsigned base=0; base<32; base++) {
        const unsigned expected = ((base > 3 ? 3 : base) << 6) | 0x20;
        if (NVIC_BUILD_PRIORITY(base, 0) != expected) return 1;
    }
    puts("out-of-range priority groups saturate at the lowest group");
}
''', extra_flags=('-Wall','-Wextra','-I'+str(root/'src/platform/CH32/include')))
run('dma_flag_masks', '''
typedef struct { uint32_t INTFR, INTFCR; } DMA_TypeDef;
typedef struct { DMA_TypeDef *dma; unsigned flagsShift; } descriptor_t;
#include "platform/dma.h"
int main(void) {
    DMA_TypeDef regs = {0};
    descriptor_t descriptors[2] = {{&regs, 0}, {&regs, 0}};
    for (unsigned channel = 0; channel < 8; channel++) {
        descriptors[1].flagsShift = channel * 4;
        const uint32_t expected = (DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF) << descriptors[1].flagsShift;
        DMA_CLEAR_FLAG(&descriptors[1], DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        if (regs.INTFCR != expected) return 1;
        regs.INTFR = expected;
        if (DMA_GET_FLAG_STATUS(&descriptors[1], DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF) != expected) return 2;
    }
    puts("combined flags target the selected channel");
}
''', extra_flags=('-Wall','-Wextra','-I'+str(host_include),'-I'+str(root/'src/main'),'-I'+str(root/'src/platform/CH32/include')))
run('rcc_high_bit', '''
typedef uint32_t rccPeriphTag_t;
typedef enum { DISABLE, ENABLE } FunctionalState;
static struct { uint32_t HBPRSTR, HB1PRSTR, HB2PRSTR; } registers;
#define RCC (&registers)
#define RCC_HB 1
#define RCC_HB1 2
#define RCC_HB2 3
#define NOSUFFIX
''' + fn('src/platform/CH32/rcc_ch32.c','void RCC_ResetCmd(') + '''
int main(void) {
    RCC_ResetCmd((RCC_HB1 << 5) | 31, DISABLE);
    if (registers.HB1PRSTR != 0x80000000u) return 1;
    RCC_ResetCmd((RCC_HB1 << 5) | 31, ENABLE);
    if (registers.HB1PRSTR != 0) return 2;
    puts("bit 31 and reset enable semantics");
}
''')
run('spi_divisor_rounding', '''
#define UNUSED(x) ((void)(x))
typedef struct { int unused; } SPI_TypeDef;
''' + fn('src/platform/CH32/bus_spi_ch32h41x.c','static uint16_t spiDivisorToBRbits(') + '''
int main(void) {
    for (unsigned divisor=0; divisor<=UINT16_MAX; divisor++) {
        const unsigned actual = 2u << (spiDivisorToBRbits(NULL, divisor) >> 3);
        const unsigned minimum = divisor < 2 ? 2 : divisor > 256 ? 256 : divisor;
        if (actual < minimum || actual > 256 || (actual > 2 && actual/2 >= minimum)) return 1;
    }
    puts("all 16-bit divisors select the smallest supported non-faster clock");
}
''')
run('flash_fast_wait', '''
#define ProgramTimeout 8
static struct { uint32_t STATR; } registers;
#define FLASH (&registers)
''' + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','static uint32_t FLASH_FastWait(') + '''
int main(void) {
    registers.STATR=0;
    if (!FLASH_FastWait(3)) return 1;
    registers.STATR=1;
    if (FLASH_FastWait(1)) return 2;
    if (!FLASH_FastWait(2)) return 3;
    puts("busy and write-busy polling terminate");
}
''')
run('flash_fast_timeout', '''
#define SR_BSY 1u
#define SR_WR_BSY 2u
#define SR_WRPRTERR 0x10u
#define SR_EOP 0x20u
#define CR_PG_Set 1u
#define CR_PER_Set 2u
#define CR_PAGE_PG 0x10000u
#define CR_BER 0x40000u
#define CR_PG_STRT 0x200000u
#define CR_MODE_Mask (CR_PG_Set | CR_PER_Set | CR_PAGE_PG | CR_BER)
#define ProgramTimeout 8u
typedef enum { FLASH_BUSY=1, FLASH_ERROR_WRP, FLASH_COMPLETE, FLASH_TIMEOUT } FLASH_Status;
static struct { uint32_t STATR, CTLR; } registers;
#define FLASH (&registers)
static unsigned waits, failAt, syncs;
static bool raiseWrpOnCommit;
''' + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','static uint32_t FLASH_FastIrqSave(')
    + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','static void FLASH_FastIrqRestore(') + '''
static void FLASH_RegSync(void) {
    (void)*(volatile uint32_t *)&FLASH->CTLR;
    syncs++;
    if (raiseWrpOnCommit && (registers.CTLR & CR_PG_STRT)) {
        registers.STATR = SR_WRPRTERR;
    } else if (!(registers.CTLR & CR_PG_STRT)) {
        /* Simulate hardware RC_W1 clear of sticky EOP/WRPRTERR before commit */
        registers.STATR &= ~(SR_EOP | SR_WRPRTERR);
    }
}
static uint32_t FLASH_FastWait(uint32_t mask) {
    waits++;
    if (waits == failAt) { registers.STATR=mask; return 0; }
    registers.STATR &= ~mask;
    return 1;
}
static uint32_t flash[64] __attribute__((aligned(256)));
''' + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','FLASH_Status FLASH_ProgramPage_Fast(') + '''
int main(void) {
    uint32_t input[64]; for (unsigned i=0; i<64; i++) input[i]=i+1;
    if ((uintptr_t)flash > UINT32_MAX) return 1;
    failAt=1; registers.STATR=0; registers.CTLR=0; waits=syncs=0;
    if (FLASH_ProgramPage_Fast((uint32_t)(uintptr_t)flash,input) != FLASH_TIMEOUT || registers.CTLR) return 2;
    failAt=2; registers.STATR=0; registers.CTLR=0; waits=syncs=0; memset(flash,0,sizeof(flash));
    if (FLASH_ProgramPage_Fast((uint32_t)(uintptr_t)flash,input) != FLASH_TIMEOUT || flash[0]!=1 || flash[1] || !(registers.CTLR & CR_PAGE_PG) || (registers.CTLR & CR_PG_STRT)) return 3;
    failAt=66; registers.STATR=0; registers.CTLR=0; waits=syncs=0; memset(flash,0,sizeof(flash));
    if (FLASH_ProgramPage_Fast((uint32_t)(uintptr_t)flash,input) != FLASH_TIMEOUT || !(registers.CTLR & CR_PAGE_PG)) return 4;
    failAt=0; registers.STATR=SR_WRPRTERR | SR_EOP; registers.CTLR=CR_PER_Set | CR_PG_Set; waits=syncs=0;
    if (FLASH_ProgramPage_Fast((uint32_t)(uintptr_t)flash,input) != FLASH_COMPLETE || memcmp(flash,input,sizeof(flash)) || (registers.CTLR & CR_MODE_Mask) || syncs<66) return 5;
    raiseWrpOnCommit=true; registers.STATR=0; registers.CTLR=0; waits=syncs=0;
    if (FLASH_ProgramPage_Fast((uint32_t)(uintptr_t)flash,input) != FLASH_ERROR_WRP || (registers.CTLR & CR_MODE_Mask)) return 6;
    puts("entry/latch/commit timeouts, stale flag clear, WRPRTERR detection and host IRQ guard");
}
''',extra_flags=('-fno-pie','-no-pie'))
run('flash_erase_status_recovery', '''
typedef volatile uint32_t vu32;
#define FLASH_FLAG_BSY 1u
#define FLASH_FLAG_WRPRTERR 0x10u
#define SR_BSY 1u
#define SR_WR_BSY 2u
#define SR_WRPRTERR 0x10u
#define SR_EOP 0x20u
#define CR_PG_Set 1u
#define CR_PER_Set 2u
#define CR_PER_Reset (~2u)
#define CR_STRT_Set 0x40u
#define CR_LOCK_Set 0x80u
#define CR_FLOCK_Set 0x8000u
#define CR_PAGE_PG 0x10000u
#define CR_BER 0x40000u
#define CR_MODE_Mask (CR_PG_Set | CR_PER_Set | CR_PAGE_PG | CR_BER)
#define FLASH_KEY1 0x45670123u
#define FLASH_KEY2 0xCDEF89ABu
#define EraseTimeout 8u
typedef enum { FLASH_BUSY=1, FLASH_ERROR_WRP, FLASH_COMPLETE, FLASH_TIMEOUT } FLASH_Status;
static struct { uint32_t KEYR, STATR, CTLR, ADDR, MODEKEYR; } registers;
#define FLASH (&registers)
static uint32_t flashMode;
#define FLASH_CFGR0_BASE ((uintptr_t)&flashMode)
static unsigned restored;
static bool stickBusyOnStart, raiseWrpOnStart;
static uint32_t startCtlrSeen;
static uint32_t FLASH_FastIrqSave(void) { return 0x88; }
static void FLASH_FastIrqRestore(uint32_t state) { restored=state; }
static void FLASH_RegSync(void) {
    if (!(registers.CTLR & CR_STRT_Set) && (registers.STATR == (SR_EOP | SR_WRPRTERR))) {
        /* Simulate hardware RC_W1 clear of sticky EOP/WRPRTERR before STRT */
        registers.STATR = 0;
    }
    if (registers.CTLR & CR_STRT_Set) {
        startCtlrSeen = registers.CTLR;
        registers.CTLR &= ~CR_STRT_Set;
        if (stickBusyOnStart) {
            registers.STATR = SR_BSY;
        } else if (raiseWrpOnStart) {
            registers.STATR = FLASH_FLAG_WRPRTERR;
        } else {
            registers.STATR = 0;
        }
    }
}
''' + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','FLASH_Status FLASH_GetStatus(')
    + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','FLASH_Status FLASH_WaitForLastOperation(')
    + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','FLASH_Status FLASH_ErasePage(')
    + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','void FLASH_Unlock_Fast(')
    + fn('lib/modules/CH32H41x/Peripheral/src/ch32h417_flash.c','void FLASH_Lock_Fast(') + '''
int main(void) {
    registers.STATR=SR_WR_BSY;
    if (FLASH_GetStatus() != FLASH_BUSY) return 1;
    if (FLASH_WaitForLastOperation(4u) != FLASH_TIMEOUT) return 2;
    registers.STATR=FLASH_FLAG_WRPRTERR;
    if (FLASH_GetStatus() != FLASH_ERROR_WRP) return 3;
    registers.STATR=0;
    if (FLASH_GetStatus() != FLASH_COMPLETE) return 4;

    /* Stale WRPRTERR on idle entry must not poison a subsequent valid erase. */
    flashMode=1u << 28;
    registers.STATR=FLASH_FLAG_WRPRTERR;
    registers.CTLR=CR_PAGE_PG;
    restored=startCtlrSeen=0;
    stickBusyOnStart=raiseWrpOnStart=false;
    if (FLASH_ErasePage(0x08019234u) != FLASH_COMPLETE) return 5;
    if (registers.ADDR != 0x08018000u || (startCtlrSeen & CR_PAGE_PG) || !(startCtlrSeen & CR_PER_Set)) return 6;
    if ((registers.CTLR & CR_MODE_Mask) || restored != 0x88) return 7;

    /* Fresh WRPRTERR raised by protected sector must return FLASH_ERROR_WRP and clear CR_PER. */
    raiseWrpOnStart=true;
    registers.STATR=0;
    registers.CTLR=0;
    restored=0;
    if (FLASH_ErasePage(0x08018000u) != FLASH_ERROR_WRP || (registers.CTLR & CR_MODE_Mask) || restored != 0x88) return 8;

    /* Erase timeout while busy must preserve CR_PER_Set. */
    raiseWrpOnStart=false;
    stickBusyOnStart=true;
    registers.STATR=0;
    registers.CTLR=0;
    restored=0;
    if (FLASH_ErasePage(0x08018000u) != FLASH_TIMEOUT || !(registers.CTLR & CR_PER_Set) || restored != 0x88) return 9;

    stickBusyOnStart=false;
    registers.STATR=0;
    registers.CTLR=CR_PAGE_PG;
    FLASH_Lock_Fast();
    if ((registers.CTLR & CR_MODE_Mask) || (registers.CTLR & (CR_LOCK_Set | CR_FLOCK_Set)) != (CR_LOCK_Set | CR_FLOCK_Set)) return 10;

    registers.CTLR=CR_PAGE_PG | CR_PER_Set;
    FLASH_Unlock_Fast();
    if ((registers.CTLR & CR_MODE_Mask) || registers.KEYR != FLASH_KEY2 || registers.MODEKEYR != FLASH_KEY2) return 11;
    puts("real WaitForLastOperation, stale/fresh WRPRTERR erase handling and fast-mode recovery");
}
''')
run('ch32_config_flash_bounds', '''
#define CH32H4
#define CONFIG_STREAMER_BUFFER_SIZE 256
#define STATIC_ASSERT(c, m) _Static_assert(c, m)
#define FLASH_FLAG_EOP 0x20u
#define FLASH_FLAG_WRPRTERR 0x10u
typedef uint32_t config_streamer_buffer_type_t;
typedef enum { CONFIG_RESULT_SUCCESS=0, CONFIG_RESULT_FAILURE=-1, CONFIG_RESULT_ADDRESS_INVALID=-3, CONFIG_RESULT_TIMEOUT=-4 } configStreamerResult_e;
typedef enum { FLASH_BUSY=1, FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE, FLASH_TIMEOUT } FLASH_Status;
static uint8_t flash[16384] __attribute__((aligned(8192)));
static uint8_t physicalFlash[16384];
static uintptr_t cfgStart, cfgEnd;
#define __config_start (*(uint8_t *)cfgStart)
#define __config_end (*(uint8_t *)cfgEnd)
static uint32_t flashMode;
#define FLASH_CFGR0_BASE ((uintptr_t)&flashMode)
static unsigned erases, writes, eraseStep, invalidations, locks, unlocks, clearedFlags;
static bool suppressWrite;
static FLASH_Status forceEraseStatus = FLASH_COMPLETE;
static FLASH_Status forceProgStatus = FLASH_COMPLETE;
static FLASH_Status forceFinalStatus = FLASH_COMPLETE;
static void ch32InvalidateFlashCache(void) {
    memcpy(flash, physicalFlash, sizeof(flash));
    invalidations++;
}
static void FLASH_Unlock_Fast(void) { unlocks++; }
static void FLASH_Lock_Fast(void) { locks++; }
static void FLASH_ClearFlag(uint32_t flags) { clearedFlags |= flags; }
static FLASH_Status FLASH_ErasePage(uintptr_t addr) {
    if (forceEraseStatus != FLASH_COMPLETE) return forceEraseStatus;
    if (addr != (uintptr_t)flash + erases * eraseStep) return FLASH_ERROR_PG;
    memset(physicalFlash + (addr - (uintptr_t)flash), 0xff, eraseStep);
    erases++;
    return FLASH_COMPLETE;
}
static FLASH_Status FLASH_ProgramPage_Fast(uintptr_t addr, uint32_t *buf) {
    if (forceProgStatus != FLASH_COMPLETE) return forceProgStatus;
    if (!suppressWrite) memcpy(physicalFlash + (addr - (uintptr_t)flash), buf, CONFIG_STREAMER_BUFFER_SIZE);
    writes++;
    return FLASH_COMPLETE;
}
static FLASH_Status FLASH_GetStatus(void) { return forceFinalStatus; }
''' + fn('src/platform/CH32/config_flash.c','static bool ch32ConfigAddressIsValid(')
    + fn('src/platform/CH32/config_flash.c','static configStreamerResult_e ch32FlashStatusToConfigResult(')
    + fn('src/platform/CH32/config_flash.c','void configUnlock(')
    + fn('src/platform/CH32/config_flash.c','void configLock(')
    + fn('src/platform/CH32/config_flash.c','void configClearFlags(')
    + fn('src/platform/CH32/config_flash.c','configStreamerResult_e configWriteWord(') + '''
int main(void) {
    uint32_t buffer[64];
    for (unsigned i=0; i<64; i++) buffer[i]=0xA55A0000u + i;
    cfgStart = (uintptr_t)flash;
    cfgEnd = (uintptr_t)flash + sizeof(flash);

    configUnlock();
    configClearFlags();
    if (unlocks != 1 || clearedFlags != (FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR)) return 1;

    for (int dual=0; dual<2; dual++) {
        erases=writes=invalidations=0;
        memset(flash, 0x3C, sizeof(flash));
        memset(physicalFlash, 0x3C, sizeof(physicalFlash));
        flashMode=dual ? 1u<<28 : 0;
        eraseStep=dual ? 8192 : 4096;
        if (configWriteWord((uintptr_t)flash-256, buffer) != CONFIG_RESULT_ADDRESS_INVALID) return 2;
        if (configWriteWord((uintptr_t)flash+sizeof(flash), buffer) != CONFIG_RESULT_ADDRESS_INVALID) return 3;
        if (configWriteWord((uintptr_t)flash+1, buffer) != CONFIG_RESULT_ADDRESS_INVALID) return 4;
        if (configWriteWord(UINTPTR_MAX-127, buffer) != CONFIG_RESULT_ADDRESS_INVALID) return 5;
        for (unsigned offset=0; offset<sizeof(flash); offset+=256)
            if (configWriteWord((uintptr_t)flash+offset, buffer) != CONFIG_RESULT_SUCCESS) return 6;
        if (erases != sizeof(flash)/eraseStep || writes != sizeof(flash)/256) return 7;
        if (invalidations != erases + writes) return 8;
    }

    /* Misaligned linker __config_start / __config_end rejected */
    cfgStart = (uintptr_t)flash + 256;
    if (configWriteWord((uintptr_t)flash + 256, buffer) != CONFIG_RESULT_ADDRESS_INVALID) return 9;
    cfgStart = (uintptr_t)flash;

    memset(flash, 0, sizeof(flash));
    memcpy(physicalFlash, buffer, 256);
    locks=invalidations=0;
    configLock();
    if (locks != 1 || invalidations != 1 || memcmp(flash, buffer, 256) != 0) return 10;

    memset(flash, 0, 256);
    memset(physicalFlash, 0, 256);
    suppressWrite=true;
    erases=0;
    if (configWriteWord((uintptr_t)flash, buffer) != CONFIG_RESULT_FAILURE) return 11;

    suppressWrite=false;
    forceProgStatus=FLASH_TIMEOUT;
    erases=invalidations=0;
    if (configWriteWord((uintptr_t)flash, buffer) != CONFIG_RESULT_TIMEOUT || invalidations < 2) return 12;

    forceProgStatus=FLASH_ERROR_WRP;
    erases=invalidations=0;
    if (configWriteWord((uintptr_t)flash, buffer) != CONFIG_RESULT_ADDRESS_INVALID || invalidations < 2) return 13;

    forceProgStatus=FLASH_COMPLETE;
    forceEraseStatus=FLASH_ERROR_WRP;
    erases=invalidations=0;
    if (configWriteWord((uintptr_t)flash, buffer) != CONFIG_RESULT_ADDRESS_INVALID || invalidations != 1) return 14;

    forceEraseStatus=FLASH_COMPLETE;
    forceFinalStatus=FLASH_ERROR_WRP;
    erases=writes=invalidations=0;
    if (configWriteWord((uintptr_t)flash, buffer) != CONFIG_RESULT_ADDRESS_INVALID || invalidations < 2) return 15;
    puts("CH32 config_flash bounds, cache invalidation, WRP/timeout status mapping and lock/unlock");
}
''')
run('ch32_reset_flash_drain', '''
#include <setjmp.h>
#define CH32_FLASH_RESET_WAIT_LIMIT 8U
#define FLASH_STATR_BSY 1U
#define FLASH_STATR_WRBSY 2U
#define PERSISTENT_OBJECT_RESET_REASON 0
#define RESET_NONE 0U
#define UNUSED(x) ((void)(x))
typedef int bootloaderRequestType_e;
static unsigned polls, irqDisabled, resets, nopSpins, persistWrites;
static uint32_t persistReason = 0xDEADBEEFU;
static bool letResetFallThrough;
static jmp_buf resetJmp;
static uint32_t busySequence[16];
typedef struct { uint32_t STATR; } flash_t;
static flash_t *flashRead(void) {
    static flash_t f;
    f.STATR = polls < 16 ? busySequence[polls] : FLASH_STATR_BSY;
    polls++;
    return &f;
}
#define FLASH (flashRead())
static void __disable_irq(void) { irqDisabled++; }
static void persistentObjectWrite(int id, uint32_t val) {
    if (id != PERSISTENT_OBJECT_RESET_REASON) abort();
    persistReason = val;
    persistWrites++;
}
static void NVIC_SystemReset(void) {
    resets++;
    if (!letResetFallThrough) {
        longjmp(resetJmp, 1);
    }
}
static void __NOP(void) {
    if (++nopSpins >= 3) {
        longjmp(resetJmp, 2);
    }
}
''' + fn('src/platform/CH32/system_ch32h41x.c','static void ch32WaitFlashIdleBeforeReset(')
    + fn('src/platform/CH32/system_ch32h41x.c','void systemReset(')
    + fn('src/platform/CH32/system_ch32h41x.c','void systemResetToBootloader(')
    + fn('src/main/drivers/usb_msc_common.c','void systemResetFromMsc(') + '''
int main(void) {
    memset(busySequence, 0, sizeof(busySequence));
    busySequence[0]=FLASH_STATR_BSY;
    busySequence[1]=FLASH_STATR_WRBSY;
    busySequence[2]=0;
    polls=irqDisabled=resets=nopSpins=0;
    letResetFallThrough=true;
    if (setjmp(resetJmp) == 0) {
        systemReset();
        return 1; /* systemReset must never return to caller */
    }
    if (irqDisabled != 1 || polls != 3 || resets != 1 || nopSpins != 3) return 2;

    letResetFallThrough=false;
    for (unsigned i=0; i<16; i++) busySequence[i]=FLASH_STATR_BSY;
    polls=irqDisabled=resets=persistWrites=0;
    persistReason=99U;
    if (setjmp(resetJmp) == 0) {
        systemResetToBootloader(0);
        return 3;
    }
    if (persistWrites != 1 || persistReason != RESET_NONE || irqDisabled != 1 || polls != CH32_FLASH_RESET_WAIT_LIMIT || resets != 1) return 4;

    memset(busySequence, 0, sizeof(busySequence));
    busySequence[0]=FLASH_STATR_WRBSY;
    polls=irqDisabled=resets=persistWrites=0;
    persistReason=77U;
    if (setjmp(resetJmp) == 0) {
        systemResetFromMsc();
        return 5;
    }
    if (persistWrites != 1 || persistReason != RESET_NONE || irqDisabled != 1 || polls != 2 || resets != 1) return 6;
    puts("real systemReset/Bootloader/MSC paths drain busy flash and spin without returning");
}
''')
run('pfic', '#define PLATFORM_CUSTOM_ATOMIC_BLOCK_ALL\n#include "platform/priority.h"\nstatic volatile uint32_t threshold;\n#define PFIC_ITHRESDR threshold\n'
    +fn(pf,'__attribute__((always_inline)) static inline uint32_t __get_BASEPRI')
    +fn(pf,'__attribute__((always_inline)) static inline void __set_BASEPRI_nb')
    +fn(pf,'__attribute__((always_inline)) static inline void __set_BASEPRI_MAX_nb')
    +(root/'src/test/ch32h415/pfic_test.c').read_text(),
    extra_flags=('-Wall','-Wextra','-I'+str(host_include),'-I'+str(root/'src/main'),'-I'+str(root/'src/platform/CH32/include')))

platform_source=(root/pf).read_text()
macro_start=platform_source.index('#define ATOMIC_BLOCK_ALL()')
macro_end=platform_source.index('\n', platform_source.index('\n', macro_start)+1)
atomic_macro=platform_source[macro_start:macro_end]
run('global_irq_scope', '''
static uint32_t csr;
static uint32_t ch32IrqSave(void) { uint32_t state = csr & 0x88u; csr &= ~0x88u; return state; }
static void ch32IrqRestore(uint32_t *state) { csr |= *state; }
''' + atomic_macro + '''
static int earlyReturn(void) { ATOMIC_BLOCK_ALL() { return csr == 0; } return 0; }
int main(void) {
    for (unsigned i = 0; i < 4; i++) {
        csr = ((i & 1) ? 0x08 : 0) | ((i & 2) ? 0x80 : 0);
        const uint32_t initial = csr;
        ATOMIC_BLOCK_ALL() {
            if (csr != 0) return 1;
            ATOMIC_BLOCK_ALL() { if (csr != 0) return 2; }
            if (csr != 0) return 3;
        }
        if (csr != initial || !earlyReturn() || csr != initial) return 4;
        ATOMIC_BLOCK_ALL() { break; }
        if (csr != initial) return 5;
    }
    puts("nested, return and break preserve each enable bit");
}
''')

bb='src/platform/CH32/dshot_bitbang.c'
run('dshot_dma_recovery', '''
#define USE_DSHOT_TELEMETRY
#define DISABLE 0
#define ENABLE 1
#define NVIC_PRIO_DSHOT_DMA 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define DSHOT_BITBANG_DIRECTION_INPUT 1
#define DSHOT_BITBANG_DIRECTION_OUTPUT 0
#define DSHOT_BITBANG_STATUS_OK 0
#define DSHOT_BITBANG_STATUS_DMA_ERROR 4
#define DEBUG_DSHOT_TELEMETRY_COUNTS 0
#define DEBUG_SET(a,b,c) (debug[b] = (c))
typedef int32_t timeDelta_t;
typedef uint32_t timeUs_t;
typedef struct { void *tim; uint16_t dmaSources, armedOutputSources, pendingOutputSources, pendingInputSources, outputARR, inputARR; } bbPacer_t;
typedef struct { bbPacer_t *pacer; uint16_t dmaSource, outputARR, inputARR; bool telemetryPending, telemetryAborted, reinitRequired, inputActive; int direction; } bbPort_t;
static bbPort_t bbPorts[2];
static bbPacer_t bbPacers[1];
static int usedMotorPorts=2, usedMotorPacers, dshotMotorCount, bbStatus, debug[4];
static bool useDshotTelemetry=true, stopSucceedsPort0;
static int switches, enables;
static timeUs_t now, lastSendUs;
static timeDelta_t bbTelemetryTimeoutUs=10;
static timeUs_t micros(void) { return now++; }
static timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a-b); }
static bool dshotCommandQueueEmpty(void) { return true; }
static bool dshotCommandOutputIsEnabled(int n) { (void)n; return true; }
static bool bbSwitchToOutput(bbPort_t *p) {
    switches++;
    if (p == &bbPorts[0] && !stopSucceedsPort0) return false;
    p->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
    return true;
}
static void bbDMA_Cmd(bbPort_t *p, int enabled) { (void)p; enables += enabled; }
static void bbTIM_ReloadPeriod(void *tim, uint16_t period) { (void)tim; (void)period; }
static void bbTIM_DMACmd(void *tim, uint16_t sources, int enabled) { (void)tim; (void)sources; (void)enabled; }
''' + fn(bb,'static bool bbTelemetryWait(void)') + fn(bb,'static void bbUpdateComplete(void)') + '''
int main(void) {
    bbPorts[0] = (bbPort_t){ .telemetryPending=true, .direction=DSHOT_BITBANG_DIRECTION_INPUT, .reinitRequired=true };
    if (!bbTelemetryWait() || !bbPorts[0].telemetryAborted || bbPorts[1].telemetryAborted || debug[2] != 1) return 1;
    bbUpdateComplete();
    if (!bbPorts[0].reinitRequired || !bbPorts[0].telemetryPending || enables != 1) return 2;
    stopSucceedsPort0=true;
    switches=enables=0;
    bbUpdateComplete();
    if (bbPorts[0].reinitRequired || bbPorts[0].telemetryPending || switches != 2 || enables != 2) return 3;
    puts("timeout isolates partial capture; stopped DMA is required before recovery");
}
''')
run('dshot_dma_stop', '''
#define DISABLE 0
#define ENABLE 1
#define DSHOT_BITBANG_DIRECTION_OUTPUT 0
#define DMA_IT_TC 1
#define DMA_IT_TE 2
#define DMA_IT_GLOB 1
#define BB_DMA_FLAGS 0x0Eu
#define NVIC_PRIO_TIMER 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define WRITE_REG(reg, value) ((reg)=(value))
#define MODIFY_REG(reg, mask, value) ((reg)=((reg)&~(mask))|(value))
#define dbgPinHi(x) ((void)0)
#define dbgPinLo(x) ((void)0)
typedef int32_t IRQn_Type;
typedef struct { uint32_t CFGR; } dmaResource_t;
typedef struct { int32_t irqN; uint32_t clearedFlags; } dmaChannelDescriptor_t;
typedef struct { uint32_t BSHR, CFGLR, CFGHR; } gpio_t;
typedef struct { uint32_t ATRLR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *tim; } mockTimer_t;
typedef struct { dmaResource_t *dmaResource; dmaChannelDescriptor_t *dmaDescriptor; gpio_t *gpio; mockTimer_t *timhw; uint32_t gpioIdleBSRR, gpioModeMask, gpioModeOutput, outputARR, gpioCfgLrMask, gpioCfgHrMask, gpioCfgLrOutput, gpioCfgHrOutput; int outputDmaInit, direction; bool telemetryPending; } bbPort_t;
static bool stuck;
static int dmaWrites, pendingCleared;
static dmaChannelDescriptor_t mockDesc = { .irqN = 42 };
#define DMA_CLEAR_FLAG(d, f) ((d)->clearedFlags |= (f))
static void NVIC_ClearPendingIRQ(int32_t irq) { if (irq == 42) pendingCleared++; }
static void bbDMA_Cmd(bbPort_t *p, int state) { if (!stuck) p->dmaResource->CFGR=state; }
#define IS_DMA_ENABLED(d) ((d)->CFGR & 1u)
#define BB_DMA_STOP_SPIN_LIMIT 100
static void xDMA_Init(dmaResource_t *d, int *init) { (void)d; (void)init; dmaWrites++; }
static void xDMA_ITConfig(dmaResource_t *d, int flags, int state) { (void)d; (void)flags; (void)state; }
''' + fn('src/platform/common/stm32/dshot_bitbang_impl.h','static inline bool bbDMAWaitStopped(')
    + fn('src/platform/CH32/dshot_bitbang_stdperiph.c','bool bbSwitchToOutput(') + '''
int main(void) {
    dmaResource_t dma={.CFGR=1}; gpio_t gpio={.BSHR=17}; TIM_TypeDef tim={.ATRLR=42}; mockTimer_t hw={&tim};
    bbPort_t port={.dmaResource=&dma,.dmaDescriptor=&mockDesc,.gpio=&gpio,.timhw=&hw,.gpioIdleBSRR=7,.outputARR=99,.direction=1,.telemetryPending=true};
    stuck=true;
    if (bbSwitchToOutput(&port) || gpio.BSHR != 17 || tim.ATRLR != 42 || dmaWrites || port.direction != 1 || mockDesc.clearedFlags || pendingCleared || !port.telemetryPending) return 1;
    stuck=false;
    if (!bbSwitchToOutput(&port) || gpio.BSHR != 7 || tim.ATRLR != 99 || dmaWrites != 1 || port.direction != 0 || mockDesc.clearedFlags != (DMA_IT_GLOB | BB_DMA_FLAGS) || pendingCleared != 1 || port.telemetryPending) return 2;
    puts("busy channel leaves GPIO, timer and DMA configuration untouched and stopped channel clears stale flags");
}
''')
pre='''
#define USE_DMA_SPEC
#define MAX_PERIPHERAL_DMA_OPTIONS 16
#define MAX_SUPPORTED_MOTOR_PORTS 4
#define ARRAYLEN(x) (sizeof(x)/sizeof((x)[0]))
#define CC_CHANNELS_PER_TIMER 4
#define CC_CHANNEL_FROM_INDEX(x) ((x)*4)
#define DSHOT_BITBANGED_TIMER_TIM1 1
#define DSHOT_BITBANGED_TIMER_TIM8 8
#define OWNER_FREE 0
#define OWNER_DSHOT_BITBANG 1
#define TIM1 1
#define TIM8 8
#define NONE 0
typedef int resourceOwner_e;
typedef int dmaoptValue_t;
typedef int dmaIdentifier_e;
typedef int dmaResource_t;
typedef struct {int tim, channel, opt;} timerHardware_t;
#define DEF_TIM(t,ch,tag,usage,dma,output) {t,ch,dma}
#define CH1 0
#define CH2 4
#define CH3 8
#define CH4 12
static const timerHardware_t bbTimerHardware[] = {
 DEF_TIM(TIM8,CH1,NONE,0,8,0),DEF_TIM(TIM1,CH1,NONE,0,9,0),DEF_TIM(TIM8,CH2,NONE,0,10,0),DEF_TIM(TIM1,CH2,NONE,0,11,0),
 DEF_TIM(TIM8,CH3,NONE,0,12,0),DEF_TIM(TIM1,CH3,NONE,0,14,0),DEF_TIM(TIM8,CH4,NONE,0,15,0),DEF_TIM(TIM1,CH4,NONE,0,0,0)};
struct { const timerHardware_t *timhw; dmaResource_t *dmaResource; uint32_t dmaChannel; } bbPorts[4];
typedef struct {int owner;} resourceOwner_t;
static resourceOwner_t freeOwner={0},busyOwner={2};
static struct {struct {int useDshotBitbangedTimer;}dev;} config;
#define motorConfig() (&config)
static int lookupTimerIndex(int tim){return tim;}
static int timerGetNumberByIndex(int idx){return idx;}
static const timerHardware_t *timerGetAllocatedByNumberAndChannel(int t,int c){return NULL;}
static const resourceOwner_t *timerGetOwner(const timerHardware_t *t){return &freeOwner;}
static dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *t){return t->opt;}
typedef struct {dmaResource_t *ref; uint32_t dmaMuxId;} dmaChannelSpec_t;
static int channel;
static const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(int t,int c,int o){static const dmaChannelSpec_t s={&channel,1}; return &s;}
static dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *d){return 1;}
static const resourceOwner_t *dmaGetOwner(int id){return &busyOwner;}
'''
run('pacer_exhaustion',pre+fn('src/platform/CH32/timer_ch32bsp.c','int8_t timerGetTIMNumber(')+fn(bb,'static void bbFindPacerTimer(void)')+'int main(void){bbFindPacerTimer();puts("returned without dereferencing null");}\n')

pre='''
#define MAX_SUPPORTED_MOTORS 4
#define TIMER_OUTPUT_INVERTED 1
#define BB_GPIO_PULLDOWN 1
#define BB_GPIO_PULLUP 2
#define DSHOT_BITBANG_STATUS_OK 0
#define DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT 1
#define DIR_OUT 3
#define GPIO_MODE_OUT_AF_PP 2
#define GPIO_SPEED_VERY_HIGH 3
#define IO_CONFIG(a,b,c,d) 0
#define OWNER_MOTOR 1
#define RESOURCE_INDEX(x) ((x)+1)
#define dbgPinLo(x) ((void)0)
typedef int motorProtocolTypes_e;
typedef void *IO_t;
typedef struct {int output;} timerHardware_t;
typedef struct {unsigned motorOutputReordering[4];unsigned ioTags[4]; int motorInversion,motorProtocol;} motorDevConfig_t;
typedef struct {unsigned count;const int *vTable;} motorDevice_t;
static int bbVTable,motorProtocol,dshotMotorCount,bbStatus,bbPuPdMode,bbOutputBuffer[1];
static struct {int pinIndex,output,iocfg;IO_t io;} bbMotors[4];
static const timerHardware_t *timerGetConfiguredByTag(unsigned tag){return NULL;}
static IO_t IOGetByTag(unsigned tag){return NULL;}
static bool IOIsFreeOrPreinit(IO_t io){return true;}
static int IO_GPIOPinIdx(IO_t io){return -1;}
static void IOInit(IO_t a,int b,int c){}
static void IOConfigGPIO(IO_t a,int b){}
static void IOLo(IO_t a){}
static void IOHi(IO_t a){}
'''
run('motor_unconfigured',pre+fn(bb,'bool dshotBitbangDevInit(')+'''int main(void){motorDevice_t d={.count=4}; motorDevConfig_t c={0}; return dshotBitbangDevInit(&d,&c);}''')

run('dshot_bb_gpio_precompute', '''
#define USE_DSHOT_TELEMETRY
#define DIR_IN 0u
#define DIR_OUT 1u
#define GPIO_PULL_NONE 0u
#define GPIO_PULL_DOWN 1u
#define GPIO_PULL_UP 2u
#define IO_CONFIG_GET_PULL(cfg) (((cfg) >> 6) & 3u)
#define DSHOT_BITBANG_DIRECTION_OUTPUT 1
#define DSHOT_BITBANG_DIRECTION_INPUT 2
#define NVIC_PRIO_TIMER 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define WRITE_REG(r,v) ((r)=(v))
#define MODIFY_REG(r,m,v) ((r)=((r)&~(m))|(v))
#define dbgPinHi(x) ((void)0)
#define dbgPinLo(x) ((void)0)
#define DISABLE 0
#define ENABLE 1
#define DMA_IT_TC 1
#define DMA_IT_TE 2
#define DMA_IT_GLOB 1
#define BB_DMA_FLAGS 0x0Eu
typedef struct { uint32_t CFGLR, CFGHR, BSHR; } GPIO_TypeDef;
typedef struct { uint32_t ATRLR, CNT; } TIM_TypeDef;
typedef struct { int unused; } dmaResource_t;
typedef struct { int32_t irqN; } dmaChannelDescriptor_t;
typedef struct { void *tim; } timerHardware_t;
typedef struct { GPIO_TypeDef *gpio; dmaResource_t *dmaResource; dmaChannelDescriptor_t *dmaDescriptor; timerHardware_t *timhw;
    uint32_t gpioModeMask,gpioModeInput,gpioModeOutput,gpioIdleBSRR;
    uint32_t gpioCfgLrMask,gpioCfgHrMask,gpioCfgLrOutput,gpioCfgHrOutput,gpioCfgLrInput,gpioCfgHrInput,gpioInputBSRR;
    uint32_t outputDmaInit,inputDmaInit,outputARR,inputARR,direction;
} bbPort_t;
typedef struct { bbPort_t *bbPort; int pinIndex; unsigned iocfg; void *io; } bbMotor_t;
static bool useDshotTelemetry=true;
#define DMA_CLEAR_FLAG(d, f) ((void)(d), (void)(f))
static void IOWrite(void *io,int value) { (void)io;(void)value; }
static void bbDMA_Cmd(bbPort_t *p,int state) { (void)p;(void)state; }
static bool bbDMAWaitStopped(bbPort_t *p) { (void)p;return true; }
static void xDMA_Init(dmaResource_t *p,void *init) { (void)p;(void)init; }
static void xDMA_ITConfig(dmaResource_t *p,int flags,int state) { (void)p;(void)flags;(void)state; }
''' + fn('src/platform/CH32/dshot_bitbang_stdperiph.c','void bbGpioSetup(')
    + fn('src/platform/CH32/dshot_bitbang_stdperiph.c','bool bbSwitchToOutput(')
    + fn('src/platform/CH32/dshot_bitbang_stdperiph.c','void bbSwitchToInput(') + '''
int main(void) {
    GPIO_TypeDef gpio={.CFGLR=0x55555555,.CFGHR=0x55555555};
    TIM_TypeDef timer={0};timerHardware_t hw={&timer};dmaResource_t dma={0};
    bbPort_t p={.gpio=&gpio,.timhw=&hw,.dmaResource=&dma};
    const int pins[]={3,4,5,6,8,15};
    const unsigned pulls[]={GPIO_PULL_UP,GPIO_PULL_DOWN,GPIO_PULL_UP,GPIO_PULL_DOWN,GPIO_PULL_NONE,GPIO_PULL_DOWN};
    for (unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);i++) {
        bbMotor_t m={.bbPort=&p,.pinIndex=pins[i],.iocfg=pulls[i]<<6};bbGpioSetup(&m);
    }
    if (p.gpioCfgLrMask!=0x0ffff000u || p.gpioCfgHrMask!=0xf000000fu) return 1;
    if (p.gpioCfgLrOutput!=0x03333000u || p.gpioCfgHrOutput!=0x30000003u) return 2;
    if (p.gpioCfgLrInput!=0x08888000u || p.gpioCfgHrInput!=0x80000004u) return 3;
    if (p.gpioInputBSRR!=((1u<<3)|(1u<<5)|(1u<<20)|(1u<<22)|(1u<<31))) return 4;
    if (!bbSwitchToOutput(&p) || gpio.CFGLR!=0x53333555u || gpio.CFGHR!=0x35555553u || gpio.BSHR!=p.gpioIdleBSRR) return 5;
    bbSwitchToInput(&p);
    if (gpio.CFGLR!=0x58888555u || gpio.CFGHR!=0x85555554u || gpio.BSHR!=p.gpioInputBSRR) return 6;
    puts("mixed low/high pins, pull directions, and untouched GPIO bits are preserved");
}
''')
run('dshot_bb_port_reuse', '''
#define MAX_SUPPORTED_MOTOR_PORTS 1
#define DSHOT_BITBANG_STATUS_TOO_MANY_PORTS 1
#define DSHOT_BITBANG_STATUS_NO_PACER 2
#define OWNER_DSHOT_BITBANG 3
#define RESOURCE_INDEX(i) ((i)+1)
typedef struct { const void *timhw; int portIndex; struct { int owner,index; } resourceOwner;
    uint32_t gpioModeMask,gpioModeInput,gpioModeOutput,gpioIdleBSRR,gpioInputBSRR;
    uint32_t gpioCfgLrMask,gpioCfgHrMask,gpioCfgLrOutput,gpioCfgHrOutput,gpioCfgLrInput,gpioCfgHrInput;
} bbPort_t;
static bbPort_t bbPorts[1];static int usedMotorPorts,bbStatus;
''' + fn('src/platform/CH32/dshot_bitbang.c','static bbPort_t *bbAllocateMotorPort(') + '''
int main(void) {
    int timer;bbPort_t *p=&bbPorts[0];p->timhw=&timer;
    p->gpioModeMask=p->gpioModeInput=p->gpioModeOutput=p->gpioIdleBSRR=p->gpioInputBSRR=~0u;
    p->gpioCfgLrMask=p->gpioCfgHrMask=p->gpioCfgLrOutput=p->gpioCfgHrOutput=p->gpioCfgLrInput=p->gpioCfgHrInput=~0u;
    if (bbAllocateMotorPort(2)!=p || p->timhw!=&timer || p->portIndex!=2) return 1;
    if (p->gpioModeMask|p->gpioModeInput|p->gpioModeOutput|p->gpioIdleBSRR|p->gpioInputBSRR|
        p->gpioCfgLrMask|p->gpioCfgHrMask|p->gpioCfgLrOutput|p->gpioCfgHrOutput|p->gpioCfgLrInput|p->gpioCfgHrInput) return 2;
    puts("reused port clears cached masks without losing its allocated timer");
}
''')

uart='src/platform/common/stm32/serial_uart_hw.c'
pre='''
#define USE_DMA_SPEC
#define DMA_TRAIT_MUX 1
#define DMA_OPT_UNUSED -1
#define UARTDEV_INVALID -1
#define UART_REG_TXD(base) ((base)->data)
#define UART_REG_RXD(base) ((base)->data)
typedef int serialPortIdentifier_e;
typedef int uartDeviceIdx_e;
typedef int resourceOwner_e;
typedef int dmaIdentifier_e;
typedef struct {uint32_t data;} usart_t;
typedef struct {int identifier,txPriority;usart_t *reg;} uartHardware_t;
typedef struct {void *txDMAResource,*rxDMAResource;int txDMAMuxId,rxDMAMuxId;uint32_t txDMAPeripheralBaseAddr,rxDMAPeripheralBaseAddr;} uartPort_t;
typedef struct {uartPort_t port; const uartHardware_t *hardware;} uartDevice_t;
typedef struct {int txDmaopt,rxDmaopt;} serialUartConfig_t;
typedef struct {void *ref;int dmaMuxId;} dmaChannelSpec_t;
#define DMA_PERIPH_UART_TX 1
#define DMA_PERIPH_UART_RX 2
static int used_channel;
static const dmaChannelSpec_t spec={&used_channel,42};
static const serialUartConfig_t cfg={8,8};
static int uartDeviceIdxFromIdentifier(int x){return 0;}
static int serialResourceIndex(int x){return 0;}
static int serialOwnerIndex(int x){return 1;}
static int serialOwnerTxRx(int x){return 2;}
static const serialUartConfig_t *serialUartConfig(int x){return &cfg;}
static const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(int a,int b,int c){return &spec;}
static dmaIdentifier_e dmaGetIdentifier(void *r){return 9;}
static bool dmaAllocate(int a,int b,int c){return false;}
static void dmaEnable(int a){}
static void dmaMuxEnable(int a,int b){}
static void uartDmaIrqHandler(void){}
static void dmaSetHandler(int a,void(*f)(void),int c,uint32_t d){}
'''
run('uart_dma_refusal',pre+fn(uart,'void uartConfigureDma(')+'''
int main(void){usart_t reg={0}; uartHardware_t h={1,0,&reg};uartDevice_t d={.hardware=&h};uartConfigureDma(&d);
 printf("allocation denied: txDMAResource=%p rxDMAResource=%p peripheral addresses=%x/%x\\n",d.port.txDMAResource,d.port.rxDMAResource,d.port.txDMAPeripheralBaseAddr,d.port.rxDMAPeripheralBaseAddr);
 return !!d.port.txDMAResource||!!d.port.rxDMAResource;}
''')

usb='src/platform/CH32/serial_usb_vcp_ch32h4.c'
run('usb_cdc_callbacks', '''
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define USB_LOG_DBG(...) ((void)0)
#define USB_LOG_WRN(...) ((void)0)
#define LO_BYTE(x) ((x)&0xff)
#define CDC_REQUEST_SET_LINE_CODING 1
#define CDC_REQUEST_SET_CONTROL_LINE_STATE 2
#define CDC_REQUEST_GET_LINE_CODING 3
#define CDC_REQUEST_SEND_BREAK 4
static const char *stop_name[] = { "1", "1.5", "2" };
static const char *parity_name[] = { "N", "O", "E", "M", "S" };
struct usb_setup_packet { uint8_t bRequest; uint16_t wValue, wIndex, wLength; };
struct cdc_line_coding { uint32_t dwDTERate; uint8_t bCharFormat, bParityType, bDataBits; } __attribute__((packed));
static struct cdc_line_coding cdc_vcp_line_coding;
static void (*baudRateCb)(void *, uint32_t);
static void *baudRateCbContext;
static void (*ctrlLineStateCb)(void *, uint16_t);
static void *ctrlLineStateCbContext;
static bool usbHostPortOpen, usbTxStalled;
#define CTRL_LINE_STATE_DTR 1
static void usbd_cdc_acm_set_dtr(uint8_t bus, uint8_t intf, bool value) { (void)bus; (void)intf; (void)value; }
static void usbd_cdc_acm_set_rts(uint8_t bus, uint8_t intf, bool value) { (void)bus; (void)intf; (void)value; }
static void usbd_cdc_acm_send_break(uint8_t bus, uint8_t intf) { (void)bus; (void)intf; }
''' + fn(usb,'void CDC_SetBaudRateCb(') + fn(usb,'void CDC_SetCtrlLineStateCb(')
    + fn(usb,'void usbVcpBaudRateChanged(') + fn(usb,'void usbVcpControlLineStateChanged(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c','void usbd_cdc_acm_set_line_coding(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c','void usbd_cdc_acm_get_line_coding(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c','void usbd_cdc_acm_set_control_line_state(')
    + fn('lib/modules/CH32H41x/middlewares/class/cdc/usbd_cdc_acm.c','static int cdc_acm_class_interface_request_handler(') + '''
static unsigned baudCalls, stateCalls, seenBaud, seenState;
static void onBaud(void *ctx, uint32_t baud) { if (ctx) { baudCalls++; seenBaud=baud; } }
static void onState(void *ctx, uint16_t state) { if (ctx) { stateCalls++; seenState=state; } }
int main(void) {
    int token=1;
    CDC_SetBaudRateCb(onBaud, &token); CDC_SetCtrlLineStateCb(onState, &token);
    struct cdc_line_coding coding={230400,0,0,8}; uint8_t *data=(uint8_t *)&coding; uint32_t len=7;
    struct usb_setup_packet setup={.bRequest=CDC_REQUEST_SET_LINE_CODING,.wLength=7};
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) || baudCalls!=1 || seenBaud!=230400 || cdc_vcp_line_coding.dwDTERate!=230400) return 1;
    for (unsigned n=0; n<32; n++) {
        if (n==7) continue;
        setup.wLength=n; len=n;
        if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) != -1 || baudCalls!=1) return 4;
    }
    setup.wLength=len=7;
    coding.dwDTERate=0;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) != -1) return 5;
    coding.dwDTERate=0x40000000u;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) != -1) return 6;
    coding.dwDTERate=230400; coding.bParityType=5;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) != -1) return 7;
    coding.bParityType=0; coding.bCharFormat=3;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) != -1 || baudCalls!=1) return 8;
    setup.bRequest=CDC_REQUEST_SET_CONTROL_LINE_STATE; setup.wValue=3;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) || stateCalls!=1 || seenState!=3 || !usbHostPortOpen) return 2;
    CDC_SetCtrlLineStateCb(NULL,NULL); setup.wValue=0;
    if (cdc_acm_class_interface_request_handler(0,&setup,&data,&len) || stateCalls!=1 || usbHostPortOpen) return 3;
    puts("CDC control requests deliver baud and combined line state once");
}
''')
run('usb_tx_empty_state', '''
#define UNUSED(x) ((void)(x))
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
typedef int serialPort_t;
static struct { uint32_t txAt; } vcpPort;
static uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static uint8_t ep_tx_busy_flag;
static bool usbHostPortOpen, usbZlpPending, usbTxStalled, configured;
static bool usbIsConfigured(void) { return configured; }
''' + fn(usb,'static bool isUsbVcpTransmitBufferEmpty(') + '''
int main(void) {
    UserTxBufPtrIn=5; ep_tx_busy_flag=1;
    if (!isUsbVcpTransmitBufferEmpty(NULL)) return 1;
    configured=true;
    if (!isUsbVcpTransmitBufferEmpty(NULL)) return 2;
    usbHostPortOpen=true;
    if (isUsbVcpTransmitBufferEmpty(NULL)) return 3;
    UserTxBufPtrOut=5;
    if (isUsbVcpTransmitBufferEmpty(NULL)) return 4;
    ep_tx_busy_flag=0; lastBuffsize=5;
    if (isUsbVcpTransmitBufferEmpty(NULL)) return 5;
    lastBuffsize=0; vcpPort.txAt=1;
    if (isUsbVcpTransmitBufferEmpty(NULL)) return 6;
    vcpPort.txAt=0;
    if (!isUsbVcpTransmitBufferEmpty(NULL)) return 7;
    puts("disconnected, closed, buffered, queued, in-flight, ZLP and empty states");
}
''')
run('usb_tx_wait_stall', '''
#define PLATFORM_VCP_TX_STALL_TIMEOUT_MS 500
#define USE_VCP
#define SERIAL_PORT_USB_VCP 1
typedef struct { int identifier; } serialPort_t;
static uint32_t now, freeBytes;
static bool empty;
static uint32_t millis(void) { return now; }
static uint32_t serialTxBytesFree(serialPort_t *p) { (void)p; return freeBytes; }
static bool isSerialTransmitBufferEmpty(serialPort_t *p) { (void)p; return empty; }
static void delay(int ms) { now+=ms; if (now == 300) freeBytes++; if (now == 900) empty=true; }
''' + fn('src/main/io/serial.c','void waitForSerialPortToFinishTransmitting(') + '''
int main(void) {
    serialPort_t p={SERIAL_PORT_USB_VCP};
    waitForSerialPortToFinishTransmitting(&p);
    if (now != 800 || empty) return 1;
    now=0; empty=true;
    waitForSerialPortToFinishTransmitting(&p);
    if (now != 0) return 2;
    puts("host stall is bounded and the timeout resets when transmission progresses");
}
''')
run('printf_vcp_no_10ms_stall', '''
#define PLATFORM_VCP_TX_STALL_TIMEOUT_MS 500
#define USE_VCP
#define SERIAL_PORT_USB_VCP 1
typedef struct { int identifier; } serialPort_t;
static uint32_t nowMs, freeBytes, emptyChecks, ignoreCalls;
static bool completeOnCheck3;
static uint32_t millis(void) { return nowMs; }
static uint32_t serialTxBytesFree(serialPort_t *p) { (void)p; return freeBytes; }
static void schedulerIgnoreTaskExecTime(void) {
    ignoreCalls++;
    if (!completeOnCheck3) {
        nowMs += 100;
        if (nowMs == 300) {
            freeBytes++;
        }
    }
}
static bool isSerialTransmitBufferEmpty(serialPort_t *p) {
    (void)p;
    emptyChecks++;
    return completeOnCheck3 && emptyChecks >= 3;
}
''' + fn('src/main/common/printf_serial.c','static void waitForPrintfTransmitComplete(') + '''
int main(void) {
    serialPort_t p={SERIAL_PORT_USB_VCP};
    /* Fast microframe drain completes with zero millisecond sleep */
    nowMs=0; emptyChecks=ignoreCalls=0; completeOnCheck3=true;
    waitForPrintfTransmitComplete(&p);
    if (nowMs != 0 || emptyChecks != 3 || ignoreCalls != 2) return 1;

    /* Stalled host terminates at PLATFORM_VCP_TX_STALL_TIMEOUT_MS after last progress */
    nowMs=0; freeBytes=0; emptyChecks=ignoreCalls=0; completeOnCheck3=false;
    waitForPrintfTransmitComplete(&p);
    if (nowMs != 800 || ignoreCalls != 8) return 2;
    puts("printf VCP drain avoids 10ms sleep while bounding host stalls and discounting task time");
}
''')
for mps in (64, 512):
 run('usb_zlp_mps_'+str(mps), '''
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define CDC_MAX_MPS TEST_MPS
#define USB_VCP_BULK_IN_MPS CDC_MAX_MPS
static uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static uint8_t ep_tx_busy_flag;
static bool usbZlpPending;
static unsigned sendCount, lastLength;
static uint8_t usbIsConfigured(void) { return 1; }
static int usb_vcp_send_data(int bus, uint8_t *buffer, unsigned length) {
    (void)bus; (void)buffer;
    sendCount++; lastLength=length; ep_tx_busy_flag=1;
    return 0;
}
''' + fn(usb,'static void usbVcpKickTx(void)') + fn(usb,'void usb_int_rxsof_handler(void)') + '''
static void tick(void) { usb_int_rxsof_handler(); }
int main(void) {
    UserTxBufPtrIn=64;
    tick();
    if (sendCount != 1 || lastLength != 64) return 1;
    ep_tx_busy_flag=0;
    tick();
    if (sendCount != (TEST_MPS == 64 ? 2u : 1u)) return 2;
    ep_tx_busy_flag=0;
    UserTxBufPtrIn=576;
    tick();
    if (lastLength != 512) return 3;
    ep_tx_busy_flag=0;
    unsigned before=sendCount;
    tick();
    if (sendCount != before+1 || lastLength != 0) return 4;
    puts("ZLP follows the configured bulk endpoint packet size without 17ms SOF throttle");
}
''',extra_flags=('-DTEST_MPS='+str(mps),))
pre='''
#define USB_TIMEOUT 50
#define UNUSED(x) ((void)(x))
#define NVIC_BUILD_PRIORITY(a,b) 0
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for(int once=1;once;once=0)
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define CDC_MAX_MPS 64
#define USB_VCP_BULK_IN_MPS CDC_MAX_MPS
static volatile uint32_t now_ms,UserTxBufPtrIn,UserTxBufPtrOut,lastBuffsize;
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE],ep_tx_busy_flag;
static volatile bool usbZlpPending,usbTxStalled;
typedef uint32_t timeUs_t;
typedef int serialPort_t;
static timeUs_t micros(void){return now_ms*1000;}
static uint32_t millis(void){return now_ms;}
static void delay(int ms){now_ms+=ms;}
static unsigned ignoredExecCalls;
static void schedulerIgnoreTaskExecTime(void){ignoredExecCalls++;}
static bool usbVcpIsConnected(void){return true;}
static bool usbIsConnected(void){return true;}
static bool usbIsConfigured(void){return true;}
static uint16_t usb_vcp_send_data(uint8_t bus, uint8_t *buf, uint16_t len){(void)bus;(void)buf;(void)len;if(ep_tx_busy_flag)return 1;ep_tx_busy_flag=1;return 0;}
'''
run('usb_host_stops_reading',pre+fn(usb,'static void usbVcpKickTx(void)')+fn(usb,'uint32_t CDC_Send_FreeBytes(void)')+fn(usb,'uint32_t CDC_Send_DATA(')+fn(usb,'static void usbVcpWriteBuf(')+'''
int main(void){static uint8_t data[APP_TX_DATA_SIZE]={0};usbVcpWriteBuf(NULL,data,sizeof(data));if(now_ms<50||now_ms>60)return 1;printf("returned after %u ms\\n",now_ms);}
''',timeout=0.3)

uc='src/platform/CH32/serial_uart_ch32bsp.c'
pre='''
#define NVIC_PRIO_SERIALUART_TXDMA 0x50
#define ATOMIC_BLOCK(x) for(int once=1;once;once=0)
#define ENABLE 1
#define DISABLE 0
#define USART_FLAG_TC 1
#define DMA_IT_TCIF 2
#define DMA_IT_HTIF 4
#define DMA_IT_TEIF 8
typedef struct {uint32_t CFGR,CNTR,MADDR;} dma_t;
typedef struct {int unused;} USART_TypeDef;
#define DMA_ARCH_TYPE dma_t
#define IS_DMA_ENABLED(d) (((dma_t *)(d))->CFGR & 1)
#define DMA_GET_FLAG_STATUS(d,f) ((d)->flags & (f))
#define DMA_CLEAR_FLAG(d,f) ((d)->flags &= ~(f))
typedef struct {void *USARTx;dma_t *txDMAResource;bool txDMAEmpty,txCompletePending;struct{uint8_t txBuffer[8];unsigned txBufferHead,txBufferTail,txBufferSize;}port;} uartPort_t;
typedef struct {uartPort_t port;} uartDevice_t;
typedef struct {uintptr_t userParam;uint32_t flags;} dmaChannelDescriptor_t;
static unsigned xDMA_GetCurrDataCounter(dma_t *d){return d->CNTR;}
static void xDMA_SetCurrDataCounter(dma_t *d,unsigned n){d->CNTR=n;}
static void xDMA_Cmd(dma_t *d,int enabled){d->CFGR=enabled;}
static void USART_ClearFlag(USART_TypeDef *u,int flag){(void)u;(void)flag;}
static void ch32DmaBarrier(void){}
static void uartTxMonitor(uartPort_t *p){}
'''
run('uart_tx_complete',pre+fn(uc,'void uartTryStartTxDMA(')+fn(uc,'static void uartTxMonitorIfComplete')+fn(uc,'static void handleUsartTxDma(')+fn(uc,'void uartDmaIrqHandler(')+'''
int main(void){dma_t reg={.CFGR=1,.CNTR=0};uartDevice_t d={.port={.txDMAResource=&reg,.port={.txBufferHead=3,.txBufferTail=1,.txBufferSize=8}}};dmaChannelDescriptor_t desc={.userParam=(uintptr_t)&d,.flags=DMA_IT_TCIF};uartDmaIrqHandler(&desc);printf("after TC: EN=%u CNTR=%u head=%u tail=%u (expected CNTR=2, tail=3)\\n",reg.CFGR,reg.CNTR,d.port.port.txBufferHead,d.port.port.txBufferTail);return reg.CNTR!=2;}
''')

ac='src/platform/CH32/adc_ch32h41x.c'
pre='''
#define USE_ADC_INTERNAL
#define USE_DMA_SPEC
#define ADCDEV_COUNT 2
#define ADCDEV_1 0
#define ADC_TAG_MAP_TEMPSENSOR 1
#define ADC_TAG_MAP_VREFINT 0
#define ADC_SampleTime_CyclesMode7 7
#define ARRAYLEN(x) (sizeof(x)/sizeof((x)[0]))
#define IO_CONFIG(a,b,c,d) 0
#define OWNER_ADC_BATT 1
#define OWNER_ADC 7
#define RESOURCE_INDEX(x) ((x)+1)
#define ENABLE 1
#define DISABLE 0
#define RCC_ADCCLKSource_HCLK 0
#define RCC_PPRE2_DIV0 0
#define RCC_HCLK_ADCPRE_DIV8 8
#define DMA_PERIPH_ADC 1
#define DMA_DIR_PeripheralSRC 0
#define DMA_PeripheralInc_Disable 0
#define DMA_MemoryInc_Enable 1
#define DMA_PeripheralDataSize_HalfWord 1
#define DMA_MemoryDataSize_Word 2
#define DMA_Mode_Circular 1
#define DMA_Priority_Low 0
#define DMA_M2M_Disable 0
#define ADC_SW_PRE 0x80000000U
#define VREFINT_CAL_VREF 3300
typedef enum {ADC_BATTERY,ADC_CURRENT,ADC_EXTERNAL1,ADC_RSSI,ADC_EXTERNAL_COUNT,ADC_TEMPSENSOR=ADC_EXTERNAL_COUNT,ADC_VREFINT,ADC_SOURCE_COUNT} adcSource_e;
typedef struct {int adcDevice,adcChannel,sampleTime,tag,dmaIndex;bool enabled;} adcOperatingConfig_t;
static adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
static uint32_t adcConversionBuffer[ADC_SOURCE_COUNT];
static int32_t adcVREFINTCAL;
typedef struct {uint32_t RDATAR,CTLR1;} ADC_TypeDef;
typedef struct {ADC_TypeDef *ADCx;int rccADC;} adcDevice_t;
static ADC_TypeDef regs[2];
static const adcDevice_t adcHardware[]={{&regs[0],1},{&regs[1],2}};
static struct {int channel;} adcTagMap[]={{17},{16}};
typedef struct {struct {bool enabled;int ioTag;}vbat,rssi,external1,current;int dmaopt[2];} adcConfig_t;
typedef struct {uint32_t DMA_PeripheralBaseAddr,DMA_Memory0BaseAddr,DMA_DIR,DMA_BufferSize,DMA_PeripheralInc,DMA_MemoryInc,DMA_PeripheralDataSize,DMA_MemoryDataSize,DMA_Mode,DMA_Priority,DMA_M2M;} DMA_InitTypeDef;
typedef struct {void *ref;int dmaMuxId;} dmaChannelSpec_t;
typedef int dmaIdentifier_e;
static int conversion_started;
static void setScalingFactors(void){adcVREFINTCAL=1501;}
static int adcFindTagMapEntry(int tag){return -1;}
static void *IOGetByTag(int tag){return NULL;}
static void IOInit(void *a,int b,int c){}
static void IOConfigGPIO(void *a,int b){}
static void RCC_ADCCLKConfig(int a){}
static void RCC_ADCHCLKCLKAsSourceConfig(int a,int b){}
static void RCC_ClockCmd(int a,int b){}
static void adcInitDevice(const adcDevice_t *a,int b){}
static const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(int a,int b,int opt){return NULL;}
static int dmaGetIdentifier(void *a){return 1;}
static bool dmaAllocate(int a,int b,int c){return false;}
static void dmaEnable(int a){}
static void xDMA_DeInit(void *a){}
static void xDMA_Init(void *a,DMA_InitTypeDef *b){}
static void dmaMuxEnable(int a,int b){}
static void xDMA_Cmd(void *a,int b){}
static void ADC_DMACmd(ADC_TypeDef *a,int b){}
static void ADC_RegularChannelConfig(ADC_TypeDef *a,int b,int c,int d){}
static void ADC_Cmd(ADC_TypeDef *a,int b){}
static void ADC_BufferCmd(ADC_TypeDef *a,int b){}
static void ADC_LowPowerModeCmd(ADC_TypeDef *a,int b){}
static void ADC_TempSensorVrefintCmd(int a){}
static void ADC_SoftwareStartConvCmd(ADC_TypeDef *a,int b){conversion_started++;}
'''
run('adc_no_dma',pre+fn(ac,'void adcInit(')+fn(ac,'bool adcInternalIsBusy(')+fn(ac,'uint16_t adcInternalRead(')+fn('src/main/drivers/adc.c','uint16_t adcInternalCompensateVref(')+'''
int main(void){
    setbuf(stdout,NULL);
    adcConversionBuffer[0]=2048U;
    adcConfig_t c={.dmaopt={-1,-1}};
    adcInit(&c);
    uint16_t raw=adcInternalRead(ADC_VREFINT);
    uint16_t vref=adcInternalCompensateVref(raw);
    if (c.dmaopt[0] != -1 || conversion_started != 0) return 1;
    if (adcOperatingConfig[ADC_VREFINT].enabled || adcOperatingConfig[ADC_TEMPSENSOR].enabled) return 2;
    if (!adcInternalIsBusy() || raw != 0 || vref != VREFINT_CAL_VREF) return 3;
    printf("DMA opt=%d conversions started=%d vref enabled=%d busy=%d raw=%u vref=%u\\n",c.dmaopt[0],conversion_started,adcOperatingConfig[ADC_VREFINT].enabled,adcInternalIsBusy(),raw,vref);
    return 0;
}
''')

run('adc_internal_init_timeout', '''
#define ADC_INTERNAL_INIT_TIMEOUT_US 10000u
typedef uint32_t timeUs_t;
static uint32_t now, processed;
static bool unavailable;
static timeUs_t micros(void) { return now += 100; }
static bool adcInternalIsBusy(void) { return unavailable; }
static void adcInternalProcess(timeUs_t unused) { (void)unused; processed++; }
''' + fn('src/main/sensors/adcinternal.c','void adcInternalInit(') + '''
int main(void) {
    unavailable = true; adcInternalInit();
    if (processed || now > 10200) return 1;
    unavailable = false; now = 0; adcInternalInit();
    if (processed != 9) return 2;
    return 0;
}
''')

ic='src/platform/CH32/bus_i2c_ch32h41x.c'
pre='''
#define I2CINVALID -1
#define ENABLE 1
#define DISABLE 0
#define OWNER_I2C_SCL 1
#define OWNER_I2C_SDA 2
#define RESOURCE_INDEX(x) ((x)+1)
#define I2C_IT_EVT 1
#define I2C_IT_ERR 2
#define IOCFG_I2C_PU 1
#define IOCFG_I2C 0
#define I2C_Mode_I2C 0
#define I2C_DutyCycle_2 0
#define I2C_Ack_Enable 1
#define I2C_AcknowledgedAddress_7bit 0
#define NVIC_PRIO_I2C_ER 0
#define NVIC_PRIO_I2C_EV 0
typedef int i2cDevice_e;
typedef int *IO_t;
typedef int I2C_TypeDef;
typedef struct {I2C_TypeDef *reg;int rcc,er_irq,ev_irq;} i2cHardware_t;
typedef struct {const i2cHardware_t *hardware;IO_t scl,sda;struct {bool busy,error;}state;int pullUp,sclAF,sdaAF,clockSpeed;} i2cDevice_t;
static i2cDevice_t i2cDevice[1];
typedef struct {int I2C_Mode,I2C_DutyCycle,I2C_OwnAddress1,I2C_Ack,I2C_AcknowledgedAddress,I2C_ClockSpeed;} I2C_InitTypeDef;
static int recovery_attempts,i2cErrorCount;
static int IOGetOwner(IO_t io){return *io;}
static bool IOIsFreeOrPreinit(IO_t io){return *io==0;}
static void IOInit(IO_t io,int owner,int idx){*io=owner;}
static void RCC_ClockCmd(int a,int b){}
static void I2C_ITConfig(I2C_TypeDef *a,int b,int c){}
static void i2cUnstick(IO_t a,IO_t b){recovery_attempts++;}
static void IOConfigGPIOAF(IO_t a,int b,int c){}
static void I2C_DeInit(I2C_TypeDef *a){}
static void I2C_StructInit(I2C_InitTypeDef *a){}
static void I2C_Init(I2C_TypeDef *a,I2C_InitTypeDef *b){}
static void I2C_Cmd(I2C_TypeDef *a,int b){}
static void I2C_StretchClockCmd(I2C_TypeDef *a,int b){}
static void NVIC_SetPriority(int a,int b){}
static void NVIC_EnableIRQ(int a){}
'''
run('i2c_recovery',pre+fn(ic,'void i2cInit(')+fn(ic,'static bool i2cHandleHardwareFailure(')+'''
int main(void){int scl=0,sda=0,reg=0;i2cHardware_t hw={.reg=&reg};i2cDevice[0]=(i2cDevice_t){.hardware=&hw,.scl=&scl,.sda=&sda};i2cInit(0);recovery_attempts=0;i2cDevice[0].state.busy=true;i2cHandleHardwareFailure(0);printf("timeout recovery: unstick calls=%d busy=%d owners=%d/%d\\n",recovery_attempts,i2cDevice[0].state.busy,scl,sda);return recovery_attempts!=1;}
''')

pre='''
#define TIMER_CHANNEL_COUNT 3
typedef int ioTag_t;
typedef struct {int unused;} TIM_TypeDef;
typedef TIM_TypeDef timerResource_t;
typedef struct {ioTag_t tag;timerResource_t *tim;} timerHardware_t;
static TIM_TypeDef tim2,tim5,tim9;
#define TIM2 (&tim2)
#define TIM5 (&tim5)
#define TIM9 (&tim9)
#define TIMER_HARDWARE fullTimerHardware
static const timerHardware_t fullTimerHardware[]={{1,TIM2},{1,TIM5},{1,TIM9}};
'''
run('timer_af_index',pre+fn('src/main/drivers/timer_common.c','const timerHardware_t *timerGetByTagAndIndex')+'''
int main(void){
 const timerHardware_t *first=timerGetByTagAndIndex(1,1), *second=timerGetByTagAndIndex(1,2), *third=timerGetByTagAndIndex(1,3);
 if (!first || first->tim!=TIM2 || !second || second->tim!=TIM5 || !third || third->tim!=TIM9) return 1;
 if (timerGetByTagAndIndex(1,4) != NULL || timerGetByTagAndIndex(2,1) != NULL) return 2;
 puts("timerGetByTagAndIndex enumerates all hardware timer channels cleanly");
}
''')

pre='''
#define FAST_IRQ_HANDLER
#define DMA_IT_TCIF 2
#define DMA_IT_HTIF 4
#define DMA_IT_TEIF 8
#define USART_FLAG_TC 0x40
#define USART_STATR_ORE 0x08
#define USART_STATR_IDLE 0x10
#define USART_STATR_RXNE 0x20
#define USART_STATR_TC 0x40
#define USART_STATR_TXE 0x80
#define USART_CTLR1_IDLEIE 0x10
#define USART_CTLR1_RXNEIE 0x20
#define USART_CTLR1_TCIE 0x40
#define USART_CTLR1_TXEIE 0x80
#define CLEAR_BIT(reg,bit) ((reg)&=~(bit))
#define ENABLE 1
#define DISABLE 0
typedef struct {uint32_t CFGR,CNTR,MADDR;} dma_t;
typedef struct {uint16_t STATR,reserved0,DATAR,reserved1,BRR,reserved2,CTLR1,reserved3;} USART_TypeDef;
#define IS_DMA_ENABLED(d) (((dma_t *)(d))->CFGR & 1)
typedef struct {void *USARTx;dma_t *txDMAResource,*rxDMAResource;bool txDMAEmpty,txCompletePending;struct{uint8_t txBuffer[8];unsigned txBufferHead,txBufferTail,txBufferSize;uint8_t rxBuffer[8];unsigned rxBufferHead,rxBufferSize;void(*rxCallback)(uint32_t,void*);void*rxCallbackData;void(*idleCallback)(void);}port;} uartPort_t;
typedef struct {uartPort_t port;} uartDevice_t;
static unsigned monitorCalls,reads,received,receivedData,idleCalls,injectLate;
static void fakeClearTC(USART_TypeDef *u) { u->STATR &= ~USART_STATR_TC; if (injectLate) { u->STATR |= USART_STATR_RXNE; u->DATAR=0x5a; } }
static uint16_t fakeReadData(USART_TypeDef *u) { uint16_t data=u->DATAR; reads++; u->STATR &= ~(USART_STATR_IDLE|USART_STATR_RXNE|USART_STATR_ORE); return data; }
static void onRx(uint32_t data,void *ctx) { (void)ctx; received++; receivedData=data; }
static void onIdle(void) { idleCalls++; }
static void uartTxMonitor(uartPort_t *p){(void)p;monitorCalls++;}
'''
uart_irq = fn('src/platform/CH32/serial_uart_ch32bsp.c','FAST_IRQ_HANDLER void uartIrqHandler').replace(
    'USARTx->STATR = (uint16_t)~USART_STATR_TC;', 'fakeClearTC(USARTx);').replace(
    'const uint16_t data = USARTx->DATAR;', 'const uint16_t data = fakeReadData(USARTx);')
uart_rx = fn('src/platform/CH32/serial_uart_ch32bsp.c','static inline void uartStoreRxByte')
run('uart_idle_registration', '''
#include <stddef.h>
#define USE_PLATFORM_UART_IDLE_CALLBACK
#define NVIC_PRIO_SERIALUART1 0
#define ATOMIC_BLOCK(x) for (int once=1; once; once=0)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define USART_IT_IDLE 0x10
#define ENABLE 1
#define DISABLE 0
#define MODE_RX 1
typedef void (*serialIdleCallbackPtr)(void);
typedef struct serialPort_s serialPort_t;
struct vtable { void (*setIdleCallback)(serialPort_t *, serialIdleCallbackPtr); };
struct serialPort_s { const struct vtable *vTable; serialIdleCallbackPtr idleCallback; int mode; };
typedef struct { uint32_t CTLR1; } USART_TypeDef;
typedef struct { serialPort_t port; void *USARTx, *rxDMAResource; } uartPort_t;
static void USART_ITConfig(USART_TypeDef *u, int flag, int enabled) { if (enabled) u->CTLR1 |= flag; else u->CTLR1 &= ~flag; }
''' + fn('src/platform/CH32/serial_uart_ch32bsp.c','void uartSetIdleCallback(')
    + fn('src/main/drivers/serial.c','void serialSetIdleCallback(') + '''
static void cb(void) {}
int main(void) {
    USART_TypeDef u={0}; static const struct vtable vt={uartSetIdleCallback}, plain={NULL};
    uartPort_t p={.port={.vTable=&vt,.mode=MODE_RX},.USARTx=&u};
    serialSetIdleCallback(&p.port,cb);
    if (p.port.idleCallback!=cb || !(u.CTLR1 & USART_IT_IDLE)) return 1;
    serialSetIdleCallback(&p.port,NULL);
    if (p.port.idleCallback || (u.CTLR1 & USART_IT_IDLE)) return 2;
    p.rxDMAResource=&u;serialSetIdleCallback(&p.port,cb);
    if (u.CTLR1 & USART_IT_IDLE) return 3;
    p.port.vTable=&plain;serialSetIdleCallback(&p.port,NULL);
    if (p.port.idleCallback) return 4;
    puts("callbacks registered after open enable IDLE without reconfiguring the UART");
}
''')
run('uart_line_end_release',pre+uart_rx+fn('src/platform/CH32/serial_uart_ch32bsp.c','static void uartTxMonitorIfComplete')+uart_irq+'''
int main(void){USART_TypeDef usart={.CTLR1=USART_CTLR1_TCIE};uartDevice_t d={.port={.USARTx=&usart,.txDMAResource=NULL,.rxDMAResource=NULL,.port={.txBufferHead=2,.txBufferTail=1,.txBufferSize=8}}};
 usart.STATR=USART_STATR_TC;uartIrqHandler(&d.port);if(monitorCalls)return 1;
 d.port.port.txBufferTail=d.port.port.txBufferHead;usart.STATR=USART_STATR_TC;uartIrqHandler(&d.port);return monitorCalls!=1;}
''')
run('uart_full_duplex_rx',pre+uart_rx+fn('src/platform/CH32/serial_uart_ch32bsp.c','static void uartTxMonitorIfComplete')+uart_irq+'''
int main(void) {
 USART_TypeDef u={.STATR=USART_STATR_TC|USART_STATR_IDLE,.CTLR1=USART_CTLR1_TCIE};
 uartPort_t s={.USARTx=&u,.port={.rxCallback=onRx,.idleCallback=onIdle,.txBufferSize=8,.rxBufferSize=8}};
 injectLate=1; uartIrqHandler(&s);
 if (reads || received || idleCalls || !(u.STATR & USART_STATR_RXNE)) return 1;
 injectLate=0; uartIrqHandler(&s);
 if (received!=1 || receivedData!=0x5a || idleCalls) return 2;
 received=reads=0; u.STATR=USART_STATR_TC|USART_STATR_IDLE;u.CTLR1=USART_CTLR1_TCIE|USART_CTLR1_IDLEIE;
 injectLate=1; uartIrqHandler(&s);
 if (received!=1 || receivedData!=0x5a || idleCalls!=1 || reads!=1) return 3;
 received=reads=0;injectLate=0;u.STATR=USART_STATR_RXNE|USART_STATR_ORE|USART_STATR_TC|USART_STATR_TXE;u.CTLR1=USART_CTLR1_RXNEIE;u.DATAR=0x37;
 uartIrqHandler(&s);
 if (received!=1 || receivedData!=0x37 || reads!=1 || !(u.STATR & USART_STATR_TC) || !(u.STATR & USART_STATR_TXE)) return 4;
 puts("disabled flags do not trigger TX or IDLE; late RX and RXNE+ORE are delivered");
}
''')
pre += '''
#define NVIC_PRIO_SERIALUART_TXDMA 0x50
#define ATOMIC_BLOCK(x) for(int once=1;once;once=0)
#define DMA_ARCH_TYPE dma_t
#define DMA_GET_FLAG_STATUS(d,f) ((d)->flags & (f))
#define DMA_CLEAR_FLAG(d,f) ((d)->flags &= ~(f))
typedef struct {uintptr_t userParam;uint32_t flags;} dmaChannelDescriptor_t;
static unsigned xDMA_GetCurrDataCounter(dma_t *d){return d->CNTR;}
static void USART_ClearFlag(USART_TypeDef *u, int flag){u->STATR &= ~flag;}
static void xDMA_SetCurrDataCounter(dma_t *d,unsigned n){d->CNTR=n;}
static void xDMA_Cmd(dma_t *d,int enabled){d->CFGR=enabled;}
static void ch32DmaBarrier(void){}
'''
run('uart_usart_before_dma',pre+fn('src/platform/CH32/serial_uart_ch32bsp.c','void uartTryStartTxDMA(')+fn('src/platform/CH32/serial_uart_ch32bsp.c','static void uartTxMonitorIfComplete')+fn('src/platform/CH32/serial_uart_ch32bsp.c','static void handleUsartTxDma(')+fn('src/platform/CH32/serial_uart_ch32bsp.c','void uartDmaIrqHandler(')+uart_rx+uart_irq+'''
int main(void){USART_TypeDef usart={.CTLR1=USART_CTLR1_TCIE};dma_t reg={.CFGR=1,.CNTR=0};uartDevice_t d={.port={.USARTx=&usart,.txDMAResource=&reg,.rxDMAResource=NULL,.port={.txBufferHead=2,.txBufferTail=2,.txBufferSize=8}}};
 usart.STATR=USART_STATR_TC;uartIrqHandler(&d.port);if(monitorCalls||!d.port.txCompletePending)return 1;
 dmaChannelDescriptor_t desc={.userParam=(uintptr_t)&d,.flags=DMA_IT_TCIF};uartDmaIrqHandler(&desc);
 return reg.CFGR!=0||monitorCalls!=1||d.port.txCompletePending;}
''')

dma_hdr = (root/'lib/modules/CH32H41x/Peripheral/inc/ch32h417_dma.h').read_text()
dma_enum = dma_hdr[dma_hdr.index('typedef enum'):dma_hdr.index('} dmamux_requst_id_sel_type;')+len('} dmamux_requst_id_sel_type;')]
reqmap_txt = (root/'src/platform/CH32/dma_reqmap_mcu.c').read_text()
reqmap_body = reqmap_txt[reqmap_txt.index('typedef struct dmaPeripheralMapping_s'):reqmap_txt.index('const dmaChannelSpec_t *dmaGetChannelSpecByTimer(')]
timer_def_txt = (root/'src/platform/CH32/timer_def.h').read_text()
timer_def_body = timer_def_txt[timer_def_txt.index('#define USED_TIMERS'):timer_def_txt.index('// AF table for timer')]
reqmap_common = '''
#include "''' + str(root/'src/main/common/utils.h') + '''"
#include "''' + str(root/'src/platform/CH32/dma_reqmap_mcu.h') + '''"
''' + dma_enum + '''
typedef struct dmaResource_s { int dummy; } dmaResource_t;
typedef struct timerResource_s { int dummy; } timerResource_t;
typedef uint8_t ioTag_t;
typedef uint16_t dmaCode_t;
typedef int8_t dmaoptValue_t;
#define DMA_OPT_UNUSED (-1)
typedef struct dmaChannelSpec_s {
    dmaCode_t code;
    dmaResource_t *ref;
    uint32_t dmaMuxId;
} dmaChannelSpec_t;
#define DMA_CODE(dma, stream, chanreq) (((dma) << 12) | ((stream) << 8) | ((chanreq) << 0))
#define DMA_CODE_CONTROLLER(code) (((code) >> 12) & 0xf)
#define DMA_CODE_STREAM(code) (((code) >> 8) & 0xf)
typedef enum {
    DMA_PERIPH_SPI_SDO,
    DMA_PERIPH_SPI_SDI,
    DMA_PERIPH_ADC,
    DMA_PERIPH_SDIO,
    DMA_PERIPH_UART_TX,
    DMA_PERIPH_UART_RX,
    DMA_PERIPH_TIMUP,
} dmaPeripheral_e;
#define TIM_Channel_1 0x00
#define TIM_Channel_2 0x04
#define TIM_Channel_3 0x08
#define TIM_Channel_4 0x0C
#define TIMER_OUTPUT_NONE 0
#define TIMER_OUTPUT_N_CHANNEL 1
static timerResource_t t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12;
#define TIM1 (&t1)
#define TIM2 (&t2)
#define TIM3 (&t3)
#define TIM4 (&t4)
#define TIM5 (&t5)
#define TIM6 (&t6)
#define TIM7 (&t7)
#define TIM8 (&t8)
#define TIM9 (&t9)
#define TIM10 (&t10)
#define TIM11 (&t11)
#define TIM12 (&t12)
static dmaResource_t d1[8], d2[8];
#define DMA1_Channel1 (&d1[0])
#define DMA1_Channel2 (&d1[1])
#define DMA1_Channel3 (&d1[2])
#define DMA1_Channel4 (&d1[3])
#define DMA1_Channel5 (&d1[4])
#define DMA1_Channel6 (&d1[5])
#define DMA1_Channel7 (&d1[6])
#define DMA1_Channel8 (&d1[7])
#define DMA2_Channel1 (&d2[0])
#define DMA2_Channel2 (&d2[1])
#define DMA2_Channel3 (&d2[2])
#define DMA2_Channel4 (&d2[3])
#define DMA2_Channel5 (&d2[4])
#define DMA2_Channel6 (&d2[5])
#define DMA2_Channel7 (&d2[6])
#define DMA2_Channel8 (&d2[7])
#define USE_TIMER
#define USE_SPI
#define USE_ADC
#define SPIDEV_1 0
#define SPIDEV_2 1
#define SPIDEV_3 2
#define SPIDEV_4 3
#define ADCDEV_1 0
#define ADCDEV_2 1
#define TIM_N(n) (1U << (n))
''' + timer_def_body + '''
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)
''' + reqmap_body

run('dma_reqmap_full', reqmap_common + '''
int main(void) {
    if (MAX_TIMER_DMA_OPTIONS != 16 || MAX_PERIPHERAL_DMA_OPTIONS != 16) return 1;
    timerResource_t *tims[9] = {TIM1, TIM2, TIM3, TIM4, TIM5, TIM8, TIM9, TIM10, TIM11};
    uint32_t ch1_reqs[9] = {
        DMAMUX_DMAREQ_ID_TIM1_CH1, DMAMUX_DMAREQ_ID_TIM2_CH1, DMAMUX_DMAREQ_ID_TIM3_CH1,
        DMAMUX_DMAREQ_ID_TIM4_CH1, DMAMUX_DMAREQ_ID_TIM5_CH1, DMAMUX_DMAREQ_ID_TIM8_CH1,
        DMAMUX_DMAREQ_ID_TIM9_CH1, DMAMUX_DMAREQ_ID_TIM10_CH1, DMAMUX_DMAREQ_ID_TIM11_CH1,
    };
    for (unsigned i = 0; i < 9; i++) {
        const dmaChannelSpec_t *s = dmaGetChannelSpecByTimerValue(tims[i], TIM_Channel_1, 13);
        if (!s || s->dmaMuxId != ch1_reqs[i]) return 1 + i;
    }
    const dmaChannelSpec_t *s12 = dmaGetChannelSpecByTimerValue(TIM12, TIM_Channel_4, 15);
    if (!s12 || s12->dmaMuxId != DMAMUX_DMAREQ_ID_TIM12_CH4) return 20;
    uint32_t up_reqs[12] = {
        DMAMUX_DMAREQ_ID_TIM1_UP, DMAMUX_DMAREQ_ID_TIM2_UP, DMAMUX_DMAREQ_ID_TIM3_UP,
        DMAMUX_DMAREQ_ID_TIM4_UP, DMAMUX_DMAREQ_ID_TIM5_UP, DMAMUX_DMAREQ_ID_TIM6_UP,
        DMAMUX_DMAREQ_ID_TIM7_UP, DMAMUX_DMAREQ_ID_TIM8_UP, DMAMUX_DMAREQ_ID_TIM9_UP,
        DMAMUX_DMAREQ_ID_TIM10_UP, DMAMUX_DMAREQ_ID_TIM11_UP, DMAMUX_DMAREQ_ID_TIM12_UP,
    };
    for (unsigned n = 1; n <= 12; n++) {
        const dmaChannelSpec_t *su = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, TIMER_INDEX(n), 0);
        if (!su || su->dmaMuxId != up_reqs[n - 1]) return 30 + n;
    }
    if (dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, TIMER_INDEX(3), 16) != NULL) return 50;
    if (dmaGetChannelSpecByTimerValue(TIM9, TIM_Channel_1, 16) != NULL) return 51;
    puts("exposes all TIM1..12 CC and TIM_UP DMAMUX mappings across 16 channels");
}
''')

run('dma_mux_enable_bounds_and_clocks', dma_enum + '''
typedef int dmaIdentifier_e;
#define UNUSED(x) ((void)(x))
#define DMA_NONE 0
#define DMA_FIRST_HANDLER 1
#define DMA_LAST_HANDLER 16
#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - DMA_FIRST_HANDLER)
#define ATOMIC_BLOCK_ALL() for (int __once = (atomic_all_entries++, 1); __once; __once = 0)
#define ENABLE 1
#define RCC_HB_DMA1 0x01u
#define RCC_HB_DMA2 0x02u
#define RCC_HB(periph) RCC_HB_ ## periph
typedef struct { uint32_t CFGR0_3, CFGR4_7, CFGR8_11, CFGR12_15; } DMAMUX_TypeDef;
static DMAMUX_TypeDef mux_hw;
#define DMAMUX (&mux_hw)
static int dma1_token, dma2_token;
#define DMA1 (&dma1_token)
#define DMA2 (&dma2_token)
typedef void (*dmaCallbackHandlerFuncPtr)(void *);
typedef struct { void *dma; dmaCallbackHandlerFuncPtr irqHandlerCallback; int32_t irqN; uint32_t userParam; } dmaChannelDescriptor_t;
static dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER];
static uint32_t enabled_clocks;
static unsigned atomic_all_entries;
static void RCC_ClockCmd(uint32_t periph, int state) { if (state) enabled_clocks |= periph; }
static void NVIC_SetPriority(int32_t irq, uint32_t prio) { (void)irq; (void)prio; }
static void NVIC_EnableIRQ(int32_t irq) { (void)irq; }
static void dummyCb(void *p) { (void)p; }
#define CH32_DMAMUX_MAX_REQUEST_ID DMAMUX_DMAREQ_ID_TIM7_UP
''' + fn('src/platform/CH32/dma_ch32h41x.c', 'static void enableDmaClock(')
    + fn('src/platform/CH32/dma_ch32h41x.c', 'void dmaEnable(')
    + fn('src/platform/CH32/dma_ch32h41x.c', 'void dmaMuxEnable(')
    + fn('src/platform/CH32/dma_ch32h41x.c', 'void dmaSetHandler(') + '''
int main(void) {
    for (int i = 0; i < 8; i++) dmaDescriptors[i].dma = DMA1;
    for (int i = 8; i < 16; i++) dmaDescriptors[i].dma = DMA2;
    mux_hw.CFGR0_3 = 0x55555555u;
    dmaMuxEnable(DMA_NONE, DMAMUX_DMAREQ_ID_TIM1_CH1);
    dmaMuxEnable(DMA_LAST_HANDLER + 1, DMAMUX_DMAREQ_ID_TIM1_CH1);
    dmaMuxEnable(1, 0);
    dmaMuxEnable(1, 0x7Cu);
    dmaEnable(DMA_NONE);
    dmaEnable(DMA_LAST_HANDLER + 1);
    dmaSetHandler(DMA_NONE, dummyCb, 0, 0);
    dmaSetHandler(DMA_LAST_HANDLER + 1, dummyCb, 0, 0);
    if (enabled_clocks != 0 || mux_hw.CFGR0_3 != 0x55555555u) return 1;
    if (atomic_all_entries != 0) return 2;
    dmaMuxEnable(9, DMAMUX_DMAREQ_ID_TIM9_CH1);
    if (enabled_clocks != (RCC_HB_DMA1 | RCC_HB_DMA2)) return 3;
    if ((mux_hw.CFGR8_11 & 0xFFu) != (DMAMUX_DMAREQ_ID_TIM9_CH1 - 1u)) return 4;
    dmaMuxEnable(13, DMAMUX_DMAREQ_ID_TIM8_CH1);
    dmaMuxEnable(16, DMAMUX_DMAREQ_ID_TIM7_UP);
    if (((mux_hw.CFGR12_15 >> 0) & 0xFFu) != (DMAMUX_DMAREQ_ID_TIM8_CH1 - 1u)) return 5;
    if (((mux_hw.CFGR12_15 >> 24) & 0xFFu) != (DMAMUX_DMAREQ_ID_TIM7_UP - 1u)) return 7;
    if (atomic_all_entries != 3) return 8;
    puts("dmaMuxEnable guards bounds/zero-underflow and updates CFGR atomically");
}
''')

bb_txt = (root/'src/platform/CH32/dshot_bitbang.c').read_text()
bb_hw_table = bb_txt[bb_txt.index('const timerHardware_t bbTimerHardware[]'):bb_txt.index(';', bb_txt.index('const timerHardware_t bbTimerHardware[]'))+1]

run('dshot_bb_multiport_dmamux', '''
#define USE_DMA_SPEC
#define MAX_PERIPHERAL_DMA_OPTIONS 16
#define MAX_SUPPORTED_MOTOR_PORTS 4
#define ARRAYLEN(x) (sizeof(x)/sizeof((x)[0]))
#define CC_CHANNELS_PER_TIMER 4
#define CC_CHANNEL_FROM_INDEX(x) ((x)*4)
#define DSHOT_BITBANGED_TIMER_AUTO 0
#define DSHOT_BITBANGED_TIMER_TIM1 1
#define DSHOT_BITBANGED_TIMER_TIM8 2
#define OWNER_FREE 0
#define OWNER_DSHOT_BITBANG 1
#define OWNER_SPI 2
#define TIM1 1
#define TIM8 8
#define NONE 0
#define CH1 0
#define CH2 4
#define CH3 8
#define CH4 12
typedef int resourceOwner_e;
typedef int8_t dmaoptValue_t;
typedef int dmaIdentifier_e;
typedef struct { int idx; } dmaResource_t;
typedef struct { int tim; uint8_t channel; int8_t opt; } timerHardware_t;
#define DEF_TIM(t,ch,tag,out,dma,up) { t, ch, dma }
''' + bb_hw_table + '''
typedef struct { int owner; } resourceOwner_t;
typedef struct { dmaResource_t *ref; uint32_t dmaMuxId; } dmaChannelSpec_t;
static struct { const timerHardware_t *timhw; dmaResource_t *dmaResource; uint32_t dmaChannel; } bbPorts[4];
static dmaResource_t dmas[16];
static resourceOwner_t dmaOwners[16];
static resourceOwner_t freeOwner = { OWNER_FREE };
static struct { struct { int useDshotBitbangedTimer; } dev; } cfg;
#define motorConfig() (&cfg)
static int lookupTimerIndex(int tim) { return tim; }
static int timerGetNumberByIndex(int idx) { return idx; }
static const timerHardware_t *timerGetAllocatedByNumberAndChannel(int t, int c) { (void)t; (void)c; return NULL; }
static const resourceOwner_t *timerGetOwner(const timerHardware_t *t) { (void)t; return &freeOwner; }
static dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *t) { return t->opt; }
static dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *d) { return d->idx + 1; }
static const resourceOwner_t *dmaGetOwner(dmaIdentifier_e id) { return &dmaOwners[id - 1]; }
static dmaChannelSpec_t specBuf;
static const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(int tim, uint8_t ch, dmaoptValue_t opt) {
    if (opt < 0 || opt >= 16) return NULL;
    specBuf.ref = &dmas[opt];
    specBuf.dmaMuxId = (tim == TIM8 ? 32u : 1u) + (ch / 4u);
    return &specBuf;
}
''' + fn('src/platform/CH32/timer_ch32bsp.c', 'int8_t timerGetTIMNumber(')
    + fn('src/platform/CH32/dshot_bitbang.c', 'static void bbFindPacerTimer(void)') + '''
int main(void) {
    for (int i = 0; i < 16; i++) { dmas[i].idx = i; dmaOwners[i].owner = OWNER_FREE; }
    /* Occupy DMA2_Ch1..4 (8..11) and DMA2_Ch6..8 (13..15).
     * Only DMA2_Ch5 (12) and DMA1_Ch1..8 (0..7) remain free! */
    for (int i = 8; i <= 11; i++) dmaOwners[i].owner = OWNER_SPI;
    dmaOwners[13].owner = OWNER_SPI;
    dmaOwners[14].owner = OWNER_SPI;
    dmaOwners[15].owner = OWNER_SPI;

    cfg.dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_AUTO;
    bbFindPacerTimer();
    /* AUTO mode interleaves TIM8, TIM1, TIM8, TIM1 with 4 unique non-reserved DMAMUX channels */
    int expectedTim[4] = { TIM8, TIM1, TIM8, TIM1 };
    uint8_t expectedCh[4] = { CH1, CH1, CH2, CH2 };
    int expectedOpt[4] = { 12, 0, 1, 2 };
    uint32_t expectedMux[4] = { 32, 1, 33, 2 };
    for (int p = 0; p < 4; p++) {
        if (!bbPorts[p].timhw || bbPorts[p].timhw->tim != expectedTim[p] || bbPorts[p].timhw->channel != expectedCh[p]) return 1 + p;
        if (bbPorts[p].dmaResource != &dmas[expectedOpt[p]] || bbPorts[p].dmaChannel != expectedMux[p]) return 10 + p;
    }

    /* Forced TIM1 mode assigns TIM1 CH1..4 across 4 unique DMAMUX channels */
    cfg.dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_TIM1;
    bbFindPacerTimer();
    for (int p = 0; p < 4; p++) {
        if (!bbPorts[p].timhw || bbPorts[p].timhw->tim != TIM1 || bbPorts[p].timhw->channel != p * 4) return 20 + p;
        for (int q = 0; q < p; q++) {
            if (bbPorts[p].dmaResource == bbPorts[q].dmaResource) return 30 + p;
        }
    }
    puts("multi-port bitbang interleaves TIM8/TIM1 and dynamically allocates collision-free DMAMUX channels across DMA2/DMA1");
}
''')

run('dshot_bb_shared_pacer_bidir_sync', '''
#define USE_DSHOT_TELEMETRY
#define DEBUG_COUNT_INTERRUPT
#define DISABLE 0
#define ENABLE 1
#define DSHOT_BITBANG_DIRECTION_OUTPUT 0
#define DSHOT_BITBANG_DIRECTION_INPUT 1
#define DSHOT_BITBANG_STATUS_OK 0
#define DSHOT_BITBANG_STATUS_NO_PACER 2
#define DSHOT_BITBANG_STATUS_DMA_ERROR 4
#define DMA_IT_TCIF 0x02u
#define DMA_IT_HTIF 0x04u
#define DMA_IT_TEIF 0x08u
#define BB_DMA_FLAGS (DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF)
#define TIM_PSCReloadMode_Immediate 0x01u
#define TIM_FLAG_Update 0x01u
#define NVIC_PRIO_DSHOT_DMA 0xA0u
#define ATOMIC_BLOCK(x) for (int __once = (atomic_dshot_entries++, 1); __once; __once = 0)
#define MAX_SUPPORTED_MOTORS 4
#define DSHOT_TELEMETRY_NOEDGE 0xFFFEu
#define DSHOT_TELEMETRY_INVALID 0xFFFFu
#define DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE 13
#define DSHOT_TELEMETRY_TYPE_STATE_EVENTS 1
#define DSHOT_RAW_VALUE_STATE_NOT_PROCESSED 1
#define DEBUG_SET(a, b, c) ((void)0)
#define FAST_IRQ_HANDLER
#define FAST_CODE
#define dbgPinHi(x) ((void)0)
#define dbgPinLo(x) ((void)0)
typedef uint32_t timeUs_t;
typedef struct { uint32_t ATRLR, CNT, SWEVGR, INTFR, DMAINTENR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *tim; } timerHardware_t;
typedef struct { uint32_t CFGR, CNTR; } dmaResource_t;
typedef struct { TIM_TypeDef *tim; uint16_t dmaSources, armedOutputSources; volatile uint16_t pendingOutputSources, pendingInputSources; uint16_t outputARR, inputARR; } bbPacer_t;
typedef struct {
    bbPacer_t *pacer;
    timerHardware_t *timhw;
    dmaResource_t *dmaResource;
    uint16_t *portInputBuffer;
    uint16_t portInputCount;
    uint16_t dmaSource, outputARR, inputARR;
    uint8_t direction;
    volatile bool telemetryPending, reinitRequired;
    bool inputActive, telemetryAborted;
    uint32_t outputIrq, inputIrq, errorIrq;
} bbPort_t;
typedef struct { bbPort_t *bbPort; int pinIndex; int io, iocfg, output; bool configured, enabled; } bbMotor_t;
typedef struct { uintptr_t userParam; uint32_t flags; } dmaChannelDescriptor_t;
#define DMA_GET_FLAG_STATUS(d, f) ((d)->flags & (f))
#define DMA_CLEAR_FLAG(d, f) ((d)->flags &= ~(f))
static bbPacer_t bbPacers[1];
static bbPort_t bbPorts[2];
static bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];
static struct { uint32_t readCount, invalidPacketCount, rawValueState; struct { uint32_t rawValue, telemetryTypes; } motorState[MAX_SUPPORTED_MOTORS]; } dshotTelemetryState;
static int usedMotorPacers = 1, usedMotorPorts = 2, dshotMotorCount = 4, bbStatus, motorProtocol;
static bool useDshotTelemetry = true;
static unsigned atomic_dshot_entries, decodeCalls, switchOutCalls;
static bool failSwitchOutPort1;
static timeUs_t lastSendUs;
static timeUs_t micros(void) { return 100; }
static uint8_t dshotCommandGetCurrent(int idx) { (void)idx; return 0; }
static bool dshotCommandQueueEmpty(void) { return true; }
static bool dshotCommandOutputIsEnabled(int n) { (void)n; return true; }
static void bbDMA_Cmd(bbPort_t *p, int state) { p->dmaResource->CFGR = (uint32_t)state; }
''' + fn('src/platform/CH32/dshot_bitbang_stdperiph.c', 'void bbTIM_ReloadPeriod(').replace(
    'tim->SWEVGR = TIM_PSCReloadMode_Immediate;',
    'tim->SWEVGR = TIM_PSCReloadMode_Immediate; tim->CNT = 0;'
) + '''
static void bbTIM_DMACmd(void *TIMx, uint16_t src, int state) {
    TIM_TypeDef *tim = (TIM_TypeDef *)TIMx;
    if (state) tim->DMAINTENR |= src;
    else tim->DMAINTENR &= (uint16_t)~src;
}
static bool bbSwitchToOutput(bbPort_t *p) {
    switchOutCalls++;
    if (failSwitchOutPort1 && p == &bbPorts[1]) return false;
    p->timhw->tim->ATRLR = p->outputARR;
    p->dmaResource->CNTR = 51;
    p->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
    return true;
}
static void bbSwitchToInput(bbPort_t *p) {
    p->dmaResource->CNTR = 140;
    bbDMA_Cmd(p, ENABLE);
    p->direction = DSHOT_BITBANG_DIRECTION_INPUT;
}
static uint32_t decode_bb(uint16_t *buf, uint16_t cnt, int pin) {
    (void)buf; (void)cnt; (void)pin;
    decodeCalls++;
    return 0x0123u;
}
''' + fn('src/platform/common/stm32/dshot_bitbang_impl.h', 'static inline bool bbDMAServiceFlags(')
    + fn('src/platform/common/stm32/dshot_bitbang_impl.h', 'static inline bool bbDMAEventPending(')
    + fn('src/platform/CH32/dshot_bitbang.c', 'FAST_IRQ_HANDLER void bbDMAIrqHandler(')
    + fn('src/platform/CH32/dshot_bitbang.c', 'static bool bbDecodeTelemetry(void)')
    + fn('src/platform/CH32/dshot_bitbang.c', 'static void bbUpdateComplete(void)') + '''
int main(void) {
    uint16_t inBuf0[4] = {0}, inBuf1[4] = {0};
    TIM_TypeDef tim1 = { .ATRLR = 154, .CNT = 77 };
    timerHardware_t hw0 = { &tim1 }, hw1 = { &tim1 };
    dmaResource_t dma0 = {0}, dma1 = {0};
    bbPacers[0] = (bbPacer_t){ .tim = &tim1, .dmaSources = 0x06u };
    bbPorts[0] = (bbPort_t){ .pacer = &bbPacers[0], .timhw = &hw0, .dmaResource = &dma0, .portInputBuffer = inBuf0, .portInputCount = 4, .dmaSource = 0x02u, .outputARR = 193, .inputARR = 154, .direction = DSHOT_BITBANG_DIRECTION_INPUT };
    /* Port 1 starts in OUTPUT with CNTR == 0 ([B2]): bbUpdateComplete MUST still call bbSwitchToOutput and reload CNTR = 51! */
    bbPorts[1] = (bbPort_t){ .pacer = &bbPacers[0], .timhw = &hw1, .dmaResource = &dma1, .portInputBuffer = inBuf1, .portInputCount = 4, .dmaSource = 0x04u, .outputARR = 193, .inputARR = 154, .direction = DSHOT_BITBANG_DIRECTION_OUTPUT };
    bbMotors[0] = (bbMotor_t){ .bbPort = &bbPorts[0], .pinIndex = 0, .configured = true };
    bbMotors[1] = (bbMotor_t){ .bbPort = &bbPorts[1], .pinIndex = 1, .configured = true };
    bbMotors[2] = (bbMotor_t){ .configured = false, .bbPort = NULL }; /* unconfigured motor must be skipped safely */

    /* 1. Normal frame: output reload happens via real bbTIM_ReloadPeriod, CNTR reloaded to 51 on both ports, ATOMIC_BLOCK entered */
    bbUpdateComplete();
    if (tim1.ATRLR != 193 || tim1.CNT != 0 || tim1.DMAINTENR != 0x06u || dma0.CNTR != 51 || dma1.CNTR != 51 || atomic_dshot_entries != 1) return 1;
    tim1.CNT = 120;

    /* Port 0 finishes output first: must NOT reset CNT or reload inputARR while Port 1 is still outputting */
    dmaChannelDescriptor_t desc0 = { .userParam = (uintptr_t)&bbPorts[0], .flags = DMA_IT_TCIF };
    bbDMAIrqHandler(&desc0);
    if (tim1.CNT != 120 || tim1.ATRLR != 193 || tim1.DMAINTENR != 0x04u) return 2;
    if (!bbPorts[0].telemetryPending || bbPorts[0].direction != DSHOT_BITBANG_DIRECTION_INPUT || dma0.CFGR != 1) return 3;

    /* Port 1 finishes output: single synchronized UG switches ATRLR -> 154, CNT -> 0, and enables CC1|CC2 input */
    dmaChannelDescriptor_t desc1 = { .userParam = (uintptr_t)&bbPorts[1], .flags = DMA_IT_TCIF };
    bbDMAIrqHandler(&desc1);
    if (tim1.CNT != 0 || tim1.ATRLR != 154 || tim1.DMAINTENR != 0x06u) return 4;

    /* Both ports complete input reception cleanly -> 2 configured motors decoded */
    desc0.flags = DMA_IT_TCIF; bbDMAIrqHandler(&desc0);
    desc1.flags = DMA_IT_TCIF; bbDMAIrqHandler(&desc1);
    if (bbPorts[0].telemetryPending || bbPorts[1].telemetryPending || tim1.DMAINTENR != 0) return 5;
    bbDecodeTelemetry();
    if (decodeCalls != 2 || dshotTelemetryState.readCount != 2) return 6;

    /* 2. Fault isolation ([B3]): if Port 1 hits TEIF during output, Port 0 still receives input clocks AND Port 1 sets telemetryAborted so stale GCR is NOT re-decoded */
    decodeCalls = 0;
    bbUpdateComplete();
    desc0.flags = DMA_IT_TCIF; bbDMAIrqHandler(&desc0);
    desc1.flags = DMA_IT_TEIF; bbDMAIrqHandler(&desc1);
    if (bbStatus != DSHOT_BITBANG_STATUS_DMA_ERROR || !bbPorts[1].reinitRequired || !bbPorts[1].telemetryAborted || tim1.ATRLR != 154 || tim1.DMAINTENR != 0x02u) return 7;
    desc0.flags = DMA_IT_TCIF; bbDMAIrqHandler(&desc0);
    bbDecodeTelemetry();
    if (decodeCalls != 1) return 8; /* Only Port 0 decoded; faulted Port 1 skipped! */

    /* 3. Switch-to-output failure on Port 1 sets telemetryAborted too ([B3]) and keeps DMA_ERROR active */
    decodeCalls = 0;
    failSwitchOutPort1 = true;
    bbUpdateComplete();
    if (bbStatus != DSHOT_BITBANG_STATUS_DMA_ERROR || !bbPorts[1].telemetryAborted || bbPacers[0].armedOutputSources != 0x02u) return 9;
    bbDecodeTelemetry();
    if (decodeCalls != 1) return 10;

    /* 4. Once all ports recover to output, transient DMA_ERROR clears to OK, while non-DMA init errors stay sticky */
    failSwitchOutPort1 = false;
    bbUpdateComplete();
    if (bbStatus != DSHOT_BITBANG_STATUS_OK || bbPorts[1].reinitRequired) return 11;
    bbStatus = DSHOT_BITBANG_STATUS_NO_PACER;
    bbUpdateComplete();
    if (bbStatus != DSHOT_BITBANG_STATUS_NO_PACER) return 12;
    puts("shared pacer synchronizes ARPE shadow reload, reloads CNTR in OUTPUT state, blocks stale GCR decode on TEIF/abort, and clears transient DMA_ERROR after full recovery");
}
''')

run('dshot_bb_postinit_fail_safe', '''
#define MAX_SUPPORTED_MOTORS 4
#define DSHOT_BITBANG_STATUS_OK 0
#define DSHOT_BITBANG_STATUS_NO_PACER 2
#define OWNER_FREE 0
#define OWNER_DSHOT_BITBANG 9
#define DISABLE 0
typedef int IO_t;
typedef struct { void *tim; uint16_t dmaSources; } bbPacer_t;
typedef struct { struct { int owner; } resourceOwner; int dmaStopped; } bbPort_t;
typedef struct { IO_t io; int output, iocfg; bbPort_t *bbPort; bool configured, enabled; } bbMotor_t;
static bbPacer_t bbPacers[2];
static bbPort_t bbPorts[2];
static bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];
static int usedMotorPacers, usedMotorPorts, dshotMotorCount = 4, bbStatus, motorProtocol, gpioConfigs;
static void bbTIM_DMACmd(void *tim, uint16_t src, int state) { (void)tim; (void)src; if (!state) bbPacers[0].dmaSources = 0; }
static void bbDMA_Cmd(bbPort_t *p, int state) { if (!state) p->dmaStopped = 1; }
static void IOConfigGPIO(IO_t io, int cfg) { (void)io; (void)cfg; gpioConfigs++; }
static void bbFindPacerTimer(void) {
    bbStatus = DSHOT_BITBANG_STATUS_OK;
    usedMotorPacers = 1;
    bbPacers[0].dmaSources = 0x02u;
}
static bool bbMotorConfig(IO_t io, uint8_t idx, int proto, uint8_t out) {
    (void)io; (void)proto; (void)out;
    if (idx == 0) {
        usedMotorPorts = 1;
        bbPorts[0].resourceOwner.owner = OWNER_DSHOT_BITBANG;
        bbMotors[0].bbPort = &bbPorts[0];
        bbMotors[0].configured = true;
        return true;
    }
    bbStatus = DSHOT_BITBANG_STATUS_NO_PACER;
    return false;
}
''' + fn('src/platform/CH32/dshot_bitbang.c', 'static bool bbEnableMotors(void)')
    + fn('src/platform/CH32/dshot_bitbang.c', 'static void bbPostInit(void)') + '''
int main(void) {
    bbPostInit();
    if (bbStatus != DSHOT_BITBANG_STATUS_NO_PACER) return 1;
    if (usedMotorPorts != 0 || usedMotorPacers != 0) return 2;
    if (!bbPorts[0].dmaStopped || bbPorts[0].resourceOwner.owner != OWNER_FREE || bbPacers[0].dmaSources != 0) return 3;
    if (bbMotors[0].configured || bbMotors[0].enabled || bbMotors[0].bbPort != NULL) return 4;
    if (bbEnableMotors() || gpioConfigs != 0) return 5;
    puts("bbPostInit tears down partial ports on pacer exhaustion and bbEnableMotors refuses partial/failed enable");
}
''')

run('usb_vcp_chained_16k_throughput', '''
#define UNUSED(x) ((void)(x))
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once = 1; once; once = 0)
#define USB_TIMEOUT 50
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define TX_BUFFER_SIZE 512
#define CDC_IN_EP 0x82
#define CDC_MAX_MPS 64
#define USB_VCP_BULK_IN_MPS CDC_MAX_MPS
typedef uint32_t timeUs_t;
typedef int serialPort_t;
static struct { uint32_t txAt; } vcpPort;
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static volatile uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static volatile uint8_t ep_tx_busy_flag;
static bool usbHostPortOpen = true, usbZlpPending, usbTxStalled;
static uint8_t write_buffer[TX_BUFFER_SIZE];
static uint8_t hostRx[16384];
static uint32_t hostRxLen;
static bool epPending;
static uint32_t epPendingLen, zlpCount, chunkCount, nowUs;
static timeUs_t micros(void) { return nowUs; }
static uint8_t usbIsConfigured(void) { return 1; }
static uint8_t usbVcpIsConnected(void) { return 1; }
void usbVcpTxDoneISR(void);
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes);
uint16_t usb_vcp_send_data(uint8_t busid, uint8_t *buffer, uint16_t len);
static int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len) {
    (void)busid;
    if (ep != CDC_IN_EP || epPending) return -1;
    if (data_len > 0) {
        if (hostRxLen + data_len > sizeof(hostRx)) return -2;
        memcpy(&hostRx[hostRxLen], data, data_len);
        hostRxLen += data_len;
        chunkCount++;
    } else {
        zlpCount++;
    }
    epPending = true;
    epPendingLen = data_len;
    return 0;
}
static void pumpUsbInCompletions(void) {
    while (epPending) {
        uint32_t n = epPendingLen;
        epPending = false;
        epPendingLen = 0;
        usbd_cdc_acm_bulk_in(0, CDC_IN_EP, n);
    }
}
static void delay(int ms) {
    nowUs += (uint32_t)ms * 1000u;
    pumpUsbInCompletions();
}
static void schedulerIgnoreTaskExecTime(void) {}
''' + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'void usbd_cdc_acm_bulk_in(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_send_data(')
    + fn(usb, 'static void usbVcpKickTx(void)')
    + fn(usb, 'void usbVcpTxDoneISR(void)')
    + fn(usb, 'uint32_t CDC_Send_FreeBytes(void)')
    + fn(usb, 'uint32_t CDC_Send_DATA(')
    + fn(usb, 'static bool isUsbVcpTransmitBufferEmpty(') + '''
int main(void) {
    uint8_t src[16384];
    for (uint32_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)((i * 131u + 17u) & 0xffu);
    }
    /* Stream four 4096B frames back-to-back purely via bulk-IN completion chaining (zero SOF ticks) */
    for (uint32_t frame = 0; frame < 4; frame++) {
        if (CDC_Send_DATA(&src[frame * 4096], 4096) != 4096) return 1;
    }
    pumpUsbInCompletions();
    if (hostRxLen != sizeof(src) || memcmp(src, hostRx, sizeof(src)) != 0) return 2;
    /* 16KB across an 8191B ring ends on a natural short packet -> 0 mid-stream ZLPs wasted */
    if (chunkCount < 32 || zlpCount != 0 || !isUsbVcpTransmitBufferEmpty(NULL)) return 3;
    /* Exact 512B burst ending at empty queue -> exactly 1 terminating ZLP */
    hostRxLen = 0;
    if (CDC_Send_DATA(src, 512) != 512) return 4;
    pumpUsbInCompletions();
    if (hostRxLen != 512 || zlpCount != 1 || !isUsbVcpTransmitBufferEmpty(NULL)) return 5;
    puts("16KB streamed intact with zero mid-stream ZLPs and exact end-of-burst ZLP termination");
}
''')

run('usb_vcp_ep_start_write_fail_recovery', '''
#define UNUSED(x) ((void)(x))
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once = 1; once; once = 0)
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define TX_BUFFER_SIZE 512
#define CDC_IN_EP 0x82
#define CDC_MAX_MPS 64
#define USB_VCP_BULK_IN_MPS CDC_MAX_MPS
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static volatile uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static volatile uint8_t ep_tx_busy_flag;
static volatile bool usbZlpPending, usbTxStalled;
static uint8_t write_buffer[TX_BUFFER_SIZE];
static bool epEnabled;
static uint8_t delivered[128];
static uint32_t deliveredLen;
void usbVcpTxDoneISR(void);
uint16_t usb_vcp_send_data(uint8_t busid, uint8_t *buffer, uint16_t len);
static uint8_t usbIsConfigured(void) { return 1; }
static int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len) {
    (void)busid; (void)ep;
    if (!epEnabled) return -2;
    memcpy(delivered, data, data_len);
    deliveredLen = data_len;
    return 0;
}
''' + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'void usbd_cdc_acm_bulk_in(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_send_data(')
    + fn(usb, 'static void usbVcpKickTx(void)')
    + fn(usb, 'void usbVcpTxDoneISR(void)')
    + fn(usb, 'void usb_int_rxsof_handler(void)') + '''
int main(void) {
    for (uint32_t i = 0; i < 37; i++) UserTxBuffer[i] = (uint8_t)(0xa0u + i);
    UserTxBufPtrIn = 37;
    /* Endpoint disabled: usbd_ep_start_write returns -2; busy flag MUST roll back to 0 */
    usb_int_rxsof_handler();
    if (ep_tx_busy_flag != 0 || lastBuffsize != 0 || UserTxBufPtrOut != 0) return 1;
    /* Oversized length guard */
    if (usb_vcp_send_data(0, (uint8_t *)UserTxBuffer, TX_BUFFER_SIZE + 1) != 1 || ep_tx_busy_flag != 0) return 2;
    /* Endpoint becomes enabled: SOF recovery immediately delivers intact payload */
    epEnabled = true;
    usb_int_rxsof_handler();
    if (ep_tx_busy_flag != 1 || lastBuffsize != 37 || deliveredLen != 37) return 3;
    usbd_cdc_acm_bulk_in(0, CDC_IN_EP, 37);
    if (ep_tx_busy_flag != 0 || lastBuffsize != 0 || UserTxBufPtrOut != 37) return 4;
    for (uint32_t i = 0; i < 37; i++) if (delivered[i] != (uint8_t)(0xa0u + i)) return 5;
    puts("usbd_ep_start_write failure rolls back ep_tx_busy_flag and recovers cleanly on next SOF");
}
''')

run('usb_hs_pll_hse_mapping', '''
#define RCC_USBHSPLLRefer_25M 0x00u
#define RCC_USBHSPLLRefer_20M 0x01u
#define RCC_USBHSPLLRefer_24M 0x02u
#define RCC_USBHSPLLRefer_32M 0x03u
''' + fn('src/platform/CH32/usb_ch32h41x.h', 'static inline bool ch32UsbHsResolvePllConfig(') + '''
int main(void) {
    uint32_t ref = 0, div = 0;
    if (!ch32UsbHsResolvePllConfig(25000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_25M || div != 0x0000u) return 1;
    if (!ch32UsbHsResolvePllConfig(20000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_20M || div != 0x0000u) return 2;
    if (!ch32UsbHsResolvePllConfig(24000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_24M || div != 0x0000u) return 3;
    if (!ch32UsbHsResolvePllConfig(32000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_32M || div != 0x0000u) return 4;
    if (!ch32UsbHsResolvePllConfig(48000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_24M || div != 0x0100u) return 5;
    if (!ch32UsbHsResolvePllConfig(50000000u, &ref, &div) || ref != RCC_USBHSPLLRefer_25M || div != 0x0100u) return 6;
    if (ch32UsbHsResolvePllConfig(0u, &ref, &div)) return 7;
    if (ch32UsbHsResolvePllConfig(13000000u, &ref, &div)) return 8;
    if (ch32UsbHsResolvePllConfig(25000000u * 33u, &ref, &div)) return 9;
    puts("USBHS PLL resolver maps 20/24/25/32/48/50 MHz crystals and rejects unsupported frequencies");
}
''')

usb_cfg_text = (root / 'lib/modules/CH32H41x/middlewares/board/usb_config.h').read_text()
if '\n#define CONFIG_USBDEV_MSC_POLLING\n' not in usb_cfg_text:
    raise RuntimeError('usb_config.h must enable CONFIG_USBDEV_MSC_POLLING')
usbd_msc_body = ('#define MSD_OUT_EP_IDX 0' + (root / 'lib/modules/CH32H41x/middlewares/class/msc/usbd_msc.c').read_text().split('#define MSD_OUT_EP_IDX 0', 1)[1]).split('__WEAK void usbd_msc_get_cap', 1)[0]
run('usb_msc_polling_and_readonly', '''
#define UNUSED(x) ((void)(x))
#define __PACKED __attribute__((packed))
#define __WEAK __attribute__((weak))
#define USB_NOCACHE_RAM_SECTION
#define USB_MEM_ALIGNX __attribute__((aligned(4)))
#define CONFIG_USBDEV_MAX_BUS 1
#define CONFIG_USBDEV_MSC_MAX_LUN 1
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 4096
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING "BETAFLT"
#define CONFIG_USBDEV_MSC_PRODUCT_STRING "Onboard Flash"
#define CONFIG_USBDEV_MSC_VERSION_STRING "    "
#define CONFIG_USBDEV_MSC_POLLING
#define USB_LOG_DBG(...) ((void)0)
#define USB_LOG_WRN(...) ((void)0)
#define USB_LOG_ERR(...) ((void)0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define GET_BE16(p) ((uint16_t)(((uint16_t)(p)[0] << 8) | (uint16_t)(p)[1]))
#define GET_BE32(p) (((uint32_t)(p)[0] << 24) | ((uint32_t)(p)[1] << 16) | ((uint32_t)(p)[2] << 8) | (uint32_t)(p)[3])
enum { USBD_EVENT_RESET = 1, USBD_EVENT_INIT = 2, USBD_EVENT_DEINIT = 3, USBD_EVENT_CONFIGURED = 4, USBD_EVENT_CLEAR_HALT = 5 };
struct usb_setup_packet { uint8_t bmRequestType, bRequest; uint16_t wValue, wIndex, wLength; };
struct usbd_endpoint { uint8_t ep_addr; void (*ep_cb)(uint8_t busid, uint8_t ep, uint32_t nbytes); };
struct usbd_interface {
    int (*class_interface_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*class_endpoint_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*vendor_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    void (*notify_handler)(uint8_t busid, uint8_t event, void *arg);
};
#include "usb_msc.h"
#include "usb_scsi.h"
#include "platform/usb_msc_storage.h"
static uint8_t *lastReadBuf;
static uint32_t lastReadMaxLen;
static uint8_t lastInBuf[4096];
static uint32_t lastInLen;
static unsigned stallCalls;
static void usbd_add_endpoint(uint8_t busid, struct usbd_endpoint *ep) { (void)busid; (void)ep; }
static int usbd_ep_set_stall(uint8_t busid, uint8_t ep) { (void)busid; (void)ep; stallCalls++; return 0; }
static int usbd_ep_start_read(uint8_t busid, uint8_t ep, uint8_t *data, uint32_t len) {
    (void)busid; (void)ep; lastReadBuf = data; lastReadMaxLen = len; return 0;
}
static int usbd_ep_start_write(uint8_t busid, uint8_t ep, const uint8_t *data, uint32_t len) {
    (void)busid; (void)ep;
    if (len > sizeof(lastInBuf)) abort();
    if (len > 0) memcpy(lastInBuf, data, len);
    lastInLen = len;
    return 0;
}
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size);
bool usbd_msc_get_inquiry(uint8_t busid, uint8_t lun, uint8_t *inquiry_buf);
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);
void usbd_msc_polling(uint8_t busid);
void usbd_msc_set_readonly(uint8_t busid, bool readonly);
struct usbd_interface *usbd_msc_init_intf(uint8_t busid, struct usbd_interface *intf, const uint8_t out_ep, const uint8_t in_ep);
''' + usbd_msc_body + '''
static USBD_MSC_StorageType *USBD_STORAGE_fops;
static unsigned readCalls, writeCalls, activeCalls;
static uint8_t fakeDisk[16 * 512];
static void mscSetActive(void) { activeCalls++; }
static int8_t mockGetCap(uint8_t lun, uint32_t *num, usbd_msc_block_size_t *sz) { (void)lun; *num = 16; *sz = 512; return 0; }
static int8_t mockIsWp(uint8_t lun) { (void)lun; return 1; }
static int8_t mockRead(uint8_t lun, uint8_t *buf, uint32_t addr, uint16_t len) {
    (void)lun; readCalls++;
    memcpy(buf, &fakeDisk[addr * 512u], (size_t)len * 512u);
    return 0;
}
static int8_t mockWrite(uint8_t lun, uint8_t *buf, uint32_t addr, uint16_t len) {
    (void)lun; (void)buf; (void)addr; (void)len; writeCalls++; return 0;
}
static USBD_MSC_StorageType mockRoFops = {
    .GetCapacity = mockGetCap, .IsWriteProtected = mockIsWp, .Read = mockRead, .Write = mockWrite
};
''' + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'void usbd_msc_get_cap(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'bool usbd_msc_get_inquiry(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'int usbd_msc_sector_read(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'int usbd_msc_sector_write(') + '''
static void sendCbw(uint32_t tag, uint32_t dataLen, uint8_t flags, const uint8_t *cdb, uint8_t cdbLen) {
    struct CBW cbw = { .dSignature = MSC_CBW_Signature, .dTag = tag, .dDataLength = dataLen, .bmFlags = flags, .bLUN = 0, .bCBLength = cdbLen };
    memcpy(cbw.CB, cdb, cdbLen);
    memcpy(lastReadBuf, &cbw, sizeof(cbw));
    mass_storage_bulk_out(0, 0x01, sizeof(cbw));
}
int main(void) {
    for (size_t i = 0; i < sizeof(fakeDisk); i++) fakeDisk[i] = (uint8_t)((i * 37u + 9u) & 0xffu);
    USBD_STORAGE_fops = &mockRoFops;
    struct usbd_interface intf = {0};
    usbd_msc_init_intf(0, &intf, 0x01, 0x81);
    usbd_msc_set_readonly(0, mockRoFops.IsWriteProtected(0) != 0);

    /* 1. USB Bus Reset + MSC Class Reset MUST preserve readonly = true */
    msc_storage_notify_handler(0, USBD_EVENT_RESET, NULL);
    struct usb_setup_packet rstReq = { .bRequest = MSC_REQUEST_RESET };
    uint8_t dummy = 0, *dp = &dummy; uint32_t dl = 0;
    if (intf.class_interface_handler(0, &rstReq, &dp, &dl) != 0 || !g_usbd_msc[0].readonly) return 1;
    msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);

    /* 2. MODE SENSE(6) and MODE SENSE(10) both report WP bit 0x80 */
    const uint8_t ms6[6] = { SCSI_CMD_MODESENSE6, 0, 0x3f, 0, 4, 0 };
    sendCbw(1, 4, 0x80, ms6, 6);
    if (lastInLen != 4 || (lastInBuf[2] & 0x80u) != 0x80u) return 2;
    mass_storage_bulk_in(0, 0x81, 4);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    const uint8_t ms10[10] = { SCSI_CMD_MODESENSE10, 0, 0x3f, 0, 0, 0, 0, 0, 27, 0 };
    sendCbw(2, 27, 0x80, ms10, 10);
    if (lastInLen != 27 || (lastInBuf[3] & 0x80u) != 0x80u) return 3;
    mass_storage_bulk_in(0, 0x81, 27);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    /* 3. READ(10) defers Flash read out of ISR into usbd_msc_polling(0) */
    const uint8_t rd10[10] = { SCSI_CMD_READ10, 0, 0, 0, 0, 2, 0, 0, 2, 0 };
    sendCbw(3, 1024, 0x80, rd10, 10);
    if (readCalls != 0) return 4;
    usbd_msc_polling(0);
    if (readCalls != 1 || activeCalls != 1 || lastInLen != 1024 || memcmp(lastInBuf, &fakeDisk[2 * 512], 1024) != 0) return 5;
    mass_storage_bulk_in(0, 0x81, 1024);
    struct CSW csw = {0};
    memcpy(&csw, lastInBuf, sizeof(csw));
    if (lastInLen != sizeof(csw) || csw.bStatus != CSW_STATUS_CMD_PASSED || csw.dTag != 3) return 6;
    mass_storage_bulk_in(0, 0x81, sizeof(csw));

    /* 4. WRITE(10) on read-only volume drains OUT data without touching storage and reports KCQ 0x072700 */
    const uint8_t wr10[10] = { SCSI_CMD_WRITE10, 0, 0, 0, 0, 2, 0, 0, 2, 0 };
    sendCbw(4, 1024, 0x00, wr10, 10);
    if (lastReadMaxLen != 1024) return 7;
    memset(lastReadBuf, 0x55, 1024);
    mass_storage_bulk_out(0, 0x01, 1024);
    if (writeCalls != 0) return 8;
    usbd_msc_polling(0);
    memcpy(&csw, lastInBuf, sizeof(csw));
    if (writeCalls != 0 || stallCalls != 0 || lastInLen != sizeof(csw) || csw.bStatus != CSW_STATUS_CMD_FAILED || csw.dDataResidue != 1024) return 9;
    mass_storage_bulk_in(0, 0x81, sizeof(csw));

    const uint8_t reqSense[6] = { SCSI_CMD_REQUESTSENSE, 0, 0, 0, 18, 0 };
    sendCbw(5, 18, 0x80, reqSense, 6);
    if (lastInLen != 18 || lastInBuf[2] != 0x07u || lastInBuf[12] != 0x27u || lastInBuf[13] != 0x00u) return 10;
    puts("MSC polling defers storage I/O out of ISR and preserves WP across USB reset, MODE SENSE 6/10, and WRITE(10)");
}
''', extra_flags=('-I' + str(root / 'lib/modules/CH32H41x/middlewares/class/msc'), '-I' + str(root / 'src/platform/CH32/include')))

usbhs_reg_body = ('#define USB_BASE 0x40030000u' + (root / 'lib/modules/CH32H41x/middlewares/ch32h41x_hs/usb_ch32h41x_usbhs_reg.c').read_text().split('#define USB_BASE 0x40030000u', 1)[1]).split('void USBHS_IRQHandler(void)', 1)[0]
run('usb_hs_ep_done_clear_order', '''
#define _GNU_SOURCE
#include <sys/mman.h>
#define __IO volatile
#define __WEAK __attribute__((weak))
#define UNUSED(x) ((void)(x))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CONFIG_USB_HS
#define USB_LOG_ERR(...) ((void)0)
#define USB_SPEED_FULL 1
#define USB_SPEED_HIGH 2
#define USB_ENDPOINT_TYPE_BULK 2
#define USB_ENDPOINT_TYPE_ISOCHRONOUS 1
#define USB_REQUEST_DIR_SHIFT 7
#define USB_REQUEST_SET_ADDRESS 5
#define USB_EP_GET_IDX(ep) ((uint8_t)((ep) & 0x7fu))
#define USB_EP_DIR_IS_IN(ep) (((ep) & 0x80u) != 0)
#define USB_EP_DIR_IS_OUT(ep) (((ep) & 0x80u) == 0)
#define USB_GET_MAXPACKETSIZE(w) ((uint16_t)((w) & 0x07ffu))
#define USB_GET_ENDPOINT_TYPE(a) ((uint8_t)((a) & 0x03u))
typedef void (*usb_rxsof_handler_t)(void);
struct usb_setup_packet { uint8_t bmRequestType, bRequest; uint16_t wValue, wIndex, wLength; };
struct usb_endpoint_descriptor { uint8_t bLength, bDescriptorType, bEndpointAddress, bmAttributes; uint16_t wMaxPacketSize; uint8_t bInterval; };
#include "usb_ch32h41x_usbhs_reg.h"
static unsigned inCompleteCalls, outCompleteCalls, suspendCalls, resumeCalls, sofCalls;
static uint8_t *nextBuf;
int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t data_len);
int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t data_len);
static void usbd_event_reset_handler(uint8_t busid) { (void)busid; }
static void usbd_event_suspend_handler(uint8_t busid) { (void)busid; suspendCalls++; }
static void usbd_event_resume_handler(uint8_t busid) { (void)busid; resumeCalls++; }
static void usbd_event_ep0_setup_complete_handler(uint8_t busid, uint8_t *s) { (void)busid; (void)s; }
static void usbd_event_ep_in_complete_handler(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    (void)busid; (void)nbytes;
    inCompleteCalls++;
    if (inCompleteCalls == 1) {
        /* Chain next packet AND simulate preempting FC ISR delaying return while new packet finishes on wire */
        usbd_ep_start_write(0, ep, nextBuf, 0);
        ((USBHSD_TypeDef *)0x40030000u)->UEP2_TX_CTRL |= USBHS_UEP_T_DONE;
    }
}
static void usbd_event_ep_out_complete_handler(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    (void)busid; (void)nbytes;
    outCompleteCalls++;
    if (outCompleteCalls == 1) {
        /* Re-arm RX AND simulate next OUT packet arriving on wire before usb_process_ep_out returns */
        usbd_ep_start_read(0, ep, nextBuf, 512);
        ((USBHSD_TypeDef *)0x40030000u)->UEP1_RX_LEN = 31;
        ((USBHSD_TypeDef *)0x40030000u)->UEP1_RX_CTRL |= USBHS_UEP_R_DONE | USBHS_UEP_R_TOG_MATCH;
    }
}
static void mockSof(void) { sofCalls++; }
''' + usbhs_reg_body + '''
int main(void) {
    uint8_t *page = mmap((void *)0x40030000u, 8192, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (page == MAP_FAILED) return 1;
    nextBuf = page + 4096;
    USBHSD_TypeDef *dev = (USBHSD_TypeDef *)page;

    /* 1. Conditional SOF IRQ enable: MSC (NULL handler) disables SOF_ACT; VCP enables SOF_ACT */
    usb_rxsof_handler = NULL;
    usb_dc_init(0);
    if ((dev->INT_EN & USBHS_UDIE_SOF_ACT) != 0) return 2;
    usb_rxsof_handler = mockSof;
    usb_dc_init(0);
    if ((dev->INT_EN & USBHS_UDIE_SOF_ACT) == 0) return 3;

    /* 2. High-Speed 512B MPS MUST NOT truncate through uint8_t to 0 */
    struct usb_endpoint_descriptor epIn = { .bEndpointAddress = 0x82, .bmAttributes = USB_ENDPOINT_TYPE_BULK, .wMaxPacketSize = 512 };
    struct usb_endpoint_descriptor epOut = { .bEndpointAddress = 0x01, .bmAttributes = USB_ENDPOINT_TYPE_BULK, .wMaxPacketSize = 512 };
    usbd_ep_open(0, &epIn);
    usbd_ep_open(0, &epOut);
    if (usb_dc_cfg.ep_in[2].mps != 512 || usb_dc_cfg.ep_out[1].mps != 512) return 4;

    /* 3. [BUG-1] Bulk-IN: T_DONE set for chained packet before ISR exit MUST NOT be cleared */
    usbd_ep_start_write(0, 0x82, nextBuf, 512);
    dev->UEP2_TX_CTRL |= USBHS_UEP_T_DONE;
    dev->INT_FG = USBHS_UDIF_TRANSFER;
    dev->INT_ST = USBHS_UDIS_EP_DIR | 2u;
    USBD_IRQHandler(0);
    if (inCompleteCalls != 1 || (dev->UEP2_TX_CTRL & USBHS_UEP_T_DONE) == 0) return 5;
    USBD_IRQHandler(0);
    if (inCompleteCalls != 2 || (dev->UEP2_TX_CTRL & USBHS_UEP_T_DONE) != 0) return 6;

    /* 4. [BUG-1] Bulk-OUT: R_DONE set for re-armed OUT packet before ISR exit MUST NOT be cleared */
    usbd_ep_start_read(0, 0x01, nextBuf, 512);
    dev->UEP1_RX_LEN = 31;
    dev->UEP1_RX_CTRL |= USBHS_UEP_R_DONE | USBHS_UEP_R_TOG_MATCH;
    dev->INT_FG = USBHS_UDIF_TRANSFER;
    dev->INT_ST = 1u;
    USBD_IRQHandler(0);
    if (outCompleteCalls != 1 || (dev->UEP1_RX_CTRL & USBHS_UEP_R_DONE) == 0) return 7;
    USBD_IRQHandler(0);
    if (outCompleteCalls != 2 || (dev->UEP1_RX_CTRL & USBHS_UEP_R_DONE) != 0) return 8;

    /* 5. [BUG-2] USBHS_UDIF_SUSPEND dispatches suspend / resume callbacks cleanly */
    dev->INT_FG = USBHS_UDIF_SUSPEND;
    dev->MIS_ST = USBHS_UDMS_SUSPEND;
    USBD_IRQHandler(0);
    dev->INT_FG = USBHS_UDIF_SUSPEND;
    dev->MIS_ST = 0;
    USBD_IRQHandler(0);
    if (suspendCalls != 1 || resumeCalls != 1) return 9;
    puts("USBHS clears T_DONE/R_DONE before re-arming, preserves 512B HS MPS, and dispatches suspend/resume");
}
''', extra_flags=('-I' + str(root / 'lib/modules/CH32H41x/middlewares/ch32h41x_hs'),))

run('usb_vcp_suspend_and_stall_latch', '''
#define UNUSED(x) ((void)(x))
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once = 1; once; once = 0)
#define USB_TIMEOUT 50
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define RX_BUFFER_SIZE 2048
#define TX_BUFFER_SIZE 512
#define CDC_IN_EP 0x82
#define CDC_OUT_EP 0x02
#define CDC_MAX_MPS 512
#define USB_VCP_BULK_IN_MPS CDC_MAX_MPS
#define USB_VCP_OUT_MPS(busid) ((void)(busid), (uint16_t)CDC_MAX_MPS)
#define CTRL_LINE_STATE_DTR 1
enum {
    USBD_EVENT_RESET = 1,
    USBD_EVENT_CONNECTED = 3,
    USBD_EVENT_DISCONNECTED = 4,
    USBD_EVENT_SUSPEND = 5,
    USBD_EVENT_RESUME = 6,
    USBD_EVENT_CONFIGURED = 7,
    USBD_EVENT_SET_REMOTE_WAKEUP = 9,
    USBD_EVENT_CLR_REMOTE_WAKEUP = 10,
    USBD_EVENT_INIT = 11,
    USBD_EVENT_DEINIT = 12,
    USBD_EVENT_UNKNOWN = 14,
};
typedef uint32_t timeUs_t;
typedef int serialPort_t;
static struct { uint32_t txAt; } vcpPort;
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static volatile uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static volatile bool usbZlpPending, usbTxStalled, usbHostPortOpen;
static volatile uint8_t ep_tx_busy_flag, usbd_cdc_info = USBD_EVENT_UNKNOWN, cdc_configured, cdc_connected, rx_ep_paused;
static volatile uint16_t rx_head, rx_tail;
static uint8_t usb_drivers_buffer[CDC_MAX_MPS], write_buffer[TX_BUFFER_SIZE];
static void (*ctrlLineStateCb)(void *, uint16_t);
static void *ctrlLineStateCbContext;
static uint32_t nowMs, zlpSent, dataChunksSent, failNextZlp;
static timeUs_t micros(void) { return nowMs * 1000u; }
static uint32_t millis(void) { return nowMs; }
static void delay(int ms) { nowMs += (uint32_t)ms; }
static void schedulerIgnoreTaskExecTime(void) {}
void usbVcpResetState(void);
void usbVcpTxDoneISR(void);
uint16_t usb_vcp_send_data(uint8_t busid, uint8_t *buffer, uint16_t len);
static int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t len) { (void)busid; (void)ep; (void)data; (void)len; return 0; }
static int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t len) {
    (void)busid; (void)ep; (void)data;
    if (len == 0 && failNextZlp) { failNextZlp = 0; return -1; }
    if (len == 0) zlpSent++; else dataChunksSent++;
    return 0;
}
''' + fn(usb, 'void usbVcpControlLineStateChanged(')
    + fn(usb, 'uint8_t usbIsConnected(void)')
    + fn(usb, 'uint8_t usbIsConfigured(void)')
    + fn(usb, 'uint8_t usbVcpIsConnected(void)')
    + fn(usb, 'uint8_t usbVcpIsActive(void)')
    + fn(usb, 'void usbVcpResetState(void)')
    + fn(usb, 'static void usbVcpKickTx(void)')
    + fn(usb, 'void usbVcpTxDoneISR(void)')
    + fn(usb, 'uint32_t CDC_Send_FreeBytes(void)')
    + fn(usb, 'uint32_t CDC_Send_DATA(')
    + fn(usb, 'static void usbVcpWriteBuf(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static void usbd_event_handler(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'void usbd_cdc_acm_bulk_in(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_send_data(') + '''
int main(void) {
    static uint8_t payload[APP_TX_DATA_SIZE];
    /* 1. Resume before CONFIGURED must NOT forge CONFIGURED */
    usbd_event_handler(0, USBD_EVENT_RESUME);
    if (usbIsConfigured()) return 1;

    /* 2. CONFIGURED -> SUSPEND clears usbVcpIsActive() so writes take 0 ms; RESUME restores CONFIGURED */
    usbd_event_handler(0, USBD_EVENT_CONFIGURED);
    if (!usbVcpIsActive()) return 2;
    usbd_event_handler(0, USBD_EVENT_SUSPEND);
    if (usbVcpIsActive()) return 3;
    usbVcpWriteBuf(NULL, payload, sizeof(payload));
    if (nowMs != 0) return 4;
    usbd_event_handler(0, USBD_EVENT_RESUME);
    if (!usbVcpIsActive()) return 5;

    /* 3. Host not reading Bulk-IN: 1st overflow waits 50 ms & latches usbTxStalled; 2nd+ writes return in 0 ms! */
    usbVcpWriteBuf(NULL, payload, sizeof(payload));
    if (nowMs < 50 || nowMs > 60 || !usbTxStalled) return 6;
    uint32_t tBefore = nowMs;
    for (int i = 0; i < 20; i++) {
        usbVcpWriteBuf(NULL, payload, 256);
    }
    if (nowMs != tBefore) return 7;

    /* 4. Host starts reading -> usbTxStalled clears immediately; continuous 1024B burst sends ZERO mid-burst ZLPs */
    usbd_event_handler(0, USBD_EVENT_CONFIGURED);
    zlpSent = 0; dataChunksSent = 0;
    if (CDC_Send_DATA(payload, 1024) != 1024) return 8;
    if (dataChunksSent != 1 || zlpSent != 0) return 9;
    /* Complete 1st 512B chunk while 2nd 512B is already queued: must send 2nd 512B directly, NOT a ZLP! */
    usbd_cdc_acm_bulk_in(0, CDC_IN_EP, 512);
    if (dataChunksSent != 2 || zlpSent != 0) return 10;
    /* Complete 2nd 512B chunk (end of burst), with transient ZLP send failure to test ZLP retry */
    failNextZlp = 1;
    usbd_cdc_acm_bulk_in(0, CDC_IN_EP, 512);
    if (zlpSent != 0 || !usbZlpPending) return 11;
    usbVcpKickTx();
    if (zlpSent != 1 || usbZlpPending) return 12;
    puts("VCP tracks suspend/resume, latches stall after one 50ms timeout, and sends ZLP only at end of burst");
}
''')

run('usb_msc_sdcard_spi_fault_detection', '''
#define UNUSED(x) ((void)(x))
#define MSC_SD_IO_TIMEOUT_MS 1000
typedef uint32_t timeMs_t;
typedef int32_t timeDelta_t;
typedef uint16_t usbd_msc_block_size_t;
typedef enum { SDCARD_BLOCK_OPERATION_READ, SDCARD_BLOCK_OPERATION_WRITE } sdcardBlockOperation_e;
typedef enum { SDCARD_OPERATION_IN_PROGRESS, SDCARD_OPERATION_BUSY, SDCARD_OPERATION_SUCCESS, SDCARD_OPERATION_FAILURE } sdcardOperationStatus_e;
typedef void (*sdcard_operationCompleteCallback_c)(sdcardBlockOperation_e op, uint32_t blk, uint8_t *buf, uint32_t data);
typedef struct { uint32_t numBlocks; } sdcardMetadata_t;
static timeMs_t nowMs;
static timeMs_t millis(void) { return nowMs; }
static timeDelta_t cmpTimeMs(timeMs_t a, timeMs_t b) { return (timeDelta_t)(a - b); }
static bool cardFunctional = true, cardInitialized = true, cardReady = true;
static int simMode; /* 0=ok, 1=read_crc_err_reinit, 2=write_crc_err_reinit, 3=write_busy_timeout_reset */
static sdcard_operationCompleteCallback_c pendingCb;
static sdcardBlockOperation_e pendingOp;
static uint32_t pendingBlk, pendingData;
static uint8_t *pendingBuf;
static int pollStep;
static bool sdcard_isFunctional(void) { return cardFunctional; }
static bool sdcard_isInitialized(void) { return cardInitialized; }
static bool sdcard_poll(void) {
    nowMs += 1;
    pollStep++;
    if (pendingCb) {
        sdcard_operationCompleteCallback_c cb = pendingCb;
        pendingCb = NULL;
        if (simMode == 1 && pendingOp == SDCARD_BLOCK_OPERATION_READ) {
            /* Simulate sdcard_spi.c read error + transparent re-init back to READY */
            cb(pendingOp, pendingBlk, NULL, pendingData);
            cardReady = true;
            return true;
        }
        if (simMode == 2 && pendingOp == SDCARD_BLOCK_OPERATION_WRITE) {
            /* Simulate sdcard_spi.c write token reject + transparent re-init back to READY */
            cb(pendingOp, pendingBlk, NULL, pendingData);
            cardReady = true;
            return true;
        }
        if (simMode == 3 && pendingOp == SDCARD_BLOCK_OPERATION_WRITE) {
            /* SENDING_WRITE succeeds, enters WAITING_FOR_WRITE */
            cb(pendingOp, pendingBlk, pendingBuf, pendingData);
            cardReady = false;
            return false;
        }
        cb(pendingOp, pendingBlk, pendingBuf, pendingData);
        cardReady = true;
        return true;
    }
    if (simMode == 3 && !cardReady) {
        /* WAITING_FOR_WRITE times out -> sdcard_reset() drops initialized state! */
        cardInitialized = false;
        return false;
    }
    return cardReady;
}
static bool sdcard_readBlock(uint32_t blk, uint8_t *buf, sdcard_operationCompleteCallback_c cb, uint32_t data) {
    if (!cardReady) return false;
    cardReady = false; pendingOp = SDCARD_BLOCK_OPERATION_READ; pendingBlk = blk; pendingBuf = buf; pendingCb = cb; pendingData = data; pollStep = 0;
    return true;
}
static sdcardOperationStatus_e sdcard_writeBlock(uint32_t blk, uint8_t *buf, sdcard_operationCompleteCallback_c cb, uint32_t data) {
    if (!cardReady) return SDCARD_OPERATION_BUSY;
    cardReady = false; pendingOp = SDCARD_BLOCK_OPERATION_WRITE; pendingBlk = blk; pendingBuf = buf; pendingCb = cb; pendingData = data; pollStep = 0;
    return SDCARD_OPERATION_IN_PROGRESS;
}
typedef enum { CH32_SD_OP_PENDING = 0, CH32_SD_OP_OK, CH32_SD_OP_ERROR } ch32SdOpStatus_e;
static volatile ch32SdOpStatus_e ch32SdOpStatus;
''' + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'static void ch32SdSpiOpCallback(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'static bool ch32SdSpiWaitReady(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'static int8_t ch32SdSpiRead(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'static int8_t ch32SdSpiWrite(') + '''
int main(void) {
    uint8_t buf[1024] = {0};
    simMode = 0;
    if (ch32SdSpiRead(0, buf, 10, 2) != 0) return 1;
    if (ch32SdSpiWrite(0, buf, 10, 2) != 0) return 2;
    simMode = 1;
    if (ch32SdSpiRead(0, buf, 10, 1) == 0) return 3;
    simMode = 2;
    if (ch32SdSpiWrite(0, buf, 10, 1) == 0) return 4;
    simMode = 3; cardReady = true; cardInitialized = true;
    if (ch32SdSpiWrite(0, buf, 10, 1) == 0) return 5;
    puts("SD SPI MSC layer detects read CRC errors, write CRC rejects, and write-busy resets without silent data loss");
}
''')

run('usb_msc_async_reset_and_bounds', '''
#define UNUSED(x) ((void)(x))
#define __PACKED __attribute__((packed))
#define __WEAK __attribute__((weak))
#define USB_NOCACHE_RAM_SECTION
#define USB_MEM_ALIGNX __attribute__((aligned(4)))
#define CONFIG_USBDEV_MAX_BUS 1
#define CONFIG_USBDEV_MSC_MAX_LUN 1
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 4096
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING "BETAFLT"
#define CONFIG_USBDEV_MSC_PRODUCT_STRING "Onboard Flash"
#define CONFIG_USBDEV_MSC_VERSION_STRING "    "
#define CONFIG_USBDEV_MSC_POLLING
#define USB_LOG_DBG(...) ((void)0)
#define USB_LOG_WRN(...) ((void)0)
#define USB_LOG_ERR(...) ((void)0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define GET_BE16(p) ((uint16_t)(((uint16_t)(p)[0] << 8) | (uint16_t)(p)[1]))
#define GET_BE32(p) (((uint32_t)(p)[0] << 24) | ((uint32_t)(p)[1] << 16) | ((uint32_t)(p)[2] << 8) | (uint32_t)(p)[3])
enum { USBD_EVENT_RESET = 1, USBD_EVENT_INIT = 2, USBD_EVENT_DEINIT = 3, USBD_EVENT_CONFIGURED = 4, USBD_EVENT_CLEAR_HALT = 5 };
struct usb_setup_packet { uint8_t bmRequestType, bRequest; uint16_t wValue, wIndex, wLength; };
struct usbd_endpoint { uint8_t ep_addr; void (*ep_cb)(uint8_t busid, uint8_t ep, uint32_t nbytes); };
struct usbd_interface {
    int (*class_interface_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*class_endpoint_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*vendor_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    void (*notify_handler)(uint8_t busid, uint8_t event, void *arg);
};
#include "usb_msc.h"
#include "usb_scsi.h"
#include "platform/usb_msc_storage.h"
static uint8_t *lastReadBuf;
static uint32_t lastReadMaxLen;
static uint8_t lastInBuf[4096];
static uint32_t lastInLen;
static unsigned epWriteCalls, epReadCalls;
static void usbd_add_endpoint(uint8_t busid, struct usbd_endpoint *ep) { (void)busid; (void)ep; }
static int usbd_ep_set_stall(uint8_t busid, uint8_t ep) { (void)busid; (void)ep; return 0; }
static int usbd_ep_start_read(uint8_t busid, uint8_t ep, uint8_t *data, uint32_t len) {
    (void)busid; (void)ep; epReadCalls++; lastReadBuf = data; lastReadMaxLen = len; return 0;
}
static int usbd_ep_start_write(uint8_t busid, uint8_t ep, const uint8_t *data, uint32_t len) {
    (void)busid; (void)ep; epWriteCalls++;
    if (len > 0) memcpy(lastInBuf, data, len);
    lastInLen = len;
    return 0;
}
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size);
bool usbd_msc_get_inquiry(uint8_t busid, uint8_t lun, uint8_t *inquiry_buf);
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length);
void usbd_msc_polling(uint8_t busid);
void msc_storage_notify_handler(uint8_t busid, uint8_t event, void *arg);
struct usbd_interface *usbd_msc_init_intf(uint8_t busid, struct usbd_interface *intf, const uint8_t out_ep, const uint8_t in_ep);
''' + usbd_msc_body + '''
static USBD_MSC_StorageType *USBD_STORAGE_fops;
static bool injectResetDuringIo;
static void mscSetActive(void) {}
static const uint8_t customInq[USBD_STD_INQUIRY_LENGTH] = {
    0x00, 0x80, 0x02, 0x02, (USBD_STD_INQUIRY_LENGTH - 5), 0, 0, 0,
    'B','E','T','A','F','L','T',' ',
    'S','D',' ','S','t','o','r','a','g','e',' ',' ',' ',' ',' ',' ',
    '0','.','0','1'
};
static int8_t mockGetCap(uint8_t lun, uint32_t *num, usbd_msc_block_size_t *sz) { (void)lun; *num = 32; *sz = 512; return 0; }
static int8_t mockIsWp(uint8_t lun) { (void)lun; return 0; }
static int8_t mockRead(uint8_t lun, uint8_t *buf, uint32_t addr, uint16_t len) {
    (void)lun; (void)addr;
    memset(buf, 0xaa, (size_t)len * 512u);
    if (injectResetDuringIo) {
        /* Simulate USB Bus Reset + re-CONFIGURED arriving in ISR while slow SPI SD read is running! */
        msc_storage_notify_handler(0, USBD_EVENT_RESET, NULL);
        msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);
    }
    return 0;
}
static int8_t mockWrite(uint8_t lun, uint8_t *buf, uint32_t addr, uint16_t len) {
    (void)lun; (void)buf; (void)addr; (void)len;
    if (injectResetDuringIo) {
        msc_storage_notify_handler(0, USBD_EVENT_RESET, NULL);
        msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);
    }
    return 0;
}
static USBD_MSC_StorageType mockRwFops = {
    .GetCapacity = mockGetCap, .IsWriteProtected = mockIsWp, .Read = mockRead, .Write = mockWrite, .pInquiry = (int8_t *)customInq
};
''' + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'void usbd_msc_get_cap(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'bool usbd_msc_get_inquiry(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'int usbd_msc_sector_read(')
    + fn('src/platform/CH32/usb_msc_ch32h41x.c', 'int usbd_msc_sector_write(') + '''
static void sendCbw(uint32_t tag, uint32_t dataLen, uint8_t flags, const uint8_t *cdb, uint8_t cdbLen) {
    struct CBW cbw = { .dSignature = MSC_CBW_Signature, .dTag = tag, .dDataLength = dataLen, .bmFlags = flags, .bLUN = 0, .bCBLength = cdbLen };
    memcpy(cbw.CB, cdb, cdbLen);
    memcpy(lastReadBuf, &cbw, sizeof(cbw));
    mass_storage_bulk_out(0, 0x01, sizeof(cbw));
}
int main(void) {
    USBD_STORAGE_fops = &mockRwFops;
    struct usbd_interface intf = {0};
    usbd_msc_init_intf(0, &intf, 0x01, 0x81);
    msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);

    /* 1. Dynamic pInquiry string returned for SD storage */
    const uint8_t inqCmd[6] = { SCSI_CMD_INQUIRY, 0, 0, 0, 36, 0 };
    sendCbw(1, 36, 0x80, inqCmd, 6);
    if (lastInLen != 36 || memcmp(&lastInBuf[16], "SD Storage      ", 16) != 0) return 1;
    mass_storage_bulk_in(0, 0x81, 36);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    /* 2. [BUG-4] Async USB Reset during READ(10) polling MUST NOT underflow nsectors or touch EP1 IN */
    injectResetDuringIo = true;
    const uint8_t rd10[10] = { SCSI_CMD_READ10, 0, 0, 0, 0, 4, 0, 0, 4, 0 };
    sendCbw(2, 2048, 0x80, rd10, 10);
    unsigned writesBefore = epWriteCalls;
    usbd_msc_polling(0);
    if (epWriteCalls != writesBefore || g_usbd_msc[0].nsectors != 0 || g_usbd_msc[0].stage != MSC_READ_CBW) return 2;
    if (lastReadBuf != (uint8_t *)&g_usbd_msc[0].cbw || lastReadMaxLen != USB_SIZEOF_MSC_CBW) return 3;

    /* 3. [BUG-4] Async USB Reset during WRITE(10) polling MUST NOT underflow nsectors or send phantom CSW */
    const uint8_t wr10[10] = { SCSI_CMD_WRITE10, 0, 0, 0, 0, 4, 0, 0, 4, 0 };
    sendCbw(3, 2048, 0x00, wr10, 10);
    mass_storage_bulk_out(0, 0x01, 2048);
    writesBefore = epWriteCalls;
    usbd_msc_polling(0);
    if (epWriteCalls != writesBefore || g_usbd_msc[0].nsectors != 0 || g_usbd_msc[0].stage != MSC_READ_CBW) return 4;
    injectResetDuringIo = false;

    /* 4. Medium #2: 32-bit LBA overflow (0xFFFFFFFF + 2) in WRITE(10) stalls OUT until CLEAR_HALT and reports LBAOUTOFRANGE (0x05/0x21/0x00) */
    const uint8_t badWr10[10] = { SCSI_CMD_WRITE10, 0, 0xff, 0xff, 0xff, 0xff, 0, 0, 2, 0 };
    sendCbw(4, 1024, 0x00, badWr10, 10);
    if (g_usbd_msc[0].stage != MSC_BS_ABORT_OUT) return 5;
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x01);
    if (g_usbd_msc[0].stage != MSC_WAIT_CSW) return 5;
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);
    if (g_usbd_msc[0].stage != MSC_READ_CBW) return 5;
    const uint8_t reqSense[6] = { SCSI_CMD_REQUESTSENSE, 0, 0, 0, 18, 0 };
    sendCbw(5, 18, 0x80, reqSense, 6);
    if (lastInLen != 18 || lastInBuf[2] != 0x05u || lastInBuf[12] != 0x21u) return 6;
    mass_storage_bulk_in(0, 0x81, 18);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    /* 5. dDataLength mismatch stalls OUT until CLEAR_HALT; short Bulk-OUT packet (< 512B) fails cleanly with CSW_STATUS_CMD_FAILED */
    sendCbw(6, 1024, 0x00, wr10, 10);
    if (g_usbd_msc[0].stage != MSC_BS_ABORT_OUT) return 7;
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x01);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);
    if (g_usbd_msc[0].stage != MSC_READ_CBW) return 7;
    const uint8_t wr2[10] = { SCSI_CMD_WRITE10, 0, 0, 0, 0, 4, 0, 0, 2, 0 };
    sendCbw(7, 1024, 0x00, wr2, 10);
    mass_storage_bulk_out(0, 0x01, 64); /* bogus short OUT packet */
    usbd_msc_polling(0);
    struct CSW csw = {0};
    memcpy(&csw, lastInBuf, sizeof(csw));
    if (lastInLen != sizeof(csw) || csw.bStatus != CSW_STATUS_CMD_FAILED) return 8;
    puts("MSC survives async USB reset during sector I/O, guards 32-bit LBA overflow, and serves dynamic INQUIRY");
}
''', extra_flags=('-I' + str(root / 'lib/modules/CH32H41x/middlewares/class/msc'), '-I' + str(root / 'src/platform/CH32/include')))

run('ws2811_ledstrip_dma_and_isr', '''
#define USE_DMA_SPEC
#define WS2811_TIMER_MHZ 48
#define WS2811_CARRIER_HZ 800000
#define WS2811_DMA_BUFFER_SIZE 106
#define TIMER_OUTPUT_N_CHANNEL 0x02
#define TIMER_OUTPUT_INVERTED  0x01
#define DIR_OUT 3
#define GPIO_MODE_OUT_AF_PP 2
#define GPIO_SPEED_VERY_HIGH 3
#define GPIO_PULL_UP 2
#define IO_CONFIG(d,m,s,p) ((d)|((m)<<2)|((s)<<4)|((p)<<6))
#define IO_NONE NULL
#define OWNER_LED_STRIP 9
#define NVIC_PRIO_WS2811_DMA 0x60
#define ENABLE 1
#define DISABLE 0
#define TIM_CKD_DIV1 0
#define TIM_CounterMode_Up 0
#define TIM_OCMode_PWM1 6
#define TIM_OutputNState_Enable 4
#define TIM_OCNIdleState_Reset 0
#define TIM_OCNPolarity_Low 8
#define TIM_OCNPolarity_High 0
#define TIM_OutputState_Enable 1
#define TIM_OCIdleState_Set 0x100
#define TIM_OCPolarity_Low 2
#define TIM_OCPolarity_High 0
#define TIM_OCPreload_Enable 8
#define TIM_CCxN_Enable 4
#define TIM_CCx_Enable 1
#define DMA_PeripheralInc_Disable 0
#define DMA_MemoryInc_Enable 1
#define DMA_DIR_PeripheralDST 1
#define DMA_PeripheralDataSize_HalfWord 0x100
#define DMA_PeripheralDataSize_Word 0x200
#define DMA_MemoryDataSize_Word 0x800
#define DMA_Priority_VeryHigh 0x3000
#define DMA_M2M_Disable 0
#define DMA_Mode_Normal 0
#define DMA_IT_TC 2
#define DMA_IT_GLOB 1
#define DMA_IT_TCIF 2
#define DMA_IT_HTIF 4
#define DMA_IT_TEIF 8
typedef int ioTag_t;
typedef int dmaIdentifier_e;
typedef void *IO_t;
typedef struct { int id; } dmaResource_t;
typedef struct { int dummy; } TIM_TypeDef;
typedef TIM_TypeDef timerResource_t;
static TIM_TypeDef tim4_inst, tim9_inst, tim10_inst, tim11_inst, tim12_inst;
#define TIM4 (&tim4_inst)
#define TIM9 (&tim9_inst)
#define TIM10 (&tim10_inst)
#define TIM11 (&tim11_inst)
#define TIM12 (&tim12_inst)
typedef struct { TIM_TypeDef *tim; uint8_t channel; uint8_t output; uint8_t alternateFunction; } timerHardware_t;
typedef struct { dmaResource_t *ref; uint32_t dmaMuxId; } dmaChannelSpec_t;
typedef struct dmaChannelDescriptor_s { dmaResource_t *ref; uint32_t flags; int irqN; } dmaChannelDescriptor_t;
typedef void (*dmaCallbackHandlerFuncPtr)(dmaChannelDescriptor_t *);
typedef struct { uint32_t TIM_Period; uint16_t TIM_Prescaler; uint16_t TIM_ClockDivision; uint16_t TIM_CounterMode; } TIM_TimeBaseInitTypeDef;
typedef struct { uint16_t TIM_OCMode; uint16_t TIM_OutputState; uint16_t TIM_OutputNState; uint32_t TIM_Pulse; uint16_t TIM_OCPolarity; uint16_t TIM_OCNPolarity; uint16_t TIM_OCIdleState; uint16_t TIM_OCNIdleState; } TIM_OCInitTypeDef;
typedef struct { uint32_t DMA_PeripheralBaseAddr; uint32_t DMA_Memory0BaseAddr; uint32_t DMA_DIR; uint32_t DMA_BufferSize; uint32_t DMA_PeripheralInc; uint32_t DMA_MemoryInc; uint32_t DMA_PeripheralDataSize; uint32_t DMA_MemoryDataSize; uint32_t DMA_Mode; uint32_t DMA_Priority; uint32_t DMA_M2M; } DMA_InitTypeDef;

static ioTag_t ledStripIoTag = 1;
static uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
static uint16_t BIT_COMPARE_1, BIT_COMPARE_0;
static volatile bool ws2811LedDataTransferInProgress;
static IO_t ws2811IO = IO_NONE;
static dmaResource_t fakeDma = { .id = 6 };
static dmaChannelDescriptor_t fakeDesc = { .ref = &fakeDma, .flags = 0x0Fu, .irqN = 55 };
static dmaResource_t *dmaRef = NULL;
static TIM_TypeDef *timer = NULL;
static timerHardware_t activeHw;
static dmaChannelSpec_t activeSpec = { .ref = &fakeDma, .dmaMuxId = 45 };
static DMA_InitTypeDef lastDmaInit;
static TIM_TimeBaseInitTypeDef lastTimBase;
static uint32_t lastMuxId;
static int dmaEnabledState = -1;
static int timEnabledState = -1;
static int timStateAtDmaEnable = -1;
static int stateWhenReadyCleared = -1;
static int clearedIrqN = -1;
static uint32_t flagsAtItEnable = 0xFFu;
static dmaCallbackHandlerFuncPtr registeredHandler;
static uint32_t fakeCcr;
static int fakeIoObj;

static uint32_t HCLKClock = 400000000U;
static uint32_t timerClockFromInstance(const timerResource_t *t) { (void)t; return HCLKClock; }
#define MHZ_TO_HZ(x) ((x) * 1000000U)
static uint16_t timerGetPrescalerByDesiredHertz(timerResource_t *t, uint32_t hz);
''' + fn('src/platform/CH32/timer_ch32bsp.c', 'uint16_t timerGetPrescalerByDesiredMhz(')
    + fn('src/platform/CH32/timer_ch32bsp.c', 'uint16_t timerGetPeriodByPrescaler(')
    + fn('src/platform/CH32/timer_ch32bsp.c', 'uint16_t timerGetPrescalerByDesiredHertz(') + '''
static const timerHardware_t *timerAllocate(ioTag_t tag, int owner, int idx) { (void)tag; (void)owner; (void)idx; return &activeHw; }
static const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *hw) { (void)hw; return &activeSpec; }
static dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *r) { return r ? r->id : 0; }
static dmaChannelDescriptor_t *dmaGetDescriptorByIdentifier(dmaIdentifier_e id) { return id == 6 ? &fakeDesc : NULL; }
static bool dmaAllocate(int id, int owner, int idx) { (void)owner; (void)idx; return id == 6; }
static IO_t IOGetByTag(ioTag_t tag) { return tag ? &fakeIoObj : NULL; }
static void IOInit(IO_t io, int owner, int idx) { (void)io; (void)owner; (void)idx; }
static void IOConfigGPIOAF(IO_t io, uint32_t cfg, uint8_t af) { (void)io; (void)cfg; (void)af; }
static int timerRCC(const TIM_TypeDef *t) { (void)t; return 1; }
static void RCC_ClockCmd(int rcc, int cmd) { (void)rcc; (void)cmd; }
static void TIM_Cmd(TIM_TypeDef *t, int cmd) { (void)t; timEnabledState = cmd; }
static void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *s) { memset(s, 0, sizeof(*s)); }
static void TIM_TimeBaseInit(TIM_TypeDef *t, TIM_TimeBaseInitTypeDef *s) { (void)t; lastTimBase = *s; }
static void TIM_OCStructInit(TIM_OCInitTypeDef *s) { memset(s, 0, sizeof(*s)); }
static void timerOCInit(TIM_TypeDef *t, uint8_t ch, TIM_OCInitTypeDef *s) { (void)t; (void)ch; (void)s; }
static void timerOCPreloadConfig(TIM_TypeDef *t, uint8_t ch, uint16_t p) { (void)t; (void)ch; (void)p; }
static void TIM_CtrlPWMOutputs(TIM_TypeDef *t, int cmd) { (void)t; (void)cmd; }
static void TIM_ARRPreloadConfig(TIM_TypeDef *t, int cmd) { (void)t; (void)cmd; }
static void TIM_CCxNCmd(TIM_TypeDef *t, uint8_t ch, uint16_t cmd) { (void)t; (void)ch; (void)cmd; }
static void TIM_CCxCmd(TIM_TypeDef *t, uint8_t ch, uint16_t cmd) { (void)t; (void)ch; (void)cmd; }
static void dmaEnable(int id) { (void)id; }
static void dmaSetHandler(int id, dmaCallbackHandlerFuncPtr cb, uint32_t prio, uint32_t user) { (void)id; (void)prio; (void)user; registeredHandler = cb; }
static void xDMA_Cmd(dmaResource_t *r, int cmd) { (void)r; dmaEnabledState = cmd; if (cmd == ENABLE) timStateAtDmaEnable = timEnabledState; }
static void xDMA_DeInit(dmaResource_t *r) { (void)r; }
static void DMA_StructInit(DMA_InitTypeDef *s) { memset(s, 0, sizeof(*s)); }
static uint32_t *timerCCR(TIM_TypeDef *t, uint8_t ch) { (void)t; (void)ch; return &fakeCcr; }
static void xDMA_Init(dmaResource_t *r, DMA_InitTypeDef *s) { (void)r; lastDmaInit = *s; }
static void dmaMuxEnable(int id, uint32_t mux) { (void)id; lastMuxId = mux; }
static uint16_t timerDmaSource(uint8_t ch) { return (uint16_t)(1u << ch); }
static void TIM_DMACmd(TIM_TypeDef *t, uint16_t src, int cmd) { (void)t; (void)src; (void)cmd; }
static void xDMA_ITConfig(dmaResource_t *r, uint32_t it, int cmd) { (void)r; (void)it; if (cmd == ENABLE) flagsAtItEnable = fakeDesc.flags; }
static void xDMA_SetCurrDataCounter(dmaResource_t *r, uint16_t cnt) { (void)r; (void)cnt; }
static void TIM_SetCounter(TIM_TypeDef *t, uint32_t cnt) { (void)t; (void)cnt; }
static void NVIC_ClearPendingIRQ(int irq) { clearedIrqN = irq; }
static void ch32DmaBarrier(void) { stateWhenReadyCleared = dmaEnabledState; }
#define DMA_GET_FLAG_STATUS(d, f) (((d)->flags & (f)) != 0)
#define DMA_CLEAR_FLAG(d, f) ((d)->flags &= ~(uint32_t)(f))
''' + fn('src/platform/CH32/light_ws2811strip_ch32h41x.c', 'static inline bool isWs2811Timer32Bit(')
    + fn('src/platform/CH32/light_ws2811strip_ch32h41x.c', 'static void WS2811_DMA_IRQHandler(')
    + fn('src/platform/CH32/light_ws2811strip_ch32h41x.c', 'bool ws2811LedStripHardwareInit(')
    + fn('src/platform/CH32/light_ws2811strip_ch32h41x.c', 'void ws2811LedStripStartTransfer(') + '''
int main(void) {
    /* 1. 16-bit TIM4 (e.g. TIM4 CH1 PB6) uses HalfWord peripheral width and clears stale flags/IRQ before TCIE */
    activeHw = (timerHardware_t){ .tim = TIM4, .channel = 0, .output = 0, .alternateFunction = 2 };
    if (!ws2811LedStripHardwareInit()) return 1;
    if (lastTimBase.TIM_Prescaler != 7 || lastTimBase.TIM_Period != 61) return 2;
    if (BIT_COMPARE_0 != 20 || BIT_COMPARE_1 != 40) return 3;
    if (lastDmaInit.DMA_PeripheralDataSize != DMA_PeripheralDataSize_HalfWord || lastDmaInit.DMA_MemoryDataSize != DMA_MemoryDataSize_Word) return 4;
    if (lastMuxId != 45 || !registeredHandler || flagsAtItEnable != 0 || clearedIrqN != 55) return 5;

    /* 2. Start transfer keeps timer disabled until DMA is armed, then enables timer; TC ISR disables DMA + barriers first */
    ws2811LedDataTransferInProgress = true;
    ws2811LedStripStartTransfer();
    if (dmaEnabledState != ENABLE || timStateAtDmaEnable != DISABLE || timEnabledState != ENABLE) return 6;
    dmaChannelDescriptor_t desc = { .ref = &fakeDma, .flags = DMA_IT_TCIF };
    registeredHandler(&desc);
    if (ws2811LedDataTransferInProgress || dmaEnabledState != DISABLE || stateWhenReadyCleared != DISABLE || desc.flags != 0) return 7;

    /* 3. 32-bit TIM9 uses Word peripheral width for CHxCVR_32 */
    activeHw.tim = TIM9;
    if (!ws2811LedStripHardwareInit()) return 8;
    if (lastDmaInit.DMA_PeripheralDataSize != DMA_PeripheralDataSize_Word) return 9;
    puts("WS2811 LED strip selects 16/32-bit CCR width, clears stale flags, aligns bit 0, and completes race-free");
}
''')
run('camera_control_modes_and_fallback', '''
#include <math.h>
#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
#define CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#define CAMERA_CONTROL_DAC_AVAILABLE
#define USE_PWM_OUTPUT
#define __FAST_INTERRUPT
#define CAMERA_CONTROL_VDD_VOLTAGE 3.3f
#define CAMERA_CONTROL_PWM_RESOLUTION 128
#define CAMERA_CONTROL_SOFT_PWM_RESOLUTION 448
#define CAMERA_CONTROL_DAC_MAX_CODE 4095U
#define CAMERA_CONTROL_SOFT_PWM_TIMER_MHZ 20U
#define DIR_IN 0x00
#define DIR_OUT 0x03
#define GPIO_MODE_IN_AN 0x00
#define GPIO_MODE_OUT_PP 0x00
#define GPIO_MODE_OUT_AF_PP 0x02
#define GPIO_SPEED_VERY_HIGH 0x03
#define GPIO_PULL_NONE 0x00
#define IO_CONFIG(dir, mode, speed, pupd) ((dir) | ((mode) << 2) | ((speed) << 4) | ((pupd) << 6))
#define IOCFG_OUT_PP IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_AF_PP IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define CAMERA_CONTROL_IO_ANALOG_CFG IO_CONFIG(DIR_IN, GPIO_MODE_IN_AN, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IO_NONE NULL
#define OWNER_CAMERA_CONTROL 20
#define NVIC_PRIO_TIMER 0x50
#define NVIC_PRIO_MAX 0x10
#define ENABLE 1
#define DISABLE 0
#define TIM6_IRQn 100
#define TIM7_IRQn 101
#define RCC_HB1Periph_TIM6 0x10U
#define RCC_HB1Periph_TIM7 0x20U
#define RCC_HB1Periph_DAC 0x20000000U
#define RCC_HB1(periph) RCC_HB1Periph_##periph
#define TIM_CEN 0x0001U
#define TIM_IT_Update 0x0001U
#define TIM_PSCReloadMode_Immediate 0x0001U
#define DAC_Channel_1 0x00U
#define DAC_Channel_2 0x10U
#define DAC_Trigger_None 0U
#define DAC_WaveGeneration_None 0U
#define DAC_OutputBuffer_Enable 0U
#define DAC_EN1 0x0001U
#define ATOMIC_BLOCK(p) for (int _once = ((void)(p), 1); _once; _once = 0)
#define DEFIO_TAG_E(pin) TAG_##pin
enum { TAG_NONE = 0, TAG_PA4 = 1, TAG_PA5 = 0, TAG_PB8 = 2 };

typedef int ioTag_t;
typedef void *IO_t;
typedef uint32_t timeMs_t;
typedef enum {
    CAMERA_CONTROL_KEY_ENTER,
    CAMERA_CONTROL_KEY_LEFT,
    CAMERA_CONTROL_KEY_UP,
    CAMERA_CONTROL_KEY_RIGHT,
    CAMERA_CONTROL_KEY_DOWN,
    CAMERA_CONTROL_KEYS_COUNT
} cameraControlKey_e;
typedef enum {
    CAMERA_CONTROL_MODE_HARDWARE_PWM,
    CAMERA_CONTROL_MODE_SOFTWARE_PWM,
    CAMERA_CONTROL_MODE_DAC,
    CAMERA_CONTROL_MODES_COUNT
} cameraControlMode_e;
typedef struct {
    bool enabled;
    IO_t io;
    uint32_t period;
    uint8_t inverted;
    timeMs_t endTimeMillis;
} cameraControlRuntime_t;
typedef struct {
    cameraControlMode_e mode;
    uint16_t refVoltage;
    uint16_t keyDelayMs;
    uint16_t internalResistance;
    ioTag_t ioTag;
    uint8_t inverted;
    uint16_t buttonResistanceValues[CAMERA_CONTROL_KEYS_COUNT];
} cameraControlConfig_t;
typedef struct {
    uint16_t CTLR1;
    uint16_t DMAINTENR;
    uint16_t INTFR;
    uint16_t SWEVGR;
    uint16_t CNT;
    uint16_t PSC;
    uint16_t ATRLR;
    uint16_t swevgrCount;
} TIM_TypeDef;
typedef TIM_TypeDef timerResource_t;
typedef struct {
    TIM_TypeDef *tim;
    uint8_t alternateFunction;
} timerHardware_t;
typedef struct {
    uint32_t *ccr;
} timerChannel_t;
typedef struct {
    uint32_t HB1ENR;
} RCC_TypeDef;
typedef struct {
    uint32_t CTLR;
    uint32_t R12BDHR1;
    uint32_t R12BDHR2;
} DAC_TypeDef;
typedef struct {
    uint32_t DAC_Trigger;
    uint32_t DAC_WaveGeneration;
    uint32_t DAC_OutputBuffer;
} DAC_InitTypeDef;

static TIM_TypeDef tim6_inst, tim7_inst, tim3_inst;
static RCC_TypeDef rcc_inst;
static DAC_TypeDef dac_inst;
#define TIM6 (&tim6_inst)
#define TIM7 (&tim7_inst)
#define RCC (&rcc_inst)
#define DAC (&dac_inst)

static int pa4Obj, pb8Obj;
static cameraControlConfig_t cfg;
static const cameraControlConfig_t *cameraControlConfig(void) { return &cfg; }
static cameraControlRuntime_t *cameraControlRuntime;
static cameraControlMode_e activeMode = CAMERA_CONTROL_MODES_COUNT;
static timerChannel_t channel;
static volatile uint16_t softPwmHiTime;
static uint32_t activeDacChannel;
static const timerHardware_t *allocTim;
static uint32_t hwCcr;
static uint32_t hwCcrAtAfMux;
static int pinLevel = -1;
static int pinLevelAtOutPpSwitch = -1;
static int contentionDetected = 0;
static uint32_t dacCodeAtEnable = 0xFFFFFFFFU;
static uint32_t lastIoCfg = CAMERA_CONTROL_IO_ANALOG_CFG;
static timeMs_t fakeNowMs = 1000;

static timeMs_t millis(void) { return fakeNowMs; }
static void ch32DmaBarrier(void) {}
static IO_t IOGetByTag(ioTag_t tag) {
    if (tag == TAG_PA4) return &pa4Obj;
    if (tag == TAG_PB8) return &pb8Obj;
    return IO_NONE;
}
static void IOHi(IO_t io) { (void)io; pinLevel = 1; }
static void IOLo(IO_t io) { (void)io; pinLevel = 0; }
static void IOConfigGPIO(IO_t io, uint32_t c) {
    (void)io;
    if (c == IOCFG_OUT_PP) {
        pinLevelAtOutPpSwitch = pinLevel;
        if ((DAC->CTLR & DAC_EN1) != 0) contentionDetected = 1;
    }
    lastIoCfg = c;
}
static void IOConfigGPIOAF(IO_t io, uint32_t c, uint8_t af) {
    (void)io; (void)af;
    hwCcrAtAfMux = hwCcr;
    lastIoCfg = c;
}
static const timerHardware_t *timerAllocate(ioTag_t tag, int owner, int idx) { (void)tag; (void)owner; (void)idx; return allocTim; }
static uint32_t timerClock(const timerHardware_t *hw) { (void)hw; return 400000000U; }
static void pwmOutputConfig(timerChannel_t *ch, const timerHardware_t *hw, uint32_t hz, uint16_t period, uint16_t val, uint8_t inv) {
    (void)hw; (void)hz; (void)period; (void)inv;
    ch->ccr = &hwCcr;
    hwCcr = val;
}
static uint32_t HCLKClock = 400000000U;
static uint32_t timerClockFromInstance(const timerResource_t *t) { (void)t; return HCLKClock; }
#define MHZ_TO_HZ(x) ((x) * 1000000U)
static uint16_t timerGetPrescalerByDesiredHertz(timerResource_t *t, uint32_t hz);
''' + fn('src/platform/CH32/timer_ch32bsp.c', 'uint16_t timerGetPrescalerByDesiredMhz(')
    + fn('src/platform/CH32/timer_ch32bsp.c', 'uint16_t timerGetPrescalerByDesiredHertz(') + '''
static void NVIC_SetPriority(int irq, uint32_t prio) { (void)irq; (void)prio; }
static void NVIC_EnableIRQ(int irq) { (void)irq; }
static void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static void RCC_ClockCmd(uint32_t mask, int state) { if (state) RCC->HB1ENR |= mask; else RCC->HB1ENR &= ~mask; }
static void DAC_StructInit(DAC_InitTypeDef *s) { memset(s, 0, sizeof(*s)); }
static void DAC_Init(uint32_t ch, DAC_InitTypeDef *s) { (void)ch; (void)s; }
static void DAC_Cmd(uint32_t ch, int state) {
    if (state) {
        if (lastIoCfg == IOCFG_OUT_PP) contentionDetected = 1;
        dacCodeAtEnable = (ch == DAC_Channel_1) ? DAC->R12BDHR1 : DAC->R12BDHR2;
        DAC->CTLR |= (DAC_EN1 << ch);
    } else {
        DAC->CTLR &= ~(DAC_EN1 << ch);
    }
}
''' + fn('src/platform/CH32/camera_control_ch32.c', 'static float calculateKeyPressVoltage(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static float calculateKeyPressRatio(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static bool cameraControlHardwarePwmTryInit(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlHi(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlLo(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'void TIM6_IRQHandler(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'void TIM7_IRQHandler(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlSoftwarePwmDisable(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlSoftwarePwmEnable(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static bool cameraControlSoftwarePwmTryInit(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static bool cameraControlResolveDacChannel(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlDacWriteCode(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static bool cameraControlDacTryInit(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'static void cameraControlStopActiveBackend(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'void cameraControlInitImpl(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'void cameraControlProcessImpl(')
    + fn('src/platform/CH32/camera_control_ch32.c', 'void cameraControlKeyPressImpl(') + '''
int main(void) {
    cfg = (cameraControlConfig_t){
        .mode = CAMERA_CONTROL_MODE_DAC,
        .refVoltage = 330,
        .keyDelayMs = 180,
        .internalResistance = 470,
        .ioTag = TAG_PA4,
        .inverted = 0,
        .buttonResistanceValues = { 450, 270, 150, 68, 0 },
    };

    /* 1. Default DAC-first mode: PA4 prefers 12-bit DAC1 with break-before-make (zero CMOS/DAC contention) */
    timerHardware_t hw = { .tim = &tim3_inst, .alternateFunction = 2 };
    allocTim = &hw;
    cameraControlRuntime_t rt = { .io = &pa4Obj, .inverted = 0 };
    cameraControlInitImpl(&rt);
    if (!rt.enabled || activeMode != CAMERA_CONTROL_MODE_DAC || (RCC->HB1ENR & RCC_HB1Periph_DAC) == 0) return 1;
    if (lastIoCfg != IOCFG_OUT_PP || pinLevel != 1 || pinLevelAtOutPpSwitch != 1 || (DAC->CTLR & DAC_EN1) != 0) return 2;

    const uint16_t expectedCodes[4] = { 2003, 1494, 991, 518 };
    for (int k = 0; k < 4; k++) {
        cameraControlKeyPressImpl((cameraControlKey_e)k, 200);
        if (lastIoCfg != CAMERA_CONTROL_IO_ANALOG_CFG || (DAC->CTLR & DAC_EN1) == 0) return 3;
        if (DAC->R12BDHR1 != expectedCodes[k] || dacCodeAtEnable != expectedCodes[k]) return 4;
        if (rt.endTimeMillis != 1380) return 5;
    }

    cameraControlKeyPressImpl(CAMERA_CONTROL_KEY_DOWN, 0);
    if (lastIoCfg != IOCFG_OUT_PP || pinLevel != 0 || pinLevelAtOutPpSwitch != 0 || (DAC->CTLR & DAC_EN1) != 0) return 6;
    cameraControlProcessImpl();
    if (lastIoCfg != IOCFG_OUT_PP || pinLevel != 1 || pinLevelAtOutPpSwitch != 1 || (DAC->CTLR & DAC_EN1) != 0) return 7;
    if (contentionDetected) return 8;

    /* Inverted DAC mode + 49.7-day millis() rollover sentinel guard (holdEndMillis == 0 -> 1) */
    rt.inverted = 1;
    cameraControlInitImpl(&rt);
    if (pinLevel != 0 || (DAC->CTLR & DAC_EN1) != 0) return 9;
    fakeNowMs = 0xFFFFFFFFU - 180U + 1U;
    cameraControlKeyPressImpl(CAMERA_CONTROL_KEY_ENTER, 0);
    if (DAC->R12BDHR1 != (4095 - 2003) || lastIoCfg != CAMERA_CONTROL_IO_ANALOG_CFG || rt.endTimeMillis != 1) return 10;
    fakeNowMs = 1000;
    cameraControlKeyPressImpl(CAMERA_CONTROL_KEY_DOWN, 0);
    if (pinLevel != 1 || lastIoCfg != IOCFG_OUT_PP || (DAC->CTLR & DAC_EN1) != 0 || contentionDetected) return 11;

    /* 2. Default DAC-first mode on non-DAC pin WITH free timer falls back to HARDWARE_PWM without boot low pulse */
    cfg.ioTag = TAG_PB8;
    rt = (cameraControlRuntime_t){ .io = &pb8Obj, .inverted = 0 };
    cameraControlInitImpl(&rt);
    if (!rt.enabled || activeMode != CAMERA_CONTROL_MODE_HARDWARE_PWM || hwCcr != 128 || hwCcrAtAfMux != 128) return 12;
    cameraControlKeyPressImpl(CAMERA_CONTROL_KEY_ENTER, 0);
    if (hwCcr != 63 || rt.endTimeMillis != 1180) return 13;
    cameraControlProcessImpl();
    if (hwCcr != 128) return 14;

    /* 3. Default DAC-first mode on non-DAC pin WITHOUT free timer falls back to 20 MHz TIM6+TIM7 SOFTWARE_PWM */
    allocTim = NULL;
    cameraControlInitImpl(&rt);
    if (!rt.enabled || activeMode != CAMERA_CONTROL_MODE_SOFTWARE_PWM) return 15;
    if (TIM6->PSC != 19 || TIM7->PSC != 19 || TIM6->SWEVGR != TIM_PSCReloadMode_Immediate || TIM7->SWEVGR != TIM_PSCReloadMode_Immediate) return 16;
    if (pinLevel != 1 || lastIoCfg != IOCFG_OUT_PP) return 17;

    cameraControlKeyPressImpl(CAMERA_CONTROL_KEY_ENTER, 2000);
    if (TIM6->CNT != 219 || TIM6->ATRLR != 448 || TIM7->CNT != 0 || TIM7->ATRLR != 448) return 18;
    if (TIM6->CTLR1 != TIM_CEN || TIM7->CTLR1 != TIM_CEN || TIM6->DMAINTENR != TIM_IT_Update || TIM7->DMAINTENR != TIM_IT_Update) return 19;
    if (rt.endTimeMillis != 3180) return 20;

    /* Normal on-time ISR edges: TIM6 overflow -> CNT=0 < hiTime (Hi), TIM7 overflow -> CNT=hiTime >= hiTime (Lo) */
    TIM6->CNT = 0;
    TIM6->INTFR = TIM_IT_Update;
    TIM6_IRQHandler();
    if (pinLevel != 1 || TIM6->INTFR != 0) return 21;
    TIM6->CNT = 219;
    TIM7->INTFR = TIM_IT_Update;
    TIM7_IRQHandler();
    if (pinLevel != 0 || TIM7->INTFR != 0) return 22;

    /* [BUG-3] Simultaneous-pending TIM6+TIM7 during HIGH phase (e.g. hiTime=447 after >100ns Group-0 ISR) MUST stay HIGH */
    cameraControlSoftwarePwmEnable(447, 448);
    TIM6->CNT = 1;
    TIM6->INTFR = TIM_IT_Update;
    TIM7->INTFR = TIM_IT_Update;
    TIM6_IRQHandler();
    TIM7_IRQHandler();
    if (pinLevel != 1) return 23;

    /* [BUG-3] Simultaneous-pending TIM6+TIM7 during LOW phase (e.g. hiTime=1 after >50ns Group-0 ISR) MUST stay LOW without glitch */
    cameraControlSoftwarePwmEnable(1, 448);
    TIM6->CNT = 2;
    TIM6->INTFR = TIM_IT_Update;
    TIM7->INTFR = TIM_IT_Update;
    TIM6_IRQHandler();
    if (pinLevel != 0) return 24;
    TIM7_IRQHandler();
    if (pinLevel != 0) return 25;

    /* [BUG-5] Re-init with IO_NONE stops active SOFTWARE_PWM timers and clears runtime->enabled */
    rt.io = IO_NONE;
    cameraControlInitImpl(&rt);
    if (rt.enabled || activeMode != CAMERA_CONTROL_MODES_COUNT || (TIM6->CTLR1 & TIM_CEN) != 0 || TIM6->DMAINTENR != 0) return 26;

    /* 4. Legacy saved HARDWARE_PWM config on PA4 (where TIM10 is reserved) still falls back to DAC1 */
    cfg.mode = CAMERA_CONTROL_MODE_HARDWARE_PWM;
    cfg.ioTag = TAG_PA4;
    rt = (cameraControlRuntime_t){ .io = &pa4Obj, .inverted = 0 };
    cameraControlInitImpl(&rt);
    if (!rt.enabled || activeMode != CAMERA_CONTROL_MODE_DAC) return 27;

    puts("CH32H4 camera control prefers 12-bit DAC -> HARDWARE_PWM -> 20 MHz SOFTWARE_PWM with simultaneous-IRQ & break-before-make safety");
}
''', extra_flags=('-lm',))

run('pwm_dshot_bidir_tcie_and_gpio_atomicity', '''
#define USE_DSHOT_TELEMETRY
#define FAST_CODE
#define NVIC_PRIO_TIMER 0x50
#define ENABLE 1
#define DISABLE 0
#define DMA_CFGR1_EN 0x0001U
#define DMA_IT_TC 0x0002U
#define DMA_IT_GLOB 0x0001U
#define DMA_IT_TCIF 0x0002U
#define DMA_IT_HTIF 0x0004U
#define DMA_IT_TEIF 0x0008U
#define DMA_DIR_PeripheralDST 1U
#define DMA_DIR_PeripheralSRC 0U
#define TIM_OCPreload_Disable 0U
#define TIM_OCPreload_Enable 8U
#define TIM_ARPE 0x0080U
#define TIM_TS 0x0070U
#define TIM_TS_TI1F_ED 0x0040U
#define TIM_Channel_1 0x0000U
#define TIM_Channel_2 0x0004U
#define TIM_Channel_3 0x0008U
#define TIM_Channel_4 0x000CU
#define TIM_AUX_CAPCH2_ED 0x0001U
#define TIM_AUX_CAPCH3_ED 0x0002U
#define TIM_AUX_CAPCH4_ED 0x0004U
#define TIM_AUX_CAPCH234_ED 0x0007U
#define GCR_TELEMETRY_INPUT_LEN 42U
#define ATOMIC_BLOCK(p) for (int _once = ((void)(p), atomicEntries++, 1); _once; _once = 0)
#define READ_REG(REG) (REG)
#define WRITE_REG(REG, VAL) do { \
    if ((void *)&(REG) == (void *)&gpioe.CFGLR) { \
        regWriteCount++; \
        if (((VAL) & 0xF000U) == 0U) sawTransientAin = 1; \
    } \
    (REG) = (VAL); \
} while (0)
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

typedef int ioTag_t;
typedef int dmaIdentifier_e;
typedef void *IO_t;
typedef struct { uint32_t CFGR; uint32_t CNTR; } DMA_Channel_TypeDef;
typedef DMA_Channel_TypeDef DMA_ARCH_TYPE;
typedef DMA_Channel_TypeDef dmaResource_t;
typedef struct { uint32_t CFGLR; uint32_t CFGHR; uint32_t BSHR; } GPIO_TypeDef;
typedef struct { uint16_t CTLR1; uint16_t DMAINTENR; uint16_t SMCFGR; uint16_t AUX; uint32_t ATRLR; } TIM_TypeDef;
typedef struct { uint32_t DMA_DIR; } DMA_InitTypeDef;
typedef struct { int dummy; } TIM_OCInitTypeDef;
typedef struct { int dummy; } TIM_ICInitTypeDef;
typedef struct { TIM_TypeDef *tim; uint8_t channel; ioTag_t tag; } timerHardware_t;
typedef struct {
    const timerHardware_t *timerHardware;
    dmaResource_t *dmaRef;
    uint16_t timerDmaSource;
    bool isInput;
    TIM_OCInitTypeDef ocInitStruct;
    TIM_ICInitTypeDef icInitStruct;
    DMA_InitTypeDef dmaInitStruct;
} motorDmaOutput_t;
typedef struct dmaChannelDescriptor_s {
    dmaResource_t *ref;
    uint32_t flags;
    int irqN;
    uint32_t userParam;
} dmaChannelDescriptor_t;

static TIM_TypeDef tim3_inst, tim9_inst, tim10_inst, tim11_inst, tim12_inst;
#define TIM3 (&tim3_inst)
#define TIM9 (&tim9_inst)
#define TIM10 (&tim10_inst)
#define TIM11 (&tim11_inst)
#define TIM12 (&tim12_inst)

static GPIO_TypeDef gpioe;
static DMA_Channel_TypeDef dmaCh;
static dmaChannelDescriptor_t dmaDesc = { .ref = &dmaCh, .flags = 0, .irqN = 47, .userParam = 0 };
static motorDmaOutput_t dmaMotors[2];
static bool useDshotTelemetry = true;
static uint32_t inputStampUs;
static struct { uint32_t irqAt; uint32_t changeDirectionCompletedAt; } dshotDMAHandlerCycleCounters;
static unsigned atomicEntries, regWriteCount;
static int sawTransientAin;
static uint32_t flagsWhenOutputTcEnabled = 0xFFU;
static int clearedIrqN = -1;

static void ch32DmaBarrier(void) {}
static uint32_t micros(void) { return 12345U; }
static uint32_t getCycleCounter(void) { return 77U; }
static IO_t IOGetByTag(ioTag_t tag) { return (IO_t)(uintptr_t)tag; }
static GPIO_TypeDef *IO_GPIO(IO_t io) { (void)io; return &gpioe; }
static uint16_t IO_Pin(IO_t io) { return (uint16_t)(1U << ((uintptr_t)io & 0xFU)); }
static void timerOCPreloadConfig(TIM_TypeDef *t, uint8_t ch, uint16_t p) { (void)t; (void)ch; (void)p; }
static void timerOCInit(TIM_TypeDef *t, uint8_t ch, TIM_OCInitTypeDef *s) { (void)t; (void)ch; (void)s; }
static void TIM_SetAutoreload(TIM_TypeDef *t, uint32_t arr) { t->ATRLR = arr; }
static void TIM_ICInit(TIM_TypeDef *t, TIM_ICInitTypeDef *s) { (void)t; (void)s; }
static void xDMA_Init(dmaResource_t *r, DMA_InitTypeDef *s) { (void)s; /* WCH DMA_Init preserves TCIE bit in CFGR! */ (void)r; }
static void xDMA_ITConfig(dmaResource_t *r, uint32_t it, int state) {
    if (state) {
        flagsWhenOutputTcEnabled = dmaDesc.flags;
        r->CFGR |= it;
    } else {
        r->CFGR &= ~it;
    }
}
static dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *r) { return r == &dmaCh ? 1 : 0; }
static dmaChannelDescriptor_t *dmaGetDescriptorByIdentifier(dmaIdentifier_e id) { return id == 1 ? &dmaDesc : NULL; }
static void NVIC_ClearPendingIRQ(int irq) { clearedIrqN = irq; }
#define DMA_GET_FLAG_STATUS(d, f) (((d)->flags & (f)) != 0)
#define DMA_CLEAR_FLAG(d, f) ((d)->flags &= ~(uint32_t)(f))
''' + fn('src/platform/CH32/pwm_output_dshot.c', 'static inline bool isTimer32Bit(')
    + fn('src/platform/CH32/pwm_output_dshot.c', 'static inline bool pwmDshotSupportsTelemetryCapture(')
    + fn('src/platform/CH32/pwm_output_dshot.c', 'static inline void pwmDshotConfigGpioPin(')
    + fn('src/platform/CH32/pwm_output_dshot.c', 'FAST_CODE void pwmDshotSetDirectionOutput(')
    + fn('src/platform/CH32/pwm_output_dshot.c', 'FAST_CODE static void pwmDshotSetDirectionInput(')
    + fn('src/platform/CH32/pwm_output_dshot.c', 'FAST_CODE static void motor_DMA_IRQHandler(') + '''
int main(void) {
    /* Pin PE3 (nibble 3 = mask 0xF000) with sibling motor pins PE4..PE6 pre-set to 0xBBB0000 */
    gpioe.CFGLR = 0x0BBBF333U;
    gpioe.CFGHR = 0x88888888U;
    timerHardware_t hw3 = { .tim = TIM3, .channel = TIM_Channel_2, .tag = 3 };
    dmaMotors[0] = (motorDmaOutput_t){ .timerHardware = &hw3, .dmaRef = &dmaCh, .timerDmaSource = 0x02U };

    /* 1. Output mode enables TCIE after clearing stale flags and sets PE3 nibble to 0xB in one atomic write */
    dmaDesc.flags = DMA_IT_GLOB | DMA_IT_TCIF | DMA_IT_HTIF;
    regWriteCount = 0; sawTransientAin = 0;
    pwmDshotSetDirectionOutput(&dmaMotors[0]);
    if (flagsWhenOutputTcEnabled != 0 || clearedIrqN != 47 || (dmaCh.CFGR & DMA_IT_TC) == 0) return 1;
    if (gpioe.CFGLR != 0x0BBBB333U || regWriteCount != 1 || sawTransientAin || atomicEntries == 0) return 2;

    /* 2. TX TC ISR turns line around to Input: disables TCIE, sets PE3 to 0x8 + BSHR pull-up, enables only TIM3 CH2 AUX bit */
    dmaDesc.flags = DMA_IT_TCIF;
    regWriteCount = 0;
    motor_DMA_IRQHandler(&dmaDesc);
    if (!dmaMotors[0].isInput || (dmaCh.CFGR & DMA_IT_TC) != 0 || dmaCh.CNTR != GCR_TELEMETRY_INPUT_LEN) return 3;
    if (gpioe.CFGLR != 0x0BBB8333U || gpioe.BSHR != (1U << 3) || regWriteCount != 1 || sawTransientAin) return 4;
    if (TIM3->AUX != TIM_AUX_CAPCH2_ED || (TIM3->SMCFGR & TIM_TS) != 0) return 5;

    /* 3. Even if 25 edges are captured (CNTR=17) and a spurious TCIF arrives in Input mode, CNTR is NOT reset to 42 */
    dmaCh.CNTR = 17U;
    dmaDesc.flags = DMA_IT_TCIF;
    motor_DMA_IRQHandler(&dmaDesc);
    if (dmaCh.CNTR != 17U) return 6;

    /* 4. 32-bit TIM9 CH1 routes TI1F_ED via SMCFGR.TS without touching unsupported AUX */
    timerHardware_t hw9Ch1 = { .tim = TIM9, .channel = TIM_Channel_1, .tag = 10 };
    dmaMotors[0].timerHardware = &hw9Ch1;
    dmaMotors[0].isInput = false;
    TIM9->AUX = 0;
    TIM9->SMCFGR = 0;
    dmaDesc.flags = DMA_IT_TCIF;
    motor_DMA_IRQHandler(&dmaDesc);
    if (!pwmDshotSupportsTelemetryCapture(TIM9, TIM_Channel_1) || TIM9->AUX != 0 || (TIM9->SMCFGR & TIM_TS) != TIM_TS_TI1F_ED || ((gpioe.CFGHR >> 8) & 0xFU) != 0x8U) return 7;

    /* 5. 32-bit TIM9..TIM12 CH2..CH4 are rejected and never misroute TI1F_ED onto CH2..CH4 */
    if (pwmDshotSupportsTelemetryCapture(TIM9, TIM_Channel_2)
        || pwmDshotSupportsTelemetryCapture(TIM10, TIM_Channel_3)
        || pwmDshotSupportsTelemetryCapture(TIM11, TIM_Channel_4)
        || pwmDshotSupportsTelemetryCapture(TIM12, TIM_Channel_2)) return 8;
    timerHardware_t hw9Ch2 = { .tim = TIM9, .channel = TIM_Channel_2, .tag = 11 };
    dmaMotors[0].timerHardware = &hw9Ch2;
    dmaMotors[0].isInput = false;
    TIM9->SMCFGR = 0;
    gpioe.CFGHR = 0xBBBBBBBBU;
    dmaDesc.flags = DMA_IT_TCIF;
    motor_DMA_IRQHandler(&dmaDesc);
    pwmDshotSetDirectionInput(&dmaMotors[0]);
    if (dmaMotors[0].isInput || TIM9->SMCFGR != 0 || TIM9->AUX != 0 || gpioe.CFGHR != 0xBBBBBBBBU) return 9;

    puts("Timer-driven BiDShot disables input TCIE, clears stale TCIF before TX, sets per-channel AUX, rejects TIM9..12 CH2..4, and updates GPIO atomically");
}
''')

run('usb_msc_bot_abort_and_ep_toggle_mismatch', '''
#define _GNU_SOURCE
#include <sys/mman.h>
#define __IO volatile
#define __PACKED __attribute__((packed))
#define __WEAK __attribute__((weak))
#define UNUSED(x) ((void)(x))
#define USB_NOCACHE_RAM_SECTION
#define USB_MEM_ALIGNX __attribute__((aligned(4)))
#define CONFIG_USB_HS
#define CONFIG_USBDEV_MAX_BUS 1
#define CONFIG_USBDEV_MSC_MAX_LUN 1
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 4096
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING "BETAFLT"
#define CONFIG_USBDEV_MSC_PRODUCT_STRING "Onboard Flash"
#define CONFIG_USBDEV_MSC_VERSION_STRING "    "
#define CONFIG_USBDEV_MSC_POLLING
#define USB_LOG_DBG(...) ((void)0)
#define USB_LOG_WRN(...) ((void)0)
#define USB_LOG_ERR(...) ((void)0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define GET_BE16(p) ((uint16_t)(((uint16_t)(p)[0] << 8) | (uint16_t)(p)[1]))
#define GET_BE32(p) (((uint32_t)(p)[0] << 24) | ((uint32_t)(p)[1] << 16) | ((uint32_t)(p)[2] << 8) | (uint32_t)(p)[3])
#define USB_SPEED_FULL 1
#define USB_SPEED_HIGH 2
#define USB_ENDPOINT_TYPE_BULK 2
#define USB_ENDPOINT_TYPE_ISOCHRONOUS 1
#define USB_REQUEST_DIR_SHIFT 7
#define USB_REQUEST_SET_ADDRESS 5
#define USB_EP_GET_IDX(ep) ((uint8_t)((ep) & 0x7fu))
#define USB_EP_DIR_IS_IN(ep) (((ep) & 0x80u) != 0)
#define USB_EP_DIR_IS_OUT(ep) (((ep) & 0x80u) == 0)
#define USB_GET_MAXPACKETSIZE(w) ((uint16_t)((w) & 0x07ffu))
#define USB_GET_ENDPOINT_TYPE(a) ((uint8_t)((a) & 0x03u))
enum { USBD_EVENT_RESET = 1, USBD_EVENT_INIT = 2, USBD_EVENT_DEINIT = 3, USBD_EVENT_CONFIGURED = 4, USBD_EVENT_CLEAR_HALT = 5 };
struct usb_setup_packet { uint8_t bmRequestType, bRequest; uint16_t wValue, wIndex, wLength; };
struct usb_endpoint_descriptor { uint8_t bLength, bDescriptorType, bEndpointAddress, bmAttributes; uint16_t wMaxPacketSize; uint8_t bInterval; };
struct usbd_endpoint { uint8_t ep_addr; void (*ep_cb)(uint8_t busid, uint8_t ep, uint32_t nbytes); };
struct usbd_interface {
    int (*class_interface_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*class_endpoint_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    int (*vendor_handler)(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len);
    void (*notify_handler)(uint8_t busid, uint8_t event, void *arg);
};
#include "usb_ch32h41x_usbhs_reg.h"
#include "usb_msc.h"
#include "usb_scsi.h"

static struct usbd_endpoint *regOutEp, *regInEp;
static unsigned setupObservedTxCtrl;
static uint32_t mockBlkCount = 16;

static void usbd_add_endpoint(uint8_t busid, struct usbd_endpoint *ep) {
    (void)busid;
    if (ep->ep_addr & 0x80u) regInEp = ep; else regOutEp = ep;
}
static void usbd_event_reset_handler(uint8_t busid) { (void)busid; }
static void usbd_event_suspend_handler(uint8_t busid) { (void)busid; }
static void usbd_event_resume_handler(uint8_t busid) { (void)busid; }
static void usbd_event_ep0_setup_complete_handler(uint8_t busid, uint8_t *s) {
    (void)busid; (void)s;
    setupObservedTxCtrl = ((USBHSD_TypeDef *)0x40030000u)->UEP0_TX_CTRL;
}
static void usbd_event_ep_in_complete_handler(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (ep == 0x81u && regInEp) regInEp->ep_cb(busid, ep, nbytes);
}
static void usbd_event_ep_out_complete_handler(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if (ep == 0x01u && regOutEp) regOutEp->ep_cb(busid, ep, nbytes);
}
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size) {
    (void)busid; (void)lun; *block_num = mockBlkCount; *block_size = 512;
}
bool usbd_msc_get_inquiry(uint8_t busid, uint8_t lun, uint8_t *inquiry_buf) { (void)busid; (void)lun; (void)inquiry_buf; return false; }
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length) {
    (void)busid; (void)lun; (void)sector; (void)buffer; (void)length; return 0;
}
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length) {
    (void)busid; (void)lun; (void)sector; (void)buffer; (void)length; return 0;
}
''' + usbhs_reg_body + usbd_msc_body + '''
int main(void) {
    uint8_t *page = mmap((void *)0x40030000u, 8192, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (page == MAP_FAILED) return 1;
    USBHSD_TypeDef *dev = (USBHSD_TypeDef *)page;

    /* 1. Port speed follows USBHS_UDMS_HS_MOD hardware chirp result */
    dev->MIS_ST = 0;
    if (usbd_get_port_speed(0) != USB_SPEED_FULL) return 2;
    dev->MIS_ST = USBHS_UDMS_HS_MOD;
    if (usbd_get_port_speed(0) != USB_SPEED_HIGH) return 3;

    /* 2. EP0 SETUP does NOT prematurely arm Status-IN before usbd_event_ep0_setup_complete_handler,
     *    and EP0 stall / Status-OUT restore UEP0_DMA to &usb_dc_cfg.setup */
    struct usb_endpoint_descriptor ep0In = { .bEndpointAddress = 0x80, .bmAttributes = 0, .wMaxPacketSize = 64 };
    struct usb_endpoint_descriptor ep0Out = { .bEndpointAddress = 0x00, .bmAttributes = 0, .wMaxPacketSize = 64 };
    struct usb_endpoint_descriptor ep1In = { .bEndpointAddress = 0x81, .bmAttributes = USB_ENDPOINT_TYPE_BULK, .wMaxPacketSize = 512 };
    struct usb_endpoint_descriptor ep1Out = { .bEndpointAddress = 0x01, .bmAttributes = USB_ENDPOINT_TYPE_BULK, .wMaxPacketSize = 512 };
    usbd_ep_open(0, &ep0In);
    usbd_ep_open(0, &ep0Out);
    usbd_ep_open(0, &ep1In);
    usbd_ep_open(0, &ep1Out);
    usb_dc_cfg.setup.bmRequestType = 0x21; /* Host-to-device class request */
    dev->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    dev->UEP0_RX_CTRL = USBHS_UEP_R_DONE | USBHS_UEP_R_SETUP_IS;
    dev->USB_EP0_RX_LEN = 8;
    dev->INT_FG = USBHS_UDIF_TRANSFER;
    dev->INT_ST = 0u;
    USBD_IRQHandler(0);
    if ((setupObservedTxCtrl & USBHS_UEP_T_RES_MASK) != USBHS_UEP_T_RES_NAK) return 4;

    uint32_t dummyAligned = 0;
    usbd_ep_start_write(0, 0x80, (const uint8_t *)&dummyAligned, 4);
    if (dev->UEP0_DMA == (uint32_t)(uintptr_t)&usb_dc_cfg.setup) return 5;
    usbd_ep_start_read(0, 0x00, NULL, 0);
    if (dev->UEP0_DMA != (uint32_t)(uintptr_t)&usb_dc_cfg.setup) return 6;
    usbd_ep_start_write(0, 0x80, (const uint8_t *)&dummyAligned, 4);
    usbd_ep_set_stall(0, 0x80);
    if (dev->UEP0_DMA != (uint32_t)(uintptr_t)&usb_dc_cfg.setup) return 7;

    /* 3. Duplicate Bulk-OUT packet (TOG_MATCH == 0) re-arms RX_VALID instead of deadlocking in NAK */
    struct usbd_interface intf = {0};
    usbd_msc_init_intf(0, &intf, 0x01, 0x81);
    usbd_msc_set_readonly(0, true);
    msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);
    dev->UEP1_RX_LEN = 31;
    dev->UEP1_RX_CTRL = USBHS_UEP_R_DONE; /* TOG_MATCH == 0! */
    dev->INT_FG = USBHS_UDIF_TRANSFER;
    dev->INT_ST = 1u;
    USBD_IRQHandler(0);
    if ((dev->UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) != USBHS_UEP_R_RES_ACK || g_usbd_msc[0].stage != MSC_READ_CBW) return 8;

    /* 4. Rogue 512B HS packet into CBW buffer does NOT corrupt readonly/csw and wedges into MSC_BS_RESET */
    memset(g_usbd_msc[0].cbw_buf, 0xA5, 512);
    mass_storage_bulk_out(0, 0x01, 512);
    uint8_t stalledIn = 0, stalledOut = 0;
    usbd_ep_is_stalled(0, 0x81, &stalledIn);
    usbd_ep_is_stalled(0, 0x01, &stalledOut);
    if (!g_usbd_msc[0].readonly || g_usbd_msc[0].stage != MSC_BS_RESET || !stalledIn || !stalledOut) return 9;
    /* Premature CLEAR_HALT before MSC_REQUEST_RESET must re-stall! */
    usbd_ep_clear_stall(0, 0x81);
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x81);
    usbd_ep_is_stalled(0, 0x81, &stalledIn);
    if (!stalledIn) return 10;
    /* Reset Recovery: MSC_REQUEST_RESET -> CLEAR_HALT(IN) -> CLEAR_HALT(OUT) */
    struct usb_setup_packet rstReq = { .bRequest = MSC_REQUEST_RESET, .wValue = 0, .wLength = 0 };
    uint8_t *dp = NULL; uint32_t dl = 0;
    if (intf.class_interface_handler(0, &rstReq, &dp, &dl) != 0) return 11;
    usbd_ep_clear_stall(0, 0x81);
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x81);
    usbd_ep_clear_stall(0, 0x01);
    if ((dev->UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) != USBHS_UEP_R_RES_NAK) return 12;
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x01);
    if ((dev->UEP1_RX_CTRL & USBHS_UEP_R_RES_MASK) != USBHS_UEP_R_RES_ACK || g_usbd_msc[0].stage != MSC_READ_CBW) return 13;

    /* 5. READ CAPACITY(16) (0x9E) stalls Bulk-IN, sends CSW_STATUS_CMD_FAILED after CLEAR_HALT, and sets INVALIDCOMMAND */
    struct CBW cbw = { .dSignature = MSC_CBW_Signature, .dTag = 99, .dDataLength = 32, .bmFlags = 0x80, .bLUN = 0, .bCBLength = 16 };
    cbw.CB[0] = SCSI_CMD_READCAPACITY16;
    memcpy(&g_usbd_msc[0].cbw, &cbw, sizeof(cbw));
    mass_storage_bulk_out(0, 0x01, sizeof(cbw));
    usbd_ep_is_stalled(0, 0x81, &stalledIn);
    if (g_usbd_msc[0].stage != MSC_BS_ABORT_IN || !stalledIn) return 14;
    usbd_ep_clear_stall(0, 0x81);
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x81);
    if (g_usbd_msc[0].stage != MSC_WAIT_CSW || g_usbd_msc[0].csw.bStatus != CSW_STATUS_CMD_FAILED || g_usbd_msc[0].sKey != 0x05) return 15;
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    /* 6. VERIFY(10) succeeds for BYTCHK=0 in-bounds LBA and rejects BYTCHK=1 with INVALIDFIELDINCBA */
    struct CBW verCbw = { .dSignature = MSC_CBW_Signature, .dTag = 100, .dDataLength = 0, .bmFlags = 0x00, .bLUN = 0, .bCBLength = 10 };
    verCbw.CB[0] = SCSI_CMD_VERIFY10;
    verCbw.CB[5] = 2; verCbw.CB[8] = 4;
    memcpy(&g_usbd_msc[0].cbw, &verCbw, sizeof(verCbw));
    mass_storage_bulk_out(0, 0x01, sizeof(verCbw));
    if (g_usbd_msc[0].stage != MSC_WAIT_CSW || g_usbd_msc[0].csw.bStatus != CSW_STATUS_CMD_PASSED) return 16;
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);
    verCbw.CB[1] = SCSICMD_VERIFY10_BYTCHK;
    verCbw.dDataLength = 512;
    memcpy(&g_usbd_msc[0].cbw, &verCbw, sizeof(verCbw));
    mass_storage_bulk_out(0, 0x01, sizeof(verCbw));
    if (g_usbd_msc[0].stage != MSC_BS_ABORT_OUT || g_usbd_msc[0].sKey != 0x05 || g_usbd_msc[0].ASC != 0x24) return 16;
    usbd_ep_clear_stall(0, 0x01);
    msc_storage_notify_handler(0, USBD_EVENT_CLEAR_HALT, (void *)(uintptr_t)0x01);
    mass_storage_bulk_in(0, 0x81, USB_SIZEOF_MSC_CSW);

    /* 7. Zero-block media does NOT underflow READ CAPACITY(10) to 0xFFFFFFFF or hang init */
    mockBlkCount = 0;
    usbd_msc_init_intf(0, &intf, 0x01, 0x81);
    msc_storage_notify_handler(0, USBD_EVENT_CONFIGURED, NULL);
    struct CBW cap10 = { .dSignature = MSC_CBW_Signature, .dTag = 101, .dDataLength = 8, .bmFlags = 0x80, .bLUN = 0, .bCBLength = 10 };
    cap10.CB[0] = SCSI_CMD_READCAPACITY10;
    memcpy(&g_usbd_msc[0].cbw, &cap10, sizeof(cap10));
    mass_storage_bulk_out(0, 0x01, sizeof(cap10));
    if (g_usbd_msc[0].stage != MSC_BS_ABORT_IN || g_usbd_msc[0].sKey != 0x02 || g_usbd_msc[0].ASC != 0x3a) return 17;

    puts("USBHS toggle mismatch, EP0 DMA/stall safety, 512B CBW guard, BOT Reset-Recovery/Abort-IN, VERIFY(10), and 0-block media verified");
}
''', extra_flags=('-I' + str(root / 'lib/modules/CH32H41x/middlewares/ch32h41x_hs'), '-I' + str(root / 'lib/modules/CH32H41x/middlewares/class/msc')))

run('usb_vcp_cold_boot_fs_mps_and_rx_nak', '''
#define UNUSED(x) ((void)(x))
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define NVIC_PRIO_USB 0
#define ATOMIC_BLOCK(x) for (int once = 1; once; once = 0)
#define USB_TIMEOUT 50
#define APP_TX_DATA_SIZE 8192
#define APP_TX_BLOCK_SIZE 512
#define RX_BUFFER_SIZE 2048
#define TX_BUFFER_SIZE 512
#define CDC_IN_EP 0x82
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83
#define CDC_MAX_MPS 512
#define USBD_VID 0x1A86
#define USBD_PID 0xE204
#define USBD_MAX_POWER 100
#define USBD_PRODUCT_STRING "Betaflight CH32H41x"
#define CTRL_LINE_STATE_DTR 1
static uint32_t uid0 = 0x12345678u, uid1 = 0x9ABCDEF0u, uid2 = 0x0FEDCBA9u;
#define U_ID_0 uid0
#define U_ID_1 uid1
#define U_ID_2 uid2

#include "usb_util.h"
#include "usb_def.h"
#include "usbd_cdc.h"
#define USB_CONFIG_SIZE (9 + CDC_DESCRIPTOR_LEN)

enum {
    USBD_EVENT_RESET = 1,
    USBD_EVENT_CONNECTED = 3,
    USBD_EVENT_DISCONNECTED = 4,
    USBD_EVENT_SUSPEND = 5,
    USBD_EVENT_RESUME = 6,
    USBD_EVENT_CONFIGURED = 7,
    USBD_EVENT_SET_REMOTE_WAKEUP = 9,
    USBD_EVENT_CLR_REMOTE_WAKEUP = 10,
    USBD_EVENT_INIT = 11,
    USBD_EVENT_DEINIT = 12,
    USBD_EVENT_UNKNOWN = 14,
};

typedef uint32_t timeUs_t;
typedef struct { int dummy; } serialPort_t;
static struct { uint32_t txAt; } vcpPort;
static volatile uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
static volatile uint32_t UserTxBufPtrIn, UserTxBufPtrOut, lastBuffsize;
static volatile bool usbZlpPending, usbTxStalled, usbHostPortOpen;
static volatile uint8_t ep_tx_busy_flag, usbd_cdc_info = USBD_EVENT_UNKNOWN, cdc_configured, cdc_connected, rx_ep_paused;
static volatile uint16_t rx_head, rx_tail;
static uint8_t usb_drivers_buffer[CDC_MAX_MPS], read_buffer[RX_BUFFER_SIZE], write_buffer[TX_BUFFER_SIZE];
static void (*ctrlLineStateCb)(void *, uint16_t);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *, uint32_t);
static void *baudRateCbContext;
static void (*serialBaudRateCb)(serialPort_t *, uint32_t);

static uint16_t simEpMps = 64;
static bool readArmed;
static uint32_t lastReadMps, zlpSent, dataSent;
static uint16_t usbd_get_ep_mps(uint8_t busid, uint8_t ep) { (void)busid; (void)ep; return simEpMps; }
static int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t len) {
    (void)busid; (void)data;
    if (ep != CDC_OUT_EP || readArmed) return -1;
    readArmed = true;
    lastReadMps = len;
    return 0;
}
static int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t len) {
    (void)busid; (void)ep; (void)data;
    if (len == 0) zlpSent++; else dataSent++;
    return 0;
}

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x04)
};
static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x04)
};
static const uint8_t other_speed_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x04)
};
static const uint8_t other_speed_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x04)
};
static char cdcSerialString[25];
static const char *const string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, "Betaflight", USBD_PRODUCT_STRING, NULL, "WCH HS Virtual COM Port",
};
void usbVcpResetState(void);
void usbVcpTxDoneISR(void);
uint16_t usb_vcp_send_data(uint8_t busid, uint8_t *buffer, uint16_t len);
''' + fn('src/platform/CH32/usb_ch32h41x.h', 'static inline const char *ch32UsbGetUidSerialString(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static const uint8_t *config_descriptor_callback(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static const uint8_t *other_speed_descriptor_callback(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static const char *string_descriptor_callback(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static uint16_t usbVcpOutMps(')
    + '#define USB_VCP_OUT_MPS(busid) (usbVcpOutMps(busid))\n'
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static uint16_t usbVcpRxUsedLocked(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static uint16_t usbVcpRxFreeLocked(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'static void usbd_event_handler(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'void usbd_cdc_acm_bulk_out(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'void usbd_cdc_acm_bulk_in(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_rx_available(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_get_rx_data(')
    + fn('lib/modules/CH32H41x/middlewares/board/cdc_vcp_ch32h41x.c', 'uint16_t usb_vcp_send_data(')
    + fn(usb, 'static uint16_t usbVcpBulkInMps(')
    + '#define USB_VCP_BULK_IN_MPS (usbVcpBulkInMps())\n'
    + fn(usb, 'void CDC_SetBaudRateCb(')
    + fn(usb, 'void usbVcpBaudRateChanged(')
    + fn(usb, 'uint8_t usbIsConnected(void)')
    + fn(usb, 'uint8_t usbIsConfigured(void)')
    + fn(usb, 'uint8_t usbVcpIsConnected(void)')
    + fn(usb, 'uint8_t usbVcpIsActive(void)')
    + fn(usb, 'void usbVcpResetState(void)')
    + fn(usb, 'static void usbVcpKickTx(void)')
    + fn(usb, 'void usbVcpTxDoneISR(void)')
    + fn(usb, 'static void usbVcpSerialBaudRateTrampoline(')
    + fn(usb, 'static void usbVcpSetBaudRateCb(') + '''
static serialPort_t *seenPort;
static uint32_t seenTypedBaud;
static void onSerialBaud(serialPort_t *ctx, uint32_t baud) { seenPort = ctx; seenTypedBaud = baud; }

static uint16_t descBulkMps(const uint8_t *p) {
    uint32_t total = p[2] | ((uint32_t)p[3] << 8);
    uint32_t off = 0;
    while (off + 7 <= total && p[off] != 0) {
        if (p[off + 1] == USB_DESCRIPTOR_TYPE_ENDPOINT && p[off + 2] == CDC_OUT_EP) {
            return (uint16_t)(p[off + 4] | ((uint16_t)p[off + 5] << 8));
        }
        off += p[off];
    }
    return 0;
}

int main(void) {
    /* 1. Cold LiPo boot without USB cable: INIT + idle SUSPEND/RESUME must keep usbVcpIsConnected() == 0 */
    if (usbVcpIsConnected() || usbVcpIsActive()) return 1;
    usbd_event_handler(0, USBD_EVENT_INIT);
    usbd_event_handler(0, USBD_EVENT_SUSPEND);
    usbd_event_handler(0, USBD_EVENT_RESUME);
    if (usbVcpIsConnected() || usbVcpIsActive()) return 2;

    /* 2. Host cable connected at Full-Speed (64B MPS): RESET marks connected, CONFIGURED arms 64B OUT & 64B ZLP */
    simEpMps = 64;
    readArmed = false;
    usbd_event_handler(0, USBD_EVENT_RESET);
    if (!usbVcpIsConnected() || usbVcpIsActive()) return 3;
    usbd_event_handler(0, USBD_EVENT_CONFIGURED);
    if (!usbVcpIsConnected() || !usbVcpIsActive() || !readArmed || lastReadMps != 64) return 4;
    UserTxBufPtrIn = 64;
    usbVcpKickTx();
    if (dataSent != 1 || zlpSent != 0) return 5;
    usbd_cdc_acm_bulk_in(0, CDC_IN_EP, 64);
    if (zlpSent != 1) return 6;

    /* 3. Speed-aware config/other-speed descriptors + 24-hex UID serial + UBSan-clean baud trampoline */
    const uint8_t *cfgHs = config_descriptor_callback(USB_SPEED_HIGH);
    const uint8_t *cfgFs = config_descriptor_callback(USB_SPEED_FULL);
    const uint8_t *othHs = other_speed_descriptor_callback(USB_SPEED_HIGH);
    const uint8_t *othFs = other_speed_descriptor_callback(USB_SPEED_FULL);
    if (descBulkMps(cfgHs) != 512 || descBulkMps(cfgFs) != 64) return 7;
    if (othHs[1] != USB_DESCRIPTOR_TYPE_OTHER_SPEED || descBulkMps(othHs) != 64) return 8;
    if (othFs[1] != USB_DESCRIPTOR_TYPE_OTHER_SPEED || descBulkMps(othFs) != 512) return 9;
    const char *ser = string_descriptor_callback(USB_SPEED_HIGH, USB_STRING_SERIAL_INDEX);
    if (!ser || strcmp(ser, "123456789ABCDEF00FEDCBA9") != 0) return 10;
    serialPort_t portObj = { .dummy = 42 };
    usbVcpSetBaudRateCb(&portObj, onSerialBaud, &portObj);
    usbVcpBaudRateChanged(921600);
    if (seenPort != &portObj || seenTypedBaud != 921600) return 11;

    /* 4. Lossless Bulk-OUT NAK backpressure across 4096B (2x RX_BUFFER_SIZE) under HS 512B bursts */
    simEpMps = 512;
    readArmed = false;
    usbd_event_handler(0, USBD_EVENT_CONFIGURED);
    if (!readArmed || lastReadMps != 512) return 12;
    uint8_t src[4096], dst[4096];
    for (uint32_t i = 0; i < sizeof(src); i++) src[i] = (uint8_t)((i * 73u + 19u) & 0xFFu);
    uint32_t txOff = 0, rxOff = 0, pausesSeen = 0;
    while (rxOff < sizeof(dst)) {
        while (readArmed && txOff < sizeof(src)) {
            uint32_t n = sizeof(src) - txOff;
            if (n > 512) n = 512;
            readArmed = false;
            memcpy(usb_drivers_buffer, &src[txOff], n);
            txOff += n;
            usbd_cdc_acm_bulk_out(0, CDC_OUT_EP, n);
            if (!readArmed) pausesSeen++;
        }
        uint16_t got = usb_vcp_get_rx_data(0, &dst[rxOff], 200);
        if (got == 0) return 13;
        rxOff += got;
    }
    if (pausesSeen == 0 || memcmp(src, dst, sizeof(src)) != 0 || usb_vcp_rx_available() != 0) return 14;

    puts("cold-boot VCP disconnect state, FS/HS descriptors & ZLP, UID serial, baud trampoline, and lossless RX NAK backpressure verified");
}
''', extra_flags=('-I' + str(root / 'lib/modules/CH32H41x/middlewares/common'), '-I' + str(root / 'lib/modules/CH32H41x/middlewares/class/cdc')))

run('clock_cycles_and_target_io_masks', '''
static uint32_t SystemCoreClock = 350000000u;
#define CYCLE_TICKS_PER_US (SystemCoreClock / 1000000)
''' + fn('src/platform/CH32/system_ch32h41x.c', 'int32_t clockCyclesToMicros(')
    + fn('src/platform/CH32/system_ch32h41x.c', 'int32_t clockCyclesTo10thMicros(')
    + fn('src/platform/CH32/system_ch32h41x.c', 'int32_t clockCyclesTo100thMicros(') + '''
#include "target.h"
static const char boardId[] = TARGET_BOARD_IDENTIFIER;
int main(void) {
    if (clockCyclesToMicros(-700) != -2 || clockCyclesToMicros(700) != 2) return 1;
    if (clockCyclesTo10thMicros(-350) != -10 || clockCyclesTo100thMicros(-350) != -100) return 2;
    if (clockCyclesTo100thMicros(INT32_MAX) <= 0 || clockCyclesTo100thMicros(INT32_MIN + 1) >= 0) return 3;
    if (strlen(boardId) != 4 || strcmp(boardId, "H415") != 0) return 4;
    if (TARGET_IO_PORTA != 0xffff || TARGET_IO_PORTB != 0xffff || TARGET_IO_PORTC != 0xffff) return 5;
    if (TARGET_IO_PORTD != 0xffff || TARGET_IO_PORTE != 0xffff || TARGET_IO_PORTF != 0xffff) return 6;
    puts("signed cycle conversions, 4-char H415 board ID, and full generic IO masks verified");
}
''', extra_flags=('-I' + str(root / 'src/platform/CH32/target/CH32H415'),))

import importlib.util
spec = importlib.util.spec_from_file_location("split_image", root / "src/utils/ch32h415_split_image.py")
split_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(split_mod)
cfg_bytes = bytes((i * 13) & 255 for i in range(split_mod.CONFIG_END - split_mod.CONFIG_START))
flash_img = bytearray([0xff] * (split_mod.END - split_mod.BASE))
flash_img[split_mod.CONFIG_START - split_mod.BASE:split_mod.CONFIG_END - split_mod.BASE] = cfg_bytes
raw_img = bytes([0x42] * (split_mod.CONFIG_START - split_mod.BASE)) + bytes(len(cfg_bytes)) + bytes([0x91] * 1234)
parts = split_mod.split_image(raw_img)
assert [p[0] for p in parts] == [split_mod.CONFIG_END, split_mod.BASE]
for address, payload, _ in parts:
    flash_img[address - split_mod.BASE:address - split_mod.BASE + len(payload)] = payload
assert flash_img[split_mod.CONFIG_START - split_mod.BASE:split_mod.CONFIG_END - split_mod.BASE] == cfg_bytes
for bad_img in (b"", raw_img[:split_mod.CONFIG_END - split_mod.BASE],
                raw_img[:split_mod.CONFIG_START - split_mod.BASE] + b"\x01" + raw_img[split_mod.CONFIG_START - split_mod.BASE + 1:],
                bytes(split_mod.END - split_mod.BASE + 1)):
    try:
        split_mod.split_image(bad_img)
    except ValueError:
        continue
    raise AssertionError("invalid split image accepted")
print("split_image_layout PASS preserves live configuration sectors, flashes code before entry, and rejects malformed images")
