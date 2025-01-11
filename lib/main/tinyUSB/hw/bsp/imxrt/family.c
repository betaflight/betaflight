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
 * This file is part of the TinyUSB stack.
 */

#include "bsp/board_api.h"
#include "board/clock_config.h"
#include "board/pin_mux.h"
#include "board.h"

// Suppress warning caused by mcu driver
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "fsl_clock.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "fsl_ocotp.h"

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif

/* --- Note about USB buffer RAM ---
  For M7 core it's recommended to put USB buffer in DTCM for better performance (flexspi_nor linker default)
  Otherwise you have to put the buffer in a non-cacheable section by configurate MPU manually or using BOARD_ConfigMPU():
  - Define CFG_TUSB_MEM_SECTION=__attribute__((section("NonCacheable")))
  - (IAR only) Change __NCACHE_REGION_SIZE in linker script to cover the size of non-cacheable section, multiple of 2^N

  For secondary M4 core, the USB controller doesn't support transfer from DTCM so OCRAM must be used:
  - __NCACHE_REGION_SIZE is defined by the linker script by default
  - Define CFG_TUSB_MEM_SECTION=__attribute__((section("NonCacheable")))
*/

static void BOARD_ConfigMPU(void);

// needed by fsl_flexspi_nor_boot
TU_ATTR_USED const uint8_t dcd_data[] = {0x00};

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// unify naming convention
#if !defined(USBPHY1) && defined(USBPHY)
  #define USBPHY1 USBPHY
#endif

static void init_usb_phy(uint8_t usb_id) {
  USBPHY_Type *usb_phy;

  if (usb_id == 0) {
    usb_phy = USBPHY1;
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, BOARD_XTAL0_CLK_HZ);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, BOARD_XTAL0_CLK_HZ);
  }
#ifdef USBPHY2
  else if (usb_id == 1) {
    usb_phy = USBPHY2;
    CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, BOARD_XTAL0_CLK_HZ);
    CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, BOARD_XTAL0_CLK_HZ);
  }
#endif
  else {
    return;
  }

  // Enable PHY support for Low speed device + LS via FS Hub
  usb_phy->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK | USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;

  // Enable all power for normal operation
  // TODO may not be needed since it is called within CLOCK_EnableUsbhs0PhyPllClock()
  usb_phy->PWD = 0;

  // TX Timing
  uint32_t phytx = usb_phy->TX;
  phytx &= ~(USBPHY_TX_D_CAL_MASK | USBPHY_TX_TXCAL45DM_MASK | USBPHY_TX_TXCAL45DP_MASK);
  phytx |= USBPHY_TX_D_CAL(0x0C) | USBPHY_TX_TXCAL45DP(0x06) | USBPHY_TX_TXCAL45DM(0x06);
  usb_phy->TX = phytx;
}

void board_init(void) {
  BOARD_ConfigMPU();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  SystemCoreClockUpdate();

#ifdef TRACE_ETM
  //CLOCK_EnableClock(kCLOCK_Trace);
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_OTG1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  #ifdef USBPHY2
  NVIC_SetPriority(USB_OTG2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  #endif
#endif

  board_led_write(true);

  // UART
  lpuart_config_t uart_config;
  LPUART_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx = true;
  uart_config.enableRx = true;

  if (kStatus_Success != LPUART_Init(UART_PORT, &uart_config, UART_CLK_ROOT)) {
    // failed to init uart, probably baudrate is not supported
    // TU_BREAKPOINT();
  }

  //------------- USB -------------//
  // Note: RT105x RT106x and later have dual USB controllers.
  init_usb_phy(0);// USB0
#ifdef USBPHY2
  init_usb_phy(1);// USB1
#endif
}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USB_OTG1_IRQHandler(void) {
  tusb_int_handler(0, true);
}

void USB_OTG2_IRQHandler(void) {
  tusb_int_handler(1, true);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_PinWrite(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == GPIO_PinRead(BUTTON_PORT, BUTTON_PIN);
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;

#if FSL_FEATURE_OCOTP_HAS_TIMING_CTRL
  OCOTP_Init(OCOTP, CLOCK_GetFreq(kCLOCK_IpgClk));
#else
  OCOTP_Init(OCOTP, 0u);
#endif

  // Reads shadow registers 0x01 - 0x04 (Configuration and Manufacturing Info)
  // into 8 bit wide destination, avoiding punning.
  for (int i = 0; i < 4; ++i) {
    uint32_t wr = OCOTP_ReadFuseShadowRegister(OCOTP, i + 1);
    for (int j = 0; j < 4; j++) {
      id[i * 4 + j] = wr & 0xff;
      wr >>= 8;
    }
  }
  OCOTP_Deinit(OCOTP);

  return 16;
}

int board_uart_read(uint8_t *buf, int len) {
  int count = 0;

  while (count < len) {
    uint8_t const rx_count = LPUART_GetRxFifoCount(UART_PORT);
    if (!rx_count) {
      // clear all error flag if any
      uint32_t status_flags = LPUART_GetStatusFlags(UART_PORT);
      status_flags &= (kLPUART_RxOverrunFlag | kLPUART_ParityErrorFlag | kLPUART_FramingErrorFlag |
                       kLPUART_NoiseErrorFlag);
      LPUART_ClearStatusFlags(UART_PORT, status_flags);
      break;
    }

    for (int i = 0; i < rx_count; i++) {
      buf[count] = LPUART_ReadByte(UART_PORT);
      count++;
    }
  }

  return count;
}

int board_uart_write(void const *buf, int len) {
  LPUART_WriteBlocking(UART_PORT, (uint8_t const *) buf, len);
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif


#ifndef __ICCARM__
// Implement _start() since we use linker flag '-nostartfiles'.
// Requires defined __STARTUP_CLEAR_BSS,
extern int main(void);
TU_ATTR_UNUSED void _start(void) {
  // called by startup code
  main();
  while (1) {}
}

#ifdef __clang__
void _exit(int __status) {
  while (1) {}
}
#endif
#endif

//--------------------------------------------------------------------
// MPU configuration
//--------------------------------------------------------------------
#if __CORTEX_M == 7
static void BOARD_ConfigMPU(void) {
  #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$RW_m_ncache$$Base[];
  /* RW_m_ncache_unused is a auxiliary region which is used to get the whole size of noncache section */
  extern uint32_t Image$$RW_m_ncache_unused$$Base[];
  extern uint32_t Image$$RW_m_ncache_unused$$ZI$$Limit[];
  uint32_t nonCacheStart = (uint32_t) Image$$RW_m_ncache$$Base;
  uint32_t size = ((uint32_t) Image$$RW_m_ncache_unused$$Base == nonCacheStart) ? 0 : ((uint32_t) Image$$RW_m_ncache_unused$$ZI$$Limit - nonCacheStart);
  #elif defined(__MCUXPRESSO)
    #if defined(__USE_SHMEM)
  extern uint32_t __base_rpmsg_sh_mem;
  extern uint32_t __top_rpmsg_sh_mem;
  uint32_t nonCacheStart = (uint32_t) (&__base_rpmsg_sh_mem);
  uint32_t size = (uint32_t) (&__top_rpmsg_sh_mem) - nonCacheStart;
    #else
  extern uint32_t __base_NCACHE_REGION;
  extern uint32_t __top_NCACHE_REGION;
  uint32_t nonCacheStart = (uint32_t) (&__base_NCACHE_REGION);
  uint32_t size = (uint32_t) (&__top_NCACHE_REGION) - nonCacheStart;
    #endif
  #elif defined(__ICCARM__) || defined(__GNUC__)
  extern uint32_t __NCACHE_REGION_START[];
  extern uint32_t __NCACHE_REGION_SIZE[];
  uint32_t nonCacheStart = (uint32_t) __NCACHE_REGION_START;
  uint32_t size = (uint32_t) __NCACHE_REGION_SIZE;
  #endif
  volatile uint32_t i = 0;

  #if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
  /* Disable I cache and D cache */
  if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {
    SCB_DisableICache();
  }
  #endif
  #if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
  if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {
    SCB_DisableDCache();
  }
  #endif

  /* Disable MPU */
  ARM_MPU_Disable();

  /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable,
     * SubRegionDisable, Size)
     * API in mpu_armv7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches
     * disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and
     * Privileged mode.
     *      Use MACROS defined in mpu_armv7.h:
     * ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribute    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write
     * through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write
     * back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write
     * through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write
     * back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner
     * noncache
     *     1             1           0           0              Normal             shareable       outer and inner
     * noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write
     * back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write
     * back write/read acllocate
     *     2             x           0           0              Device              not shareable
     *  Above are normal use settings, if your want to see more details or want to config different inner/outer cache
     * policy.
     *  please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in
     * mpu_armv7.h.
     */

  /*
     * Add default region to deny access to whole address space to workaround speculative prefetch.
     * Refer to Arm errata 1013783-B for more details.
     *
     */
  /* Region 0 setting: Instruction access disabled, No data access permission. */
  MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);
  MPU->RASR = ARM_MPU_RASR(1, ARM_MPU_AP_NONE, 0, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_4GB);

  /* Region 1 setting: Memory with Device type, not shareable, non-cacheable. */
  MPU->RBAR = ARM_MPU_RBAR(1, 0x80000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

  /* Region 2 setting: Memory with Device type, not shareable,  non-cacheable. */
  MPU->RBAR = ARM_MPU_RBAR(2, 0x60000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

  /* Region 3 setting: Memory with Device type, not shareable, non-cacheable. */
  MPU->RBAR = ARM_MPU_RBAR(3, 0x00000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

  /* Region 4 setting: Memory with Normal type, not shareable, outer/inner write back */
  MPU->RBAR = ARM_MPU_RBAR(4, 0x00000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

  /* Region 5 setting: Memory with Normal type, not shareable, outer/inner write back */
  MPU->RBAR = ARM_MPU_RBAR(5, 0x20000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

  #if defined(CACHE_MODE_WRITE_THROUGH) && CACHE_MODE_WRITE_THROUGH
  /* Region 6 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(6, 0x20200000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_1MB);

  /* Region 7 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(7, 0x20300000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_512KB);
  #else
  /* Region 6 setting: Memory with Normal type, not shareable, outer/inner write back */
  MPU->RBAR = ARM_MPU_RBAR(6, 0x20200000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_1MB);

  /* Region 7 setting: Memory with Normal type, not shareable, outer/inner write back */
  MPU->RBAR = ARM_MPU_RBAR(7, 0x20300000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_512KB);
  #endif

  #if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
  /* Region 8 setting: Memory with Normal type, not shareable, outer/inner write back. */
  MPU->RBAR = ARM_MPU_RBAR(8, 0x30000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_RO, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_16MB);
  #endif

  #ifdef USE_SDRAM
    #if defined(CACHE_MODE_WRITE_THROUGH) && CACHE_MODE_WRITE_THROUGH
  /* Region 9 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(9, 0x80000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_64MB);
    #else
  /* Region 9 setting: Memory with Normal type, not shareable, outer/inner write back */
  MPU->RBAR = ARM_MPU_RBAR(9, 0x80000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_64MB);
    #endif
  #endif

  while ((size >> i) > 0x1U) {
    i++;
  }

  if (i != 0) {
    /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
    assert(!(nonCacheStart % size));
    assert(size == (uint32_t) (1 << i));
    assert(i >= 5);

    /* Region 10 setting: Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(10, nonCacheStart);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, i - 1);
  }

  /* Region 11 setting: Memory with Device type, not shareable, non-cacheable */
  MPU->RBAR = ARM_MPU_RBAR(11, 0x40000000);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_16MB);

  /* Region 12 setting: Memory with Device type, not shareable, non-cacheable */
  MPU->RBAR = ARM_MPU_RBAR(12, 0x41000000);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_2MB);

  /* Region 13 setting: Memory with Device type, not shareable, non-cacheable */
  MPU->RBAR = ARM_MPU_RBAR(13, 0x41400000);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1MB);

  /* Region 14 setting: Memory with Device type, not shareable, non-cacheable */
  MPU->RBAR = ARM_MPU_RBAR(14, 0x41800000);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_2MB);

  /* Region 15 setting: Memory with Device type, not shareable, non-cacheable */
  MPU->RBAR = ARM_MPU_RBAR(15, 0x42000000);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1MB);

  /* Enable MPU */
  ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk);

  /* Enable I cache and D cache */
  #if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
  SCB_EnableDCache();
  #endif
  #if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
  SCB_EnableICache();
  #endif
}

#elif __CORTEX_M == 4

void BOARD_ConfigMPU(void) {
  #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$RW_m_ncache$$Base[];
  /* RW_m_ncache_unused is a auxiliary region which is used to get the whole size of noncache section */
  extern uint32_t Image$$RW_m_ncache_unused$$Base[];
  extern uint32_t Image$$RW_m_ncache_unused$$ZI$$Limit[];
  uint32_t nonCacheStart = (uint32_t) Image$$RW_m_ncache$$Base;
  uint32_t nonCacheSize = ((uint32_t) Image$$RW_m_ncache_unused$$Base == nonCacheStart) ? 0 : ((uint32_t) Image$$RW_m_ncache_unused$$ZI$$Limit - nonCacheStart);
  #elif defined(__MCUXPRESSO)
  extern uint32_t __base_NCACHE_REGION;
  extern uint32_t __top_NCACHE_REGION;
  uint32_t nonCacheStart = (uint32_t) (&__base_NCACHE_REGION);
  uint32_t nonCacheSize = (uint32_t) (&__top_NCACHE_REGION) - nonCacheStart;
  #elif defined(__ICCARM__) || defined(__GNUC__)
  extern uint32_t __NCACHE_REGION_START[];
  extern uint32_t __NCACHE_REGION_SIZE[];
  uint32_t nonCacheStart = (uint32_t) __NCACHE_REGION_START;
  uint32_t nonCacheSize = (uint32_t) __NCACHE_REGION_SIZE;
  #endif
  #if defined(__USE_SHMEM)
    #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  extern uint32_t Image$$RPMSG_SH_MEM$$Base[];
  /* RPMSG_SH_MEM_unused is a auxiliary region which is used to get the whole size of RPMSG_SH_MEM section */
  extern uint32_t Image$$RPMSG_SH_MEM_unused$$Base[];
  extern uint32_t Image$$RPMSG_SH_MEM_unused$$ZI$$Limit[];
  uint32_t rpmsgShmemStart = (uint32_t) Image$$RPMSG_SH_MEM$$Base;
  uint32_t rpmsgShmemSize = (uint32_t) Image$$RPMSG_SH_MEM_unused$$ZI$$Limit - rpmsgShmemStart;
    #elif defined(__MCUXPRESSO)
  extern uint32_t __base_rpmsg_sh_mem;
  extern uint32_t __top_rpmsg_sh_mem;
  uint32_t rpmsgShmemStart = (uint32_t) (&__base_rpmsg_sh_mem);
  uint32_t rpmsgShmemSize = (uint32_t) (&__top_rpmsg_sh_mem) - rpmsgShmemStart;
    #elif defined(__ICCARM__) || defined(__GNUC__)
  extern uint32_t __RPMSG_SH_MEM_START[];
  extern uint32_t __RPMSG_SH_MEM_SIZE[];
  uint32_t rpmsgShmemStart = (uint32_t) __RPMSG_SH_MEM_START;
  uint32_t rpmsgShmemSize = (uint32_t) __RPMSG_SH_MEM_SIZE;
    #endif
  #endif
  uint32_t i = 0;

  /* Only config non-cacheable region on system bus */
  assert(nonCacheStart >= 0x20000000);

  /* Disable code bus cache */
  if (LMEM_PCCCR_ENCACHE_MASK == (LMEM_PCCCR_ENCACHE_MASK & LMEM->PCCCR)) {
    /* Enable the processor code bus to push all modified lines. */
    LMEM->PCCCR |= LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK | LMEM_PCCCR_GO_MASK;
    /* Wait until the cache command completes. */
    while ((LMEM->PCCCR & LMEM_PCCCR_GO_MASK) != 0U) {
    }
    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PCCCR &= ~(LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK);
    /* Now disable the cache. */
    LMEM->PCCCR &= ~LMEM_PCCCR_ENCACHE_MASK;
  }

  /* Disable system bus cache */
  if (LMEM_PSCCR_ENCACHE_MASK == (LMEM_PSCCR_ENCACHE_MASK & LMEM->PSCCR)) {
    /* Enable the processor system bus to push all modified lines. */
    LMEM->PSCCR |= LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK | LMEM_PSCCR_GO_MASK;
    /* Wait until the cache command completes. */
    while ((LMEM->PSCCR & LMEM_PSCCR_GO_MASK) != 0U) {
    }
    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PSCCR &= ~(LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK);
    /* Now disable the cache. */
    LMEM->PSCCR &= ~LMEM_PSCCR_ENCACHE_MASK;
  }

  /* Disable MPU */
  ARM_MPU_Disable();

  #if defined(CACHE_MODE_WRITE_THROUGH) && CACHE_MODE_WRITE_THROUGH
  /* Region 0 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(0, 0x20200000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_1MB);

  /* Region 1 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(1, 0x20300000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_512KB);

  /* Region 2 setting: Memory with Normal type, not shareable, write through */
  MPU->RBAR = ARM_MPU_RBAR(2, 0x80000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_64MB);

  while ((nonCacheSize >> i) > 0x1U) {
    i++;
  }

  if (i != 0) {
    /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
    assert(!(nonCacheStart % nonCacheSize));
    assert(nonCacheSize == (uint32_t) (1 << i));
    assert(i >= 5);

    /* Region 3 setting: Memory with device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(3, nonCacheStart);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
  }

    #if defined(__USE_SHMEM)
  i = 0;

  while ((rpmsgShmemSize >> i) > 0x1U) {
    i++;
  }

  if (i != 0) {
    /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
    assert(!(rpmsgShmemStart % rpmsgShmemSize));
    assert(rpmsgShmemSize == (uint32_t) (1 << i));
    assert(i >= 5);

    /* Region 4 setting: Memory with device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(4, rpmsgShmemStart);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
  }
    #endif
  #else
  while ((nonCacheSize >> i) > 0x1U) {
    i++;
  }

  if (i != 0) {
    /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
    assert(!(nonCacheStart % nonCacheSize));
    assert(nonCacheSize == (uint32_t) (1 << i));
    assert(i >= 5);

    /* Region 0 setting: Memory with device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(0, nonCacheStart);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
  }

    #if defined(__USE_SHMEM)
  i = 0;

  while ((rpmsgShmemSize >> i) > 0x1U) {
    i++;
  }

  if (i != 0) {
    /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
    assert(!(rpmsgShmemStart % rpmsgShmemSize));
    assert(rpmsgShmemSize == (uint32_t) (1 << i));
    assert(i >= 5);

    /* Region 1 setting: Memory with device type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(1, rpmsgShmemStart);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, i - 1);
  }
    #endif
  #endif

  /* Enable MPU */
  ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk);

  /* Enables the processor system bus to invalidate all lines in both ways.
    and Initiate the processor system bus cache command. */
  LMEM->PSCCR |= LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK | LMEM_PSCCR_GO_MASK;
  /* Wait until the cache command completes */
  while ((LMEM->PSCCR & LMEM_PSCCR_GO_MASK) != 0U) {
  }
  /* As a precaution clear the bits to avoid inadvertently re-running this command. */
  LMEM->PSCCR &= ~(LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK);
  /* Now enable the system bus cache. */
  LMEM->PSCCR |= LMEM_PSCCR_ENCACHE_MASK;

  /* Enables the processor code bus to invalidate all lines in both ways.
    and Initiate the processor code bus code cache command. */
  LMEM->PCCCR |= LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK | LMEM_PCCCR_GO_MASK;
  /* Wait until the cache command completes. */
  while ((LMEM->PCCCR & LMEM_PCCCR_GO_MASK) != 0U) {
  }
  /* As a precaution clear the bits to avoid inadvertently re-running this command. */
  LMEM->PCCCR &= ~(LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK);
  /* Now enable the code bus cache. */
  LMEM->PCCCR |= LMEM_PCCCR_ENCACHE_MASK;
}
#endif
