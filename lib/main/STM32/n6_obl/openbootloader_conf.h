/*
 * Override of CubeN6's openbootloader_conf.h.
 *
 * The submodule version uses _S secure aliases (SRAM2_AXI_BASE_S etc.)
 * which are only defined when the build sets CPU_IN_SECURE_STATE +
 * -mcmse — see feedback_n6_peripheral_alias_cmse.md. We don't (matches
 * the FSBL stub and main BF build), so all peripheral / SRAM macros
 * resolve to NS variants. RIFSC is configured for OPEN-lifecycle silicon
 * to permit NS access regardless.
 *
 * Picked up before the submodule version because -I. comes first in
 * INCLUDES.
 */

#ifndef OPENBOOTLOADER_CONF_H
#define OPENBOOTLOADER_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"

#define DEVICE_ID_MSB                     0x04U
#define DEVICE_ID_LSB                     0x86U

/* RAM scratch — OBL middleware uses these for its receive/scratch
 * buffers. Targets SRAM2 AXI (NS alias) per ST's reference. */
#define RAM_MEM_START_ADDRESS             (SRAM2_AXI_BASE_NS + 0xB9000U)
#define RAM_MEM_SIZE                      (SRAM2_AXI_SIZE - 0xB9000U)
#define RAM_MEM_END_ADDRESS               (RAM_MEM_START_ADDRESS + RAM_MEM_SIZE - 1U)

#define FLASH_LOADER_START_ADDRESS        SRAM1_AHB_BASE_NS
#define FLASH_LOADER_SIZE                 (SRAM1_AHB_SIZE + SRAM2_AHB_SIZE)
#define FLASH_LOADER_END_ADDRESS          (FLASH_LOADER_START_ADDRESS + FLASH_LOADER_SIZE - 1U)

#define FLASHLAYOUT_ADDRESS               (RAM_MEM_START_ADDRESS + RAM_MEM_SIZE - 1024U)

#define RAM_WRITE_ADDRESS                 RAM_MEM_START_ADDRESS
#define FLASH_LOADER_WRITE_ADDRESS        FLASH_LOADER_START_ADDRESS

#define EXT_MEMORY_START_ADDRESS          0x70000000U
#define EXT_MEMORY_END_ADDRESS            0x78000000U
#define EXT_MEMORY_SIZE                   0x08000000U
#define EXT_MEMORY_SECTOR_SIZE            0x10000U

#define OPENBL_DEFAULT_MEM                0xFFFFFFFFU
#define UNDEF_ADDRESS                     0xFFFFFFFFU

#define RDP_LEVEL_0                       0xEEEEEEEEU
#define RDP_LEVEL_1                       0xEEEEEEEEU

#define AREA_ERROR                        0x0U
#define RAM_AREA                          0x1U
#define OTP_AREA                          0x2U
#define EXTERNAL_MEMORY_AREA              0x3U

#define FLASH_MASS_ERASE                  0xFFFFU

/* Only USB transport in our build (no USART/I2C/SPI/CAN). The
 * INTERFACES_SUPPORTED count gates how many OPENBL_RegisterInterface
 * slots OBL pre-allocates. We register USB + IWDG = 2 in
 * app_openbootloader.c. */
#define INTERFACES_SUPPORTED              2U

#ifdef __cplusplus
}
#endif

#endif /* OPENBOOTLOADER_CONF_H */
