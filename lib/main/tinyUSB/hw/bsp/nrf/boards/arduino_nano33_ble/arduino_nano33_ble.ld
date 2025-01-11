/* Linker script to configure memory regions. */

SEARCH_DIR(.)
/*GROUP(-lgcc -lc -lnosys) not compatible with clang*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x10000, LENGTH = 0xf0000
  RAM_NVIC (rwx) : ORIGIN = 0x20000000, LENGTH = 0x100
  RAM_CRASH_DATA (rwx) : ORIGIN = (0x20000000 + 0x100), LENGTH = 0x100
  RAM (rwx) : ORIGIN = ((0x20000000 + 0x100) + 0x100), LENGTH = (0x40000 - (0x100 + 0x100))
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf_common.ld"
