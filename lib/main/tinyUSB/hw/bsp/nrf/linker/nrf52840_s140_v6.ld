/* Linker script to configure memory regions. */

SEARCH_DIR(.)
/*GROUP(-lgcc -lc -lnosys) not compatible with clang*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0xED000 - 0x26000

  /* SRAM required by S132 depend on
   * - Attribute Table Size
   * - Vendor UUID count
   * - Max ATT MTU
   * - Concurrent connection peripheral + central + secure links
   * - Event Len, HVN queue, Write CMD queue
   */
  RAM (rwx) : ORIGIN = 0x20003400, LENGTH = 0x20040000 - 0x20003400
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

/* nrfx v2 linker does not define __tbss_start/end__ __sbss_start/end__*/
__tbss_start__ = __tbss_start;
__tbss_end__ = __tbss_end;
__sbss_start__ = __sbss_start;
__sbss_end__ = __sbss_end;
