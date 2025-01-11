/* Linker script to configure memory regions. */

SEARCH_DIR(.)
/*GROUP(-lgcc -lc -lnosys) not compatible with clang*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000
  CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x20000
}


INCLUDE "nrf_common.ld"

/* nrfx v2 linker does not define __tbss_start/end__ __sbss_start/end__*/
__tbss_start__ = __tbss_start;
__tbss_end__ = __tbss_end;
__sbss_start__ = __sbss_start;
__sbss_end__ = __sbss_end;
