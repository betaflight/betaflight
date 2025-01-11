/* Default linker script, for normal executables */
OUTPUT_FORMAT("elf32-ft32")
OUTPUT_ARCH(ft32)
SEARCH_DIR("/data/win8/ft32/ft32-elf/lib");
/* Allow the command line to override the memory region sizes.  */
__PMSIZE = DEFINED(__PMSIZE)  ? __PMSIZE : 256K;
__RAMSIZE = DEFINED(__RAMSIZE) ? __RAMSIZE : 64K;
MEMORY
{
  flash     (rx)   : ORIGIN = 0,        LENGTH = __PMSIZE
  ram       (rw!x) : ORIGIN = 0x800000, LENGTH = __RAMSIZE
  ehci_mmap (rw!x) : ORIGIN = 0x811000, LENGTH = 2K
}
SECTIONS
{
  .text :
  {
    *(.text*)
    *(.strings)
    *(._pm*)
    *(.init)
    *(.fini)
     _etext = . ;
    . = ALIGN(4);
  }  > flash
  .tors :
  {
    ___ctors = . ;
    *(.ctors)
    ___ctors_end = . ;
    ___dtors = . ;
    *(.dtors)
    ___dtors_end = . ;
    . = ALIGN(4);
  } > ram
  .data :  AT (ADDR (.text) + SIZEOF (.text))
  {
    *(.data)
    *(.data*)
    *(.rodata)
    *(.rodata*)
     _edata = . ;
    . = ALIGN(4);
  }  > ram
  .bss   SIZEOF(.data) + ADDR(.data) :
  {
     _bss_start = . ;
    *(.bss)
    *(.bss*)
    *(COMMON)
     _end = . ;
    . = ALIGN(4);
  }  > ram
   __data_load_start = LOADADDR(.data);
   __data_load_end = __data_load_start + SIZEOF(.data);
  .stab 0 (NOLOAD) :
  {
    *(.stab)
  }
  .stabstr 0 (NOLOAD) :
  {
    *(.stabstr)
  }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end ) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  /* DWARF Extension.  */
  .debug_macro    0 : { *(.debug_macro) }
  .debug_addr     0 : { *(.debug_addr) }
}
