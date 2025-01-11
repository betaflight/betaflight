/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for LPC11U37/401
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v11.0.0 [Build 2516] [2019-06-05] on Sep 6, 2019 12:16:06 PM
 */

MEMORY
{
  /* Define each memory region */
  MFlash128 (rx) : ORIGIN = 0x0, LENGTH = 0x20000 /* 128K bytes (alias Flash) */
  RamLoc8 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x2000 /* 8K bytes (alias RAM) */
  RamUsb2 (rwx) : ORIGIN = 0x20004000, LENGTH = 0x800 /* 2K bytes (alias RAM2) */
}

  /* Define a symbol for the top of each memory region */
  __base_MFlash128 = 0x0  ; /* MFlash128 */
  __base_Flash = 0x0 ; /* Flash */
  __top_MFlash128 = 0x0 + 0x20000 ; /* 128K bytes */
  __top_Flash = 0x0 + 0x20000 ; /* 128K bytes */
  __base_RamLoc8 = 0x10000000  ; /* RamLoc8 */
  __base_RAM = 0x10000000 ; /* RAM */
  __top_RamLoc8 = 0x10000000 + 0x2000 ; /* 8K bytes */
  __top_RAM = 0x10000000 + 0x2000 ; /* 8K bytes */
  __base_RamUsb2 = 0x20004000  ; /* RamUsb2 */
  __base_RAM2 = 0x20004000 ; /* RAM2 */
  __top_RamUsb2 = 0x20004000 + 0x800 ; /* 2K bytes */
  __top_RAM2 = 0x20004000 + 0x800 ; /* 2K bytes */


ENTRY(ResetISR)

SECTIONS
{
     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

    } > MFlash128

    .text : ALIGN(4)
    {
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > MFlash128
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash128

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash128
    __exidx_end = .;

    _etext = .;

    /* DATA section for RamUsb2 */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$RamUsb2)
        *(.data.$RAM2)
        *(.data.$RamUsb2)
        *(.data.$RAM2.*)
        *(.data.$RamUsb2.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
     } > RamUsb2 AT>MFlash128
    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) :
    {
        . = ALIGN(4) ;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc8

    /* Main DATA section (RamLoc8) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > RamLoc8 AT>MFlash128

    /* BSS section for RamUsb2 */
    .bss_RAM2 :
    {
       . = ALIGN(4) ;
       PROVIDE(__start_bss_RAM2 = .) ;
       *(.bss.$RAM2)
       *(.bss.$RamUsb2)
       *(.bss.$RAM2.*)
       *(.bss.$RamUsb2.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
    } > RamUsb2

    /* MAIN BSS SECTION */
    .bss :
    {
        . = ALIGN(4) ;
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RamLoc8

    /* NOINIT section for RamUsb2 */
    .noinit_RAM2 (NOLOAD) :
    {
       . = ALIGN(4) ;
       *(.noinit.$RAM2)
       *(.noinit.$RamUsb2)
       *(.noinit.$RAM2.*)
       *(.noinit.$RamUsb2.*)
       . = ALIGN(4) ;
    } > RamUsb2

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD):
    {
         . = ALIGN(4) ;
        _noinit = .;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc8
    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc8 - 0);

    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 -
                                         (_vStackTop
                                         + (ResetISR + 1)
                                         + (( DEFINED(NMI_Handler) ? NMI_Handler : M0_NMI_Handler ) + 1)
                                         + (( DEFINED(HardFault_Handler) ? HardFault_Handler : M0_HardFault_Handler ) + 1)
                                         )
           );

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
