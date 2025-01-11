/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2013
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for LPC4088
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.2.1 [Build 795] [2018-07-25] on May 15, 2019 5:16:07 PM
 */

MEMORY
{
  /* Define each memory region */
  MFlash512 (rx) : ORIGIN = 0x0, LENGTH = 0x80000 /* 512K bytes (alias Flash) */
  RamLoc64 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x10000 /* 64K bytes (alias RAM) */
  RamPeriph32 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32K bytes (alias RAM2) */
}

  /* Define a symbol for the top of each memory region */
  __base_MFlash512 = 0x0  ; /* MFlash512 */
  __base_Flash = 0x0 ; /* Flash */
  __top_MFlash512 = 0x0 + 0x80000 ; /* 512K bytes */
  __top_Flash = 0x0 + 0x80000 ; /* 512K bytes */
  __base_RamLoc64 = 0x10000000  ; /* RamLoc64 */
  __base_RAM = 0x10000000 ; /* RAM */
  __top_RamLoc64 = 0x10000000 + 0x10000 ; /* 64K bytes */
  __top_RAM = 0x10000000 + 0x10000 ; /* 64K bytes */
  __base_RamPeriph32 = 0x20000000  ; /* RamPeriph32 */
  __base_RAM2 = 0x20000000 ; /* RAM2 */
  __top_RamPeriph32 = 0x20000000 + 0x8000 ; /* 32K bytes */
  __top_RAM2 = 0x20000000 + 0x8000 ; /* 32K bytes */

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

    } > MFlash512

    .text : ALIGN(4)
    {
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > MFlash512
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash512

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash512
    __exidx_end = .;

    _etext = .;

    /* DATA section for RamPeriph32 */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$RamPeriph32)
        *(.data.$RAM2*)
        *(.data.$RamPeriph32*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
     } > RamPeriph32 AT>MFlash512
    /* MAIN DATA SECTION */
    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc64

    /* Main DATA section (RamLoc64) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > RamLoc64 AT>MFlash512

    /* BSS section for RamPeriph32 */
    .bss_RAM2 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       *(.bss.$RAM2*)
       *(.bss.$RamPeriph32*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
    } > RamPeriph32

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
/*        PROVIDE(end = .);*/
    } > RamLoc64

    /* NOINIT section for RamPeriph32 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       *(.noinit.$RAM2*)
       *(.noinit.$RamPeriph32*)
       . = ALIGN(4) ;
    } > RamPeriph32

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc64

    /* hathach add heap section for clang */
    .heap (NOLOAD): {
        __heap_start = .;
        __HeapBase = .;
        __heap_base = .;
        __end = .;
        PROVIDE(end = .);
        PROVIDE(_end = .);
        PROVIDE(__end__ = .);
        KEEP(*(.heap*))
        __HeapLimit = .;
        __heap_limit = .;
        __heap_end = .;
    } > RamLoc64

/*    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);*/
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc64 - 0);

    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 -
                                         (_vStackTop
                                         + (ResetISR + 1)
                                         + (NMI_Handler + 1)
                                         + (HardFault_Handler + 1)
                                         + (( DEFINED(MemManage_Handler) ? MemManage_Handler : 0 ) + 1)   /* MemManage_Handler may not be defined */
                                         + (( DEFINED(BusFault_Handler) ? BusFault_Handler : 0 ) + 1)     /* BusFault_Handler may not be defined */
                                         + (( DEFINED(UsageFault_Handler) ? UsageFault_Handler : 0 ) + 1) /* UsageFault_Handler may not be defined */
                                         ) );

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
