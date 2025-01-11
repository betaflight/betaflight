/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2023
 * Generated linker script file for LPC4337
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.7.1 [Build 9221] [2023-03-28] on Aug 14, 2023, 3:36:29 PM
 */

MEMORY
{
  /* Define each memory region */
  MFlashA512 (rx) : ORIGIN = 0x1a000000, LENGTH = 0x80000 /* 512K bytes (alias Flash) */
  MFlashB512 (rx) : ORIGIN = 0x1b000000, LENGTH = 0x80000 /* 512K bytes (alias Flash2) */
  RamLoc32 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x8000 /* 32K bytes (alias RAM) */
  RamLoc40 (rwx) : ORIGIN = 0x10080000, LENGTH = 0xa000 /* 40K bytes (alias RAM2) */
  RamAHB32 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000 /* 32K bytes (alias RAM3) */
  RamAHB16 (rwx) : ORIGIN = 0x20008000, LENGTH = 0x4000 /* 16K bytes (alias RAM4) */
  RamAHB_ETB16 (rwx) : ORIGIN = 0x2000c000, LENGTH = 0x4000 /* 16K bytes (alias RAM5) */
}

  /* Define a symbol for the top of each memory region */
  __base_MFlashA512 = 0x1a000000  ; /* MFlashA512 */
  __base_Flash = 0x1a000000 ; /* Flash */
  __top_MFlashA512 = 0x1a000000 + 0x80000 ; /* 512K bytes */
  __top_Flash = 0x1a000000 + 0x80000 ; /* 512K bytes */
  __base_MFlashB512 = 0x1b000000  ; /* MFlashB512 */
  __base_Flash2 = 0x1b000000 ; /* Flash2 */
  __top_MFlashB512 = 0x1b000000 + 0x80000 ; /* 512K bytes */
  __top_Flash2 = 0x1b000000 + 0x80000 ; /* 512K bytes */
  __base_RamLoc32 = 0x10000000  ; /* RamLoc32 */
  __base_RAM = 0x10000000 ; /* RAM */
  __top_RamLoc32 = 0x10000000 + 0x8000 ; /* 32K bytes */
  __top_RAM = 0x10000000 + 0x8000 ; /* 32K bytes */
  __base_RamLoc40 = 0x10080000  ; /* RamLoc40 */
  __base_RAM2 = 0x10080000 ; /* RAM2 */
  __top_RamLoc40 = 0x10080000 + 0xa000 ; /* 40K bytes */
  __top_RAM2 = 0x10080000 + 0xa000 ; /* 40K bytes */
  __base_RamAHB32 = 0x20000000  ; /* RamAHB32 */
  __base_RAM3 = 0x20000000 ; /* RAM3 */
  __top_RamAHB32 = 0x20000000 + 0x8000 ; /* 32K bytes */
  __top_RAM3 = 0x20000000 + 0x8000 ; /* 32K bytes */
  __base_RamAHB16 = 0x20008000  ; /* RamAHB16 */
  __base_RAM4 = 0x20008000 ; /* RAM4 */
  __top_RamAHB16 = 0x20008000 + 0x4000 ; /* 16K bytes */
  __top_RAM4 = 0x20008000 + 0x4000 ; /* 16K bytes */
  __base_RamAHB_ETB16 = 0x2000c000  ; /* RamAHB_ETB16 */
  __base_RAM5 = 0x2000c000 ; /* RAM5 */
  __top_RamAHB_ETB16 = 0x2000c000 + 0x4000 ; /* 16K bytes */
  __top_RAM5 = 0x2000c000 + 0x4000 ; /* 16K bytes */

ENTRY(ResetISR)

SECTIONS
{
     .text_Flash2 : ALIGN(4)
    {
       FILL(0xff)
        *(.text_Flash2) /* for compatibility with previous releases */
        *(.text_MFlashB512) /* for compatibility with previous releases */
        *(.text.$Flash2)
        *(.text.$MFlashB512)
        *(.text_Flash2.*) /* for compatibility with previous releases */
        *(.text_MFlashB512.*) /* for compatibility with previous releases */
        *(.text.$Flash2.*)
        *(.text.$MFlashB512.*)
        *(.rodata.$Flash2)
        *(.rodata.$MFlashB512)
        *(.rodata.$Flash2.*)
        *(.rodata.$MFlashB512.*)            } > MFlashB512

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
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        LONG(LOADADDR(.data_RAM4));
        LONG(    ADDR(.data_RAM4));
        LONG(  SIZEOF(.data_RAM4));
        LONG(LOADADDR(.data_RAM5));
        LONG(    ADDR(.data_RAM5));
        LONG(  SIZEOF(.data_RAM5));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        LONG(    ADDR(.bss_RAM4));
        LONG(  SIZEOF(.bss_RAM4));
        LONG(    ADDR(.bss_RAM5));
        LONG(  SIZEOF(.bss_RAM5));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > MFlashA512
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlashA512

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > MFlashA512

    _etext = .;

    /* DATA section for RamLoc40 */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        PROVIDE(__start_data_RamLoc40 = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$RamLoc40)
        *(.data.$RAM2)
        *(.data.$RamLoc40)
        *(.data.$RAM2.*)
        *(.data.$RamLoc40.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_RamLoc40 = .) ;
     } > RamLoc40 AT>MFlashA512

    /* DATA section for RamAHB32 */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        PROVIDE(__start_data_RamAHB32 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$RamAHB32)
        *(.data.$RAM3)
        *(.data.$RamAHB32)
        *(.data.$RAM3.*)
        *(.data.$RamAHB32.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
        PROVIDE(__end_data_RamAHB32 = .) ;
     } > RamAHB32 AT>MFlashA512

    /* DATA section for RamAHB16 */

    .data_RAM4 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM4 = .) ;
        PROVIDE(__start_data_RamAHB16 = .) ;
        *(.ramfunc.$RAM4)
        *(.ramfunc.$RamAHB16)
        *(.data.$RAM4)
        *(.data.$RamAHB16)
        *(.data.$RAM4.*)
        *(.data.$RamAHB16.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM4 = .) ;
        PROVIDE(__end_data_RamAHB16 = .) ;
     } > RamAHB16 AT>MFlashA512

    /* DATA section for RamAHB_ETB16 */

    .data_RAM5 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM5 = .) ;
        PROVIDE(__start_data_RamAHB_ETB16 = .) ;
        *(.ramfunc.$RAM5)
        *(.ramfunc.$RamAHB_ETB16)
        *(.data.$RAM5)
        *(.data.$RamAHB_ETB16)
        *(.data.$RAM5.*)
        *(.data.$RamAHB_ETB16.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM5 = .) ;
        PROVIDE(__end_data_RamAHB_ETB16 = .) ;
     } > RamAHB_ETB16 AT>MFlashA512

    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc32 AT> RamLoc32

    /* Main DATA section (RamLoc32) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_RamLoc32 = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_RamLoc32 = .) ;
    } > RamLoc32 AT>MFlashA512

    /* BSS section for RamLoc40 */
    .bss_RAM2 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_RamLoc40 = .) ;
       *(.bss.$RAM2)
       *(.bss.$RamLoc40)
       *(.bss.$RAM2.*)
       *(.bss.$RamLoc40.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_RamLoc40 = .) ;
    } > RamLoc40 AT> RamLoc40

    /* BSS section for RamAHB32 */
    .bss_RAM3 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_RamAHB32 = .) ;
       *(.bss.$RAM3)
       *(.bss.$RamAHB32)
       *(.bss.$RAM3.*)
       *(.bss.$RamAHB32.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_RamAHB32 = .) ;
    } > RamAHB32 AT> RamAHB32

    /* BSS section for RamAHB16 */
    .bss_RAM4 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM4 = .) ;
       PROVIDE(__start_bss_RamAHB16 = .) ;
       *(.bss.$RAM4)
       *(.bss.$RamAHB16)
       *(.bss.$RAM4.*)
       *(.bss.$RamAHB16.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM4 = .) ;
       PROVIDE(__end_bss_RamAHB16 = .) ;
    } > RamAHB16 AT> RamAHB16

    /* BSS section for RamAHB_ETB16 */
    .bss_RAM5 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM5 = .) ;
       PROVIDE(__start_bss_RamAHB_ETB16 = .) ;
       *(.bss.$RAM5)
       *(.bss.$RamAHB_ETB16)
       *(.bss.$RAM5.*)
       *(.bss.$RamAHB_ETB16.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM5 = .) ;
       PROVIDE(__end_bss_RamAHB_ETB16 = .) ;
    } > RamAHB_ETB16 AT> RamAHB_ETB16

    /* MAIN BSS SECTION: EDIT change to RamLoc40 */
    .bss : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_RamLoc32 = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_RamLoc32 = .) ;
/*        PROVIDE(end = .);*/
    } > RamLoc40 AT> RamLoc40 /* > RamLoc32 AT> RamLoc32 */

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
    } > RamLoc40

    /* NOINIT section for RamLoc40 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_RamLoc40 = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$RamLoc40)
       *(.noinit.$RAM2.*)
       *(.noinit.$RamLoc40.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_RamLoc40 = .) ;
    } > RamLoc40 AT> RamLoc40

    /* NOINIT section for RamAHB32 */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM3 = .) ;
       PROVIDE(__start_noinit_RamAHB32 = .) ;
       *(.noinit.$RAM3)
       *(.noinit.$RamAHB32)
       *(.noinit.$RAM3.*)
       *(.noinit.$RamAHB32.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM3 = .) ;
       PROVIDE(__end_noinit_RamAHB32 = .) ;
    } > RamAHB32 AT> RamAHB32

    /* NOINIT section for RamAHB16 */
    .noinit_RAM4 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM4 = .) ;
       PROVIDE(__start_noinit_RamAHB16 = .) ;
       *(.noinit.$RAM4)
       *(.noinit.$RamAHB16)
       *(.noinit.$RAM4.*)
       *(.noinit.$RamAHB16.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM4 = .) ;
       PROVIDE(__end_noinit_RamAHB16 = .) ;
    } > RamAHB16 AT> RamAHB16

    /* NOINIT section for RamAHB_ETB16 */
    .noinit_RAM5 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM5 = .) ;
       PROVIDE(__start_noinit_RamAHB_ETB16 = .) ;
       *(.noinit.$RAM5)
       *(.noinit.$RamAHB_ETB16)
       *(.noinit.$RAM5.*)
       *(.noinit.$RamAHB_ETB16.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM5 = .) ;
       PROVIDE(__end_noinit_RamAHB_ETB16 = .) ;
    } > RamAHB_ETB16 AT> RamAHB_ETB16

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_RamLoc32 = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_RamLoc32 = .) ;
    } > RamLoc32 AT> RamLoc32

/*    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);*/

    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc32 - 0);

    /* ## Create checksum value (used in startup) ## */
    /* This cause issue with clang linker, so it is disabled */
    /* MemManage_Handler, BusFault_Handler, UsageFault_Hander may not be defined */
/*    PROVIDE(__valid_user_code_checksum = 0 -*/
/*                                         (_vStackTop*/
/*                                         + (ResetISR + 1)*/
/*                                         + (NMI_Handler + 1)*/
/*                                         + (HardFault_Handler + 1)*/
/*                                         + (( DEFINED(MemManage_Handler) ? MemManage_Handler : 0 ) + 1)*/
/*                                         + (( DEFINED(BusFault_Handler) ? BusFault_Handler : 0 ) + 1)*/
/*                                         + (( DEFINED(UsageFault_Handler) ? UsageFault_Handler : 0 ) + 1)*/
/*                                         ) );*/

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
