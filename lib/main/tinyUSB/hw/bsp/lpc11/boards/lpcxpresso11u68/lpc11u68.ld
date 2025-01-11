/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2013
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for LPC11U68
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.2.1 [Build 795] [2018-07-25] on May 14, 2019 4:55:54 PM
 */

MEMORY
{
  /* Define each memory region */
  MFlash256 (rx) : ORIGIN = 0x0, LENGTH = 0x40000 /* 256K bytes (alias Flash) */
  Ram0_32 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x8000 /* 32K bytes (alias RAM) */
  Ram1_2 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x800 /* 2K bytes (alias RAM2) */
  Ram2USB_2 (rwx) : ORIGIN = 0x20004000, LENGTH = 0x800 /* 2K bytes (alias RAM3) */
}

  /* Define a symbol for the top of each memory region */
  __base_MFlash256 = 0x0  ; /* MFlash256 */
  __base_Flash = 0x0 ; /* Flash */
  __top_MFlash256 = 0x0 + 0x40000 ; /* 256K bytes */
  __top_Flash = 0x0 + 0x40000 ; /* 256K bytes */
  __base_Ram0_32 = 0x10000000  ; /* Ram0_32 */
  __base_RAM = 0x10000000 ; /* RAM */
  __top_Ram0_32 = 0x10000000 + 0x8000 ; /* 32K bytes */
  __top_RAM = 0x10000000 + 0x8000 ; /* 32K bytes */
  __base_Ram1_2 = 0x20000000  ; /* Ram1_2 */
  __base_RAM2 = 0x20000000 ; /* RAM2 */
  __top_Ram1_2 = 0x20000000 + 0x800 ; /* 2K bytes */
  __top_RAM2 = 0x20000000 + 0x800 ; /* 2K bytes */
  __base_Ram2USB_2 = 0x20004000  ; /* Ram2USB_2 */
  __base_RAM3 = 0x20004000 ; /* RAM3 */
  __top_Ram2USB_2 = 0x20004000 + 0x800 ; /* 2K bytes */
  __top_RAM3 = 0x20004000 + 0x800 ; /* 2K bytes */

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
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

    } > MFlash256

    .text : ALIGN(4)
    {
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > MFlash256
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash256

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash256
    __exidx_end = .;

    _etext = .;

    /* possible MTB section for Ram1_2 */
    .mtb_buffer_RAM2 (NOLOAD) :
    {
        KEEP(*(.mtb.$RAM2*))
        KEEP(*(.mtb.$Ram1_2*))
    } > Ram1_2

    /* DATA section for Ram1_2 */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$Ram1_2)
        *(.data.$RAM2*)
        *(.data.$Ram1_2*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
     } > Ram1_2 AT>MFlash256
    /* possible MTB section for Ram2USB_2 */
    .mtb_buffer_RAM3 (NOLOAD) :
    {
        KEEP(*(.mtb.$RAM3*))
        KEEP(*(.mtb.$Ram2USB_2*))
    } > Ram2USB_2

    /* DATA section for Ram2USB_2 */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$Ram2USB_2)
        *(.data.$RAM3*)
        *(.data.$Ram2USB_2*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
     } > Ram2USB_2 AT>MFlash256
    /* MAIN DATA SECTION */
        /* Default MTB section */
        .mtb_buffer_default (NOLOAD) :
        {
           KEEP(*(.mtb*))
        } > Ram0_32
    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > Ram0_32

    /* Main DATA section (Ram0_32) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > Ram0_32 AT>MFlash256

    /* BSS section for Ram1_2 */
    .bss_RAM2 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       *(.bss.$RAM2*)
       *(.bss.$Ram1_2*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
    } > Ram1_2

    /* BSS section for Ram2USB_2 */
    .bss_RAM3 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       *(.bss.$RAM3*)
       *(.bss.$Ram2USB_2*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
    } > Ram2USB_2

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > Ram0_32

    /* NOINIT section for Ram1_2 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       *(.noinit.$RAM2*)
       *(.noinit.$Ram1_2*)
       . = ALIGN(4) ;
    } > Ram1_2

    /* NOINIT section for Ram2USB_2 */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       *(.noinit.$RAM3*)
       *(.noinit.$Ram2USB_2*)
       . = ALIGN(4) ;
    } > Ram2USB_2

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
    } > Ram0_32
    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_Ram0_32 - 0);

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
