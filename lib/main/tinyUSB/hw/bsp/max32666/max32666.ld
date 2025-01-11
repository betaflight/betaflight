/* SPID and SPIX Sections here are maximum possible sizes */
/* If used, they should be adjusted for the external Flash/RAM size */
MEMORY {

    SPIX (rx)  : ORIGIN = 0x08000000,                           LENGTH = 0x08000000
    FLASH (rx) : ORIGIN = 0x10000000,                           LENGTH = 0x00100000
    SRAM (rwx)      : ORIGIN = 0x20000000,                      LENGTH = 0x0008C000
    SPID (rw)       : ORIGIN = 0x80000000,                      LENGTH = 512M
}

/* Sections Definitions */
SECTIONS {
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)    /* program code */
        *(.rodata*)  /* read-only data: "const" */

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    /* This section will keep the SPIX data until loaded into the external device */
    /* Upon initialization of SPIX (user code needs to do this) */
    .xip_section :
    {
        KEEP(*(.xip_section*))
    } > SPIX AT>FLASH

    __load_start_xip = LOADADDR(.xip_section);
    __load_length_xip = SIZEOF(.xip_section);

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    .data :
    {
        _data = ALIGN(., 4);
        *(vtable)
        *(.data*)           /*read-write initialized data: initialized global variable*/
        *(.spix_config*)    /* SPIX configuration functions need to be run from SRAM */
        *(.flashprog*)      /* Flash program */


        /* These array sections are used by __libc_init_array to call static C++ constructors */
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        _edata = ALIGN(., 4);
    } > SRAM AT>FLASH
    __load_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialized global variable*/
        *(COMMON)
        _ebss = ALIGN(., 4);
    } > SRAM

    /* Setup the stack for Core 1, it will only be used if the user code
     * includes a definition of Stack_Size_Core1, which defines the space
     * reserved above the main core's stack for core 1's stack */

    __StackTop_Core1 = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit_Core1 = DEFINED(Stack_Size_Core1) ? __StackTop_Core1 - Stack_Size_Core1 : __StackTop_Core1;

    /* Set stack top to end of RAM, and stack limit move down by Stack_Size.
     * If core 1 is used, set the stack to the bottom of Core 1's stack region */

    __StackTop = DEFINED(Stack_Size_Core1) ? __StackLimit_Core1 : ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - Stack_Size;

    .heap (COPY):
    {
        . = ALIGN(4);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM

    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack(s) exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
