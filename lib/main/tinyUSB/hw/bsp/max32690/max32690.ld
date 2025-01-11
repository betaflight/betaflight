MEMORY {
    ROM        (rx) : ORIGIN = 0x00000000,    LENGTH = 0x00020000 /* 128kB ROM */
    FLASH      (rx) : ORIGIN = 0x10000000,    LENGTH = 0x00340000
    SRAM      (rwx) : ORIGIN = 0x20000000,    LENGTH = 0x00100000

	/*
     * Note that CS0/CS1 address mappings may be reversed using MXC_HPC->mbr0 and ->mbr1
     * The following mappings are selected for simplicity
     */
    HPB_CS0 (rwx)  : ORIGIN = 0x60000000, LENGTH = 0x10000000  /* External Hyperbus/Xccelabus chip select 0 */
    HPB_CS1 (rwx)  : ORIGIN = 0x70000000, LENGTH = 0x10000000  /* External Hyperbus/Xccelabus chip select 1 */
}

SECTIONS {
    .rom :
    {
        KEEP(*(.rom_vector))
        *(.rom_handlers*)
    } > ROM

    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        EXCLUDE_FILE (*riscv.o) *(.text*)    /* program code, exclude RISCV code */
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

    /* These sections allow code to be compiled/linked for HPB addresses, but reside in
     * flash until copied by code to the external HPB flash device
     */
    .hpb_cs0_section :
    {
	__hpb_cs0_start = ABSOLUTE(.);
        KEEP(*(.hpb_cs0_section*))
    } > HPB_CS0 AT>FLASH

    __load_start_hpb_cs0 = LOADADDR(.hpb_cs0_section);
    __load_length_hpb_cs0 = SIZEOF(.hpb_cs0_section);

    .hpb_cs1_section :
    {
    	__hpb_cs1_start = ABSOLUTE(.);
        KEEP(*(.hpb_cs1_section*))
    } > HPB_CS1 AT>FLASH

    __load_start_hpb_cs1 = LOADADDR(.hpb_cs1_section);
    __load_length_hpb_cs1 = SIZEOF(.hpb_cs1_section);

    /* Binary import */
    .bin_storage :
    {
       FILL(0xFF)
      _bin_start_ = .;
      KEEP(*(.bin_storage_img))
      _bin_end_ = .;
      . = ALIGN(4);
    } > FLASH

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

        /* Run the flash programming functions from SRAM */
        *(.flashprog)

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

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SRAM

    .heap (COPY):
    {
        . = ALIGN(4);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM

    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
