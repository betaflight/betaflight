ENTRY(Reset_Handler)

_estack = 0x20008000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x1000; /* required amount of stack */


MEMORY
{
    FLASH(rx) : ORIGIN = 0x00000000, LENGTH = 256K
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4) ;
        *(.vectors)
        *(.text)
        *(.text.*)
        *(.init)
        *(.fini)
        *(.rodata)
        *(.rodata.*)
        *(.ARM.exidx*)
        . = ALIGN(4) ;
        __end_text = . ;
    } >FLASH

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        . = ALIGN(4);
        __start_data = . ;
        __la_data = LOADADDR(.data);
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        __end_data = . ;

    } >SRAM

    .bss :
    {
        . = ALIGN(4) ;
        __start_bss = . ;
        __bss_start__ = __start_bss;
        *(.bss)
        *(.bss.*)
        *(.COMMON)
        __end_bss = . ;
        . = ALIGN(4);
    }>SRAM

    /* User_heap_stack section, used to check that there is enough RAM left */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >SRAM
}
