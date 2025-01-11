/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

MEMORY
{
    /*
     * Flash is remapped at 0x0 by 1st stage bootloader, but this is done with
     * an offset derived from image header thus it is safer to use remapped
     * address space at 0x0 instead of QSPI_M address space at 0x16000000.
     * Bootloader partition is 32K, but 9K is currently reserved for product
     * header (8K) and image header (1K).
     * First 512 bytes of SYSRAM are remapped at 0x0 and used as ISR vector
     * (there's no need to reallocate ISR vector) and thus cannot be used by
     * application.
     */

    FLASH (r)  : ORIGIN = (0x00000000), LENGTH = (1024 * 1024)
    RAM (rw)   : ORIGIN = (0x20000000), LENGTH = (512 * 1024)
}

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __HeapBase
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   __bssnz_start__
 *   __bssnz_end__
 */
ENTRY(Reset_Handler)

SECTIONS
{
    __text = .;

    .text :
    {
        __isr_vector_start = .;
        KEEP(*(.isr_vector))
        /* ISR vector shall have exactly 512 bytes */
        . = __isr_vector_start + 0x200;
        __isr_vector_end = .;

        *(.text)
        *(.text.*)

        *(.libcmac.rom)

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

        *(.rodata*)

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > FLASH

    __exidx_start = .;
    .ARM :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } > FLASH
    __exidx_end = .;

    .intvect :
    {
        . = ALIGN(4);
        __intvect_start__ = .;
        . = . + (__isr_vector_end - __isr_vector_start);
        . = ALIGN(4);
    } > RAM

    .sleep_state (NOLOAD) :
    {
        . = ALIGN(4);
        *(sleep_state)
    } > RAM

    /* This section will be zeroed by RTT package init */
    .rtt (NOLOAD):
    {
        . = ALIGN(4);
        *(.rtt)
        . = ALIGN(4);
    } > RAM

    __text_ram_addr = LOADADDR(.text_ram);

    .text_ram :
    {
        . = ALIGN(4);
        __text_ram_start__ = .;
        *(.text_ram*)
        . = ALIGN(4);
        __text_ram_end__ = .;
    } > RAM AT > FLASH

    __etext = LOADADDR(.data);

    .data :
    {
        __data_start__ = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        *(.preinit_array)
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        *(SORT(.init_array.*))
        *(.init_array)
        PROVIDE_HIDDEN (__init_array_end = .);


        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
    } > RAM AT > FLASH

    .bssnz :
    {
        . = ALIGN(4);
        __bssnz_start__ = .;
        *(.bss.core.nz*)
        . = ALIGN(4);
        __bssnz_end__ = .;
    } > RAM

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .cmac (NOLOAD) :
    {
        . = ALIGN(0x400);
        *(.libcmac.ram)
    } > RAM

    /* Heap starts after BSS */
    . = ALIGN(8);
    __HeapBase = .;

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > RAM

    _ram_start = ORIGIN(RAM);

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Top of head is the bottom of the stack */
    __HeapLimit = __StackLimit;
    end = __HeapLimit;

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__HeapBase <= __HeapLimit, "region RAM overflowed with stack")

    /* Check that intvect is at the beginning of RAM */
    ASSERT(__intvect_start__ == ORIGIN(RAM), "intvect is not at beginning of RAM")
}
