/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Common split-Flash linker script for CH32H41x devices.
 */

/* Entry Point */
ENTRY( _start )

/* Highest address of the user mode stack */
_Hot_Reboot_Flags_Size = 16;
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size;    /* end of RAM */

/* Base address where the config is stored. */
__config_start = ORIGIN(FLASH_CONFIG);
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 4096;  /* required amount of stack */

SECTIONS
{
    /* startup code & copy code to ITCM / SRAM */
    .init :
    {
        _sinit = .;
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.init)))
        KEEP(*(SORT_NONE(.handle_reset)))
        . = ALIGN(4);
        _einit = .;
    } >FLASH AT>FLASH

    /* Critical program code goes into ITCM RAM */
    /* Copy specific fast-executing code to ITCM RAM */
    tcm_code = LOADADDR(.tcm_code);
    .tcm_code :
    {
        . = ALIGN(4);
        tcm_code_start = .;
        /* Code that must be placed in the ITCM area */
        *(.vector);
        KEEP(*(SORT_NONE(.vector_handler)))
        *(.tcm_code)
        *(.tcm_code*)
        /* Prioritize placing code in ITCM area */
        obj/main/CH32H4*/flight/*.o(.text .text.*)
        obj/main/CH32H4*/fc/*.o(.text .text.*)
        /* Display, camera-menu, LED animation, SD FAT and VTX configuration
         * protocols run in cooperative tasks. Their peripheral/IRQ/DMA
         * drivers stay in ITCM; GPS, RC input, motor and scheduler paths
         * also stay here. CMS/CLI/OSD code falls through to SRAM below. */
        obj/main/CH32H4*/io/*.o(EXCLUDE_FILE (*displayport*.o *rcdevice*.o *vtx*.o *smartaudio_protocol.o *tramp_protocol.o *ledstrip*.o *asyncfatfs*.o *fat_standard*.o) .text*)
        obj/main/CH32H4*/config/*.o(.text .text.*)
        obj/main/CH32H4*/pg/*.o(.text .text.*)
        obj/main/CH32H4*/build/*.o(.text .text.*)
        obj/main/CH32H4*/common/*.o(.text .text.*)
        obj/main/CH32H4*/common/stm32/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/*.o(EXCLUDE_FILE (*serial_escserial.o) .text*)
        obj/main/CH32H4*/drivers/accgyro/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/barometer/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/compass/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/flash/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/opticalflow/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/rangefinder/*.o(.text .text.*)
        obj/main/CH32H4*/drivers/rx/*.o(.text .text.*)
        obj/main/CH32H4*/rx/crsf.o(.text .text.*)
        obj/main/CH32H4*/scheduler/*.o(.text .text.*)
        obj/main/CH32H4*/sensors/*.o(.text .text.*)
        obj/main/CH32H4*/telemetry/*.o(.text .text.*)
        obj/main/CH32H4*/CH32/*.o(.text .text.*)
        obj/main/CH32H4*/main.o(.text .text.*)
        obj/main/CH32H4*/ch32h417_dma.o(.text .text.*)
        obj/main/CH32H4*/ch32h417_tim.o(.text .text.*)
        obj/main/CH32H4*/ch32h417_spi.o(.text .text.*)
        obj/main/CH32H4*/ch32h417_gpio.o(.text .text.*)
        obj/main/CH32H4*/ch32h417_usart.o(.text .text.*)
        *libgcc.a:*save-restore*.o(.text .text.*)
        . = ALIGN(4);
        tcm_code_end = .;
    } >ITCM AT >MOVABLE_FLASH

    ram_code = LOADADDR(.ram_code);
    .ram_code :
    {
        . = ALIGN(4);
        ram_code_start = .;
        *(.ram_code)
        *(.ram_code*)
        *(.text)
        *(.text.*)

        *(.rodata)
        *(.rodata*)

        . = ALIGN(4);
        ram_code_end = .;
    } >SRAM AT >MOVABLE_FLASH

    .text :
    {
        . = ALIGN(4);
        *(.gnu.linkonce.t.*)
        . = ALIGN(4);
    } >FLASH1 AT>MOVABLE_FLASH

    .fini :
    {
        KEEP(*(SORT_NONE(.fini)))
        . = ALIGN(4);
    } >FLASH1 AT>MOVABLE_FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH1 AT>MOVABLE_FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH1 AT>MOVABLE_FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
        KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH1 AT>MOVABLE_FLASH

    .pg_registry :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_registry_start = .);
        KEEP (*(.pg_registry))
        KEEP (*(SORT(.pg_registry.*)))
        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_registry_end = .);
    } >MOVABLE_FLASH

    .pg_resetdata :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_resetdata_start = .);
        KEEP (*(.pg_resetdata))
        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_resetdata_end = .);
    } >MOVABLE_FLASH

    .ctors :
    {
        /* gcc uses crtbegin.o to find the start of
           the constructors, so we make sure it is
           first. Because this is a wildcard, it
           doesn't matter if the user does not
           actually link against crtbegin.o; the
           linker won't look for a file to match a
           wildcard. The wildcard also means that it
           doesn't matter which directory crtbegin.o
           is in. */
        KEEP (*crtbegin.o(.ctors))
        KEEP (*crtbegin?.o(.ctors))
        /* We don't want to include the .ctor section from
           the crtend.o file until after the sorted ctors.
           The .ctor section from the crtend file contains the
           end of ctors marker and it must be last */
        KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*(.ctors))
    } >FLASH1 AT>MOVABLE_FLASH

    .dtors :
    {
        KEEP (*crtbegin.o(.dtors))
        KEEP (*crtbegin?.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*(.dtors))
    } >FLASH1 AT>MOVABLE_FLASH

    _sfastram_idata = LOADADDR(.fastram_data);
    .fastram_data :
    {
        . = ALIGN(4);
        _sfastram_data = .;        /* create a global symbol at data start */
        *(.fastram_data)           /* .data sections */
        *(.fastram_data*)          /* .data* sections */
        . = ALIGN(4);
        _efastram_data = .;        /* define a global symbol at data end */
    } >FASTRAM AT >MOVABLE_FLASH

    . = ALIGN(4);
    .fastram_bss (NOLOAD) :
    {
        _sfastram_bss = .;
        __fastram_bss_start__ = _sfastram_bss;
        *(.fastram_bss)
        *(SORT_BY_ALIGNMENT(.fastram_bss*))
        . = ALIGN(4);
        _efastram_bss = .;
        __fastram_bss_end__ = _efastram_bss;
    } >FASTRAM

    /* Place large uninitialized RAM (.noncacheable and .bss) before .data/.sdata/.sbss
     * so __global_pointer$ sits high in DTCM (~0x200FA000), well clear of unrelaxed
     * Pass-0 .tcm_code addresses near the 0x200E0000 ITCM/DTCM boundary. */
    .noncacheable (NOLOAD) :
    {
        . = ALIGN(32);
        PROVIDE( _sbss = .);
        *(.noncacheable)
        *(.noncacheable.*)
        . = ALIGN(32);
    } >RAM

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = _sbss;
        *(.bss*)
        *(.gnu.linkonce.b.*)
        *(COMMON*)
        . = ALIGN(4);
        PROVIDE( _ebss = .);
        __bss_end__ = _ebss;
    } >RAM

    _data_lma = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data_vma = .);
        KEEP(*(.region_section))
        *(.gnu.linkonce.r.*)
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(8);
        PROVIDE( __global_pointer$ = . + 0x800 );
        *(.sdata .sdata.*)
        *(.sdata2.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(8);
        *(.srodata.cst16)
        *(.srodata.cst8)
        *(.srodata.cst4)
        *(.srodata.cst2)
        *(.srodata .srodata.*)
        . = ALIGN(4);
        PROVIDE( _edata = .);
    } >RAM AT>MOVABLE_FLASH

    .sbss (NOLOAD) :
    {
        . = ALIGN(4);
        PROVIDE( _ssbss = .);
        *(.sbss*)
        *(.gnu.linkonce.sb.*)
        . = ALIGN(4);
        PROVIDE( _esbss = .);
        PROVIDE( _dtcm_end = .);
    } >RAM

    PROVIDE( _end = _esbss);
    PROVIDE( end = . );

    .stack ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size - _Min_Stack_Size :
    {
        PROVIDE( _heap_end = . );
        . = ALIGN(4);
        PROVIDE( _susrstack = . );
        . = . + _Min_Stack_Size;
        PROVIDE( _eusrstack = . );
    } >STACKRAM
}
