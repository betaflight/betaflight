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
 * Section layout for NXP parts with 256 KB of ITCM running XIP from FlexSPI NOR.
 * The including script defines these regions:
 *   FLASH_CONFIG_BLOCK  4 KB at the flash base, holds the FCFB
 *   FLASH_IVT           4 KB, holds the image vector table and boot data
 *   FLASH               the image itself
 *   FLASH_CONFIG        the last sectors, holding the saved configuration
 *   ITCM, DTCM, OCRAM2  the on-chip RAMs
 */

ENTRY(Reset_Handler)

/* 0 for an image started by a bootloader, which drops the boot ROM checks. */
__has_boot_rom = DEFINED(__has_boot_rom) ? __has_boot_rom : 1;

__stack_size = DEFINED(__stack_size) ? __stack_size : 0x2000;

/* Read by drivers/stack_check.c to report stack headroom. */
_Min_Stack_Size = __stack_size;

/* __flash_size defaults to the end of the configuration region; a board with reserved top sectors sets it. */
__flash_start = ORIGIN(FLASH_CONFIG_BLOCK);
PROVIDE(__flash_size = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG) - ORIGIN(FLASH_CONFIG_BLOCK));

/* Where the boot ROM reads the FlexSPI configuration block: 0 on the 10xx, 0x400 on the 11xx. */
__fcb_offset = DEFINED(__fcb_offset) ? __fcb_offset : 0;

SECTIONS
{
    /* FlexSPI configuration block, read by the boot ROM at flash + __fcb_offset. */
    .flash_config ORIGIN(FLASH_CONFIG_BLOCK) + __fcb_offset :
    {
        . = ALIGN(4);
        KEEP(*(.boot_hdr.conf))
        . = ALIGN(4);
    } > FLASH_CONFIG_BLOCK

    /* Image vector table and boot data, read by the boot ROM at flash + 0x1000.
       There is no DCD: no board here has SDRAM to bring up. */
    .ivt :
    {
        . = ALIGN(4);
        KEEP(*(.boot_hdr.ivt))
        KEEP(*(.boot_hdr.boot_data))
        . = ALIGN(4);
    } > FLASH_IVT

    /* The initial stack pointer and reset address, at ORIGIN(FLASH). */
    .boot_vector :
    {
        KEEP(*(.boot_vector))
    } > FLASH

    /*
     * Code runs from flash unless claimed below for ITCM; flight path constants go to DTCM.
     * The first matching rule wins, so these come before the flash rules. Init functions stay
     * in flash. To free ITCM, delete lines from the bottom of the second .itcm_text block.
     * check_placement.py fails the build when a claim matches nothing or leaves bytes in flash.
     */
    .modules :
    {
        /* Any flash writer code or constant caught here fails the ASSERT at the end. */
        __flash_writer_left_in_flash_start = .;
        *flexspi_nor_imxrt.o(.text .text* .rodata .rodata*)
        __flash_writer_left_in_flash_end = .;

        /* Third party modules stay in flash. */
        */lib/modules/*.o(.text .text*)
        */lib/modules/*.o(.rodata .rodata*)

        /* Functions named for init, detection or reset defaults. */
        *.o(.text.*Init .text.init* .text.*InitFilters .text.*InitConfig .text.*InitProfile .text.*Detect .text.pgResetFn_*)
        . = ALIGN(4);
    } > FLASH

    .itcm_text :
    {
        . = ALIGN(4);
        __itcm_start = .;

        /* The vector table goes first: VTOR needs it aligned to 1024 bytes. */
        __vector_table = .;
        KEEP(*(.isr_vector))
        __vector_table_end = .;
        . = ALIGN(4);

        /* Flight path: handlers, DMA, flash writer, gyro to motors, scheduler, libc, CRSF, baro, battery, LEDs. */
        *(.ramfunc)
        *(.ramfunc*)
        *(.ram_code)
        *(.ram_code*)
        *(.itcm_text)
        *(.itcm_text*)
        /* Task check functions run on every scheduler pass, whatever file they live in. */
        *.o(.text.*UpdateCheck)
        /* The run loop that calls the scheduler; from flash the call needs a veneer. */
        */main.o(.text .text*)
        */drivers/bus_spi.o(.text .text*)
        */drivers/accgyro/*.o(.text .text*)
        */sensors/*.o(.text .text*)
        */common/*.o(.text .text*)
        */flight/*.o(.text .text*)
        *libm.a:libm_a-sf_lrint.o(.text .text*)
        *libc*.a:libc_a-memcpy*.o(.text .text*)
        *libc*.a:libc_a-memset.o(.text .text*)
        *libc*.a:libc_a-memmove.o(.text .text*)
        *libm.a:libm_a-sf_pow.o(.text .text*)
        *libgcc.a:_aeabi_uldivmod.o(.text .text*)
        *libgcc.a:_aeabi_ldivmod.o(.text .text*)
        *libgcc.a:_udivmoddi4.o(.text .text*)
        *libgcc.a:_dvmd_tls.o(.text .text*)
        *libgcc.a:_popcountsi2.o(.text .text*)
        */drivers/motor.o(.text .text*)
        */drivers/dshot.o(.text .text*)
        */drivers/dshot_command.o(.text .text*)
        */scheduler/*.o(.text .text*)
        */fc/*.o(.text .text*)
        */config/feature.o(.text .text*)
        */rx/*.o(.text .text*)
        */blackbox/*.o(.text .text*)
        */telemetry/telemetry.o(.text .text*)
        */telemetry/crsf.o(.text .text*)
        */drivers/barometer/*.o(.text .text*)
        */io/statusindicator.o(.text .text*)
        */drivers/light_led.o(.text .text*)

        /* Serial and logging: delete from here first. USB stays in flash, it never runs in flight. */
        */drivers/serial*.o(.text .text*)
        */io/serial.o(.text .text*)
        */msp/*.o(.text .text*)
        */io/asyncfatfs/*.o(.text .text*)
        */drivers/sdcard*.o(.text .text*)

        . = ALIGN(4);
        __itcm_end = .;
    } > ITCM AT > FLASH
    __itcm_load = LOADADDR(.itcm_text);

    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data)
        *(.data*)

        /* Constants of the flight path objects. */
        */drivers/bus_spi.o(.rodata .rodata*)
        */drivers/accgyro/*.o(.rodata .rodata*)
        */sensors/*.o(.rodata .rodata*)
        */common/*.o(.rodata .rodata*)
        */flight/*.o(.rodata .rodata*)
        */drivers/motor.o(.rodata .rodata*)
        */drivers/dshot.o(.rodata .rodata*)
        */drivers/dshot_command.o(.rodata .rodata*)
        */scheduler/*.o(.rodata .rodata*)
        */fc/*.o(.rodata .rodata*)
        */rx/*.o(.rodata .rodata*)
        */blackbox/*.o(.rodata .rodata*)
        */telemetry/telemetry.o(.rodata .rodata*)
        */telemetry/crsf.o(.rodata .rodata*)
        *libm.a:libm_a-sf_pow_log2_data.o(.rodata .rodata*)
        *libm.a:libm_a-sf_exp2_data.o(.rodata .rodata*)

        . = ALIGN(4);
        __data_end = .;
    } > DTCM AT > FLASH
    __data_load = LOADADDR(.data);

    /* DMA_DATA, FAST_DATA and FAST_RODATA, copied like .data. */
    .dtcm_data :
    {
        . = ALIGN(32);
        __dtcm_data_start = .;
        *(.dtcm_data)
        *(.dtcm_data*)
        *(.dtcm_rodata)
        *(.dtcm_rodata*)
        . = ALIGN(32);
        __dtcm_data_end = .;
    } > DTCM AT > FLASH
    __dtcm_data_load = LOADADDR(.dtcm_data);

    /* Everything not claimed above runs in place from flash. */
    .text :
    {
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* Parameter group registry. The generic config code walks the range
       between these symbols. */
    .pg_registry :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__pg_registry_start = .);
        KEEP(*(.pg_registry))
        KEEP(*(SORT(.pg_registry.*)))
        PROVIDE_HIDDEN(__pg_registry_end = .);
    } > FLASH

    .pg_resetdata :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__pg_resetdata_start = .);
        KEEP(*(.pg_resetdata))
        PROVIDE_HIDDEN(__pg_resetdata_end = .);
    } > FLASH

    /* Nothing here unwinds, so the unwind tables are discarded below. */
    __exidx_start = .;
    __exidx_end = .;

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* Image size for the boot data block; nothing after this point goes in FLASH. */
    __flash_image_end = .;
    __flash_image_size = __flash_image_end - ORIGIN(FLASH_CONFIG_BLOCK);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(32);
        /* Zero-initialised DMA buffers (DMA_DATA_ZERO_INIT). */
        *(.dtcm_bss)
        *(.dtcm_bss*)
        . = ALIGN(4);
        __bss_end = .;
    } > DTCM

    /* Newlib's sbrk grows the heap up from here; the stack grows down from the
       top of DTCM. */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        PROVIDE(__end__ = .);
    } > DTCM

    /* Symbols rather than a section, so the free DTCM is not counted as used. */
    __stack_top = ORIGIN(DTCM) + LENGTH(DTCM);
    __stack_limit = __stack_top - __stack_size;
    _estack = __stack_top;

    /* Cached write back, so never a DMA buffer. */
    .ocram2 (NOLOAD) :
    {
        . = ALIGN(4);
        __ocram2_start = .;
        *(.ocram2)
        *(.ocram2*)
        . = ALIGN(4);
        __ocram2_end = .;
    } > OCRAM2

    __config_start = ORIGIN(FLASH_CONFIG);
    __config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

    /DISCARD/ :
    {
        *(.ARM.attributes)
        *(.comment)
        *(.ARM.exidx* .ARM.extab* .gnu.linkonce.armextab.*)
    }
}

/* The flash regions must be contiguous; a bootloader image may keep its configuration further out. */
ASSERT(ORIGIN(FLASH_CONFIG_BLOCK) + LENGTH(FLASH_CONFIG_BLOCK) == ORIGIN(FLASH_IVT),
       "flash regions: the image vector table must follow the configuration block")
ASSERT(ORIGIN(FLASH_IVT) + LENGTH(FLASH_IVT) == ORIGIN(FLASH),
       "flash regions: the image must follow the image vector table")
ASSERT(__has_boot_rom ? ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(FLASH_CONFIG)
                      : ORIGIN(FLASH) + LENGTH(FLASH) <= ORIGIN(FLASH_CONFIG),
       "flash regions: the saved configuration must follow the image")

/* The boot ROM fixes the size and place of the 512 byte FCB and the 48 byte IVT and boot data. */
ASSERT(!__has_boot_rom || SIZEOF(.flash_config) == 0x200,
       "the FlexSPI configuration block must be 512 bytes, see flexspiNorConfig_t")
ASSERT(__fcb_offset + SIZEOF(.flash_config) <= 0x1000,
       "the FlexSPI configuration block must end before the image vector table, see __fcb_offset")
ASSERT(!__has_boot_rom || SIZEOF(.ivt) == 0x30,
       "the image vector table plus boot data must be 48 bytes, see xip/image_vector_table.c")
ASSERT(!__has_boot_rom || ADDR(.ivt) - ORIGIN(FLASH_CONFIG_BLOCK) == 0x1000,
       "the image vector table must start 0x1000 into flash, see the FLASH_IVT region")

/* The words the ROM starts the image with must stay at the head of the image. */
ASSERT(ADDR(.boot_vector) == ORIGIN(FLASH),
       "the boot vector must be the first thing in FLASH, see .boot_vector")
ASSERT(SIZEOF(.boot_vector) == (__has_boot_rom ? 8 : 64),
       "the boot vector is the initial stack pointer and the reset address, see bootVector, or without a boot ROM the 16 core vectors")

/* The size the ROM is told must cover every section loaded from FLASH. */
ASSERT(__flash_image_end >= LOADADDR(.itcm_text) + SIZEOF(.itcm_text)
       && __flash_image_end >= LOADADDR(.data) + SIZEOF(.data)
       && __flash_image_end >= LOADADDR(.dtcm_data) + SIZEOF(.dtcm_data)
       && __flash_image_end <= ORIGIN(FLASH_CONFIG),
       "the image does not end where __flash_image_end says, see the sections loaded from FLASH")

/* The TCMs are only as big as the FlexRAM banks the reset handler gives them. */
ASSERT(LENGTH(ITCM) == __flexram_itcm_size,
       "LENGTH(ITCM) must equal the ITCM size set in startup/startup_imxrt*.S")
ASSERT(LENGTH(DTCM) == __flexram_dtcm_size,
       "LENGTH(DTCM) must equal the DTCM size set in startup/startup_imxrt*.S")

ASSERT(__target_flash_size * 1024 == __flash_size,
       "TARGET_FLASH_SIZE or MCU_FLASH_SIZE, in KB, must equal the flash this script describes")

ASSERT((__vector_table & 0x3ff) == 0,
       "the vector table must be 1024 byte aligned, VTOR ignores the low bits")

/* A dropped or extra .word shifts every vector below it. */
ASSERT(__vector_table_end - __vector_table == 4 * __vector_table_entries,
       "the vector table is not the length the startup file declares, see __vector_table_entries")

ASSERT(__stack_limit >= __bss_end, "DTCM overflowed: data, bss and stack do not fit in 256 KB")
ASSERT(__itcm_end + 8K <= ORIGIN(ITCM) + LENGTH(ITCM),
       "ITCM nearly full: remove lines from the second .itcm_text block in link/nxp_itcm256k_sections.ld")
ASSERT(__flash_writer_left_in_flash_end == __flash_writer_left_in_flash_start,
       "flexspi_nor_imxrt.c left code or constants in flash: every function in it must be RAMFUNC")
