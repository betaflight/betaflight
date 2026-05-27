/* Entry Point */
ENTRY(_start)


MEMORY
{
    /*             origin         length       */
    FLASH    (rx)  : ORIGIN = 0x15020000, LENGTH = 0x00190000   /* 1600 KB NOR-Flash (XIP) */
    FLASH_CONFIG    (r)  : ORIGIN = 0x151C0000, LENGTH = 0x00020000   /* 128 KB NOR-Flash (XIP) */
    ITCM     (xrw) : ORIGIN = 0x00000000, LENGTH = 0x0009FC00   /* 640 KB - 1KB            */
    DTCM     (xrw) : ORIGIN = 0x20000000, LENGTH = 0x00060000   /* 384 KB                  */
    AXI_SRAM (xrw) : ORIGIN = 0x24000000, LENGTH = 0x00020000   /* 128 KB                   */
    AHB_SRAM (xrw) : ORIGIN = 0x30000000, LENGTH = 0x00058000   /* 352 KB                   */
}


/* Entry Point */
ENTRY(_start)

_estack = ORIGIN(AHB_SRAM) + LENGTH(AHB_SRAM);   /* boot-time stack (AHB_SRAM top) */
_app_estack = ORIGIN(DTCM) + LENGTH(DTCM) - 8;       /* app-time stack (DTCM top)      */

__config_start = ORIGIN(FLASH_CONFIG);
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

_Min_Heap_Size  = 0x1000;
_Min_Stack_Size = 0x1000;


SECTIONS
{
    /* ===== Stage 1: Boot code in NOR-Flash (XIP) ===== */

    /* Boot vector table: SP + Reset_Handler, placed at FLASH base */
    .isr_vector_boot :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector_boot))
        . = ALIGN(4);
    } > FLASH

    /* Boot text: reset handler, lowlevel_init, _main, copy routines, Default_Handler
     *
     * SDK object files that are called during boot (before TCM copy) must also
     * be placed here.  
     *
     * We use a wildcard object-file pattern to match regardless of
     * build-dir prefix, for example ".../system_x32m7xx.o".
     */
    .text.boot :
    {
        . = ALIGN(4);
        KEEP(*(.text.reset))
        KEEP(*(.text.Default_Handler))
        *(.boot_text)
        *(.boot_text*)
        *(.rodata.boot)
        *(.rodata.boot*)

        /* SDK: system init -- called from lowlevel_init via SystemInit() */
        */system_x32m7xx.o(.text*)
        */system_x32m7xx.o(.rodata*)

        . = ALIGN(4);
    } > FLASH

    /* ===== Stage 2: Runtime image copied into ITCM as one block ===== */

    .runtime_image :
    {
        . = ALIGN(4);
        __image_copy_start__ = .;

        /* Runtime vector table sits at the start of the relocated image. */
        __tcm_vector_start__ = .;
        KEEP(*(.tcm_vector))
        __tcm_vector_end__ = .;

        . = ALIGN(4);
        /* Application code follows contiguously so _main() can copy one image. */
        __itcm_text_start__ = .;
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        __itcm_text_end__ = .;

        __image_copy_end__ = .;
    } > ITCM AT > FLASH

    __image_copy_load_start__ = LOADADDR(.runtime_image);
    __tcm_vector_load_start__ = __image_copy_load_start__;
    __itcm_text_load_start__ = __image_copy_load_start__ + (__itcm_text_start__ - __image_copy_start__);

    __tcm_vector_copy_src_start__ = __image_copy_load_start__ + 250 * 4;  
    __tcm_vector_copy_dst_start__ = __image_copy_start__ + 15 * 4;       
    __tcm_vector_copy_end__ = __tcm_vector_copy_dst_start__ + 4;         

    .ARM.extab   :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*) 
        __exidx_end = .;
    } >FLASH

    .pg_registry :
    {
        PROVIDE_HIDDEN (__pg_registry_start = .);
        KEEP (*(.pg_registry))
        KEEP (*(SORT(.pg_registry.*)))
        PROVIDE_HIDDEN (__pg_registry_end = .);
        . = ALIGN(4);
    } >FLASH

    .pg_resetdata :
    {
        PROVIDE_HIDDEN (__pg_resetdata_start = .);
        KEEP (*(.pg_resetdata))
        PROVIDE_HIDDEN (__pg_resetdata_end = .);
        . = ALIGN(4);
    } >FLASH



    /* ===== Stage 3: DMA-safe and general data/bss in AHB_SRAM ===== */

    _sdmaram_idata = LOADADDR(.dmaram_data);

    .dmaram_data :
    {
        . = ALIGN(32);
        _dmaram_start__ = .;
        _sdmaram_data = .;
        *(.dmaram_data)
        *(.dmaram_data*)
        . = ALIGN(32);
        _edmaram_data = .;
    } > AHB_SRAM AT > FLASH

    .dmaram_bss (NOLOAD) :
    {
        . = ALIGN(32);
        _sdmaram_bss = .;
        __dmaram_bss_start__ = _sdmaram_bss;
        *(.dmaram_bss)
        *(SORT_BY_ALIGNMENT(.dmaram_bss*))
        KEEP(*(SORT_BY_ALIGNMENT(.DMA_RAM*)))
        KEEP(*(SORT_BY_ALIGNMENT(.DMA_RW_AXI*)))
        . = ALIGN(32);
        _edmaram_bss = .;
        __dmaram_bss_end__ = _edmaram_bss;
        _dmaram_end__ = .;
    } > AHB_SRAM

    _sfastram_idata = LOADADDR(.fastram_data);

    .fastram_data :
    {
        . = ALIGN(4);
        _sfastram_data = .;
        *(.fastram_data)
        *(.fastram_data*)
        . = ALIGN(4);
        _efastram_data = .;
    } > AHB_SRAM AT > FLASH

    .fastram_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sfastram_bss = .;
        __fastram_bss_start__ = _sfastram_bss;
        *(.fastram_bss)
        *(SORT_BY_ALIGNMENT(.fastram_bss*))
        . = ALIGN(4);
        _efastram_bss = .;
        __fastram_bss_end__ = _efastram_bss;
    } > AHB_SRAM

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > AHB_SRAM AT > FLASH
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > AHB_SRAM

    /* Crash info section - preserved across resets for post-mortem debugging */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        __noinit_start__ = .;
        *(.noinit)
        *(.noinit*)
        . = ALIGN(4);
        __noinit_end__ = .;
    } > AHB_SRAM

    /* ===== Stage 4: TCM data/bss in DTCM ===== */

    .app_data :
    {
        . = ALIGN(4);
        __app_data_start__ = .;
        *(.tcm_data)
        *(.tcm_data*)
        . = ALIGN(4);
        __app_data_end__ = .;
    } > DTCM AT > FLASH
    __app_data_load_start__ = LOADADDR(.app_data);

    .app_bss (NOLOAD) :
    {
        . = ALIGN(4);
        __app_bss_start__ = .;
        *(.tcm_bss)
        *(.tcm_bss*)
        . = ALIGN(4);
        __app_bss_end__ = .;
    } > DTCM

    /* ===== Heap & Stack ===== */

    ._user_heap_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > AHB_SRAM

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
