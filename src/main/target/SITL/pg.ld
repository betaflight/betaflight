

SECTIONS {
  /* BLOCK: on Windows (PE) output section must be page-aligned. Use 4-byte alignment otherwise */
  /* SUBALIGN: force 4-byte alignment of input sections for pg_registry.
     Gcc defaults to 32 bytes; padding is then inserted between object files, breaking the init structure. */
  .pg_registry BLOCK( DEFINED(__section_alignment__) ? __section_alignment__ : 4 ) :   SUBALIGN(4)
  {
    PROVIDE_HIDDEN (__pg_registry_start = . );
    PROVIDE_HIDDEN (___pg_registry_start = . );
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = . );
    PROVIDE_HIDDEN (___pg_registry_end = . );

    PROVIDE_HIDDEN (__pg_resetdata_start = . );
    PROVIDE_HIDDEN (___pg_resetdata_start = . );
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = . );
    PROVIDE_HIDDEN (___pg_resetdata_end = . );
  }
}
INSERT AFTER .text;



