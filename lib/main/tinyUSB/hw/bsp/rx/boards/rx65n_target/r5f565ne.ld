__USTACK_SIZE = 0x00000800;
__ISTACK_SIZE = 0x00000800;

MEMORY
{
	RAM  : ORIGIN = 0x4,        LENGTH = 0x3fffc
	RAM2 : ORIGIN = 0x00800000, LENGTH = 0x60000
	OFS  : ORIGIN = 0xFE7F5D00, LENGTH = 128
	ROM  : ORIGIN = 0xFFE00000, LENGTH = 0x200000
}
SECTIONS
{
	.exvectors 0xFFFFFF80: AT(0xFFFFFF80)
	{
		"_exvectors_start" = .;
		KEEP(*(.exvectors))
		"_exvectors_end" = .;
	} >ROM
	.fvectors 0xFFFFFFFC: AT(0xFFFFFFFC)
	{
		KEEP(*(.fvectors))
	} > ROM
	.text 0xFFE00000: AT(0xFFE00000)
	{
		*(.text)
		*(.text.*)
		*(P)
		KEEP(*(.text.*_isr))
		etext = .;
	} > ROM
	.rvectors ALIGN(4):
	{
		_rvectors_start = .;
		KEEP(*(.rvectors))
		_rvectors_end = .;
	} > ROM
	.init :
	{
		KEEP(*(.init))
		__preinit_array_start = .;
		KEEP(*(.preinit_array))
		__preinit_array_end = .;
		__init_array_start = (. + 3) & ~ 3;
		KEEP(*(.init_array))
		KEEP(*(SORT(.init_array.*)))
		__init_array_end = .;
		__fini_array_start = .;
		KEEP(*(.fini_array))
		KEEP(*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} > ROM
	.fini :
	{
		KEEP(*(.fini))
	} > ROM
	.got :
	{
		*(.got)
		*(.got.plt)
	} > ROM
	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
		*(C_1)
		*(C_2)
		*(C)
		_erodata = .;
	} > ROM
	.eh_frame_hdr :
	{
		*(.eh_frame_hdr)
	} > ROM
	.eh_frame :
	{
		*(.eh_frame)
	} > ROM
	.jcr :
	{
		*(.jcr)
	} > ROM
	.tors :
	{
		__CTOR_LIST__ = .;
		. = ALIGN(2);
		___ctors = .;
		*(.ctors)
		___ctors_end = .;
		__CTOR_END__ = .;
		__DTOR_LIST__ = .;
		___dtors = .;
		*(.dtors)
		___dtors_end = .;
		__DTOR_END__ = .;
		. = ALIGN(2);
		_mdata = .;
	} > ROM
	.data : AT(_mdata)
	{
		_data = .;
		*(.data)
		*(.data.*)
		*(D)
		*(D_1)
		*(D_2)
		_edata = .;
	} > RAM
	.gcc_exc :
	{
		*(.gcc_exc)
	} > RAM
	.bss :
	{
		_bss = .;
		*(.bss)
		*(.bss.**)
		*(COMMON)
		*(B)
		*(B_1)
		*(B_2)
		_ebss = .;
		_end = .;
	} > RAM
	.ustack :
	{
		. = ALIGN(8);
		. = . + __USTACK_SIZE;
		PROVIDE(_ustack = .);
	} > RAM
	.istack :
	{
		. = ALIGN(8);
		. = . + __ISTACK_SIZE;
		PROVIDE(_istack = .);
	} > RAM
	.ofs1 0xFE7F5D00: AT(0xFE7F5D00)
	{
		KEEP(*(.ofs1))
	} > OFS
	.ofs2 0xFE7F5D10: AT(0xFE7F5D10)
	{
		KEEP(*(.ofs2))
	} > OFS
	.ofs3 0xFE7F5D20: AT(0xFE7F5D20)
	{
		KEEP(*(.ofs3))
	} > OFS
	.ofs4 0xFE7F5D40: AT(0xFE7F5D40)
	{
		KEEP(*(.ofs4))
	} > OFS
	.ofs5 0xFE7F5D48: AT(0xFE7F5D48)
	{
		KEEP(*(.ofs5))
	} > OFS
	.ofs6 0xFE7F5D50: AT(0xFE7F5D50)
	{
		KEEP(*(.ofs6))
	} > OFS
	.ofs7 0xFE7F5D64: AT(0xFE7F5D64)
	{
		KEEP(*(.ofs7))
	} > OFS
	.ofs8 0xFE7F5D70: AT(0xFE7F5D70)
	{
		KEEP(*(.ofs8))
	} > OFS
}
