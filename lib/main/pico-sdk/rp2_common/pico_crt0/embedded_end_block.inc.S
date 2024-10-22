#if PICO_CRT0_INCLUDE_PICOBIN_END_BLOCK
.section .embedded_end_block, "a"
embedded_end_block:
.word PICOBIN_BLOCK_MARKER_START

.byte PICOBIN_BLOCK_ITEM_2BS_IGNORED
.byte 0x1 // 1 word
.hword 0

.byte PICOBIN_BLOCK_ITEM_2BS_LAST
.hword (embedded_end_block_end - embedded_end_block - 16 ) / 4 // total size of all items
.byte 0
// offset from this block to first block
.word embedded_block - embedded_end_block
.word PICOBIN_BLOCK_MARKER_END
embedded_end_block_end:
#endif
