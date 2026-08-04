/*
 * ESP-IDF application descriptor.
 *
 * The second-stage bootloader reads this 256-byte structure from the very
 * start of the first mapped (DROM) segment of the app image. It validates the
 * magic word and uses the eFuse-block-revision fields to decide whether the
 * image may run on this chip. Without it the bootloader interprets arbitrary
 * rodata as the descriptor and rejects the image ("Image requires efuse blk
 * rev >= v..."). The linker (esp32s3.ld) places section .rodata_desc first in
 * the DROM window so this lands at the required offset.
 *
 * The layout mirrors esp_app_desc_t in ESP-IDF (components/esp_app_format);
 * it is reproduced here so the betaflight build stays independent of the IDF
 * application framework. min/max efuse revision are left "unset" (0 / 65535)
 * so the bootloader skips the revision gate.
 */

#include <stdint.h>

#include "build/version.h"

#define ESP_APP_DESC_MAGIC_WORD 0xABCD5432

typedef struct {
    uint32_t magic_word;
    uint32_t secure_version;
    uint32_t reserv1[2];
    char version[32];
    char project_name[32];
    char time[16];
    char date[16];
    char idf_ver[32];
    uint8_t app_elf_sha256[32];
    uint16_t min_efuse_blk_rev_full;
    uint16_t max_efuse_blk_rev_full;
    uint8_t mmu_page_size;
    uint8_t reserv3[3];
    uint32_t reserv2[18];
} esp_app_desc_t;

__attribute__((used, section(".rodata_desc")))
const esp_app_desc_t esp_app_desc = {
    .magic_word = ESP_APP_DESC_MAGIC_WORD,
    .secure_version = 0,
    .version = FC_VERSION_STRING,
    .project_name = "betaflight",
    .time = __TIME__,
    .date = __DATE__,
    .idf_ver = "v5.4",
    .min_efuse_blk_rev_full = 0,        /* 0 => bootloader skips the min check */
    .max_efuse_blk_rev_full = 65535,    /* 65535 => no maximum */
    .mmu_page_size = 16,                /* log2(64 KiB); ignored on ESP32-S3 */
};
