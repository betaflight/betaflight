/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !PICO_NO_BINARY_INFO && !PICO_NO_PROGRAM_INFO
#include "pico/binary_info.h"

#if LIB_BOOT_STAGE2_HEADERS && !PICO_NO_FLASH
#include "boot_stage2/config.h"
#endif

// Note we put at most 4 pieces of binary info in the binary_info_header section because that's how much spare space we
// have before the vector table in a RAM binary (we use the attribute for the most common ones since the choice is static)...
// if there is a link failure because of .reset section overflow then move more out.
#if PICO_NO_FLASH
#define section_hack_attr __attribute__((section(".binary_info_header")))
#else
#define section_hack_attr
#endif

#if !PICO_NO_FLASH
#ifndef PICO_NO_BI_BINARY_SIZE
extern char __flash_binary_end;
bi_decl_with_attr(bi_binary_end((intptr_t)&__flash_binary_end), section_hack_attr)
#endif
#endif

#if !PICO_NO_BI_PROGRAM_BUILD_DATE
#ifndef PICO_PROGRAM_BUILD_DATE
#define PICO_PROGRAM_BUILD_DATE __DATE__
#endif
bi_decl_with_attr(bi_program_build_date_string(PICO_PROGRAM_BUILD_DATE), section_hack_attr);
#endif

#if !PICO_NO_BI_PROGRAM_NAME
#if !defined(PICO_PROGRAM_NAME) && defined(PICO_TARGET_NAME)
#define PICO_PROGRAM_NAME PICO_TARGET_NAME
#endif
#ifdef PICO_PROGRAM_NAME
bi_decl_with_attr(bi_program_name(PICO_PROGRAM_NAME), section_hack_attr)
#endif
#endif

#if !PICO_NO_BI_PICO_BOARD
#ifdef PICO_BOARD
bi_decl(bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PICO_BOARD, PICO_BOARD))
#endif
#endif

#if !PICO_NO_BI_SDK_VERSION
#ifdef PICO_SDK_VERSION_STRING
bi_decl_with_attr(bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_SDK_VERSION, PICO_SDK_VERSION_STRING), section_hack_attr)
#endif
#endif

#if !PICO_NO_BI_PROGRAM_VERSION_STRING
#ifdef PICO_PROGRAM_VERSION_STRING
bi_decl(bi_program_version_string(PICO_PROGRAM_VERSION_STRING))
#endif
#endif

#if !PICO_NO_BI_PROGRAM_DESCRIPTION
#ifdef PICO_PROGRAM_DESCRIPTION
bi_decl(bi_program_description(PICO_PROGRAM_DESCRIPTION))
#endif
#endif

#if !PICO_NO_BI_PROGRAM_URL
#ifdef PICO_PROGRAM_URL
bi_decl(bi_program_url(PICO_PROGRAM_URL))
#endif
#endif

#if !PICO_NO_BI_BOOT_STAGE2_NAME
#ifdef PICO_BOOT_STAGE2_NAME
bi_decl(bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_BOOT2_NAME, PICO_BOOT_STAGE2_NAME))
#endif
#endif

#if !PICO_NO_BI_BUILD_TYPE
#ifdef PICO_CMAKE_BUILD_TYPE
bi_decl(bi_program_build_attribute(PICO_CMAKE_BUILD_TYPE))
#else
#ifndef NDEBUG
bi_decl(bi_program_build_attribute("Debug"))
#else
bi_decl(bi_program_build_attribute("Release"))
#endif
#endif

#if PICO_DEOPTIMIZED_DEBUG
bi_decl(bi_program_build_attribute("All optimization disabled"))
#endif
#endif

#endif
