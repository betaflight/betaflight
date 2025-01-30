/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_BINARY_INFO_CODE_H
#define _PICO_BINARY_INFO_CODE_H

// pico.h is not available when PICO_NO_BINARY_INFO=1 is used for builds outside of the SDK (e.g. picotool)
// and only needed anyway (because of macro definitions) in PICO_NO_BINARY_INFO=0 builds
#if !PICO_NO_BINARY_INFO
#include "pico.h"
#endif

#include "pico/binary_info/structure.h"

#if !PICO_NO_BINARY_INFO
#define __bi_decl(name, bi, section_prefix, attr) static const attr __attribute__((section(section_prefix __STRING(name)))) struct _binary_info_core *const name = bi
#define __bi_lineno_var_name __CONCAT(__bi_, __LINE__)
#define __bi_ptr_lineno_var_name __CONCAT(__bi_ptr, __LINE__)
#define __bi_enclosure_check_lineno_var_name __CONCAT(_error_bi_is_missing_enclosing_decl_,__LINE__)
#define __bi_mark_enclosure static const __unused int __bi_enclosure_check_lineno_var_name=0;
#if __cplusplus || __GNUC__ >= 8
#define __bi_enclosure_check(x) (x + __bi_enclosure_check_lineno_var_name)
#else
// skip the version check on older GCC non C++, as it doesn't compile.. this is only here to catch the
// user accidentally forgetting to enclose the binary item with bi_decl
#define __bi_enclosure_check(x) (x)
#endif
/**
 * \brief Declare some binary information that will be included if the contain source file/line is compiled into the binary
 * \ingroup pico_binary_info
 */
#define bi_decl(_decl) __bi_mark_enclosure _decl; __bi_decl(__bi_ptr_lineno_var_name, &__bi_lineno_var_name.core, ".binary_info.keep.", __used);
/**
 * \brief Declare some binary information that will be included if the function containing the decl is linked into the binary.
 * The SDK uses --gc-sections, so functions that are never called will be removed by the linker, and any associated
 * binary information declared this way will also be stripped
 * \ingroup pico_binary_info
 */
#define bi_decl_if_func_used(_decl) ({__bi_mark_enclosure _decl; __bi_decl(__bi_ptr_lineno_var_name, &__bi_lineno_var_name.core, ".binary_info.", ); *(const volatile uint8_t *)&__bi_ptr_lineno_var_name;});

#define bi_decl_with_attr(_decl, _attr) __bi_mark_enclosure _attr _decl; __bi_decl(__bi_ptr_lineno_var_name, &__bi_lineno_var_name.core, ".binary_info.keep.", __used);
#define bi_decl_if_func_used_with_attr(_decl, _attr) ({__bi_mark_enclosure _attr _decl; __bi_decl(__bi_ptr_lineno_var_name, &__bi_lineno_var_name.core, ".binary_info.", ); *(const volatile uint8_t *)&__bi_ptr_lineno_var_name;});
#else
#define __bi_decl(bi, name, attr)
#define bi_decl_with_attr(_decl, _attr)
#define bi_decl(_decl)
#define bi_decl_if_func_used_with_attr(_decl, _attr) ((void)0);
#define bi_decl_if_func_used(_decl) ((void)0);
#endif

#define bi_int(_tag, _id, _value) \
     static const struct _binary_info_id_and_int __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_ID_AND_INT), \
            .tag = _tag, \
        },\
        .id = _id, \
        .value = _value \
    };

#define bi_string(_tag, _id, _value) \
    static const struct _binary_info_id_and_string __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_ID_AND_STRING), \
            .tag = _tag, \
        },\
        .id = _id, \
        .value = _value, \
    }

#define __bi_ptr_int32_with_name(_tag, _id, _label, _value) \
    static const struct _binary_info_ptr_int32_with_name __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PTR_INT32_WITH_NAME), \
            .tag = _tag, \
        },\
        .id = _id, \
        .value = &_value, \
        .label = _label, \
    }

#define bi_ptr_int32(_tag, _id, _var, _default) __attribute__((section(".data"))) static int _var = _default; __bi_ptr_int32_with_name(_tag, _id, __STRING(_var), _var)

#define __bi_ptr_string_with_name(_tag, _id, _label, _value, _len) \
    static const struct _binary_info_ptr_string_with_name __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PTR_STRING_WITH_NAME), \
            .tag = _tag, \
        },\
        .id = _id, \
        .value = _value, \
        .label = _label, \
        .len = _len, \
    }

#define bi_ptr_string(_tag, _id, _var, _default, _max_len) static char _var[_max_len] = _default; __bi_ptr_string_with_name(_tag, _id, __STRING(_var), _var, _max_len)

#define bi_block_device(_tag, _name, _address, _size, _extra, _flags) \
    static const struct _binary_info_block_device __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_BLOCK_DEVICE), \
            .tag = _tag, \
        },\
        .name = _name, \
        .address = _address, \
        .size = _size, \
        .extra = _extra, \
        .flags = _flags, \
    }

#define __bi_encoded_pins_with_func(_encoding) \
    static const struct _binary_info_pins_with_func __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PINS_WITH_FUNC), \
            .tag = BINARY_INFO_TAG_RASPBERRY_PI, \
        },\
        .pin_encoding = _encoding \
    }

#define __bi_encoded_pins_64_with_func(_encoding) \
    static const struct _binary_info_pins64_with_func __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PINS64_WITH_FUNC), \
            .tag = BINARY_INFO_TAG_RASPBERRY_PI, \
        },\
        .pin_encoding = _encoding \
    }

#define __bi_pins_with_name(_mask, _label) \
    static const struct _binary_info_pins_with_name __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PINS_WITH_NAME), \
            .tag = BINARY_INFO_TAG_RASPBERRY_PI, \
        },\
        .pin_mask = _mask, \
        .label = _label \
    }

#define __bi_pins_64_with_name(_mask, _label) \
    static const struct _binary_info_pins64_with_name __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_PINS64_WITH_NAME), \
            .tag = BINARY_INFO_TAG_RASPBERRY_PI, \
        },\
        .pin_mask = _mask, \
        .label = _label \
    }

#define __bi_named_group(_parent_tag, _parent_id, _group_tag, _group_id, _label, _flags) \
static const struct _binary_info_named_group __bi_lineno_var_name = { \
        .core = { \
            .type = __bi_enclosure_check(BINARY_INFO_TYPE_NAMED_GROUP), \
            .tag = _parent_tag, \
        },\
        .parent_id = _parent_id, \
        .group_tag = _group_tag, \
        .flags = _flags, \
        .group_id = _group_id, \
        .label = _label \
    }

#define bi_binary_end(end) bi_int(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_BINARY_END, end)
#define bi_program_name(name) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_NAME, name)
#define bi_program_description(description) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_DESCRIPTION, description)
#define bi_program_version_string(version_string) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_VERSION_STRING, version_string)
#define bi_program_build_date_string(date_string) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_BUILD_DATE_STRING, date_string)
#define bi_program_url(url) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_URL, url)
// multiple of these may be added
#define bi_program_feature(feature) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_FEATURE, feature)
#define bi_program_build_attribute(attr) bi_string(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_BUILD_ATTRIBUTE, attr)
#define bi_program_feature_group(tag, id, name) __bi_named_group(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_FEATURE, tag, id, name, 0)
#define bi_program_feature_group_with_flags(tag, id, name, flags) __bi_named_group(BINARY_INFO_TAG_RASPBERRY_PI, BINARY_INFO_ID_RP_PROGRAM_FEATURE, tag, id, name, flags)


#ifndef PICO_BINARY_INFO_USE_PINS_64
#define PICO_BINARY_INFO_USE_PINS_64 (NUM_BANK0_GPIOS > 32)
#endif

#if !PICO_BINARY_INFO_USE_PINS_64
#define bi_1pin_with_func(p0, func)                  __bi_encoded_pins_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 7) | ((p0) << 12))
#define bi_2pins_with_func(p0, p1, func)             __bi_encoded_pins_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 7) | ((p1) << 12) | ((p1) << 17))
#define bi_3pins_with_func(p0, p1, p2, func)         __bi_encoded_pins_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 7) | ((p1) << 12) | ((p2) << 17) | ((p2) << 22))
#define bi_4pins_with_func(p0, p1, p2, p3, func)     __bi_encoded_pins_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 7) | ((p1) << 12) | ((p2) << 17) | ((p3) << 22) | ((p3) << 27))
#define bi_5pins_with_func(p0, p1, p2, p3, p4, func) __bi_encoded_pins_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 7) | ((p1) << 12) | ((p2) << 17) | ((p3) << 22) | ((p4) << 27))
#define bi_pin_range_with_func(plo, phi, func)       __bi_encoded_pins_with_func(BI_PINS_ENCODING_RANGE | ((func << 3)) | ((plo) << 7) | ((phi) << 12))

#define bi_pin_mask_with_name(pmask, label)          __bi_pins_with_name((pmask), (label))
// names are separated by | ... i.e. "name1|name2|name3"
#define bi_pin_mask_with_names(pmask, label)          __bi_pins_with_name((pmask), (label))
#else
#define bi_1pin_with_func(p0, func)                         __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p0) << 16))
#define bi_2pins_with_func(p0, p1, func)                    __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p1) << 24))
#define bi_3pins_with_func(p0, p1, p2, func)                __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p2) << 24) | ((uint64_t)(p2) << 32))
#define bi_4pins_with_func(p0, p1, p2, p3, func)            __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p2) << 24) | ((uint64_t)(p3) << 32) | ((uint64_t)(p3) << 40))
#define bi_5pins_with_func(p0, p1, p2, p3, p4, func)        __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p2) << 24) | ((uint64_t)(p3) << 32) | ((uint64_t)(p4) << 40) | ((uint64_t)(p4) << 48))
#define bi_pin_range_with_func(plo, phi, func)              __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_RANGE | ((func << 3)) | ((plo) << 8) | ((phi) << 16))

#define bi_pin_mask_with_name(pmask, label)          __bi_pins_64_with_name((uint64_t)(pmask), (label))
// names are separated by | ... i.e. "name1|name2|name3"
#define bi_pin_mask_with_names(pmask, label)          __bi_pins_64_with_name((uint64_t)(pmask), (label))
#endif

// 6 and 7 pins require pins_64
#define bi_6pins_with_func(p0, p1, p2, p3, p4, p5, func)    __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p2) << 24) | ((uint64_t)(p3) << 32) | ((uint64_t)(p4) << 40) | ((uint64_t)(p5) << 48) | ((uint64_t)(p5) << 56))
#define bi_7pins_with_func(p0, p1, p2, p3, p4, p5, p6,func) __bi_encoded_pins_64_with_func(BI_PINS_ENCODING_MULTI | ((func << 3)) | ((p0) << 8) | ((p1) << 16) | ((p2) << 24) | ((uint64_t)(p3) << 32) | ((uint64_t)(p4) << 40) | ((uint64_t)(p5) << 48) | ((uint64_t)(p6) << 56))

#define bi_1pin_with_name(p0, name)                   bi_pin_mask_with_name(1ull << (p0), name)
#define bi_2pins_with_names(p0, name0, p1, name1)     bi_pin_mask_with_names((1ull << (p0)) | (1ull << (p1)), p0 < p1 ? name0 "|" name1 : name1 "|" name0)
#define bi_3pins_with_names(p0, name0, p1, name1, p2, name2)  bi_pin_mask_with_names((1ull << (p0)) | (1ull << (p1)) | (1ull << (p2)),\
    p0 < p1 ?\
        (p1 < p2 ?\
            name0 "|" name1 "|" name2:\
            (p0 < p2 ? name0 "|" name2 "|" name1 : name2 "|" name0 "|" name1)):\
        (p1 < p2 ?\
            (p0 < p2 ? name1 "|" name0 "|" name2 : name1 "|" name2 "|" name0) :\
            name2 "|" name1 "|" name0))
#define bi_4pins_with_names(p0, name0, p1, name1, p2, name2, p3, name3)  bi_pin_mask_with_names((1ull << (p0)) | (1ull << (p1)) | (1ull << (p2)) | (1ull << (p3)),\
    p0 < p1 ?\
        (p1 < p2 ?\
            (p2 < p3 ?\
                name0 "|" name1 "|" name2 "|" name3:\
                (p0 < p3 ?\
                    (p1 < p3 ?\
                        name0 "|" name1 "|" name3 "|" name2:\
                        name0 "|" name3 "|" name1 "|" name2):\
                    name3 "|" name0 "|" name1 "|" name2)):\
            (p2 < p3 ?\
                (p0 < p2 ?\
                    (p1 < p3 ?\
                        name0 "|" name2 "|" name1 "|" name3:\
                        name0 "|" name2 "|" name3 "|" name1):\
                    (p0 < p3 ?\
                        (p1 < p3 ?\
                            name2 "|" name0 "|" name1 "|" name3:\
                            name2 "|" name0 "|" name3 "|" name1):\
                        name2 "|" name3 "|" name0 "|" name1)):\
                (p0 < p2 ?\
                    (p0 < p3 ?\
                        name0 "|" name3 "|" name2 "|" name1:\
                        name3 "|" name0 "|" name2 "|" name1):\
                    name3 "|" name2 "|" name0 "|" name1))):\
        (p1 < p2 ?\
            (p2 < p3 ?\
                (p0 < p2 ?\
                    name1 "|" name0 "|" name2 "|" name3:\
                    (p0 < p3 ?\
                        name1 "|" name2 "|" name0 "|" name3:\
                        name1 "|" name2 "|" name3 "|" name0)):\
                (p0 < p2 ?\
                    (p0 < p3 ?\
                        name1 "|" name0 "|" name3 "|" name2:\
                        (p1 < p3 ?\
                            name1 "|" name3 "|" name0 "|" name2:\
                            name3 "|" name1 "|" name0 "|" name2)):\
                    (p1 < p3 ?\
                        name1 "|" name3 "|" name2 "|" name0:\
                        name3 "|" name1 "|" name2 "|" name0))):\
            (p2 < p3 ?\
                (p0 < p3 ?\
                    name2 "|" name1 "|" name0 "|" name3:\
                    (p1 < p3 ?\
                        name2 "|" name1 "|" name3 "|" name0:\
                        name2 "|" name3 "|" name1 "|" name0)):\
                name3 "|" name2 "|" name1 "|" name0)))

#endif
