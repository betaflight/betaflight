#!/usr/bin/env python3
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from hashlib import md5
import json
import logging
import os

import requests


HEADER_FILE_TEMPLATE = """{license_header}

{generated_warning}

#pragma once

#include "common/streambuf.h"

{defines}

void sbufWriteBuildInfoFlags(sbuf_t *dst);
"""

SOURCE_FILE_TEMPLATE = """{license_header}

{generated_warning}

#include <stdint.h>

#include "platform.h"

#include "common/streambuf.h"

#include "msp/msp_build_info.h"

void sbufWriteBuildInfoFlags(sbuf_t *dst)
{
    static const uint16_t options[] = {
{build_options}
    };

    for (unsigned i = 0; i < ARRAYLEN(options); i++)
    {
        sbufWriteU16(dst, options[i]);
    }
}
"""

WARNING_COMMENT_TEMPLATE = """
/*
 * WARNING: This is an auto-generated file, please do not edit directly!
 *
 * Generator    : `src/utils/make-build-info.py`
 * Source       : {source}
 * Input hash   : {input_hash}
 */
"""


def __find_project_root() -> str:
    utils_dir = os.path.abspath(os.path.dirname(__file__))
    src_dir = os.path.dirname(utils_dir)
    root_dir = os.path.dirname(src_dir)
    return os.path.realpath(root_dir)


def camel_case_to_title(s: str) -> str:
    if not s:
        return "Unspecified"
    else:
        spaceless = s.replace(" ", "")
        return "".join([(c if not c.isupper() else f" {c}") for c in spaceless]) \
            .lstrip() \
            .title()


def fetch_build_options(endpoint_url: str) -> tuple:
    logging.info(f"Fetching JSON: {endpoint_url}")
    data = requests.get(endpoint_url, timeout=2).json()
    input_hash = md5(json.dumps(data, sort_keys=True).encode()).hexdigest()
    logging.info(f"Input hash: {input_hash}")

    defines  = []
    options = []
    groups = list(data.keys())
    for group_index, option_list in enumerate(data.values()):
        for option in option_list:
            define = option.get("value")
            if define:
                defines.append(define)
                number = option.get("key")
                name = define.replace("USE_", "BUILD_OPTION_")
                options.append((name, number, camel_case_to_title(groups[group_index])))
    logging.info(f"Number of defines: {len(defines)}")
    return defines, options, input_hash


def get_warning_comment(source: str, input_hash: str) -> str:
    return WARNING_COMMENT_TEMPLATE \
        .strip() \
        .replace("{source}", source) \
        .replace("{input_hash}", input_hash) \


def main(root_path: str, target_path: str, endpoint_url: str):
    logging.info(f"Project root: {root_path}")

    license_file_path = os.path.join(root_path, "DEFAULT_LICENSE.md")
    msp_build_info_c_path = os.path.join(target_path, "msp_build_info.c")
    msp_build_info_h_path = os.path.join(target_path, "msp_build_info.h")

    with open(license_file_path) as f:
        license_header = f.read().rstrip()

    gates, options, input_hash = fetch_build_options(endpoint_url)

    generated_warning = get_warning_comment(endpoint_url, input_hash)

    with open(msp_build_info_h_path, "w+") as f:
        max_len = max(map(lambda x: len(x[0]), options)) + 4
        lines = []
        last_group = None
        for option_name, option_value, group in options:
            if group != last_group:
                lines.append(f"// {group}")
                last_group = group
            lines.append(f"#define {option_name:<{max_len}}{option_value}")
        data = HEADER_FILE_TEMPLATE \
            .replace("{license_header}", license_header) \
            .replace("{generated_warning}", generated_warning) \
            .replace("{defines}", "\n".join(lines))
        f.write(data)

    logging.info(f"Written header file: {msp_build_info_h_path}")

    with open(msp_build_info_c_path, "w+") as f:
        lines = []
        indent = " " * 8
        for i, define in enumerate(gates):
            option_name, _, _ = options[i]
            lines.append(f"#ifdef {define}")
            lines.append(f"{indent}{option_name},")
            lines.append("#endif")
        data = SOURCE_FILE_TEMPLATE \
            .replace("{license_header}", license_header) \
            .replace("{generated_warning}", generated_warning) \
            .replace("{build_options}", "\n".join(lines))
        f.write(data)

    logging.info(f"Written source file: {msp_build_info_c_path}")


if __name__ == "__main__":
    PROJECT_ROOT_DIR = __find_project_root()
    DEFAULT_TARGET_DIR = os.path.join(PROJECT_ROOT_DIR, "src", "main", "msp")

    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("endpoint_url", help="URL to build options API endpoint")
    parser.add_argument("-d", "--target-dir", default=DEFAULT_TARGET_DIR, help="Path to output directory")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO if args.verbose else logging.ERROR)

    main(
        root_path=PROJECT_ROOT_DIR,
        target_path=args.target_dir,
        endpoint_url=args.endpoint_url,
    )
