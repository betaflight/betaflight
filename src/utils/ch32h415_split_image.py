#!/usr/bin/env python3
#
# This file is part of Betaflight.
#
# Betaflight is free software. You can redistribute this software
# and/or modify this software under the terms of the GNU General
# Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later
# version.
#
# Betaflight is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public
# License along with this software.
#
# If not, see <http://www.gnu.org/licenses/>.
#
"""Split an H415 V5F binary into writable ranges around its live config.

The binary is linked from 0x08016000 and contains an objcopy padding hole
over the configuration sectors. Never flash that hole during a firmware update.
The entry range is listed last, after its code range. An interrupted update
can still leave the previous entry pointing at a partially written image.
"""
import argparse
import hashlib
import json
from pathlib import Path

BASE = 0x08016000
CONFIG_START = 0x08018000
CONFIG_END = 0x0801C000
END = 0x080F0000


def split_image(data):
    if not CONFIG_END - BASE < len(data) <= END - BASE:
        raise ValueError("image does not fit the H415 V5F layout")
    hole = data[CONFIG_START - BASE:CONFIG_END - BASE]
    if any(hole):
        raise ValueError("image contains non-padding bytes in the config region")
    return [(CONFIG_END, data[CONFIG_END - BASE:], "code.bin"),
            (BASE, data[:CONFIG_START - BASE], "entry.bin")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    data = args.image.read_bytes()
    parts = split_image(data)
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {"source": str(args.image), "sha256": hashlib.sha256(data).hexdigest(),
                "preserved": [hex(CONFIG_START), hex(CONFIG_END)], "flash_order": []}
    for address, payload, name in parts:
        (args.output / name).write_bytes(payload)
        manifest["flash_order"].append({"file": name, "address": hex(address),
                                       "bytes": len(payload),
                                       "sha256": hashlib.sha256(payload).hexdigest()})
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
