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

"""Check the ITCM and DTCM claims of link/nxp_itcm256k_sections.ld against the map file.

    check_placement.py <map file> <linker script>...

Prints ITCM, DTCM and OCRAM use, and exits 1 when a claim matches nothing, when a
claimed object has bytes in flash, or when either memory has under 8 KB free.

    check_placement.py --self-check

runs the parser and the matching over a known good and a known bad input.
"""

import fnmatch
import re
import sys

# Output sections whose memory region is named FLASH... hold the image in place;
# everything else is copied into a RAM the flight loop runs from.
FLASH_REGION = re.compile(r"^FLASH")

# A comment. It has to open at the start of a line or after a space: the slash
# star inside a wildcard such as */drivers/*.o opens nothing.
LD_COMMENT = re.compile(r"(?:^|(?<=\s))/\*.*?\*/", re.M | re.S)

# An output section, its body and the memory region it is placed in. No section
# body in the script nests braces.
LD_SECTION = re.compile(r"^\s*(\.[\w./]+)[^{}]*?\{([^{}]*)\}\s*>\s*(\w+)", re.M | re.S)

# One input section rule that names the objects it takes, as
# "*/flight/*.o(.text .text*)", possibly led by an EXCLUDE_FILE(...) of the
# objects it leaves out, or naming archive members as "*libm.a:member.o". Only
# rules whose file part names object files are kept: the bare *(...) fallbacks
# take whatever is left over and never decide where a named object goes, and
# the rest of the brackets in a linker script are ALIGN, KEEP and SORT.
LD_RULE = re.compile(r"(?:EXCLUDE_FILE\(([^()]*)\)\s*)?(?<![\w*])(\S*[^\s*]\S*)\(([^()]*)\)")

# A line of the map file that places bytes: one leading space, the input section
# name, then the address, the size and the object. Long names push the rest onto
# the next line.
MAP_PLACEMENT = re.compile(r"^ (\.\S+)(?:\s+(0x\S+)\s+(0x\S+)\s+(\S+))?$")
MAP_CONTINUED = re.compile(r"^\s+(0x\S+)\s+(0x\S+)\s+(\S+)$")

# A symbol assignment in the map file, as "0x0003b690   __itcm_end = .".
MAP_SYMBOL = re.compile(r"^\s+0x([0-9a-f]+)\s+(\w+) = ", re.M)

# An object the link reads, as "LOAD obj/main/T/fc/tasks.o".
MAP_LOAD = re.compile(r"^LOAD (\S+)$", re.M)

LD_FILE = "link/nxp_itcm256k_sections.ld"
MARGIN = 8 * 1024


def parse_ld(text):
    """Return the input section rules in script order, as (file, sections, region, excluded)."""
    text = LD_COMMENT.sub("", text)
    rules = []
    for _, body, region in LD_SECTION.findall(text):
        for excluded, filepat, sections in LD_RULE.findall(body):
            if filepat.endswith(".o"):
                # The map file names an archive member as archive(member).
                archive, colon, member = filepat.rpartition(":")
                if colon:
                    filepat = "%s(%s)" % (archive, member)
                rules.append((filepat, sections.split(), region, excluded.split()))
    return rules


def parse_map(text):
    """Return the placements the map file records, as (section, address, size, object)."""
    placements = []
    lines = text[text.find("\nLinker script and memory map\n"):].splitlines()
    for index, line in enumerate(lines):
        match = MAP_PLACEMENT.match(line)
        if not match:
            continue
        section, address, size, obj = match.groups()
        if address is None:
            following = MAP_CONTINUED.match(lines[index + 1]) if index + 1 < len(lines) else None
            if not following:
                continue
            address, size, obj = following.groups()
        if int(size, 16):
            placements.append((section, int(address, 16), int(size, 16), obj))
    return placements


def parse_regions(text, wanted):
    """Return the address ranges of the memory regions whose name matches wanted."""
    ranges = []
    for line in text.splitlines():
        if line.startswith("Linker script and memory map"):
            break
        fields = line.split()
        if len(fields) >= 3 and wanted.match(fields[0]):
            try:
                origin, length = int(fields[1], 16), int(fields[2], 16)
            except ValueError:
                continue
            ranges.append((origin, origin + length))
    return ranges


def usage(text):
    """Return the used and free bytes of ITCM, DTCM and OCRAM, the stack counted as used."""
    symbols = {}
    for value, name in MAP_SYMBOL.findall(text):
        symbols.setdefault(name, int(value, 16))
    itcm = parse_regions(text, re.compile(r"^ITCM$"))[0]
    dtcm = parse_regions(text, re.compile(r"^DTCM$"))[0]
    itcm_free = itcm[1] - symbols["__itcm_end"]
    dtcm_free = symbols["__stack_limit"] - symbols["__bss_end"]
    ocram = parse_regions(text, re.compile(r"^OCRAM2$"))[0]
    ocram_free = ocram[1] - symbols["__ocram2_end"]
    used = (itcm[1] - itcm[0] - itcm_free, itcm_free, dtcm[1] - dtcm[0] - dtcm_free, dtcm_free,
            ocram[1] - ocram[0] - ocram_free, ocram_free)
    if "__ocram_text_end" in symbols:
        used += (symbols["__ocram_text_end"] - symbols["__ocram_text_start"],)
    return used


def summary(used):
    """Return the one line report of ITCM, DTCM and OCRAM use, and a warning when a TCM is short."""
    lines = [("ITCM used %.1f KB, free %.1f KB; DTCM used %.1f KB, free %.1f KB; "
              "OCRAM used %.1f KB, free %.1f KB" + ", code %.1f KB" * (len(used) - 6))
             % tuple(n / 1024 for n in used)]
    if used[1] < MARGIN:
        lines.append("ITCM has under 8 KB free: remove lines from the second .itcm_text block in "
                     + LD_FILE)
    if used[3] < MARGIN:
        lines.append("DTCM has under 8 KB free: data, bss and stack must shrink")
    return lines


def check(rules, placements, flash, loaded=()):
    """Return a warning for each RAM claim met by a placement in flash, or by nothing.

    A claim on an object that is linked but empty on this target is not reported.
    """
    warnings = {}
    taken, shadowed = set(), set()
    for section, address, size, obj in placements:
        first = True
        for filepat, sections, region, excluded in rules:
            if not fnmatch.fnmatchcase(obj, filepat):
                continue
            if any(fnmatch.fnmatchcase(obj, e) for e in excluded):
                continue
            if not any(fnmatch.fnmatchcase(section, s) for s in sections):
                continue
            # The first rule that matches takes the section. The rest are only
            # noted, so that a rule which wins nothing can be reported below.
            if not first:
                shadowed.add(filepat)
                continue
            first = False
            taken.add(filepat)
            if not FLASH_REGION.match(region) and any(s <= address < e for s, e in flash):
                key = (obj, "." + section.lstrip(".").split(".")[0], region)
                warnings[key] = warnings.get(key, 0) + size
    lines = ["%s: %u bytes of %s are in flash although %s claims it for %s: "
             "an earlier rule takes it, narrow that rule"
             % (obj, total, kind, LD_FILE, region)
             for (obj, kind, region), total in warnings.items()]
    for filepat, region in dict.fromkeys((f, r) for f, _, r, _ in rules
                                         if not FLASH_REGION.match(r) and f not in taken):
        if any(fnmatch.fnmatchcase(obj, filepat) for obj in loaded):
            continue
        if filepat in shadowed:
            lines.append("%s: %s for %s claims nothing, an earlier rule takes it all: "
                         "delete the rule" % (LD_FILE, filepat, region))
        else:
            lines.append("%s: %s for %s matches no object: the rule's pattern no longer "
                         "matches, update it" % (LD_FILE, filepat, region))
    return lines


def self_check():
    linker = """
    .text : {
        EXCLUDE_FILE(*/lib/modules/usb/*) */lib/modules/*.o(.text .text*)
        *(EXCLUDE_FILE(*/fc/*.o) .text)
    } > FLASH
    .itcm_text : {
        *(.itcm_text)
        */lib/modules/usb/*.o(.text .text*)
        *libm.a:libm_a-sf_lrint.o(.text .text*)
        */fc/*.o(.text .text*)
        */common/*.o(.text .text*)
        */renamed/*.o(.text .text*)
        */drivers/empty.o(.text .text*)
    } > ITCM AT > FLASH
    """
    mapfile = """
Memory Configuration

Name             Origin             Length             Attributes
FLASH            0x60002000         0x007fb000         xr
ITCM             0x00000000         0x00040000         xrw
DTCM             0x20000000         0x00040000         rw
OCRAM2           0x20480000         0x0003e000         rw

Linker script and memory map

LOAD obj/main/T/drivers/empty.o
                0x00000000                __itcm_start = .
                0x0003b690                __itcm_end = .
                0x20029600                __bss_end = .
                0x2003e000                __stack_limit = (__stack_top - __stack_size)
                0x20481000                __ocram2_end = .

 .text.tu_fifo_count
                0x60002000       0x2c obj/main/T/./lib/modules/tinyusb/src/common/tusb_fifo.o
 .text.taskMain 0x60003000       0x40 obj/main/T/fc/tasks.o
 .text.constrainf
                0x00000100      0x2a0 obj/main/T/common/maths.o
 .text.dcd_int  0x60004000       0x20 obj/main/T/./lib/modules/usb/dcd.o
 .text.lrintf   0x00000400       0x98 /lib/libm.a(libm_a-sf_lrint.o)
"""
    rules = parse_ld(linker)
    assert ("*/fc/*.o", [".text", ".text*"], "ITCM", []) in rules, rules
    flash = parse_regions(mapfile, FLASH_REGION)
    assert flash == [(0x60002000, 0x607fd000)], flash
    placements = parse_map(mapfile)
    assert len(placements) == 5, placements
    warnings = check(rules, placements, flash, MAP_LOAD.findall(mapfile))
    # maths.o is claimed and in ITCM, and tusb_fifo.o matches the */common/*.o
    # claim but the flash rule for the third party modules comes first and wins,
    # so neither is reported, nor is the libm member claimed and in ITCM.
    # dcd.o is excluded from that flash rule, so its ITCM claim stands and it is
    # reported in flash, as is tasks.o, and nothing at all is left for the
    # renamed directory. empty.o is linked with no bytes, so its claim stands.
    narrow = (" are in flash although link/nxp_itcm256k_sections.ld claims it for ITCM: "
              "an earlier rule takes it, narrow that rule")
    assert warnings == ["obj/main/T/fc/tasks.o: 64 bytes of .text" + narrow,
                        "obj/main/T/./lib/modules/usb/dcd.o: 32 bytes of .text" + narrow,
                        "link/nxp_itcm256k_sections.ld: */renamed/*.o for ITCM matches no object: "
                        "the rule's pattern no longer matches, update it"], warnings
    used = usage(mapfile)
    assert used == (0x3b690, 0x40000 - 0x3b690, 0x40000 - 0x14a00, 0x14a00, 0x1000, 0x3d000), used
    assert summary(used) == ["ITCM used 237.6 KB, free 18.4 KB; "
                             "DTCM used 173.5 KB, free 82.5 KB; "
                             "OCRAM used 4.0 KB, free 244.0 KB"], summary(used)
    assert len(summary((0x3f000, 0x1000, 0, 0x10000, 0, 0))) == 2
    assert summary((0, 0, 0, 0x10000, 0, 0, 0x330))[0].endswith(", code 0.8 KB")
    print("check_placement: self check passed")


def main(argv):
    if argv[1:2] == ["--self-check"]:
        self_check()
        return 0
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 1
    try:
        with open(argv[1]) as handle:
            mapfile = handle.read()
        rules = []
        for path in argv[2:]:
            with open(path) as handle:
                rules += parse_ld(handle.read())
    except OSError as error:
        print("check_placement: %s" % error, file=sys.stderr)
        return 1
    flash = parse_regions(mapfile, FLASH_REGION)
    lines = summary(usage(mapfile))
    print("check_placement: " + lines[0])
    errors = sorted(check(rules, parse_map(mapfile), flash, MAP_LOAD.findall(mapfile))) + lines[1:]
    for error in errors:
        print("%s: error: %s" % (argv[1], error), file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
