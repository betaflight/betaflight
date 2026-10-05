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

"""Build the i.MXRT pad tables from the vendor IOMUXC header.

The vendor header holds one macro per pad and signal, each a five value tuple:

    #define IOMUXC_GPIO_AD_B0_00_GPIO1_IO00 0x401F80BCU, 0x5U, 0, 0, 0x401F82ACU
                                            mux reg     mode  in daisy config reg

The header cannot be compiled on a host because it pulls in fsl_common.h, which
is not part of the checked out submodule, so the macros are read as text.

Two headers come out of it.

The family header holds the pad registers of every GPIO capable pad, indexed by
port and pin, which is what a plain input or output pin needs, and then one
table per peripheral family saying which pad can carry which signal, which is
what a board author needs to move a peripheral to another pad without copying
register addresses. Only the families in KEPT_FAMILIES are written out.

The pad name header turns every one of those pads into a Betaflight pin name,
so a board file can use the pad name the schematic and the datasheet use.

Usage:
    iomux_def_gen.py <fsl_iomuxc.h> <family header> [pad name header]
"""

import re
import sys

PORT_PINS = 32

# What each family calls the GPIO function of a pad, and which Betaflight port
# that is.
#
# On 10xx the pad mux names the GPIO block straight out, GPIO1..GPIO5, which
# become ports A..E. GPIO6..GPIO9 are the CM7 fast alias view of GPIO1..GPIO4
# and GPIO10 only reaches the pads of the stacked flash die, so none of them
# adds a pad and none is a port here.
#
# On 11xx the pad mux calls the GPIO function GPIO_MUX1..GPIO_MUX6, which
# become ports A..F, and every one of those pads appears again as GPIO7..GPIO12,
# the CM7 fast alias view, left out for the same reason. The always on pads are
# the exception: they have no GPIO_MUX name and are only ever GPIO13, so that
# block is port G. Seven ports is also as many as an ioTag_t can hold.
GPIO_PAD_10XX = re.compile(
    r"^IOMUXC_(?P<pad>[A-Z0-9_]+?)_GPIO(?P<port>[1-5])_IO(?P<pin>[0-9]{2})$"
)
GPIO_PAD_11XX = re.compile(
    r"^IOMUXC_(?P<pad>[A-Z0-9_]+?)_GPIO(?:_MUX(?P<port>[1-6])|(?P<snvs>13))"
    r"_IO(?P<pin>[0-9]{2})$"
)
# On i.MX95 the pad and the signal are split by a double underscore and the GPIO
# function is GPIO1..GPIO5 bit n, which become ports A..E.
GPIO_PAD_IMX95 = re.compile(
    r"^IOMUXC_(?P<pad>[A-Z0-9_]+?)__GPIO(?P<port>[1-5])_IO_BIT(?P<pin>[0-9]+)$"
)

# Every pad and signal tuple, whatever the signal is. The values are kept as
# text so the generated macro reads exactly like the vendor one.
VALUE = r"(?:0x[0-9A-Fa-f]+U?|[0-9]+U?)"
TUPLE = re.compile(
    r"^#define\s+IOMUXC_(?P<prefix>PAD_)?(?P<name>[A-Za-z0-9_]+)\s+"
    r"(?P<mux>%s)\s*,\s*(?P<mode>%s)\s*,\s*"
    r"(?P<input>%s)\s*,\s*(?P<daisy>%s)\s*,\s*"
    r"(?P<config>%s)\s*$" % (VALUE, VALUE, VALUE, VALUE, VALUE)
)

# A FlexPWM pad is one channel of one submodule of one instance, and a FlexIO
# pad is one pin of one instance. A driver has to know those numbers to reach
# the right registers, and the muxing does not carry them, so they are taken
# out of the vendor signal name. The two families spell both names differently.
FLEXPWM_10XX = re.compile(
    r"^FLEXPWM(?P<instance>[0-9]+)_PWM(?P<channel>[ABX])(?P<submodule>[0-9]{2})$"
)
FLEXPWM_11XX = re.compile(
    r"^FLEXPWM(?P<instance>[0-9]+)_PWM(?P<submodule>[0-9]+)_(?P<channel>[ABX])$"
)
FLEXIO_10XX = re.compile(r"^FLEXIO(?P<instance>[0-9]+)_FLEXIO(?P<pin>[0-9]{2})$")
FLEXIO_11XX = re.compile(r"^FLEXIO(?P<instance>[0-9]+)_D(?P<pin>[0-9]{2})$")
FLEXIO_IMX95 = re.compile(r"^FLEXIO(?P<instance>[0-9]+)_FLEXIO_BIT(?P<pin>[0-9]+)$")

# The peripheral families this platform has a driver for. Every other family the
# chip offers (ENET, SEMC, CSI, SAI, QTimer and the rest) would be table rows no
# code ever reads, so they are left out. Adding one is a word in this tuple and
# a re-run of this script.
KEPT_FAMILIES = ("LPUART", "LPSPI", "LPI2C", "USDHC", "FLEXPWM", "FLEXIO",
                 "FLEXCAN", "TPM")

# i.MX95 calls its FlexCAN signals CAN1..CAN5.
FAMILY_ALIASES = {"CAN": "FLEXCAN"}

# FlexPWM and FlexIO get a table of their own shape instead of the plain one:
# their drivers start from a pad and need the submodule, channel or pin number
# that goes with it, which an instance and a role cannot carry.
STRUCTURED_FAMILIES = ("FLEXPWM", "FLEXIO")

# What the vendor calls each job a wire can do, per family, and the role name
# iomux_signal_nxp.h gives it. The spellings differ between chip families,
# so this is where the divergences are folded together: TX and TXD are both
# the transmit role, SDI and SIN both the input, SDO and SOUT both the output,
# and the 10xx LPSPI2 output on GPIO_SD_B1_08 is spelled SD0 with a zero.
#
# Every spelling a kept family uses has to be in here. One that is not stops
# the run rather than dropping the row, because a dropped row is a pad a board
# can no longer name and nothing else would say so.
SIGNAL_ROLES = {
    "LPUART": {"TX": "TX", "TXD": "TX", "RX": "RX", "RXD": "RX",
               "CTS_B": "CTS", "RTS_B": "RTS", "DCD_B": "DCD", "DSR_B": "DSR",
               "DTR_B": "DTR", "RIN_B": "RIN"},
    "LPSPI": {"SCK": "SCK", "SDI": "SDI", "SIN": "SDI",
              "SDO": "SDO", "SOUT": "SDO", "SD0": "SDO",
              "PCS0": "PCS0", "PCS1": "PCS1", "PCS2": "PCS2", "PCS3": "PCS3"},
    "LPI2C": {"SCL": "SCL", "SDA": "SDA", "SCLS": "SCLS", "SDAS": "SDAS",
              "HREQ": "HREQ"},
    "FLEXCAN": {"TX": "TX", "RX": "RX"},
    "TPM": {"CH0": "CH0", "CH1": "CH1", "CH2": "CH2", "CH3": "CH3",
            "EXTCLK": "EXTCLK"},
    "USDHC": {"CLK": "CLK", "CMD": "CMD", "WP": "WP", "CD_B": "CD",
              "VSELECT": "VSELECT", "RESET_B": "RESET", "STROBE": "STROBE",
              "DATA0": "DATA0", "DATA1": "DATA1", "DATA2": "DATA2",
              "DATA3": "DATA3", "DATA4": "DATA4", "DATA5": "DATA5",
              "DATA6": "DATA6", "DATA7": "DATA7"},
}

FAMILIES = {
    "10xx": {
        "name": "10xx",
        "gpioPad": GPIO_PAD_10XX,
        "portLabels": ["GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5"],
        "baseComment": [
            "// Base address of the IOMUXC block that holds each port's pad",
            "// registers. The GPIO5 pads live in the always on SNVS domain and",
            "// have their own block, so the base is per port.",
        ],
        "flexpwm": FLEXPWM_10XX,
        "flexio": FLEXIO_10XX,
        "padGroups": False,
        "define": "IMXRT10XX",
        "title": "i.MXRT 10xx",
    },
    "11xx": {
        "name": "11xx",
        "gpioPad": GPIO_PAD_11XX,
        "portLabels": [
            "GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5", "GPIO6", "GPIO13",
        ],
        "baseComment": [
            "// Base address of the IOMUXC block that holds each port's pad",
            "// registers. The GPIO6 pads sit in the low power domain block and",
            "// the GPIO13 pads in the always on SNVS block, so the base is per",
            "// port.",
        ],
        "flexpwm": FLEXPWM_11XX,
        "flexio": FLEXIO_11XX,
        "padGroups": True,
        "define": "IMXRT11XX",
        "title": "i.MXRT 11xx",
    },
    "imx95": {
        "name": "imx95",
        "gpioPad": GPIO_PAD_IMX95,
        "portLabels": ["GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5"],
        "baseComment": [
            "// Base address of the IOMUXC block that holds each port's pad",
            "// registers. One block holds every pad on this chip.",
        ],
        "flexpwm": None,
        "flexio": FLEXIO_IMX95,
        "padGroups": False,
        "define": "IMX95",
        "title": "i.MX95",
    },
}


def familyGuard(chip):
    """The lines that keep a generated header with the family it describes.

    The pad and daisy chain registers are at different addresses on the 10xx and
    the 11xx parts, and a driver includes one of these headers by name rather
    than choosing between them, so without this a build for a family that has no
    header of its own would mux every pad through another family's registers and
    say nothing.
    """
    return [
        "// The pad registers differ per family.",
        "#if !defined(%s)" % chip["define"],
        '#error "%s pad tables in a build for another family"' % chip["title"],
        "#endif",
    ]


def chipFamily(entries):
    """Which family a header describes, from how it names a pad's GPIO mux."""
    for entry in entries:
        if GPIO_PAD_IMX95.match("IOMUXC_" + entry["name"]):
            return "imx95"
        if GPIO_PAD_11XX.match("IOMUXC_" + entry["name"]):
            return "11xx"
    return "10xx"

LICENCE = """/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
"""


def number(text):
    """A vendor tuple value, which may carry a U suffix."""
    return int(text.rstrip("Uu"), 0)


def gpioPad(entry, family):
    """(pad name, port, pin) if this macro is a pad's GPIO function."""
    match = family["gpioPad"].match("IOMUXC_" + entry["name"])
    if not match:
        return None
    # The always on block has no GPIO_MUX name, so it is the last port.
    port = int(match.group("port")) if match.group("port") else len(family["portLabels"])
    return match.group("pad"), port, int(match.group("pin"))


def parse(entries, family):
    """Return {(port, pin): (pad name, mux address, mux mode, config address)}."""
    pads = {}
    for entry in entries:
        found = gpioPad(entry, family)
        if not found:
            continue
        pad, port, pin = found
        if pin >= PORT_PINS:
            continue
        item = (pad, number(entry["mux"]), number(entry["mode"]),
                number(entry["config"]))
        if (port, pin) in pads:
            raise SystemExit(
                "two pads claim port %d pin %02d: %s and %s"
                % (port, pin, pads[(port, pin)][0], item[0])
            )
        pads[(port, pin)] = item
    if not pads:
        raise SystemExit("no GPIO pad macros found")
    return pads


def portBases(pads, family):
    """One IOMUXC block owns each port's pads. Find it, per port."""
    bases = []
    for port in range(1, len(family["portLabels"]) + 1):
        found = set()
        for (padPort, _pin), entry in pads.items():
            if padPort == port:
                # A block is 4KB aligned; the pad registers sit inside it.
                found.add(entry[1] & ~0xFFF)
                found.add(entry[3] & ~0xFFF)
        if len(found) != 1:
            raise SystemExit(
                "%s pads span more than one IOMUXC block: %s"
                % (family["portLabels"][port - 1],
                   sorted(hex(base) for base in found))
            )
        bases.append(found.pop())
    return bases


def parseTuples(path):
    """Return every pad and signal macro, in the order the header lists them."""
    entries = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = TUPLE.match(line.strip())
            if match:
                entries.append(match.groupdict())
    if not entries:
        raise SystemExit("no pad macros found in %s" % path)
    return entries


def sharedPrefix(names):
    """The leading underscore separated words that all of these names share."""
    if len(names) == 1:
        return names[0]
    words = [name.split("_") for name in names]
    shared = []
    for index in range(min(len(word) for word in words)):
        if len({word[index] for word in words}) != 1:
            break
        shared.append(words[0][index])
    return "_".join(shared)


def padOf(entries):
    """Return {macro name: pad name}.

    One pad has one pad control register, so that address says which macros
    belong to the same pad. The pad name is what those macro names share; a pad
    with a single macro and no signal is named by that macro alone.
    """
    groups = {}
    order = []
    for entry in entries:
        key = int(entry["config"].rstrip("Uu"), 0)
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(entry["name"])

    pads = {}
    for key in order:
        names = groups[key]
        # A double underscore, where the vendor uses one, ends the pad name.
        pad = names[0].split("__")[0] if "__" in names[0] else sharedPrefix(names)
        for name in names:
            pads[name] = pad
    return pads


def padPins(entries, family):
    """Return {pad name: (port, pin)} for the pads that are also a GPIO pin.

    A pad with no GPIO function on a port this build can name cannot be named
    in a Betaflight target, so it gets no entry and stays out of the lookup
    tables.
    """
    pins = {}
    for entry in entries:
        found = gpioPad(entry, family)
        if found and found[2] < PORT_PINS:
            pins[found[0]] = (found[1], found[2])
    return pins


def signalOf(name, pad):
    """The signal part of a macro name, or None for a macro that is pad only."""
    return name[len(pad):].lstrip("_") or None


def familyOf(signal):
    """The peripheral family a signal belongs to, so LPUART3_TX is LPUART.

    The instance number is part of the family word in the vendor names, so it
    is stripped here. That keeps one table per peripheral rather than one per
    instance, which is what a driver wants to search.
    """
    family = re.sub(r"[0-9]+$", "", signal.split("_")[0])
    return FAMILY_ALIASES.get(family, family)


def instanceRoleOf(family, signal):
    """The instance number and role name of a signal, so LPUART3_TX is (3, TX).

    An instance the name does not carry, or a spelling SIGNAL_ROLES has never
    seen, stops the run. Skipping the row instead would take a pad away from
    every board silently, which is the one failure this table cannot report.
    """
    head = signal.split("_")[0]
    instance = head[len(head.rstrip("0123456789")):]
    role = SIGNAL_ROLES[family].get(signal[len(head) + 1:])
    if not instance.isdigit() or role is None:
        raise SystemExit(
            "%s: cannot read the signal name %s. Add its suffix to "
            "SIGNAL_ROLES and, if the role is new, to nxpSignalRole_e in "
            "iomux_signal_nxp.h." % (family, signal)
        )
    return int(instance), role


def tableName(family):
    return "nxp%s%sPins" % (family[0], family[1:].lower())


def padComment(port, pin):
    return "// P%s%d" % (chr(ord("A") + port - 1), pin)


def muxTuple(entry):
    """The five vendor values of one pad and signal pair, written out.

    The vendor header defines the same five values under a macro name, but that
    header cannot be included here, so copying the macros out would mean a
    second copy of a file the compiler can already find. The values go in the
    row and the macro name goes in the row comment, which is what a reader
    greps for to check a row against the datasheet.
    """
    return "{ %s, %s, %s, %s, %s }" % (entry["mux"], entry["mode"],
                                       entry["input"], entry["daisy"],
                                       entry["config"])


def emitFlexpwm(rows, chip):
    """The FlexPWM pads, with the output each pad drives spelled out."""
    if not rows:
        return []

    out = [""]
    out.append("/*")
    out.append(" * The FlexPWM pads, with the instance, submodule and channel each one")
    out.append(" * drives. A driver writes the pulse width into the submodule that owns the")
    out.append(" * pad, so the muxing alone is not enough to put a signal on a pin.")
    out.append(" */")
    out.append("static const imxrtFlexpwmPin_t imxrtFlexpwmPads[] = {")
    for (port, pin), signal, entry in rows:
        match = chip["flexpwm"].match(signal)
        if not match:
            raise SystemExit("cannot read the FlexPWM signal name %s" % signal)
        out.append("    %s IOMUXC_%s%s" % (padComment(port, pin), entry["prefix"] or "", entry["name"]))
        out.append(
            "    { %d, %2d, %s, %s, IMXRT_FLEXPWM_CHANNEL_%s, %s },"
            % (port - 1, pin, match.group("instance"), int(match.group("submodule")),
               match.group("channel"), muxTuple(entry))
        )
    out.append("};")
    return out


def emitFlexio(rows, chip):
    """The FlexIO pads, with the pin number each pad is."""
    if not rows:
        return []

    out = [""]
    out.append("/*")
    out.append(" * The FlexIO pads, with the instance and the FlexIO pin number each one")
    out.append(" * is. A shifter or timer names the pin it works on, so a driver needs that")
    out.append(" * number as well as the muxing. It is not a shifter number: any shifter can")
    out.append(" * drive any pin.")
    out.append(" */")
    out.append("static const nxpFlexioPin_t nxpFlexioPads[] = {")
    for (port, pin), signal, entry in rows:
        match = chip["flexio"].match(signal)
        if not match:
            raise SystemExit("cannot read the FlexIO signal name %s" % signal)
        out.append("    %s IOMUXC_%s%s" % (padComment(port, pin), entry["prefix"] or "", entry["name"]))
        out.append(
            "    { %d, %2d, %s, %2d, %s },"
            % (port - 1, pin, match.group("instance"), int(match.group("pin")),
               muxTuple(entry))
        )
    out.append("};")
    return out


def pinName(port, pin):
    return "P%s%d" % (chr(ord("A") + port - 1), pin)


def emitPadNames(pads, chip, source):
    """One Betaflight pin name per pad, spelled the way the vendor names it."""
    out = [LICENCE.rstrip("\n")]
    out.append("")
    out.append("// Generated by src/platform/NXP/iomux_def_gen.py from %s, do not edit." % source)
    out.append("")
    out.append("#pragma once")
    out.append("")
    out.extend(familyGuard(chip))
    out.append("")
    out.append("// Vendor pad names as aliases of the Pxx tags, so a board can write")
    out.append("// #define SDIO_CK_PIN GPIO_SD_B0_01; the CLI still prints Pxx.")
    out.append("")
    for (port, pin) in sorted(pads):
        pad = pads[(port, pin)][0]
        name = pinName(port, pin)
        out.append("#define DEFIO_TAG__%-20s DEFIO_TAG__%s" % (pad, name))
        out.append("#define DEFIO_TAG_E__%-18s DEFIO_TAG_E__%s" % (pad, name))
        out.append("#define DEFIO_REC__%-20s DEFIO_REC__%s" % (pad, name))
    out.append("")
    out.append("// Must match the bonded pads in the GPIO table.")
    out.append("#define NXP_IOMUX_PAD_NAME_COUNT %d" % len(pads))
    out.append("")
    return "\n".join(out)


def padGroupOf(pad):
    """The group of pads a pad belongs to, from its name.

    Pads are named per group and numbered inside it, so the name minus the
    number is the group. The always on pads are named one by one instead, but
    they are one block with one register layout, so they are one group.
    """
    name = re.sub(r"_DIG$", "", pad)
    name = re.sub(r"_[0-9]+$", "", name)
    if name.startswith("GPIO_SNVS") or name in ("WAKEUP", "PMIC_ON_REQ",
                                                "PMIC_STBY_REQ"):
        return "GPIO_SNVS"
    return name


def padGroups(pads, family):
    """The pad groups this family has, or none when it needs no group at all."""
    if not family["padGroups"]:
        return []
    return sorted({padGroupOf(entry[0]) for entry in pads.values()})


def emitPadGroupEnum(groups):
    """The groups a pad control encoder has to tell apart."""
    if not groups:
        return []

    out = [""]
    out.append("/*")
    out.append(" * The groups of pads on this family.")
    out.append(" *")
    out.append(" * The pad control register does not hold the same fields in every group")
    out.append(" * on this family. The always on pads have no slew rate and no drive")
    out.append(" * strength field and put open drain in bit 6, the low power domain pads")
    out.append(" * put open drain in bit 5, and the main block pads put it in bit 4. Inside")
    out.append(" * the main block the vendor header gives bit 1 and bits 3:2 two meanings,")
    out.append(" * drive strength with a pull enable and select, or a pull drive with a")
    out.append(" * pull select, and which one a pad has is a property of its group. So a")
    out.append(" * pad control value cannot be built from the wanted configuration alone,")
    out.append(" * and the group comes with each pad in the table below.")
    out.append(" */")
    out.append("typedef enum {")
    out.append("    IMXRT_PAD_GROUP_NONE = 0,")
    for group in groups:
        out.append("    IMXRT_PAD_GROUP_%s," % group)
    out.append("} imxrtPadGroup_e;")
    return out


def emitPadGrid(pads, bases, family):
    """Every GPIO capable pad's registers, indexed by port and pin."""
    labels = family["portLabels"]
    withGroup = family["padGroups"]

    out = [""]
    out.append("// Pad registers for every GPIO capable pad, indexed by port and pin.")
    out.append("// Offsets are from the port's base above. A pin with no pad has a")
    out.append("// zero padCtlRegOffset; no real pad control register sits at offset 0.")
    out.append(
        "static const nxpGpioPad_t "
        "nxpGpioPads[NXP_IOMUX_PORT_COUNT][NXP_IOMUX_PORT_PINS] = {"
    )
    for port in range(1, len(labels) + 1):
        out.append("    { // %s" % labels[port - 1])
        for pin in range(PORT_PINS):
            entry = pads.get((port, pin))
            if entry is None:
                out.append("        { 0, 0, 0, 0 }, // IO%02d not bonded to a pad" % pin)
                continue
            name, mux, mode, config = entry
            base = bases[port - 1]
            # A family whose pad control register has one layout everywhere
            # needs no group, so the field is written as a plain zero.
            group = ("IMXRT_PAD_GROUP_%s" % padGroupOf(name)) if withGroup else "0"
            out.append(
                "        { 0x%03x, 0x%03x, %d, %s }, // IO%02d %s"
                % (mux - base, config - base, mode, group, pin, name)
            )
        out.append("    },")
    out.append("};")
    return out


def emitFamily(entries, pads, bases, chip, source):
    """The whole per family header: pad grid, signal numbers and the tables."""
    labels = chip["portLabels"]
    padNames = padOf(entries)
    pins = padPins(entries, chip)

    # A signal only reaches the lookup tables when its pad is a GPIO pin, since
    # that is the only way a target can name the pad. Plain GPIO is left out:
    # the pad grid above already covers it.
    tables = {}
    for entry in entries:
        pad = padNames[entry["name"]]
        signal = signalOf(entry["name"], pad)
        if not signal or pad not in pins:
            continue
        family = familyOf(signal)
        if family not in KEPT_FAMILIES:
            continue
        tables.setdefault(family, []).append((pins[pad], signal, entry))

    plain = sorted(family for family in tables if family not in STRUCTURED_FAMILIES)

    out = [LICENCE.rstrip("\n")]
    out.append("")
    out.append("// Generated by src/platform/NXP/iomux_def_gen.py from %s, do not edit." % source)
    out.append("")
    out.append("#pragma once")
    out.append("")
    out.extend(familyGuard(chip))
    out.append("")
    out.append('#include "iomux_signal_nxp.h"')
    out.append("")
    out.append("#define NXP_IOMUX_PORT_COUNT %d" % len(labels))
    out.append("#define NXP_IOMUX_PORT_PINS    %d" % PORT_PINS)
    out.append("")
    out.extend(chip["baseComment"])
    out.append(
        "static const uint32_t nxpIomuxPortBase[NXP_IOMUX_PORT_COUNT] = {"
    )
    # The addresses, not the vendor symbols IOMUXC_BASE, IOMUXC_LPSR_BASE and
    # IOMUXC_SNVS_BASE that carry them. This table is included by every driver
    # that muxes a pin, and their host tests stand in for the vendor CMSIS
    # headers, so naming the symbols would put the same addresses back in eight
    # test headers instead of one generated one.
    for label, base in zip(labels, bases):
        out.append("    0x%08xu, // %s" % (base, label))
    out.append("};")
    out.extend(emitPadGroupEnum(padGroups(pads, chip)))
    out.extend(emitPadGrid(pads, bases, chip))

    out.append("")
    out.append("/*")
    out.append(" * Which pad can carry which signal, one table per peripheral family. Each")
    out.append(" * table is static, so a build that never names a family pays nothing for")
    out.append(" * it. Only pads that are also a GPIO pin are listed, because a target names")
    out.append(" * a pad by its pin.")
    out.append(" *")
    out.append(" * A row names the peripheral instance and the role the wire plays, so a")
    out.append(" * driver asks for LPUART3 transmit. The role names are in")
    out.append(" * iomux_signal_nxp.h and are the same on every chip family, whatever the")
    out.append(" * vendor header of the day spells them.")
    out.append(" *")
    out.append(" * Above each row is the name the vendor header gives the same five values,")
    out.append(" * so a row can be checked against fsl_iomuxc.h by grepping that name.")
    out.append(" */")
    for family in plain:
        out.append("")
        out.append("static const nxpSignalPin_t %s[] = {" % tableName(family))
        for (port, pin), signal, entry in tables[family]:
            instance, role = instanceRoleOf(family, signal)
            out.append("    %s IOMUXC_%s%s" % (padComment(port, pin), entry["prefix"] or "", entry["name"]))
            out.append(
                "    { %d, %2d, %d, NXP_ROLE_%s_%s, %s },"
                % (port - 1, pin, instance, family, role, muxTuple(entry))
            )
        out.append("};")
    out.extend(emitFlexpwm(tables.get("FLEXPWM", []), chip))
    out.extend(emitFlexio(tables.get("FLEXIO", []), chip))

    out.append("")
    out.append("// Every signal keyed table above, for a test that wants to walk all of")
    out.append("// them. This is a macro, so naming it is what pulls the tables into a build.")
    out.append("#define NXP_IOMUX_SIGNAL_TABLES(X) \\")
    for index, family in enumerate(plain):
        tail = "" if index == len(plain) - 1 else " \\"
        out.append('    X("%s", %s)%s' % (family, tableName(family), tail))
    out.append("")

    counts = {family: len(rows) for family, rows in tables.items()}
    return "\n".join(out), counts


def main(argv):
    if len(argv) not in (3, 4):
        raise SystemExit(
            "usage: %s <fsl_iomuxc.h> <family header> [pad name header]" % argv[0]
        )

    source = argv[1].split("/")[-1]

    entries = parseTuples(argv[1])
    chip = FAMILIES[chipFamily(entries)]
    pads = parse(entries, chip)
    bases = portBases(pads, chip)

    text, counts = emitFamily(entries, pads, bases, chip, source)
    with open(argv[2], "w", encoding="utf-8") as handle:
        handle.write(text)
    print("%d GPIO pads written to %s" % (len(pads), argv[2]))
    for family in sorted(counts):
        print("    %-10s %3d" % (family, counts[family]))

    if len(argv) == 4:
        with open(argv[3], "w", encoding="utf-8") as handle:
            handle.write(emitPadNames(pads, chip, source))
        print("%d pad names written to %s" % (len(pads), argv[3]))

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
