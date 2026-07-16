#!/usr/bin/env python3
"""
gen_cli_docs.py — Generate docs/CLI/parameters-reference.md from settings.c

Parses valueTable[], lookupTables[], and section comments from settings.c/h
to produce a complete, always-current CLI parameter reference in Markdown.

No external dependencies (Python 3.6+ stdlib only).

Usage (from repo root):
    python3 docs/gen_cli_docs.py
    python3 docs/gen_cli_docs.py --settings src/main/cli/settings.c
    python3 docs/gen_cli_docs.py --output docs/CLI/parameters-reference.md
"""

import re
import sys
import argparse
import subprocess
from pathlib import Path
from datetime import date
from collections import OrderedDict


# ---------------------------------------------------------------------------
# PG group → human-readable section name (derived, not hardcoded)
# ---------------------------------------------------------------------------

# Known tokens to preserve in uppercase when converting PG_xxx names.
# Words containing digits are kept as-is automatically (e.g. 3D, MAX7456).
_ACRONYMS = {
    'ADC', 'CAN', 'CRSF', 'DMA', 'DRONECAN', 'ESC', 'GPS', 'IMU', 'LED', 'LPF', 'MSP',
    'OSD', 'PID', 'PWM', 'RC', 'RCDEVICE', 'RPM', 'RTC', 'RX', 'SDCARD', 'SDIO',
    'SPI', 'TX', 'USB', 'VCD', 'VTX',
}


def pg_to_human(pg_name):
    """Derive a human-readable section name from a PG_xxx constant name.

    Examples:
        PG_GYRO_CONFIG           -> 'Gyro Config'
        PG_PID_PROFILE           -> 'PID Profile'
        PG_GPS_CONFIG            -> 'GPS Config'
        PG_CONTROLRATE_PROFILES  -> 'Controlrate Profiles'
        PG_MOTOR_3D_CONFIG       -> 'Motor 3D Config'
        PG_SDCARD_CONFIG         -> 'SDCARD Config'
    """
    name = pg_name[3:] if pg_name.startswith('PG_') else pg_name  # strip PG_
    parts = name.split('_')

    def _word(p):
        if p in _ACRONYMS:
            return p
        if any(c.isdigit() for c in p):  # e.g. 3D, MAX7456 — keep as-is
            return p
        return p.capitalize()

    return ' '.join(_word(p) for p in parts)


# ---------------------------------------------------------------------------
# Lookup table parsing
# ---------------------------------------------------------------------------

def strip_block_comments(text):
    return re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)


def parse_lookup_defs(text):
    """Returns dict: variable_name -> [string_values] from all const char* const arrays.

    Handles both unsized (name[]) and sized (name[COUNT]) array declarations.
    """
    tables = {}
    pattern = re.compile(
        r'(?:static\s+)?const\s+char\s*\*\s*const\s+(\w+)\s*\[[^\]]*\]\s*=\s*\{([^}]+)\}',
        re.DOTALL
    )
    for m in pattern.finditer(text):
        values = re.findall(r'"([^"]*)"', m.group(2))
        if values:
            tables[m.group(1)] = values
    return tables


def parse_param_names(text):
    """Returns dict: PARAM_NAME_XXX -> literal string, from #define PARAM_NAME_XXX "value"."""
    return dict(re.findall(r'#define\s+(PARAM_NAME_\w+)\s+"([^"]*)"', text))


def parse_enum_members(text, enum_name):
    """Returns ordered list of enum member names, stripping #ifdef guard lines."""
    m = re.search(
        r'typedef\s+enum\s*\{([^}]+)\}\s*' + enum_name + r'\s*;',
        text, re.DOTALL
    )
    if not m:
        return []
    members = []
    for line in m.group(1).splitlines():
        s = line.strip()
        if not s or s.startswith(('#', '//', '*')):
            continue
        em = re.match(r'([A-Z_][A-Z0-9_]+)', s)
        if em and em.group(1) != 'LOOKUP_TABLE_COUNT':
            members.append(em.group(1))
    return members


def parse_lookup_table_array(text):
    """Returns ordered list of variable names from lookupTables[] = { LOOKUP_TABLE_ENTRY(x)... }"""
    m = re.search(
        r'const lookupTableEntry_t lookupTables\[\]\s*=\s*\{(.+?)^\};',
        text, re.DOTALL | re.MULTILINE
    )
    if not m:
        return []
    entries = []
    for line in m.group(1).splitlines():
        s = line.strip()
        if s.startswith(('#', '//')):
            continue
        em = re.search(r'LOOKUP_TABLE_ENTRY\((\w+)\)', s)
        if em:
            entries.append(em.group(1))
    return entries


def build_table_map(h_text, c_text, lookup_defs):
    """Returns dict: TABLE_xxx -> [string_values]."""
    enum_members = parse_enum_members(h_text, 'lookupTableIndex_e')
    lookup_array = parse_lookup_table_array(c_text)
    result = {}
    for i, name in enumerate(enum_members):
        if i < len(lookup_array):
            var = lookup_array[i]
            result[name] = lookup_defs.get(var, [])
        else:
            result[name] = []
    return result


# ---------------------------------------------------------------------------
# valueTable[] parsing
# ---------------------------------------------------------------------------

def expand_valuetable_macros(block):
    """Expand '#define NAME(args) { ... }, \\ ...' entry-generator macros and their
    call-site invocations (e.g. GYRO_DEVICE_RECORDS(1, 0)) into literal entry text.

    Handles the "str1 STR(param) str2" adjacent-string-literal concatenation pattern
    used to build per-index parameter names (e.g. "gyro_" STR(N) "_bustype").
    """
    macro_re = re.compile(r'#define\s+(\w+)\(([^)]*)\)((?:[^\n]*\\\n)*[^\n]*)\n?')
    macros = {}

    def _capture(m):
        name = m.group(1)
        params = [p.strip() for p in m.group(2).split(',') if p.strip()]
        # Keep line breaks between sub-entries so the caller's line-based
        # entry parser sees one entry per line, same as hand-written entries.
        body = m.group(3).replace('\\\n', '\n')
        macros[name] = (params, body)
        return ''

    block = macro_re.sub(_capture, block)

    def _merge_string_literals(text):
        while True:
            new_text, n = re.subn(
                r'"([^"]*)"\s*STR\((\w+)\)\s*"([^"]*)"',
                lambda mm: '"' + mm.group(1) + mm.group(2) + mm.group(3) + '"',
                text
            )
            if n == 0:
                return text
            text = new_text

    for name, (params, body) in macros.items():
        inv_re = re.compile(re.escape(name) + r'\(([^)]*)\)')

        def _expand(m, params=params, body=body):
            args = [a.strip() for a in m.group(1).split(',')]
            text = body
            for p, a in zip(params, args):
                text = re.sub(r'\b' + re.escape(p) + r'\b', a, text)
            return _merge_string_literals(text)

        block = inv_re.sub(_expand, block)

    return block


def parse_value_table(c_text, param_names):
    """
    Returns list of entry dicts, tracking PG section comments and #ifdef conditions.
    Each dict: name, var_type, scope, mode, table_name, min, max, array_len, bitpos, pg, ifdef_conds
    """
    start = c_text.find('const clivalue_t valueTable[] = {')
    if start == -1:
        sys.exit("ERROR: cannot find 'const clivalue_t valueTable[]' in settings.c")

    end_marker = '\nconst uint16_t valueTableEntryCount'
    end = c_text.find(end_marker, start)
    block = c_text[start: end if end != -1 else len(c_text)]
    block = expand_valuetable_macros(block)

    entries = []
    current_pg = 'UNKNOWN'
    ifdef_stack = []
    pending = ''
    pending_pg = 'UNKNOWN'
    pending_conds = []

    entry_start_re = re.compile(r'\{\s*(?:"|PARAM_NAME_)')

    for raw_line in block.splitlines():
        line = raw_line.strip()

        # Track PG section comments
        pg_m = re.search(r'//\s*(PG_\w+)', line)
        if pg_m:
            current_pg = pg_m.group(1)
            continue

        # Strip trailing '//' line comments (e.g. "... }, // deactivate time in ms")
        line = re.sub(r'//.*$', '', line).rstrip()

        # Preprocessor directives
        if re.match(r'#ifdef\s+(\w+)', line):
            ifdef_stack.append(re.match(r'#ifdef\s+(\w+)', line).group(1))
            continue
        if re.match(r'#ifndef\s+(\w+)', line):
            ifdef_stack.append('!' + re.match(r'#ifndef\s+(\w+)', line).group(1))
            continue
        if re.match(r'#if\s+', line):
            ifdef_stack.append(line[3:].strip())
            continue
        if line.startswith('#endif'):
            if ifdef_stack:
                ifdef_stack.pop()
            continue
        if line.startswith('#else'):
            if ifdef_stack:
                top = ifdef_stack[-1]
                ifdef_stack[-1] = top[1:] if top.startswith('!') else ('!' + top)
            continue
        if re.match(r'#elif\s', line):
            if ifdef_stack:
                ifdef_stack.pop()
            ifdef_stack.append(line[5:].strip())
            continue

        # Accumulate entry lines
        if entry_start_re.match(line) and not pending:
            pending = line
            pending_pg = current_pg
            pending_conds = list(ifdef_stack)
        elif pending:
            pending += ' ' + line

        # Detect complete entry: has offsetof(/PG_ARRAY_ELEMENT_OFFSET( and ends with }
        if pending and ('offsetof(' in pending or 'PG_ARRAY_ELEMENT_OFFSET(' in pending):
            stripped = pending.rstrip().rstrip(',').rstrip()
            if stripped.endswith('}'):
                entry = _parse_entry(pending, pending_pg, pending_conds, param_names)
                if entry:
                    entries.append(entry)
                pending = ''
                pending_pg = 'UNKNOWN'
                pending_conds = []

    return entries


def _parse_entry(text, pg, ifdef_conds, param_names):
    """Parse a single valueTable entry string into a dict. Returns None on failure."""
    m = re.match(r'\{\s*(?:"(\w+)"|(PARAM_NAME_\w+))\s*,\s*(.+)', text, re.DOTALL)
    if not m:
        return None
    literal, macro, rest = m.group(1), m.group(2), m.group(3)
    name = literal if literal is not None else param_names.get(macro, macro)

    config_idx = rest.find('.config')
    if config_idx == -1:
        return None

    flags_str = rest[:config_idx].rstrip(',').strip()
    config_part = rest[config_idx:]

    # Override pg from inline PG_xxx token if present
    pg_m = re.search(r'\b(PG_\w+)\b', config_part)
    if pg_m:
        pg = pg_m.group(1)

    var_type = _decode_type(flags_str)
    scope = _decode_scope(flags_str)
    mode, table_name, min_val, max_val, array_len, bitpos = _decode_config(flags_str, config_part, var_type)

    return {
        'name':        name,
        'var_type':    var_type,
        'scope':       scope,
        'mode':        mode,
        'table_name':  table_name,
        'min':         min_val,
        'max':         max_val,
        'array_len':   array_len,
        'bitpos':      bitpos,
        'pg':          pg,
        'ifdef_conds': ifdef_conds,
    }


def _decode_type(flags):
    for token, label in [
        ('VAR_UINT32', 'uint32'), ('VAR_INT32', 'int32'),
        ('VAR_INT16', 'int16'), ('VAR_UINT16', 'uint16'),
        ('VAR_INT8', 'int8'), ('VAR_UINT8', 'uint8'),
    ]:
        if token in flags:
            return label
    return 'uint8'


def _decode_scope(flags):
    if 'PROFILE_RATE_VALUE' in flags:
        return 'rate'
    if 'PROFILE_BATTERY_VALUE' in flags:
        return 'battery'
    if 'PROFILE_VALUE' in flags:
        return 'profile'
    if 'HARDWARE_VALUE' in flags:
        return 'hardware'
    return 'master'


def _decode_config(flags_str, config_part, var_type):
    """Returns (mode, table_name, min, max, array_len, bitpos)."""
    mode = 'direct'
    table_name = min_val = max_val = array_len = bitpos = None

    if 'MODE_LOOKUP' in flags_str:
        mode = 'lookup'
        m = re.search(r'\.config\.lookup\s*=\s*\{\s*(\w+)\s*\}', config_part)
        if m:
            table_name = m.group(1)

    elif 'MODE_ARRAY' in flags_str:
        mode = 'array'
        m = re.search(r'\.config\.array\.length\s*=\s*(\w+)', config_part)
        if m:
            array_len = m.group(1)

    elif 'MODE_BITSET' in flags_str:
        mode = 'bitset'
        m = re.search(r'\.config\.bitpos\s*=\s*(\w+)', config_part)
        if m:
            bitpos = m.group(1)

    elif 'MODE_STRING' in flags_str:
        mode = 'string'
        m = re.search(r'\.config\.string\s*=\s*\{\s*([^,}]+),\s*([^,}]+)', config_part)
        if m:
            min_val = m.group(1).strip()
            max_val = m.group(2).strip()

    else:
        m = re.search(r'\.config\.minmaxUnsigned\s*=\s*\{\s*([^,}]+),\s*([^}]+)\}', config_part)
        if not m:
            m = re.search(r'\.config\.minmax\s*=\s*\{\s*([^,}]+),\s*([^}]+)\}', config_part)
        if m:
            min_val = m.group(1).strip()
            max_val = m.group(2).strip()
        elif var_type == 'uint32':
            m = re.search(r'\.config\.u32Max\s*=\s*(.+?)\s*,\s*PG_\w+', config_part)
            if m:
                min_val = '0'
                max_val = m.group(1).strip()
        elif var_type == 'int32':
            m = re.search(r'\.config\.d32Max\s*=\s*(.+?)\s*,\s*PG_\w+', config_part)
            if m:
                expr = m.group(1).strip()
                min_val = f'-{expr}'
                max_val = expr

    return mode, table_name, min_val, max_val, array_len, bitpos


# ---------------------------------------------------------------------------
# Markdown generation
# ---------------------------------------------------------------------------

def _anchor(text):
    """Convert section heading to GitHub-flavored markdown anchor."""
    return re.sub(r'[^a-z0-9-]', '', text.lower().replace(' ', '-'))


def _format_range(entry, table_map):
    mode = entry['mode']
    if mode == 'lookup':
        tn = entry['table_name'] or ''
        values = table_map.get(tn, [])
        if values:
            return ', '.join(f'`{v}`' for v in values)
        return f'*{tn}*'
    if mode == 'direct':
        mn, mx = entry['min'], entry['max']
        if mn is not None and mx is not None:
            return f'`{mn}` – `{mx}`'
        return ''
    if mode == 'array':
        return f'array\\[{entry["array_len"]}\\]'
    if mode == 'bitset':
        return 'bitflag'
    if mode == 'string':
        mn, mx = entry['min'], entry['max']
        if mn is not None and mx is not None:
            return f'string\\[{mn}-{mx}\\]'
        return 'string'
    return ''


def _extract_use_conditions(cond):
    """Extract all USE_* symbols from a condition, preserving negation."""
    out = []
    # Match negated or plain USE_* tokens, including defined() forms
    for m in re.finditer(r'(!)?\s*(?:defined\()?\s*(USE_[A-Z0-9_]+)\s*\)?', cond):
        neg, sym = m.groups()
        out.append(f'!{sym}' if neg else sym)
    return out


def _format_requires(ifdef_conds):
    """Return USE_xxx requirements, including negated/defined forms."""
    simple = []
    for c in ifdef_conds:
        simple.extend(_extract_use_conditions(c))
        # Also catch standalone simple forms
        if re.match(r'!?USE_\w+$', c):
            simple.append(c)
    if not simple:
        return ''
    return ', '.join(f'`{c}`' for c in dict.fromkeys(simple))  # deduplicated, ordered


def generate_markdown(entries, table_map, settings_c_path, git_hash=None,
                      fw_version=None, msp_version=None):
    today = date.today().isoformat()
    ref = git_hash or 'unknown'
    fw_str  = f' | Firmware: `{fw_version}`'  if fw_version  else ''
    msp_str = f' | MSP: `{msp_version}`'      if msp_version else ''

    # Group by PG, preserving first-seen order
    sections = OrderedDict()
    for e in entries:
        sections.setdefault(e['pg'], []).append(e)

    lines = [
        '# CLI Parameters Reference',
        '',
        '> **Auto-generated** — do not edit manually.',
        f'> Source: `{settings_c_path}` | Generated: {today} | Commit: `{ref}`{fw_str}{msp_str}',
        '',
        '---',
        '',
        '## Table of Contents',
        '',
    ]

    for pg, pg_entries in sections.items():
        human = pg_to_human(pg)
        lines.append(f'- [{human}](#{_anchor(human)})')

    lines += ['', '---', '']

    for pg, pg_entries in sections.items():
        human = pg_to_human(pg)
        lines.append(f'## {human}')
        lines.append('')

        has_requires = any(_format_requires(e['ifdef_conds']) for e in pg_entries)

        if has_requires:
            lines.append('| Parameter | Type | Scope | Range / Values | Requires |')
            lines.append('|-----------|------|-------|----------------|----------|')
        else:
            lines.append('| Parameter | Type | Scope | Range / Values |')
            lines.append('|-----------|------|-------|----------------|')

        for e in pg_entries:
            name  = f'`{e["name"]}`'
            vtype = e['var_type']
            scope = e['scope']
            rng   = _format_range(e, table_map)
            if has_requires:
                req = _format_requires(e['ifdef_conds'])
                lines.append(f'| {name} | {vtype} | {scope} | {rng} | {req} |')
            else:
                lines.append(f'| {name} | {vtype} | {scope} | {rng} |')

        lines.append('')

    lines += [
        '---',
        f'*Generated by `docs/gen_cli_docs.py` from `{settings_c_path}`*',
        '',
    ]
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _git_hash(path):
    try:
        r = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=path, capture_output=True, text=True, timeout=5
        )
        return r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        return None


def _firmware_version(repo_root: Path):
    """Return 'MAJOR.MINOR.PATCH' from src/main/build/version.h, or None."""
    vh = repo_root / 'src' / 'main' / 'build' / 'version.h'
    if not vh.exists():
        return None
    text = vh.read_text()
    def _def(name):
        m = re.search(rf'#define\s+{name}\s+(\d+)', text)
        return m.group(1) if m else None
    major = _def('FC_VERSION_MAJOR') or _def('FC_VERSION_YEAR')
    minor = _def('FC_VERSION_MINOR') or _def('FC_VERSION_MONTH')
    patch = _def('FC_VERSION_PATCH_LEVEL')
    if major and minor and patch:
        return f'{major}.{minor}.{patch}'
    return None


def _msp_version(repo_root: Path):
    """Return 'PROTO.MAJOR.MINOR' MSP version from msp_protocol.h, or None."""
    mh = repo_root / 'src' / 'main' / 'msp' / 'msp_protocol.h'
    if not mh.exists():
        mh = repo_root / 'src' / 'main' / 'interface' / 'msp_protocol.h'
    if not mh.exists():
        return None
    text = mh.read_text()
    def _def(name):
        m = re.search(rf'#define\s+{name}\s+(\d+)', text)
        return m.group(1) if m else None
    proto = _def('MSP_PROTOCOL_VERSION')
    major = _def('API_VERSION_MAJOR')
    minor = _def('API_VERSION_MINOR')
    if major and minor:
        proto = proto or '0'
        return f'{proto}.{major}.{minor}'
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        '--settings', default='src/main/cli/settings.c',
        help='Path to settings.c (default: src/main/cli/settings.c)'
    )
    parser.add_argument(
        '--header', default='src/main/cli/settings.h',
        help='Path to settings.h (default: src/main/cli/settings.h)'
    )
    parser.add_argument(
        '--param-names', default='src/main/fc/parameter_names.h',
        help='Path to parameter_names.h (default: src/main/fc/parameter_names.h)'
    )
    parser.add_argument(
        '--output', default='docs/CLI/parameters-reference.md',
        help='Output path (default: docs/CLI/parameters-reference.md)'
    )
    args = parser.parse_args()

    c_path  = Path(args.settings)
    h_path  = Path(args.header)
    pn_path = Path(args.param_names)
    out_path = Path(args.output)

    for p in (c_path, h_path):
        if not p.exists():
            sys.exit(f"ERROR: {p} not found. Run from repo root.")

    c_text = strip_block_comments(c_path.read_text())
    h_text = strip_block_comments(h_path.read_text())

    param_names = {}
    if pn_path.exists():
        param_names = parse_param_names(pn_path.read_text())
    else:
        print(f"  WARNING: {pn_path} not found — PARAM_NAME_* macros will show as-is")

    # Also parse extern lookup arrays from sibling source files
    extra_sources = [
        c_path.parent.parent / 'sensors' / 'current.c',
        c_path.parent.parent / 'sensors' / 'voltage.c',
        c_path.parent.parent / 'build'   / 'debug.c',
    ]

    print(f"Parsing lookup table definitions...")
    lookup_defs = parse_lookup_defs(c_text)
    for extra in extra_sources:
        if extra.exists():
            lookup_defs.update(parse_lookup_defs(strip_block_comments(extra.read_text())))
        else:
            print(f"  WARNING: {extra} not found — some lookup values may be unresolved")
    print(f"  {len(lookup_defs)} arrays found")

    print("Building TABLE_xxx -> values map...")
    table_map = build_table_map(h_text, c_text, lookup_defs)
    print(f"  {len(table_map)} TABLE_xxx entries mapped")

    print("Parsing valueTable[]...")
    entries = parse_value_table(c_text, param_names)
    print(f"  {len(entries)} parameters found")

    if not entries:
        sys.exit("ERROR: no entries parsed — check settings.c path and format")

    # Resolve repo root: walk up from settings.c until we find version.h
    repo_root = c_path.resolve().parent
    while repo_root != repo_root.parent:
        if (repo_root / 'src' / 'main' / 'build' / 'version.h').exists():
            break
        repo_root = repo_root.parent

    git_hash   = _git_hash(repo_root)
    fw_version = _firmware_version(repo_root)
    msp_version = _msp_version(repo_root)

    if fw_version:
        print(f"Firmware version: {fw_version}")
    if msp_version:
        print(f"MSP version:      {msp_version}")

    print(f"Generating {out_path}...")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    md = generate_markdown(entries, table_map, str(c_path), git_hash,
                           fw_version=fw_version, msp_version=msp_version)
    out_path.write_text(md)

    n_sections = len({e['pg'] for e in entries})
    print(f"Done: {len(entries)} parameters across {n_sections} sections -> {out_path}")


if __name__ == '__main__':
    main()
