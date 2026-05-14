# check_pg_ids.awk
# Usage: awk -f check_pg_ids.awk path/to/pg_ids.h
# Prints duplicates and exits non-zero if any found

function hexval(s,   i, c, d, lo, digits) {
    lo = "0123456789abcdef"
    digits = tolower(s)
    v = 0
    for (i = 3; i <= length(digits); i++) {
        c = substr(digits, i, 1)
        d = index(lo, c) - 1
        if (d < 0) return ""
        v = v * 16 + d
    }
    return v
}

function tonum(s,   v) {
    if (s ~ /^0x[0-9a-fA-F]+$/) {
        return hexval(s)
    } else if (s ~ /^[0-9]+$/) {
        return s + 0
    }
    return ""
}

/^[ \t]*#define[ \t]+PG_[A-Z0-9_]+[ \t]+[^ \/\t]+/ {
    line = NR
    line_text = $0
    sub(/\/\/.*$/, "", line_text)
    # split fields after removing comments
    n = split(line_text, fields, /[ \t]+/)
    name = fields[2]
    val = fields[3]
    v = tonum(val)
    if (v == "") next
    if (v in valmap) {
        valmap_dup[v] = valmap_dup[v] ? valmap_dup[v] ", " name : valmap[v] ", " name
        dup = 1
    } else {
        valmap[v] = name
    }
    if (name in namemap) {
        namemap[name] = namemap[name] ", " line
        dup = 1
    } else {
        namemap[name] = line
    }
}

END {
    if (dup) {
        for (k in valmap_dup) {
            print "Duplicate numeric value " k " ->" valmap_dup[k]
        }
        for (n in namemap) {
            if (index(namemap[n], ",") > 0) {
                print "Duplicate define name " n " on lines " namemap[n]
            }
        }
        exit 1
    }
}
