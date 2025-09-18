###############################################################################
# Generic preprocessor macro expansion helpers
#
# Public interface (pp_*):
#   pp_def_value(<header.h>, EXPR)       -> expanded EXPR with whitespace removed
#   pp_def_value_str(<header.h>, EXPR)   -> expanded EXPR, quotes + whitespace removed
#
# Notes:
# - Uses $(CROSS_CC) -E with -imacros to include only the given header's macros.
# - Removes all spaces and tabs from the expansion to keep values filename-safe.
# - For string expressions, also strips the surrounding quotes.
###############################################################################

_pp_expand_raw = $(strip $(shell \
  printf '%s' "$2" | \
  $(CROSS_CC) $(CPPFLAGS) \
    $(addprefix -D,$(OPTIONS)) \
    $(addprefix -I,$(INCLUDE_DIRS)) \
    $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
    -E -P -xc -imacros "$1" - \
  | tr -d '\r\n' \
))


# concatenate adjacent string (C rules), then remote remaining quotes
# preprocessor alredy merged whitespace between tokens
_pp_quote :="
_pp_unquote    = $(subst $_pp_quote,,$(subst " ",,$1))

# Expanded value (remove all spaces)
pp_def_value      = $(call _pp_expand_raw,$1,$2)

# String values: collapse C string concatenation (" " -> ""), remove quotes and spaces
pp_def_value_str  = $(call _pp_unquote,$(call _pp_expand_raw,$1,$2))
