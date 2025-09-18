###############################################################################
# Generic preprocessor macro expansion helpers
#
# Public interface (pp_*):
#   pp_def_value(<header.h>, EXPR)       -> expanded EXPR
#   pp_def_value_str(<header.h>, EXPR)   -> expanded EXPR containing string, unquoting it
#
# Notes:
# - Uses $(CROSS_CC) -E with -imacros to include only the given header's macros.
# - value_str: merge adjacent strings and remove quotes ("a""b" "B C" -> abB C)
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


# Concatenate adjacent strings (C rules) and remove quotes
# Preprocessor already merges whitespace between tokens
_pp_quote :="
_pp_unquote    = $(subst $(_pp_quote),,$(subst " ",,$1))

# Expanded value (remove all spaces)
pp_def_value      = $(call _pp_expand_raw,$1,$2)

# String values: collapse C string concatenation (" " -> ""), remove quotes and spaces
pp_def_value_str  = $(call _pp_unquote,$(call _pp_expand_raw,$1,$2))
