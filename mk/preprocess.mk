###############################################################################
# Generic preprocessor macro expansion helpers
#
# Public interface (pp_*):
#   pp_def_value(<header.h>, EXPR)       -> expanded EXPR
#   pp_def_value_str(<header.h>, EXPR)   -> expanded EXPR containing string, unquoting it
#
# Notes:
# - Uses $(CROSS_CC) -E with -imacros to include only the given header's macros.
# - Keeps only the BF_PP_VALUE line (MinGW-w64 headers add #pragma lines).
# - value_str: merge adjacent strings and remove quotes ("a""b" "B C" -> abB C)
###############################################################################

_pp_expand_raw = $(strip $(shell \
  printf 'BF_PP_VALUE %s\n' "$2" | \
  $(CROSS_CC) $(CPPFLAGS) \
    $(addprefix -D,$(OPTIONS)) \
    $(addprefix -I,$(INCLUDE_DIRS)) \
    $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
    -E -P -xc -imacros "$1" - \
  | sed -n 's/^BF_PP_VALUE //p' \
  | tr -d '\r\n' \
))

# Expand only if the macro NAME is defined in header $1; otherwise yield empty
# Use octal-escaped '#' so old make (3.81 on macOS) doesn't treat it as a comment.
_pp_expand_guarded_raw = $(strip $(shell \
  printf '\043 if defined(%s)\nBF_PP_VALUE %s\n\043 endif\n' "$2" "$2" | \
  $(CROSS_CC) $(CPPFLAGS) \
    $(addprefix -D,$(OPTIONS)) \
    $(addprefix -I,$(INCLUDE_DIRS)) \
    $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
    -E -P -xc -imacros "$1" - \
  | sed -n 's/^BF_PP_VALUE //p' \
  | tr -d '\r\n' \
))

# Concatenate adjacent strings (C rules) and remove quotes
# Preprocessor already merged whitespace between tokens
_pp_quote :="
_pp_unquote    = $(subst $(_pp_quote),,$(subst " ",,$1))

# Expanded value; empty if macro is not defined
pp_def_value      = $(call _pp_expand_guarded_raw,$1,$2)

# String values: collapse C string concatenation (" " -> ""), remove quotes
pp_def_value_str  = $(call _pp_unquote,$(call _pp_expand_guarded_raw,$1,$2))
