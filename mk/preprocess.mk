###############################################################################
# Generic preprocessor macro extraction helpers
#
# Public interface (pp_*):
#   pp_dump(<header.h>)                   -> macro dump with whitespace escaped
#   pp_def_value(<dump>, NAME)            -> macro value (whitespace restored)
#   pp_def_value_nq(<dump>, NAME)         -> macro value with outer quotes removed
#
# Typical usage:
#   PP_VERSION := $(call pp_dump,src/main/build/version.h)
#   YEAR       := $(call pp_def_value,$(PP_VERSION),FC_VERSION_YEAR)
#   SUFFIX     := $(call pp_def_value_nq,$(PP_VERSION),FC_VERSION_SUFFIX)
#
# Notes:
# - Whitespace is escaped to '|' in the dump and restored by accessors.
# - Only outer quotes are removed by _pp_unquote; interior quotes are preserved.
# - Uses $(CROSS_CC) at expansion time to match the project toolchain.
###############################################################################

# Literal '#'
_pp_hash := $(shell printf '#')

# Dump all macros visible when including header $1; escape whitespace as '|'
# Late binding: flags from the call site are used at expansion time.
pp_dump = $(strip $(shell \
  $(CROSS_CC) $(CPPFLAGS) \
    $(addprefix -D,$(OPTIONS)) \
    $(addprefix -I,$(INCLUDE_DIRS)) \
    $(addprefix -isystem,$(SYS_INCLUDE_DIRS)) \
    -E -dM -xc /dev/null \
    -include $1 \
  | sed 's/[ \t]/|/g' \
))

# Internal helpers
# Find full "#define NAME value" line (escaped as #define|NAME|value|with||multiple||spaces)
# $1 = precomputed dump, $2 = macro name
_pp_get_define =  $(strip $(filter $(_pp_hash)define|$2|%,$1))
# Extract RHS (still '|' escaped)
_pp_def_raw    = $(patsubst $(_pp_hash)define|$2|%,%,$(call _pp_get_define,$1,$2))

# Remove surrounding quotes (but not interior ones); $1 is '|' escaped
_pp_unquote    = $(patsubst "%",%,$1)

# Collapse escaped whitespace between adjacent quotes: '"|"' -> '""' and
# remove adjacent quote pairs inside the string: '""' -> '' (joins tokens like "A" "B" -> "AB")
_pp_strip_inner_qq = $(subst "",, $(subst "|","",$1))

# Public helpers: parse from cached dump and return unescaped values
pp_def_value      = $(subst |, ,$(call _pp_def_raw,$1,$2))

# Extract C string content: merges adjacent quoted tokens and strips outer quotes.
# Handles both '"A" "B"' and '"A""B"' forms; restores whitespace from '|'.
pp_def_value_str  = $(subst |, ,$(call _pp_unquote,$(call _pp_strip_inner_qq,$(call _pp_def_raw,$1,$2))))
