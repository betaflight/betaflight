# Vendored Boost.Preprocessor (subset)

This directory contains a minimal subset of Boost.Preprocessor headers required by the macro-based filter framework in `src/main/common`.

Layout
- Headers live under `lib/boost/preprocessor/...` and are included as `<boost/preprocessor/...>`.
- Only the headers used transitively by `filter_magic_h.h` and `filter_generic.h` are present.

Refreshing the subset
- If you modify macros or add new Boost.PP usages, regenerate the needed list:
  - Preprocess a probe that includes the macro headers and record includes with `-H`.
  - Copy any newly referenced headers from your system Boost into `lib/boost/preprocessor/`.

Example (clang):
- Create `.pp_probe.c` with:
  - `#include "src/main/common/filter_magic_h.h"`
  - `#include "src/main/common/filter_generic.h"`
- Run: `clang -E -H -I lib -I src/main -I . .pp_probe.c >/dev/null 2> .pp_include.log`
- From `.pp_include.log`, keep only `lib/boost/preprocessor/...` files. Remove others to keep the subset small.

Notes
- The build’s include paths add `$(ROOT)/lib`, so these headers take precedence over any system Boost install.
- This vendoring is for POC convenience and deterministic builds; for upstream, consider depending on a packaged Boost.PP instead.

