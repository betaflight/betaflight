// libcanard is external to src/main, so the test Makefile has no rule to
// compile it directly. Pull the plain-C translation unit in here (as C) so the
// decoder under test links against the real canardDecodeScalar/Encode helpers.
#if !__has_include("canard.c")
#error "libcanard missing; run: git submodule update --init lib/modules/dronecan/libcanard"
#endif
#include "canard.c"
