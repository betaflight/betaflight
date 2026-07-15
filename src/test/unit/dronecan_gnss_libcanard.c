// libcanard is external to src/main, so the test Makefile has no rule to
// compile it directly. Pull the plain-C translation unit in here (as C) so the
// decoder under test links against the real canardDecodeScalar/Encode helpers.
#include "canard.c"
