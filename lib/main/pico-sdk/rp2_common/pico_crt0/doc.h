/**
 * \defgroup pico_crt0 pico_crt0
 * \brief Provides the default linker scripts and the program entry/exit point
 */

// PICO_CONFIG: PICO_EMBED_XIP_SETUP, Embed custom XIP setup (boot2) in an RP2350 binary, type=bool, default=0, group=pico_base
// unused but keeps tooling happy
#define PICO_EMBED_XIP_SETUP 0