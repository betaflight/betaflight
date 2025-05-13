#include "pico_trace.h"

// Wrap main to insert the initialisation code.
extern int main(int argc, char * argv[]);
extern int REAL_FUNC(main)(int argc, char * argv[]);
int WRAPPER_FUNC(main)(int argc, char * argv[])
{
    stdio_init_all();
    return REAL_FUNC(main)(argc, argv);
}

#define TRACEvoidvoid(x)                        \
    extern void x(void);\
    extern void REAL_FUNC(x)(void);         \
    void WRAPPER_FUNC(x)(void)\
    {\
    tprintf("*** enter " #x " ***");\
    REAL_FUNC(x)();\
    tprintf("*** exit  " #x " ***");\
    }

// remember to add to PICO_WRAPPED_FUNCTIONS in PICO_trace.mk
TRACEvoidvoid(init)
TRACEvoidvoid(initEEPROM)
TRACEvoidvoid(isEEPROMVersionValid)
