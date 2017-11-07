#include "build/atomic.h"

struct barrierTrace {
    int enter, leave;
};

int testAtomicBarrier_C(struct barrierTrace *b0, struct barrierTrace *b1, struct barrierTrace sample[][2])
{
    int sIdx = 0;

// replace barrier macros to track barrier invocation
// pass known struct as barrier variable, keep track inside it
#undef ATOMIC_BARRIER_ENTER
#undef ATOMIC_BARRIER_LEAVE
#define ATOMIC_BARRIER_ENTER(ptr, refStr) do {(ptr)->enter++; } while(0)
#define ATOMIC_BARRIER_LEAVE(ptr, refStr) do {(ptr)->leave++; } while(0)

    b0->enter = 0; b0->leave = 0;
    b1->enter = 0; b1->leave = 0;
    sample[sIdx][0]=*b0; sample[sIdx][1]=*b1; sIdx++;
    do {
        ATOMIC_BARRIER(*b0);
        ATOMIC_BARRIER(*b1);
        sample[sIdx][0]=*b0; sample[sIdx][1]=*b1; sIdx++;
        do {
            ATOMIC_BARRIER(*b0);
            sample[sIdx][0]=*b0; sample[sIdx][1]=*b1; sIdx++;
        } while(0);
        sample[sIdx][0]=*b0; sample[sIdx][1]=*b1; sIdx++;
    } while(0);
    sample[sIdx][0]=*b0; sample[sIdx][1]=*b1; sIdx++;
    return sIdx;

// ATOMIC_BARRIER is broken in rest of this file
#undef ATOMIC_BARRIER_ENTER
#undef ATOMIC_BARRIER_LEAVE
}
