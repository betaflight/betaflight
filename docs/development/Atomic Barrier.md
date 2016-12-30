# Atomic Barrier implementation

```
static int markme_bar = 0;
static int markme = 0;

markme++;
// (1) markme is read into register, but not changed
markme_bar++;
// markme_bar is read from memory and incremented
ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
   ATOMIC_BARRIER(markme_bar);
// start of ATOMIC_BLOCK_NB scope:
//  markme_bar is stored into memory (it is input/output - "+m" output operand - of asm volatile)
//  BASEPRI is saved into temporary variable
//  BASEPRI_MAX is decremented to NVIC_PRIO_TIMER (if it is higher than NVIC_PRIO_TIMER or zero; lower number means higher priority on ARM)
   markme++;
// nothing happens, markme value is not needed yet
   markme_bar++;
// (2) markme_bar re-read from memory (ATOMIC_BARRIER marked it as modified - "+m" output operand of asm volatile)
//  and incremented

// end of ATOMIC_BLOCK_NB scope:
//  markme_bar is stored into memory (cleanup function from ATOMIC_BARRIER) / input "m" operand), but kept for later use in register
//    (actually markme_bar+1 is stored and pre-increment value kept in register)
// BASEPRI value is restored
};

markme++;
// register value read in (1) is incremented by 3
markme_bar++;
// register value read in (2) is incremented (actually +=2, because register contains pre-increment value)

// markme and markme_bar are stored into memory
```

# Atomic Barrier Warning


The ATOMIC_BLOCK/ATOMIC_BARRIER construction is dependent on gcc extensions. I relies on gcc cleanup function (`attribute ((cleanup))`) and assumes that cleanup handler is called, when leaving block, even when associated variable is eliminated.

There is (a bit paranoid) safeguard warning to make sure that generated assembly is hand-checked on new gcc version. It is assumed that only major gcc version versions need to be checked.

If GCC is upgraded and a warning appears when compiling then the generated asm source must be verified.

e.g.
```
%% serial_softserial.c
warning "Please verify that ATOMIC_BARRIER works as intended"
```

To perform the verification, proceed as per discusson on issue #167 which reads:

I hope it's enough to check that optimized-away variable still has cleanup code at end of scope.

```
static int markme=0;
markme++;
ATOMIC_BLOCK_NB(0xff) {
   ATOMIC_BARRIER(markme);
   markme++;
};
markme++;
```

pass `-save-temps=obj` (or `-save-temps=cwd`, but lots of files will end up in same directory as makefile) to gcc link step (LTO is in use), find resulting `*.ltrans*.ltrans.s` (grep for `markme`, on linux it ends up in `/tmp`) and check that generated assembly sequence is:

```
                 MSR basepri_max, r3
# (possibly markme address load)
                # barier (markme) start

# (increment markme, load and store to memory)
        ldr     r2, [r3]
        adds    r0, r2, #1
        str     r0, [r3]

                # barier(markme)  end
                MSR basepri, r3

# (markme value should be cached in register on next increment)
```

The # barrier(markme) must surround access code and must be inside MSR basepri instructions .

Similar approach is used for ATOMIC_BLOCK in avr libraries, so gcc should not break this behavior.

IMO attribute(cleanup) and asm volatile is defined in a way that should guarantee this.

attribute(cleanup) is probably safer way to implement atomic sections - another possibility is to explicitly place barriers in code, but that can (and will eventually) lead to missed barrier/basepri restore on same path creating very hard to find bug.

The 'MEMORY_BARRIER()' code can be omitted when 'ATOMIC_BLOCK' (with full memory barriers) is used, but it is better to explicitly state what memory is protected by barrier. gcc 5 can use this knowledge to greatly improve generated code.
