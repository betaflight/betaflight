# Atomic Barrier Warning

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

The # barrier(markme) must surround access code and must be inside MSR basepri instructions ..

Similar approach is used for ATOMIC_BLOCK in avr libraries, so gcc should not break this behavior.

IMO attribute(cleanup) and asm volatile is defined in a way that should guarantee this.

attribute(cleanup) is probably safer way to implement atomic sections - another possibility is to explicitly place barriers in code, but that can (and will eventually) lead to missed barrier/basepri restore on same path creating very hard to find bug.

The MEMORY_BARRIER() code can be omitted and use ATOMIC_BLOCK with full memory barriers, but IMO it is better to explicitly state what memory is protected by barrier and gcc can use this knowledge to greatly improve generated code in future.
