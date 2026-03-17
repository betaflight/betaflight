# PICO_trace
#
# Allow tracing into and out of functions without modifying the source code
#
# Enable by setting PICO_TRACE variable in make, e.g. by export PICO_TRACE=1; make ...
#
PICO_WRAPPED_FUNCTIONS = main
PICO_WRAPPED_FUNCTIONS += initPhase1 initPhase2 initPhase3 initEEPROM isEEPROMVersionValid resetEEPROM writeUnmodifiedConfigToEEPROM resetConfig pgResetAll
PICO_TRACE_LD_FLAGS += $(foreach fn, $(PICO_WRAPPED_FUNCTIONS), -Wl,--wrap=$(fn))
PICO_TRACE_SRC += PICO/pico_trace.c

DEVICE_FLAGS += -DPICO_TRACE
