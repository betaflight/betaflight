# Experimental Flight-Control Changes

This branch collects a set of flight-control experiments and supporting fixes that were developed together:

- a multirotor-only `BRAKING` AUX mode
- a pre-differentiation D-term low-pass stage (`dterm_prediff_hz`)
- an adaptive D-term LPF1 cutoff controller (`adaptive_dterm_lpf*`)
- related control-path, telemetry, CLI, blackbox, and debug plumbing

The detailed behavior and configuration notes live in [experimental-flight-control.md](experimental-flight-control.md).

## Build Verification

The current branch was built successfully with:

```sh
make -j4 TARGET=STM32F405
```

## Scope Notes

- `BRAKING` is only active for multirotors with ACC enabled.
- `BRAKING` is blocked during failsafe and GPS Rescue.
- The adaptive D-term controller only learns when a compatible static PT1 D-term LPF1 configuration is active.
- Tuning changes in this branch should be verified with blackbox logs before flight use.
