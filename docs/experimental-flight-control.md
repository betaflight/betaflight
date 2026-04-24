# Experimental Flight-Control Overview

## Summary

This branch combines three related changes in the flight-control path:

1. A dedicated multirotor `BRAKING` mode that injects aggressive roll/pitch rate setpoints to kill horizontal motion without relying on GPS navigation.
2. A pre-differentiation D-term low-pass filter that reduces the amount of noise amplified by the derivative stage.
3. An adaptive D-term LPF1 controller that can move the cutoff inside a configured range based on measured D-term noise.

These changes share the same goal: reduce control-path inconsistencies, keep high-frequency noise under tighter control, and make short-horizon recovery or braking behavior more deliberate.

## BRAKING Mode

`BRAKING` is exposed as a normal AUX mode through `BOXBRAKING` and `BRAKING_MODE`.

Activation is limited to the multirotor path and requires:

- the craft to be armed
- ACC availability
- a non-wing target
- the `BOXBRAKING` switch to be active
- throttle to have been raised previously
- no active failsafe
- no active GPS Rescue mode

When active, the controller:

- estimates short-horizon horizontal motion from ACC and attitude
- keeps only a short-lived velocity state with deliberate leak
- generates roll/pitch rate setpoints directly
- leaves yaw and throttle on the normal Betaflight path

The main loop integration point is:

```text
subTaskRcCommand() -> brakingUpdate() -> pidController()
```

Two safety fixes were included before submission:

- `BRAKING_MODE` now has a generic `!failsafeIsActive()` guard, so non-GPS failsafe states cannot activate braking.
- the controller resets itself whenever the mode is not allowed, preventing stale braking state from leaking across mode changes.

## D-term Pre-differentiation LPF

`dterm_prediff_hz` adds an optional PT1 low-pass stage before the gyro delta is calculated for D-term.

The intent is straightforward:

- filter raw gyro data before differentiation
- avoid amplifying high-frequency noise too early
- allow the downstream D-term LPF stack to run with less unnecessary load

The setting is available through CLI and blackbox headers:

- `dterm_prediff_hz`

`0` disables the stage. Positive values configure the cutoff in hertz.

## Adaptive D-term LPF1 Controller

The adaptive controller is profile-scoped and controlled by these CLI fields:

- `adaptive_dterm_lpf`
- `adaptive_dterm_lpf_min_hz`
- `adaptive_dterm_lpf_max_hz`
- `adaptive_dterm_lpf_start_hz`
- `adaptive_dterm_lpf_update_ms`
- `adaptive_dterm_lpf_step_hz`
- `adaptive_dterm_lpf_learn_delay_s`

At runtime the controller:

- watches D-term first-difference energy
- updates the noise estimator once per PID loop
- freezes learning during disarm, learn delay, large stick input, protected flight modes, large gyro error, low throttle, or motor saturation
- adjusts only the D-term LPF1 cutoff, inside the configured range

Before submission, the disarm path was hardened so re-arm starts from a clean adaptive state:

- cutoff is restored to `adaptive_dterm_lpf_start_hz`
- stored noise energy is cleared
- previous D-term samples are cleared
- LPF coefficients are reapplied immediately

That prevents one flight from carrying its learned state into the next arming cycle.

## Observability

The branch also adds supporting observability hooks:

- `DEBUG_ADAPTIVE_DTERM_LPF`
- blackbox header fields for `dterm_prediff_hz`
- blackbox header fields for adaptive D-term LPF configuration
- telemetry / OSD mode visibility for `BRAKING_MODE`

## Validation

The branch was verified with:

```sh
make -j4 TARGET=STM32F405
```

Result:

- successful compile and link
- `betaflight_2026.6.0-alpha_STM32F405.hex` generated

## Tuning Notes

- `BRAKING` gains are intentionally conservative enough to make the control path testable before airframe-specific tuning.
- The adaptive D-term controller should be tuned with blackbox review, not by enabling it blindly on a noisy setup.
- `dterm_prediff_hz` is intended to be used together with a review of the downstream D-term LPF cutoffs, not as an isolated toggle.
