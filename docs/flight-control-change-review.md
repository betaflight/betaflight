# Flight-Control Change Review

## Purpose

This document records the key behavior changes in this branch with focused before/after code excerpts and the reasoning behind each change.

It is intentionally limited to the highest-impact control-path updates:

- `BRAKING_MODE` failsafe gating
- adaptive D-term LPF disarm reset behavior
- D-term pre-differentiation LPF insertion
- FPV angle mix and feedforward ordering

## 1. BRAKING_MODE Failsafe Gating

### Before

`BRAKING_MODE` only excluded a GPS Rescue-specific failsafe path. Other failsafe procedures could still satisfy the braking activation chain.

```c
if (ARMING_FLAG(ARMED)
    && !FLIGHT_MODE(GPS_RESCUE_MODE)
    && !(failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE)
    && !isFixedWing()
    && IS_RC_MODE_ACTIVE(BOXBRAKING)
    && sensors(SENSOR_ACC)
    && wasThrottleRaised()) {
```

### After

The activation path now uses a generic failsafe guard:

```c
if (ARMING_FLAG(ARMED)
    && !failsafeIsActive()
    && !FLIGHT_MODE(GPS_RESCUE_MODE)
    && !IS_RC_MODE_ACTIVE(BOXGPSRESCUE)
    && !isFixedWing()
    && IS_RC_MODE_ACTIVE(BOXBRAKING)
    && sensors(SENSOR_ACC)
    && wasThrottleRaised()) {
```

### Why This Changed

`BRAKING_MODE` is an aggressive horizontal deceleration controller. Letting it activate during a generic failsafe is unsafe because the craft is no longer in a normal pilot-commanded state.

The old guard was too narrow:

- it only special-cased GPS Rescue failsafe
- it left non-GPS failsafe procedures exposed

The new guard closes that gap by making braking unavailable whenever any failsafe procedure is active.

### Effect

- `BRAKING_MODE` cannot engage during non-GPS failsafe
- GPS Rescue still retains priority over braking
- the activation logic now matches the intended safety boundary

## 2. Adaptive D-term LPF Disarm Reset

### Before

When disarmed, the adaptive controller only marked the disarmed freeze reason and cleared the arm timestamp:

```c
if (!ARMING_FLAG(ARMED)) {
    adFreezeReason |= ADAPTIVE_FREEZE_DISARMED;
    adArmedAtUs = 0;
}
```

### After

The disarm path now restores a clean initial state:

```c
if (!ARMING_FLAG(ARMED)) {
    adFreezeReason |= ADAPTIVE_FREEZE_DISARMED;
    adArmedAtUs = 0;
    adCurrentCutoff = adStartHz;
    adNoiseEnergy = 0.0f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        adPrevDterm[i] = 0.0f;
        adPrevDtermValid[i] = false;
    }
    applyNewCutoff(adStartHz);
}
```

### Why This Changed

The adaptive controller learns from in-flight D-term noise history. Without a reset, the next arming cycle could inherit:

- the learned cutoff from the previous flight
- the stored noise estimate
- stale previous-sample history

That behavior is undesirable because each new arm should start from the configured baseline, not from the previous pack's learned state.

### Effect

- re-arm always starts from `adaptive_dterm_lpf_start_hz`
- previous-flight noise history is cleared
- filter coefficients are immediately updated to match the reset cutoff
- repeated short flights behave deterministically

## 3. D-term Pre-differentiation LPF

### Before

The D-term path differentiated the post-filtered gyro signal directly:

```c
gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
```

### After

An optional PT1 stage can now run before the D-term delta is taken:

```c
if (pidRuntime.dtermPreDiffEnabled) {
    gyroRateDterm[axis] = pt1FilterApply(&pidRuntime.dtermPreDiffLpf[axis], gyroRateDterm[axis]);
}

if (hasDtermNotch) {
    gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
}
if (hasDtermLpf) {
    gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
}
if (hasDtermLpf2) {
    gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
}
```

### Why This Changed

Differentiation amplifies high-frequency noise. Adding an optional low-pass stage before the derivative:

- reduces the amount of noise that reaches the derivative step
- lowers the burden on later D-term filtering
- gives more room to tune total D-term latency versus noise suppression

This change also avoids unnecessary null-filter calls by caching whether each downstream stage is actually active.

### Effect

- a new CLI-exposed `dterm_prediff_hz` control becomes available
- D-term filtering can be tuned with a cleaner upstream signal
- hot-path filter dispatch is slightly tighter

## 4. FPV Angle Mix and Feedforward Ordering

### Before

Feedforward was calculated before FPV angle mixing modified the final raw setpoint:

```c
rawSetpoint[axis] = constrainf(angleRate, ...);
calculateFeedforward(&pidRuntime, axis);

if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
    scaleRawSetpointToFpvCamAngle();
}
```

### After

The final setpoint is now mixed first, then feedforward is derived from the mixed result:

```c
rawSetpoint[axis] = constrainf(angleRate, ...);

if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
    scaleRawSetpointToFpvCamAngle();
}

for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
    calculateFeedforward(&pidRuntime, axis);
}
```

### Why This Changed

Feedforward should be based on the same setpoint that the PID loop ultimately tracks. Previously:

- `rawSetpoint` was still in its pre-mix state when feedforward was calculated
- the control path later altered that setpoint through FPV camera angle mixing

That meant feedforward could be reacting to a setpoint that no longer matched the command being passed forward.

### Effect

- feedforward now follows the final mixed setpoint
- the RC command chain is more internally consistent
- roll/yaw mixed-command behavior is easier to reason about

## Validation

This branch was validated with:

```sh
make -j4 TARGET=STM32F405
```

Result:

- compile and link succeeded
- `betaflight_2026.6.0-alpha_STM32F405.hex` was generated

## Residual Risks

- `BRAKING_MODE` still needs airframe-specific tuning; the current gains are safe starting values, not universal final values.
- The adaptive D-term controller should be reviewed with blackbox logs before use on noisy setups.
- The pre-differentiation LPF adds another tuning dimension and should be evaluated together with the rest of the D-term filter stack.
