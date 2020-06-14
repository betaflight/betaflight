# Integrated Yaw

## Overview

Integrated Yaw is a feature which corrects a fundamental issue with quad control: while the pitch and roll axis are controlled by the thrust differentials the props generate yaw is different. Yaw gyro changes happen primarily due to inertia when the RPM of the motors change. A constant acceleration of yaw requires a constant rate of increase/decrease in the RPM of the yaw motor pairs.

This means that on Yaw I has the effect of P and P the effect of D. FF is a control element that adds impetus based on the acceleration in setpoint rate.

This results in several issues on yaw. Typically yaw will react very fast initially, then fall back and catch up again over time.

Integrated Yaw fixes this by integrating the output of the yaw pid before applying them to the mixer. This normalizes the way the pids work. You can now tune as any other axis. It requires use of absolute control since no I is needed with Integrated Yaw. 


## Setup

From Betaflight 4.1.0 / Betaflight configurator 10.6.0 on Integrated Yaw can be set up in configurator, in the 'PID Tuning' tab.

As a fallback, these are the CLI commands to enable it:

```
set use_integrated_yaw=on
set iterm_rotation=off
set abs_control_gain=10
set p_yaw=30
set d_yaw=20
set i_yaw=0
set f_yaw=60
```

The yaw pids serve as starting point and might need adjustment. Make sure I is low or zero since significant I leads to strong oscillations. Pitch and roll might require slight retuning - primarily reducing I a bit since absolute control acts similar to a second I term.

**To be noted:**

If `i_yaw` is not set to `0`, then the following additional command is needed:

```
set iterm_relax=RPY
```
