[1mdiff --git a/src/main/io/rc_controls.c b/src/main/io/rc_controls.c[m
[1mindex fdde2cf..53464ef 100644[m
[1m--- a/src/main/io/rc_controls.c[m
[1m+++ b/src/main/io/rc_controls.c[m
[36m@@ -67,6 +67,7 @@[m [mstatic pidProfile_t *pidProfile;[m
 static bool isUsingSticksToArm = true;[m
 [m
 int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW[m
[32m+[m[32mint16_t rcCommandSmooth[4];     // Smoothed RcCommand[m
 [m
 uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e[m
 [m
[1mdiff --git a/src/main/io/rc_controls.h b/src/main/io/rc_controls.h[m
[1mindex eaec277..dd8afaf 100644[m
[1m--- a/src/main/io/rc_controls.h[m
[1m+++ b/src/main/io/rc_controls.h[m
[36m@@ -147,6 +147,7 @@[m [mtypedef struct controlRateConfig_s {[m
 } controlRateConfig_t;[m
 [m
 extern int16_t rcCommand[4];[m
[32m+[m[32mextern int16_t rcCommandSmooth[4];     // Smoothed RcCommand[m
 [m
 typedef struct rcControlsConfig_s {[m
     uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.[m
[1mdiff --git a/src/main/mw.c b/src/main/mw.c[m
[1mindex 125674c..5da79cf 100644[m
[1m--- a/src/main/mw.c[m
[1m+++ b/src/main/mw.c[m
[36m@@ -181,7 +181,7 @@[m [mvoid filterRc(void)[m
 [m
     if (isRXDataNew) {[m
         for (int channel=0; channel < 4; channel++) {[m
[31m-            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);[m
[32m+[m[32m            deltaRC[channel] = rcCommand[channel] - (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);[m
             lastCommand[channel] = rcCommand[channel];[m
         }[m
 [m
[36m@@ -194,7 +194,7 @@[m [mvoid filterRc(void)[m
     // Interpolate steps of rcCommand[m
     if (factor > 0) {[m
         for (int channel=0; channel < 4; channel++) {[m
[31m-            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;[m
[32m+[m[32m            rcCommandSmooth[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;[m
          }[m
     } else {[m
         factor = 0;[m
[36m@@ -649,8 +649,11 @@[m [mvoid subTaskMainSubprocesses(void) {[m
 [m
     const uint32_t startTime = micros();[m
 [m
[32m+[m[32m    filterRc();[m
[32m+[m
     if (masterConfig.rxConfig.rcSmoothing || flightModeFlags) {[m
[31m-        filterRc();[m
[32m+[m[32m        int axis;[m
[32m+[m[32m        for (axis = 0; axis <= 4; axis++) rcCommand[axis] = rcCommandSmooth[axis];[m
     }[m
 [m
     // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.[m
