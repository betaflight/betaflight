# Dynamic Notch Park

With the advent of the RPM filter, the dynamic notch has been relegated to a role of 'cleaning up' residual noise left over after the RPM filter. The Dynamic Notch Park is meant to offer more flexibility for the function of the dynamic notch in its new role.

The Dynamic Notch Park feature "parks" the dynamic notch at an alternate location (from the peak frequency detected by the FFT) when the amplitude of the noise of the peak FFT bin does not surpass a threshold value. The purpose is to relocate the notch to a location that is more helpful or at least contributes less to filter latency when the dynamic notch would otherwise be having the least effect because the the gyro signal is relatively quiet and does not require the additional filtering.  Even when the dynamic notch is 'parked', the normal dynamic notch is maintained in the background so that it resume immediately when the park threshold is surpassed.

Instead of raw amplitude values, the Notch Park feature calculates the mean and standard deviations of the amplitude of all of the FFT bins, and threshold is based around multiples of standard deviations in relation to the mean.  This helps the park feature function consistently for both noisey and clean quads.

When the threshold is not passed, the notch can be parked at a high frequency (default) to minimize added latency, a specific frequency to target a static noise band outside of the range of the FFT, or when the RPM filter is enabled, to the first motor harmonic (based on the average frequency of all motors as reported by the RPM filter) not already being filtered by the RPM filter.

## Dynamic Notch Park settings

**dyn_notch_park_high_thresh**: The threshold for frequencies above *dyn_notch_park_high_t_hz*. The dynamic notch will be parked if the center frequency determined by the FFT is above *dyn_notch_park_high_t_hz* and the amplitude value of the FFT bin at the center frequency does not surpass this value. Threshold values should be chosen based on data from the FFT_PARK or FFT_THRESH debug modes.
Allowed range: 0 - 250
Default value: 0

**dyn_notch_park_low_thresh**: The threshold for frequencies below *dyn_notch_park_low_t_hz*. The dynamic notch will be parked if the center frequency determined by the FFT is below *dyn_notch_park_low_t_hz* and the amplitude value of the FFT bin at the center frequency does not surpass this value. Threshold values should be chosen based on data from the the FFT_PARK or FFT_THRESH debug modes.
Allowed range: 0 - 250
Default value: 0

**dyn_notch_park_high_t_hz**: Frequency above which the *dyn_notch_park_high_thresh* is used.
Allowed range: 60 - 1000
Default value: 60

**dyn_notch_park_low_t_hz**: Frequency below which the *dyn_notch_park_low_thresh* is used.
Allowed range: 60 - 1000
Default value: 60

**Note**: If both *dyn_notch_park_low_t_hz* and *dyn_notch_park_high_t_hz* are set, a threshold value is linearly interpolated from *dyn_notch_park_low_thresh* to *dyn_notch_park_high_thresh* for FFT frequencies from *dyn_notch_park_low_t_hz* to *dyn_notch_park_high_t_hz*.

**dyn_notch_park_hz**: Frequency that the dynamic notch is centered at when parked (only if *dyn_notch_park_harmonic* is not enabled and functioning).
Allowed range: 60 - 1000
Default value: 1000

**dyn_notch_park_harmonic**: Instead of *dyn_notch_park_hz*, parks the dynamic notch at the first harmonic frequency not being filtered by the RPM filter. Does not function if RPM filter is not enabled, or if the RPM gyro filter is set to use 3 harmonics.
Allowed values: OFF, 
Default value: OFF

## Dynamic Notch Park Debug Modes

**FFT_PARK**
[0]: (Debug axis) The center frequency chosen for the dynamic notch by the FFT.
[1]: (Debug axis) The actual frequency where the dynamic notch is centered, including when parked.
[2]: (Debug axis) The effective park threshold at the current FFT center frequency.
[3]: (Debug axis) The value being compared against the park threshold to determine if the Dynamic Notch is parked. 

**FFT_THRESH**
[0]: (Roll) Frequency where the Dynamic Notch is centered.
[1]: (Roll) The threshold value below witch the notch would be parked at this frequency.
[2]: (Pitch) Frequency where the Dynamic Notch is centered.
[3]: (Pitch) The threshold value below witch the notch would be parked at this frequency.