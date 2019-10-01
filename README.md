Provides the following changes from master:

- 3 and 4 point feedforward smoothing options via LMA filtering (thanks iCr4sh)
- anti-glitch code for feedforward and boost (improvement for evaluation)
- excludes yaw from ff_max_rate_limit (bugfix)

The glitch reduction code significantly improves the smoothness of feedforward trace when duplicate packets are received.  It is always active when ff_interpolate_sp is active, regardless of averaging settings.  To minimise delay, the code only acts when two RC steps in a row are identical.  Early packets, or more than 2 missing steps in a row, will still cause glitches in ff and boost.  The code is so effective that for good Rx's with reliable packet data, eg some Futaba and Spektrum systems, it is possible to use no feed forward averaging at all, for super crisp responses.  To turn feedforward averaging off, CLI `set ff_interpolate_sp = ON` - check your feed forward traces after trying this.

3 point averaging eliminates the 20hz periodicity seen in TBS CRSF Rx systems with OpenTx 2.2 and 2.3, making for a much smoother FF trace.  It is best used with locked 150hz mode.  3 point smoothing in 50hz mode will be smoother, but slower, than the default 2 point smoothing.  CLI `set ff_interpolate_sp = AVERAGE3`

4 point averaging completely eliminates the cyclical 4-step periodicity in feed forward seen with R9 Rx when in 150hz mode with OpenTx 2.2 and 2.3.  R9's cannot be locked in 150hz mode, so 4 point averaging will give smooth but very delayed feed forward singals when the R9 goes to 50hz mode.  This option is primarily intended when R9's are for smooth HD flying.  

4 point averaging may also be useful for HD quads to reduce large FF steps.  CLI `set ff_interpolate_sp = AVERAGE4`

To revert to default feedforward averaging, CLI `set ff_interpolate_sp = AVERAGE2`

`ff_max_rate_limit` attenuates feedforward to stop rolls and flips from overshooting as the sticks approach full rates.  It should have only been applied to pitch and roll, but accidentally also included yaw.  This code corrects that bug.  It allows full rate yaw spins to get the required full FF drive as the sticks approach maximum.
