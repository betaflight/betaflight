# Tiny whoop builds

Betaflight's stock configuration targets a 5" freestyle quad. Several of the
defaults say so in the source - `flight/pid.c` notes that the feedforward yaw
hold time of 100ms "is OK for a 5in; smaller values decay faster, eg for smaller
props". A 65-85mm ducted whoop is a different aircraft: props with an order of
magnitude less rotational inertia, spinning two to three times faster, on a
board that is usually an F411 with no GPS, no compass and no spare flash.

`USE_TINYWHOOP` is an opt-in build option that adapts the firmware to that
craft. It does three things: it removes the subsystems a whoop has no hardware
for, it ships defaults tuned for a small fast prop instead of a 5" one, and it
adds an in-goggle menu for the handful of settings that get changed between
packs.

Nothing here changes a build that does not set the option. A stock build of the
same target before and after this feature is byte-for-byte identical, and a
whoop build produces an ordinary Betaflight configuration: same parameter names,
same MSP, same Configurator, and every value can be changed back afterwards.

## Building

```sh
make CONFIG=CRAZYBEEF4SX1280 TINYWHOOP=yes
```

`TINYWHOOP=yes` is a convenience for `OPTIONS="USE_TINYWHOOP"`, so a board
config or a cloud build can enable it the same way any other option is enabled.

Measured on `CRAZYBEEF4SX1280` (STM32F411, 480KB usable flash):

| | stock | `TINYWHOOP=yes` | saved |
|---|---|---|---|
| flash | 395 990 B (80.6%) | 377 325 B (76.8%) | 18.7 KB |
| RAM | 82 384 B | 78 388 B | 4.0 KB |

The saving is larger on targets whose board config does not already exclude the
navigation drivers - the same cut on the SITL build removes 96 KB.

### Race variant

```sh
make CONFIG=CRAZYBEEF4SX1280 WHOOP_RACE=yes
```

`WHOOP_RACE=yes` implies `TINYWHOOP=yes` (setting `TINYWHOOP=no` at the same
time is a build error, not a silent override) and layers a second, smaller set
of changes for whoops raced around a short indoor track rather than flown
freestyle: snappier rates, a shorter feedforward hold time, and - on a board
whose config actually defines `USE_BARO` - baro and vario are dropped as well,
since a race whoop flown wide open on a fixed line has no use for altitude
hold. IR lap transponder support (`USE_TRANSPONDER` - Arcitimer, ImmersionRC
ILap, ERLT) is deliberately **not** part of the base feature cut for exactly
this reason: it is standard equipment at indoor whoop tracks. On a board that
does not define `USE_BARO` at all (most whoop AIO boards), `WHOOP_RACE=yes` is
identical in size to `TINYWHOOP=yes` and only the tuning differs.

### Overriding a single number

Every constant in `tinywhoop_profile.c` follows the codebase's existing
`#ifndef X / #define X value / #endif` convention for a build-time default
(the same pattern as `DEFAULT_PID_PROCESS_DENOM` in `flight/pid.c`), so any one
of them can be overridden without editing the file - from a board's
`config.h`, or directly on the command line:

```sh
make CONFIG=CRAZYBEEF4SX1280 TINYWHOOP=yes OPTIONS="TINYWHOOP_RATE_ROLL_PITCH=70"
```

The full list of names is in `tinywhoop_profile.c`.

## What the build drops

Dropped because a whoop has no hardware for it, so the code is dead weight and,
for the ones with a scheduler task, loop time the PID controller could have had:

- GPS and everything that needs it: GPS rescue, lap timer, plus codes, position
  hold, flight plan, optical flow
- compass, rangefinder
- fixed wing support and launch control
- legacy telemetry protocols: FrSky Hub, HoTT, IBUS, JetiExBus, LTM, MAVLink,
  SRXL. CRSF and GHST (ELRS) and MSP telemetry are kept, because that is what a
  whoop actually flies on
- dashboard, camera control
- servos, and the mixer is fixed to `USE_QUAD_MIXER_ONLY`
- IR lap transponder support, but only in `WHOOP_RACE=yes` builds - see below

Baro is deliberately **kept**: whoops increasingly carry one and use altitude
hold. `WHOOP_RACE=yes` drops it, since a race whoop has no use for altitude
hold on a fixed track. Blackbox is kept in both variants - a whoop is easier
to tune with logs, not harder.

If you want any of it back, build without `TINYWHOOP=yes`.

## What the defaults change

Applied by `config/tinywhoop_profile.c` on top of the stock defaults, so each
value is a documented delta rather than a fork of the reset functions.

### Reaction time

| setting | stock | whoop | why |
|---|---|---|---|
| gyro filter slider | 100% | 125% | PT1 group delay is `1/(2*pi*fc)`; +25% cutoff removes ~20% of the filter delay in the gyro -> PID -> motor path. A whoop's noise sits far higher in frequency than a 5"'s, and ducted props put much less energy into the frame |
| D term filter slider | 100% | 125% | same, and the D term filters contribute the larger share of the delay |
| `rc_smoothing_auto_factor_rpy` | 30 | 20 | auto cutoff is `rxRate * 1.5 / (1 + factor/10)`, so a *lower* factor is a higher cutoff. 0.375x -> 0.5x of link rate: 141Hz -> 188Hz at 500Hz ELRS |
| `dyn_notch_min_hz` | 100 | 150 | whoop motors idle at 8-10k RPM, so nothing useful lives at 100Hz and the tracker only gets distracted |
| `dyn_notch_max_hz` | 600 | 700 | they run past 40k RPM |
| `dyn_notch_count` | 3 | 2 | costs measurable loop time on an F4 for little gain when one peak dominates |

Throttle smoothing is left at the stock factor: on a craft this light, throttle
smoothness is worth more than throttle latency.

### Feel and movement

| setting | stock | whoop | why |
|---|---|---|---|
| rates (ACTUAL) | 70 centre / 670 max | 60 / 620 roll-pitch, 50 / 550 yaw (freestyle); 80 / 750 roll-pitch, 60 / 620 yaw (race) | softer centre for threading gaps, still quick at the stops. Yaw is slower because a ducted whoop has little yaw authority to spare. Race trades some of that centre softness for a flatter response suited to holding a line |
| `feedforward_yaw_hold_time` | 100 | 60 (freestyle), 40 (race) | the stock value is documented as a 5" value; small props decay faster, and a track line rewards immediacy over freestyle's smoothness |
| `feedforward_smooth_factor` | 65 | 50 (freestyle), 35 (race) | less smoothing, less lag |
| `feedforward_jitter_factor` | 7 | 10 | whoop links are noisier at small stick deflections |
| `thrust_linear` | 0 | 20 (freestyle), 30 (race) | a small prop's thrust curve is strongly non-linear; without this a whoop is mushy at hover and twitchy up high. Race motors typically run a flatter curve, hence the higher value |
| `throttle_boost` | 5 | 8 | small props spool fast enough to use more of it |
| `motor_idle` | 550 | 650 | a ducted prop stalls and desyncs more easily on a hard throttle chop. Brushed targets already idle higher and are left alone |
| `crashflip_motor_percent` / `crashflip_rate` / `crashflip_auto_rearm` | 0 / 0 / off | 10 / 30 / on | whoops hit things; turtle mode saves the walk. `crashflip_rate` is not optional: the mixer disables both crashflip attenuators outright when it is zero (the stock non-race default), so it has to be set alongside the motor percentage or the craft never eases off power as it rights itself. The 10/30 pairing mirrors the rate/auto-rearm combination `USE_RACE_PRO` already ships |

PID gains are **not** touched. The stock gains fly a whoop perfectly well, and
inventing numbers that cannot be verified on a bench would be worse than
leaving them alone.

### OSD

Stock Betaflight positions every OSD element near the centre of the screen and
leaves all but the warnings hidden, so a freshly flashed craft shows nothing
until the pilot lays out the screen by hand. The whoop profile turns on the four
that matter and puts them in the corners:

```
LQ                                  FLIGHT MODE

                 (warnings)

CELL V                                  TIMER
```

Average cell voltage rather than pack voltage, because that is the number a
whoop pilot lands on. Elements that fall outside the display actually in use -
an NTSC screen is 13 rows, not 16 - are pulled back onto the canvas by `osdInit`
as usual.

## Using it

### In the goggles

`MAIN > WHOOP TUNE` collects the settings that get changed between packs, which
otherwise live across three different menus:

```
-- WHOOP TUNE --
CENTER R/P      centre sensitivity, deg/s / 10
RATE R/P        maximum rate, deg/s / 10
CENTER YAW
RATE YAW
EXPO R/P
-- RESPONSE --
FILTER PCT      one number for both filter sliders: higher = less delay, more noise
RC SMOOTH       lower = higher cutoff = less stick-to-motor delay
THR BOOST
THRUST LIN
MOTOR IDLE
-- PRESET --
WHOOP DEFAULTS  put everything back on the profile above
```

Roll and pitch are edited together on purpose - a whoop that turns at different
rates on the two axes is a tuning mistake rather than a preference. Entries that
need the filters rebuilt are marked as requiring a reboot.

### On the bench

```
# whoop apply
Applied tiny whoop profile to all profiles. Use 'save' to keep it.
# save
```

`whoop apply` writes the profile over the running configuration, which is the
way to pick it up on a craft that was already set up, or to get back to a known
state after an experiment. `defaults` applies it too, since it is part of the
reset path for a whoop build.

## Where it lives

| file | what |
|---|---|
| `src/main/config/tinywhoop_profile.c` | the defaults, one commented delta per line |
| `src/main/cms/cms_menu_tinywhoop.c` | the in-goggle menu |
| `src/main/target/common_post.h` | the feature cut, and the race-mode baro/transponder deltas |
| `src/main/cli/cli.c` | the `whoop` command |
| `Makefile` | `TINYWHOOP=yes` / `WHOOP_RACE=yes` |

The profile is applied from `resetConfig()` *before* `targetConfiguration()`, so
a board config still has the final say on anything hardware specific it sets.
