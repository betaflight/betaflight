# Profiles

A profile is a set of configuration settings.

Currently three profiles are supported. The default profile is profile `0`.

## Changing profiles

Profiles can be selected using a GUI or the following stick combinations:

| Profile | Throttle | Yaw   | Pitch  | Roll   |
| ------- | -------- | ----- | ------ | ------ |
| 0       | Down     | Left  | Middle | Left   |
| 1       | Down     | Left  | Up     | Middle |
| 2       | Down     | Left  | Middle | Right  |

The CLI `profile` command can also be used:

```
profile <index>
```

# Rate Profiles

INAV supports rate profiles in addition to regular profiles.

Rate profiles contain settings that adjust how your craft behaves based on control input.

Three rate profiles are supported.

Rate profiles can be selected while flying using the inflight adjustments feature.

Each normal profile has a setting called 'default_rate_profile`.  When a profile is activated the
corresponding rate profile is also activated.

Profile 0 has a default rate profile of 0.
Profile 1 has a default rate profile of 1.
Profile 2 has a default rate profile of 2.

The defaults are set this way so that it's simple to configure a profile and a rate profile at the same.

The current rate profile can be shown or set using the CLI `rateprofile` command:

```
rateprofile <index>
```

The values contained within a rate profile can be seen by using the CLI `dump rates` command.

e.g
```
# dump rates

# rateprofile
rateprofile 0

set rc_expo = 65
set thr_mid = 50
set thr_expo = 0
set roll_pitch_rate = 0
set yaw_rate = 0
set tpa_rate = 0
set tpa_breakpoint = 1500
```
