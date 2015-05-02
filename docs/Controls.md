# Controls

## Arming

When armed, the aircraft is ready to fly and the motors will spin when throttle is applied.  The motors will 
spin at a slow speed when armed (this feature may be disabled by setting MOTOR_STOP, but for safety reasons,
that is not recommended).

By default, arming and disarming is done using stick positions.  (NOTE: this feature is disabled when using a 
switch to arm.)

## Stick Positions

The three stick positions are:

|Position        | Approx. Channel Input|
|----------------|----------------------|
|LOW             | 1000                 |
|CENTER          | 1500                 |
|HIGH            | 2000                 |

The stick positions are combined to activate different functions:

| Function                      | Throttle | Yaw     | Pitch  | Roll   |
| ----------------------------- | -------- | ------- | ------ | ------ |
| ARM                           | LOW      | HIGH    | CENTER | CENTER | 
| DISARM                        | LOW      | LOW     | CENTER | CENTER |
| Profile 1                     | LOW      | LOW     | CENTER | LOW    | 
| Profile 2                     | LOW      | LOW     | HIGH   | CENTER | 
| Profile 3                     | LOW      | LOW     | CENTER | HIGH   | 
| Calibrate Gyro                | LOW      | LOW     | LOW    | CENTER |
| Calibrate Acc                 | HIGH     | LOW     | LOW    | CENTER |
| Calibrate Mag/Compass         | HIGH     | HIGH    | LOW    | CENTER |
| Inflight calibration controls | LOW      | LOW     | HIGH   | HIGH   |
| Trim Acc Left                 | HIGH     | CENTER  | CENTER | LOW    |
| Trim Acc Right                | HIGH     | CENTER  | CENTER | HIGH   |
| Trim Acc Forwards             | HIGH     | CENTER  | HIGH   | CENTER |
| Trim Acc Backwards            | HIGH     | CENTER  | LOW    | CENTER |
| Disable LCD Page Cycling      | LOW      | CENTER  | HIGH   | LOW    |
| Enable LCD Page Cycling       | LOW      | CENTER  | HIGH   | HIGH   |
| Save setting                  | LOW      | LOW     | LOW    | HIGH   |


Download a graphic [cheat sheet](https://multiwii.googlecode.com/svn/branches/Hamburger/MultiWii-StickConfiguration-23_v0-5772156649.pdf) with Tx stick commands (the latest version can always be found 
[here](https://code.google.com/p/multiwii/source/browse/#svn%2Fbranches%2FHamburger)).
