# Controls

## Arming

When armed the aircraft is ready to fly and any motors attached will spin when throttle is applied.

By default arming and disarming is done using stick positions.  Arming with stick positions is disabled when using a switch to arm.


## Stick positions

LOW - the channel value for the mapped channel input is around 1000
CENTER - the channel value for the mapped channel input is around 1500
HIGH - the channel value for the mapped channel input is around 2000

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


##### Download a graphic [pdf cheat-sheet](https://multiwii.googlecode.com/svn/branches/Hamburger/MultiWii-StickConfiguration-23_v0-5772156649.pdf) with TX stick commands.

The Latest version of this pdf can always be found [Here](https://code.google.com/p/multiwii/source/browse/#svn%2Fbranches%2FHamburger)

## Yaw control

While arming/disarming with sticks, your yaw stick will be moving to extreme values. In order to prevent your craft
from trying to yaw during arming/disarming while on the ground, your yaw input will not cause the craft to yaw when the
throttle is LOW (i.e. below the `min_check` setting).

For tricopters, you may want to retain the ability to yaw while on the ground, so that you can verify that your tail
servo is working correctly before takeoff. You can do this by setting `tri_unarmed_servo` to `1` on the CLI (this is the
default). If you are having issues with your tail rotor contacting the ground during arm/disarm, you can set this to
`0` instead. Check this table to decide which setting will suit you:

<table>
    <tr>
        <th colspan="5">Is yaw control of the tricopter allowed?</th>
    </tr>
    <tr>
        <th></th><th colspan="2">Disarmed</th><th colspan="2">Armed</th>
    </tr>
    <tr>
        <th></th><th>Throttle low</th><th>Throttle normal</th><th>Throttle low</th><th>Throttle normal</th>
    </tr>
    <tr>
        <td rowspan="2">tri_unarmed_servo = 0</td><td>No</td><td>No</td><td>No</td><td>Yes</td>
    </tr>
    <tr>
        <td>No</td><td>No</td><td>No</td><td>Yes</td>
    </tr>
    <tr>
        <td rowspan="2">tri_unarmed_servo = 1</td><td>Yes</td><td>Yes</td><td>Yes</td><td>Yes</td>
    </tr>
    <tr>
        <td>Yes</td><td>Yes</td><td>Yes</td><td>Yes</td>
    </tr>
</table>