# Tx PID

This allows control of the PID settings for Roll, Pitch and Yaw to be adjusted using an aux channel on the transmitter. 

## Configuration via CLI

Tx PID configuration is performed by setting the following variables.

These are each arrays of eight values, interpreted thus:

| Array Item | Function |
| ---------- | -------- |
| 1 | Roll P Value |
| 2 | Roll I Value |
| 3 | Roll D Value |
| 4 | Pitch P Value |
| 5 | Pitch I Value |
| 6 | Pitch D Value |
| 7 | Yaw P Value |
| 8 | Yaw I Value |


| Variable | Function |
| -------- | --------- |
| txpid_channel  | Sets the Aux channel number, or zero to disable |
| txpid_center | Sets the P, I or D value to set when aux channel is at mid-point |
| txpid_adjust | Setes the amount to increase/decrease the P, I or D value when the aux channel is at max/min point respectively |

### Example Setting

The following example will allow the Pitch P value to be adjusted from Aux channel 1, with a center value of 50, adjustable between 40 and 60 by adjusting the Aux 1 channel setting.

```
set txpid_channel=0,0.0,1,0,0,0,0
set txpid_center=0,0,0,50,0,0,0,0
set txpid_adjust=0,0,0,10,0,0,0,0
```

The following example adjusts all value of Roll P, I and D at the same time using Aux channel 2 with center settings of 40, 40 and 40 respectively, all values of Pitch P, I and D at the same time using Aux channel 3 with center settings of 58,50 and 35 respectively, and both value of Yaw P and I at the same time using Aud channel 4 with center settings of 70 and 45. In this example the adjustment range is set to ± half the center value.

```
set txpid_channel=2,2,2,3,3,3,4,4
set txpid_center=40,40,40,58,50,35,70,45
set txpid_adjust=20,20,15,29,25,17,35,22
```

