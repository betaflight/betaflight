### IO variables

`gyroADC/8192*2000 = deg/s`

`gyroADC/4 ~ deg/s`

`rcCommand` - `<-500 - 500>` nominal, but is scaled with `rcRate/100`, max +-1250

`inclination` - in 0.1 degree, roll and pitch deviation from horizontal position
`max_angle_inclination` - in 0.1 degree, default 50 degrees (500)

`axisPID` - output to mixer, will be added to throttle(`<1000-2000>`), output range is `<minthrottle, maxthrottle>` (default `<1150 - 1850>`)

### PID controller 0, "MultiWii" (default)


#### Leveling term
```
error = constrain(2*rcCommand[axis], limit +- max_angle_inclination) - inclination[axis]
Pacc = constrain(P8[PIDLEVEL]/100 * error, limit +- 5 * D8[PIDLEVEL])
Iacc = intergrate(error, limit +-10000) * I8[PIDLEVEL] / 4096
```
#### Gyro term
```
Pgyro = rcCommand[axis];
error = rcCommand[axis] * 10 * 8 / pidProfile->P8[axis] - gyroADC[axis] / 4; (conversion so that error is in deg/s ?)
Igyro = integrate(error, limit +-16000) / 10 / 8  * I8[axis] / 100 (conversion back to mixer units ?)
```

reset I term if
  - axis rotation rate > +-64deg/s
  - axis is YAW and rcCommand>+-100

##### Mode dependent mix(yaw is always from gyro)

- HORIZON - proportionally according to max deflection
```
  deflection = MAX(ABS(rcCommand[PITCH]), ABS(rcCommand[ROLL])) / 500 ; limit to 0.0 .. 1.0
  P = Pacc * (1-deflection) + Pgyro * deflection
  I = Iacc * (1-deflection) + Igyro * deflection
```
- gyro
```
  P = Pgyro
  I = Igyro
```
- ANGLE
```
  P = Pacc
  I = Iacc
```
#### Gyro stabilization

```
P -=  gyroADC[axis] / 4 * dynP8 / 10 / 8
D = -mean(diff(gyroADC[axis] / 4), over 3 samples) * 3 * dynD8 / 32
[equivalent to :]
D = - (gyroADC[axis]/4 - (<3 loops old>gyroADC[axis]/4)) * dynD8 / 32
```

This can be seen as sum of
 - PI controller (handles rcCommand, HORIZON/ANGLE); `Igyro` is only output based on gyroADC
 - PD controller(parameters dynP8/dynD8) with zero setpoint acting on gyroADC
