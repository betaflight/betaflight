### IO variables

`gyroData/8192*2000 = deg/s`

`gyroData/4 ~ deg/s`

`rcCommand` - `<-500 - 500>` nominal, but is scaled with `rcRate/100`, max +-1250

`inclination` - in 0.1 degree, default 50 degrees (500)

`axisPID` - output to mixer, will be added to throttle(`<1000-2000>`), output range is `<minthrottle, maxthrottle>` (default `<1150 - 1850>`)

### PID controller 0, "MultiWii" (default)


#### Leveling term
```
error = constrain(2*rcCommand[axis], limit +- max_angle_inclination) - inclination[axis]
P = constrain(P8[PIDLEVEL]/100 * error, limit +- 5 * D8[PIDLEVEL])
I = intergrate(error, limit +-10000) * I8[PIDLEVEL] / 4096
```
#### Gyro term
```
P = rcCommand[axis];
error = rcCommand[axis] * 10 * 8 / pidProfile->P8[axis] - gyroData[axis] / 4; (conversion so that error is in deg/s ?)
I = integrate(error, limit +-16000) / 10 / 8  * I8[axis] / 100 (conversion back to mixer units ?)
```

reset I term if
  - axis rotation rate > +-64deg/s
  - axis is YAW and rcCommand>+-100

##### Mode dependent mix(yaw is always from gyro)

- HORIZON - proportionally according to max deflection

- gyro - gyro term

- ANGLE - leveling term

#### Gyro stabilization

```
P = -gyroData[axis] / 4 * dynP8 / 10 / 8
D = -mean(diff(gyroData[axis] / 4), over 3 samples) * 3 * dynD8 / 32
```