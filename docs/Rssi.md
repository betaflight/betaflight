# RSSI

RSSI is a measurement of signal strength.  RSSI is very handy so you know when you are going out of range or there is interference.

Some receivers have RSSI outputs.  3 types are supported.

1. RSSI via PPM channel
2. RSSI via Parallel PWM channel
3. RSSI via PWM with PPM RC that does not have RSSI output - aka RSSI PWM

## RSSI via PPM

Configure your receiver to output RSSI on a spare channel, then select the channel used via the cli.

e.g. if you used channel 1 then you would set:

```
set rssi_channel = 1
```

## RSSI via Parallel PWM channel

Connect the RSSI signal to any PWM input channel then set the RSSI channel as you would for RSSI via PPM

## RSSI PWM

Connect the RSSI PWM signal to the RC2/CH2 input.

Enable using the RSSI_PWM feature:

```
feature RSSI_PWM
```

The feature can not be used when RX_PARALLEL_PWM is enabled.


### RSSI PWM Providers

When using RSSI PWM it is possible to use standard ~18ms RSSI signals or a faster 1khz/1m RSSI signal.

The RSSI output on the FrSky X8R (and probably the FrSky X6R) is 1khz.

To support the 1khz rate enable it via the cli:

```
set rssi_pwm_provider = 1
```

| Value | Meaning     |
| ----- | ----------- |
| 0     | ~18ms pulse |
| 1     | 1ms pulse   |
