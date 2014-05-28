# RSSI

RSSI is a measurement of signal strength.  RSSI is very handy so you know when you are going out of range or there is interference.

Some receivers have RSSI outputs.  3 types are supported.

1. RSSI via PPM channel
2. RSSI via Parallel PWM channel
3. RSSI via ADC with PPM RC that has an RSSI output - aka RSSI ADC

## RSSI via PPM

Configure your receiver to output RSSI on a spare channel, then select the channel used via the cli.

e.g. if you used channel 1 then you would set:

```
set rssi_channel = 1
```

## RSSI via Parallel PWM channel

Connect the RSSI signal to any PWM input channel then set the RSSI channel as you would for RSSI via PPM

## RSSI ADC

Connect the RSSI signal to the RC2/CH2 input.  The signal must be between 0v and 3.3v to indicate between 0% and 100% RSSI.
Use inline resistors to lower voltage if required, inline smoothing capacitors may also help.

FrSky D4R-II and X8R supported.

Enable using the RSSI_ADC feature:

```
feature RSSI_ADC
```

The feature can not be used when RX_PARALLEL_PWM is enabled.
