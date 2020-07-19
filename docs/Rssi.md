# RSSI

RSSI is a measurement of signal strength and is very handy so you know when your aircraft is going out of range or if it is suffering RF interference.

Some receivers have RSSI outputs. 3 types are supported.

1. RSSI via PPM channel
1. RSSI via Parallel PWM channel
1. RSSI via ADC with PPM RC that has an RSSI output - aka RSSI ADC

## RSSI via PPM

Configure your receiver to output RSSI on a spare channel, then select the channel used via the CLI.

e.g. if you used channel 9 then you would set:

```
set rssi_channel = 9
```
Note: Some systems such as EZUHF invert the RSSI ( 0 = Full signal / 100 = Lost signal). To correct this problem you can invert the channel input so you will get a correct reading by using command:

```
set rssi_invert = ON
```
Default is set to "OFF" for normal operation ( 100 = Full signal / 0 = Lost signal).

## RSSI via Parallel PWM channel

Connect the RSSI signal to any PWM input channel then set the RSSI channel as you would for RSSI via PPM

## RSSI from Futaba S.Bus receiver

The S.Bus serial protocol includes detection of dropped frames. These may be monitored and reported as RSSI by using the following command.

```
set rssi_src_frame_errors = ON
```

Note that RSSI stands for Received Signal Strength Indicator; the detection of S.Bus dropped frames is really a signal quality, not strength indication. Consequently you may experience a more rapid drop in reported RSSI at the extremes of range when using this facility than when using RSSI reporting signal strength.

## RSSI ADC

Connect the RSSI signal to the RC2/CH2 input. The signal must be between 0v and 3.3v.
Use inline resistors to lower voltage if required; inline smoothing capacitors may also help.
A simple PPM->RSSI conditioner can easily be made. See the  PPM-RSSI conditioning.pdf  for details.

Under CLI :
- enable using the RSSI_ADC feature  :  `feature RSSI_ADC`
- set the RSSI_SCALE parameter (between 1 and 255) to adjust RSSI level according to your configuration. The raw ADC value is divided by the value of this parameter.

Note: Some systems invert the RSSI ( 0 = Full signal / 100 = Lost signal). To correct this problem you can invert the input so you will get a correct reading by using command:

```
set rssi_invert = ON
```

### RSSI_SCALE setup method

- set rssi_scale = 100. The displayed percentage will then be the raw ADC value.
- turn on RX (close to board). RSSI value should vary a little.
- Update rssi_scale to the maximum RSSI value previously measured.

FrSky D4R-II and X8R supported.

The feature can not be used when RX_PARALLEL_PWM is enabled.


## RSSI_SCALE setup method

To calculate the rssi offset and scale, check the rc value at full signal (`rssi_fullsig`) and at almost no signal strength (`rssi_nosig`).
Then, calculate the offset and scale values using the following formula:

```
rssi_offset = (1000-rssi_nosig) / 10
rssi_scale = 100 * 1000 / (rssi_fullsig - rssi_nosig)
```

Examples are:

| RC System | RC value at full strength | RC value at no strength | `rssi_offset` | `rssi_scale` |
|:----------|:--------------------------|:------------------------|:--------------|:-------------|
| Graupner  | `1900`                    | `1100`                  | `-10`         | `125`        |

Then set these values via CLI:
```
set rssi_offset = -10
set rssi_scale = 125
```
