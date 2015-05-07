# RSSI

RSSI is a measurement of signal strength and is very handy so you know when your aircraft isw going out of range or if it is suffering RF interference.

Some receivers have RSSI outputs.  3 types are supported.

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
set rssi_ppm_invert = 1
```
Default is set to "0" for normal operation ( 100 = Full signal / 0 = Lost signal).

## RSSI via Parallel PWM channel

Connect the RSSI signal to any PWM input channel then set the RSSI channel as you would for RSSI via PPM

## RSSI ADC

Connect the RSSI signal to the RC2/CH2 input. The signal must be between 0v and 3.3v.
Use inline resistors to lower voltage if required; inline smoothing capacitors may also help.
A simple PPM->RSSI conditioner can easily be made. See the  PPM-RSSI conditioning.pdf  for details.

Under CLI :
- enable using the RSSI_ADC feature  :  `feature RSSI_ADC`
- set the RSSI_SCALE parameter (between 1 and 255) to adjust RSSI level according to your configuration.


FrSky D4R-II and X8R supported.

The feature can not be used when RX_PARALLEL_PWM is enabled.
