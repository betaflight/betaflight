# Led Strip

Cleanflight supports the use of addressable LED strips.  Addressable LED strips allow each LED in the strip to
be programmed with a unique and independant color.  This is far more advanced than the normal RGB strips which
require that all the LEDs in the strip show the same color.

Addressable LED strips can be used to show information from the flight controller system, the current implementation
supports the following:

* Indicators showing pitch/roll stick positions.
* Heading/Orientation lights.
* Flight mode specific color schemes.
* Low battery warning.

In the future, if someone codes it, they could be used to show GPS navigation status, thrust levels, RSSI, etc.
Lots of scope for ideas and improvements.

Likewise, support for more than 10 LEDs is possible, it just requires additional development.  10 was chosen to
start with due to current draw and RAM usage.

## Supported hardware

Only strips of 10 WS2812 LEDs are supported currently.  If the strip is longer than 10 leds it does not matter,
but only the first 10 are used.

WS2812 LEDs require an 800khz signal and precise timings and thus requires the use of a dedicated hardware timer.

Note: The initial code may work with WS2801 + External LEDs since the protocol is the same, WS2811/WS2812B should also work but
may require very simple timing adjustments to be made in the source.
Not all WS2812 ICs use the same timings, some batches use different timings.  

It could be possible to be able to specify the timings required via CLI if users request it.

## Connections

WS2812 LED strips generally require a single data line, 5V and GND.

WS2812 LEDs on full brightness can consume quite a bit of current.  It is recommended to verify the current draw and ensure your
supply can cope with the load.  On a multirotor that uses multiple BEC ESC's you can try use a different BEC to the one the FC
uses.  e.g. ESC1/BEC1 -> FC, ESC2/BEC2 -> LED strip. 


| Target                | Pin | Led Strip |
| --------------------- | --- | --------- |
| Naze/Olimexino        | RC5 | Data In   |
| ChebuzzF3/F3Discovery | PB8 | Data In   |


Since RC5 is also used for SoftSerial on the Naze/Olimexino it means that you cannot use softserial and led strips at the same time.
Additionally, since RC5 is also used for Parallel PWM RC input on both the Naze, Chebuzz and STM32F3Discovery targets, led strips
can not be used at the same time at Parallel PWM.

## Positioning

Cut the strip of 10 LED's in half,  the first half goes at the front of the quad, from front left to from right. Position
the center LED in the middle.  When the strips are cut ensure you reconnect each output to each input with cable where the break is made.
e.g. connect 5V out to 5V in, GND to GND and Data Out to Data In.

The second half of the strip goes at the back of the aircraft, from back right to back left, again place the center LED in the middle.

Orientation is when viewed with the front of the aircraft facing away from you and viewed from above.

Example LED numbers and positions for a quad.

```
 1  2        4  5
   \           /
    \    3    / 
     \ FRONT /
     /  BACK \
    /    8    \
   /           \
10  9         7  6  
```

## Configuration

Enable the `LED_STRIP` feature via the cli:

```
feature LED_STRIP
```

If you enable LED_STRIP feature and the feature is turned off again after a reboot then check your config does not conflict with other features, as above.

## Troubleshooting

On initial power up the first 10 LEDs on the strip will be set to WHITE.  This means you can attach a current meter to verify
the current draw if your measurement equipment is fast enough.  This also means that you can make sure that each R,G and B LED
in each LED module on the strip is also functioning.

After a short delay the LEDs will show the unarmed color sequence and or low-battery warning sequence.

If the LEDs flash intermittently or do not show the correct colors verify all connections and check the specifications of the
LEDs you have against the supported timings (for now, you'll have to look in the source).

Also check that the feature `LED_STRIP` was correctly enabled and that it does not conflict with other features, as above.
