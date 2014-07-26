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

The function and orientation configuration is fixed for now but later it should be able to be set via the UI or CLI..

In the future, if someone codes it, they could be used to show GPS navigation status, thrust levels, RSSI, etc.
Lots of scope for ideas and improvements.

Likewise, support for more than 28 LEDs is possible, it just requires additional development.

## Supported hardware

Only strips of 28 WS2812 LEDs are supported currently.  If the strip is longer than 28 leds it does not matter,
but only the first 28 are used.

WS2812 LEDs require an 800khz signal and precise timings and thus requires the use of a dedicated hardware timer.

Note: The initial code may work with WS2801 + External LEDs since the protocol is the same, WS2811/WS2812B should also work but
may require very simple timing adjustments to be made in the source.
Not all WS2812 ICs use the same timings, some batches use different timings.  

It could be possible to be able to specify the timings required via CLI if users request it.

## Connections

WS2812 LED strips generally require a single data line, 5V and GND.

WS2812 LEDs on full brightness can consume quite a bit of current.  It is recommended to verify the current draw and ensure your
supply can cope with the load.  On a multirotor that uses multiple BEC ESC's you can try use a different BEC to the one the FC
uses.  e.g. ESC1/BEC1 -> FC, ESC2/BEC2 -> LED strip.   It's also possible to power one half of the strip from one BEC and the other half
from another BEC.  Just ensure that the GROUND is the same for all BEC outputs and LEDs.


| Target                | Pin | Led Strip |
| --------------------- | --- | --------- |
| Naze/Olimexino        | RC5 | Data In   |
| ChebuzzF3/F3Discovery | PB8 | Data In   |


Since RC5 is also used for SoftSerial on the Naze/Olimexino it means that you cannot use softserial and led strips at the same time.
Additionally, since RC5 is also used for Parallel PWM RC input on both the Naze, Chebuzz and STM32F3Discovery targets, led strips
can not be used at the same time at Parallel PWM.

## Positioning

Cut the strip into 5 sections as per diagram below.  When the strips are cut ensure you reconnect each output to each input with cable where the break is made.
e.g. connect 5V out to 5V in, GND to GND and Data Out to Data In.

Orientation is when viewed with the front of the aircraft facing away from you and viewed from above.

LED numbers and positions for a quad.

```
       17-19  10-12
20-22 \           / 7-9
       \  13-16  / 
        \ FRONT /
        /  BACK \
       /         \
23-25 /           \ 4-6
       26-28   1-3  
```

## Configuration

Enable the `LED_STRIP` feature via the cli:

```
feature LED_STRIP
```

If you enable LED_STRIP feature and the feature is turned off again after a reboot then check your config does not conflict with other features, as above.

## Troubleshooting

On initial power up the LEDs on the strip will be set to WHITE.  This means you can attach a current meter to verify
the current draw if your measurement equipment is fast enough.  This also means that you can make sure that each R,G and B LED
in each LED module on the strip is also functioning.

After a short delay the LEDs will show the unarmed color sequence and or low-battery warning sequence.

If the LEDs flash intermittently or do not show the correct colors verify all connections and check the specifications of the
LEDs you have against the supported timings (for now, you'll have to look in the source).

Also check that the feature `LED_STRIP` was correctly enabled and that it does not conflict with other features, as above.
