# LED Strip

Cleanflight supports the use of addressable LED strips.  Addressable LED strips allow each LED in the strip to
be programmed with a unique and independant color.  This is far more advanced than the normal RGB strips which
require that all the LEDs in the strip show the same color.

Addressable LED strips can be used to show information from the flight controller system, the current implementation
supports the following:

* Up to 32 LEDs.
* Indicators showing pitch/roll stick positions.
* Heading/Orientation lights.
* Flight mode specific color schemes.
* Low battery warning.

The function and orientation configuration is fixed for now but later it should be able to be set via the UI or CLI..

In the future, if someone codes it, they could be used to show GPS navigation status, thrust levels, RSSI, etc.
Lots of scope for ideas and improvements.

Likewise, support for more than 32 LEDs is possible, it just requires additional development.

## Supported hardware

Only strips of 32 WS2812 LEDs are supported currently.  If the strip is longer than 32 LEDs it does not matter,
but only the first 32 are used.

WS2812 LEDs require an 800khz signal and precise timings and thus requires the use of a dedicated hardware timer.

Note: The initial code may work with WS2801 + External LEDs since the protocol is the same, WS2811/WS2812B should also work but
may require very simple timing adjustments to be made in the source.
Not all WS2812 ICs use the same timings, some batches use different timings.  

It could be possible to be able to specify the timings required via CLI if users request it.

### Tested Hardware

* [Adafruit NeoPixel Jewel 7](https://www.adafruit.com/products/2226) (preliminary testing)
  * Measured current consumption in all white mode ~ 350 mA.
  * Fits well under motors on mini 250 quads.
* [Adafruit NeoPixel Stick](https://www.adafruit.com/products/1426) (works well)
  * Measured current consumption in all white mode ~ 350 mA.

## Connections

WS2812 LED strips generally require a single data line, 5V and GND.

WS2812 LEDs on full brightness can consume quite a bit of current.  It is recommended to verify the current draw and ensure your
supply can cope with the load.  On a multirotor that uses multiple BEC ESC's you can try use a different BEC to the one the FC
uses.  e.g. ESC1/BEC1 -> FC, ESC2/BEC2 -> LED strip.   It's also possible to power one half of the strip from one BEC and the other half
from another BEC.  Just ensure that the GROUND is the same for all BEC outputs and LEDs.


| Target                | Pin | LED Strip | Signal |
| --------------------- | --- | --------- | -------|
| Naze/Olimexino        | RC5 | Data In   | PA6    |
| CC3D                  | ??? | Data In   | PB4    |
| ChebuzzF3/F3Discovery | PB8 | Data In   | PB8    |


Since RC5 is also used for SoftSerial on the Naze/Olimexino it means that you cannot use SoftSerial and led strips at the same time.
Additionally, since RC5 is also used for Parallel PWM RC input on both the Naze, Chebuzz and STM32F3Discovery targets, led strips
can not be used at the same time at Parallel PWM.

Ensure that your 5V supply is not too high, if the voltage at the input to the LED strip is to high then the LEDs may not light; The
problem occurs because of the difference in voltage between the data signal and the power signal.

If you are using an BEC from an ESC to for the 5v supply check the output is as close to 5v as possible.

It was observed that a 5.4v supply from a BEC was fine for powering the FC and other devices but the LED strip would not light until
the voltage was reduced, this was acheived by placing an IN4007 diode between the BEC 5v output and the 5V input pin of the LED strip.
  

## Configuration

Enable the `LED_STRIP` feature via the cli:

```
feature LED_STRIP
```

If you enable LED_STRIP feature and the feature is turned off again after a reboot then check your config does not conflict with other features, as above.

Configure the LEDs using the `led` command.

The `led` command takes either zero or two arguments - an zero-based led number and a pair of coordinates, direction flags and mode flags.

If used with zero arguments it prints out the led configuration which can be copied for future reference.

Each led is configured using the following template: `x,y:ddd:mmm`

`x` and `y` are grid coordinates of a 0 based 16x16 grid, north west is 0,0, south east is 15,15
`ddd` specifies the directions, since an led can face in any direction it can have multiple directions.  Directions are:

 `N` - North
 `E` - East
 `S` - South
 `W` - West
 `U` - Up
 `D` - Down

For instance, an LED that faces South-east at a 45 degree downwards angle could be configured as `SED`.

Note: It is perfectly possible to configure an LED to have all directions `NESWUD` but probably doesn't make sense.

`mmm` specifies the modes that should be applied an LED.  Modes are:

* `W` - `W`warnings.
* `F` - `F`light mode & Orientation
* `I` - `I`ndicator.
* `A` - `A`rmed state.
* `T` - `T`hrust state.

Example:

```
led 0 0,15:SD:IAW
led 1 15,0:ND:IAW
led 2 0,0:ND:IAW
led 3 0,15:SD:IAW
```

to erase an led, and to mark the end of the chain, use `0,0::` as the second argument, like this:

```
led 4 0,0::
```


### Modes

#### Warning

This mode simply uses the LEDs to flash when warnings occur.

| Warning | LED Pattern | Notes |
|---------|-------------|-------|
| Arm-lock enabled | flash between green and off | occurs calibration or when unarmed and the aircraft is tilted too much |
| Low Battery | flash red and off | battery monitoring must be enabled.  May trigger temporarily under high-throttle due to voltage drop |
| Failsafe | flash between light blue and yellow | Failsafe must be enabled |

Flash patterns appear in order, so that it's clear which warnings are enabled.

#### Flight Mode & Orientation

This mode shows the flight mode and orientation.

When flight modes are active then the LEDs are updated to show different colors depending on the mode, placement on the grid and direction.

LEDs are set in a specific order:
 * LEDs that marked as facing up or down.
 * LEDs that marked as facing west or east AND are on the west or east side of the grid.
 * LEDs that marked as facing north or south AND are on the north or south side of the grid.

That is, south facing LEDs have priority.

#### Indicator

This mode flashes LEDs that correspond to roll and pitch stick positions.  i.e.  they indicate the direction the craft is going to turn.

#### Armed state

This mode toggles LEDs between green and blue when disarmed and armed, respectively.

Note: Armed State cannot be used with Flight Mode.

#### Thrust state

This mode fades the LED current LED color to the previous/next color in the HSB color space depending on throttle stick position.  When the
throttle is in the middle position the color is unaffected, thus it can be mixed with orientation colors to indicate orientation and throttle at
the same time.

## Positioning

Cut the strip into sections as per diagrams below.  When the strips are cut ensure you reconnect each output to each input with cable where the break is made.
e.g. connect 5V out to 5V in, GND to GND and Data Out to Data In.

Orientation is when viewed with the front of the aircraft facing away from you and viewed from above.

### Example 12 LED config

The default configuration is as follows
```
led 0 2,2:ES:IA
led 1 2,1:E:WF
led 2 2,0:NE:IA
led 3 1,0:N:F
led 4 0,0:NW:IA
led 5 0,1:W:WF
led 6 0,2:SW:IA
led 7 1,2:S:WF
led 8 1,1:U:WF
led 9 1,1:U:WF
led 10 1,1:D:WF
led 11 1,1:D:WF
```

Which translates into the following positions:

```
     5             3
      \           / 
       \    4    / 
        \ FRONT / 
      6 | 9-12  | 2
        /  BACK \ 
       /    8    \
      /           \ 
     7             1  
```

LEDs 1,3,5 and 7 should be placed underneath the quad, facing downwards.
LEDs 2, 4, 6 and 8 should be positioned so the face east/north/west/south, respectively.
LEDs 9-10 should be placed facing down, in the middle
LEDs 11-12 should be placed facing up, in the middle

This is the default so that if you don't want to place LEDs top and bottom in the middle just connect the first 8 LEDs.

### Example 16 LED config

```
led 0 15,15:SD:IA
led 1 8,8:E:FW
led 2 8,7:E:FW
led 3 15,0:ND:IA
led 4 7,7:N:FW
led 5 8,7:N:FW
led 6 0,0:ND:IA
led 7 7,7:W:FW
led 8 7,8:W:FW
led 9 0,15:SD:IA
led 10 7,8:S:FW
led 11 8,8:S:FW
led 12 7,7:D:FW
led 13 8,7:D:FW
led 14 7,7:U:FW
led 15 8,7:U:FW
```

Which translates into the following positions:

```
     7             4
      \           / 
       \   6-5   / 
      8 \ FRONT / 3
        | 13-16 | 
      9 /  BACK \ 2
       /  11-12  \
      /           \ 
    10             1  
```

LEDs 1,4,7 and 10 should be placed underneath the quad, facing downwards.
LEDs 2-3, 6-5, 8-9 and 11-12 should be positioned so the face east/north/west/south, respectively.
LEDs 13-14 should be placed facing down, in the middle
LEDs 15-16 should be placed facing up, in the middle
 
### Exmple 28 LED config

```
#right rear cluster
led 0 9,9:S:FWT
led 1 10,10:S:FWT
led 2 11,11:S:IA
led 3 11,11:E:IA
led 4 10,10:E:AT
led 5 9,9:E:AT
# right front cluster
led 6 10,5:S:F
led 7 11,4:S:F
led 8 12,3:S:IA
led 9 12,2:N:IA
led 10 11,1:N:F
led 11 10,0:N:F
# center front cluster
led 12 7,0:N:FW
led 13 6,0:N:FW
led 14 5,0:N:FW
led 15 4,0:N:FW
# left front cluster
led 16 2,0:N:F
led 17 1,1:N:F
led 18 0,2:N:IA
led 19 0,3:W:IA
led 20 1,4:S:F
led 21 2,5:S:F
# left rear cluster
led 22 2,9:W:AT
led 23 1,10:W:AT
led 24 0,11:W:IA
led 25 0,11:S:IA
led 26 1,10:S:FWT
led 27 2,9:S:FWT
```

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

All LEDs should face outwards from the chassis in this configuration.

Note:
This configuration is specifically designed for the [Alien Spider AQ50D PRO 250mm frame](http://www.goodluckbuy.com/alien-spider-aq50d-pro-250mm-mini-quadcopter-carbon-fiber-micro-multicopter-frame.html).


## Troubleshooting

On initial power up the LEDs on the strip will be set to WHITE.  This means you can attach a current meter to verify
the current draw if your measurement equipment is fast enough.  Most 5050 LEDs will draw 0.3 Watts a piece.
This also means that you can make sure that each R,G and B LED in each LED module on the strip is also functioning.

After a short delay the LEDs will show the unarmed color sequence and or low-battery warning sequence.

If the LEDs flash intermittently or do not show the correct colors verify all connections and check the specifications of the
LEDs you have against the supported timings (for now, you'll have to look in the source).

Also check that the feature `LED_STRIP` was correctly enabled and that it does not conflict with other features, as above.
