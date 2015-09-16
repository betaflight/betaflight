# Buzzer

Cleanflight supports a buzzer which is used for the following purposes:

 * Low and critical battery alarms (when battery monitoring enabled)
 * Arm/disarm tones (and warning beeps while armed)
 * Notification of calibration complete status
 * TX-AUX operated beeping - useful for locating your aircraft after a crash
 * Failsafe status
 * Flight mode change
 * Rate profile change (via TX-AUX switch)

If the arm/disarm is via the control stick, holding the stick in the disarm position will sound a repeating tone.  This can be used as a lost-model locator.

Three beeps immediately after powering the board means that the gyroscope calibration has completed successfully. Cleanflight calibrates the gyro automatically upon every power-up. It is important that the copter stay still on the ground until the three beeps sound, so that gyro calibration isn't thrown off. If you move the copter significantly during calibration, Cleanflight will detect this, and will automatically re-start the calibration once the copter is still again. This will delay the "three beeps" tone. If you move the copter just a little bit, the gyro calibration may be incorrect, and the copter may not fly correctly. In this case, the gyro calibration can be performed manually via [stick command](Controls.md), or you may simply power cycle the board.

There is a special arming tone used if a GPS fix has been attained, and there's a "ready" tone sounded after a GPS fix has been attained (only happens once).  The tone sounded via the TX-AUX-switch will count out the number of satellites (if GPS fix).

The CLI command `play_sound` is useful for demonstrating the buzzer tones. Repeatedly entering the command will play the various tones in turn. Entering the command with a numeric-index parameter will play the associated tone.

Buzzer is enabled by default on platforms that have buzzer connections.

## Tone sequences

Buzzer tone sequences (square wave generation) are made so that : 1st, 3rd, 5th, .. are the delays how long the beeper is on and 2nd, 4th, 6th, .. are the delays how long beeper is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).

Sequences available in Cleanflight v1.10 are :

    Gyro calibrated            20, 10, 20, 10, 20, 10
    RX lost landing            beeps SOS
    TX off or signal lost      50, 50	 (repeats until TX is okay)
    Disarming the board        15, 5, 15, 5
    Arming the board           30, 5, 5, 5
    Arming and GPS has fix     5, 5, 15, 5, 5, 5, 15, 30
    Battery is critically low  50, 2	(repeats)
    Battery is getting low     25, 50	 (repeats) 
    GPS status                 multi beeps	(sat count)
    RX set                     short beep	(when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled)
    ACC inflight calibration completed    two short beeps
    ACC inflight calibration failed       two longer beeps
    GPS locked and copter ready           4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5
    Multi beeps                           variable # of beeps	(confirmation, GPS sat count, etc)
    Stick held in disarm position         0, 100, 10	(after pause)
    Board is armed                        0, 245, 10, 5	(after pause ; repeats until board is disarmed or throttle is increased)

with

    SOS morse code      10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70
    Short fast beep     10, 10
    Two short beeps     5, 5, 5, 5
    Two longer beeps    20, 15, 35, 5


## Types of buzzer supported

The buzzers are enabled/disabled by simply enabling or disabling a GPIO output pin on the board.
This means the buzzer must be able to generate its own tone simply by having power applied to it.

Buzzers that need an analog or PWM signal do not work and will make clicking noises or no sound at all.

Examples of a known-working buzzers.

 * [Hcm1205x Miniature Buzzer 5v](http://www.rapidonline.com/Audio-Visual/Hcm1205x-Miniature-Buzzer-5v-35-0055)
 * [5V Electromagnetic Active Buzzer Continuous Beep](http://www.banggood.com/10Pcs-5V-Electromagnetic-Active-Buzzer-Continuous-Beep-Continuously-p-943524.html)
 * [Radio Shack Model: 273-074 PC-BOARD 12VDC (3-16v) 70DB PIEZO BUZZER](http://www.radioshack.com/pc-board-12vdc-70db-piezo-buzzer/2730074.html#.VIAtpzHF_Si)
 * [MultiComp MCKPX-G1205A-3700 TRANSDUCER, THRU-HOLE, 4V, 30MA](http://uk.farnell.com/multicomp/mckpx-g1205a-3700/transducer-thru-hole-4v-30ma/dp/2135914?CMP=i-bf9f-00001000)

## Connections

### Naze32

Connect a supported buzzer directly to the BUZZ pins. Observe polarity. Also if you are working with flight controller outside of a craft, on a bench for example, you need to supply 5 volts and ground to one of the ESC connections or the buzzer will not function.


### CC3D

Buzzer support on the CC3D requires that a buzzer circuit be created to which the input is PA15.
PA15 is unused and not connected according to the CC3D Revision A schematic.
Connecting to PA15 requires careful soldering.

See the [CC3D - buzzer circuit.pdf](Wiring/CC3D - buzzer circuit.pdf) for details.
