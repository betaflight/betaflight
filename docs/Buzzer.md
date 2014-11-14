# Buzzer

Cleanflight supports a buzzer which is used for the following purposes, and more:

Low Battery alarm (when battery monitoring enabled)
Notification of calibration complete status.
AUX operated beeping - useful for locating your aircraft after a crash.
Failsafe status.

Buzzer is enabled by default on platforms that have buzzer connections.

## Types of buzzer supported

The buzzers are enabled/disabled by simply enabling or disabling a GPIO output pin on the board.
This means the buzzer must be able to generate it's own tone simply by having power applied to it.

Buzzers that need an analogue or PWM signal do not work and will make clicking noises or no sound at all.

Examples of a known-working buzzers.

 * [Hcm1205x Miniature Buzzer 5v](http://www.rapidonline.com/Audio-Visual/Hcm1205x-Miniature-Buzzer-5v-35-0055)
 * [5V Electromagnetic Active Buzzer Continuous Beep](http://www.banggood.com/10Pcs-5V-Electromagnetic-Active-Buzzer-Continuous-Beep-Continuously-p-943524.html)
 

## Connections

### Naze32

Connect a supported buzzer directly to the BUZZ pins. Observe polarity.

### CC3D

Buzzer support on the CC3D requires that a buzzer circuit be created to which the input is PA15.
PA15 is unused and not connected according to the CC3D Revision A schematic.
Connecting to PA15 requires careful soldering.

See the [CC3D - buzzer circuir.pdf](Wiring/CC3D - buzzer circuir.pdf) for details.
