# Transponder

Cleanflight supports the generation of race transponder signals on compatiable F3 and F4 targets.

IR led connections is target specific. Please consult the reference manual for your FC.

Cleanflight currently supports 3 tranponder protocol providers (as of 2.10).

## iLap Provider

### Links: 
[Web](http://www.rclapcounter.com/)
[Contact](cs@rclapcounter.com)

### Description:
iLap is a commercial system that uses 6 byte transponder codes and a 460kHz carrier.

Transponder codes are entered in the CF Configurator Transponder tab as 12 hex digits.

Codes are theoretical unique. Codes are obtained for iLap or come with some flight controllers.

<img src="Screenshots/Provider%20iLap.png" width="800">

## ArcTimer Provider

### Links:
[Web](http://www.arcitimer.com)
[Contact](info@arcitimer.com)

### Description:
Arctimer is a commercial system that uses 9 byte transponder codes and a 42kHz carrier.

There are only 9 unique Arctimer codes. Codes are pick from a list on CF Configurator Transponder tab.

<img src="Screenshots/Provider%20ArcTimer.png" width="800">

## EasyRaceLapTimer (ERLT) Provider

### Links: 
[Web](http://www.easyracelaptimer.com/)
[Facebook](https://www.facebook.com/groups/1015588161838713/)
[RCGroups](https://www.rcgroups.com/forums/showthread.php?2538917-EasyRaceLapTimer-open-source-and-open-hardware-FPV-racing-lap-time-tracking-system)
[GitHub](https://github.com/polyvision/EasyRaceLapTimer)

### Description
EasyRaceLapTimer is a open source system that uses 6bit transponder codes and a 38kHz carrier.

There are 64 unique ERLT codes. Codes are pick from a list on CF Configurator Transponder tab.

<img src="Screenshots/Provider%20ERLT.png" width="800">




