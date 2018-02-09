# CLI parameter <gyro_offset_yaw>

## Introduction
This parameter allows to manually compensate YAW gyro drift over time.  

## Audience
 This parameter is intended for LOS flyers who use the HEADFREE feature of betaflight.  
 It reduces the usage of on sight realignment HEADADJ  
 from 3-6 times to 0-1 during a flight.  

## Warnings
Users with exotic board alignment configurations !!! this feature may not work as intended for your purpose !!!  
The gyro drift depends on many factors, so the results of manual calibration have a big variance  

Factors:  
*  gyro (mpu6000 ICM260X ...)
*  sampling rate
*  temperature
*  LiPo (3S ... 5S)
*  Flight style
*  Flight time
*  and many more ...

## Setup
The basic setup can be done on the bench without LiPo, USB powered  
power on, do not move the quad for 10 minutes  
on CW  drift 70° --> gyro_offset_yaw = 70  
on CCW drift 70° --> gyro_offset_yaw = -70  
This is only a coarse adjustment.  

Fine tuning requires real flights and some fingertip.  
Basically you have to measure the drift at the end of the flight,  
and modify the parameter accordingly.  

## Wishes
Happy flight !  
