# Building in windows

1- Install cygwin, and GNU Tools ARM Embedded (make sure make is installed).
cygwin: https://cygwin.com/install.html
gnu tools arm embedded: https://launchpad.net/gcc-arm-embedded

2- In a terminal go to cleanflight directory

3- make clean

4- rm -rf obj

5- make

This should work.Something like this output should be the result: 
C:\Users\nico\Documents\GitHub\cleanflight>make
%% startup_stm32f10x_md_gcc.s
%% accgyro_adxl345.c
%% accgyro_bma280.c
%% accgyro_l3g4200d.c
%% accgyro_mma845x.c
%% accgyro_mpu3050.c
%% accgyro_mpu6050.c
%% adc_common.c
%% adc_stm32f10x.c
%% barometer_bmp085.c
%% barometer_ms5611.c
%% bus_spi.c
%% bus_i2c_stm32f10x.c
%% compass_hmc5883l.c
%% gpio_stm32f10x.c
%% light_ledring.c
%% sonar_hcsr04.c
%% pwm_mapping.c
%% pwm_output.c
%% pwm_rssi.c
%% pwm_rx.c
%% serial_softserial.c
%% serial_uart_common.c
%% serial_uart_stm32f10x.c
%% timer_common.c
%% build_config.c
%% battery.c
%% boardalignment.c
%% beeper.c
%% config.c
%% maths.c
%% printf.c
%% typeconversion.c
%% failsafe.c
%% main.c
%% mw.c
%% sensors_acceleration.c
%% sensors_barometer.c
%% sensors_compass.c
%% sensors_gyro.c
%% sensors_initialisation.c
%% sensors_sonar.c
%% bus_i2c_soft.c
%% serial_common.c
%% sound_beeper.c
%% system_common.c
%% flight_common.c
%% flight_imu.c
%% flight_mixer.c
%% gps_common.c
%% runtime_config.c
%% rc_controls.c
%% rc_curves.c
%% rx_common.c
%% rx_msp.c
%% rx_pwm.c
%% rx_sbus.c
%% rx_sumd.c
%% rx_spektrum.c
%% telemetry_common.c
%% telemetry_frsky.c
%% telemetry_hott.c
%% serial_common.c
%% serial_cli.c
%% serial_msp.c
%% statusindicator.c
%% core_cm3.c
%% system_stm32f10x.c
%% stm32f10x_gpio.c
%% stm32f10x_fsmc.c
%% stm32f10x_exti.c
%% stm32f10x_bkp.c
%% stm32f10x_spi.c
%% stm32f10x_adc.c
%% stm32f10x_iwdg.c
%% misc.c
%% stm32f10x_cec.c
%% stm32f10x_wwdg.c
%% stm32f10x_tim.c
%% stm32f10x_usart.c
%% stm32f10x_crc.c
%% stm32f10x_flash.c
%% stm32f10x_rcc.c
%% stm32f10x_sdio.c
%% stm32f10x_i2c.c
%% stm32f10x_pwr.c
%% stm32f10x_rtc.c
%% stm32f10x_can.c
%% stm32f10x_dac.c
%% stm32f10x_dbgmcu.c
%% stm32f10x_dma.c
arm-none-eabi-gcc -o obj/cleanflight_NAZE.elf obj/NAZE/startup_stm32f10x_md_gcc.
o obj/NAZE/drivers/accgyro_adxl345.o obj/NAZE/drivers/accgyro_bma280.o obj/NAZE/
drivers/accgyro_l3g4200d.o obj/NAZE/drivers/accgyro_mma845x.o obj/NAZE/drivers/a
ccgyro_mpu3050.o obj/NAZE/drivers/accgyro_mpu6050.o obj/NAZE/drivers/adc_common.
o obj/NAZE/drivers/adc_stm32f10x.o obj/NAZE/drivers/barometer_bmp085.o obj/NAZE/
drivers/barometer_ms5611.o obj/NAZE/drivers/bus_spi.o obj/NAZE/drivers/bus_i2c_s
tm32f10x.o obj/NAZE/drivers/compass_hmc5883l.o obj/NAZE/drivers/gpio_stm32f10x.o
 obj/NAZE/drivers/light_ledring.o obj/NAZE/drivers/sonar_hcsr04.o obj/NAZE/drive
rs/pwm_mapping.o obj/NAZE/drivers/pwm_output.o obj/NAZE/drivers/pwm_rssi.o obj/N
AZE/drivers/pwm_rx.o obj/NAZE/drivers/serial_softserial.o obj/NAZE/drivers/seria
l_uart_common.o obj/NAZE/drivers/serial_uart_stm32f10x.o obj/NAZE/drivers/timer_
common.o obj/NAZE/build_config.o obj/NAZE/battery.o obj/NAZE/boardalignment.o ob
j/NAZE/beeper.o obj/NAZE/config.o obj/NAZE/common/maths.o obj/NAZE/common/printf
.o obj/NAZE/common/typeconversion.o obj/NAZE/failsafe.o obj/NAZE/main.o obj/NAZE
/mw.o obj/NAZE/sensors_acceleration.o obj/NAZE/sensors_barometer.o obj/NAZE/sens
ors_compass.o obj/NAZE/sensors_gyro.o obj/NAZE/sensors_initialisation.o obj/NAZE
/sensors_sonar.o obj/NAZE/drivers/bus_i2c_soft.o obj/NAZE/drivers/serial_common.
o obj/NAZE/drivers/sound_beeper.o obj/NAZE/drivers/system_common.o obj/NAZE/flig
ht_common.o obj/NAZE/flight_imu.o obj/NAZE/flight_mixer.o obj/NAZE/gps_common.o
obj/NAZE/runtime_config.o obj/NAZE/rc_controls.o obj/NAZE/rc_curves.o obj/NAZE/r
x_common.o obj/NAZE/rx_msp.o obj/NAZE/rx_pwm.o obj/NAZE/rx_sbus.o obj/NAZE/rx_su
md.o obj/NAZE/rx_spektrum.o obj/NAZE/telemetry_common.o obj/NAZE/telemetry_frsky
.o obj/NAZE/telemetry_hott.o obj/NAZE/serial_common.o obj/NAZE/serial_cli.o obj/
NAZE/serial_msp.o obj/NAZE/statusindicator.o obj/NAZE/core_cm3.o obj/NAZE/system
_stm32f10x.o obj/NAZE/stm32f10x_gpio.o obj/NAZE/stm32f10x_fsmc.o obj/NAZE/stm32f
10x_exti.o obj/NAZE/stm32f10x_bkp.o obj/NAZE/stm32f10x_spi.o obj/NAZE/stm32f10x_
adc.o obj/NAZE/stm32f10x_iwdg.o obj/NAZE/misc.o obj/NAZE/stm32f10x_cec.o obj/NAZ
E/stm32f10x_wwdg.o obj/NAZE/stm32f10x_tim.o obj/NAZE/stm32f10x_usart.o obj/NAZE/
stm32f10x_crc.o obj/NAZE/stm32f10x_flash.o obj/NAZE/stm32f10x_rcc.o obj/NAZE/stm
32f10x_sdio.o obj/NAZE/stm32f10x_i2c.o obj/NAZE/stm32f10x_pwr.o obj/NAZE/stm32f1
0x_rtc.o obj/NAZE/stm32f10x_can.o obj/NAZE/stm32f10x_dac.o obj/NAZE/stm32f10x_db
gmcu.o obj/NAZE/stm32f10x_dma.o -lm -mthumb -mcpu=cortex-m3 -static -Wl,-gc-sect
ions,-Map,.//obj/cleanflight_NAZE.map -T.//stm32_flash_f103.ld
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/cleanflight_NAZE.elf obj
/cleanflight_NAZE.hex

cleanflight_NAZE.hex is about 220kb. 
