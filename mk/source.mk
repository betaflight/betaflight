PG_SRC = \
            pg/adc.c \
            pg/alt_hold_multirotor.c \
            pg/alt_hold_wing.c \
            pg/autopilot_multirotor.c \
            pg/autopilot_wing.c \
            pg/beeper.c \
            pg/beeper_dev.c \
            pg/board.c \
            pg/bus_i2c.c \
            pg/bus_quadspi.c \
            pg/bus_spi.c \
            pg/dashboard.c \
            pg/displayport_profiles.c \
            pg/dyn_notch.c \
            pg/flash.c \
            pg/gimbal.c \
            pg/gps.c \
            pg/gps_lap_timer.c \
            pg/gps_rescue_multirotor.c \
            pg/gps_rescue_wing.c \
            pg/gyrodev.c \
            pg/max7456.c \
            pg/mco.c \
            pg/motor.c \
            pg/msp.c \
            pg/pg.c \
            pg/pilot.c \
            pg/piniobox.c \
            pg/pinio.c \
            pg/pin_pull_up_down.c \
            pg/pos_hold_multirotor.c \
            pg/pos_hold_wing.c \
            pg/rcdevice.c \
            pg/rpm_filter.c \
            pg/rx.c \
            pg/rx_pwm.c \
            pg/rx_spi.c \
            pg/rx_spi_cc2500.c \
            pg/rx_spi_expresslrs.c \
            pg/scheduler.c \
            pg/sdcard.c \
            pg/sdio.c \
            pg/serial_uart.c \
            pg/stats.c \
            pg/timerio.c \
            pg/timerup.c \
            pg/usb.c \
            pg/vcd.c \
            pg/vtx_io.c \
            pg/vtx_table.c

COMMON_SRC = \
            build/build_config.c \
            build/debug.c \
            build/debug_pin.c \
            build/version.c \
            main.c \
            common/bitarray.c \
            common/chirp.c \
            common/colorconversion.c \
            common/crc.c \
            common/encoding.c \
            common/explog_approx.c \
            common/filter.c \
            common/gps_conversion.c \
            common/huffman.c \
            common/huffman_table.c \
            common/maths.c \
            common/printf.c \
            common/printf_serial.c \
            common/pwl.c \
            common/sdft.c \
            common/sensor_alignment.c \
            common/stopwatch.c \
            common/streambuf.c \
            common/string_light.c \
            common/strtol.c \
            common/time.c \
            common/typeconversion.c \
            common/uvarint.c \
            common/vector.c \
            config/config.c \
            config/config_eeprom.c \
            config/config_streamer.c \
            config/feature.c \
            config/simplified_tuning.c \
            cli/cli.c \
            cli/settings.c \
            drivers/dshot.c \
            drivers/dshot_command.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_i2c_busdev.c \
            drivers/bus_i2c_utils.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_octospi.c \
            drivers/bus_quadspi.c \
            drivers/buttons.c \
            drivers/camera_control.c \
            drivers/display.c \
            drivers/display_canvas.c \
            drivers/dma_common.c \
            drivers/io.c \
            drivers/io_preinit.c \
            drivers/light_led.c \
            drivers/mco.c \
            drivers/motor.c \
            drivers/pinio.c \
            drivers/pin_pull_up_down.c \
            drivers/pwm_output.c \
            drivers/resource.c \
            drivers/serial.c \
            drivers/serial_impl.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/timer_common.c \
            drivers/transponder_ir_arcitimer.c \
            drivers/transponder_ir_ilap.c \
            drivers/transponder_ir_erlt.c \
            fc/board_info.c \
            fc/dispatch.c \
            fc/hardfaults.c \
            fc/tasks.c \
            fc/runtime_config.c \
            fc/stats.c \
            io/beeper.c \
            io/piniobox.c \
            io/serial.c \
            io/serial_resource.c \
            io/smartaudio_protocol.c \
            io/statusindicator.c \
            io/tramp_protocol.c \
            io/transponder_ir.c \
            io/usb_cdc_hid.c \
            io/usb_msc.c \
            msp/msp.c \
            msp/msp_box.c \
            msp/msp_build_info.c \
            msp/msp_serial.c \
            scheduler/scheduler.c \
            sensors/adcinternal.c \
            sensors/battery.c \
            sensors/current.c \
            sensors/voltage.c \
            target/config_helper.c \
            fc/init.c \
            fc/controlrate_profile.c \
            drivers/accgyro/gyro_sync.c \
            drivers/rx/rx_spi.c \
            drivers/rx/rx_xn297.c \
            drivers/rx/rx_pwm.c \
            drivers/serial_softserial.c \
            fc/core.c \
            fc/gps_lap_timer.c \
            fc/rc.c \
            fc/rc_adjustments.c \
            fc/rc_controls.c \
            fc/rc_modes.c \
            flight/alt_hold_multirotor.c \
            flight/alt_hold_wing.c \
            flight/autopilot_multirotor.c \
            flight/autopilot_wing.c \
            flight/dyn_notch_filter.c \
            flight/failsafe.c \
            flight/gps_rescue_multirotor.c \
            flight/gps_rescue_wing.c \
            flight/imu.c \
            flight/mixer.c \
            flight/mixer_init.c \
            flight/mixer_tricopter.c \
            flight/pid.c \
            flight/pid_init.c \
            flight/position.c \
            flight/pos_hold_multirotor.c \
            flight/pos_hold_wing.c \
            flight/rpm_filter.c \
            flight/servos.c \
            flight/servos_tricopter.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/pwm.c \
            rx/frsky_crc.c \
            rx/rc_stats.c \
            rx/rx.c \
            rx/rx_bind.c \
            rx/rx_spi.c \
            rx/rx_spi_common.c \
            rx/crsf.c \
            rx/ghst.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            rx/spektrum.c \
            rx/srxl2.c \
            io/spektrum_vtx_control.c \
            io/spektrum_rssi.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/xbus.c \
            rx/fport.c \
            rx/msp_override.c \
            sensors/acceleration.c \
            sensors/acceleration_init.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/gyro.c \
            sensors/gyro_init.c \
            sensors/initialisation.c \
            blackbox/blackbox.c \
            blackbox/blackbox_encoding.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_failsafe.c \
            cms/cms_menu_firmware.c \
            cms/cms_menu_gps_rescue_multirotor.c \
            cms/cms_menu_gps_rescue_wing.c \
            cms/cms_menu_gps_lap_timer.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_main.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_power.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_common.c \
            cms/cms_menu_vtx_rtc6705.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            cms/cms_menu_persistent_stats.c \
            cms/cms_menu_rpm_limit.c \
            cms/cms_menu_quick.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/rangefinder/rangefinder_hcsr04.c \
            drivers/rangefinder/rangefinder_lidartf.c \
            drivers/rangefinder/rangefinder_lidarmt.c \
            drivers/vtx_common.c \
            drivers/vtx_table.c \
            io/dashboard.c \
            io/displayport_frsky_osd.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/displayport_srxl.c \
            io/displayport_crsf.c \
            io/displayport_hott.c \
            io/frsky_osd.c \
            io/gimbal_control.c \
            io/rcdevice_cam.c \
            io/rcdevice.c \
            io/gps.c \
            io/ledstrip.c \
            io/pidaudio.c \
            osd/osd.c \
            osd/osd_elements.c \
            osd/osd_warnings.c \
            sensors/barometer.c \
            sensors/rangefinder.c \
            sensors/opticalflow.c \
            telemetry/telemetry.c \
            telemetry/crsf.c \
            telemetry/ghst.c \
            telemetry/srxl.c \
            telemetry/frsky_hub.c \
            telemetry/hott.c \
            telemetry/jetiexbus.c \
            telemetry/smartport.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/msp_shared.c \
            telemetry/ibus.c \
            telemetry/ibus_shared.c \
            sensors/esc_sensor.c \
            io/vtx.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c \
            io/vtx_msp.c \
            cms/cms_menu_vtx_msp.c

ifneq ($(SIMULATOR_BUILD),yes)

COMMON_SRC += \
            drivers/bus_spi.c \
            drivers/serial_uart.c \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_bmi160.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/accgyro/accgyro_spi_icm20649.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/accgyro/accgyro_spi_icm456xx.c \
            drivers/accgyro/accgyro_spi_icm40609.c \
            drivers/accgyro/accgyro_spi_l3gd20.c \
            drivers/accgyro/accgyro_spi_lsm6dso.c \
            drivers/accgyro/accgyro_spi_lsm6dso_init.c \
            drivers/accgyro/accgyro_spi_lsm6dsv16x.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/accgyro/accgyro_virtual.c \
            BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
            drivers/barometer/barometer_2smpb_02b.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_bmp388.c \
            drivers/barometer/barometer_dps310.c \
            drivers/barometer/barometer_lps22df.c \
            drivers/barometer/barometer_lps.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_qmp6988.c \
            drivers/barometer/barometer_virtual.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_ist8310.c \
            drivers/compass/compass_lis2mdl.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/compass/compass_mpu925x_ak8963.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_virtual.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c

RX_SRC = \
            drivers/rx/expresslrs_driver.c \
            drivers/rx/rx_a7105.c \
            drivers/rx/rx_cc2500.c \
            drivers/rx/rx_cyrf6936.c \
            drivers/rx/rx_nrf24l01.c \
            drivers/rx/rx_pwm.c \
            drivers/rx/rx_spi.c \
            drivers/rx/rx_sx127x.c \
            drivers/rx/rx_sx1280.c \
            drivers/rx/rx_xn297.c \
            rx/cc2500_common.c \
            rx/cc2500_frsky_shared.c \
            rx/cc2500_frsky_d.c \
            rx/cc2500_frsky_x.c \
            rx/cc2500_sfhss.c \
            rx/cc2500_redpine.c \
            rx/a7105_flysky.c \
            rx/cyrf6936_spektrum.c \
            rx/expresslrs.c \
            rx/expresslrs_common.c \
            rx/expresslrs_telemetry.c

FLASH_SRC += \
            drivers/flash/flash.c \
            drivers/flash/flash_m25p16.c \
            drivers/flash/flash_w25m.c \
            drivers/flash/flash_w25n.c \
            drivers/flash/flash_w25q128fv.c \
            io/flashfs.c

SDCARD_SRC += \
            drivers/sdcard.c \
            drivers/sdcard_spi.c \
            drivers/sdcard_sdio_baremetal.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c

INCLUDE_DIRS += $(FATFS_DIR)
VPATH        := $(VPATH):$(FATFS_DIR)

# Gyro driver files that only contain initialization and configuration code - not runtime code
SIZE_OPTIMISED_SRC += \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/accgyro/accgyro_spi_lsm6dso_init.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_lps.c \
            drivers/barometer/barometer_qmp6988.c \
            drivers/barometer/barometer_2smpb_02b.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_lis2mdl.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/compass/compass_ist8310.c \
            drivers/display_ug2864hsweg01.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/vtx_rtc6705.c


SPEED_OPTIMISED_SRC += \
            drivers/bus_spi.c \
            drivers/serial_uart.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_bmi160.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/accgyro/accgyro_spi_lsm6dso.c

endif

COMMON_DEVICE_SRC = $(CMSIS_SRC) $(DEVICE_STDPERIPH_SRC)

COMMON_SRC += $(CONFIG_SRC) $(PG_SRC) $(COMMON_DEVICE_SRC) $(RX_SRC)

ifeq ($(EXST),yes)
TARGET_FLAGS += -DUSE_EXST
endif

ifeq ($(RAM_BASED),yes)
TARGET_FLAGS += -DUSE_EXST -DCONFIG_IN_RAM -DRAMBASED
endif

ifeq ($(SIMULATOR_BUILD),yes)
TARGET_FLAGS += -DSIMULATOR_BUILD
endif

SPEED_OPTIMISED_SRC += \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/pwl.c \
            common/sdft.c \
            common/stopwatch.c \
            common/typeconversion.c \
            common/vector.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_quadspi.c \
            drivers/io.c \
            drivers/serial.c \
            fc/core.c \
            fc/tasks.c \
            fc/rc.c \
            fc/rc_controls.c \
            fc/runtime_config.c \
            flight/dyn_notch_filter.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/rpm_filter.c \
            rx/ibus.c \
            rx/rc_stats.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/crsf.c \
            rx/frsky_crc.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            rx/spektrum.c \
            rx/srxl2.c \
            rx/sumd.c \
            rx/xbus.c \
            rx/fport.c \
            rx/frsky_crc.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \

SIZE_OPTIMISED_SRC += \
            sensors/gyro_init.c \
            sensors/acceleration_init.c \
            flight/pid_init.c \
            flight/mixer_init.c \
            cli/cli.c \
            cli/settings.c \
            drivers/light_ws2811strip.c \
            drivers/vtx_common.c \
            fc/init.c \
            fc/board_info.c \
            config/config_eeprom.c \
            config/feature.c \
            config/config_streamer.c \
            config/simplified_tuning.c \
            io/dashboard.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/transponder_ir.c \
            io/usb_cdc_hid.c \
            msp/msp_serial.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_failsafe.c \
            cms/cms_menu_firmware.c \
            cms/cms_menu_gps_rescue_multirotor.c \
            cms/cms_menu_gps_rescue_wing.c \
            cms/cms_menu_gps_lap_timer.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_main.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_power.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_common.c \
            cms/cms_menu_vtx_rtc6705.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            cms/cms_menu_persistent_stats.c \
            cms/cms_menu_rpm_limit.c \
            cms/cms_menu_quick.c \
            io/vtx.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c \
            io/spektrum_vtx_control.c \
            osd/osd.c \
            osd/osd_elements.c \
            osd/osd_warnings.c \
            rx/rx_bind.c \
            io/vtx_msp.c \
            cms/cms_menu_vtx_msp.c

# check if target.mk supplied
SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) $(VARIANT_SRC)

# Files that should not be optimized, useful for debugging IMPRECISE cpu faults.
# Specify FULL PATH, e.g. "./lib/main/STM32F7/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c"
NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) \

ifneq ($(DSP_LIB),)

INCLUDE_DIRS += $(DSP_LIB)/Include
SRC += $(wildcard $(DSP_LIB)/Source/*/*.S)

endif

SRC += $(FLASH_SRC) $(MSC_SRC) $(SDCARD_SRC) $(COMMON_SRC)

#excludes
SRC   := $(filter-out $(MCU_EXCLUDES), $(SRC))

SRC += $(VCP_SRC)

# end target specific make file checks

# Search path and source files for the Open Location Code library
OLC_DIR := google/olc

ifneq ($(OLC_DIR),)
INCLUDE_DIRS += $(LIB_MAIN_DIR)/$(OLC_DIR)
SRC += $(OLC_DIR)/olc.c
SIZE_OPTIMISED_SRC += $(OLC_DIR)/olc.c
endif
