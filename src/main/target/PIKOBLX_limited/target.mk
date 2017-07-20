F3_TARGETS  += $(TARGET)
FEATURES    = VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/serial_softserial.c
			
# seth neiman dec06.16 porting inav to PIKO BLX using betaflight target related files from before timer.h/timer_def.h changes
# current inav does not have transponder related files - removing from build
#            drivers/transponder_ir.c \
#            drivers/transponder_ir_stm32f30x.c \
#            io/transponder_ir.c
