CFLAGS += -DCPU_MIMXRT1011DAE5A -DCFG_EXAMPLE_VIDEO_READONLY
MCU_VARIANT = MIMXRT1011

# LD file with uf2
LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# For flash-jlink target
JLINK_DEVICE = MIMXRT1011xxx5A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1010

# flash using pyocd
flash: flash-uf2
flash-uf2: $(BUILD)/$(PROJECT).uf2
	@echo copying $<
	@$(CP) $< /media/$(USER)/METROM7BOOT
