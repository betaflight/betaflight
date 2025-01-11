#DEPS_SUBMODULES +=

UF2_FAMILY_ID_esp32s2 = 0xbfdd4eee
UF2_FAMILY_ID_esp32s3 = 0xc47e5767

BOARD_CMAKE := $(file < $(TOP)/$(BOARD_PATH)/board.cmake)
ifneq ($(findstring esp32s2,$(BOARD_CMAKE)),)
	IDF_TARGET = esp32s2
else
ifneq ($(findstring esp32s3,$(BOARD_CMAKE)),)
	IDF_TARGET = esp32s3
endif
endif

.PHONY: all clean flash bootloader-flash app-flash erase monitor dfu-flash dfu

all:
	idf.py -B$(BUILD) -DFAMILY=$(FAMILY) -DBOARD=$(BOARD) $(CMAKE_DEFSYM) build

build: all

fullclean:
	if test -f sdkconfig; then $(RM) -f sdkconfig ; fi
	if test -d $(BUILD); then $(RM) -rf $(BUILD) ; fi
	idf.py -B$(BUILD) -DFAMILY=$(FAMILY) -DBOARD=$(BOARD) $(CMAKE_DEFSYM) $@

clean flash bootloader-flash app-flash erase monitor dfu-flash dfu size size-components size-files:
	idf.py -B$(BUILD) -DFAMILY=$(FAMILY) -DBOARD=$(BOARD) $(CMAKE_DEFSYM) $@

uf2: $(BUILD)/$(PROJECT).uf2

$(BUILD)/$(PROJECT).uf2: $(BUILD)/$(PROJECT).bin
	@echo CREATE $@
	$(PYTHON) $(TOP)/tools/uf2/utils/uf2conv.py -f $(UF2_FAMILY_ID_$(IDF_TARGET)) -b 0x0 -c -o $@ $^
