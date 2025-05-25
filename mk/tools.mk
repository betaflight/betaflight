###############################################################
#
# Installers for tools
#
# NOTE: These are not tied to the default goals and must be invoked manually
#
# ARM SDK Version: 13.3.Rel1
#
# Release date: July 04, 2024
#
###############################################################

##############################
#
# Check that environmental variables are sane
#
##############################

# Set up ARM (STM32) SDK
# Checked below, Should match the output of $(shell arm-none-eabi-gcc -dumpversion)
# must match arm-none-eabi-gcc-<version> file in arm sdk distribution
GCC_REQUIRED_VERSION ?= 13.3.1

## arm_sdk_install   : Install Arm SDK
.PHONY: arm_sdk_install

# source: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
ifeq ($(OSFAMILY)-$(ARCHFAMILY), linux-x86_64)
  ARM_SDK_URL := https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
  DL_CHECKSUM = 0601a9588bc5b9c99ad2b56133b7f118
else ifeq ($(OSFAMILY)-$(ARCHFAMILY), macosx-x86_64)
  ARM_SDK_URL := https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-darwin-x86_64-arm-none-eabi.tar.xz
  DL_CHECKSUM = 4bb141e44b831635fde4e8139d470f1f
else ifeq ($(OSFAMILY)-$(ARCHFAMILY), macosx-arm64)
  ARM_SDK_URL := https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi.tar.xz
  DL_CHECKSUM = f1c18320bb3121fa89dca11399273f4e
else ifeq ($(OSFAMILY), windows)
  ARM_SDK_URL := https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi.zip
  DL_CHECKSUM = 39d9882ca0eb475e81170ae826c1435d
else
  $(error No toolchain URL defined for $(OSFAMILY)-$(ARCHFAMILY))
endif

ARM_SDK_FILE := $(notdir $(ARM_SDK_URL))
# remove compression suffixes
ARM_SDK_DIR := $(TOOLS_DIR)/$(patsubst %.zip,%, 	\
			    $(patsubst %.tar.xz,%, 	\
			    $(notdir $(ARM_SDK_URL))))

SDK_INSTALL_MARKER := $(ARM_SDK_DIR)/.installed

.PHONY: arm_sdk_version

arm_sdk_version: | $(ARM_SDK_DIR)
	$(V1) $(ARM_SDK_DIR)/bin/arm-none-eabi-gcc --version

# order-only prereq on directory existance:
arm_sdk_install: | $(TOOLS_DIR)
arm_sdk_install: arm_sdk_download $(SDK_INSTALL_MARKER)

$(SDK_INSTALL_MARKER): $(DL_DIR)/$(ARM_SDK_FILE)
        # verify ckecksum first
	@checksum=$$(md5sum "$<" | awk '{print $$1}'); \
	if [ "$$checksum" != "$(DL_CHECKSUM)" ]; then \
		echo "$@ Checksum mismatch! Expected $(DL_CHECKSUM), got $$checksum."; \
		exit 1; \
	fi
ifeq ($(OSFAMILY), windows)
	$(V1) unzip -q -d $(TOOLS_DIR) "$<"
else
        # binary only release so just extract it
	$(V1) tar -C $(TOOLS_DIR) -xf "$<"
endif
	$(V1) touch $(SDK_INSTALL_MARKER)

.PHONY: arm_sdk_download
arm_sdk_download: | $(DL_DIR)
arm_sdk_download: $(DL_DIR)/$(ARM_SDK_FILE)
$(DL_DIR)/$(ARM_SDK_FILE):
        # download the source only if it's newer than what we already have
	$(V1) curl -L -k -o "$@" $(if $(wildcard $@), -z "$@",) "$(ARM_SDK_URL)"

## arm_sdk_clean     : Uninstall Arm SDK
.PHONY: arm_sdk_clean
arm_sdk_clean:
	$(V1) [ ! -d "$(ARM_SDK_DIR)" ] || $(RM) -r $(ARM_SDK_DIR)
	$(V1) [ ! -d "$(DL_DIR)" ] || $(RM) -r $(DL_DIR)

.PHONY: openocd_win_install

openocd_win_install: | $(DL_DIR) $(TOOLS_DIR)
openocd_win_install: OPENOCD_URL  := git://git.code.sf.net/p/openocd/code
openocd_win_install: OPENOCD_REV  := cf1418e9a85013bbf8dbcc2d2e9985695993d9f4
openocd_win_install: OPENOCD_OPTIONS :=

ifeq ($(OPENOCD_FTDI), yes)
openocd_win_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --enable-ft2232_ftd2xx --with-ftd2xx-win32-zipdir=$(FTD2XX_DIR)
endif

openocd_win_install: openocd_win_clean libusb_win_install ftd2xx_install
        # download the source
	@echo " DOWNLOAD     $(OPENOCD_URL) @ $(OPENOCD_REV)"
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_BUILD_DIR)"
	$(V1) git clone --no-checkout $(OPENOCD_URL) "$(DL_DIR)/openocd-build"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git checkout -q $(OPENOCD_REV) ; \
	)

        # apply patches
	@echo " PATCH        $(OPENOCD_BUILD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git apply < $(ROOT_DIR)/flight/Project/OpenOCD/0003-freertos-cm4f-fpu-support.patch ; \
	  git apply < $(ROOT_DIR)/flight/Project/OpenOCD/0004-st-icdi-disable.patch ; \
	)

        # build and install
	@echo " BUILD        $(OPENOCD_WIN_DIR)"
	$(V1) mkdir -p "$(OPENOCD_WIN_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  ./bootstrap ; \
	  ./configure --enable-maintainer-mode --prefix="$(OPENOCD_WIN_DIR)" \
		--build=i686-pc-linux-gnu --host=i586-mingw32msvc \
		CPPFLAGS=-I$(LIBUSB_WIN_DIR)/include \
		LDFLAGS=-L$(LIBUSB_WIN_DIR)/lib/gcc \
		$(OPENOCD_OPTIONS) \
		--disable-werror \
		--enable-stlink ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)

        # delete the extracted source when we're done
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"

.PHONY: openocd_win_clean
openocd_win_clean:
	@echo " CLEAN        $(OPENOCD_WIN_DIR)"
	$(V1) [ ! -d "$(OPENOCD_WIN_DIR)" ] || $(RM) -r "$(OPENOCD_WIN_DIR)"

# Set up openocd tools
OPENOCD_DIR       := $(TOOLS_DIR)/openocd
OPENOCD_WIN_DIR   := $(TOOLS_DIR)/openocd_win
OPENOCD_BUILD_DIR := $(DL_DIR)/openocd-build

.PHONY: openocd_install

openocd_install: | $(DL_DIR) $(TOOLS_DIR)
openocd_install: OPENOCD_URL     := git://git.code.sf.net/p/openocd/code
openocd_install: OPENOCD_TAG     := v0.9.0
openocd_install: OPENOCD_OPTIONS := --enable-maintainer-mode --prefix="$(OPENOCD_DIR)" --enable-buspirate --enable-stlink

ifeq ($(OPENOCD_FTDI), yes)
openocd_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --enable-ftdi
endif

ifeq ($(UNAME), Darwin)
openocd_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --disable-option-checking
endif

openocd_install: openocd_clean
        # download the source
	@echo " DOWNLOAD     $(OPENOCD_URL) @ $(OPENOCD_TAG)"
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_BUILD_DIR)"
	$(V1) git clone --no-checkout $(OPENOCD_URL) "$(OPENOCD_BUILD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git checkout -q tags/$(OPENOCD_TAG) ; \
	)

        # build and install
	@echo " BUILD        $(OPENOCD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  ./bootstrap ; \
	  ./configure  $(OPENOCD_OPTIONS) ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)

        # delete the extracted source when we're done
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"

.PHONY: openocd_clean
openocd_clean:
	@echo " CLEAN        $(OPENOCD_DIR)"
	$(V1) [ ! -d "$(OPENOCD_DIR)" ] || $(RM) -r "$(OPENOCD_DIR)"

STM32FLASH_DIR := $(TOOLS_DIR)/stm32flash

.PHONY: stm32flash_install
stm32flash_install: STM32FLASH_URL := http://stm32flash.googlecode.com/svn/trunk
stm32flash_install: STM32FLASH_REV := 61
stm32flash_install: stm32flash_clean
        # download the source
	@echo " DOWNLOAD     $(STM32FLASH_URL) @ r$(STM32FLASH_REV)"
	$(V1) svn export -q -r "$(STM32FLASH_REV)" "$(STM32FLASH_URL)" "$(STM32FLASH_DIR)"

        # build
	@echo " BUILD        $(STM32FLASH_DIR)"
	$(V1) $(MAKE) --silent -C $(STM32FLASH_DIR) all

.PHONY: stm32flash_clean
stm32flash_clean:
	@echo " CLEAN        $(STM32FLASH_DIR)"
	$(V1) [ ! -d "$(STM32FLASH_DIR)" ] || $(RM) -r "$(STM32FLASH_DIR)"

# Set up uncrustify tools
UNCRUSTIFY_DIR := $(TOOLS_DIR)/uncrustify-0.61
UNCRUSTIFY_BUILD_DIR := $(DL_DIR)/uncrustify

.PHONY: uncrustify_install
uncrustify_install: | $(DL_DIR) $(TOOLS_DIR)
uncrustify_install: UNCRUSTIFY_URL := http://downloads.sourceforge.net/project/uncrustify/uncrustify/uncrustify-0.61/uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_FILE := uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_OPTIONS := prefix=$(UNCRUSTIFY_DIR)
uncrustify_install: uncrustify_clean
ifneq ($(OSFAMILY), windows)
	@echo " DOWNLOAD     $(UNCRUSTIFY_URL)"
	$(V1) curl -L -k -o "$(DL_DIR)/$(UNCRUSTIFY_FILE)" "$(UNCRUSTIFY_URL)"
endif
        # extract the src
	@echo " EXTRACT      $(UNCRUSTIFY_FILE)"
	$(V1) tar -C $(TOOLS_DIR) -xf "$(DL_DIR)/$(UNCRUSTIFY_FILE)"

	@echo " BUILD        $(UNCRUSTIFY_DIR)"
	$(V1) ( \
	  cd $(UNCRUSTIFY_DIR) ; \
	  ./configure --prefix="$(UNCRUSTIFY_DIR)" ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)
	      # delete the extracted source when we're done
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

.PHONY: uncrustify_clean
uncrustify_clean:
	@echo " CLEAN        $(UNCRUSTIFY_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_DIR)"
	@echo " CLEAN        $(UNCRUSTIFY_BUILD_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

# ZIP download URL
zip_install: ZIP_URL  := http://pkgs.fedoraproject.org/repo/pkgs/zip/zip30.tar.gz/7b74551e63f8ee6aab6fbc86676c0d37/zip30.tar.gz

zip_install: ZIP_FILE := $(notdir $(ZIP_URL))

ZIP_DIR = $(TOOLS_DIR)/zip30

# order-only prereq on directory existance:
zip_install : | $(DL_DIR) $(TOOLS_DIR)
zip_install: zip_clean
	$(V1) curl -L -k -o "$(DL_DIR)/$(ZIP_FILE)" "$(ZIP_URL)"
	$(V1) tar --force-local -C $(TOOLS_DIR) -xzf "$(DL_DIR)/$(ZIP_FILE)"
ifneq ($(OSFAMILY), windows)
	$(V1) cd "$(ZIP_DIR)" && $(MAKE) -f unix/Makefile generic_gcc
else
	$(V1) cd "$(ZIP_DIR)" && $(MAKE) -f win32/makefile.gcc
endif

.PHONY: zip_clean
zip_clean:
	$(V1) [ ! -d "$(ZIP_DIR)" ] || $(RM) -rf $(ZIP_DIR)

##############################
#
# Set up paths to tools
#
##############################

ifeq ($(shell [ -d "$(ARM_SDK_DIR)" ] && echo "exists"), exists)
  ARM_SDK_PREFIX := $(ARM_SDK_DIR)/bin/arm-none-eabi-
else ifeq (,$(filter %_install test% clean% %-print checks help configs, $(MAKECMDGOALS)))
  GCC_VERSION = $(shell arm-none-eabi-gcc -dumpversion)
  ifeq ($(GCC_VERSION),)
    $(error **ERROR** arm-none-eabi-gcc not in the PATH. Run 'make arm_sdk_install' to install automatically in the tools folder of this repo)
  else ifneq ($(GCC_VERSION), $(GCC_REQUIRED_VERSION))
    $(error **ERROR** your arm-none-eabi-gcc is '$(GCC_VERSION)', but '$(GCC_REQUIRED_VERSION)' is expected. Override with 'GCC_REQUIRED_VERSION' in mk/local.mk or run 'make arm_sdk_install' to install the right version automatically in the tools folder of this repo)
  endif

  # ARM toolchain is in the path, and the version is what's required.
  ARM_SDK_PREFIX ?= arm-none-eabi-
endif

ifeq ($(shell [ -d "$(ZIP_DIR)" ] && echo "exists"), exists)
  export ZIPBIN := $(ZIP_DIR)/zip
else
  export ZIPBIN := zip
endif

ifeq ($(shell [ -d "$(OPENOCD_DIR)" ] && echo "exists"), exists)
  OPENOCD := $(OPENOCD_DIR)/bin/openocd
else
  # not installed, hope it's in the path...
  OPENOCD ?= openocd
endif

ifeq ($(shell [ -d "$(UNCRUSTIFY_DIR)" ] && echo "exists"), exists)
  UNCRUSTIFY := $(UNCRUSTIFY_DIR)/bin/uncrustify
else
  # not installed, hope it's in the path...
  UNCRUSTIFY ?= uncrustify
endif

# Google Breakpad
DUMP_SYMBOLS_TOOL := $(TOOLS_DIR)/breakpad/$(OSFAMILY)-$(ARCHFAMILY)/dump_syms
BREAKPAD_URL := http://dronin.tracer.nz/tools/breakpad.zip
BREAKPAD_DL_FILE := $(DL_DIR)/$(notdir $(BREAKPAD_URL))
BREAKPAD_DIR := $(TOOLS_DIR)/breakpad

.PHONY: breakpad_install
breakpad_install: | $(DL_DIR) $(TOOLS_DIR)
breakpad_install: breakpad_clean
	@echo " DOWNLOAD     $(BREAKPAD_URL)"
	$(V1) $(V1) curl -L -k -z "$(BREAKPAD_DL_FILE)" -o "$(BREAKPAD_DL_FILE)" "$(BREAKPAD_URL)"
	@echo " EXTRACT      $(notdir $(BREAKPAD_DL_FILE))"
	$(V1) mkdir -p "$(BREAKPAD_DIR)"
	$(V1) unzip -q -d $(BREAKPAD_DIR) "$(BREAKPAD_DL_FILE)"
ifeq ($(OSFAMILY), windows)
	$(V1) ln -s "$(TOOLS_DIR)/breakpad/$(OSFAMILY)-i686" "$(TOOLS_DIR)/breakpad/$(OSFAMILY)-x86_64"
endif

.PHONY: breakpad_clean
breakpad_clean:
	@echo " CLEAN        $(BREAKPAD_DIR)"
	$(V1) [ ! -d "$(BREAKPAD_DIR)" ] || $(RM) -rf $(BREAKPAD_DIR)
	@echo " CLEAN        $(BREAKPAD_DL_FILE)"
	$(V1) $(RM) -f $(BREAKPAD_DL_FILE)

# Raspberry Pi Pico tools
PICOTOOL_REPO   := https://github.com/raspberrypi/picotool.git
PICOTOOL_DL_DIR := $(DL_DIR)/picotool
PICOTOOL_BUILD_DIR := $(PICOTOOL_DL_DIR)/build
PICOTOOL_DIR    := $(TOOLS_DIR)/picotool
PICO_SDK_PATH   ?= $(ROOT_DIR)/lib/main/pico-sdk
PICOTOOL        ?= $(PICOTOOL_DIR)/picotool

ifneq ($(filter picotool_install uf2,$(MAKECMDGOALS)),)
    ifeq ($(wildcard $(PICO_SDK_PATH)/CMakeLists.txt),)
        $(error "PICO_SDK_PATH ($(PICO_SDK_PATH)) does not point to a valid Pico SDK. Please 'make submodules' to hydrate the Pico SDK.")
    endif
endif

ifneq ($(filter uf2,$(MAKECMDGOALS)),)
    ifeq (,$(wildcard $(PICOTOOL)))
        ifeq (,$(shell which picotool 2>/dev/null))
            $(error "picotool not in the PATH or configured. Run 'make picotool_install' to install in the tools folder.")
        else
            PICOTOOL := picotool
        endif
    endif
endif

.PHONY: picotool_install
picotool_install: | $(DL_DIR) $(TOOLS_DIR)
picotool_install: picotool_clean
	@echo "\n CLONE     $(PICOTOOL_REPO)"
	$(V1) git clone --depth 1 $(PICOTOOL_REPO) "$(PICOTOOL_DL_DIR)" || { echo "Failed to clone picotool repository"; exit 1; }
	@echo "\n BUILD      $(PICOTOOL_BUILD_DIR)"
	$(V1) [ -d "$(PICOTOOL_DIR)" ] || mkdir -p $(PICOTOOL_DIR)
	$(V1) [ -d "$(PICOTOOL_BUILD_DIR)" ] || mkdir -p $(PICOTOOL_BUILD_DIR)
	$(V1) cmake -S $(PICOTOOL_DL_DIR) -B $(PICOTOOL_BUILD_DIR) -D PICO_SDK_PATH=$(PICO_SDK_PATH) || { echo "CMake configuration failed"; exit 1; }
	$(V1) $(MAKE) -C $(PICOTOOL_BUILD_DIR) || { echo "picotool build failed"; exit 1; }
	$(V1) cp $(PICOTOOL_BUILD_DIR)/picotool $(PICOTOOL_DIR)/picotool || { echo "Failed to install picotool binary"; exit 1; }
	@echo "\n VERSION:"
	$(V1) $(PICOTOOL_DIR)/picotool version

.PHONY: picotool_clean
picotool_clean:
	@echo " CLEAN        $(PICOTOOL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DIR)" ] || $(RM) -rf $(PICOTOOL_DIR)
	@echo " CLEAN        $(PICOTOOL_DL_DIR)"
	$(V1) [ ! -d "$(PICOTOOL_DL_DIR)" ] || $(RM) -rf $(PICOTOOL_DL_DIR)
