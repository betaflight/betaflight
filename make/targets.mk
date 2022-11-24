BASE_TARGETS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(ROOT)/src/main/target/*/target.mk)))))
VALID_TARGETS     = $(sort $(BASE_TARGETS))

CI_TARGETS := $(VALID_TARGETS)

include $(ROOT)/src/main/target/$(TARGET)/target.mk

F4_TARGETS      := $(F405_TARGETS) $(F411_TARGETS) $(F446_TARGETS)
F7_TARGETS      := $(F7X2RE_TARGETS) $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)
G4_TARGETS      := $(G47X_TARGETS)
H7_TARGETS      := $(H743xI_TARGETS) $(H750xB_TARGETS) $(H7A3xI_TARGETS) $(H7A3xIQ_TARGETS) $(H723xG_TARGETS) $(H725xG_TARGETS) $(H730xB_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif

ifeq ($(filter $(TARGET), $(F4_TARGETS) $(F7_TARGETS) $(G4_TARGETS) $(H7_TARGETS) $(SITL_TARGETS)),)
$(error Target '$(TARGET)' has not specified a valid STM group, must be one of F405, F411, F446, F7X2RE, F7X5XE, F7X5XG, F7X5XI, F7X6XG, G47X or H7X3XI. Have you prepared a valid target.mk?)
endif

ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
TARGET_MCU := STM32F4

else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))
TARGET_MCU := STM32F7

else ifeq ($(TARGET),$(filter $(TARGET), $(G4_TARGETS)))
TARGET_MCU := STM32G4

else ifeq ($(TARGET),$(filter $(TARGET), $(H7_TARGETS)))
TARGET_MCU := STM32H7

else ifeq ($(TARGET),$(filter $(TARGET), $(SITL_TARGETS)))
TARGET_MCU := SITL
SIMULATOR_BUILD = yes

else
$(error Unknown target MCU specified.)
endif

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)
