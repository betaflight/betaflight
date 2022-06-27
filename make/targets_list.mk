ALT_TARGET_PATHS  = $(filter-out %/target,$(basename $(wildcard $(ROOT)/src/main/target/*/*.mk)))
ALT_TARGET_NAMES  = $(notdir $(ALT_TARGET_PATHS))
BASE_TARGET_NAMES = $(notdir $(patsubst %/,%,$(dir $(ALT_TARGET_PATHS))))
BASE_ALT_PAIRS    = $(join $(BASE_TARGET_NAMES:=/),$(ALT_TARGET_NAMES))

ALT_TARGETS       = $(sort $(notdir $(BASE_ALT_PAIRS)))
BASE_TARGETS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(ROOT)/src/main/target/*/target.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target,$(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.nomk)))))
VALID_TARGETS     = $(sort $(filter-out $(NOBUILD_TARGETS),$(BASE_TARGETS) $(ALT_TARGETS)))

# For alt targets, returns their base target name.
# For base targets, returns the (same) target name.
# param $1 = target name
find_target_pair  = $(filter %/$(1),$(BASE_ALT_PAIRS))
get_base_target   = $(if $(call find_target_pair,$(1)),$(patsubst %/,%,$(dir $(call find_target_pair,$(1)))),$(1))

UNSUPPORTED_TARGETS := \
    AFROMINI \
    BEEBRAIN \
    CJMCU \
    MICROSCISKY \
    AIR32 \
    BEEBRAIN_V2D \
    BEEBRAIN_V2F \
    BEESTORM \
    COLIBRI_RACE \
    FF_ACROWHOOPSP \
    FF_KOMBINI \
    FF_PIKOBLX \
    FF_RADIANCE \
    KISSCC \
    KISSFC \
    LUXV2_RACE \
    LUX_RACE \
    MOTOLAB \
    MULTIFLITEPICO \
    OMNIBUS \
    RACEBASE \
    RMDO \
    SINGULARITY \
    SIRINFPV \
    SPARKY \
    TINYFISH \
    X_RACERSPI

UNIFIED_TARGETS := STM32F405 \
	STM32F411 \
	STM32F7X2 \
	STM32F745 \
	STM32G47X \
	STM32H743 \
	STM32F411SX1280

# Legacy targets are targets that have been replaced by Unified Target configurations
LEGACY_TARGETS := MATEKF405 \
    AIKONF4 \
    ALIENFLIGHTF4 \
    BEEROTORF4 \
    CLRACINGF4 \
    CRAZYBEEF4FR \
    DYSF4PRO \
    ELINF405 \
    FF_RACEPIT \
    FLYWOOF405 \
    FLYWOOF411 \
    FURYF4OSD \
    HAKRCF411 \
    KAKUTEF4V2 \
    MAMBAF411 \
    MATEKF411 \
    MATEKF411RX \
    MERAKRCF405 \
    MLTEMPF4 \
    MLTYPHF4 \
    OMNIBUSF4 \
    OMNIBUSF4FW \
    OMNIBUSF4NANOV7 \
    OMNIBUSF4SD \
    OMNIBUSF4V6 \
    SPEEDYBEEF4 \
    SYNERGYF4 \
    TMOTORF4 \
    TRANSTECF411 \
    VGOODRCF4 \
    XILOF4 \
    AIRBOTF7 \
    AIRF7 \
    ALIENFLIGHTNGF7 \
    CLRACINGF7 \
    ELINF722 \
    EXF722DUAL \
    FLYWOOF7DUAL \
    FOXEERF722DUAL \
    FPVM_BETAFLIGHTF7 \
    JHEF7DUAL \
    KAKUTEF7 \
    KAKUTEF7MINI \
    LUXMINIF7 \
    MAMBAF722 \
    MATEKF722 \
    MATEKF722SE \
    MERAKRCF722 \
    NERO \
    OMNIBUSF7NANOV7 \
    OMNIBUSF7V2 \
    RUSHCORE7 \
    SPEEDYBEEF7 \
    SPRACINGF7DUAL \
    TMOTORF7 \
    TRANSTECF7 \
    AIRBOTF4 \
    AIRBOTF4SD \
    BLUEJAYF4 \
    CRAZYBEEF4DX \
    CRAZYBEEF4FS \
    DALRCF405 \
    FOXEERF405 \
    HAKRCF405 \
    KAKUTEF4 \
    NOX \
    OMNINXT4 \
    REVO \
    DALRCF722DUAL \
    OMNINXT7 \
    BETAFLIGHTF4 \
    EXUAVF4PRO \
    FRSKYF4 \
    KIWIF4 \
    KIWIF4V2 \
    PLUMF4 \
    SKYZONEF405 \
    XRACERF4 \
    AG3XF7 \
    YUPIF7 \
    PYRODRONEF4 \
    AG3XF4 \
    COLIBRI \
    ELLE0 \
    F4BY \
    FF_FORTINIF4 \
    FF_FORTINIF4_REV03 \
    FF_PIKOF4 \
    FF_PIKOF4OSD \
    FURYF4 \
    LUXF4OSD \
    REVOLT \
    REVOLTOSD \
    REVONANO \
    SOULF4 \
    SPARKY2 \
    SPRACINGF4EVO \
    SPRACINGF4NEO \
    STM32F411DISCOVERY \
    UAVPNG030MINI \
    WORMFC \
    YUPIF4 \
    ANYFCF7 \
    ANYFCM7 \
    HAKRCF722 \
    KAKUTEF7V2 \
    NUCLEOF722 \
    OMNIBUSF7 \
    ALIENWHOOPF4 \
    FISHDRONEF4 \
    PIRXF4 \
    PODIUMF4 \
    STACKX \
    VRRACE \
    KROOZX

# Temporarily excluded to get CI coverage for USE_SPI_TRANSACTION
#    STM32F4DISCOVERY \

CI_TARGETS := $(filter-out $(LEGACY_TARGETS) $(UNSUPPORTED_TARGETS), $(VALID_TARGETS))

TARGETS_TOTAL := $(words $(CI_TARGETS))
TARGET_GROUPS := 3
TARGETS_PER_GROUP := $(shell expr $(TARGETS_TOTAL) / $(TARGET_GROUPS) )

ST := 1
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_1_TARGETS := $(wordlist  $(ST), $(ET), $(CI_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_2_TARGETS := $(wordlist $(ST), $(ET), $(CI_TARGETS))

GROUP_OTHER_TARGETS := $(filter-out $(GROUP_1_TARGETS) $(GROUP_2_TARGETS), $(CI_TARGETS))
