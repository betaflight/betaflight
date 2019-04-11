OFFICIAL_TARGETS  = \
    ALIENFLIGHTF3 \
    ALIENFLIGHTF4 \
    ANYFCF7 \
    BETAFLIGHTF3 \
    BLUEJAYF4 \
    FURYF4 REVO \
    SIRINFPV \
    SPARKY \
    SPRACINGF3 \
    SPRACINGF3EVO \
    SPRACINGF3NEO \
    SPRACINGF4EVO \
    SPRACINGF7DUAL \
    STM32F3DISCOVERY

ALT_TARGET_PATHS  = $(filter-out %/target,$(basename $(wildcard $(ROOT)/src/main/target/*/*.mk)))
ALT_TARGET_NAMES  = $(notdir $(ALT_TARGET_PATHS))
BASE_TARGET_NAMES = $(notdir $(patsubst %/,%,$(dir $(ALT_TARGET_PATHS))))
BASE_ALT_PAIRS    = $(join $(BASE_TARGET_NAMES:=/),$(ALT_TARGET_NAMES))

ALT_TARGETS       = $(sort $(notdir $(BASE_ALT_PAIRS)))
BASE_TARGETS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(ROOT)/src/main/target/*/target.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target,$(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.nomk)))))
OPBL_TARGETS      = $(sort $(filter %_OPBL,$(ALT_TARGETS)))

VALID_TARGETS     = $(sort $(filter-out $(NOBUILD_TARGETS),$(BASE_TARGETS) $(ALT_TARGETS)))

# For alt targets, returns their base target name.
# For base targets, returns the (same) target name.
# param $1 = target name
find_target_pair  = $(filter %/$(1),$(BASE_ALT_PAIRS))
get_base_target   = $(if $(call find_target_pair,$(1)),$(patsubst %/,%,$(dir $(call find_target_pair,$(1)))),$(1))

UNSUPPORTED_TARGETS := \
    AFROMINI \
    ALIENFLIGHTF1 \
    BEEBRAIN \
    CC3D \
    CC3D_OPBL \
    CJMCU \
    MICROSCISKY \
    NAZE \
    AIORACERF3 \
    AIR32 \
    AIRHEROF3 \
    ALIENFLIGHTF3 \
    BEEBRAIN_V2D \
    BEEBRAIN_V2F \
    BEESTORM \
    BETAFLIGHTF3 \
    CHEBUZZF3 \
    COLIBRI_RACE \
    CRAZYBEEF3DX \
    CRAZYBEEF3FR \
    CRAZYBEEF3FS \
    DOGE EACHIF3 \
    FF_ACROWHOOPSP \
    FF_KOMBINI \
    FF_PIKOBLX \
    FF_RADIANCE \
    FLIP32F3OSD \
    FRSKYF3 \
    FURYF3 \
    FURYF3OSD \
    IMPULSERCF3 \
    IRCFUSIONF3 \
    IRCSYNERGYF3 \
    ISHAPEDF3 \
    KISSCC \
    KISSFC \
    LUMBAF3 \
    LUXV2_RACE \
    LUX_RACE \
    MIDELICF3 \
    MOTOLAB \
    MULTIFLITEPICO \
    OMNIBUS \
    RACEBASE \
    RCEXPLORERF3 \
    RG_SSD_F3 \
    RMDO \
    SINGULARITY \
    SIRINFPV \
    SPARKY \
    SPRACINGF3 \
    SPRACINGF3EVO \
    SPRACINGF3MINI \
    SPRACINGF3MQ \
    SPRACINGF3NEO \
    STM32F3DISCOVERY \
    TINYBEEF3 \
    TINYFISH \
    X_RACERSPI \
    ZCOREF3

SUPPORTED_TARGETS := $(filter-out $(UNSUPPORTED_TARGETS), $(VALID_TARGETS))

TARGETS_TOTAL := $(words $(SUPPORTED_TARGETS))
TARGET_GROUPS := 5
TARGETS_PER_GROUP := $(shell expr $(TARGETS_TOTAL) / $(TARGET_GROUPS) )

ST := 1
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_1_TARGETS := $(wordlist  $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_2_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_3_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

ST := $(shell expr $(ET) + 1)
ET := $(shell expr $(ST) + $(TARGETS_PER_GROUP))
GROUP_4_TARGETS := $(wordlist $(ST), $(ET), $(SUPPORTED_TARGETS))

GROUP_OTHER_TARGETS := $(filter-out $(GROUP_1_TARGETS) $(GROUP_2_TARGETS) $(GROUP_3_TARGETS) $(GROUP_4_TARGETS), $(SUPPORTED_TARGETS))
