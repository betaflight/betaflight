# shared.mk
#
# environment variables common to all operating systems supported by the make system

# Make sure we know a few things about the architecture
UNAME := $(shell uname)
ARCH := $(shell uname -m)
ifneq (,$(filter $(ARCH), x86_64 amd64))
  X86-64 := 1
  X86_64 := 1
  AMD64 := 1
  ARCHFAMILY := x86_64
else
  ARCHFAMILY := $(ARCH)
endif

# configure some variables dependent upon what type of system this is

# Linux
ifeq ($(UNAME), Linux)
  OSFAMILY := linux
endif

# Mac OSX
ifeq ($(UNAME), Darwin)
  OSFAMILY := macosx
endif

# Detect all Windows-based POSIX environments (MinGW, MSYS, Cygwin)
ifneq (,$(filter MINGW% MSYS% CYGWIN%,$(UNAME)))
    OSFAMILY := windows
    
    # Set specific environment flags if needed
    ifneq (,$(filter CYGWIN%,$(UNAME)))
        CYGWIN := 1
    else
        MINGW := 1
    endif
endif

# report an error if we couldn't work out what OS this is running on
ifndef OSFAMILY
  $(info uname reports $(UNAME))
  $(info uname -m reports $(ARCH))
  $(error failed to detect operating system)
endif
