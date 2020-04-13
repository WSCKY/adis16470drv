################################################################################
# Project Makefile
# kyChu@2020-3-9
################################################################################

# Project Name
ProjName = adis_imu

# TOP path
TopPath = $(shell pwd)
# BUILD path
BuildPath = $(TopPath)/build
# Output path
OutPath = $(TopPath)/output

#Platform
PLATFORM = 

# user definitions
-include .config

# Include Dirs
INCS = -I$(TopPath)/apps \
       -I$(TopPath)/adis \
       -I$(TopPath)/kylink \
       -I$(TopPath)/configs

LD_INCS = 

# get version from git
__VERSION_STR__ := $(shell git rev-parse --abbrev-ref HEAD):$(shell git rev-parse --short HEAD)

# Global definitions
DEFS = -D__VERSION_STR__=$(__VERSION_STR__)

# External Library path
LIB_PATH = -L$(BuildPath)/kylink -L$(BuildPath)/sdk

# External Libraries
LIBS := -lpthread -lkylink -lADIS16470DRV

# Other configuration
CFGS = -c -fPIC
LD_CFGS = -Wl,--hash-style=sysv,-Bsymbolic,-rpath=$(BuildPath)/sdk

ifeq ($(D), 1)
# Debug information
DBGS = -DDEBUG -g3
else
DBGS =
endif

# Set verbosity
ifeq ($(V), 1)
Q =
else
Q = @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

# Commands
CC      := $(Q)gcc
AS      := $(Q)as
LD      := $(Q)ld
AR      := $(Q)ar
CPP     := $(Q)cpp
SIZE    := $(Q)size
STRIP   := $(Q)strip -s
OBJCOPY := $(Q)objcopy
OBJDUMP := $(Q)objdump
OPENOCD ?= $(Q)openocd
RM      := $(Q)rm -rf
MKDIR   := $(Q)mkdir
ECHO    := $(Q)@echo
PRINTF  := $(Q)@printf
MAKE    := $(Q)make
JAVA    := $(Q)java
CAT     := $(Q)cat

# Export Commands
# -Operate
export Q
export CC
export AS
export LD
export AR
export SIZE
export OBJCOPY
export OBJDUMP
export MKDIR
export ECHO
export MAKE
# -Variables
export INCS
export DBGS
export CFGS
export DEFS
export PLATFORM
export TopPath

# All Target
all: $(ProjName) | $(BuildPath)
sdk: libraries

# All of the sources participating in the build are defined here
-include apps/subdir.mk

# Add inputs and outputs from these tool invocations to the build variables 

$(BuildPath):
	$(MKDIR) -p $@

$(OutPath):
	$(MKDIR) -p $@

$(OBJ_DIRS):
	$(MKDIR) -p $@

# generate target
$(ProjName): libraries $(OBJS) | $(OutPath)
	@echo ''
	@echo ' Building $@'
	$(ECHO) $(OBJS) > $(OutPath)/objects.list
	$(CC) $(PLATFORM) $(LIB_PATH) $(LD_CFGS) -Wl,-Map=$(OutPath)/output.map -Wl,--gc-sections -o $(OutPath)/$(ProjName) @$(OutPath)/objects.list $(LIBS) -lm
	$(ECHO) ' Completed.'

# Other Targets
clean:
	-$(RM) $(BuildPath)/ $(OutPath)/
	-$(ECHO) 'Cleaned.'

libraries:
	$(MAKE) -C kylink/ BUILD=$(BuildPath)/kylink
	$(MAKE) -C adis/ BUILD=$(BuildPath)/sdk

.PHONY: all clean
.SECONDARY: libraries

