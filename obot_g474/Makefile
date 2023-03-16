
######################################
# target
######################################

$(shell touch version.h)

CONFIG=osa14
CONFIG_FILE = config/config_obot_g474_$(CONFIG).cpp
ifndef PARAM
PARAM=$(CONFIG)
endif
PARAM_FILE = param/param_obot_g474_$(PARAM).c


TARGET = $(CONFIG)

TARGET_MCU := stm32g474

# for submake
export

include ../motorlib/scripts/configure_gcc.mk
include ../motorlib/scripts/configure.mk
include ../motorlib/scripts/build.mk

