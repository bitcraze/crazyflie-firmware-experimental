# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../../..

#
# To include header files from the directory
#
EXTRA_CFLAGS += -I$(PWD)/src/common_files

#
# Setting  CONFIG_PARAM_SILENT_UPDATES will prevent the firmware from sending 
# an update message to the cfclient which is only useful when there is communication between them
#  
EXTRA_CFLAGS += -DCONFIG_PARAM_SILENT_UPDATES


#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
BRUSHLESS ?= 0

ifeq ($(BRUSHLESS),1)
    OOT_CONFIG := $(PWD)/app-config-brushless
else
    OOT_CONFIG := $(PWD)/app-config
endif

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
