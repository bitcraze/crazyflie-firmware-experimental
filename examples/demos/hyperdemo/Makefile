
## Weight of the Crazyflie + Qi + Lighthouse deck
CFLAGS += -DCF_MASS=0.037f

## To build with lighthouse support
CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0

## Force lighthouse 2
CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2

# CLOAD_CMDS = -w radio://0/30/2M
# DEBUG=1

CFLAGS += -DLOCODECK_USE_ALT_PINS
CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4
CFLAGS += -DLPS_FULL_TX_POWER




# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += hyperdemo.o

CRAZYFLIE_BASE=../../..
include $(CRAZYFLIE_BASE)/Makefile
