
# enable app support
APP=1
APP_STACKSIZE=300

## Weight of the Crazyflie + Qi + Lighthouse deck
CFLAGS += -DCF_MASS=0.036f

## Force lighthouse 2
CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2

# CFLAGS += -DBUILD_SNIFFER
# DEBUG=1


VPATH += src/
PROJ_OBJ += bam_2021_autonomous_swarm_app.o pilot.o tower.o sniffer.o

CRAZYFLIE_BASE=../../..
include $(CRAZYFLIE_BASE)/Makefile
