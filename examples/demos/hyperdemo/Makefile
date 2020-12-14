
## Weight of the Bolt Crazyflie + Qi + Lighthouse deck
CFLAGS += -DCF_MASS=0.1f

## To build with lighthouse support
CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0

## Force lighthouse 2
CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2

# CLOAD_CMDS = -w radio://0/30/2M
# DEBUG=1

CFLAGS += -DLOCODECK_USE_ALT_PINS
CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4
CFLAGS += -DLPS_FULL_TX_POWER

# Bolt show drone compile flags
#--------------------------------------------------
#CFLAGS += -DENABLE_ONESHOT125
#CFLAGS += -DSTART_DISARMED
#CFLAGS += -DDEFAULT_IDLE_THRUST=5000

#-- Bolt specific tuning setting. Change in pid.h
#--------------------------------------------------
#define PID_ROLL_RATE_KP  70.0
#define PID_ROLL_RATE_KI  200.0
#define PID_ROLL_RATE_KD  2.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  70.0
#define PID_PITCH_RATE_KI  200.0
#define PID_PITCH_RATE_KD  2.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  120.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  7.0
#define PID_ROLL_KI  3.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  7.0
#define PID_PITCH_KI  3.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0
#--------------------------------------------------

# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += hyperdemo.o

CRAZYFLIE_BASE=../../..
include $(CRAZYFLIE_BASE)/Makefile
