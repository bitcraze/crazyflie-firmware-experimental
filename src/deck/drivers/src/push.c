#include "deck.h"
#include "system.h"
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"

#define DEBUG_MODULE "PUSH"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

static State state = idle;

static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

static uint16_t idUp = 0;
static uint16_t idDown = 0;
static uint16_t idLeft = 0;
static uint16_t idRight = 0;
static uint16_t idFront = 0;
static uint16_t idBack = 0;

static uint16_t up = 0;
static uint16_t down = 0;
static uint16_t front = 0;
static uint16_t back = 0;
static uint16_t left = 0;
static uint16_t right = 0;

static uint16_t up_o = 0;
//static uint16_t down_o = 0;
static uint16_t front_o = 0;
static uint16_t back_o = 0;
static uint16_t left_o = 0;
static uint16_t right_o = 0;

static const float velMax = 1.0f;
static const uint16_t radius = 300;

static const float landing_height = 0.2f;
static const float height_sp = 0.2f;

static float factor = 0;

static float velSide = 0;
static float velFront = 0;
static float height = 0;
static float l_comp = 0;
static float r_comp = 0;
static float f_comp = 0;
static float b_comp = 0;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

static void sequenceTask()
{
  static setpoint_t setpoint;

  systemWaitStart();

  vTaskDelay(M2T(3000));

  idUp = logGetVarId("range", "up");
  idDown = logGetVarId("range", "zrange");
  idLeft = logGetVarId("range", "left");
  idRight = logGetVarId("range", "right");
  idFront = logGetVarId("range", "front");
  idBack = logGetVarId("range", "back");

  factor = velMax/radius;

  //DEBUG_PRINT("%i", idUp);

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    //DEBUG_PRINT(".");
    up = logGetUint(idUp);
    if (state == unlocked) {
      down = logGetUint(idDown);
      left = logGetUint(idLeft);
      right = logGetUint(idRight);
      front = logGetUint(idFront);
      back = logGetUint(idBack);

      left_o = radius - MIN(left, radius);
      right_o = radius - MIN(right, radius);
      l_comp = (-1) * left_o * factor;
      r_comp = right_o * factor;
      velSide = r_comp + l_comp;

      front_o = radius - MIN(front, radius);
      back_o = radius - MIN(back, radius);
      f_comp = (-1) * front_o * factor;
      b_comp = back_o * factor;
      velFront = b_comp + f_comp;

      up_o = radius - MIN(up, radius);
      height = height_sp - up_o/1000.0f;

      /*DEBUG_PRINT("l=%i, r=%i, lo=%f, ro=%f, vel=%f\n", left_o, right_o, l_comp, r_comp, velSide);
      DEBUG_PRINT("f=%i, b=%i, fo=%f, bo=%f, vel=%f\n", front_o, back_o, f_comp, b_comp, velFront);
      DEBUG_PRINT("u=%i, d=%i, height=%f\n", up_o, height);*/

      if (1) {
        setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        commanderSetSetpoint(&setpoint, 3);
      }

      if (height < 0.1f) {
        state = stopping;
        DEBUG_PRINT("X\n");
      }

    } else {

      if (state == stopping && up > stoppedTh) {
        DEBUG_PRINT("%i", up);
        state = idle;
        DEBUG_PRINT("S\n");
      }

      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }

      if (up > unlockThHigh && state == lowUnlock) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }

      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}

static void sequenceInit()
{
  xTaskCreate(sequenceTask, "sequence", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/3, NULL);
}

static bool sequenceTest()
{
  return true;
}

const DeckDriver sequence_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcPush",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = sequenceInit,
  .test = sequenceTest,
};

DECK_DRIVER(sequence_deck);
