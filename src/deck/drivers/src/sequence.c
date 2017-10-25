#include "deck.h"
#include "system.h"
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#define DEBUG_MODULE "SEQ"

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

static void sequenceTask()
{
  static setpoint_t setpoint;
  int i = 0;

  systemWaitStart();

  vTaskDelay(M2T(3000));
  DEBUG_PRINT("Starting sequence ...\n");

  for (i=0; i<20; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.2, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<20; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.4, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<50; i++) {
    setHoverSetpoint(&setpoint, 0.5, 0, 0.4, 72);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<50; i++) {
    setHoverSetpoint(&setpoint, 0.5, 0, 0.4, -72);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<30; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.4, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }

  for (i=0; i<30; i++) {
    setHoverSetpoint(&setpoint, 0, 0, 0.2, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(100));
  }
  
  while(1) {
    vTaskDelay(1000);
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
  .name = "bcSequence",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = sequenceInit,
  .test = sequenceTest,
};

DECK_DRIVER(sequence_deck);
