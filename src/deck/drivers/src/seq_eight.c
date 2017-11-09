#include <string.h>

#include "deck.h"
#include "system.h"
#include "commander.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "sound.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#define DEBUG_MODULE "SEQ"

extern bool resetEstimation;
extern uint16_t range_last;

static bool badSituation() {
  if (range_last > 400) {
    return true;
  }
  return false;
}

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

static bool isFrontPressed() {
  //DEBUG_PRINT("%X", digitalRead(DECK_GPIO_IO2));
  return digitalRead(DECK_GPIO_IO2) == 0;
}

/*static bool isBackPressed() {
  return digitalRead(DECK_GPIO_IO3) == 0;
}*/

static void sequenceTask()
{
  static setpoint_t setpoint;
  int i = 0;
  //float height = 0.2

  systemWaitStart();

  while(1) {
    DEBUG_PRINT("Waiting for btn press...\n");

    soundSetEffect(0);

    while (!isFrontPressed()) {
      vTaskDelay(50);
    }

    DEBUG_PRINT("Starting sequence!\n");
    soundSetEffect(13);

    // Wait so it doesn't take off into your hand...
    resetEstimation = true;
    vTaskDelay(100);
    resetEstimation = false;
    vTaskDelay(2000);

    soundSetEffect(0);


      for (i=0; i<5 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.2, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, (i*(0.1/10)) + 0.2, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<30 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.3, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        //setHoverSetpoint(&setpoint, 0.5, 0, 0.4, -72);
        setHoverSetpoint(&setpoint, 0, 0.3, 0.3, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.3, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<90 && !badSituation(); i++) {
        //setHoverSetpoint(&setpoint, 0.5, 0, 0.4, 72);
        setHoverSetpoint(&setpoint, 0.21, 0, 0.3, 40);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.3, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        //setHoverSetpoint(&setpoint, 0.5, 0, 0.4, -72);
        setHoverSetpoint(&setpoint, 0, -0.3, 0.3, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      for (i=0; i<10 && !badSituation(); i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.3 - (i*(0.2/10)), 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }

      /*for (i=0; i<5; i++) {
        setHoverSetpoint(&setpoint, 0, 0, 0.1, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
      }*/
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 3);
  }
}

static void sequenceInit()
{
  pinMode(DECK_GPIO_IO2, INPUT_PULLUP);

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
