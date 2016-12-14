/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * glow.c - Deck driver for the Luxeon high power LED deck
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "pm.h"
#include "log.h"
#include "param.h"

#define GLOW_UPDATE_PERIOD_MS   10
#define GLOW_PWM_FREQ           20000
#define GLOW_TIM_PERIOD
#define GLOW_LED_MAX_AMP        0.50f
#define GLOW_AMP_AT_3p7_VOLT    1.35f
#define GLOW_AMP_AT_4p2_VOLT    1.80f


static uint8_t power;
static uint16_t duty;

static void glowOn(uint8_t newPower)
{
  power = newPower;
}

static void glowOff()
{
  TIM_SetCompare2(TIM3, 0);
  power = 0;
}

void glowTask(void* parameters)
{
  TickType_t xLastWakeTime;
  uint16_t maxDuty;
  float batComp;

  systemWaitStart();

  while (1)
  {
    xLastWakeTime = xTaskGetTickCount();

    // TODO: Safety handling
    batComp = (((4.2f / pmGetBatteryVoltage()) - 1.0) * 4.0) + 1.0;
    maxDuty = ((GLOW_LED_MAX_AMP / GLOW_AMP_AT_3p7_VOLT) * 0xFFFF) * batComp;
    duty = ((uint32_t)power * maxDuty) / 256;

    TIM_SetCompare2(TIM3, duty);

    vTaskDelayUntil(&xLastWakeTime, M2T(GLOW_UPDATE_PERIOD_MS));
  }
}

static void glowInit(DeckInfo *info)
{
  static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  static GPIO_InitTypeDef GPIO_InitStructure;
  static TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOB Configuration: TIM3 Channel 2 as alternate function push-pull */
  // Configure the GPIO PB5 for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Map timer to alternate functions
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);

  DBGMCU_APB1PeriphConfig(DBGMCU_TIM3_STOP, ENABLE);

  TIM_Cmd(TIM3, ENABLE);

  xTaskCreate(glowTask, "glow", configMINIMAL_STACK_SIZE, NULL, 1/*priority*/, NULL);

  glowOff();
  glowOn(20);

}

static const DeckDriver glow_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcGlow",

  .usedPeriph = DECK_USING_TIMER3,
  .usedGpio = DECK_USING_IO_2,

  .init = glowInit,
};

DECK_DRIVER(glow_deck);

LOG_GROUP_START(glow)
LOG_ADD(LOG_UINT16, duty, &duty)
LOG_GROUP_STOP(glow)

PARAM_GROUP_START(glow)
PARAM_ADD(PARAM_UINT8, power, &power)
PARAM_GROUP_STOP(glow)

