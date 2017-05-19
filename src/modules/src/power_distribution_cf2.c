/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution_cf2.c - Crazyflie 2.0 model-based power distribution code
 */

#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"

#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"
#include "physical_constants.h"

#include "motors.h"

float SHUTOFF_THRUST = 4.47e-3; // this would cause all motors to rotate with cmd 1000, too slow.

static float motor_pwm[4] = {0};
static float RollForce, PitchForce, YawForce, ThrustForce;

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

static uint32_t lastEnableTime = 0;
static bool enabled = false;

void powerDistribution(const control_t *control)
{
  // if the motors should be disabled, disable them
  if (!control->enable || control->thrust * CRAZYFLIE_MASS < SHUTOFF_THRUST) {
    lastEnableTime = xTaskGetTickCount();
    enabled = false;
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    return;
  }

  // if the motors should be enabled after being disabled, spin them up first, this reduces the likelihood of flips on the ground
  if (!enabled && xTaskGetTickCount() - lastEnableTime < M2T(200)) {
    motorsSetRatio(MOTOR_M1, 10000);
    motorsSetRatio(MOTOR_M2, 10000);
    motorsSetRatio(MOTOR_M3, 10000);
    motorsSetRatio(MOTOR_M4, 10000);
    return;
  }

  // otherwise enable the motors and calculate required commands
  enabled = true;

  float motor_forces[4] = {0};
  ThrustForce = control->thrust * CRAZYFLIE_MASS; // force to provide control->thrust
  YawForce = control->torque[2] / THRUST_TO_TORQUE_m; // force to provide z torque

  #ifdef QUAD_FORMATION_X
  RollForce =  control->torque[0] / (CRAZYFLIE_ARM_LENGTH * .707106781f); // force to provide x torque
  PitchForce = control->torque[1] / (CRAZYFLIE_ARM_LENGTH * .707106781f); // force to provide y torque

  motor_forces[0] = ThrustForce/4.0f - RollForce/4.0f - PitchForce/4.0f - YawForce/4.0f;
  motor_forces[1] = ThrustForce/4.0f - RollForce/4.0f + PitchForce/4.0f + YawForce/4.0f;
  motor_forces[2] = ThrustForce/4.0f + RollForce/4.0f + PitchForce/4.0f - YawForce/4.0f;
  motor_forces[3] = ThrustForce/4.0f + RollForce/4.0f - PitchForce/4.0f + YawForce/4.0f;

  #else // TODO: QUAD_FORMATION_NORMAL

  #endif

  for (int i=0; i<4; i++) {
    if (motor_forces[i] < 0) {
      motor_forces[i] = 0;
      motor_pwm[i] = 0;
    } else {
      motor_pwm[i] = (-PWM_TO_THRUST_b + arm_sqrt(PWM_TO_THRUST_b*PWM_TO_THRUST_b + 4.0f * PWM_TO_THRUST_a * motor_forces[i])) /
                     (2.0f * PWM_TO_THRUST_a);
    }

    motor_pwm[i] = constrain(motor_pwm[i], 0.1, 1); // keep the motor spinning by lower-bounding with 0.1, otherwise motor friction plays a big role in performance
    motorsSetRatio(i, (uint16_t)(65535*motor_pwm[i]));
  }
}

LOG_GROUP_START(pwmsignal)
LOG_ADD(LOG_FLOAT, m1, &motor_pwm[0])
LOG_GROUP_STOP(pwmsignal)

LOG_GROUP_START(motorpwm)
LOG_ADD(LOG_FLOAT, m1, &motor_pwm[0])
LOG_ADD(LOG_FLOAT, m2, &motor_pwm[1])
LOG_ADD(LOG_FLOAT, m3, &motor_pwm[2])
LOG_ADD(LOG_FLOAT, m4, &motor_pwm[3])
LOG_ADD(LOG_FLOAT, fRoll, &RollForce)
LOG_ADD(LOG_FLOAT, fPitch, &PitchForce)
LOG_ADD(LOG_FLOAT, fYaw, &YawForce)
LOG_ADD(LOG_FLOAT, fThrust, &ThrustForce)
LOG_GROUP_STOP(motorpwm)
