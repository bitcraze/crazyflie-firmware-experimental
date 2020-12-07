/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 *
 * api_app.c - App layer application that ilustrates how to use functions in the API.
 *             This app may not be possible to run, the intention is to show how to
 *             use functions.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "lighthouse_core.h"
#include "lighthouse_position_est.h"
#include "crtp_commander_high_level.h"
#include "pptraj.h"
#include "param.h"

#define DEBUG_MODULE "HYPER"

#define START_X 8.0f
#define START_Y 5.0f
#define LAND_X -1.92f
#define LAND_Y -0.67f
#define TRAJ_Z 0.5f

#define OFFS_ARENA_X -1.92
#define OFFS_ARENA_Y -0.67

#define OFFS_HALL_X 4.80
#define OFFS_HALL_Y 1.98

#define OFFS_CONF_X 9.30
#define OFFS_CONF_Y 4.94

const baseStationGeometry_t geoArena0 = {.valid = true, .origin = {-2.057947 + OFFS_ARENA_X, 0.398319 + OFFS_ARENA_Y, 3.109704, }, .mat = {{0.807210, 0.002766, 0.590258, }, {0.067095, 0.993078, -0.096409, }, {-0.586439, 0.117426, 0.801437, }, }};
const baseStationGeometry_t geoArena1 = {.valid = true, .origin = {0.866244 + OFFS_ARENA_X, -2.566829 + OFFS_ARENA_Y, 3.132632, }, .mat = {{-0.043296, -0.997675, -0.052627, }, {0.766284, -0.066962, 0.639003, }, {-0.641042, -0.012661, 0.767401, }, }};
const baseStationGeometry_t geoHallway = {.valid = true, .origin = {0.011919 + OFFS_HALL_X, -2.296393 + OFFS_HALL_Y, 2.085851, }, .mat = {{0.063295, -0.997300, 0.037237, }, {0.796754, 0.072966, 0.599882, }, {-0.600979, -0.008301, 0.799221, }, }};
const baseStationGeometry_t geoConf5 = {.valid = true, .origin = {-1.736219 + OFFS_CONF_X, -1.547099 + OFFS_CONF_Y, 2.053168, }, .mat = {{0.510291, -0.798886, 0.318410, }, {0.763317, 0.591292, 0.260232, }, {-0.396169, 0.110254, 0.911534, }, }};
const baseStationGeometry_t geoConf6 = {.valid = true, .origin = {2.474905 + OFFS_CONF_X, 1.031354 + OFFS_CONF_Y, 2.700770, }, .mat = {{-0.676608, 0.539050, -0.501624, }, {-0.459605, -0.841406, -0.284251, }, {-0.575295, 0.038222, 0.817053, }, }};

const lighthouseCalibration_t calibrationArena0 = {
  .valid = true,
  .sweep = {
    {.tilt = -0.047058, .phase = 0.0, .curve = 0.052215, .gibphase = 2.087890, .gibmag = -0.003913, .ogeephase = 0.433105, .ogeemag = -0.049285},
    {.tilt = 0.048065, .phase = -0.005336, .curve = 0.122375, .gibphase = 2.097656, .gibmag = -0.003883, .ogeephase = 0.631835, .ogeemag = -0.034851},
  }
};
const lighthouseCalibration_t calibrationArena1 = {
  .valid = true,
  .sweep = {
    {.tilt = -0.051208, .phase = 0.0, .curve = 0.011756, .gibphase = 2.136718, .gibmag = -0.006057, .ogeephase = 2.705078,},
    {.tilt = 0.045623, .phase = -0.004142, .curve = 0.104736, .gibphase = 2.349609, .gibmag = -0.003332, .ogeephase = 0.380859, .ogeemag = -0.240112,},
  },
};
const lighthouseCalibration_t calibrationHallway = {
  .sweep = {
    {.tilt = -0.049643, .phase = 0.000000, .curve = 0.199703, .gibphase = 1.400573, .gibmag = -0.003551, .ogeephase = 1.119688, .ogeemag = -0.071342, },
    {.tilt = 0.046603, .phase = -0.006064, .curve = 0.246613, .gibphase = 1.846917, .gibmag = -0.003608, .ogeephase = 0.971186, .ogeemag = -0.074050, },
  },
  .valid = true,
};
const lighthouseCalibration_t calibrationConf5 =  {
  .sweep = {
    {.tilt = -0.046841, .phase = 0.000000, .curve = -0.113471, .gibphase = 0.771659, .gibmag = 0.007080, .ogeephase = 1.087232, .ogeemag = -0.681087, },
    {.tilt = 0.045224, .phase = -0.004105, .curve = 0.026427, .gibphase = 1.905682, .gibmag = 0.005737, .ogeephase = 1.869040, .ogeemag = -0.685554, },
  },
  .valid = true,
};
const lighthouseCalibration_t calibrationConf6 = {
  .sweep = {
    {.tilt = -0.047181, .phase = 0.000000, .curve = -0.165181, .gibphase = 2.971715, .gibmag = -0.006175, .ogeephase = 0.563175, .ogeemag = -0.385662, },
    {.tilt = 0.047797, .phase = -0.001269, .curve = 0.254146, .gibphase = 0.822974, .gibmag = 0.004853, .ogeephase = 1.386157, .ogeemag = -0.473413, },
  },
  .valid = true,
};

static struct poly4d trajectory[] = {
  {.duration = 3.98253,  .p = {{8,0,0,0,-0.0197834,0.012463,-0.00263311,0.000188583}, {5,0,0,0,0.350417,-0.191491,0.0374539,-0.00256411}, {TRAJ_Z,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 1.83232,  .p = {{8,0.116514,0.0214334,-0.00449444,-0.197613,0.217886,-0.0924504,0.0139629}, {10,1.20514,-0.22669,-0.0493878,2.31972,-3.0885,1.43356,-0.227189}, {TRAJ_Z,-7.45648e-20,-1.85452e-20,1.73654e-20,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 2.15784,  .p = {{8,-0.190649,-0.155537,-0.00363435,-0.254545,0.225044,-0.0708872,0.00788965}, {12,0.35721,0.0805593,0.00791068,1.30713,-1.50158,0.586066,-0.0779471}, {TRAJ_Z,4.10142e-19,1.47189e-19,-1.68459e-20, 0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 2.27467,  .p = {{6.4,-1.06997,0.0186635,0.0201014,0.174494,-0.129001,0.03508,-0.0034097}, {13.5,-0.00106785,-0.205008,-0.00630235,-1.33539,1.3979,-0.505294,0.0626929}, {TRAJ_Z,-4.32397e-19,-1.93294e-19,1.03184e-19,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 1.65439,  .p = {{4.9,-0.214736,0.117058,-0.0043866,-0.0173875,0.0218055,-0.013913,0.00294825}, {11.5,-0.613316,0.0805191,-0.0244295,-1.6542,2.26176,-1.0971,0.184558}, {TRAJ_Z,2.29519e-18,6.83935e-19,-1.43668e-19,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 2.68572,  .p = {{4.8,0.0269552,0.00105513,-0.00960518,0.00939224,-0.00656532,0.00199788,-0.000213284}, {10,-0.884246,-0.03737,-0.0318068,-0.185276,0.131554,-0.0333674,0.00301602}, {TRAJ_Z,-5.38137e-18,-7.09272e-19,3.4989e-19,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 3.52281,  .p = {{4.8,-0.0406191,-0.00837168,0.00341152,-0.00823158,0.00732038,-0.00194038,0.000163751}, {6,-1.95944,-0.0175592,0.0310398,0.0866946,-0.0505088,0.0104478,-0.000765394}, {TRAJ_Z,1.99638e-17,3.47525e-18,-3.86337e-19,0,-1.38778e-17,3.46945e-18,-2.1684e-19}, {0,0,0,0,0,0,0,0}}},
  {.duration = 1.15609,  .p = {{4.8,0.0993472,-0.00178404,-0.018034,0.933807,-2.36614,1.88453,-0.49474}, {1,-0.891179,0.0997002,-0.00612358,0.603024,-0.258313,-0.259965,0.138908}, {TRAJ_Z,-3.01052e-17,5.71277e-19,4.03081e-18,0,0,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 0.932444, .p = {{4.8,-0.256402,-0.0964886,-0.0183449,-3.27779,8.89748,-8.15631,2.53706}, {0.4,-0.165006,-0.0186121,-0.018881,-1.26263,3.78816,-3.64518,1.16743}, {TRAJ_Z,5.12249e-17,5.16065e-18,-4.77373e-18,0,-7.10543e-15,0,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 0.954963, .p = {{4.45,-0.306216,-0.0453163,-0.00766164,-1.03067,2.03916,-1.5584,0.428044}, {0.25,-0.0705903,0.000318196,-0.00812184,-1.80055,4.61833,-4.06015,1.21929}, {TRAJ_Z,-7.20308e-17,1.47857e-17,1.82253e-17,-3.55271e-15,0,-7.10543e-15,0}, {0,0,0,0,0,0,0,0}}},
  {.duration = 2.91071,  .p = {{4,-0.678257,-0.24992,-0.0325107,0.0293741,-0.0198713,0.00743214,-0.000894396}, {0.15,-0.0343291,0.0308729,0.00410924,0.0288977,-0.0322001,0.0097542,-0.000955622}, {TRAJ_Z,1.78325e-16,4.33186e-17,-2.45718e-17,-8.32667e-17,8.32667e-17,-1.38778e-17,1.73472e-18}, {0,0,0,0,0,0,0,0}}},
  {.duration = 2.25804,  .p = {{0,-1.68434,0.136983,0.0445563,-0.00653962,0.122268,-0.0755105,0.0122735}, {0,-0.296553,-0.0954911,0.0225388,-0.239509,0.30605,-0.12469,0.0167574}, {TRAJ_Z,-7.48086e-17,4.32972e-17,3.33038e-18,0,0,5.55112e-17,-6.93889e-18}, {0,0,0,0,0,0,0,0}}},
};

static int trajectoryId = 1;

static uint8_t takeOff = 0;
static float landX = 0.0;
static float landY = 0.0;


static float trajTime(struct poly4d trajectory[], int count) {
  float totalDuration = 0.0f;

  for (int i = 0; i < count; i++) {
    totalDuration += trajectory[i].duration;
  }

  return totalDuration;
}

static uint32_t defineTrajectory() {
  const uint32_t polyCount = sizeof(trajectory) / sizeof(struct poly4d);
  uint32_t trajectoryDurationMs = 1000 * trajTime(trajectory, polyCount);
  crtpCommanderHighLevelWriteTrajectory(0, sizeof(trajectory), (uint8_t*)trajectory);
  crtpCommanderHighLevelDefineTrajectory(trajectoryId, CRTP_CHL_TRAJECTORY_TYPE_POLY4D, 0, polyCount);
  return trajectoryDurationMs;
}

void appMain() {
  DEBUG_PRINT("This is the hyper demo\n");
  // Wait a bit to avoid race condition with default initialization
  vTaskDelay(M2T(1000));

  uint32_t trajectoryTime = defineTrajectory();

  paramVarId_t paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  paramSetInt(paramIdCommanderEnHighLevel, 1);

  lighthousePositionSetGeometryData(0, &geoArena0);
  lighthousePositionSetGeometryData(1, &geoArena1);
  lighthousePositionSetGeometryData(3, &geoHallway);
  lighthousePositionSetGeometryData(5, &geoConf5);
  lighthousePositionSetGeometryData(6, &geoConf6);

  lighthouseCoreSetCalibrationData(0, &calibrationArena0);
  lighthouseCoreSetCalibrationData(1, &calibrationArena1);
  lighthouseCoreSetCalibrationData(3, &calibrationHallway);
  lighthouseCoreSetCalibrationData(5, &calibrationConf5);
  lighthouseCoreSetCalibrationData(6, &calibrationConf6);

  DEBUG_PRINT("Ready for take off\n");
  while(!takeOff) {
    vTaskDelay(M2T(10));
  }

  DEBUG_PRINT("Take off\n");
  crtpCommanderHighLevelTakeoff(TRAJ_Z, 2.0f);
  vTaskDelay(M2T(2000));

  DEBUG_PRINT("Go to starting point\n");
  crtpCommanderHighLevelGoTo(START_X, START_Y, TRAJ_Z, 0.0f, 4.0f, false);
  // Wait a bit too short time. The trajectory will start a bit
  // early which will move the CF out of the way for the next CF. Otherwise
  // they clog up at the take off point
  vTaskDelay(M2T(3000));

  DEBUG_PRINT("Start trajectory\n");
  crtpCommanderHighLevelStartTrajectory(trajectoryId, 1.0f, false, false);
  // Wait a bit too short to avoid clogging
  vTaskDelay(M2T(trajectoryTime - 1000));

  DEBUG_PRINT("Go to landing point\n");
  crtpCommanderHighLevelGoTo(LAND_X + landX, LAND_Y + landY, TRAJ_Z, 0.0f, 2.0f, false);
  vTaskDelay(M2T(2000));

  DEBUG_PRINT("Land\n");
  crtpCommanderHighLevelLand(0.0f, 2.0f);

  DEBUG_PRINT("Over and out!\n");
  while(1) {
    vTaskDelay(M2T(3000));
  }
}


PARAM_GROUP_START(hyper)
  PARAM_ADD(PARAM_UINT8, takeoff, &takeOff)
  PARAM_ADD(PARAM_FLOAT, landX, &landX)
  PARAM_ADD(PARAM_FLOAT, landY, &landY)
PARAM_GROUP_STOP(hyper)
