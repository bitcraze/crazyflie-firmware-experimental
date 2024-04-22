/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdbool.h>
#include "motors.h"

#include "autoconf.h"

#define PLATFORM_DEVICE_TYPE_STRING_MAX_LEN (32 + 1)
#define PLATFORM_DEVICE_TYPE_MAX_LEN (4 + 1)

typedef enum {
  #ifdef CONFIG_SENSORS_BMI088_BMP3XX
  SensorImplementation_bmi088_bmp3xx,
  #endif

  #ifdef CONFIG_SENSORS_BMI088_SPI
  SensorImplementation_bmi088_spi_bmp3xx,
  #endif

  #ifdef CONFIG_SENSORS_MPU9250_LPS25H
  SensorImplementation_mpu9250_lps25h,
  #endif

  #ifdef CONFIG_SENSORS_BOSCH
  SensorImplementation_bosch,
  #endif

  SensorImplementation_COUNT,
} SensorImplementation_t;

typedef struct {
  char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];
  char deviceTypeName[30];
  SensorImplementation_t sensorImplementation;
  bool physicalLayoutAntennasAreClose;
  const MotorPerifDef** motorMap;
} platformConfig_t;

/**
 * Initializes all platform specific things.
 */
int platformInit(void);

void platformGetDeviceTypeString(char* deviceTypeString);
int platformParseDeviceTypeString(const char* deviceTypeString, char* deviceType);
int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs);

// Implemented in platform specific files
const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs);
void platformInitHardware();


void platformSetLowInterferenceRadioMode(void);

// Functions to read configuration
const char* platformConfigGetPlatformName();
const char* platformConfigGetDeviceType();
const char* platformConfigGetDeviceTypeName();
SensorImplementation_t platformConfigGetSensorImplementation();
bool platformConfigPhysicalLayoutAntennasAreClose();
const MotorPerifDef** platformConfigGetMotorMapping();

#endif /* PLATFORM_H_ */
