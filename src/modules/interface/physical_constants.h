#pragma once

#include "cfmath.h"

#define GRAVITY (9.81f)
#define SPEED_OF_LIGHT (299792458)

// constants based on Julian Förster's System Identification of a Crazyflie

extern float CRAZYFLIE_ARM_LENGTH; // arm length from CF center to motor center
extern float CRAZYFLIE_COG_HEIGHT; // height of CF CenterOfGravity from floor
extern float CRAZYFLIE_MASS;

extern float CRAZYFLIE_INERTIA[3][3];
extern arm_matrix_instance_f32 CRAZYFLIE_INERTIA_m;

// thrust force = a * pwm^2 + b * pwm
extern float PWM_TO_THRUST_a;
extern float PWM_TO_THRUST_b;

// motor torque = m * thrust force
extern float THRUST_TO_TORQUE_m;

// noises
extern float NOISE_GYRO_ROLLPITCH;
extern float NOISE_GYRO_YAW;
extern float NOISE_ACC_XY;
extern float NOISE_ACC_Z;
