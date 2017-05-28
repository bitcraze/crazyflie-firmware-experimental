#include "physical_constants.h"

#include "param.h"

// constants based on Julian Förster's System Identification of a Crazyflie

float CRAZYFLIE_INERTIA[3][3] =
    {{16.6e-6f, 0.83e-6f, 0.72e-6f},
     {0.83e-6f, 16.6e-6f, 1.8e-6f},
     {0.72e-6f, 1.8e-6f, 29.3e-6f}};

arm_matrix_instance_f32 CRAZYFLIE_INERTIA_m = {3, 3, (float*)CRAZYFLIE_INERTIA};

// thrust = a * pwm^2 + b * pwm
float PWM_TO_THRUST_a = .091492681f;
float PWM_TO_THRUST_b = .067673604f;

// motor torque = m * thrust
float THRUST_TO_TORQUE_m = 0.005964552f;

// constants
float CRAZYFLIE_ARM_LENGTH = 0.046f; // m
float CRAZYFLIE_COG_HEIGHT = 0.02f; // m
float CRAZYFLIE_MASS = 29.3e-3f; // kg

// noises
 float NOISE_GYRO_ROLLPITCH = 0.1f; // radians per second
 float NOISE_GYRO_YAW = 0.25f; // radians per second
 float NOISE_ACC_XY = 0.5f; // m/s^2
 float NOISE_ACC_Z = 1.5f; // m/s^2

PARAM_GROUP_START(physical)
PARAM_ADD(PARAM_FLOAT, mass, &CRAZYFLIE_MASS)
PARAM_ADD(PARAM_FLOAT, noiseGyroRP, &NOISE_GYRO_ROLLPITCH)
PARAM_ADD(PARAM_FLOAT, noiseGyroYAW, &NOISE_GYRO_YAW)
PARAM_ADD(PARAM_FLOAT, noiseAccXY, &NOISE_ACC_XY)
PARAM_ADD(PARAM_FLOAT, noiseAccZ, &NOISE_ACC_Z)
PARAM_GROUP_STOP(physical)
