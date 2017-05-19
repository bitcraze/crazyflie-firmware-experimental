#pragma once

#include "arm_math.h"
#include "math.h"
#include "num.h"

#define ARCMINUTE (((float)M_PI)/10800.0f)

float radians(float degrees);
float degrees(float radians);

int quadratic_real_roots(float a, float b, float c, float *plusRoot, float *minusRoot);

float arm_sqrt(float32_t in);

void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst);
void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst);
void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst);

float vec_norm(const arm_matrix_instance_f32 *pSrcA);
float vec_dot(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB);

void vec_normalize(arm_matrix_instance_f32 *pSrcA);
void vec_negate(arm_matrix_instance_f32 *pSrcA);
void vec_cross(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);

void quaternion_normalize(arm_matrix_instance_f32 *pSrcA);
void quaternion_inverse(const arm_matrix_instance_f32 *pSrcA, arm_matrix_instance_f32 *pDst);
void quaternion_multiply(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
