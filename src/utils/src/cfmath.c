#include "cfmath.h"
#include "FreeRTOSConfig.h"

float radians(float degrees) {
    return degrees * (float)M_PI / 180.0f;
}

float degrees(float radians) {
    return radians * 180.0f / (float)M_PI;
}

int quadratic_real_roots(float a, float b, float c, float *plusRoot, float *minusRoot) {
    // 1/(2*a) * (-b +/- sqrt(b^2 - 4*a*c))
    float sqrtTermSq = b*b - 4*a*c;
    if (sqrtTermSq < 0) { return 0; }
    float sqrtTerm = arm_sqrt(sqrtTermSq)/(2*a);
    float bTerm = -b/(2*a);

    *plusRoot = bTerm + sqrtTerm;
    *minusRoot = bTerm - sqrtTerm;
    if (sqrtTerm == 0) { return 1; }

    return 2;
}

float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }

void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }

void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }

float vec_norm(const arm_matrix_instance_f32 *pSrcA) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );

  float *a = pSrcA->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    s += powf(a[0+1*i], 2);
  }

  if (s<=0) { return 0; }

  return arm_sqrt(s);
}

void vec_normalize(arm_matrix_instance_f32 *pSrcA) {
  float norm = vec_norm(pSrcA);

  if (norm<=0) { return; }

  float *a = pSrcA->pData;
  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = a[i]/norm;
  }
}

void vec_negate(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );

  float *a = pSrcA->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = -1*a[i];
  }
}

float vec_dot(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcA->numRows == pSrcB->numRows );

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;

  for (int i=0; i<pSrcA->numRows; i++) {
    s += a[0+1*i] * b[0+1*i];
  }
  return s;
}

void vec_cross(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pDst->numCols == 1 );
  configASSERT( pSrcA->numRows == 3 );
  configASSERT( pSrcB->numRows == 3 );
  configASSERT( pDst->numRows == 3 );

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;

  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

void quaternion_normalize(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  vec_normalize(pSrcA);
  if (pSrcA->pData[0] < 0) {
    vec_negate(pSrcA);
  }
}

void quaternion_inverse(const arm_matrix_instance_f32 *pSrcA, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  float *a = pSrcA->pData;
  float *c = pDst->pData;
  c[0] = a[0];
  c[1] = -a[1];
  c[2] = -a[2];
  c[3] = -a[3];

  vec_normalize(pDst);
}

void quaternion_multiply(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcB->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]

  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;

  c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
