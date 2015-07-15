//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred_euler.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred_euler.h"
#include "SLAM_emxutil.h"
#include "diag.h"
#include "eye.h"
#include "RotFromQuatJ.h"
#include <stdio.h>

// Function Declarations
static int div_nzp_s32_floor(int numerator, int denominator);

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_nzp_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (numerator >= 0) {
    absNumerator = (unsigned int)numerator;
  } else {
    absNumerator = (unsigned int)-numerator;
  }

  if (denominator >= 0) {
    absDenominator = (unsigned int)denominator;
  } else {
    absDenominator = (unsigned int)-denominator;
  }

  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  if (quotientNeedsNegation) {
    quotient = -(int)tempAbsQuotient;
  } else {
    quotient = (int)tempAbsQuotient;
  }

  return quotient;
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
//  Xv meaning
//
//                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
//  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
//  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
// Arguments    : emxArray_real_T *P_apo
//                const emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                double c_numStates
// Return Type  : void
//
void SLAM_pred_euler(emxArray_real_T *P_apo, const emxArray_real_T *x, double dt,
                     const double processNoise[4], const double
                     IMU_measurements[9], double c_numStates)
{
  double c;
  double R_cw[9];
  double P_xx_apr[144];
  double b_processNoise[9];
  double w[3];
  double b_R_cw[9];
  int i21;
  int cr;
  double dv626[9];
  double c_R_cw[9];
  int k;
  double G1[144];
  static const signed char iv2[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv3[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double Phi[144];
  double b_G1[108];
  static const signed char iv4[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv5[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *b_P_apo;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double y[144];
  double c_processNoise[9];
  double dv627[81];
  double c_G1[108];
  emxArray_real_T *c_P_apo;
  emxArray_int32_T *r6;
  emxArray_int32_T *r7;
  emxArray_real_T *b_P_xs_apr;

  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  // % compute the linearization F of the non linear model f
  c = dt * dt;
  RotFromQuatJ(*(double (*)[4])&x->data[3], R_cw);

  //  want R_wi imu to world
  // ===========ACC=====================================
  b_eye(P_xx_apr);
  b_processNoise[0] = 0.0;
  for (i21 = 0; i21 < 3; i21++) {
    w[i21] = IMU_measurements[i21] - x->data[10 + i21];
    for (cr = 0; cr < 3; cr++) {
      b_R_cw[cr + 3 * i21] = -R_cw[i21 + 3 * cr];
    }
  }

  b_processNoise[3] = -w[2];
  b_processNoise[6] = w[1];
  b_processNoise[1] = w[2];
  b_processNoise[4] = 0.0;
  b_processNoise[7] = -w[0];
  b_processNoise[2] = -w[1];
  b_processNoise[5] = w[0];
  b_processNoise[8] = 0.0;
  dv626[0] = 0.0;
  dv626[3] = -IMU_measurements[5];
  dv626[6] = IMU_measurements[4];
  dv626[1] = IMU_measurements[5];
  dv626[4] = 0.0;
  dv626[7] = -IMU_measurements[3];
  dv626[2] = -IMU_measurements[4];
  dv626[5] = IMU_measurements[3];
  dv626[8] = 0.0;
  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      c_R_cw[i21 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        c_R_cw[i21 + 3 * cr] += b_R_cw[i21 + 3 * k] * dv626[k + 3 * cr];
      }
    }
  }

  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[cr + 12 * i21] = iv2[cr + 3 * i21];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i21) + 3] = 0.0;
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 3)) + 3] = -b_processNoise[cr + 3 * i21];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 6)) + 3] = 0.0;
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 9)) + 3] = iv3[cr + 3 * i21];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i21) + 6] = 0.0;
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 3)) + 6] = c_R_cw[cr + 3 * i21];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 6)) + 6] = 0.0;
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i21 + 9)) + 6] = 0.0;
    }
  }

  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i21) + 9] = 0.0;
    }

    for (cr = 0; cr < 12; cr++) {
      Phi[cr + 12 * i21] = P_xx_apr[cr + 12 * i21] + G1[cr + 12 * i21] * dt;
    }
  }

  for (i21 = 0; i21 < 9; i21++) {
    for (cr = 0; cr < 3; cr++) {
      b_G1[cr + 12 * i21] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G1[(cr + 12 * i21) + 3] = iv4[cr + 3 * i21];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      b_G1[(cr + 12 * i21) + 6] = 0.0;
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      b_G1[(cr + 12 * (i21 + 3)) + 6] = -R_cw[i21 + 3 * cr];
    }
  }

  for (i21 = 0; i21 < 3; i21++) {
    for (cr = 0; cr < 3; cr++) {
      b_G1[(cr + 12 * (i21 + 6)) + 6] = 0.0;
    }
  }

  for (i21 = 0; i21 < 9; i21++) {
    for (cr = 0; cr < 3; cr++) {
      b_G1[(cr + 12 * i21) + 9] = iv5[cr + 3 * i21];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_real_T(&P_xs_apr, 2);
  if (loop_ub == 1) {
    emxInit_real_T(&b_P_apo, 2);
    i21 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = 1;
    b_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)b_P_apo, i21, (int)sizeof(double));
    for (i21 = 0; i21 < b_loop_ub; i21++) {
      cr = 0;
      while (cr <= 0) {
        b_P_apo->data[b_P_apo->size[0] * i21] = P_apo->data[P_apo->size[0] * i21];
        cr = 1;
      }
    }

    i21 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i21, (int)sizeof(double));
    for (i21 = 0; i21 < 12; i21++) {
      loop_ub = b_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i21 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i21 + P_xs_apr->size[0] * cr] += Phi[i21 + 12 * k] *
            b_P_apo->data[k + b_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&b_P_apo);
  } else {
    i21 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i21, (int)sizeof(double));
    i21 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i21, (int)sizeof(double));
    k = 12 * b_loop_ub;
    for (i21 = 0; i21 < k; i21++) {
      P_xs_apr->data[i21] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 12 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          P_xs_apr->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * div_nzp_s32_floor(ib,
               loop_ub)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                div_nzp_s32_floor(ib, loop_ub)] * Phi[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i21] = Phi[i21 + 12 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i21 = 0; i21 < 12; i21++) {
      for (cr = 0; cr < 12; cr++) {
        y[i21 + 12 * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          y[i21 + 12 * cr] += P_xs_apr->data[i21 + 12 * k] * P_xx_apr[k + 12 *
            cr];
        }
      }
    }
  } else {
    k = P_xs_apr->size[1];
    memset(&y[0], 0, 144U * sizeof(double));
    for (cr = 0; cr < 134; cr += 12) {
      for (ic = cr; ic + 1 <= cr + 12; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 134; cr += 12) {
      ar = -1;
      i21 = br + k;
      for (ib = br; ib + 1 <= i21; ib++) {
        if (P_xx_apr[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            y[ic] += P_xx_apr[ib] * P_xs_apr->data[ia];
          }
        }

        ar += 12;
      }

      br += k;
    }
  }

  c_processNoise[0] = processNoise[1];
  c_processNoise[1] = processNoise[1];
  c_processNoise[2] = processNoise[1];
  c_processNoise[3] = processNoise[0];
  c_processNoise[4] = processNoise[0];
  c_processNoise[5] = processNoise[0];
  c_processNoise[6] = processNoise[2];
  c_processNoise[7] = processNoise[2];
  c_processNoise[8] = processNoise[2];
  for (i21 = 0; i21 < 9; i21++) {
    b_processNoise[i21] = c_processNoise[i21] * c;
  }

  diag(b_processNoise, dv627);
  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 9; cr++) {
      c_G1[i21 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        c_G1[i21 + 12 * cr] += b_G1[i21 + 12 * k] * dv627[k + 9 * cr];
      }
    }

    for (cr = 0; cr < 12; cr++) {
      G1[i21 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        G1[i21 + 12 * cr] += c_G1[i21 + 12 * k] * b_G1[cr + 12 * k];
      }
    }
  }

  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i21] = y[cr + 12 * i21] + G1[cr + 12 * i21] * dt;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i21 = 1;
    cr = 1;
  } else {
    i21 = (int)(c_numStates + 1.0);
    cr = P_apo->size[1] + 1;
  }

  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    k = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = cr - i21;
    emxEnsureCapacity((emxArray__common *)c_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i21;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        c_P_apo->data[c_P_apo->size[0] * cr] = P_apo->data[P_apo->size[0] *
          ((i21 + cr) - 1)];
        k = 1;
      }
    }

    i21 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i21, (int)sizeof(double));
    for (i21 = 0; i21 < 12; i21++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i21 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i21 + P_xs_apr->size[0] * cr] += Phi[i21 + 12 * k] *
            c_P_apo->data[k + c_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&c_P_apo);
  } else {
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = cr - i21;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 12 * (cr - i21);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i21 == 0) {
    } else {
      b_loop_ub = 12 * ((cr - i21) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          P_xs_apr->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * ((i21 +
                div_nzp_s32_floor(ib, loop_ub)) - 1)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                ((i21 + div_nzp_s32_floor(ib, loop_ub)) - 1)] * Phi[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  //  covariance between current state and trails
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_int32_T(&r6, 1);
  i21 = r6->size[0];
  r6->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r6, i21, (int)sizeof(int));
  for (i21 = 0; i21 < loop_ub; i21++) {
    r6->data[i21] = i21;
  }

  emxInit_int32_T(&r7, 1);
  i21 = r7->size[0];
  r7->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r7, i21, (int)sizeof(int));
  for (i21 = 0; i21 < b_loop_ub; i21++) {
    r7->data[i21] = i21;
  }

  for (i21 = 0; i21 < 12; i21++) {
    for (cr = 0; cr < 12; cr++) {
      G1[cr + 12 * i21] = (P_xx_apr[cr + 12 * i21] + P_xx_apr[i21 + 12 * cr]) /
        2.0;
    }
  }

  k = r6->size[0];
  b_loop_ub = r7->size[0];
  for (i21 = 0; i21 < b_loop_ub; i21++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r6->data[cr] + P_apo->size[0] * r7->data[i21]] = G1[cr + k *
        i21];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i21 = 0;
    cr = 0;
  } else {
    i21 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  k = r6->size[0];
  r6->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r6, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r6->data[k] = k;
  }

  k = r7->size[0];
  r7->size[0] = cr - i21;
  emxEnsureCapacity((emxArray__common *)r7, k, (int)sizeof(int));
  loop_ub = cr - i21;
  for (cr = 0; cr < loop_ub; cr++) {
    r7->data[cr] = i21 + cr;
  }

  k = r6->size[0];
  b_loop_ub = r7->size[0];
  for (i21 = 0; i21 < b_loop_ub; i21++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r6->data[cr] + P_apo->size[0] * r7->data[i21]] =
        P_xs_apr->data[cr + k * i21];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i21 = 0;
    cr = 0;
  } else {
    i21 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r6->size[0];
  r6->size[0] = cr - i21;
  emxEnsureCapacity((emxArray__common *)r6, k, (int)sizeof(int));
  b_loop_ub = cr - i21;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r6->data[cr] = i21 + cr;
  }

  i21 = r7->size[0];
  r7->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r7, i21, (int)sizeof(int));
  for (i21 = 0; i21 < loop_ub; i21++) {
    r7->data[i21] = i21;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i21 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i21, (int)sizeof(double));
  for (i21 = 0; i21 < 12; i21++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i21] = P_xs_apr->data[i21 +
        P_xs_apr->size[0] * cr];
    }
  }

  emxFree_real_T(&P_xs_apr);
  k = r6->size[0];
  b_loop_ub = r7->size[0];
  for (i21 = 0; i21 < b_loop_ub; i21++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r6->data[cr] + P_apo->size[0] * r7->data[i21]] =
        b_P_xs_apr->data[cr + k * i21];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r7);
  emxFree_int32_T(&r6);

  //  x(1:3) = x(1:3) + x(8:10)*dt;%+1/2*dt^2*(R_cw'*a-grav);             % position 
  //  dq = quatPlusThetaJ(w*dt);
  //  x(4:7) = quatmultJ(dq,x(4:7));
  //  x(4:7) = x(4:7) / norm(x(4:7));
  //  x(8:10) = x(8:10)+ (R_cw'*a - grav)*dt; % velocity
  //  disp('R*a-g')
  //  disp(R_cw'*a - grav)
}

//
// File trailer for SLAM_pred_euler.cpp
//
// [EOF]
//
