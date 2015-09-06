//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Sep-2015 10:04:04
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred.h"
#include "norm.h"
#include "SLAM_emxutil.h"
#include "diag.h"
#include "eye.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static double last_imu[6];
static boolean_T last_imu_not_empty;

// Function Declarations
static void dxdt_dPdt(const double meas[6], const emxArray_real_T *x,
                      emxArray_real_T *x_dot);

// Function Definitions

//
// Arguments    : const double meas[6]
//                const emxArray_real_T *x
//                emxArray_real_T *x_dot
// Return Type  : void
//
static void dxdt_dPdt(const double meas[6], const emxArray_real_T *x,
                      emxArray_real_T *x_dot)
{
  unsigned int unnamed_idx_0;
  int i11;
  int loop_ub;
  double dv5[9];
  double dv6[16];
  double dv7[16];
  double b_x[4];
  double dv8[4];

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  i11 = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, i11, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (i11 = 0; i11 < loop_ub; i11++) {
    x_dot->data[i11] = 0.0;
  }

  for (i11 = 0; i11 < 3; i11++) {
    x_dot->data[i11] = x->data[7 + i11];
  }

  //  position
  dv5[0] = 0.0;
  dv5[3] = -meas[2];
  dv5[6] = meas[1];
  dv5[1] = meas[2];
  dv5[4] = 0.0;
  dv5[7] = -meas[0];
  dv5[2] = -meas[1];
  dv5[5] = meas[0];
  dv5[8] = 0.0;
  for (i11 = 0; i11 < 3; i11++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv6[loop_ub + (i11 << 2)] = -dv5[loop_ub + 3 * i11];
    }
  }

  for (i11 = 0; i11 < 3; i11++) {
    dv6[12 + i11] = meas[i11];
  }

  for (i11 = 0; i11 < 3; i11++) {
    dv6[3 + (i11 << 2)] = -meas[i11];
  }

  dv6[15] = 0.0;
  for (i11 = 0; i11 < 4; i11++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv7[loop_ub + (i11 << 2)] = 0.5 * dv6[loop_ub + (i11 << 2)];
    }
  }

  for (i11 = 0; i11 < 4; i11++) {
    b_x[i11] = x->data[3 + i11];
  }

  for (i11 = 0; i11 < 4; i11++) {
    dv8[i11] = 0.0;
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv8[i11] += dv7[i11 + (loop_ub << 2)] * b_x[loop_ub];
    }
  }

  for (i11 = 0; i11 < 4; i11++) {
    x_dot->data[3 + i11] = dv8[i11];
  }

  //  rot angle
  for (i11 = 0; i11 < 3; i11++) {
    x_dot->data[7 + i11] = meas[3 + i11];
  }

  //  velocity
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
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double measurements_gyr_duo[3]
//                const double measurements_acc_duo[3]
//                double c_numStates
// Return Type  : void
//
void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt, const
               double processNoise[4], const double measurements_gyr_duo[3],
               const double measurements_acc_duo[3], double c_numStates)
{
  double w[3];
  int i32;
  double b_w;
  double current_imu[6];
  int k;
  double R_cw[9];
  int cr;
  double G[108];
  static const signed char iv6[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double P_xx_apr[144];
  double dv14[9];
  double b_R_cw[9];
  double dv15[9];
  double b_G[144];
  static const signed char iv8[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv9[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double Phi[144];
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
  double c;
  double b_processNoise[9];
  double dv16[81];
  double c_G[108];
  emxArray_real_T *c_P_apo;
  emxArray_int32_T *r11;
  emxArray_int32_T *r12;
  emxArray_real_T *b_P_xs_apr;
  emxArray_real_T *b_x;
  emxArray_real_T *x1;
  emxArray_real_T *xx;
  double b_xx[4];
  double dv17[6];
  emxArray_real_T *x2;
  emxArray_real_T *x3;
  emxArray_real_T *x4;
  unsigned int unnamed_idx_0;
  double dv18[9];
  double dv19[16];
  double dv20[16];
  double dv21[4];
  emxArray_int32_T *r13;
  emxArray_real_T *c_x;

  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  %% Iterative Camera Pose optimization (EKF)
  for (i32 = 0; i32 < 3; i32++) {
    b_w = measurements_gyr_duo[i32] - x->data[10 + i32];
    current_imu[i32] = b_w;
    w[i32] = b_w;
  }

  for (k = 0; k < 3; k++) {
    current_imu[k + 3] = measurements_acc_duo[k];
  }

  if (!last_imu_not_empty) {
    for (k = 0; k < 6; k++) {
      last_imu[k] = current_imu[k];
    }

    last_imu_not_empty = true;
  }

  // % compute the linearization F of the non linear model f
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((x->data[3] * x->data[3] - x->data[4] * x->data[4]) - x->data[5] *
             x->data[5]) + x->data[6] * x->data[6];
  R_cw[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  R_cw[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  R_cw[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  R_cw[4] = ((-(x->data[3] * x->data[3]) + x->data[4] * x->data[4]) - x->data[5]
             * x->data[5]) + x->data[6] * x->data[6];
  R_cw[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  R_cw[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  R_cw[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  R_cw[8] = ((-(x->data[3] * x->data[3]) - x->data[4] * x->data[4]) + x->data[5]
             * x->data[5]) + x->data[6] * x->data[6];
  for (i32 = 0; i32 < 9; i32++) {
    for (cr = 0; cr < 3; cr++) {
      G[cr + 12 * i32] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i32) + 3] = iv6[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i32) + 6] = 0.0;
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * (i32 + 3)) + 6] = -R_cw[i32 + 3 * cr];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * (i32 + 6)) + 6] = 0.0;
    }
  }

  for (i32 = 0; i32 < 9; i32++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i32) + 9] = iv7[cr + 3 * i32];
    }
  }

  d_eye(P_xx_apr);
  dv14[0] = 0.0;
  dv14[3] = -w[2];
  dv14[6] = w[1];
  dv14[1] = w[2];
  dv14[4] = 0.0;
  dv14[7] = -w[0];
  dv14[2] = -w[1];
  dv14[5] = w[0];
  dv14[8] = 0.0;
  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_R_cw[cr + 3 * i32] = -R_cw[i32 + 3 * cr];
    }
  }

  dv15[0] = 0.0;
  dv15[3] = -measurements_acc_duo[2];
  dv15[6] = measurements_acc_duo[1];
  dv15[1] = measurements_acc_duo[2];
  dv15[4] = 0.0;
  dv15[7] = -measurements_acc_duo[0];
  dv15[2] = -measurements_acc_duo[1];
  dv15[5] = measurements_acc_duo[0];
  dv15[8] = 0.0;
  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      R_cw[i32 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        R_cw[i32 + 3 * cr] += b_R_cw[i32 + 3 * k] * dv15[k + 3 * cr];
      }
    }
  }

  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[cr + 12 * i32] = iv8[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i32) + 3] = 0.0;
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 3)) + 3] = -dv14[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 6)) + 3] = 0.0;
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 9)) + 3] = iv9[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i32) + 6] = 0.0;
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 3)) + 6] = R_cw[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 6)) + 6] = 0.0;
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i32 + 9)) + 6] = 0.0;
    }
  }

  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i32) + 9] = 0.0;
    }

    for (cr = 0; cr < 12; cr++) {
      Phi[cr + 12 * i32] = P_xx_apr[cr + 12 * i32] + b_G[cr + 12 * i32] * dt;
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
    i32 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = 1;
    b_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)b_P_apo, i32, (int)sizeof(double));
    for (i32 = 0; i32 < b_loop_ub; i32++) {
      cr = 0;
      while (cr <= 0) {
        b_P_apo->data[b_P_apo->size[0] * i32] = P_apo->data[P_apo->size[0] * i32];
        cr = 1;
      }
    }

    i32 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i32, (int)sizeof(double));
    for (i32 = 0; i32 < 12; i32++) {
      loop_ub = b_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i32 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i32 + P_xs_apr->size[0] * cr] += Phi[i32 + 12 * k] *
            b_P_apo->data[k + b_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&b_P_apo);
  } else {
    i32 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i32, (int)sizeof(double));
    i32 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i32, (int)sizeof(double));
    k = 12 * b_loop_ub;
    for (i32 = 0; i32 < k; i32++) {
      P_xs_apr->data[i32] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 12 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (ib / loop_ub)] != 0.0)
          {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (ib / loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i32] = Phi[i32 + 12 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i32 = 0; i32 < 12; i32++) {
      for (cr = 0; cr < 12; cr++) {
        y[i32 + 12 * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          y[i32 + 12 * cr] += P_xs_apr->data[i32 + 12 * k] * P_xx_apr[k + 12 *
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
      ar = 0;
      i32 = br + k;
      for (ib = br; ib + 1 <= i32; ib++) {
        if (P_xx_apr[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            y[ic] += P_xx_apr[ib] * P_xs_apr->data[ia - 1];
          }
        }

        ar += 12;
      }

      br += k;
    }
  }

  c = dt * dt;
  b_processNoise[0] = processNoise[1];
  b_processNoise[1] = processNoise[1];
  b_processNoise[2] = processNoise[1];
  b_processNoise[3] = processNoise[0];
  b_processNoise[4] = processNoise[0];
  b_processNoise[5] = processNoise[0];
  b_processNoise[6] = processNoise[2];
  b_processNoise[7] = processNoise[2];
  b_processNoise[8] = processNoise[2];
  diag(b_processNoise, dv16);
  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 9; cr++) {
      c_G[i32 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        c_G[i32 + 12 * cr] += G[i32 + 12 * k] * dv16[k + 9 * cr];
      }
    }

    for (cr = 0; cr < 12; cr++) {
      b_G[i32 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        b_G[i32 + 12 * cr] += c_G[i32 + 12 * k] * G[cr + 12 * k];
      }
    }
  }

  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i32] = y[cr + 12 * i32] + b_G[cr + 12 * i32] * c;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i32 = 0;
    cr = 0;
  } else {
    i32 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    k = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = cr - i32;
    emxEnsureCapacity((emxArray__common *)c_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i32;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        c_P_apo->data[c_P_apo->size[0] * cr] = P_apo->data[P_apo->size[0] * (i32
          + cr)];
        k = 1;
      }
    }

    i32 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i32, (int)sizeof(double));
    for (i32 = 0; i32 < 12; i32++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i32 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i32 + P_xs_apr->size[0] * cr] += Phi[i32 + 12 * k] *
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
    P_xs_apr->size[1] = cr - i32;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 12 * (cr - i32);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i32 == 0) {
    } else {
      b_loop_ub = 12 * ((cr - i32) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i32 + ib / loop_ub)]
              != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i32 + ib / loop_ub)] * Phi[ia - 1];
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

  emxInit_int32_T(&r11, 1);
  i32 = r11->size[0];
  r11->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r11, i32, (int)sizeof(int));
  for (i32 = 0; i32 < loop_ub; i32++) {
    r11->data[i32] = i32;
  }

  emxInit_int32_T(&r12, 1);
  i32 = r12->size[0];
  r12->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r12, i32, (int)sizeof(int));
  for (i32 = 0; i32 < b_loop_ub; i32++) {
    r12->data[i32] = i32;
  }

  for (i32 = 0; i32 < 12; i32++) {
    for (cr = 0; cr < 12; cr++) {
      b_G[cr + 12 * i32] = (P_xx_apr[cr + 12 * i32] + P_xx_apr[i32 + 12 * cr]) /
        2.0;
    }
  }

  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i32 = 0; i32 < b_loop_ub; i32++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i32]] = b_G[cr + k *
        i32];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i32 = 0;
    cr = 0;
  } else {
    i32 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  k = r11->size[0];
  r11->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r11, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r11->data[k] = k;
  }

  k = r12->size[0];
  r12->size[0] = cr - i32;
  emxEnsureCapacity((emxArray__common *)r12, k, (int)sizeof(int));
  loop_ub = cr - i32;
  for (cr = 0; cr < loop_ub; cr++) {
    r12->data[cr] = i32 + cr;
  }

  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i32 = 0; i32 < b_loop_ub; i32++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i32]] =
        P_xs_apr->data[cr + k * i32];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i32 = 0;
    cr = 0;
  } else {
    i32 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r11->size[0];
  r11->size[0] = cr - i32;
  emxEnsureCapacity((emxArray__common *)r11, k, (int)sizeof(int));
  b_loop_ub = cr - i32;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r11->data[cr] = i32 + cr;
  }

  i32 = r12->size[0];
  r12->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r12, i32, (int)sizeof(int));
  for (i32 = 0; i32 < loop_ub; i32++) {
    r12->data[i32] = i32;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i32 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i32, (int)sizeof(double));
  for (i32 = 0; i32 < 12; i32++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i32] = P_xs_apr->data[i32 +
        P_xs_apr->size[0] * cr];
    }
  }

  emxFree_real_T(&P_xs_apr);
  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i32 = 0; i32 < b_loop_ub; i32++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i32]] =
        b_P_xs_apr->data[cr + k * i32];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r12);
  emxFree_int32_T(&r11);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&b_x, 1);
  i32 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i32, (int)sizeof(double));
  for (i32 = 0; i32 < loop_ub; i32++) {
    b_x->data[i32] = x->data[i32];
  }

  b_emxInit_real_T(&x1, 1);
  dxdt_dPdt(last_imu, b_x, x1);
  emxFree_real_T(&b_x);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&xx, 1);
  i32 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i32, (int)sizeof(double));
  for (i32 = 0; i32 < loop_ub; i32++) {
    xx->data[i32] = x->data[i32] + x1->data[i32] * dt / 2.0;
  }

  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32];
  }

  c = b_norm(b_xx);
  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32] / c;
  }

  for (i32 = 0; i32 < 4; i32++) {
    xx->data[3 + i32] = b_xx[i32];
  }

  for (k = 0; k < 6; k++) {
    dv17[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  b_emxInit_real_T(&x2, 1);
  dxdt_dPdt(dv17, xx, x2);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i32 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i32, (int)sizeof(double));
  for (i32 = 0; i32 < loop_ub; i32++) {
    xx->data[i32] = x->data[i32] + x2->data[i32] * dt / 2.0;
  }

  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32];
  }

  c = b_norm(b_xx);
  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32] / c;
  }

  for (i32 = 0; i32 < 4; i32++) {
    xx->data[3 + i32] = b_xx[i32];
  }

  for (k = 0; k < 6; k++) {
    dv17[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  b_emxInit_real_T(&x3, 1);
  dxdt_dPdt(dv17, xx, x3);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i32 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i32, (int)sizeof(double));
  for (i32 = 0; i32 < loop_ub; i32++) {
    xx->data[i32] = x->data[i32] + x3->data[i32] * dt;
  }

  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32];
  }

  c = b_norm(b_xx);
  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32] / c;
  }

  for (i32 = 0; i32 < 4; i32++) {
    xx->data[3 + i32] = b_xx[i32];
  }

  b_emxInit_real_T(&x4, 1);

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)xx->size[0];
  i32 = x4->size[0];
  x4->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x4, i32, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (i32 = 0; i32 < loop_ub; i32++) {
    x4->data[i32] = 0.0;
  }

  for (i32 = 0; i32 < 3; i32++) {
    x4->data[i32] = xx->data[7 + i32];
  }

  //  position
  dv18[0] = 0.0;
  dv18[3] = -current_imu[2];
  dv18[6] = current_imu[1];
  dv18[1] = current_imu[2];
  dv18[4] = 0.0;
  dv18[7] = -current_imu[0];
  dv18[2] = -current_imu[1];
  dv18[5] = current_imu[0];
  dv18[8] = 0.0;
  for (i32 = 0; i32 < 3; i32++) {
    for (cr = 0; cr < 3; cr++) {
      dv19[cr + (i32 << 2)] = -dv18[cr + 3 * i32];
    }
  }

  for (i32 = 0; i32 < 3; i32++) {
    dv19[12 + i32] = current_imu[i32];
  }

  for (i32 = 0; i32 < 3; i32++) {
    dv19[3 + (i32 << 2)] = -current_imu[i32];
  }

  dv19[15] = 0.0;
  for (i32 = 0; i32 < 4; i32++) {
    for (cr = 0; cr < 4; cr++) {
      dv20[cr + (i32 << 2)] = 0.5 * dv19[cr + (i32 << 2)];
    }
  }

  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = xx->data[3 + i32];
  }

  emxFree_real_T(&xx);
  for (i32 = 0; i32 < 4; i32++) {
    dv21[i32] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      dv21[i32] += dv20[i32 + (cr << 2)] * b_xx[cr];
    }
  }

  for (i32 = 0; i32 < 4; i32++) {
    x4->data[3 + i32] = dv21[i32];
  }

  //  rot angle
  for (i32 = 0; i32 < 3; i32++) {
    x4->data[7 + i32] = current_imu[3 + i32];
  }

  //  velocity
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  if (1.0 > c_numStates + 1.0) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_int32_T(&r13, 2);
  i32 = r13->size[0] * r13->size[1];
  r13->size[0] = 1;
  r13->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r13, i32, (int)sizeof(int));
  for (i32 = 0; i32 < b_loop_ub; i32++) {
    r13->data[r13->size[0] * i32] = i32;
  }

  b_emxInit_real_T(&c_x, 1);
  i32 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i32, (int)sizeof(double));
  for (i32 = 0; i32 < loop_ub; i32++) {
    c_x->data[i32] = x->data[i32] + (((x1->data[i32] + 2.0 * x2->data[i32]) +
      2.0 * x3->data[i32]) + x4->data[i32]) * dt / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r13->size[1];
  for (i32 = 0; i32 < loop_ub; i32++) {
    x->data[r13->data[r13->size[0] * i32]] = c_x->data[(*(int (*)[2])r13->size)
      [0] * i32];
  }

  emxFree_real_T(&c_x);
  emxFree_int32_T(&r13);
  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i32 = 0; i32 < 4; i32++) {
    b_xx[i32] = x->data[3 + i32] / c;
  }

  for (i32 = 0; i32 < 4; i32++) {
    x->data[3 + i32] = b_xx[i32];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void last_imu_not_empty_init()
{
  last_imu_not_empty = false;
}

//
// File trailer for SLAM_pred.cpp
//
// [EOF]
//
