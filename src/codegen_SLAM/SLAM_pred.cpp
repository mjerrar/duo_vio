//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 16:58:25
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
  int i9;
  int loop_ub;
  double dv5[9];
  double dv6[16];
  double dv7[16];
  double b_x[4];
  double dv8[4];

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  i9 = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, i9, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (i9 = 0; i9 < loop_ub; i9++) {
    x_dot->data[i9] = 0.0;
  }

  for (i9 = 0; i9 < 3; i9++) {
    x_dot->data[i9] = x->data[7 + i9];
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
  for (i9 = 0; i9 < 3; i9++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv6[loop_ub + (i9 << 2)] = -dv5[loop_ub + 3 * i9];
    }
  }

  for (i9 = 0; i9 < 3; i9++) {
    dv6[12 + i9] = meas[i9];
  }

  for (i9 = 0; i9 < 3; i9++) {
    dv6[3 + (i9 << 2)] = -meas[i9];
  }

  dv6[15] = 0.0;
  for (i9 = 0; i9 < 4; i9++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv7[loop_ub + (i9 << 2)] = 0.5 * dv6[loop_ub + (i9 << 2)];
    }
  }

  for (i9 = 0; i9 < 4; i9++) {
    b_x[i9] = x->data[3 + i9];
  }

  for (i9 = 0; i9 < 4; i9++) {
    dv8[i9] = 0.0;
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv8[i9] += dv7[i9 + (loop_ub << 2)] * b_x[loop_ub];
    }
  }

  for (i9 = 0; i9 < 4; i9++) {
    x_dot->data[3 + i9] = dv8[i9];
  }

  //  rot angle
  for (i9 = 0; i9 < 3; i9++) {
    x_dot->data[7 + i9] = meas[3 + i9];
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
  int i30;
  double b_w;
  double current_imu[6];
  int k;
  double R_cw[9];
  int cr;
  double G[180];
  static const signed char iv6[36] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv8[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double P_xx_apr[225];
  double dv13[9];
  double b_R_cw[9];
  double dv14[9];
  double b_G[225];
  static const signed char iv9[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv10[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  static const signed char iv11[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double Phi[225];
  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *b_P_apo;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double y[225];
  double c;
  double b_processNoise[12];
  double dv15[144];
  double c_G[180];
  emxArray_real_T *c_P_apo;
  emxArray_int32_T *r11;
  emxArray_int32_T *r12;
  emxArray_real_T *b_P_xs_apr;
  emxArray_real_T *b_x;
  emxArray_real_T *x1;
  emxArray_real_T *xx;
  double b_xx[4];
  double dv16[6];
  emxArray_real_T *x2;
  emxArray_real_T *x3;
  emxArray_real_T *x4;
  unsigned int unnamed_idx_0;
  double dv17[9];
  double dv18[16];
  double dv19[16];
  double dv20[4];
  emxArray_int32_T *r13;
  emxArray_real_T *c_x;

  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  %% Iterative Camera Pose optimization (EKF)
  for (i30 = 0; i30 < 3; i30++) {
    b_w = measurements_gyr_duo[i30] - x->data[10 + i30];
    current_imu[i30] = b_w;
    w[i30] = b_w;
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
  for (i30 = 0; i30 < 12; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[cr + 15 * i30] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * i30) + 3] = iv6[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * i30) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * (i30 + 3)) + 6] = -R_cw[i30 + 3 * cr];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * (i30 + 6)) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * (i30 + 9)) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 12; i30++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * i30) + 9] = iv7[cr + 3 * i30];
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 15 * i30) + 12] = iv8[cr + 3 * i30];
    }
  }

  d_eye(P_xx_apr);
  dv13[0] = 0.0;
  dv13[3] = -w[2];
  dv13[6] = w[1];
  dv13[1] = w[2];
  dv13[4] = 0.0;
  dv13[7] = -w[0];
  dv13[2] = -w[1];
  dv13[5] = w[0];
  dv13[8] = 0.0;
  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_R_cw[cr + 3 * i30] = -R_cw[i30 + 3 * cr];
    }
  }

  dv14[0] = 0.0;
  dv14[3] = -measurements_acc_duo[2];
  dv14[6] = measurements_acc_duo[1];
  dv14[1] = measurements_acc_duo[2];
  dv14[4] = 0.0;
  dv14[7] = -measurements_acc_duo[0];
  dv14[2] = -measurements_acc_duo[1];
  dv14[5] = measurements_acc_duo[0];
  dv14[8] = 0.0;
  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      R_cw[i30 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        R_cw[i30 + 3 * cr] += b_R_cw[i30 + 3 * k] * dv14[k + 3 * cr];
      }
    }
  }

  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[cr + 15 * i30] = iv9[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * i30) + 3] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 3)) + 3] = -dv13[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 6)) + 3] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 9)) + 3] = iv10[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 12)) + 3] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * i30) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 3)) + 6] = R_cw[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 6)) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 9)) + 6] = 0.0;
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * (i30 + 12)) + 6] = iv11[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * i30) + 9] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 15 * i30) + 12] = 0.0;
    }

    for (cr = 0; cr < 15; cr++) {
      Phi[cr + 15 * i30] = P_xx_apr[cr + 15 * i30] + b_G[cr + 15 * i30] * dt;
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
    i30 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = 1;
    b_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)b_P_apo, i30, (int)sizeof(double));
    for (i30 = 0; i30 < b_loop_ub; i30++) {
      cr = 0;
      while (cr <= 0) {
        b_P_apo->data[b_P_apo->size[0] * i30] = P_apo->data[P_apo->size[0] * i30];
        cr = 1;
      }
    }

    i30 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 15;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i30, (int)sizeof(double));
    for (i30 = 0; i30 < 15; i30++) {
      loop_ub = b_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i30 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 15; k++) {
          P_xs_apr->data[i30 + P_xs_apr->size[0] * cr] += Phi[i30 + 15 * k] *
            b_P_apo->data[k + b_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&b_P_apo);
  } else {
    i30 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 15;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i30, (int)sizeof(double));
    i30 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i30, (int)sizeof(double));
    k = 15 * b_loop_ub;
    for (i30 = 0; i30 < k; i30++) {
      P_xs_apr->data[i30] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 15 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 15) {
        for (ic = cr; ic + 1 <= cr + 15; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 15) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 15; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (ib / loop_ub)] != 0.0)
          {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 15; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (ib / loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 15;
        }

        br += 15;
      }
    }
  }

  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 15; cr++) {
      P_xx_apr[cr + 15 * i30] = Phi[i30 + 15 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i30 = 0; i30 < 15; i30++) {
      for (cr = 0; cr < 15; cr++) {
        y[i30 + 15 * cr] = 0.0;
        for (k = 0; k < 15; k++) {
          y[i30 + 15 * cr] += P_xs_apr->data[i30 + 15 * k] * P_xx_apr[k + 15 *
            cr];
        }
      }
    }
  } else {
    k = P_xs_apr->size[1];
    memset(&y[0], 0, 225U * sizeof(double));
    for (cr = 0; cr < 212; cr += 15) {
      for (ic = cr; ic + 1 <= cr + 15; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 212; cr += 15) {
      ar = 0;
      i30 = br + k;
      for (ib = br; ib + 1 <= i30; ib++) {
        if (P_xx_apr[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 15; ic++) {
            ia++;
            y[ic] += P_xx_apr[ib] * P_xs_apr->data[ia - 1];
          }
        }

        ar += 15;
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
  b_processNoise[9] = processNoise[3];
  b_processNoise[10] = processNoise[3];
  b_processNoise[11] = processNoise[3];
  b_diag(b_processNoise, dv15);
  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 12; cr++) {
      c_G[i30 + 15 * cr] = 0.0;
      for (k = 0; k < 12; k++) {
        c_G[i30 + 15 * cr] += G[i30 + 15 * k] * dv15[k + 12 * cr];
      }
    }

    for (cr = 0; cr < 15; cr++) {
      b_G[i30 + 15 * cr] = 0.0;
      for (k = 0; k < 12; k++) {
        b_G[i30 + 15 * cr] += c_G[i30 + 15 * k] * G[cr + 15 * k];
      }
    }
  }

  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 15; cr++) {
      P_xx_apr[cr + 15 * i30] = y[cr + 15 * i30] + b_G[cr + 15 * i30] * c;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i30 = 0;
    cr = 0;
  } else {
    i30 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    k = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = cr - i30;
    emxEnsureCapacity((emxArray__common *)c_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i30;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        c_P_apo->data[c_P_apo->size[0] * cr] = P_apo->data[P_apo->size[0] * (i30
          + cr)];
        k = 1;
      }
    }

    i30 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 15;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i30, (int)sizeof(double));
    for (i30 = 0; i30 < 15; i30++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i30 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 15; k++) {
          P_xs_apr->data[i30 + P_xs_apr->size[0] * cr] += Phi[i30 + 15 * k] *
            c_P_apo->data[k + c_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&c_P_apo);
  } else {
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 15;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = cr - i30;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 15 * (cr - i30);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i30 == 0) {
    } else {
      b_loop_ub = 15 * ((cr - i30) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 15) {
        for (ic = cr; ic + 1 <= cr + 15; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 15) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 15; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i30 + ib / loop_ub)]
              != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 15; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i30 + ib / loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 15;
        }

        br += 15;
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
  i30 = r11->size[0];
  r11->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r11, i30, (int)sizeof(int));
  for (i30 = 0; i30 < loop_ub; i30++) {
    r11->data[i30] = i30;
  }

  emxInit_int32_T(&r12, 1);
  i30 = r12->size[0];
  r12->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r12, i30, (int)sizeof(int));
  for (i30 = 0; i30 < b_loop_ub; i30++) {
    r12->data[i30] = i30;
  }

  for (i30 = 0; i30 < 15; i30++) {
    for (cr = 0; cr < 15; cr++) {
      b_G[cr + 15 * i30] = (P_xx_apr[cr + 15 * i30] + P_xx_apr[i30 + 15 * cr]) /
        2.0;
    }
  }

  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i30 = 0; i30 < b_loop_ub; i30++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i30]] = b_G[cr + k *
        i30];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i30 = 0;
    cr = 0;
  } else {
    i30 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  k = r11->size[0];
  r11->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r11, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r11->data[k] = k;
  }

  k = r12->size[0];
  r12->size[0] = cr - i30;
  emxEnsureCapacity((emxArray__common *)r12, k, (int)sizeof(int));
  loop_ub = cr - i30;
  for (cr = 0; cr < loop_ub; cr++) {
    r12->data[cr] = i30 + cr;
  }

  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i30 = 0; i30 < b_loop_ub; i30++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i30]] =
        P_xs_apr->data[cr + k * i30];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i30 = 0;
    cr = 0;
  } else {
    i30 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r11->size[0];
  r11->size[0] = cr - i30;
  emxEnsureCapacity((emxArray__common *)r11, k, (int)sizeof(int));
  b_loop_ub = cr - i30;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r11->data[cr] = i30 + cr;
  }

  i30 = r12->size[0];
  r12->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r12, i30, (int)sizeof(int));
  for (i30 = 0; i30 < loop_ub; i30++) {
    r12->data[i30] = i30;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i30 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 15;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i30, (int)sizeof(double));
  for (i30 = 0; i30 < 15; i30++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i30] = P_xs_apr->data[i30 +
        P_xs_apr->size[0] * cr];
    }
  }

  emxFree_real_T(&P_xs_apr);
  k = r11->size[0];
  b_loop_ub = r12->size[0];
  for (i30 = 0; i30 < b_loop_ub; i30++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r11->data[cr] + P_apo->size[0] * r12->data[i30]] =
        b_P_xs_apr->data[cr + k * i30];
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
  i30 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i30, (int)sizeof(double));
  for (i30 = 0; i30 < loop_ub; i30++) {
    b_x->data[i30] = x->data[i30];
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
  i30 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i30, (int)sizeof(double));
  for (i30 = 0; i30 < loop_ub; i30++) {
    xx->data[i30] = x->data[i30] + x1->data[i30] * dt / 2.0;
  }

  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30];
  }

  c = b_norm(b_xx);
  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30] / c;
  }

  for (i30 = 0; i30 < 4; i30++) {
    xx->data[3 + i30] = b_xx[i30];
  }

  for (k = 0; k < 6; k++) {
    dv16[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  b_emxInit_real_T(&x2, 1);
  dxdt_dPdt(dv16, xx, x2);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i30 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i30, (int)sizeof(double));
  for (i30 = 0; i30 < loop_ub; i30++) {
    xx->data[i30] = x->data[i30] + x2->data[i30] * dt / 2.0;
  }

  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30];
  }

  c = b_norm(b_xx);
  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30] / c;
  }

  for (i30 = 0; i30 < 4; i30++) {
    xx->data[3 + i30] = b_xx[i30];
  }

  for (k = 0; k < 6; k++) {
    dv16[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  b_emxInit_real_T(&x3, 1);
  dxdt_dPdt(dv16, xx, x3);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i30 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i30, (int)sizeof(double));
  for (i30 = 0; i30 < loop_ub; i30++) {
    xx->data[i30] = x->data[i30] + x3->data[i30] * dt;
  }

  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30];
  }

  c = b_norm(b_xx);
  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30] / c;
  }

  for (i30 = 0; i30 < 4; i30++) {
    xx->data[3 + i30] = b_xx[i30];
  }

  b_emxInit_real_T(&x4, 1);

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)xx->size[0];
  i30 = x4->size[0];
  x4->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x4, i30, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (i30 = 0; i30 < loop_ub; i30++) {
    x4->data[i30] = 0.0;
  }

  for (i30 = 0; i30 < 3; i30++) {
    x4->data[i30] = xx->data[7 + i30];
  }

  //  position
  dv17[0] = 0.0;
  dv17[3] = -current_imu[2];
  dv17[6] = current_imu[1];
  dv17[1] = current_imu[2];
  dv17[4] = 0.0;
  dv17[7] = -current_imu[0];
  dv17[2] = -current_imu[1];
  dv17[5] = current_imu[0];
  dv17[8] = 0.0;
  for (i30 = 0; i30 < 3; i30++) {
    for (cr = 0; cr < 3; cr++) {
      dv18[cr + (i30 << 2)] = -dv17[cr + 3 * i30];
    }
  }

  for (i30 = 0; i30 < 3; i30++) {
    dv18[12 + i30] = current_imu[i30];
  }

  for (i30 = 0; i30 < 3; i30++) {
    dv18[3 + (i30 << 2)] = -current_imu[i30];
  }

  dv18[15] = 0.0;
  for (i30 = 0; i30 < 4; i30++) {
    for (cr = 0; cr < 4; cr++) {
      dv19[cr + (i30 << 2)] = 0.5 * dv18[cr + (i30 << 2)];
    }
  }

  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = xx->data[3 + i30];
  }

  emxFree_real_T(&xx);
  for (i30 = 0; i30 < 4; i30++) {
    dv20[i30] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      dv20[i30] += dv19[i30 + (cr << 2)] * b_xx[cr];
    }
  }

  for (i30 = 0; i30 < 4; i30++) {
    x4->data[3 + i30] = dv20[i30];
  }

  //  rot angle
  for (i30 = 0; i30 < 3; i30++) {
    x4->data[7 + i30] = current_imu[3 + i30];
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
  i30 = r13->size[0] * r13->size[1];
  r13->size[0] = 1;
  r13->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r13, i30, (int)sizeof(int));
  for (i30 = 0; i30 < b_loop_ub; i30++) {
    r13->data[r13->size[0] * i30] = i30;
  }

  b_emxInit_real_T(&c_x, 1);
  i30 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i30, (int)sizeof(double));
  for (i30 = 0; i30 < loop_ub; i30++) {
    c_x->data[i30] = x->data[i30] + (((x1->data[i30] + 2.0 * x2->data[i30]) +
      2.0 * x3->data[i30]) + x4->data[i30]) * dt / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r13->size[1];
  for (i30 = 0; i30 < loop_ub; i30++) {
    x->data[r13->data[r13->size[0] * i30]] = c_x->data[(*(int (*)[2])r13->size)
      [0] * i30];
  }

  emxFree_real_T(&c_x);
  emxFree_int32_T(&r13);
  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i30 = 0; i30 < 4; i30++) {
    b_xx[i30] = x->data[3 + i30] / c;
  }

  for (i30 = 0; i30 < 4; i30++) {
    x->data[3 + i30] = b_xx[i30];
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
