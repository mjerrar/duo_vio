//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "SLAM_updIT.h"
#include "SLAM_pred_euler.h"
#include "QuatFromRotJ.h"
#include "norm.h"
#include "getCameraParams.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static f_struct_T cameraparams;

// Function Definitions

//
// Arguments    : double updateVect[32]
//                const double z_all_l[64]
//                const double z_all_r[64]
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                const double imNoise[4]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                double b_map[96]
// Return Type  : void
//
void SLAM(double updateVect[32], const double z_all_l[64], const double z_all_r
          [64], double dt, const double processNoise[4], const double
          IMU_measurements[9], const double imNoise[4], double, double,
          emxArray_real_T *h_u_apo, emxArray_real_T *xt_out, emxArray_real_T
          *P_apo_out, double b_map[96])
{
  double B;
  double z_n_b[3];
  int outsize_idx_0;
  double y_n_b[3];
  int i11;
  double x_n_b[3];
  double b_x_n_b[9];
  emxArray_real_T *b;
  double q1[4];
  int outsize_idx_1;
  int k;
  static const signed char a[7] = { 0, 0, 100, 0, 0, 0, 1 };

  int loop_ub;
  int i12;
  double unusedU1[96];
  if (!initialized_not_empty) {
    getCameraParams(&cameraparams);
    initialized_not_empty = true;
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      z_n_b[outsize_idx_0] = IMU_measurements[outsize_idx_0 + 3] / B;
    }

    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (i11 = 0; i11 < 3; i11++) {
      y_n_b[i11] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (i11 = 0; i11 < 3; i11++) {
      b_x_n_b[i11] = x_n_b[i11] / B;
      b_x_n_b[3 + i11] = y_n_b[i11];
      b_x_n_b[6 + i11] = z_n_b[i11];
    }

    b_emxInit_real_T(&b, 1);
    QuatFromRotJ(b_x_n_b, q1);

    //  initialize the trail poses with large values to make sure we create
    //  a new pose at the start
    outsize_idx_0 = 7 * (int)trailSize;
    i11 = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, i11, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      for (outsize_idx_0 = 1; outsize_idx_0 <= (int)trailSize; outsize_idx_0++)
      {
        outsize_idx_1 = (outsize_idx_0 - 1) * 7;
        for (k = 0; k < 7; k++) {
          b->data[outsize_idx_1 + k] = a[k];
        }
      }
    }

    i11 = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i11, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    for (i11 = 0; i11 < 4; i11++) {
      xt->data[i11 + 3] = q1[i11];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    outsize_idx_0 = b->size[0];
    for (i11 = 0; i11 < outsize_idx_0; i11++) {
      xt->data[i11 + 13] = b->data[i11];
    }

    //  initial real vector
    B = 6.0 * trailSize;
    outsize_idx_0 = (int)numStates + (int)B;
    outsize_idx_1 = (int)numStates + (int)B;
    i11 = P->size[0] * P->size[1];
    P->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)P, i11, (int)sizeof(double));
    i11 = P->size[0] * P->size[1];
    P->size[1] = outsize_idx_1;
    emxEnsureCapacity((emxArray__common *)P, i11, (int)sizeof(double));
    outsize_idx_0 *= outsize_idx_1;
    for (i11 = 0; i11 < outsize_idx_0; i11++) {
      P->data[i11] = 0.0;
    }

    if ((int)numStates > 0) {
      outsize_idx_0 = (int)numStates;
      for (i11 = 0; i11 < outsize_idx_0; i11++) {
        loop_ub = (int)numStates;
        for (outsize_idx_1 = 0; outsize_idx_1 < loop_ub; outsize_idx_1++) {
          P->data[outsize_idx_1 + P->size[0] * i11] = 0.0;
        }
      }
    }

    if ((int)B > 0) {
      if ((int)numStates + 1 > (int)numStates + (int)B) {
        i11 = 1;
      } else {
        i11 = (int)numStates + 1;
      }

      if ((int)numStates + 1 > (int)numStates + (int)B) {
        outsize_idx_1 = 1;
      } else {
        outsize_idx_1 = (int)numStates + 1;
      }

      outsize_idx_0 = (int)B;
      for (k = 0; k < outsize_idx_0; k++) {
        loop_ub = (int)B;
        for (i12 = 0; i12 < loop_ub; i12++) {
          P->data[((i11 + i12) + P->size[0] * ((outsize_idx_1 + k) - 1)) - 1] =
            0.0;
        }
      }
    }

    //  initial error state covariance
    // P_apr(10:12,10:12) = 1*eye(3);
    for (i11 = 0; i11 < 3; i11++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
        P->data[outsize_idx_1 + P->size[0] * i11] = 0.0;
      }
    }

    for (i11 = 0; i11 < 3; i11++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
        P->data[(outsize_idx_1 + P->size[0] * (3 + i11)) + 3] = 1.0;
      }
    }

    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, updateVect, z_all_l,
               z_all_r, imNoise, b, unusedU1);
    emxFree_real_T(&b);
  }

  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates);
  SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, updateVect, z_all_l,
             z_all_r, imNoise, h_u_apo, b_map);
  i11 = xt_out->size[0];
  xt_out->size[0] = xt->size[0];
  emxEnsureCapacity((emxArray__common *)xt_out, i11, (int)sizeof(double));
  outsize_idx_0 = xt->size[0];
  for (i11 = 0; i11 < outsize_idx_0; i11++) {
    xt_out->data[i11] = xt->data[i11];
  }

  i11 = P_apo_out->size[0] * P_apo_out->size[1];
  P_apo_out->size[0] = P->size[0];
  P_apo_out->size[1] = P->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo_out, i11, (int)sizeof(double));
  outsize_idx_0 = P->size[0] * P->size[1];
  for (i11 = 0; i11 < outsize_idx_0; i11++) {
    P_apo_out->data[i11] = P->data[i11];
  }

  // % output asserts for coder
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_free()
{
  emxFree_real_T(&P);
  emxFree_real_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_init()
{
  emxInit_real_T(&P, 2);
  b_emxInit_real_T(&xt, 1);
}

//
// Arguments    : void
// Return Type  : void
//
void initialized_not_empty_init()
{
  initialized_not_empty = false;
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
