//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 17-Aug-2015 14:51:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_pred.h"
#include "blkdiag.h"
#include "QuatFromRotJ.h"
#include "norm.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double delayBuffer_k[84];
static double init_counter;
static double P_att[9];
static double x_att[4];

// Function Definitions

//
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double processNoise[4]
//                double IMU_measurements[23]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                const stereoParameters *cameraParams
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, const stereoParameters
          *cameraParams, emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out,
          emxArray_real_T *P_apo_out, emxArray_real_T *map_out)
{
  double B;
  double z_n_b[3];
  int outsize_idx_0;
  double y_n_b[3];
  int nrows;
  double x_n_b[3];
  double b_x_n_b[9];
  emxArray_real_T *a;
  emxArray_real_T *b;
  int itilerow;
  int ibcol;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const double b_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  double dv36[3];
  static const double b_a[3] = { -0.0132, -0.4398, 0.2673 };

  double c_a[3];
  double d_a[3];
  static const double e_a[9] = { -0.0077435500000000435, 0.0086606800000000872,
    0.99989175999999991, -0.99987860000000006, -0.010132990000000119,
    -0.0076556800000000536, 0.010066000000000075, -0.9998704,
    0.0087384500000000087 };

  int tmp_data[16];

  //  persistents for attitude estimator
  //  for coder
  // % imu hack
  // % finish imu hack
  if (!initialized_not_empty) {
    //  initialization for attitude filter
    // delayBuffer_k_1=[0;0;0;0;0;0];
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      z_n_b[outsize_idx_0] = IMU_measurements[outsize_idx_0 + 3] / B;
    }

    // m_n_b=IMU_measurements(11:13);
    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (nrows = 0; nrows < 3; nrows++) {
      y_n_b[nrows] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (nrows = 0; nrows < 3; nrows++) {
      b_x_n_b[nrows] = x_n_b[nrows] / B;
      b_x_n_b[3 + nrows] = y_n_b[nrows];
      b_x_n_b[6 + nrows] = z_n_b[nrows];
    }

    QuatFromRotJ(b_x_n_b, x_att);
    memset(&P_att[0], 0, 9U * sizeof(double));
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      P_att[outsize_idx_0 + 3 * outsize_idx_0] = 1.0;
    }

    b_emxInit_real_T(&a, 1);

    //  other initialization
    initialized_not_empty = true;
    nrows = a->size[0];
    a->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)a, nrows, (int)sizeof(double));
    a->data[0] = 0.0;
    a->data[1] = 0.0;
    a->data[2] = 0.0;
    a->data[3] = 0.0;
    a->data[4] = 0.0;
    a->data[5] = 0.0;
    a->data[6] = 1.0;
    outsize_idx_0 = (int)numPointsPerAnchor;
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      a->data[nrows + 7] = 0.0;
    }

    b_emxInit_real_T(&b, 1);
    outsize_idx_0 = a->size[0] * (int)numAnchors;
    nrows = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, nrows, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      nrows = a->size[0];
      for (itilerow = 1; itilerow <= (int)numAnchors; itilerow++) {
        ibcol = (itilerow - 1) * nrows;
        for (outsize_idx_0 = 0; outsize_idx_0 + 1 <= nrows; outsize_idx_0++) {
          b->data[ibcol + outsize_idx_0] = a->data[outsize_idx_0];
        }
      }
    }

    emxFree_real_T(&a);
    nrows = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, nrows, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    xt->data[3] = 0.0;
    xt->data[4] = 0.0;
    xt->data[5] = 0.0;
    xt->data[6] = 1.0;
    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (nrows = 0; nrows < 3; nrows++) {
      xt->data[nrows + 10] = 0.0 * IMU_measurements[nrows];
    }

    outsize_idx_0 = b->size[0];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      xt->data[nrows + 13] = b->data[nrows];
    }

    emxFree_real_T(&b);
    emxInit_real_T(&r5, 2);

    //  initial real vector
    B = numAnchors * (6.0 + numPointsPerAnchor);
    nrows = r5->size[0] * r5->size[1];
    r5->size[0] = (int)numStates;
    r5->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r5, nrows, (int)sizeof(double));
    outsize_idx_0 = (int)numStates * (int)numStates;
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      r5->data[nrows] = 0.0;
    }

    emxInit_real_T(&r6, 2);
    nrows = r6->size[0] * r6->size[1];
    r6->size[0] = (int)B;
    r6->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r6, nrows, (int)sizeof(double));
    outsize_idx_0 = (int)B * (int)B;
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      r6->data[nrows] = 0.0;
    }

    blkdiag(r5, r6, P);

    //  initial error state covariance
    emxFree_real_T(&r6);
    emxFree_real_T(&r5);
    for (nrows = 0; nrows < 3; nrows++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[outsize_idx_0 + P->size[0] * nrows] = 0.0;
      }
    }

    //  position
    for (nrows = 0; nrows < 3; nrows++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + nrows)) + 3] =
          y[outsize_idx_0 + 3 * nrows];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (nrows = 0; nrows < 3; nrows++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (6 + nrows)) + 6] = 0.0;
      }
    }

    //  velocity
    for (nrows = 0; nrows < 3; nrows++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (9 + nrows)) + 9] =
          b_y[outsize_idx_0 + 3 * nrows];
      }
    }

    //  gyro bias
  }

  for (outsize_idx_0 = 0; outsize_idx_0 < 13; outsize_idx_0++) {
    delayBuffer_k[1 + outsize_idx_0] = delayBuffer_k[outsize_idx_0];
  }

  for (nrows = 0; nrows < 3; nrows++) {
    delayBuffer_k[nrows] = 0.0 * IMU_measurements[13 + nrows];
  }

  for (nrows = 0; nrows < 3; nrows++) {
    delayBuffer_k[nrows + 3] = 0.0 * IMU_measurements[16 + nrows];
  }

  for (nrows = 0; nrows < 3; nrows++) {
    dv36[nrows] = delayBuffer_k[78 + nrows] + b_a[nrows];
  }

  for (nrows = 0; nrows < 3; nrows++) {
    c_a[nrows] = 0.0;
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      c_a[nrows] += e_a[nrows + 3 * outsize_idx_0] * dv36[outsize_idx_0];
    }

    IMU_measurements[nrows] = c_a[nrows];
    d_a[nrows] = 0.0;
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      d_a[nrows] += e_a[nrows + 3 * outsize_idx_0] * delayBuffer_k[outsize_idx_0
        + 81];
    }
  }

  for (nrows = 0; nrows < 3; nrows++) {
    IMU_measurements[3 + nrows] = d_a[nrows];
  }

  if (init_counter == 0.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (outsize_idx_0 = 0; outsize_idx_0 < 16; outsize_idx_0++) {
      updateVect[outsize_idx_0] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      outsize_idx_0 = 0;
    } else {
      outsize_idx_0 = (int)numPointsPerAnchor;
    }

    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      tmp_data[nrows] = nrows;
    }

    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      updateVect[tmp_data[nrows]] = 2.0;
    }

    for (nrows = 0; nrows < 4; nrows++) {
      xt->data[3 + nrows] = x_att[nrows];
    }

    for (nrows = 0; nrows < 3; nrows++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + nrows)) + 3] =
          P_att[outsize_idx_0 + 3 * nrows];
      }
    }

    nrows = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, nrows, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      xt_out->data[nrows] = xt->data[nrows];
    }

    for (nrows = 0; nrows < 4; nrows++) {
      xt_out->data[3 + nrows] = x_att[nrows];
    }

    nrows = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, nrows, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      P_apo_out->data[nrows] = P->data[nrows];
    }

    nrows = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, nrows, (int)sizeof(double));
    for (nrows = 0; nrows < 64; nrows++) {
      h_u_apo_out->data[nrows] = -100.0;
    }

    nrows = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, nrows, (int)sizeof(double));
    for (nrows = 0; nrows < 48; nrows++) {
      map_out->data[nrows] = rtNaN;
    }

    init_counter = 1.0;
  } else if (init_counter == 1.0) {
    SLAM_updIT(P, xt, cameraParams->CameraParameters1.RadialDistortion,
               cameraParams->CameraParameters1.FocalLength,
               cameraParams->CameraParameters1.PrincipalPoint,
               cameraParams->CameraParameters2.RadialDistortion,
               cameraParams->CameraParameters2.FocalLength,
               cameraParams->CameraParameters2.PrincipalPoint,
               cameraParams->r_lr, cameraParams->R_lr, cameraParams->R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, (1.0 - rt_powd_snf
                (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45,
               h_u_apo_out, map_out);
    nrows = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, nrows, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      xt_out->data[nrows] = xt->data[nrows];
    }

    nrows = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, nrows, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      P_apo_out->data[nrows] = P->data[nrows];
    }

    init_counter = 2.0;
  } else {
    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates);

    //  [xt,P] =  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates); 
    SLAM_updIT(P, xt, cameraParams->CameraParameters1.RadialDistortion,
               cameraParams->CameraParameters1.FocalLength,
               cameraParams->CameraParameters1.PrincipalPoint,
               cameraParams->CameraParameters2.RadialDistortion,
               cameraParams->CameraParameters2.FocalLength,
               cameraParams->CameraParameters2.PrincipalPoint,
               cameraParams->r_lr, cameraParams->R_lr, cameraParams->R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, 0.0, h_u_apo_out, map_out);
    nrows = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, nrows, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      xt_out->data[nrows] = xt->data[nrows];
    }

    nrows = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, nrows, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (nrows = 0; nrows < outsize_idx_0; nrows++) {
      P_apo_out->data[nrows] = P->data[nrows];
    }

    // % output asserts for coder
  }
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
  init_counter = 0.0;
  memset(&delayBuffer_k[0], 0, 84U * sizeof(double));
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
