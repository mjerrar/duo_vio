//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 18:46:47
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "Att_upd.h"
#include "quatPlusThetaJ.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_pred.h"
#include "blkdiag.h"
#include "repmat.h"
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

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(double)(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

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
//                boolean_T resetFlag
//                const double ref[4]
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
//                double u_out[4]
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, const stereoParameters
          *cameraParams, boolean_T resetFlag, const double ref[4],
          emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
          *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int i;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  double B;
  double z_n_b[3];
  double y_n_b[3];
  int i12;
  double x_n_b[3];
  double b_x_n_b[9];
  static const double dv27[9] = { 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.001 };

  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const double b_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  double R_bw[9];
  double a[9];
  double b_R_bw[3];
  double Phi[9];
  double dq[4];
  double dv28[16];
  double b_dq[4];
  double b_Phi[9];
  double b_a[9];
  int i13;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char c_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int tmp_data[16];
  static const signed char d_a[9] = { 1, 0, 0, 0, 0, -1, 0, 1, 0 };

  double yaw;
  double dv29[2];
  double dv30[2];
  double d5;
  double dv31[2];
  double d6;
  double u_out_yaw;
  double u_out_x[3];
  static const double e_a[3] = { -0.0132, -0.4398, 0.2673 };

  double f_a[3];
  static const double g_a[9] = { -0.0077435500000000435, 0.0086606800000000872,
    0.99989175999999991, -0.99987860000000006, -0.010132990000000119,
    -0.0076556800000000536, 0.010066000000000075, -0.9998704,
    0.0087384500000000087 };

  //  persistents for attitude estimator
  //  for coder
  // % imu hack
  // % finish imu hack
  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  if (resetFlag) {
    for (i = 0; i < 16; i++) {
      updateVect[i] = 0.0;
    }
  }

  b_emxInit_real_T(&r4, 1);
  emxInit_real_T(&r5, 2);
  emxInit_real_T(&r6, 2);
  b_emxInit_real_T(&r7, 1);
  if ((!initialized_not_empty) || resetFlag) {
    //  initialization for attitude filter
    init_counter = 0.0;
    memset(&delayBuffer_k[0], 0, 84U * sizeof(double));

    // delayBuffer_k_1=[0;0;0;0;0;0];
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (i = 0; i < 3; i++) {
      z_n_b[i] = IMU_measurements[i + 3] / B;
    }

    // m_n_b=IMU_measurements(11:13);
    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (i12 = 0; i12 < 3; i12++) {
      y_n_b[i12] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (i12 = 0; i12 < 3; i12++) {
      b_x_n_b[i12] = x_n_b[i12] / B;
      b_x_n_b[3 + i12] = y_n_b[i12];
      b_x_n_b[6 + i12] = z_n_b[i12];
    }

    QuatFromRotJ(b_x_n_b, x_att);
    memcpy(&P_att[0], &dv27[0], 9U * sizeof(double));

    //  other initialization
    initialized_not_empty = true;
    i12 = r7->size[0];
    r7->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)r7, i12, (int)sizeof(double));
    r7->data[0] = 0.0;
    r7->data[1] = 0.0;
    r7->data[2] = 0.0;
    r7->data[3] = 0.0;
    r7->data[4] = 0.0;
    r7->data[5] = 0.0;
    r7->data[6] = 1.0;
    i = (int)numPointsPerAnchor;
    for (i12 = 0; i12 < i; i12++) {
      r7->data[i12 + 7] = 0.0;
    }

    repmat(r7, numAnchors, r4);
    i12 = xt->size[0];
    xt->size[0] = 13 + r4->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i12, (int)sizeof(double));
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
    for (i12 = 0; i12 < 3; i12++) {
      xt->data[i12 + 10] = 0.0 * IMU_measurements[i12];
    }

    i = r4->size[0];
    for (i12 = 0; i12 < i; i12++) {
      xt->data[i12 + 13] = r4->data[i12];
    }

    //  initial real vector
    B = numAnchors * (6.0 + numPointsPerAnchor);
    i12 = r5->size[0] * r5->size[1];
    r5->size[0] = (int)numStates;
    r5->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r5, i12, (int)sizeof(double));
    i = (int)numStates * (int)numStates;
    for (i12 = 0; i12 < i; i12++) {
      r5->data[i12] = 0.0;
    }

    i12 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)B;
    r6->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r6, i12, (int)sizeof(double));
    i = (int)B * (int)B;
    for (i12 = 0; i12 < i; i12++) {
      r6->data[i12] = 0.0;
    }

    blkdiag(r5, r6, P);

    //  initial error state covariance
    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P->data[i + P->size[0] * i12] = 0.0;
      }
    }

    //  position
    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (3 + i12)) + 3] = y[i + 3 * i12];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (6 + i12)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (9 + i12)) + 9] = b_y[i + 3 * i12];
      }
    }

    //  gyro bias
  }

  emxFree_real_T(&r7);
  emxFree_real_T(&r6);
  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
  if (init_counter < 10.0) {
    //      zw = R_ci * IMU_measurements(14:16);
    //      za = R_ci * IMU_measurements(17:19);
    // ATT_PRED Prediction step of the attitude estimator
    //    INPUT ARGUMENTS:
    //    - x:  The current estimated attitude (JPL quaternion)
    //    - P:  The state covariance matrix (3 x 3)
    //    - w:  The current Gyro measurement (3 x 1)
    //    - q:  The proecss noise
    //    - dt: The time step
    memset(&R_bw[0], 0, 9U * sizeof(double));
    a[0] = 0.0;
    a[3] = -IMU_measurements[2];
    a[6] = IMU_measurements[1];
    a[1] = IMU_measurements[2];
    a[4] = 0.0;
    a[7] = -IMU_measurements[0];
    a[2] = -IMU_measurements[1];
    a[5] = IMU_measurements[0];
    a[8] = 0.0;
    B = dt * dt;
    for (i = 0; i < 3; i++) {
      R_bw[i + 3 * i] = 1.0;
      for (i12 = 0; i12 < 3; i12++) {
        Phi[i12 + 3 * i] = R_bw[i12 + 3 * i] + -a[i12 + 3 * i] * dt;
      }

      b_R_bw[i] = IMU_measurements[i] * dt;
    }

    quatPlusThetaJ(b_R_bw, dq);
    dv28[0] = x_att[3];
    dv28[4] = -x_att[2];
    dv28[8] = x_att[1];
    dv28[12] = x_att[0];
    dv28[1] = x_att[2];
    dv28[5] = x_att[3];
    dv28[9] = -x_att[0];
    dv28[13] = x_att[1];
    dv28[2] = -x_att[1];
    dv28[6] = x_att[0];
    dv28[10] = x_att[3];
    dv28[14] = x_att[2];
    dv28[3] = -x_att[0];
    dv28[7] = -x_att[1];
    dv28[11] = -x_att[2];
    dv28[15] = x_att[3];
    b_dq[0] = dq[0];
    b_dq[1] = dq[1];
    b_dq[2] = dq[2];
    b_dq[3] = dq[3];
    for (i12 = 0; i12 < 4; i12++) {
      x_att[i12] = 0.0;
      for (i = 0; i < 4; i++) {
        x_att[i12] += dv28[i12 + (i << 2)] * b_dq[i];
      }
    }

    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        b_x_n_b[i12 + 3 * i] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          b_x_n_b[i12 + 3 * i] += Phi[i12 + 3 * i13] * P_att[i13 + 3 * i];
        }

        a[i12 + 3 * i] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          a[i12 + 3 * i] += (double)c_a[i12 + 3 * i13] * ((double)b[i13 + 3 * i]
            * processNoise[1] * B);
        }
      }

      for (i = 0; i < 3; i++) {
        b_Phi[i12 + 3 * i] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          b_Phi[i12 + 3 * i] += b_x_n_b[i12 + 3 * i13] * Phi[i + 3 * i13];
        }

        b_a[i12 + 3 * i] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          b_a[i12 + 3 * i] += a[i12 + 3 * i13] * (double)c_a[i13 + 3 * i];
        }
      }
    }

    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P_att[i + 3 * i12] = b_Phi[i + 3 * i12] + b_a[i + 3 * i12];
      }
    }

    Att_upd(x_att, P_att, *(double (*)[3])&IMU_measurements[3], processNoise[0],
            dt);
    i12 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i12, (int)sizeof(double));
    i = xt->size[0];
    for (i12 = 0; i12 < i; i12++) {
      xt_out->data[i12] = xt->data[i12];
    }

    for (i12 = 0; i12 < 4; i12++) {
      xt_out->data[3 + i12] = x_att[i12];
    }

    i12 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i12, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i12 = 0; i12 < i; i12++) {
      P_apo_out->data[i12] = 1.0E-5 * P->data[i12];
    }

    i12 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i12, (int)sizeof(double));
    for (i12 = 0; i12 < 64; i12++) {
      h_u_apo_out->data[i12] = -100.0;
    }

    i12 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i12, (int)sizeof(double));
    for (i12 = 0; i12 < 48; i12++) {
      map_out->data[i12] = rtNaN;
    }

    init_counter++;
  } else if (init_counter == 10.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (i = 0; i < 16; i++) {
      updateVect[i] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      i = 0;
    } else {
      i = (int)numPointsPerAnchor;
    }

    for (i12 = 0; i12 < i; i12++) {
      tmp_data[i12] = i12;
    }

    for (i12 = 0; i12 < i; i12++) {
      updateVect[tmp_data[i12]] = 2.0;
    }

    for (i12 = 0; i12 < 4; i12++) {
      xt->data[3 + i12] = x_att[i12];
    }

    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (3 + i12)) + 3] = P_att[i + 3 * i12];
      }
    }

    i12 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i12, (int)sizeof(double));
    i = xt->size[0];
    for (i12 = 0; i12 < i; i12++) {
      xt_out->data[i12] = xt->data[i12];
    }

    for (i12 = 0; i12 < 4; i12++) {
      xt_out->data[3 + i12] = x_att[i12];
    }

    i12 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i12, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i12 = 0; i12 < i; i12++) {
      P_apo_out->data[i12] = P->data[i12];
    }

    i12 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i12, (int)sizeof(double));
    for (i12 = 0; i12 < 64; i12++) {
      h_u_apo_out->data[i12] = -100.0;
    }

    i12 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i12, (int)sizeof(double));
    for (i12 = 0; i12 < 48; i12++) {
      map_out->data[i12] = rtNaN;
    }

    init_counter = 11.0;
  } else if (init_counter == 11.0) {
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
    i12 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i12, (int)sizeof(double));
    i = xt->size[0];
    for (i12 = 0; i12 < i; i12++) {
      xt_out->data[i12] = xt->data[i12];
    }

    i12 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i12, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i12 = 0; i12 < i; i12++) {
      P_apo_out->data[i12] = P->data[i12];
    }

    init_counter = 12.0;
  } else {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    //  rotation from camera to control/body frame
    a[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) - xt->data[5]
            * xt->data[5]) + xt->data[6] * xt->data[6];
    a[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    a[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    a[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    a[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
            xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    a[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    a[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    a[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    a[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
            xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (i12 = 0; i12 < 3; i12++) {
      for (i = 0; i < 3; i++) {
        R_bw[i12 + 3 * i] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          R_bw[i12 + 3 * i] += (double)d_a[i12 + 3 * i13] * a[i13 + 3 * i];
        }
      }
    }

    yaw = rt_atan2d_snf(R_bw[3], R_bw[0]);
    dv29[0] = xt->data[0] - ref[0];
    dv29[1] = xt->data[7];
    B = 0.0;
    for (i12 = 0; i12 < 2; i12++) {
      B += -K_pos[i12] * dv29[i12];
    }

    dv30[0] = xt->data[1] - ref[1];
    dv30[1] = xt->data[8];
    d5 = 0.0;
    for (i12 = 0; i12 < 2; i12++) {
      d5 += -K_pos[i12] * dv30[i12];
    }

    dv31[0] = xt->data[2] - ref[2];
    dv31[1] = xt->data[9];
    d6 = 0.0;
    for (i12 = 0; i12 < 2; i12++) {
      d6 += -K_pos[i12] * dv31[i12];
    }

    u_out_yaw = -K_yaw * (yaw - ref[3]);
    u_out_x[0] = B;
    u_out_x[1] = d5;
    u_out_x[2] = d6;
    for (i12 = 0; i12 < 3; i12++) {
      b_R_bw[i12] = 0.0;
      for (i = 0; i < 3; i++) {
        b_R_bw[i12] += R_bw[i12 + 3 * i] * u_out_x[i];
      }
    }

    for (i12 = 0; i12 < 3; i12++) {
      u_out[i12] = b_R_bw[i12];
    }

    u_out[3] = u_out_yaw;
    p_fprintf(xt->data[0] - ref[0], xt->data[1] - ref[1], xt->data[2] - ref[2],
              yaw - ref[3], B, d5, d6, u_out_yaw);
    for (i = 0; i < 13; i++) {
      for (i12 = 0; i12 < 6; i12++) {
        delayBuffer_k[i12 + 6 * (13 - i)] = delayBuffer_k[i12 + 6 * (12 - i)];
      }
    }

    for (i12 = 0; i12 < 3; i12++) {
      delayBuffer_k[i12] = IMU_measurements[13 + i12];
    }

    for (i12 = 0; i12 < 3; i12++) {
      delayBuffer_k[i12 + 3] = IMU_measurements[16 + i12];
    }

    for (i12 = 0; i12 < 3; i12++) {
      b_R_bw[i12] = delayBuffer_k[78 + i12] + e_a[i12];
    }

    for (i12 = 0; i12 < 3; i12++) {
      u_out_x[i12] = 0.0;
      for (i = 0; i < 3; i++) {
        u_out_x[i12] += g_a[i12 + 3 * i] * b_R_bw[i];
      }

      IMU_measurements[i12] = u_out_x[i12];
      f_a[i12] = 0.0;
      for (i = 0; i < 3; i++) {
        f_a[i12] += g_a[i12 + 3 * i] * delayBuffer_k[i + 81];
      }
    }

    for (i12 = 0; i12 < 3; i12++) {
      IMU_measurements[3 + i12] = f_a[i12];
    }

    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates, u_out);

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
    i12 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i12, (int)sizeof(double));
    i = xt->size[0];
    for (i12 = 0; i12 < i; i12++) {
      xt_out->data[i12] = xt->data[i12];
    }

    i12 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i12, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i12 = 0; i12 < i; i12++) {
      P_apo_out->data[i12] = P->data[i12];
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
