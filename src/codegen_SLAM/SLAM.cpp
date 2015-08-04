//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Aug-2015 14:03:28
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "quatPlusThetaJ.h"
#include "Att_pred.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_pred.h"
#include "blkdiag.h"
#include "repmat.h"
#include "getCameraParams.h"
#include "eye.h"
#include "QuatFromRotJ.h"
#include "norm.h"
#include "cross.h"
#include "rdivide.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static f_struct_T cameraparams;
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
//                const double IMU_measurements[13]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], const double
          IMU_measurements[13], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo_out,
          emxArray_real_T *xt_out, emxArray_real_T *P_apo_out, emxArray_real_T
          *map_out)
{
  double maxval;
  double z_n_b[3];
  int rtemp;
  double y_n_b[3];
  static const double dv331[3] = { 1.0, 0.0, 0.0 };

  double b_y_n_b[3];
  double x_n_b[3];
  double b_x_n_b[9];
  int r1;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  static const signed char y[9] = { 100, 0, 0, 0, 100, 0, 0, 0, 100 };

  static const double b_y[9] = { 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.001 };

  double S[9];
  double H[9];
  double a21;
  int k;
  double b_S[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double c_y[9];
  int r2;
  int r3;
  double K[9];
  double b_IMU_measurements[3];
  double q[4];
  double dv332[16];
  double b_q[4];
  int tmp_data[16];

  //  persistents for attitude estimator
  //  for coder
  if (!initialized_not_empty) {
    //  initialization for attitude filter
    maxval = norm(*(double (*)[3])&IMU_measurements[3]);
    for (rtemp = 0; rtemp < 3; rtemp++) {
      z_n_b[rtemp] = IMU_measurements[rtemp + 3] / maxval;
    }

    // m_n_b=IMU_measurements(11:13);
    cross(z_n_b, dv331, y_n_b);
    for (rtemp = 0; rtemp < 3; rtemp++) {
      b_y_n_b[rtemp] = y_n_b[rtemp];
    }

    rdivide(b_y_n_b, norm(y_n_b), y_n_b);
    cross(y_n_b, z_n_b, x_n_b);
    maxval = norm(x_n_b);
    for (r1 = 0; r1 < 3; r1++) {
      b_x_n_b[r1] = x_n_b[r1] / maxval;
      b_x_n_b[3 + r1] = y_n_b[r1];
      b_x_n_b[6 + r1] = z_n_b[r1];
    }

    b_emxInit_real_T(&r4, 1);
    QuatFromRotJ(b_x_n_b, x_att);
    eye(P_att);

    //  other initialization
    initialized_not_empty = true;
    getCameraParams(&cameraparams);
    r1 = r4->size[0];
    r4->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)r4, r1, (int)sizeof(double));
    r4->data[0] = 0.0;
    r4->data[1] = 0.0;
    r4->data[2] = 0.0;
    r4->data[3] = 0.0;
    r4->data[4] = 0.0;
    r4->data[5] = 0.0;
    r4->data[6] = 1.0;
    rtemp = (int)numPointsPerAnchor;
    for (r1 = 0; r1 < rtemp; r1++) {
      r4->data[r1 + 7] = 0.0;
    }

    b_emxInit_real_T(&r5, 1);
    repmat(r4, numAnchors, r5);
    r1 = xt->size[0];
    xt->size[0] = 13 + r5->size[0];
    emxEnsureCapacity((emxArray__common *)xt, r1, (int)sizeof(double));
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
    emxFree_real_T(&r4);
    for (r1 = 0; r1 < 3; r1++) {
      xt->data[r1 + 10] = IMU_measurements[r1];
    }

    rtemp = r5->size[0];
    for (r1 = 0; r1 < rtemp; r1++) {
      xt->data[r1 + 13] = r5->data[r1];
    }

    emxFree_real_T(&r5);
    emxInit_real_T(&r6, 2);

    //  initial real vector
    maxval = numAnchors * (6.0 + numPointsPerAnchor);
    r1 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)numStates;
    r6->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r6, r1, (int)sizeof(double));
    rtemp = (int)numStates * (int)numStates;
    for (r1 = 0; r1 < rtemp; r1++) {
      r6->data[r1] = 0.0;
    }

    emxInit_real_T(&r7, 2);
    r1 = r7->size[0] * r7->size[1];
    r7->size[0] = (int)maxval;
    r7->size[1] = (int)maxval;
    emxEnsureCapacity((emxArray__common *)r7, r1, (int)sizeof(double));
    rtemp = (int)maxval * (int)maxval;
    for (r1 = 0; r1 < rtemp; r1++) {
      r7->data[r1] = 0.0;
    }

    blkdiag(r6, r7, P);

    //  initial error state covariance
    emxFree_real_T(&r7);
    emxFree_real_T(&r6);
    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P->data[rtemp + P->size[0] * r1] = 0.0;
      }
    }

    //  position
    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P->data[(rtemp + P->size[0] * (3 + r1)) + 3] = y[rtemp + 3 * r1];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P->data[(rtemp + P->size[0] * (6 + r1)) + 6] = 0.0;
      }
    }

    //  velocity
    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P->data[(rtemp + P->size[0] * (9 + r1)) + 9] = b_y[rtemp + 3 * r1];
      }
    }

    //  gyro bias
  }

  if (init_counter < 1.0) {
    Att_pred(x_att, P_att, *(double (*)[3])&IMU_measurements[0], processNoise[1],
             dt);

    // ATT_UPD Update step of the attitude estimator
    //    INPUT ARGUMENTS:
    //    - x:  The current estimated attitude (JPL quaternion)
    //    - P:  The state covariance matrix (3 x 3)
    //    - z:  The current accelerometer measurement (3 x 1)
    //    - n:  The measurement noise
    //    - dt: The time step
    //  predicted measurement
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    S[0] = ((x_att[0] * x_att[0] - x_att[1] * x_att[1]) - x_att[2] * x_att[2]) +
      x_att[3] * x_att[3];
    S[3] = 2.0 * (x_att[0] * x_att[1] + x_att[2] * x_att[3]);
    S[6] = 2.0 * (x_att[0] * x_att[2] - x_att[1] * x_att[3]);
    S[1] = 2.0 * (x_att[0] * x_att[1] - x_att[2] * x_att[3]);
    S[4] = ((-(x_att[0] * x_att[0]) + x_att[1] * x_att[1]) - x_att[2] * x_att[2])
      + x_att[3] * x_att[3];
    S[7] = 2.0 * (x_att[1] * x_att[2] + x_att[0] * x_att[3]);
    S[2] = 2.0 * (x_att[0] * x_att[2] + x_att[1] * x_att[3]);
    S[5] = 2.0 * (x_att[1] * x_att[2] - x_att[0] * x_att[3]);
    S[8] = ((-(x_att[0] * x_att[0]) - x_att[1] * x_att[1]) + x_att[2] * x_att[2])
      + x_att[3] * x_att[3];
    for (r1 = 0; r1 < 3; r1++) {
      z_n_b[r1] = S[6 + r1] * 9.81;
    }

    H[0] = 0.0;
    H[3] = -z_n_b[2];
    H[6] = z_n_b[1];
    H[1] = z_n_b[2];
    H[4] = 0.0;
    H[7] = -z_n_b[0];
    H[2] = -z_n_b[1];
    H[5] = z_n_b[0];
    H[8] = 0.0;
    a21 = dt * dt;
    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        b_x_n_b[r1 + 3 * rtemp] = 0.0;
        for (k = 0; k < 3; k++) {
          b_x_n_b[r1 + 3 * rtemp] += H[r1 + 3 * k] * P_att[k + 3 * rtemp];
        }
      }
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        maxval = 0.0;
        for (k = 0; k < 3; k++) {
          maxval += b_x_n_b[r1 + 3 * k] * H[rtemp + 3 * k];
        }

        b_S[r1 + 3 * rtemp] = maxval + (double)b[r1 + 3 * rtemp] * processNoise
          [0] * a21;
      }
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        c_y[r1 + 3 * rtemp] = 0.0;
        for (k = 0; k < 3; k++) {
          c_y[r1 + 3 * rtemp] += P_att[r1 + 3 * k] * H[rtemp + 3 * k];
        }
      }
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = fabs(b_S[0]);
    a21 = fabs(b_S[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }

    if (fabs(b_S[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    b_S[r2] /= b_S[r1];
    b_S[r3] /= b_S[r1];
    b_S[3 + r2] -= b_S[r2] * b_S[3 + r1];
    b_S[3 + r3] -= b_S[r3] * b_S[3 + r1];
    b_S[6 + r2] -= b_S[r2] * b_S[6 + r1];
    b_S[6 + r3] -= b_S[r3] * b_S[6 + r1];
    if (fabs(b_S[3 + r3]) > fabs(b_S[3 + r2])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    b_S[3 + r3] /= b_S[3 + r2];
    b_S[6 + r3] -= b_S[3 + r3] * b_S[6 + r2];
    for (k = 0; k < 3; k++) {
      K[k + 3 * r1] = c_y[k] / b_S[r1];
      K[k + 3 * r2] = c_y[3 + k] - K[k + 3 * r1] * b_S[3 + r1];
      K[k + 3 * r3] = c_y[6 + k] - K[k + 3 * r1] * b_S[6 + r1];
      K[k + 3 * r2] /= b_S[3 + r2];
      K[k + 3 * r3] -= K[k + 3 * r2] * b_S[6 + r2];
      K[k + 3 * r3] /= b_S[6 + r3];
      K[k + 3 * r2] -= K[k + 3 * r3] * b_S[3 + r3];
      K[k + 3 * r1] -= K[k + 3 * r3] * b_S[r3];
      K[k + 3 * r1] -= K[k + 3 * r2] * b_S[r2];
      b_IMU_measurements[k] = IMU_measurements[3 + k] - z_n_b[k];
    }

    for (r1 = 0; r1 < 3; r1++) {
      b_y_n_b[r1] = 0.0;
      for (rtemp = 0; rtemp < 3; rtemp++) {
        b_y_n_b[r1] += K[r1 + 3 * rtemp] * b_IMU_measurements[rtemp];
      }
    }

    quatPlusThetaJ(b_y_n_b, q);

    //  x(1) = 0;
    //  x = normc(x);
    memset(&b_S[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      b_S[k + 3 * k] = 1.0;
    }

    dv332[0] = x_att[3];
    dv332[4] = -x_att[2];
    dv332[8] = x_att[1];
    dv332[12] = x_att[0];
    dv332[1] = x_att[2];
    dv332[5] = x_att[3];
    dv332[9] = -x_att[0];
    dv332[13] = x_att[1];
    dv332[2] = -x_att[1];
    dv332[6] = x_att[0];
    dv332[10] = x_att[3];
    dv332[14] = x_att[2];
    dv332[3] = -x_att[0];
    dv332[7] = -x_att[1];
    dv332[11] = -x_att[2];
    dv332[15] = x_att[3];
    b_q[0] = q[0];
    b_q[1] = q[1];
    b_q[2] = q[2];
    b_q[3] = q[3];
    for (r1 = 0; r1 < 4; r1++) {
      x_att[r1] = 0.0;
      for (rtemp = 0; rtemp < 4; rtemp++) {
        x_att[r1] += dv332[r1 + (rtemp << 2)] * b_q[rtemp];
      }
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        maxval = 0.0;
        for (k = 0; k < 3; k++) {
          maxval += K[r1 + 3 * k] * H[k + 3 * rtemp];
        }

        S[r1 + 3 * rtemp] = b_S[r1 + 3 * rtemp] - maxval;
      }
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        b_x_n_b[r1 + 3 * rtemp] = 0.0;
        for (k = 0; k < 3; k++) {
          b_x_n_b[r1 + 3 * rtemp] += S[r1 + 3 * k] * P_att[k + 3 * rtemp];
        }
      }
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P_att[rtemp + 3 * r1] = b_x_n_b[rtemp + 3 * r1];
      }
    }

    r1 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, r1, (int)sizeof(double));
    rtemp = xt->size[0];
    for (r1 = 0; r1 < rtemp; r1++) {
      xt_out->data[r1] = xt->data[r1];
    }

    for (r1 = 0; r1 < 4; r1++) {
      xt_out->data[3 + r1] = x_att[r1];
    }

    r1 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, r1, (int)sizeof(double));
    rtemp = P->size[0] * P->size[1];
    for (r1 = 0; r1 < rtemp; r1++) {
      P_apo_out->data[r1] = P->data[r1];
    }

    r1 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, r1, (int)sizeof(double));
    for (r1 = 0; r1 < 64; r1++) {
      h_u_apo_out->data[r1] = -100.0;
    }

    r1 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, r1, (int)sizeof(double));
    for (r1 = 0; r1 < 48; r1++) {
      map_out->data[r1] = rtNaN;
    }

    init_counter = 1.0;
  } else if (init_counter == 1.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (rtemp = 0; rtemp < 16; rtemp++) {
      updateVect[rtemp] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      rtemp = 0;
    } else {
      rtemp = (int)numPointsPerAnchor;
    }

    for (r1 = 0; r1 < rtemp; r1++) {
      tmp_data[r1] = r1;
    }

    for (r1 = 0; r1 < rtemp; r1++) {
      updateVect[tmp_data[r1]] = 2.0;
    }

    for (r1 = 0; r1 < 4; r1++) {
      xt->data[3 + r1] = x_att[r1];
    }

    for (r1 = 0; r1 < 3; r1++) {
      for (rtemp = 0; rtemp < 3; rtemp++) {
        P->data[(rtemp + P->size[0] * (3 + r1)) + 3] = P_att[rtemp + 3 * r1];
      }
    }

    r1 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, r1, (int)sizeof(double));
    rtemp = xt->size[0];
    for (r1 = 0; r1 < rtemp; r1++) {
      xt_out->data[r1] = xt->data[r1];
    }

    for (r1 = 0; r1 < 4; r1++) {
      xt_out->data[3 + r1] = x_att[r1];
    }

    r1 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, r1, (int)sizeof(double));
    rtemp = P->size[0] * P->size[1];
    for (r1 = 0; r1 < rtemp; r1++) {
      P_apo_out->data[r1] = P->data[r1];
    }

    r1 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, r1, (int)sizeof(double));
    for (r1 = 0; r1 < 64; r1++) {
      h_u_apo_out->data[r1] = -100.0;
    }

    r1 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, r1, (int)sizeof(double));
    for (r1 = 0; r1 < 48; r1++) {
      map_out->data[r1] = rtNaN;
    }

    init_counter = 2.0;
  } else if (init_counter == 2.0) {
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, (1.0 - rt_powd_snf
                (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45,
               h_u_apo_out, map_out);
    r1 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, r1, (int)sizeof(double));
    rtemp = xt->size[0];
    for (r1 = 0; r1 < rtemp; r1++) {
      xt_out->data[r1] = xt->data[r1];
    }

    r1 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, r1, (int)sizeof(double));
    rtemp = P->size[0] * P->size[1];
    for (r1 = 0; r1 < rtemp; r1++) {
      P_apo_out->data[r1] = P->data[r1];
    }

    init_counter = 3.0;
  } else {
    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates);

    //  [xt,P] =  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates); 
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, 0.0, h_u_apo_out, map_out);
    r1 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, r1, (int)sizeof(double));
    rtemp = xt->size[0];
    for (r1 = 0; r1 < rtemp; r1++) {
      xt_out->data[r1] = xt->data[r1];
    }

    r1 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, r1, (int)sizeof(double));
    rtemp = P->size[0] * P->size[1];
    for (r1 = 0; r1 < rtemp; r1++) {
      P_apo_out->data[r1] = P->data[r1];
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
