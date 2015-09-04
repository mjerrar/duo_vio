//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 11:04:36
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "QuatFromRotJ.h"
#include "SLAM_updIT.h"
#include "Att_upd.h"
#include "quatPlusThetaJ.h"
#include "fprintf.h"
#include "any.h"
#include "SLAM_pred.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double height_offset_pressure;
static boolean_T ext_pose_offset_initialized;
static double ext_pos_offset[3];
static double ext_att_offset[9];
static double i_control[3];
static double initializing_attitude;
static double x_att[4];
static double P_att[9];

// Function Declarations
static void initialize(const double ref_position[4], double measurements_bar_fmu,
  double numPointsPerAnchor, double numAnchors, double c_numStates,
  emxArray_real_T *b_xt, emxArray_real_T *b_P, double *b_height_offset_pressure);
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : const double ref_position[4]
//                double measurements_bar_fmu
//                double numPointsPerAnchor
//                double numAnchors
//                double c_numStates
//                emxArray_real_T *b_xt
//                emxArray_real_T *b_P
//                double *b_height_offset_pressure
// Return Type  : void
//
static void initialize(const double ref_position[4], double measurements_bar_fmu,
  double numPointsPerAnchor, double numAnchors, double c_numStates,
  emxArray_real_T *b_xt, emxArray_real_T *b_P, double *b_height_offset_pressure)
{
  emxArray_real_T *a;
  int k;
  int loop_ub;
  emxArray_real_T *b;
  int outsize_idx_0;
  int outsize_idx_1;
  int ibcol;
  double dv0[4];
  static const double dv1[9] = { 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0 };

  static const double gyro_offset[3] = { 0.534299594891486, -0.26782462470649,
    0.000842792650371513 };

  double d0;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  b_emxInit_real_T(&a, 1);

  //          za = measurements_.acc_duo;
  //          z_n_b = za/norm(za);
  //          m_n_b = [1;0;0];
  //          y_n_b = cross(z_n_b,m_n_b);
  //          y_n_b = y_n_b./norm(y_n_b);
  //          x_n_b = (cross(y_n_b,z_n_b));
  //          x_n_b = x_n_b./norm(x_n_b);
  //
  //          R_iw_init = [x_n_b,y_n_b,z_n_b];
  //
  //  R_iw_init = [1 0 0;
  //              0 0 -1;
  //              0 1 0];
  k = a->size[0];
  a->size[0] = 7 + (int)numPointsPerAnchor;
  emxEnsureCapacity((emxArray__common *)a, k, (int)sizeof(double));
  a->data[0] = 0.0;
  a->data[1] = 0.0;
  a->data[2] = 0.0;
  a->data[3] = 0.0;
  a->data[4] = 0.0;
  a->data[5] = 0.0;
  a->data[6] = 1.0;
  loop_ub = (int)numPointsPerAnchor;
  for (k = 0; k < loop_ub; k++) {
    a->data[k + 7] = 0.0;
  }

  b_emxInit_real_T(&b, 1);
  outsize_idx_0 = a->size[0] * (int)numAnchors;
  k = b->size[0];
  b->size[0] = outsize_idx_0;
  emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(double));
  if (!(outsize_idx_0 == 0)) {
    outsize_idx_0 = a->size[0];
    for (outsize_idx_1 = 1; outsize_idx_1 <= (int)numAnchors; outsize_idx_1++) {
      ibcol = (outsize_idx_1 - 1) * outsize_idx_0;
      for (k = 0; k + 1 <= outsize_idx_0; k++) {
        b->data[ibcol + k] = a->data[k];
      }
    }
  }

  emxFree_real_T(&a);
  QuatFromRotJ(dv1, dv0);
  k = b_xt->size[0];
  b_xt->size[0] = 13 + b->size[0];
  emxEnsureCapacity((emxArray__common *)b_xt, k, (int)sizeof(double));
  for (k = 0; k < 3; k++) {
    b_xt->data[k] = ref_position[k];
  }

  for (k = 0; k < 4; k++) {
    b_xt->data[k + 3] = dv0[k];
  }

  b_xt->data[7] = 0.0;
  b_xt->data[8] = 0.0;
  b_xt->data[9] = 0.0;
  for (k = 0; k < 3; k++) {
    b_xt->data[k + 10] = gyro_offset[k];
  }

  loop_ub = b->size[0];
  for (k = 0; k < loop_ub; k++) {
    b_xt->data[k + 13] = b->data[k];
  }

  emxFree_real_T(&b);

  //  initial real vector
  d0 = numAnchors * (6.0 + numPointsPerAnchor);
  loop_ub = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  outsize_idx_0 = (int)c_numStates + (int)d0;
  outsize_idx_1 = (int)c_numStates + (int)d0;
  k = b_P->size[0] * b_P->size[1];
  b_P->size[0] = outsize_idx_0;
  emxEnsureCapacity((emxArray__common *)b_P, k, (int)sizeof(double));
  k = b_P->size[0] * b_P->size[1];
  b_P->size[1] = outsize_idx_1;
  emxEnsureCapacity((emxArray__common *)b_P, k, (int)sizeof(double));
  outsize_idx_0 *= outsize_idx_1;
  for (k = 0; k < outsize_idx_0; k++) {
    b_P->data[k] = 0.0;
  }

  if ((int)c_numStates > 0) {
    outsize_idx_0 = (int)c_numStates;
    for (k = 0; k < outsize_idx_0; k++) {
      outsize_idx_1 = (int)c_numStates;
      for (ibcol = 0; ibcol < outsize_idx_1; ibcol++) {
        b_P->data[ibcol + b_P->size[0] * k] = 0.0;
      }
    }
  }

  if ((int)d0 > 0) {
    if ((int)c_numStates + 1 > (int)c_numStates + (int)d0) {
      k = 1;
    } else {
      k = (int)c_numStates + 1;
    }

    if ((int)c_numStates + 1 > (int)c_numStates + (int)d0) {
      ibcol = 1;
    } else {
      ibcol = (int)c_numStates + 1;
    }

    for (outsize_idx_0 = 0; outsize_idx_0 < loop_ub; outsize_idx_0++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < loop_ub; outsize_idx_1++) {
        b_P->data[((k + outsize_idx_1) + b_P->size[0] * ((ibcol + outsize_idx_0)
                    - 1)) - 1] = 0.0;
      }
    }
  }

  //  initial error state covariance
  for (k = 0; k < 3; k++) {
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_P->data[ibcol + b_P->size[0] * k] = 0.0;
    }
  }

  //  position
  for (k = 0; k < 3; k++) {
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_P->data[(ibcol + b_P->size[0] * (3 + k)) + 3] = 0.0;
    }
  }

  //  orientation
  //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
  //      P(4:6,4:6) = diag([0 0 1]);
  for (k = 0; k < 3; k++) {
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_P->data[(ibcol + b_P->size[0] * (6 + k)) + 6] = 0.0;
    }
  }

  //  velocity
  for (k = 0; k < 3; k++) {
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_P->data[(ibcol + b_P->size[0] * (9 + k)) + 9] = y[ibcol + 3 * k];
    }
  }

  //  gyro bias
  *b_height_offset_pressure = (1.0 - rt_powd_snf(measurements_bar_fmu / 101325.0,
    0.190284)) * 145366.45;
}

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
// NOTE: Comment this out for MEXing
// Arguments    : double updateVect[24]
//                const double z_all_l[48]
//                const double z_all_r[48]
//                double dt
//                const VIOMeasurements *measurements
//                ReferenceCommand *ref
//                const VIOParameters *b_VIOParameters
//                const StereoParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const ControllerGains *b_ControllerGains
//                boolean_T resetFlag
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
//                double u_out[4]
// Return Type  : void
//
void SLAM(double updateVect[24], const double z_all_l[48], const double z_all_r
          [48], double dt, const VIOMeasurements *measurements, ReferenceCommand
          *ref, const VIOParameters *b_VIOParameters, const StereoParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          ControllerGains *b_ControllerGains, boolean_T resetFlag,
          emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
          *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int k;
  double numTrackFeatures;
  VIOMeasurements measurements_;
  emxArray_boolean_T *r3;
  double b_height_offset_pressure;
  static const double b_x_att[4] = { 0.5, -0.5, 0.5, -0.5 };

  static const double b_P_att[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.01 };

  int i9;
  double err_p_b[3];
  static const double gyro_offset[3] = { 0.534299594891486, -0.26782462470649,
    0.000842792650371513 };

  double q;
  double R_cw[9];
  double b_measurements_[9];
  double c;
  double b_err_p_b[3];
  double Phi[9];
  double dq[4];
  double dv9[16];
  double b_dq[4];
  double c_measurements_[9];
  double b_Phi[9];
  double a[9];
  int i10;
  static const signed char b_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char c_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  char cv8[33];
  static const char cv9[33] = { 'G', 'o', 't', ' ', 'N', 'a', 'N', 's', ' ', 'i',
    'n', ' ', 't', 'h', 'e', ' ', 's', 't', 'a', 't', 'e', '.', ' ', 'R', 'e',
    's', 'e', 't', 't', 'i', 'n', 'g', '\x00' };

  double r;
  double yaw;
  char cv10[42];
  static const char cv11[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'x', ' ', 'p', 'o', 's', 'i', 't', 'i', 'o', 'n', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv12[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'y', ' ', 'p', 'o', 's', 'i', 't', 'i', 'o', 'n', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv13[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'z', ' ', 'p', 'o', 's', 'i', 't', 'i', 'o', 'n', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  char cv14[35];
  static const char cv15[35] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'y', 'a', 'w', ' ', 'i', 's', ' ', 'N', 'a', 'n', '!', ' ', 'P', 'u',
    't', 't', 'i', 'n', 'g', ' ', 't', 'o', ' ', '0', '\x00' };

  static const char cv16[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'x', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv17[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'y', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv18[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'z', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  char cv19[39];
  static const char cv20[39] = { 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ',
    'y', 'a', 'w', ' ', 'r', 'a', 't', 'e', ' ', 'i', 's', ' ', 'N', 'a', 'n',
    '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o', ' ', '0', '\x00'
  };

  static const signed char iv4[3] = { 0, 0, 1 };

  double c_ControllerGains[3];
  double err_v_b[3];
  double minval;
  for (k = 0; k < 4; k++) {
    u_out[k] = 0.0;
  }

  //  for coder
  numTrackFeatures = b_VIOParameters->num_anchors *
    b_VIOParameters->num_points_per_anchor;
  measurements_ = *measurements;

  //  copy for coders
  emxInit_boolean_T(&r3, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialize(ref->position, measurements->bar_fmu,
               b_VIOParameters->num_points_per_anchor,
               b_VIOParameters->num_anchors, numStates, xt, P,
               &b_height_offset_pressure);
    for (k = 0; k < 4; k++) {
      x_att[k] = b_x_att[k];
    }

    memcpy(&P_att[0], &b_P_att[0], 9U * sizeof(double));
    for (k = 0; k < 24; k++) {
      updateVect[k] = 0.0;
    }

    height_offset_pressure = b_height_offset_pressure;
    initialized_not_empty = true;
    initializing_attitude = 1.0;
    ext_pose_offset_initialized = false;
    for (k = 0; k < 3; k++) {
      ext_pos_offset[k] = 0.0;
    }

    memset(&ext_att_offset[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      ext_att_offset[k + 3 * k] = 1.0;
    }

    i9 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i9, (int)sizeof(double));
    k = xt->size[0];
    for (i9 = 0; i9 < k; i9++) {
      xt_out->data[i9] = xt->data[i9];
    }

    i9 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i9, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i9 = 0; i9 < k; i9++) {
      P_apo_out->data[i9] = P->data[i9];
    }

    i9 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i9, (int)sizeof(double));
    k = (int)(numTrackFeatures * 4.0);
    for (i9 = 0; i9 < k; i9++) {
      h_u_apo_out->data[i9] = 0.0;
    }

    i9 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = (int)numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)map_out, i9, (int)sizeof(double));
    k = 3 * (int)numTrackFeatures;
    for (i9 = 0; i9 < k; i9++) {
      map_out->data[i9] = 0.0;
    }

    //  the last control outputs (in camera frame)
    for (k = 0; k < 3; k++) {
      i_control[k] = 0.0;
    }
  } else if (initializing_attitude > 0.0) {
    for (k = 0; k < 3; k++) {
      err_p_b[k] = measurements_.gyr_duo[k] - gyro_offset[k];
    }

    q = noiseParameters->process_noise[1] * 1000.0;

    // ATT_PRED Prediction step of the attitude estimator
    //    INPUT ARGUMENTS:
    //    - x:  The current estimated attitude (JPL quaternion)
    //    - P:  The state covariance matrix (3 x 3)
    //    - w:  The current Gyro measurement (3 x 1)
    //    - q:  The proecss noise
    //    - dt: The time step
    memset(&R_cw[0], 0, 9U * sizeof(double));
    b_measurements_[0] = 0.0;
    b_measurements_[3] = -err_p_b[2];
    b_measurements_[6] = err_p_b[1];
    b_measurements_[1] = err_p_b[2];
    b_measurements_[4] = 0.0;
    b_measurements_[7] = -err_p_b[0];
    b_measurements_[2] = -err_p_b[1];
    b_measurements_[5] = err_p_b[0];
    b_measurements_[8] = 0.0;
    c = dt * dt;
    for (k = 0; k < 3; k++) {
      R_cw[k + 3 * k] = 1.0;
      for (i9 = 0; i9 < 3; i9++) {
        Phi[i9 + 3 * k] = R_cw[i9 + 3 * k] + -b_measurements_[i9 + 3 * k] * dt;
      }

      b_err_p_b[k] = err_p_b[k] * dt;
    }

    quatPlusThetaJ(b_err_p_b, dq);
    dv9[0] = x_att[3];
    dv9[4] = -x_att[2];
    dv9[8] = x_att[1];
    dv9[12] = x_att[0];
    dv9[1] = x_att[2];
    dv9[5] = x_att[3];
    dv9[9] = -x_att[0];
    dv9[13] = x_att[1];
    dv9[2] = -x_att[1];
    dv9[6] = x_att[0];
    dv9[10] = x_att[3];
    dv9[14] = x_att[2];
    dv9[3] = -x_att[0];
    dv9[7] = -x_att[1];
    dv9[11] = -x_att[2];
    dv9[15] = x_att[3];
    b_dq[0] = dq[0];
    b_dq[1] = dq[1];
    b_dq[2] = dq[2];
    b_dq[3] = dq[3];
    for (i9 = 0; i9 < 4; i9++) {
      x_att[i9] = 0.0;
      for (k = 0; k < 4; k++) {
        x_att[i9] += dv9[i9 + (k << 2)] * b_dq[k];
      }
    }

    for (i9 = 0; i9 < 3; i9++) {
      for (k = 0; k < 3; k++) {
        b_measurements_[i9 + 3 * k] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          b_measurements_[i9 + 3 * k] += Phi[i9 + 3 * i10] * P_att[i10 + 3 * k];
        }

        c_measurements_[i9 + 3 * k] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          c_measurements_[i9 + 3 * k] += (double)c_a[i9 + 3 * i10] * ((double)
            b_a[i10 + 3 * k] * q * c);
        }
      }

      for (k = 0; k < 3; k++) {
        b_Phi[i9 + 3 * k] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          b_Phi[i9 + 3 * k] += b_measurements_[i9 + 3 * i10] * Phi[k + 3 * i10];
        }

        a[i9 + 3 * k] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          a[i9 + 3 * k] += c_measurements_[i9 + 3 * i10] * (double)c_a[i10 + 3 *
            k];
        }
      }
    }

    for (i9 = 0; i9 < 3; i9++) {
      for (k = 0; k < 3; k++) {
        P_att[k + 3 * i9] = b_Phi[k + 3 * i9] + a[k + 3 * i9];
      }
    }

    Att_upd(x_att, P_att, measurements_.acc_duo, noiseParameters->process_noise
            [0] * 1.0E+10, dt);
    for (i9 = 0; i9 < 4; i9++) {
      xt->data[3 + i9] = x_att[i9];
    }

    initializing_attitude = 2.0;

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((x_att[0] * x_att[0] - x_att[1] * x_att[1]) - x_att[2] * x_att[2])
      + x_att[3] * x_att[3];
    R_cw[3] = 2.0 * (x_att[0] * x_att[1] + x_att[2] * x_att[3]);
    R_cw[6] = 2.0 * (x_att[0] * x_att[2] - x_att[1] * x_att[3]);
    R_cw[1] = 2.0 * (x_att[0] * x_att[1] - x_att[2] * x_att[3]);
    R_cw[4] = ((-(x_att[0] * x_att[0]) + x_att[1] * x_att[1]) - x_att[2] *
               x_att[2]) + x_att[3] * x_att[3];
    R_cw[7] = 2.0 * (x_att[1] * x_att[2] + x_att[0] * x_att[3]);
    R_cw[2] = 2.0 * (x_att[0] * x_att[2] + x_att[1] * x_att[3]);
    R_cw[5] = 2.0 * (x_att[1] * x_att[2] - x_att[0] * x_att[3]);
    R_cw[8] = ((-(x_att[0] * x_att[0]) - x_att[1] * x_att[1]) + x_att[2] *
               x_att[2]) + x_att[3] * x_att[3];
    for (i9 = 0; i9 < 3; i9++) {
      for (k = 0; k < 3; k++) {
        b_measurements_[i9 + 3 * k] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          b_measurements_[i9 + 3 * k] += 0.1 * R_cw[i9 + 3 * i10] * (double)
            b[i10 + 3 * k];
        }
      }
    }

    for (i9 = 0; i9 < 3; i9++) {
      for (k = 0; k < 3; k++) {
        P->data[(i9 + P->size[0] * (3 + k)) + 3] = 0.0;
        for (i10 = 0; i10 < 3; i10++) {
          P->data[(i9 + P->size[0] * (3 + k)) + 3] += b_measurements_[i9 + 3 *
            i10] * R_cw[k + 3 * i10];
        }
      }
    }

    SLAM_updIT(P, xt, cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters, &measurements_, height_offset_pressure,
               *b_VIOParameters, h_u_apo_out, map_out);
    initializing_attitude = 0.0;
    i9 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i9, (int)sizeof(double));
    k = xt->size[0];
    for (i9 = 0; i9 < k; i9++) {
      xt_out->data[i9] = xt->data[i9];
    }

    i9 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i9, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i9 = 0; i9 < k; i9++) {
      P_apo_out->data[i9] = P->data[i9];
    }
  } else {
    if (b_VIOParameters->use_ext_pose) {
      if (!ext_pose_offset_initialized) {
        for (i9 = 0; i9 < 3; i9++) {
          ext_pos_offset[i9] = xt->data[i9] - measurements_.pos_ext[i9];
        }

        //  in vio frame
        for (i9 = 0; i9 < 3; i9++) {
          measurements_.pos_ext[i9] = xt->data[i9];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        c_measurements_[0] = ((measurements_.att_ext[0] * measurements_.att_ext
          [0] - measurements_.att_ext[1] * measurements_.att_ext[1]) -
                              measurements_.att_ext[2] * measurements_.att_ext[2])
          + measurements_.att_ext[3] * measurements_.att_ext[3];
        c_measurements_[1] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] + measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        c_measurements_[2] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] - measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        c_measurements_[3] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] - measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        c_measurements_[4] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) + measurements_.att_ext[1] *
          measurements_.att_ext[1]) - measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        c_measurements_[5] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] + measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        c_measurements_[6] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] + measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        c_measurements_[7] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] - measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        c_measurements_[8] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) - measurements_.att_ext[1] *
          measurements_.att_ext[1]) + measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        b_measurements_[0] = ((xt->data[3] * xt->data[3] - xt->data[4] *
          xt->data[4]) - xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        b_measurements_[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] *
          xt->data[6]);
        b_measurements_[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] *
          xt->data[6]);
        b_measurements_[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] *
          xt->data[6]);
        b_measurements_[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] *
          xt->data[4]) - xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        b_measurements_[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] *
          xt->data[6]);
        b_measurements_[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] *
          xt->data[6]);
        b_measurements_[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] *
          xt->data[6]);
        b_measurements_[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] *
          xt->data[4]) + xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        for (i9 = 0; i9 < 3; i9++) {
          for (k = 0; k < 3; k++) {
            ext_att_offset[i9 + 3 * k] = 0.0;
            for (i10 = 0; i10 < 3; i10++) {
              ext_att_offset[i9 + 3 * k] += c_measurements_[i9 + 3 * i10] *
                b_measurements_[i10 + 3 * k];
            }
          }
        }

        for (i9 = 0; i9 < 4; i9++) {
          measurements_.att_ext[i9] = xt->data[3 + i9];
        }

        ext_pose_offset_initialized = true;
      } else {
        for (i9 = 0; i9 < 3; i9++) {
          b_height_offset_pressure = 0.0;
          for (k = 0; k < 3; k++) {
            b_height_offset_pressure += ext_att_offset[k + 3 * i9] *
              measurements_.pos_ext[k];
          }

          b_err_p_b[i9] = b_height_offset_pressure + ext_pos_offset[i9];
        }

        for (i9 = 0; i9 < 3; i9++) {
          measurements_.pos_ext[i9] = b_err_p_b[i9];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        b_measurements_[0] = ((measurements_.att_ext[0] * measurements_.att_ext
          [0] - measurements_.att_ext[1] * measurements_.att_ext[1]) -
                              measurements_.att_ext[2] * measurements_.att_ext[2])
          + measurements_.att_ext[3] * measurements_.att_ext[3];
        b_measurements_[3] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] + measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        b_measurements_[6] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] - measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        b_measurements_[1] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] - measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        b_measurements_[4] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) + measurements_.att_ext[1] *
          measurements_.att_ext[1]) - measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        b_measurements_[7] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] + measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        b_measurements_[2] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] + measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        b_measurements_[5] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] - measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        b_measurements_[8] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) - measurements_.att_ext[1] *
          measurements_.att_ext[1]) + measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        for (i9 = 0; i9 < 3; i9++) {
          for (k = 0; k < 3; k++) {
            c_measurements_[i9 + 3 * k] = 0.0;
            for (i10 = 0; i10 < 3; i10++) {
              c_measurements_[i9 + 3 * k] += b_measurements_[i9 + 3 * i10] *
                ext_att_offset[i10 + 3 * k];
            }
          }
        }

        QuatFromRotJ(c_measurements_, measurements_.att_ext);
      }
    }

    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements_.gyr_duo,
              measurements_.acc_duo, numStates);
    SLAM_updIT(P, xt, cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters, &measurements_, height_offset_pressure,
               *b_VIOParameters, h_u_apo_out, map_out);
    i9 = r3->size[0];
    r3->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)r3, i9, (int)sizeof(boolean_T));
    k = xt->size[0];
    for (i9 = 0; i9 < k; i9++) {
      r3->data[i9] = rtIsNaN(xt->data[i9]);
    }

    if (e_any(r3)) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 33; i9++) {
        cv8[i9] = cv9[i9];
      }

      ROS_ERROR(cv8);
      initialize(ref->position, measurements->bar_fmu,
                 b_VIOParameters->num_points_per_anchor,
                 b_VIOParameters->num_anchors, numStates, xt, P,
                 &b_height_offset_pressure);
      for (k = 0; k < 4; k++) {
        x_att[k] = b_x_att[k];
      }

      memcpy(&P_att[0], &b_P_att[0], 9U * sizeof(double));
      for (k = 0; k < 24; k++) {
        updateVect[k] = 0.0;
      }

      height_offset_pressure = b_height_offset_pressure;
      ext_pose_offset_initialized = false;
      for (k = 0; k < 3; k++) {
        ext_pos_offset[k] = 0.0;
      }

      memset(&ext_att_offset[0], 0, 9U * sizeof(double));
      for (k = 0; k < 3; k++) {
        ext_att_offset[k + 3 * k] = 1.0;
        i_control[k] = 0.0;
      }
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    R_cw[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    R_cw[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    R_cw[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    R_cw[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    r = rt_atan2d_snf(R_cw[3], R_cw[0]);
    yaw = rt_atan2d_snf(R_cw[3], R_cw[0]) + 1.5707963267948966;
    j_fprintf(r + 1.5707963267948966, ref->position[3]);
    if (rtIsNaN(ref->position[0])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv11[i9];
      }

      ROS_ERROR(cv10);
      ref->position[0] = 0.0;
    }

    if (rtIsNaN(ref->position[1])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv12[i9];
      }

      ROS_ERROR(cv10);
      ref->position[1] = 0.0;
    }

    if (rtIsNaN(ref->position[2])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv13[i9];
      }

      ROS_ERROR(cv10);
      ref->position[2] = 0.0;
    }

    if (rtIsNaN(ref->position[3])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 35; i9++) {
        cv14[i9] = cv15[i9];
      }

      ROS_ERROR(cv14);
      ref->position[3] = 0.0;
    }

    if (rtIsNaN(ref->velocity[0])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv16[i9];
      }

      ROS_ERROR(cv10);
      ref->position[0] = 0.0;
    }

    if (rtIsNaN(ref->velocity[1])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv17[i9];
      }

      ROS_ERROR(cv10);
      ref->position[1] = 0.0;
    }

    if (rtIsNaN(ref->velocity[2])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 42; i9++) {
        cv10[i9] = cv18[i9];
      }

      ROS_ERROR(cv10);
      ref->position[2] = 0.0;
    }

    if (rtIsNaN(ref->position[3])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i9 = 0; i9 < 39; i9++) {
        cv19[i9] = cv20[i9];
      }

      ROS_ERROR(cv19);
      ref->position[3] = 0.0;
    }

    //  transform between world and control frame (yaw-rotatate world frame)
    R_cw[0] = cos(yaw);
    R_cw[1] = -sin(yaw);
    R_cw[2] = 0.0;
    R_cw[3] = sin(yaw);
    R_cw[4] = cos(yaw);
    R_cw[5] = 0.0;
    for (i9 = 0; i9 < 3; i9++) {
      R_cw[6 + i9] = iv4[i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      b_err_p_b[i9] = xt->data[i9] - ref->position[i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      err_p_b[i9] = 0.0;
      for (k = 0; k < 3; k++) {
        err_p_b[i9] += R_cw[i9 + 3 * k] * b_err_p_b[k];
      }
    }

    //  position error in control frame
    for (i9 = 0; i9 < 3; i9++) {
      b_err_p_b[i9] = xt->data[7 + i9] - ref->velocity[i9];
    }

    //  velocity error in control frame
    c_ControllerGains[0] = b_ControllerGains->Ki_xy * err_p_b[0];
    c_ControllerGains[1] = b_ControllerGains->Ki_xy * err_p_b[1];
    c_ControllerGains[2] = b_ControllerGains->Ki_z * err_p_b[2];
    for (i9 = 0; i9 < 3; i9++) {
      err_v_b[i9] = 0.0;
      for (k = 0; k < 3; k++) {
        err_v_b[i9] += R_cw[i9 + 3 * k] * b_err_p_b[k];
      }

      i_control[i9] += c_ControllerGains[i9];
    }

    if ((b_ControllerGains->i_lim <= i_control[0]) || rtIsNaN(i_control[0])) {
      minval = b_ControllerGains->i_lim;
    } else {
      minval = i_control[0];
    }

    if ((-b_ControllerGains->i_lim >= minval) || rtIsNaN(minval)) {
      i_control[0] = -b_ControllerGains->i_lim;
    } else {
      i_control[0] = minval;
    }

    //  saturate integral part
    if ((b_ControllerGains->i_lim <= i_control[1]) || rtIsNaN(i_control[1])) {
      minval = b_ControllerGains->i_lim;
    } else {
      minval = i_control[1];
    }

    if ((-b_ControllerGains->i_lim >= minval) || rtIsNaN(minval)) {
      i_control[1] = -b_ControllerGains->i_lim;
    } else {
      i_control[1] = minval;
    }

    if ((b_ControllerGains->i_lim <= i_control[2]) || rtIsNaN(i_control[2])) {
      minval = b_ControllerGains->i_lim;
    } else {
      minval = i_control[2];
    }

    if ((-b_ControllerGains->i_lim >= minval) || rtIsNaN(minval)) {
      i_control[2] = -b_ControllerGains->i_lim;
    } else {
      i_control[2] = minval;
    }

    //  fprintf('ingegral control: %.3f %.3f %.3f\n', i_control(1), i_control(2), i_control(3)) 
    //  yaw_ctrl = atan2(R_cw(1,2), R_cw(1,1));
    //  fprintf('control frame error: %.3f %.3f %.3f %.3f\n', err_p_b(1), err_p_b(2), err_p_b(3), (yaw_trafo - ref.position(4))) 
    u_out[0] = -((b_ControllerGains->Kp_xy * err_p_b[0] + i_control[0]) +
                 b_ControllerGains->Kd_xy * err_v_b[0]);
    u_out[1] = -((b_ControllerGains->Kp_xy * err_p_b[1] + i_control[1]) +
                 b_ControllerGains->Kd_xy * err_v_b[1]);
    u_out[2] = -((b_ControllerGains->Kp_z * err_p_b[2] + i_control[2]) +
                 b_ControllerGains->Kd_z * err_v_b[2]);
    u_out[3] = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * ((r + 1.5707963267948966) - ref->position[3]);
    l_fprintf(err_p_b[0], err_p_b[1], err_p_b[2], (r + 1.5707963267948966) -
              ref->position[3], u_out[0], u_out[1], u_out[2], u_out[3]);
    i9 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i9, (int)sizeof(double));
    k = xt->size[0];
    for (i9 = 0; i9 < k; i9++) {
      xt_out->data[i9] = xt->data[i9];
    }

    i9 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i9, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i9 = 0; i9 < k; i9++) {
      P_apo_out->data[i9] = P->data[i9];
    }

    // % output asserts for coder
  }

  emxFree_boolean_T(&r3);
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
