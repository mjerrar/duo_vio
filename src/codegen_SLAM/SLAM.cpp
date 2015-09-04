//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 16:58:25
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "eye.h"
#include "QuatFromRotJ.h"
#include "SLAM_updIT.h"
#include "Att_upd.h"
#include "Att_pred.h"
#include "fprintf.h"
#include "ros_error.h"
#include "any.h"
#include "SLAM_pred.h"
#include "diag.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double height_offset_pressure;
static double last_u[3];
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

  static const double b_y[9] = { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

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
  b_xt->size[0] = 16 + b->size[0];
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

  for (k = 0; k < 3; k++) {
    b_xt->data[k + 13] = 2.0;
  }

  loop_ub = b->size[0];
  for (k = 0; k < loop_ub; k++) {
    b_xt->data[k + 16] = b->data[k];
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
  for (k = 0; k < 3; k++) {
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_P->data[(ibcol + b_P->size[0] * (12 + k)) + 12] = b_y[ibcol + 3 * k];
    }
  }

  //  control dynamics
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
  int i;
  double numTrackFeatures;
  VIOMeasurements measurements_pred;
  double b_measurements[3];
  int i15;
  static const double acc_offset[3] = { 0.904219545041063, 0.231515361023414,
    0.186293533362293 };

  static const double a[9] = { 1.026034113920804, 0.0, 0.0, 0.0,
    1.0222265280883942, 0.0, 0.0, 0.0, 1.0130438978221066 };

  VIOMeasurements measurements_upd;
  emxArray_boolean_T *r3;
  double b_height_offset_pressure;
  static const double b_x_att[4] = { 0.5, -0.5, 0.5, -0.5 };

  static const double b_P_att[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.01 };

  static const double gyro_offset[3] = { 0.534299594891486, -0.26782462470649,
    0.000842792650371513 };

  double R_cw[9];
  double dv9[9];
  int i16;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double b_measurements_pred[9];
  double yaw;
  char cv18[42];
  static const char cv19[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'x', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv20[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'y', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  static const char cv21[42] = { 'R', 'e', 'f', 'e', 'r', 'e', 'n', 'c', 'e',
    ' ', 'z', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ', 'i', 's', ' ',
    'N', 'a', 'n', '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', '0', '\x00' };

  char cv22[39];
  static const char cv23[39] = { 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', ' ',
    'y', 'a', 'w', ' ', 'r', 'a', 't', 'e', ' ', 'i', 's', ' ', 'N', 'a', 'n',
    '!', ' ', 'P', 'u', 't', 't', 'i', 'n', 'g', ' ', 't', 'o', ' ', '0', '\x00'
  };

  double R_bw[9];
  static const signed char iv4[3] = { 0, 0, 1 };

  double err_p_b[3];
  double c_ControllerGains[3];
  double err_v_b[3];
  double minval;
  double u_out_pos[3];
  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  numTrackFeatures = b_VIOParameters->num_anchors *
    b_VIOParameters->num_points_per_anchor;
  measurements_pred = *measurements;

  //  copy for coders
  for (i15 = 0; i15 < 3; i15++) {
    b_measurements[i15] = measurements->acc_duo[i15] - acc_offset[i15];
  }

  for (i15 = 0; i15 < 3; i15++) {
    measurements_pred.acc_duo[i15] = 0.0;
    for (i = 0; i < 3; i++) {
      measurements_pred.acc_duo[i15] += a[i15 + 3 * i] * b_measurements[i];
    }
  }

  measurements_upd = measurements_pred;

  //  copy for coders
  emxInit_boolean_T(&r3, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialize(ref->position, measurements->bar_fmu,
               b_VIOParameters->num_points_per_anchor,
               b_VIOParameters->num_anchors, numStates, xt, P,
               &b_height_offset_pressure);
    for (i = 0; i < 4; i++) {
      x_att[i] = b_x_att[i];
    }

    memcpy(&P_att[0], &b_P_att[0], 9U * sizeof(double));
    for (i = 0; i < 24; i++) {
      updateVect[i] = 0.0;
    }

    height_offset_pressure = b_height_offset_pressure;
    initialized_not_empty = true;
    initializing_attitude = 1.0;
    ext_pose_offset_initialized = false;
    for (i = 0; i < 3; i++) {
      ext_pos_offset[i] = 0.0;
    }

    eye(ext_att_offset);
    i15 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    i = xt->size[0];
    for (i15 = 0; i15 < i; i15++) {
      xt_out->data[i15] = xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i15 = 0; i15 < i; i15++) {
      P_apo_out->data[i15] = P->data[i15];
    }

    i15 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i15, (int)sizeof(double));
    i = (int)(numTrackFeatures * 4.0);
    for (i15 = 0; i15 < i; i15++) {
      h_u_apo_out->data[i15] = 0.0;
    }

    i15 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = (int)numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)map_out, i15, (int)sizeof(double));
    i = 3 * (int)numTrackFeatures;
    for (i15 = 0; i15 < i; i15++) {
      map_out->data[i15] = 0.0;
    }

    for (i = 0; i < 3; i++) {
      last_u[i] = 0.0;

      //  the last control outputs (in camera frame)
      i_control[i] = 0.0;
    }
  } else if (initializing_attitude > 0.0) {
    for (i = 0; i < 3; i++) {
      b_measurements[i] = measurements_pred.gyr_duo[i] - gyro_offset[i];
    }

    Att_pred(x_att, P_att, b_measurements, noiseParameters->process_noise[1] *
             1000.0, dt);
    Att_upd(x_att, P_att, measurements_pred.acc_duo,
            noiseParameters->process_noise[0] * 1.0E+10, dt);
    for (i15 = 0; i15 < 4; i15++) {
      xt->data[3 + i15] = x_att[i15];
    }

    initializing_attitude++;
    if (initializing_attitude >= 10.0) {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      R_cw[0] = ((x_att[0] * x_att[0] - x_att[1] * x_att[1]) - x_att[2] * x_att
                 [2]) + x_att[3] * x_att[3];
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
      for (i15 = 0; i15 < 3; i15++) {
        for (i = 0; i < 3; i++) {
          dv9[i15 + 3 * i] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            dv9[i15 + 3 * i] += 0.1 * R_cw[i15 + 3 * i16] * (double)b[i16 + 3 *
              i];
          }
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i = 0; i < 3; i++) {
          P->data[(i15 + P->size[0] * (3 + i)) + 3] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            P->data[(i15 + P->size[0] * (3 + i)) + 3] += dv9[i15 + 3 * i16] *
              R_cw[i + 3 * i16];
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
                 noiseParameters->image_noise, noiseParameters->sigmaInit,
                 noiseParameters->pressure_noise, noiseParameters->ext_pos_noise,
                 noiseParameters->ext_att_noise,
                 noiseParameters->gravity_alignment_noise, &measurements_pred,
                 height_offset_pressure, *b_VIOParameters, h_u_apo_out, map_out);
      initializing_attitude = 0.0;
    } else {
      i15 = h_u_apo_out->size[0];
      h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_apo_out, i15, (int)sizeof(double));
      i = (int)(numTrackFeatures * 4.0);
      for (i15 = 0; i15 < i; i15++) {
        h_u_apo_out->data[i15] = 0.0;
      }

      i15 = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, i15, (int)sizeof(double));
      i = 3 * (int)numTrackFeatures;
      for (i15 = 0; i15 < i; i15++) {
        map_out->data[i15] = 0.0;
      }
    }

    i15 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    i = xt->size[0];
    for (i15 = 0; i15 < i; i15++) {
      xt_out->data[i15] = xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i15 = 0; i15 < i; i15++) {
      P_apo_out->data[i15] = P->data[i15];
    }
  } else {
    if (b_VIOParameters->use_ext_pose && (!ext_pose_offset_initialized)) {
      for (i15 = 0; i15 < 3; i15++) {
        ext_pos_offset[i15] = xt->data[i15] - measurements_pred.pos_ext[i15];
      }

      //  in vio frame
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
      b_measurements_pred[0] = ((measurements_pred.att_ext[0] *
        measurements_pred.att_ext[0] - measurements_pred.att_ext[1] *
        measurements_pred.att_ext[1]) - measurements_pred.att_ext[2] *
        measurements_pred.att_ext[2]) + measurements_pred.att_ext[3] *
        measurements_pred.att_ext[3];
      b_measurements_pred[1] = 2.0 * (measurements_pred.att_ext[0] *
        measurements_pred.att_ext[1] + measurements_pred.att_ext[2] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[2] = 2.0 * (measurements_pred.att_ext[0] *
        measurements_pred.att_ext[2] - measurements_pred.att_ext[1] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[3] = 2.0 * (measurements_pred.att_ext[0] *
        measurements_pred.att_ext[1] - measurements_pred.att_ext[2] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[4] = ((-(measurements_pred.att_ext[0] *
        measurements_pred.att_ext[0]) + measurements_pred.att_ext[1] *
        measurements_pred.att_ext[1]) - measurements_pred.att_ext[2] *
        measurements_pred.att_ext[2]) + measurements_pred.att_ext[3] *
        measurements_pred.att_ext[3];
      b_measurements_pred[5] = 2.0 * (measurements_pred.att_ext[1] *
        measurements_pred.att_ext[2] + measurements_pred.att_ext[0] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[6] = 2.0 * (measurements_pred.att_ext[0] *
        measurements_pred.att_ext[2] + measurements_pred.att_ext[1] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[7] = 2.0 * (measurements_pred.att_ext[1] *
        measurements_pred.att_ext[2] - measurements_pred.att_ext[0] *
        measurements_pred.att_ext[3]);
      b_measurements_pred[8] = ((-(measurements_pred.att_ext[0] *
        measurements_pred.att_ext[0]) - measurements_pred.att_ext[1] *
        measurements_pred.att_ext[1]) + measurements_pred.att_ext[2] *
        measurements_pred.att_ext[2]) + measurements_pred.att_ext[3] *
        measurements_pred.att_ext[3];
      dv9[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
                xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      dv9[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
      dv9[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
      dv9[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
      dv9[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
                xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      dv9[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
      dv9[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
      dv9[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
      dv9[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
                xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      for (i15 = 0; i15 < 3; i15++) {
        for (i = 0; i < 3; i++) {
          ext_att_offset[i15 + 3 * i] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            ext_att_offset[i15 + 3 * i] += b_measurements_pred[i15 + 3 * i16] *
              dv9[i16 + 3 * i];
          }
        }
      }

      ext_pose_offset_initialized = true;
    } else {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
    }

    if (b_VIOParameters->use_controller_to_predict) {
      j_fprintf(xt->data[13], xt->data[14], xt->data[15]);
      diag(*(double (*)[3])&xt->data[13], R_cw);
      for (i15 = 0; i15 < 3; i15++) {
        measurements_pred.acc_duo[i15] = 0.0;
        for (i = 0; i < 3; i++) {
          measurements_pred.acc_duo[i15] += R_cw[i15 + 3 * i] * last_u[i];
        }
      }
    } else {
      for (i = 0; i < 3; i++) {
        measurements_pred.acc_duo[i] = 0.0;
      }
    }

    SLAM_pred(P, xt, dt, noiseParameters->process_noise,
              measurements_pred.gyr_duo, measurements_pred.acc_duo, numStates);
    SLAM_updIT(P, xt, cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters->image_noise, noiseParameters->sigmaInit,
               noiseParameters->pressure_noise, noiseParameters->ext_pos_noise,
               noiseParameters->ext_att_noise,
               noiseParameters->gravity_alignment_noise, &measurements_upd,
               height_offset_pressure, *b_VIOParameters, h_u_apo_out, map_out);
    i15 = r3->size[0];
    r3->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)r3, i15, (int)sizeof(boolean_T));
    i = xt->size[0];
    for (i15 = 0; i15 < i; i15++) {
      r3->data[i15] = rtIsNaN(xt->data[i15]);
    }

    if (e_any(r3)) {
      b_ros_error();
      initialize(ref->position, measurements->bar_fmu,
                 b_VIOParameters->num_points_per_anchor,
                 b_VIOParameters->num_anchors, numStates, xt, P,
                 &b_height_offset_pressure);
      for (i = 0; i < 4; i++) {
        x_att[i] = b_x_att[i];
      }

      memcpy(&P_att[0], &b_P_att[0], 9U * sizeof(double));
      for (i = 0; i < 24; i++) {
        updateVect[i] = 0.0;
      }

      height_offset_pressure = b_height_offset_pressure;
      ext_pose_offset_initialized = false;
      eye(ext_att_offset);
      for (i = 0; i < 3; i++) {
        ext_pos_offset[i] = 0.0;
        i_control[i] = 0.0;
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
    yaw = rt_atan2d_snf(R_cw[3], R_cw[0]) + 1.5707963267948966;
    if (rtIsNaN(ref->position[0])) {
      c_ros_error();
      ref->position[0] = 0.0;
    }

    if (rtIsNaN(ref->position[1])) {
      d_ros_error();
      ref->position[1] = 0.0;
    }

    if (rtIsNaN(ref->position[2])) {
      e_ros_error();
      ref->position[2] = 0.0;
    }

    if (rtIsNaN(ref->position[3])) {
      f_ros_error();
      ref->position[3] = 0.0;
    }

    if (rtIsNaN(ref->velocity[0])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i15 = 0; i15 < 42; i15++) {
        cv18[i15] = cv19[i15];
      }

      ROS_ERROR(cv18);
      ref->position[0] = 0.0;
    }

    if (rtIsNaN(ref->velocity[1])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i15 = 0; i15 < 42; i15++) {
        cv18[i15] = cv20[i15];
      }

      ROS_ERROR(cv18);
      ref->position[1] = 0.0;
    }

    if (rtIsNaN(ref->velocity[2])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i15 = 0; i15 < 42; i15++) {
        cv18[i15] = cv21[i15];
      }

      ROS_ERROR(cv18);
      ref->position[2] = 0.0;
    }

    if (rtIsNaN(ref->position[3])) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (i15 = 0; i15 < 39; i15++) {
        cv22[i15] = cv23[i15];
      }

      ROS_ERROR(cv22);
      ref->position[3] = 0.0;
    }

    //  transform between world and control frame (yaw-rotatate world frame)
    R_bw[0] = cos(yaw);
    R_bw[1] = -sin(yaw);
    R_bw[2] = 0.0;
    R_bw[3] = sin(yaw);
    R_bw[4] = cos(yaw);
    R_bw[5] = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      R_bw[6 + i15] = iv4[i15];
    }

    for (i15 = 0; i15 < 3; i15++) {
      b_measurements[i15] = xt->data[i15] - ref->position[i15];
    }

    for (i15 = 0; i15 < 3; i15++) {
      err_p_b[i15] = 0.0;
      for (i = 0; i < 3; i++) {
        err_p_b[i15] += R_bw[i15 + 3 * i] * b_measurements[i];
      }
    }

    //  position error in control frame
    for (i15 = 0; i15 < 3; i15++) {
      b_measurements[i15] = xt->data[7 + i15] - ref->velocity[i15];
    }

    //  velocity error in control frame
    c_ControllerGains[0] = b_ControllerGains->Ki_xy * err_p_b[0];
    c_ControllerGains[1] = b_ControllerGains->Ki_xy * err_p_b[1];
    c_ControllerGains[2] = b_ControllerGains->Ki_z * err_p_b[2];
    for (i15 = 0; i15 < 3; i15++) {
      err_v_b[i15] = 0.0;
      for (i = 0; i < 3; i++) {
        err_v_b[i15] += R_bw[i15 + 3 * i] * b_measurements[i];
      }

      i_control[i15] += c_ControllerGains[i15];
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
    u_out_pos[0] = -((b_ControllerGains->Kp_xy * err_p_b[0] + i_control[0]) +
                     b_ControllerGains->Kd_xy * err_v_b[0]);
    u_out_pos[1] = -((b_ControllerGains->Kp_xy * err_p_b[1] + i_control[1]) +
                     b_ControllerGains->Kd_xy * err_v_b[1]);
    u_out_pos[2] = -((b_ControllerGains->Kp_z * err_p_b[2] + i_control[2]) +
                     b_ControllerGains->Kd_z * err_v_b[2]);
    if (u_out_pos[2] >= 0.05) {
    } else {
      u_out_pos[2] = 0.05;
    }

    //  dont send negative force commands
    //  yaw_ctrl = atan2(R_cw(1,2), R_cw(1,1));
    //  fprintf('control frame error: %.3f %.3f %.3f %.3f\n', err_p_b(1), err_p_b(2), err_p_b(3), (yaw_trafo - ref.position(4))) 
    for (i = 0; i < 3; i++) {
      u_out[i] = u_out_pos[i];
    }

    u_out[3] = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * (yaw - ref->position[3]);
    l_fprintf(err_p_b[0], err_p_b[1], err_p_b[2], (rt_atan2d_snf(R_cw[3], R_cw[0])
               + 1.5707963267948966) - ref->position[3], u_out[0], u_out[1],
              u_out[2], u_out[3]);
    for (i15 = 0; i15 < 3; i15++) {
      last_u[i15] = 0.0;
      for (i = 0; i < 3; i++) {
        last_u[i15] += R_bw[i + 3 * i15] * u_out_pos[i];
      }
    }

    i15 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    i = xt->size[0];
    for (i15 = 0; i15 < i; i15++) {
      xt_out->data[i15] = xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i15 = 0; i15 < i; i15++) {
      P_apo_out->data[i15] = P->data[i15];
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
