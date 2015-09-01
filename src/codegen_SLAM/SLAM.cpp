//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:11:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "blkdiag.h"
#include "repmat.h"
#include "QuatFromRotJ.h"
#include "rdivide.h"
#include "norm.h"
#include "cross.h"
#include "eye.h"
#include "Att_upd.h"
#include "Att_pred.h"
#include "ros_warn.h"
#include "SLAM_pred.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Type Definitions
#ifndef struct_emxArray_real_T_3x3
#define struct_emxArray_real_T_3x3

struct emxArray_real_T_3x3
{
  double data[9];
  int size[2];
};

#endif                                 //struct_emxArray_real_T_3x3

#ifndef struct_emxArray_real_T_4
#define struct_emxArray_real_T_4

struct emxArray_real_T_4
{
  double data[4];
  int size[1];
};

#endif                                 //struct_emxArray_real_T_4

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double height_offset_pressure;
static double last_u[4];
static boolean_T ext_pose_offset_initialized;
static double ext_pos_offset[3];
static double ext_att_offset[9];
static double i_control[3];
static double initializing_attitude;
static emxArray_real_T_4 x_att;
static emxArray_real_T_3x3 P_att;

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
// NOTE: Comment this out for MEXing
// Arguments    : double updateVect[32]
//                const double z_all_l[64]
//                const double z_all_r[64]
//                double dt
//                const VIOMeasurements *measurements
//                const ReferenceCommand *ref
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
void SLAM(double updateVect[32], const double z_all_l[64], const double z_all_r
          [64], double dt, const VIOMeasurements *measurements, const
          ReferenceCommand *ref, const VIOParameters *b_VIOParameters, const
          StereoParameters *cameraParameters, const NoiseParameters
          *noiseParameters, const ControllerGains *b_ControllerGains, boolean_T
          resetFlag, emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out,
          emxArray_real_T *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int i;
  double numTrackFeatures;
  double measurements__gyr_duo[3];
  double measurements__acc_duo[3];
  double measurements__att_fmu[4];
  double measurements__pos_ext[3];
  double measurements__att_ext[4];
  emxArray_real_T *b_P;
  emxArray_real_T *xt_apo;
  emxArray_real_T *unusedU1;
  emxArray_real_T *r3;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *r9;
  double b_measurements__att_fmu[9];
  int i13;
  double R_bw[9];
  int k;
  double minval;
  double err_p_b[3];
  double err_v_b[3];
  static const double dv16[3] = { 1.0, 0.0, 0.0 };

  double b_err_v_b[3];
  double u_out_pos[3];
  double u_pred[4];
  static const double y[9] = { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

  static const signed char b_y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int P_att_size[2];
  double b_measurements__att_ext[9];
  double dv17[9];
  double dv18[9];
  double c_ControllerGains[3];
  double u_out_yaw;
  double updateVect_apo[32];
  int tmp_data[32];
  static const double c_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  numTrackFeatures = b_VIOParameters->num_anchors *
    b_VIOParameters->num_points_per_anchor;

  //  copy for coder
  //  noiseParameters_.image_noise(1) = noiseParameters_.image_noise(1) / cameraParameters.CameraParameters1.FocalLength(1); 
  //  noiseParameters_.image_noise(2) = noiseParameters_.image_noise(2) / cameraParameters.CameraParameters1.FocalLength(2); 
  for (i = 0; i < 3; i++) {
    measurements__gyr_duo[i] = measurements->gyr_duo[i];
    measurements__acc_duo[i] = measurements->acc_duo[i];
  }

  for (i = 0; i < 4; i++) {
    measurements__att_fmu[i] = measurements->att_fmu[i];
  }

  for (i = 0; i < 3; i++) {
    measurements__pos_ext[i] = measurements->pos_ext[i];
  }

  for (i = 0; i < 4; i++) {
    measurements__att_ext[i] = measurements->att_ext[i];
  }

  //  copy for coder
  emxInit_real_T(&b_P, 2);
  b_emxInit_real_T(&xt_apo, 1);
  emxInit_real_T(&unusedU1, 2);
  emxInit_real_T(&r3, 2);
  emxInit_real_T(&r4, 2);
  b_emxInit_real_T(&r5, 1);
  emxInit_real_T(&r6, 2);
  emxInit_real_T(&r7, 2);
  b_emxInit_real_T(&r8, 1);
  b_emxInit_real_T(&r9, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;
    ext_pose_offset_initialized = false;
    for (i = 0; i < 3; i++) {
      ext_pos_offset[i] = 0.0;
    }

    eye(ext_att_offset);
    for (i = 0; i < 32; i++) {
      updateVect[i] = 0.0;
    }

    if (b_VIOParameters->use_orientation) {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      b_measurements__att_fmu[0] = ((measurements__att_fmu[0] *
        measurements__att_fmu[0] - measurements__att_fmu[1] *
        measurements__att_fmu[1]) - measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      b_measurements__att_fmu[3] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[1] + measurements__att_fmu[2] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[6] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[2] - measurements__att_fmu[1] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[1] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[1] - measurements__att_fmu[2] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[4] = ((-(measurements__att_fmu[0] *
        measurements__att_fmu[0]) + measurements__att_fmu[1] *
        measurements__att_fmu[1]) - measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      b_measurements__att_fmu[7] = 2.0 * (measurements__att_fmu[1] *
        measurements__att_fmu[2] + measurements__att_fmu[0] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[2] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[2] + measurements__att_fmu[1] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[5] = 2.0 * (measurements__att_fmu[1] *
        measurements__att_fmu[2] - measurements__att_fmu[0] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[8] = ((-(measurements__att_fmu[0] *
        measurements__att_fmu[0]) - measurements__att_fmu[1] *
        measurements__att_fmu[1]) + measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          R_bw[i13 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            R_bw[i13 + 3 * i] += R_bc[k + 3 * i13] * b_measurements__att_fmu[k +
              3 * i];
          }
        }
      }

      initializing_attitude = 0.0;
      x_att.size[0] = 1;
      x_att.data[0] = 0.0;

      //  for coder
      P_att.size[0] = 1;
      P_att.size[1] = 1;
      P_att.data[0] = 0.0;

      //  for coder
    } else {
      minval = norm(measurements__acc_duo);
      for (i = 0; i < 3; i++) {
        err_p_b[i] = measurements__acc_duo[i] / minval;
      }

      cross(err_p_b, dv16, err_v_b);
      for (i = 0; i < 3; i++) {
        b_err_v_b[i] = err_v_b[i];
      }

      rdivide(b_err_v_b, norm(err_v_b), err_v_b);
      cross(err_v_b, err_p_b, u_out_pos);
      for (i = 0; i < 3; i++) {
        b_err_v_b[i] = u_out_pos[i];
      }

      rdivide(b_err_v_b, norm(u_out_pos), u_out_pos);
      for (i13 = 0; i13 < 3; i13++) {
        R_bw[i13] = u_out_pos[i13];
        R_bw[3 + i13] = err_v_b[i13];
        R_bw[6 + i13] = err_p_b[i13];
      }

      QuatFromRotJ(R_bw, u_pred);
      x_att.size[0] = 4;
      for (i13 = 0; i13 < 4; i13++) {
        x_att.data[i13] = u_pred[i13];
      }

      P_att.size[0] = 3;
      P_att.size[1] = 3;
      for (i13 = 0; i13 < 9; i13++) {
        P_att.data[i13] = y[i13];
      }

      initializing_attitude = 1.0;
    }

    QuatFromRotJ(R_bw, u_pred);
    i13 = r8->size[0];
    r8->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)r8, i13, (int)sizeof(double));
    r8->data[0] = 0.0;
    r8->data[1] = 0.0;
    r8->data[2] = 0.0;
    r8->data[3] = 0.0;
    r8->data[4] = 0.0;
    r8->data[5] = 0.0;
    r8->data[6] = 1.0;
    i = (int)b_VIOParameters->num_points_per_anchor;
    for (i13 = 0; i13 < i; i13++) {
      r8->data[i13 + 7] = 0.0;
    }

    repmat(r8, b_VIOParameters->num_anchors, xt_apo);
    i13 = xt->size[0];
    xt->size[0] = 13 + xt_apo->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i13, (int)sizeof(double));
    for (i13 = 0; i13 < 3; i13++) {
      xt->data[i13] = ref->position[i13];
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt->data[i13 + 3] = u_pred[i13];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (i13 = 0; i13 < 3; i13++) {
      xt->data[i13 + 10] = measurements__gyr_duo[i13];
    }

    i = xt_apo->size[0];
    for (i13 = 0; i13 < i; i13++) {
      xt->data[i13 + 13] = xt_apo->data[i13];
    }

    //  initial real vector
    minval = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    i13 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)numStates;
    r6->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r6, i13, (int)sizeof(double));
    i = (int)numStates * (int)numStates;
    for (i13 = 0; i13 < i; i13++) {
      r6->data[i13] = 0.0;
    }

    i13 = r7->size[0] * r7->size[1];
    r7->size[0] = (int)minval;
    r7->size[1] = (int)minval;
    emxEnsureCapacity((emxArray__common *)r7, i13, (int)sizeof(double));
    i = (int)minval * (int)minval;
    for (i13 = 0; i13 < i; i13++) {
      r7->data[i13] = 0.0;
    }

    blkdiag(r6, r7, P);

    //  initial error state covariance
    for (i13 = 0; i13 < 3; i13++) {
      for (i = 0; i < 3; i++) {
        P->data[i + P->size[0] * i13] = 0.0;
      }
    }

    //  position
    for (i13 = 0; i13 < 3; i13++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (3 + i13)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i13 = 0; i13 < 3; i13++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (6 + i13)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i13 = 0; i13 < 3; i13++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (9 + i13)) + 9] = b_y[i + 3 * i13];
      }
    }

    //  gyro bias
    height_offset_pressure = (1.0 - rt_powd_snf(measurements->bar_fmu / 101325.0,
      0.190284)) * 145366.45;
    if (b_VIOParameters->use_orientation) {
      for (i = 0; i < 32; i++) {
        updateVect[i] = 0.0;
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
                 *b_VIOParameters, xt_apo, unusedU1);
    }

    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    i = xt->size[0];
    for (i13 = 0; i13 < i; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i13 = 0; i13 < i; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    i13 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i13, (int)sizeof(double));
    i = (int)(numTrackFeatures * 4.0);
    for (i13 = 0; i13 < i; i13++) {
      h_u_apo_out->data[i13] = 0.0;
    }

    i13 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = (int)numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)map_out, i13, (int)sizeof(double));
    i = 3 * (int)numTrackFeatures;
    for (i13 = 0; i13 < i; i13++) {
      map_out->data[i13] = 0.0;
    }

    for (i = 0; i < 4; i++) {
      last_u[i] = 0.0;
    }

    //  the last control outputs (in camera frame)
    for (i = 0; i < 3; i++) {
      i_control[i] = 0.0;
    }
  } else if (initializing_attitude > 0.0) {
    Att_pred(x_att.data, P_att.data, P_att.size, measurements__gyr_duo,
             noiseParameters->process_noise[1], dt, u_pred, R_bw);
    x_att.size[0] = 4;
    for (i13 = 0; i13 < 4; i13++) {
      x_att.data[i13] = u_pred[i13];
    }

    P_att.size[0] = 3;
    P_att.size[1] = 3;
    for (i13 = 0; i13 < 9; i13++) {
      P_att.data[i13] = R_bw[i13];
    }

    P_att_size[0] = P_att.size[0];
    P_att_size[1] = P_att.size[1];
    i = P_att.size[0] * P_att.size[1];
    for (i13 = 0; i13 < i; i13++) {
      b_measurements__att_ext[i13] = P_att.data[i13];
    }

    Att_upd(x_att.data, b_measurements__att_ext, P_att_size,
            measurements__acc_duo, noiseParameters->process_noise[0], dt, u_pred);
    x_att.size[0] = 4;
    for (i13 = 0; i13 < 4; i13++) {
      x_att.data[i13] = u_pred[i13];
    }

    P_att.size[0] = 3;
    P_att.size[1] = 3;
    for (i13 = 0; i13 < 9; i13++) {
      P_att.data[i13] = b_measurements__att_ext[i13];
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt->data[3 + i13] = x_att.data[i13];
    }

    initializing_attitude++;
    if (initializing_attitude == 10.0) {
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i13)) + 3] = P_att.data[i + 3 * i13];
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
                 *b_VIOParameters, h_u_apo_out, map_out);
      initializing_attitude = 0.0;
    } else {
      i13 = h_u_apo_out->size[0];
      h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_apo_out, i13, (int)sizeof(double));
      i = (int)(numTrackFeatures * 4.0);
      for (i13 = 0; i13 < i; i13++) {
        h_u_apo_out->data[i13] = 0.0;
      }

      i13 = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, i13, (int)sizeof(double));
      i = 3 * (int)numTrackFeatures;
      for (i13 = 0; i13 < i; i13++) {
        map_out->data[i13] = 0.0;
      }
    }

    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    i = xt->size[0];
    for (i13 = 0; i13 < i; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i13 = 0; i13 < i; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }
  } else {
    if (b_VIOParameters->use_ext_pose && (!ext_pose_offset_initialized)) {
      for (i13 = 0; i13 < 3; i13++) {
        ext_pos_offset[i13] = xt->data[i13] - measurements__pos_ext[i13];
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
      b_measurements__att_ext[0] = ((measurements__att_ext[0] *
        measurements__att_ext[0] - measurements__att_ext[1] *
        measurements__att_ext[1]) - measurements__att_ext[2] *
        measurements__att_ext[2]) + measurements__att_ext[3] *
        measurements__att_ext[3];
      b_measurements__att_ext[1] = 2.0 * (measurements__att_ext[0] *
        measurements__att_ext[1] + measurements__att_ext[2] *
        measurements__att_ext[3]);
      b_measurements__att_ext[2] = 2.0 * (measurements__att_ext[0] *
        measurements__att_ext[2] - measurements__att_ext[1] *
        measurements__att_ext[3]);
      b_measurements__att_ext[3] = 2.0 * (measurements__att_ext[0] *
        measurements__att_ext[1] - measurements__att_ext[2] *
        measurements__att_ext[3]);
      b_measurements__att_ext[4] = ((-(measurements__att_ext[0] *
        measurements__att_ext[0]) + measurements__att_ext[1] *
        measurements__att_ext[1]) - measurements__att_ext[2] *
        measurements__att_ext[2]) + measurements__att_ext[3] *
        measurements__att_ext[3];
      b_measurements__att_ext[5] = 2.0 * (measurements__att_ext[1] *
        measurements__att_ext[2] + measurements__att_ext[0] *
        measurements__att_ext[3]);
      b_measurements__att_ext[6] = 2.0 * (measurements__att_ext[0] *
        measurements__att_ext[2] + measurements__att_ext[1] *
        measurements__att_ext[3]);
      b_measurements__att_ext[7] = 2.0 * (measurements__att_ext[1] *
        measurements__att_ext[2] - measurements__att_ext[0] *
        measurements__att_ext[3]);
      b_measurements__att_ext[8] = ((-(measurements__att_ext[0] *
        measurements__att_ext[0]) - measurements__att_ext[1] *
        measurements__att_ext[1]) + measurements__att_ext[2] *
        measurements__att_ext[2]) + measurements__att_ext[3] *
        measurements__att_ext[3];
      dv17[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      dv17[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
      dv17[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
      dv17[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
      dv17[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      dv17[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
      dv17[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
      dv17[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
      dv17[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          ext_att_offset[i13 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            ext_att_offset[i13 + 3 * i] += b_measurements__att_ext[i13 + 3 * k] *
              dv17[k + 3 * i];
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

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    dv18[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv18[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    dv18[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    dv18[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    dv18[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv18[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    dv18[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    dv18[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    dv18[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (i13 = 0; i13 < 3; i13++) {
      for (i = 0; i < 3; i++) {
        R_bw[i13 + 3 * i] = 0.0;
        for (k = 0; k < 3; k++) {
          R_bw[i13 + 3 * i] += R_bc[i13 + 3 * k] * dv18[k + 3 * i];
        }
      }
    }

    for (i13 = 0; i13 < 3; i13++) {
      b_err_v_b[i13] = xt->data[i13] - ref->position[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      err_p_b[i13] = 0.0;
      for (i = 0; i < 3; i++) {
        err_p_b[i13] += R_bw[i13 + 3 * i] * b_err_v_b[i];
      }
    }

    //  position error in body frame
    for (i13 = 0; i13 < 3; i13++) {
      b_err_v_b[i13] = xt->data[7 + i13] - ref->velocity[i13];
    }

    //  velocity error in body frame
    c_ControllerGains[0] = b_ControllerGains->Ki_xy * err_p_b[0];
    c_ControllerGains[1] = b_ControllerGains->Ki_xy * err_p_b[1];
    c_ControllerGains[2] = b_ControllerGains->Ki_z * err_p_b[2];
    for (i13 = 0; i13 < 3; i13++) {
      err_v_b[i13] = 0.0;
      for (i = 0; i < 3; i++) {
        err_v_b[i13] += R_bw[i13 + 3 * i] * b_err_v_b[i];
      }

      i_control[i13] += c_ControllerGains[i13];
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
    u_out_yaw = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_bw[3], R_bw[0]) -
      ref->position[3]);
    for (i = 0; i < 3; i++) {
      u_out[i] = u_out_pos[i];
    }

    u_out[3] = u_out_yaw;

    //  fprintf('position error (%.3f, %.3f, %.3f, %.3f), control: (%.3f, %.3f, %.3f, %.3f)\n', xt(1) - ref(1), xt(2) - ref(2), xt(3) - ref(3), yaw - ref(4), u_out_x, u_out_y, u_out_z, u_out_yaw); 
    if (b_VIOParameters->use_controller_to_predict) {
      for (i = 0; i < 4; i++) {
        u_pred[i] = last_u[i];
      }
    } else {
      for (i = 0; i < 4; i++) {
        u_pred[i] = 0.0;
      }
    }

    for (i13 = 0; i13 < 3; i13++) {
      b_err_v_b[i13] = 0.0;
      for (i = 0; i < 3; i++) {
        b_err_v_b[i13] += R_bc[i + 3 * i13] * u_out_pos[i];
      }

      last_u[i13] = b_err_v_b[i13];
    }

    last_u[3] = u_out_yaw;
    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements__gyr_duo,
              measurements__acc_duo, numStates, u_pred, b_P);
    i13 = P->size[0] * P->size[1];
    P->size[0] = b_P->size[0];
    P->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)P, i13, (int)sizeof(double));
    i = b_P->size[0] * b_P->size[1];
    for (i13 = 0; i13 < i; i13++) {
      P->data[i13] = b_P->data[i13];
    }

    i13 = xt_apo->size[0];
    xt_apo->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_apo, i13, (int)sizeof(double));
    i = xt->size[0];
    for (i13 = 0; i13 < i; i13++) {
      xt_apo->data[i13] = xt->data[i13];
    }

    for (i = 0; i < 32; i++) {
      updateVect_apo[i] = updateVect[i];
    }

    SLAM_updIT(P, xt_apo, cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect_apo, z_all_l, z_all_r,
               noiseParameters->image_noise, noiseParameters->sigmaInit,
               *b_VIOParameters, h_u_apo_out, map_out);

    //  if almost all features were lost, do a soft reset
    i = 0;
    for (k = 0; k < 32; k++) {
      if ((updateVect[k] != 0.0) && (updateVect_apo[k] == 1.0)) {
        i++;
      }
    }

    if (i < 0.0 * b_VIOParameters->num_points_per_anchor / 2.0) {
      b_ros_warn();
      for (i = 0; i < 32; i++) {
        updateVect[i] = 0.0;
      }

      if (1.0 > b_VIOParameters->num_points_per_anchor) {
        i = 0;
      } else {
        i = (int)b_VIOParameters->num_points_per_anchor;
      }

      for (i13 = 0; i13 < i; i13++) {
        tmp_data[i13] = i13;
      }

      for (i13 = 0; i13 < i; i13++) {
        updateVect[tmp_data[i13]] = 2.0;
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          R_bw[i + 3 * i13] = R_bc[i13 + 3 * i];
        }
      }

      b_measurements__att_fmu[0] = ((measurements__att_fmu[0] *
        measurements__att_fmu[0] - measurements__att_fmu[1] *
        measurements__att_fmu[1]) - measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      b_measurements__att_fmu[3] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[1] + measurements__att_fmu[2] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[6] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[2] - measurements__att_fmu[1] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[1] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[1] - measurements__att_fmu[2] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[4] = ((-(measurements__att_fmu[0] *
        measurements__att_fmu[0]) + measurements__att_fmu[1] *
        measurements__att_fmu[1]) - measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      b_measurements__att_fmu[7] = 2.0 * (measurements__att_fmu[1] *
        measurements__att_fmu[2] + measurements__att_fmu[0] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[2] = 2.0 * (measurements__att_fmu[0] *
        measurements__att_fmu[2] + measurements__att_fmu[1] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[5] = 2.0 * (measurements__att_fmu[1] *
        measurements__att_fmu[2] - measurements__att_fmu[0] *
        measurements__att_fmu[3]);
      b_measurements__att_fmu[8] = ((-(measurements__att_fmu[0] *
        measurements__att_fmu[0]) - measurements__att_fmu[1] *
        measurements__att_fmu[1]) + measurements__att_fmu[2] *
        measurements__att_fmu[2]) + measurements__att_fmu[3] *
        measurements__att_fmu[3];
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          b_measurements__att_ext[i13 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            b_measurements__att_ext[i13 + 3 * i] += R_bw[i13 + 3 * k] *
              b_measurements__att_fmu[k + 3 * i];
          }
        }
      }

      QuatFromRotJ(b_measurements__att_ext, u_pred);
      i13 = r5->size[0];
      r5->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
      emxEnsureCapacity((emxArray__common *)r5, i13, (int)sizeof(double));
      r5->data[0] = 0.0;
      r5->data[1] = 0.0;
      r5->data[2] = 0.0;
      r5->data[3] = 0.0;
      r5->data[4] = 0.0;
      r5->data[5] = 0.0;
      r5->data[6] = 1.0;
      i = (int)b_VIOParameters->num_points_per_anchor;
      for (i13 = 0; i13 < i; i13++) {
        r5->data[i13 + 7] = 0.0;
      }

      repmat(r5, b_VIOParameters->num_anchors, xt_apo);
      i13 = r9->size[0];
      r9->size[0] = 13 + xt_apo->size[0];
      emxEnsureCapacity((emxArray__common *)r9, i13, (int)sizeof(double));
      for (i13 = 0; i13 < 3; i13++) {
        r9->data[i13] = xt->data[i13];
      }

      for (i13 = 0; i13 < 4; i13++) {
        r9->data[i13 + 3] = u_pred[i13];
      }

      r9->data[7] = 0.0;
      r9->data[8] = 0.0;
      r9->data[9] = 0.0;
      r9->data[10] = 0.0;
      r9->data[11] = 0.0;
      r9->data[12] = 0.0;
      i = xt_apo->size[0];
      for (i13 = 0; i13 < i; i13++) {
        r9->data[i13 + 13] = xt_apo->data[i13];
      }

      i13 = xt->size[0];
      xt->size[0] = r9->size[0];
      emxEnsureCapacity((emxArray__common *)xt, i13, (int)sizeof(double));
      i = r9->size[0];
      for (i13 = 0; i13 < i; i13++) {
        xt->data[i13] = r9->data[i13];
      }

      //  initial real vector
      minval = b_VIOParameters->num_anchors * (6.0 +
        b_VIOParameters->num_points_per_anchor);
      i13 = r3->size[0] * r3->size[1];
      r3->size[0] = (int)numStates;
      r3->size[1] = (int)numStates;
      emxEnsureCapacity((emxArray__common *)r3, i13, (int)sizeof(double));
      i = (int)numStates * (int)numStates;
      for (i13 = 0; i13 < i; i13++) {
        r3->data[i13] = 0.0;
      }

      i13 = r4->size[0] * r4->size[1];
      r4->size[0] = (int)minval;
      r4->size[1] = (int)minval;
      emxEnsureCapacity((emxArray__common *)r4, i13, (int)sizeof(double));
      i = (int)minval * (int)minval;
      for (i13 = 0; i13 < i; i13++) {
        r4->data[i13] = 0.0;
      }

      blkdiag(r3, r4, P);

      //  initial error state covariance
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          P->data[i + P->size[0] * i13] = 0.0;
        }
      }

      //  position
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i13)) + 3] = 0.0;
        }
      }

      //  orientation
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (6 + i13)) + 6] = 0.0;
        }
      }

      //  velocity
      for (i13 = 0; i13 < 3; i13++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (9 + i13)) + 9] = c_y[i + 3 * i13];
        }
      }

      //  gyro bias
      for (i = 0; i < 4; i++) {
        u_out[i] = 0.0;
      }
    } else {
      i13 = xt->size[0];
      xt->size[0] = xt_apo->size[0];
      emxEnsureCapacity((emxArray__common *)xt, i13, (int)sizeof(double));
      i = xt_apo->size[0];
      for (i13 = 0; i13 < i; i13++) {
        xt->data[i13] = xt_apo->data[i13];
      }

      memcpy(&updateVect[0], &updateVect_apo[0], sizeof(double) << 5);
    }

    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    i = xt->size[0];
    for (i13 = 0; i13 < i; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i13 = 0; i13 < i; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    // % output asserts for coder
  }

  emxFree_real_T(&r9);
  emxFree_real_T(&r8);
  emxFree_real_T(&r7);
  emxFree_real_T(&r6);
  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
  emxFree_real_T(&r3);
  emxFree_real_T(&unusedU1);
  emxFree_real_T(&xt_apo);
  emxFree_real_T(&b_P);
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
  P_att.size[1] = 0;
  x_att.size[0] = 0;
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
