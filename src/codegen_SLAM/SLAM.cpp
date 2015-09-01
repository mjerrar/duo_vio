//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 16:22:53
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
// Arguments    : double updateVect[24]
//                const double z_all_l[48]
//                const double z_all_r[48]
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
void SLAM(double updateVect[24], const double z_all_l[48], const double z_all_r
          [48], double dt, const VIOMeasurements *measurements, const
          ReferenceCommand *ref, const VIOParameters *b_VIOParameters, const
          StereoParameters *cameraParameters, const NoiseParameters
          *noiseParameters, const ControllerGains *b_ControllerGains, boolean_T
          resetFlag, emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out,
          emxArray_real_T *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int i;
  double numTrackFeatures;
  VIOMeasurements measurements_;
  emxArray_real_T *xt_apo;
  emxArray_real_T *unusedU1;
  emxArray_real_T *r3;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *r9;
  double b_measurements_[9];
  int i14;
  double R_bw[9];
  int k;
  double minval;
  double err_p_b[3];
  double err_v_b[3];
  static const double dv19[3] = { 1.0, 0.0, 0.0 };

  double b_err_v_b[3];
  double u_out_pos[3];
  double u_pred[4];
  static const double y[9] = { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

  static const double b_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  int P_att_size[2];
  double c_measurements_[9];
  double dv20[9];
  double c_ControllerGains[3];
  double u_out_yaw;
  double updateVect_apo[24];
  int tmp_data[24];
  double d_measurements_[9];

  //  fprintf('\nmeasurements.gyr_duo %f %f %f\n', measurements.gyr_duo(1), measurements.gyr_duo(2), measurements.gyr_duo(3)); 
  //  fprintf('measurements.acc_duo %f %f %f\n', measurements.acc_duo(1), measurements.acc_duo(2), measurements.acc_duo(3)); 
  //  fprintf('measurements.mag_duo %f %f %f\n', measurements.mag_duo(1), measurements.mag_duo(2), measurements.mag_duo(3)); 
  //  fprintf('measurements.mag_fmu %f %f %f\n', measurements.mag_fmu(1), measurements.mag_fmu(2), measurements.mag_fmu(3)); 
  //  fprintf('measurements.gyr_fmu %f %f %f\n', measurements.gyr_fmu(1), measurements.gyr_fmu(2), measurements.gyr_fmu(3)); 
  //  fprintf('measurements.acc_fmu %f %f %f\n', measurements.acc_fmu(1), measurements.acc_fmu(2), measurements.acc_fmu(3)); 
  //  fprintf('measurements.att_fmu %f %f %f %f\n', measurements.att_fmu(1), measurements.att_fmu(2), measurements.att_fmu(3), measurements.att_fmu(4)); 
  //  fprintf('measurements.pos_ext %f %f %f\n', measurements.pos_ext(1), measurements.pos_ext(2), measurements.pos_ext(3)); 
  //  fprintf('measurements.att_ext %f %f %f %f\n', measurements.att_ext(1), measurements.att_ext(2), measurements.att_ext(3), measurements.att_ext(3)); 
  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  numTrackFeatures = b_VIOParameters->num_anchors *
    b_VIOParameters->num_points_per_anchor;

  //  copy for coder
  //  noiseParameters_.image_noise(1) = noiseParameters_.image_noise(1) / cameraParameters.CameraParameters1.FocalLength(1); 
  //  noiseParameters_.image_noise(2) = noiseParameters_.image_noise(2) / cameraParameters.CameraParameters1.FocalLength(2); 
  measurements_ = *measurements;

  //  copy for coder
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
    for (i = 0; i < 24; i++) {
      updateVect[i] = 0.0;
    }

    if (b_VIOParameters->use_orientation) {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      b_measurements_[0] = ((measurements_.att_fmu[0] * measurements_.att_fmu[0]
        - measurements_.att_fmu[1] * measurements_.att_fmu[1]) -
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      b_measurements_[3] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[1] + measurements_.att_fmu[2] *
        measurements_.att_fmu[3]);
      b_measurements_[6] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[2] - measurements_.att_fmu[1] *
        measurements_.att_fmu[3]);
      b_measurements_[1] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[1] - measurements_.att_fmu[2] *
        measurements_.att_fmu[3]);
      b_measurements_[4] = ((-(measurements_.att_fmu[0] * measurements_.att_fmu
        [0]) + measurements_.att_fmu[1] * measurements_.att_fmu[1]) -
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      b_measurements_[7] = 2.0 * (measurements_.att_fmu[1] *
        measurements_.att_fmu[2] + measurements_.att_fmu[0] *
        measurements_.att_fmu[3]);
      b_measurements_[2] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[2] + measurements_.att_fmu[1] *
        measurements_.att_fmu[3]);
      b_measurements_[5] = 2.0 * (measurements_.att_fmu[1] *
        measurements_.att_fmu[2] - measurements_.att_fmu[0] *
        measurements_.att_fmu[3]);
      b_measurements_[8] = ((-(measurements_.att_fmu[0] * measurements_.att_fmu
        [0]) - measurements_.att_fmu[1] * measurements_.att_fmu[1]) +
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          R_bw[i14 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            R_bw[i14 + 3 * i] += R_bc[k + 3 * i14] * b_measurements_[k + 3 * i];
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
      minval = norm(measurements_.acc_duo);
      for (i = 0; i < 3; i++) {
        err_p_b[i] = measurements_.acc_duo[i] / minval;
      }

      cross(err_p_b, dv19, err_v_b);
      for (i = 0; i < 3; i++) {
        b_err_v_b[i] = err_v_b[i];
      }

      rdivide(b_err_v_b, norm(err_v_b), err_v_b);
      cross(err_v_b, err_p_b, u_out_pos);
      for (i = 0; i < 3; i++) {
        b_err_v_b[i] = u_out_pos[i];
      }

      rdivide(b_err_v_b, norm(u_out_pos), u_out_pos);
      for (i14 = 0; i14 < 3; i14++) {
        R_bw[i14] = u_out_pos[i14];
        R_bw[3 + i14] = err_v_b[i14];
        R_bw[6 + i14] = err_p_b[i14];
      }

      QuatFromRotJ(R_bw, u_pred);
      x_att.size[0] = 4;
      for (i14 = 0; i14 < 4; i14++) {
        x_att.data[i14] = u_pred[i14];
      }

      P_att.size[0] = 3;
      P_att.size[1] = 3;
      for (i14 = 0; i14 < 9; i14++) {
        P_att.data[i14] = y[i14];
      }

      initializing_attitude = 1.0;
    }

    QuatFromRotJ(R_bw, u_pred);
    i14 = r8->size[0];
    r8->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(double));
    r8->data[0] = 0.0;
    r8->data[1] = 0.0;
    r8->data[2] = 0.0;
    r8->data[3] = 0.0;
    r8->data[4] = 0.0;
    r8->data[5] = 0.0;
    r8->data[6] = 1.0;
    i = (int)b_VIOParameters->num_points_per_anchor;
    for (i14 = 0; i14 < i; i14++) {
      r8->data[i14 + 7] = 0.0;
    }

    repmat(r8, b_VIOParameters->num_anchors, xt_apo);
    i14 = xt->size[0];
    xt->size[0] = 13 + xt_apo->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i14, (int)sizeof(double));
    for (i14 = 0; i14 < 3; i14++) {
      xt->data[i14] = ref->position[i14];
    }

    for (i14 = 0; i14 < 4; i14++) {
      xt->data[i14 + 3] = u_pred[i14];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    i = xt_apo->size[0];
    for (i14 = 0; i14 < i; i14++) {
      xt->data[i14 + 13] = xt_apo->data[i14];
    }

    //  initial real vector
    minval = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    i14 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)numStates;
    r6->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r6, i14, (int)sizeof(double));
    i = (int)numStates * (int)numStates;
    for (i14 = 0; i14 < i; i14++) {
      r6->data[i14] = 0.0;
    }

    i14 = r7->size[0] * r7->size[1];
    r7->size[0] = (int)minval;
    r7->size[1] = (int)minval;
    emxEnsureCapacity((emxArray__common *)r7, i14, (int)sizeof(double));
    i = (int)minval * (int)minval;
    for (i14 = 0; i14 < i; i14++) {
      r7->data[i14] = 0.0;
    }

    blkdiag(r6, r7, P);

    //  initial error state covariance
    for (i14 = 0; i14 < 3; i14++) {
      for (i = 0; i < 3; i++) {
        P->data[i + P->size[0] * i14] = 0.0;
      }
    }

    //  position
    for (i14 = 0; i14 < 3; i14++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (3 + i14)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i14 = 0; i14 < 3; i14++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (6 + i14)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i14 = 0; i14 < 3; i14++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (9 + i14)) + 9] = b_y[i + 3 * i14];
      }
    }

    //  gyro bias
    height_offset_pressure = (1.0 - rt_powd_snf(measurements_.bar_fmu / 101325.0,
      0.190284)) * 145366.45;
    if (b_VIOParameters->use_orientation) {
      for (i = 0; i < 24; i++) {
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
                 noiseParameters->orientation_noise,
                 noiseParameters->pressure_noise, noiseParameters->ext_pos_noise,
                 noiseParameters->ext_att_noise, &measurements_,
                 height_offset_pressure, *b_VIOParameters, xt_apo, unusedU1);
    }

    i14 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i14, (int)sizeof(double));
    i = xt->size[0];
    for (i14 = 0; i14 < i; i14++) {
      xt_out->data[i14] = xt->data[i14];
    }

    i14 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i14, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i14 = 0; i14 < i; i14++) {
      P_apo_out->data[i14] = P->data[i14];
    }

    i14 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i14, (int)sizeof(double));
    i = (int)(numTrackFeatures * 4.0);
    for (i14 = 0; i14 < i; i14++) {
      h_u_apo_out->data[i14] = 0.0;
    }

    i14 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = (int)numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)map_out, i14, (int)sizeof(double));
    i = 3 * (int)numTrackFeatures;
    for (i14 = 0; i14 < i; i14++) {
      map_out->data[i14] = 0.0;
    }

    for (i = 0; i < 4; i++) {
      last_u[i] = 0.0;
    }

    //  the last control outputs (in camera frame)
    for (i = 0; i < 3; i++) {
      i_control[i] = 0.0;
    }
  } else if (initializing_attitude > 0.0) {
    Att_pred(x_att.data, P_att.data, P_att.size, measurements_.gyr_duo,
             noiseParameters->process_noise[1], dt, u_pred, R_bw);
    x_att.size[0] = 4;
    for (i14 = 0; i14 < 4; i14++) {
      x_att.data[i14] = u_pred[i14];
    }

    P_att.size[0] = 3;
    P_att.size[1] = 3;
    for (i14 = 0; i14 < 9; i14++) {
      P_att.data[i14] = R_bw[i14];
    }

    P_att_size[0] = P_att.size[0];
    P_att_size[1] = P_att.size[1];
    i = P_att.size[0] * P_att.size[1];
    for (i14 = 0; i14 < i; i14++) {
      c_measurements_[i14] = P_att.data[i14];
    }

    Att_upd(x_att.data, c_measurements_, P_att_size, measurements_.acc_duo,
            noiseParameters->process_noise[0], dt, u_pred);
    x_att.size[0] = 4;
    for (i14 = 0; i14 < 4; i14++) {
      x_att.data[i14] = u_pred[i14];
    }

    P_att.size[0] = 3;
    P_att.size[1] = 3;
    for (i14 = 0; i14 < 9; i14++) {
      P_att.data[i14] = c_measurements_[i14];
    }

    for (i14 = 0; i14 < 4; i14++) {
      xt->data[3 + i14] = x_att.data[i14];
    }

    initializing_attitude++;
    if (initializing_attitude == 10.0) {
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i14)) + 3] = P_att.data[i + 3 * i14];
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
                 noiseParameters->orientation_noise,
                 noiseParameters->pressure_noise, noiseParameters->ext_pos_noise,
                 noiseParameters->ext_att_noise, &measurements_,
                 height_offset_pressure, *b_VIOParameters, h_u_apo_out, map_out);
      initializing_attitude = 0.0;
    } else {
      i14 = h_u_apo_out->size[0];
      h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_apo_out, i14, (int)sizeof(double));
      i = (int)(numTrackFeatures * 4.0);
      for (i14 = 0; i14 < i; i14++) {
        h_u_apo_out->data[i14] = 0.0;
      }

      i14 = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, i14, (int)sizeof(double));
      i = 3 * (int)numTrackFeatures;
      for (i14 = 0; i14 < i; i14++) {
        map_out->data[i14] = 0.0;
      }
    }

    i14 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i14, (int)sizeof(double));
    i = xt->size[0];
    for (i14 = 0; i14 < i; i14++) {
      xt_out->data[i14] = xt->data[i14];
    }

    i14 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i14, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i14 = 0; i14 < i; i14++) {
      P_apo_out->data[i14] = P->data[i14];
    }
  } else {
    if (b_VIOParameters->use_ext_pose) {
      if (!ext_pose_offset_initialized) {
        for (i14 = 0; i14 < 3; i14++) {
          ext_pos_offset[i14] = xt->data[i14] - measurements_.pos_ext[i14];
        }

        //  in vio frame
        for (i14 = 0; i14 < 3; i14++) {
          measurements_.pos_ext[i14] = xt->data[i14];
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
        b_measurements_[0] = ((measurements_.att_ext[0] * measurements_.att_ext
          [0] - measurements_.att_ext[1] * measurements_.att_ext[1]) -
                              measurements_.att_ext[2] * measurements_.att_ext[2])
          + measurements_.att_ext[3] * measurements_.att_ext[3];
        b_measurements_[1] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] + measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        b_measurements_[2] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] - measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        b_measurements_[3] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] - measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        b_measurements_[4] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) + measurements_.att_ext[1] *
          measurements_.att_ext[1]) - measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        b_measurements_[5] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] + measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        b_measurements_[6] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] + measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        b_measurements_[7] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] - measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        b_measurements_[8] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) - measurements_.att_ext[1] *
          measurements_.att_ext[1]) + measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        c_measurements_[0] = ((xt->data[3] * xt->data[3] - xt->data[4] *
          xt->data[4]) - xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        c_measurements_[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] *
          xt->data[6]);
        c_measurements_[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] *
          xt->data[6]);
        c_measurements_[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] *
          xt->data[6]);
        c_measurements_[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] *
          xt->data[4]) - xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        c_measurements_[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] *
          xt->data[6]);
        c_measurements_[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] *
          xt->data[6]);
        c_measurements_[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] *
          xt->data[6]);
        c_measurements_[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] *
          xt->data[4]) + xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
        for (i14 = 0; i14 < 3; i14++) {
          for (i = 0; i < 3; i++) {
            ext_att_offset[i14 + 3 * i] = 0.0;
            for (k = 0; k < 3; k++) {
              ext_att_offset[i14 + 3 * i] += b_measurements_[i14 + 3 * k] *
                c_measurements_[k + 3 * i];
            }
          }
        }

        for (i14 = 0; i14 < 4; i14++) {
          measurements_.att_ext[i14] = xt->data[3 + i14];
        }

        ext_pose_offset_initialized = true;
      } else {
        for (i14 = 0; i14 < 3; i14++) {
          minval = 0.0;
          for (i = 0; i < 3; i++) {
            minval += ext_att_offset[i + 3 * i14] * measurements_.pos_ext[i];
          }

          b_err_v_b[i14] = minval + ext_pos_offset[i14];
        }

        for (i14 = 0; i14 < 3; i14++) {
          measurements_.pos_ext[i14] = b_err_v_b[i14];
        }

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
        c_measurements_[3] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] + measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        c_measurements_[6] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] - measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        c_measurements_[1] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[1] - measurements_.att_ext[2] *
          measurements_.att_ext[3]);
        c_measurements_[4] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) + measurements_.att_ext[1] *
          measurements_.att_ext[1]) - measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        c_measurements_[7] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] + measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        c_measurements_[2] = 2.0 * (measurements_.att_ext[0] *
          measurements_.att_ext[2] + measurements_.att_ext[1] *
          measurements_.att_ext[3]);
        c_measurements_[5] = 2.0 * (measurements_.att_ext[1] *
          measurements_.att_ext[2] - measurements_.att_ext[0] *
          measurements_.att_ext[3]);
        c_measurements_[8] = ((-(measurements_.att_ext[0] *
          measurements_.att_ext[0]) - measurements_.att_ext[1] *
          measurements_.att_ext[1]) + measurements_.att_ext[2] *
                              measurements_.att_ext[2]) + measurements_.att_ext
          [3] * measurements_.att_ext[3];
        for (i14 = 0; i14 < 3; i14++) {
          for (i = 0; i < 3; i++) {
            b_measurements_[i14 + 3 * i] = 0.0;
            for (k = 0; k < 3; k++) {
              b_measurements_[i14 + 3 * i] += c_measurements_[i14 + 3 * k] *
                ext_att_offset[k + 3 * i];
            }
          }
        }

        QuatFromRotJ(b_measurements_, measurements_.att_ext);
      }
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    dv20[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv20[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    dv20[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    dv20[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    dv20[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv20[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    dv20[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    dv20[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    dv20[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (i14 = 0; i14 < 3; i14++) {
      for (i = 0; i < 3; i++) {
        R_bw[i14 + 3 * i] = 0.0;
        for (k = 0; k < 3; k++) {
          R_bw[i14 + 3 * i] += R_bc[i14 + 3 * k] * dv20[k + 3 * i];
        }
      }
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_err_v_b[i14] = xt->data[i14] - ref->position[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      err_p_b[i14] = 0.0;
      for (i = 0; i < 3; i++) {
        err_p_b[i14] += R_bw[i14 + 3 * i] * b_err_v_b[i];
      }
    }

    //  position error in body frame
    for (i14 = 0; i14 < 3; i14++) {
      b_err_v_b[i14] = xt->data[7 + i14] - ref->velocity[i14];
    }

    //  velocity error in body frame
    c_ControllerGains[0] = b_ControllerGains->Ki_xy * err_p_b[0];
    c_ControllerGains[1] = b_ControllerGains->Ki_xy * err_p_b[1];
    c_ControllerGains[2] = b_ControllerGains->Ki_z * err_p_b[2];
    for (i14 = 0; i14 < 3; i14++) {
      err_v_b[i14] = 0.0;
      for (i = 0; i < 3; i++) {
        err_v_b[i14] += R_bw[i14 + 3 * i] * b_err_v_b[i];
      }

      i_control[i14] += c_ControllerGains[i14];
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

    for (i14 = 0; i14 < 3; i14++) {
      b_err_v_b[i14] = 0.0;
      for (i = 0; i < 3; i++) {
        b_err_v_b[i14] += R_bc[i + 3 * i14] * u_out_pos[i];
      }

      last_u[i14] = b_err_v_b[i14];
    }

    last_u[3] = u_out_yaw;
    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements_.gyr_duo,
              measurements_.acc_duo, numStates, u_pred);
    i14 = xt_apo->size[0];
    xt_apo->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_apo, i14, (int)sizeof(double));
    i = xt->size[0];
    for (i14 = 0; i14 < i; i14++) {
      xt_apo->data[i14] = xt->data[i14];
    }

    for (i = 0; i < 24; i++) {
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
               noiseParameters->orientation_noise,
               noiseParameters->pressure_noise, noiseParameters->ext_pos_noise,
               noiseParameters->ext_att_noise, &measurements_,
               height_offset_pressure, *b_VIOParameters, h_u_apo_out, map_out);

    //  if almost all features were lost, do a soft reset
    i = 0;
    for (k = 0; k < 24; k++) {
      if ((updateVect[k] != 0.0) && (updateVect_apo[k] == 1.0)) {
        i++;
      }
    }

    if (i < 0.0 * b_VIOParameters->num_points_per_anchor / 2.0) {
      b_ros_warn();
      for (i = 0; i < 24; i++) {
        updateVect[i] = 0.0;
      }

      if (1.0 > b_VIOParameters->num_points_per_anchor) {
        i = 0;
      } else {
        i = (int)b_VIOParameters->num_points_per_anchor;
      }

      for (i14 = 0; i14 < i; i14++) {
        tmp_data[i14] = i14;
      }

      for (i14 = 0; i14 < i; i14++) {
        updateVect[tmp_data[i14]] = 2.0;
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          R_bw[i + 3 * i14] = R_bc[i14 + 3 * i];
        }
      }

      d_measurements_[0] = ((measurements_.att_fmu[0] * measurements_.att_fmu[0]
        - measurements_.att_fmu[1] * measurements_.att_fmu[1]) -
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      d_measurements_[3] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[1] + measurements_.att_fmu[2] *
        measurements_.att_fmu[3]);
      d_measurements_[6] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[2] - measurements_.att_fmu[1] *
        measurements_.att_fmu[3]);
      d_measurements_[1] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[1] - measurements_.att_fmu[2] *
        measurements_.att_fmu[3]);
      d_measurements_[4] = ((-(measurements_.att_fmu[0] * measurements_.att_fmu
        [0]) + measurements_.att_fmu[1] * measurements_.att_fmu[1]) -
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      d_measurements_[7] = 2.0 * (measurements_.att_fmu[1] *
        measurements_.att_fmu[2] + measurements_.att_fmu[0] *
        measurements_.att_fmu[3]);
      d_measurements_[2] = 2.0 * (measurements_.att_fmu[0] *
        measurements_.att_fmu[2] + measurements_.att_fmu[1] *
        measurements_.att_fmu[3]);
      d_measurements_[5] = 2.0 * (measurements_.att_fmu[1] *
        measurements_.att_fmu[2] - measurements_.att_fmu[0] *
        measurements_.att_fmu[3]);
      d_measurements_[8] = ((-(measurements_.att_fmu[0] * measurements_.att_fmu
        [0]) - measurements_.att_fmu[1] * measurements_.att_fmu[1]) +
                            measurements_.att_fmu[2] * measurements_.att_fmu[2])
        + measurements_.att_fmu[3] * measurements_.att_fmu[3];
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          c_measurements_[i14 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            c_measurements_[i14 + 3 * i] += R_bw[i14 + 3 * k] *
              d_measurements_[k + 3 * i];
          }
        }
      }

      QuatFromRotJ(c_measurements_, u_pred);
      i14 = r5->size[0];
      r5->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
      emxEnsureCapacity((emxArray__common *)r5, i14, (int)sizeof(double));
      r5->data[0] = 0.0;
      r5->data[1] = 0.0;
      r5->data[2] = 0.0;
      r5->data[3] = 0.0;
      r5->data[4] = 0.0;
      r5->data[5] = 0.0;
      r5->data[6] = 1.0;
      i = (int)b_VIOParameters->num_points_per_anchor;
      for (i14 = 0; i14 < i; i14++) {
        r5->data[i14 + 7] = 0.0;
      }

      repmat(r5, b_VIOParameters->num_anchors, xt_apo);
      i14 = r9->size[0];
      r9->size[0] = 13 + xt_apo->size[0];
      emxEnsureCapacity((emxArray__common *)r9, i14, (int)sizeof(double));
      for (i14 = 0; i14 < 3; i14++) {
        r9->data[i14] = xt->data[i14];
      }

      for (i14 = 0; i14 < 4; i14++) {
        r9->data[i14 + 3] = u_pred[i14];
      }

      r9->data[7] = 0.0;
      r9->data[8] = 0.0;
      r9->data[9] = 0.0;
      r9->data[10] = 0.0;
      r9->data[11] = 0.0;
      r9->data[12] = 0.0;
      i = xt_apo->size[0];
      for (i14 = 0; i14 < i; i14++) {
        r9->data[i14 + 13] = xt_apo->data[i14];
      }

      i14 = xt->size[0];
      xt->size[0] = r9->size[0];
      emxEnsureCapacity((emxArray__common *)xt, i14, (int)sizeof(double));
      i = r9->size[0];
      for (i14 = 0; i14 < i; i14++) {
        xt->data[i14] = r9->data[i14];
      }

      //  initial real vector
      minval = b_VIOParameters->num_anchors * (6.0 +
        b_VIOParameters->num_points_per_anchor);
      i14 = r3->size[0] * r3->size[1];
      r3->size[0] = (int)numStates;
      r3->size[1] = (int)numStates;
      emxEnsureCapacity((emxArray__common *)r3, i14, (int)sizeof(double));
      i = (int)numStates * (int)numStates;
      for (i14 = 0; i14 < i; i14++) {
        r3->data[i14] = 0.0;
      }

      i14 = r4->size[0] * r4->size[1];
      r4->size[0] = (int)minval;
      r4->size[1] = (int)minval;
      emxEnsureCapacity((emxArray__common *)r4, i14, (int)sizeof(double));
      i = (int)minval * (int)minval;
      for (i14 = 0; i14 < i; i14++) {
        r4->data[i14] = 0.0;
      }

      blkdiag(r3, r4, P);

      //  initial error state covariance
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          P->data[i + P->size[0] * i14] = 0.0;
        }
      }

      //  position
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i14)) + 3] = 0.0;
        }
      }

      //  orientation
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (6 + i14)) + 6] = 0.0;
        }
      }

      //  velocity
      for (i14 = 0; i14 < 3; i14++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (9 + i14)) + 9] = b_y[i + 3 * i14];
        }
      }

      //  gyro bias
      for (i = 0; i < 4; i++) {
        u_out[i] = 0.0;
      }
    } else {
      i14 = xt->size[0];
      xt->size[0] = xt_apo->size[0];
      emxEnsureCapacity((emxArray__common *)xt, i14, (int)sizeof(double));
      i = xt_apo->size[0];
      for (i14 = 0; i14 < i; i14++) {
        xt->data[i14] = xt_apo->data[i14];
      }

      memcpy(&updateVect[0], &updateVect_apo[0], 24U * sizeof(double));
    }

    i14 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i14, (int)sizeof(double));
    i = xt->size[0];
    for (i14 = 0; i14 < i; i14++) {
      xt_out->data[i14] = xt->data[i14];
    }

    i14 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i14, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i14 = 0; i14 < i; i14++) {
      P_apo_out->data[i14] = P->data[i14];
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
