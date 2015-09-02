//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Sep-2015 21:38:45
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "blkdiag.h"
#include "QuatFromRotJ.h"
#include "SLAM_updIT.h"
#include "Att_upd.h"
#include "quatPlusThetaJ.h"
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
  int outsize_idx_0;
  double numTrackFeatures;
  VIOMeasurements measurements_;
  emxArray_real_T *xt_apo;
  emxArray_real_T *b;
  emxArray_real_T *r3;
  emxArray_real_T *r4;
  int k;
  char cv8[38];
  int ibcol;
  static const char cv9[38] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i',
    'n', 'g', ' ', 'o', 'r', 'i', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', ' ',
    'w', 'i', 't', 'h', ' ', 'D', 'U', 'O', ' ', 'I', 'M', 'U', '\x00' };

  static const double dv7[9] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0 };

  static const double dv8[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  int itilerow;
  double dq[4];
  double d4;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double err_p_b[3];
  static const double gyro_offset[3] = { 0.534299594891486, -0.26782462470649,
    0.000842792650371513 };

  double q;
  double R_cw[9];
  double b_measurements_[9];
  double c;
  double b_err_p_b[3];
  double Phi[9];
  double dv9[16];
  double b_dq[4];
  double c_measurements_[9];
  double b_Phi[9];
  double a[9];
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char b_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double yaw;
  static const signed char iv4[3] = { 0, 0, 1 };

  double c_ControllerGains[3];
  double err_v_b[3];
  double minval;
  double d_ControllerGains[3];
  double e_ControllerGains[3];
  for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
    u_out[outsize_idx_0] = 0.0;
  }

  //  for coder
  numTrackFeatures = b_VIOParameters->num_anchors *
    b_VIOParameters->num_points_per_anchor;
  measurements_ = *measurements;

  //  copy for coders
  b_emxInit_real_T(&xt_apo, 1);
  b_emxInit_real_T(&b, 1);
  emxInit_real_T(&r3, 2);
  emxInit_real_T(&r4, 2);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;
    ext_pose_offset_initialized = false;
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      ext_pos_offset[outsize_idx_0] = 0.0;
    }

    memset(&ext_att_offset[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      ext_att_offset[k + 3 * k] = 1.0;
    }

    for (outsize_idx_0 = 0; outsize_idx_0 < 32; outsize_idx_0++) {
      updateVect[outsize_idx_0] = 0.0;
    }

    // #coder
    // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
    for (ibcol = 0; ibcol < 38; ibcol++) {
      cv8[ibcol] = cv9[ibcol];
    }

    ROS_INFO(cv8);

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
    QuatFromRotJ(dv7, x_att);
    memcpy(&P_att[0], &dv8[0], 9U * sizeof(double));
    initializing_attitude = 1.0;
    ibcol = xt_apo->size[0];
    xt_apo->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)xt_apo, ibcol, (int)sizeof(double));
    xt_apo->data[0] = 0.0;
    xt_apo->data[1] = 0.0;
    xt_apo->data[2] = 0.0;
    xt_apo->data[3] = 0.0;
    xt_apo->data[4] = 0.0;
    xt_apo->data[5] = 0.0;
    xt_apo->data[6] = 1.0;
    outsize_idx_0 = (int)b_VIOParameters->num_points_per_anchor;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_apo->data[ibcol + 7] = 0.0;
    }

    outsize_idx_0 = xt_apo->size[0] * (int)b_VIOParameters->num_anchors;
    ibcol = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, ibcol, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      outsize_idx_0 = xt_apo->size[0];
      for (itilerow = 1; itilerow <= (int)b_VIOParameters->num_anchors; itilerow
           ++) {
        ibcol = (itilerow - 1) * outsize_idx_0;
        for (k = 0; k + 1 <= outsize_idx_0; k++) {
          b->data[ibcol + k] = xt_apo->data[k];
        }
      }
    }

    QuatFromRotJ(dv7, dq);
    ibcol = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, ibcol, (int)sizeof(double));
    for (ibcol = 0; ibcol < 3; ibcol++) {
      xt->data[ibcol] = ref->position[ibcol];
    }

    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt->data[ibcol + 3] = dq[ibcol];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (ibcol = 0; ibcol < 3; ibcol++) {
      xt->data[ibcol + 10] = measurements_.gyr_duo[ibcol];
    }

    outsize_idx_0 = b->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt->data[ibcol + 13] = b->data[ibcol];
    }

    //  initial real vector
    d4 = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    ibcol = r3->size[0] * r3->size[1];
    r3->size[0] = (int)numStates;
    r3->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r3, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)numStates * (int)numStates;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r3->data[ibcol] = 0.0;
    }

    ibcol = r4->size[0] * r4->size[1];
    r4->size[0] = (int)d4;
    r4->size[1] = (int)d4;
    emxEnsureCapacity((emxArray__common *)r4, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)d4 * (int)d4;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r4->data[ibcol] = 0.0;
    }

    blkdiag(r3, r4, P);

    //  initial error state covariance
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[outsize_idx_0 + P->size[0] * ibcol] = 0.0;
      }
    }

    //  position
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + ibcol)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (6 + ibcol)) + 6] = 0.0;
      }
    }

    //  velocity
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (9 + ibcol)) + 9] =
          y[outsize_idx_0 + 3 * ibcol];
      }
    }

    //  gyro bias
    height_offset_pressure = (1.0 - rt_powd_snf(measurements_.bar_fmu / 101325.0,
      0.190284)) * 145366.45;
    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    ibcol = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)(numTrackFeatures * 4.0);
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      h_u_apo_out->data[ibcol] = 0.0;
    }

    ibcol = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = (int)numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)map_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = 3 * (int)numTrackFeatures;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      map_out->data[ibcol] = 0.0;
    }

    //  the last control outputs (in camera frame)
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      i_control[outsize_idx_0] = 0.0;
    }
  } else if (initializing_attitude > 0.0) {
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      err_p_b[outsize_idx_0] = measurements_.gyr_duo[outsize_idx_0] -
        gyro_offset[outsize_idx_0];
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
      for (ibcol = 0; ibcol < 3; ibcol++) {
        Phi[ibcol + 3 * k] = R_cw[ibcol + 3 * k] + -b_measurements_[ibcol + 3 *
          k] * dt;
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
    for (ibcol = 0; ibcol < 4; ibcol++) {
      x_att[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
        x_att[ibcol] += dv9[ibcol + (outsize_idx_0 << 2)] * b_dq[outsize_idx_0];
      }
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        b_measurements_[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          b_measurements_[ibcol + 3 * outsize_idx_0] += Phi[ibcol + 3 * itilerow]
            * P_att[itilerow + 3 * outsize_idx_0];
        }

        c_measurements_[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          c_measurements_[ibcol + 3 * outsize_idx_0] += (double)b_a[ibcol + 3 *
            itilerow] * ((double)b_b[itilerow + 3 * outsize_idx_0] * q * c);
        }
      }

      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        b_Phi[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          b_Phi[ibcol + 3 * outsize_idx_0] += b_measurements_[ibcol + 3 *
            itilerow] * Phi[outsize_idx_0 + 3 * itilerow];
        }

        a[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          a[ibcol + 3 * outsize_idx_0] += c_measurements_[ibcol + 3 * itilerow] *
            (double)b_a[itilerow + 3 * outsize_idx_0];
        }
      }
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P_att[outsize_idx_0 + 3 * ibcol] = b_Phi[outsize_idx_0 + 3 * ibcol] +
          a[outsize_idx_0 + 3 * ibcol];
      }
    }

    Att_upd(x_att, P_att, measurements_.acc_duo, noiseParameters->process_noise
            [0] * 1.0E+10, dt);
    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt->data[3 + ibcol] = x_att[ibcol];
    }

    initializing_attitude++;
    if (initializing_attitude >= 20.0) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
          P->data[(outsize_idx_0 + P->size[0] * (3 + ibcol)) + 3] =
            P_att[outsize_idx_0 + 3 * ibcol];
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
      ibcol = h_u_apo_out->size[0];
      h_u_apo_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_apo_out, ibcol, (int)sizeof
                        (double));
      outsize_idx_0 = (int)(numTrackFeatures * 4.0);
      for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
        h_u_apo_out->data[ibcol] = 0.0;
      }

      ibcol = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, ibcol, (int)sizeof(double));
      outsize_idx_0 = 3 * (int)numTrackFeatures;
      for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
        map_out->data[ibcol] = 0.0;
      }
    }

    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }
  } else {
    if (b_VIOParameters->use_ext_pose) {
      if (!ext_pose_offset_initialized) {
        for (ibcol = 0; ibcol < 3; ibcol++) {
          ext_pos_offset[ibcol] = xt->data[ibcol] - measurements_.pos_ext[ibcol];
        }

        //  in vio frame
        for (ibcol = 0; ibcol < 3; ibcol++) {
          measurements_.pos_ext[ibcol] = xt->data[ibcol];
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
        for (ibcol = 0; ibcol < 3; ibcol++) {
          for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
            ext_att_offset[ibcol + 3 * outsize_idx_0] = 0.0;
            for (itilerow = 0; itilerow < 3; itilerow++) {
              ext_att_offset[ibcol + 3 * outsize_idx_0] += c_measurements_[ibcol
                + 3 * itilerow] * b_measurements_[itilerow + 3 * outsize_idx_0];
            }
          }
        }

        for (ibcol = 0; ibcol < 4; ibcol++) {
          measurements_.att_ext[ibcol] = xt->data[3 + ibcol];
        }

        ext_pose_offset_initialized = true;
      } else {
        for (ibcol = 0; ibcol < 3; ibcol++) {
          d4 = 0.0;
          for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
            d4 += ext_att_offset[outsize_idx_0 + 3 * ibcol] *
              measurements_.pos_ext[outsize_idx_0];
          }

          b_err_p_b[ibcol] = d4 + ext_pos_offset[ibcol];
        }

        for (ibcol = 0; ibcol < 3; ibcol++) {
          measurements_.pos_ext[ibcol] = b_err_p_b[ibcol];
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
        for (ibcol = 0; ibcol < 3; ibcol++) {
          for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
            c_measurements_[ibcol + 3 * outsize_idx_0] = 0.0;
            for (itilerow = 0; itilerow < 3; itilerow++) {
              c_measurements_[ibcol + 3 * outsize_idx_0] +=
                b_measurements_[ibcol + 3 * itilerow] * ext_att_offset[itilerow
                + 3 * outsize_idx_0];
            }
          }
        }

        QuatFromRotJ(c_measurements_, measurements_.att_ext);
      }
    }

    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements_.gyr_duo,
              measurements_.acc_duo, numStates);
    ibcol = xt_apo->size[0];
    xt_apo->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_apo, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_apo->data[ibcol] = xt->data[ibcol];
    }

    SLAM_updIT(P, xt_apo, cameraParameters->CameraParameters1.RadialDistortion,
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

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    R_cw[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    R_cw[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    R_cw[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    R_cw[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    yaw = rt_atan2d_snf(R_cw[3], R_cw[0]);
    R_cw[0] = cos(yaw);
    R_cw[3] = sin(yaw);
    R_cw[6] = 0.0;
    R_cw[1] = sin(yaw);
    R_cw[4] = cos(yaw);
    R_cw[7] = 0.0;
    for (ibcol = 0; ibcol < 3; ibcol++) {
      R_cw[2 + 3 * ibcol] = iv4[ibcol];
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_err_p_b[ibcol] = xt->data[ibcol] - ref->position[ibcol];
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      err_p_b[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        err_p_b[ibcol] += R_cw[ibcol + 3 * outsize_idx_0] *
          b_err_p_b[outsize_idx_0];
      }
    }

    //  position error in body frame
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_err_p_b[ibcol] = xt->data[7 + ibcol] - ref->velocity[ibcol];
    }

    //  velocity error in body frame
    c_ControllerGains[0] = b_ControllerGains->Ki_xy * err_p_b[0];
    c_ControllerGains[1] = b_ControllerGains->Ki_xy * err_p_b[1];
    c_ControllerGains[2] = b_ControllerGains->Ki_z * err_p_b[2];
    for (ibcol = 0; ibcol < 3; ibcol++) {
      err_v_b[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        err_v_b[ibcol] += R_cw[ibcol + 3 * outsize_idx_0] *
          b_err_p_b[outsize_idx_0];
      }

      i_control[ibcol] += c_ControllerGains[ibcol];
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
    d_ControllerGains[0] = b_ControllerGains->Kp_xy * err_p_b[0];
    d_ControllerGains[1] = b_ControllerGains->Kp_xy * err_p_b[1];
    d_ControllerGains[2] = b_ControllerGains->Kp_z * err_p_b[2];
    e_ControllerGains[0] = b_ControllerGains->Kd_xy * err_v_b[0];
    e_ControllerGains[1] = b_ControllerGains->Kd_xy * err_v_b[1];
    e_ControllerGains[2] = b_ControllerGains->Kd_z * err_v_b[2];
    for (ibcol = 0; ibcol < 3; ibcol++) {
      u_out[ibcol] = -((d_ControllerGains[ibcol] + i_control[ibcol]) +
                       e_ControllerGains[ibcol]);
    }

    u_out[3] = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_cw[3], R_cw[0]) -
      ref->position[3]);

    //  fprintf('position error (%.3f, %.3f, %.3f, %.3f), control: (%.3f, %.3f, %.3f, %.3f)\n', xt(1) - ref(1), xt(2) - ref(2), xt(3) - ref(3), yaw - ref(4), u_out_x, u_out_y, u_out_z, u_out_yaw); 
    //  if almost all features were lost, do a soft reset
    ibcol = xt->size[0];
    xt->size[0] = xt_apo->size[0];
    emxEnsureCapacity((emxArray__common *)xt, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt_apo->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt->data[ibcol] = xt_apo->data[ibcol];
    }

    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    // % output asserts for coder
  }

  emxFree_real_T(&r4);
  emxFree_real_T(&r3);
  emxFree_real_T(&b);
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
