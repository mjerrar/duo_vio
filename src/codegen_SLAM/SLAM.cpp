//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 17:43:12
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "repmat.h"
#include "QuatFromRotJ.h"
#include "SLAM_pred.h"
#include "fprintf.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double last_u[4];

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
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double IMU_measurements[23]
//                const double ref[4]
//                const VIOParameters *b_VIOParameters
//                const stereoParameters *b_cameraParameters
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
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double IMU_measurements[23], const double ref[4],
          const VIOParameters *b_VIOParameters, const stereoParameters
          *b_cameraParameters, const NoiseParameters *noiseParameters, const
          ControllerGains *b_ControllerGains, boolean_T resetFlag,
          emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
          *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int i;
  double d5;
  signed char i8;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  double R_bw[9];
  int i9;
  int i10;
  double b_R_bw[9];
  double b_IMU_measurements[9];
  double dv24[4];
  int loop_ub;
  int unnamed_idx_1;
  static const double y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  double u_out_x;
  double u_out_y;
  double u_out_z;
  double u_out_yaw;
  double b_u_out_x[3];
  double c_R_bw[3];

  //  for coder
  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  b_fprintf(noiseParameters->image_noise[0], noiseParameters->image_noise[1]);
  d_fprintf(noiseParameters->orientation_noise);
  f_fprintf(noiseParameters->pressure_noise);
  h_fprintf(noiseParameters->sigmaInit);
  j_fprintf(b_VIOParameters->max_ekf_iterations);
  l_fprintf(b_VIOParameters->num_anchors);
  d5 = rt_roundd_snf(b_VIOParameters->num_points_per_anchor);
  if (d5 < 128.0) {
    if (d5 >= -128.0) {
      i8 = (signed char)d5;
    } else {
      i8 = MIN_int8_T;
    }
  } else if (d5 >= 128.0) {
    i8 = MAX_int8_T;
  } else {
    i8 = 0;
  }

  n_fprintf(i8);
  p_fprintf((signed char)b_VIOParameters->use_controller_to_predict);
  r_fprintf((signed char)b_VIOParameters->use_magnetometer);
  t_fprintf((signed char)b_VIOParameters->use_orientation);
  v_fprintf((signed char)b_VIOParameters->use_pressure);
  x_fprintf(b_ControllerGains->Kd_xy);
  ab_fprintf(b_ControllerGains->Kd_z);
  cb_fprintf(b_ControllerGains->Kp_xy);
  eb_fprintf(b_ControllerGains->Kp_yaw);
  gb_fprintf(b_ControllerGains->Kp_z);
  b_emxInit_real_T(&r4, 1);
  b_emxInit_real_T(&r5, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        R_bw[i10 + 3 * i9] = R_bc[i9 + 3 * i10];
      }
    }

    b_IMU_measurements[0] = ((IMU_measurements[19] * IMU_measurements[19] -
      IMU_measurements[20] * IMU_measurements[20]) - IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    b_IMU_measurements[3] = 2.0 * (IMU_measurements[19] * IMU_measurements[20] +
      IMU_measurements[21] * IMU_measurements[22]);
    b_IMU_measurements[6] = 2.0 * (IMU_measurements[19] * IMU_measurements[21] -
      IMU_measurements[20] * IMU_measurements[22]);
    b_IMU_measurements[1] = 2.0 * (IMU_measurements[19] * IMU_measurements[20] -
      IMU_measurements[21] * IMU_measurements[22]);
    b_IMU_measurements[4] = ((-(IMU_measurements[19] * IMU_measurements[19]) +
      IMU_measurements[20] * IMU_measurements[20]) - IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    b_IMU_measurements[7] = 2.0 * (IMU_measurements[20] * IMU_measurements[21] +
      IMU_measurements[19] * IMU_measurements[22]);
    b_IMU_measurements[2] = 2.0 * (IMU_measurements[19] * IMU_measurements[21] +
      IMU_measurements[20] * IMU_measurements[22]);
    b_IMU_measurements[5] = 2.0 * (IMU_measurements[20] * IMU_measurements[21] -
      IMU_measurements[19] * IMU_measurements[22]);
    b_IMU_measurements[8] = ((-(IMU_measurements[19] * IMU_measurements[19]) -
      IMU_measurements[20] * IMU_measurements[20]) + IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        b_R_bw[i9 + 3 * i10] = 0.0;
        for (i = 0; i < 3; i++) {
          b_R_bw[i9 + 3 * i10] += R_bw[i9 + 3 * i] * b_IMU_measurements[i + 3 *
            i10];
        }
      }
    }

    QuatFromRotJ(b_R_bw, dv24);
    i9 = r5->size[0];
    r5->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)r5, i9, (int)sizeof(double));
    r5->data[0] = 0.0;
    r5->data[1] = 0.0;
    r5->data[2] = 0.0;
    r5->data[3] = 0.0;
    r5->data[4] = 0.0;
    r5->data[5] = 0.0;
    r5->data[6] = 1.0;
    loop_ub = (int)b_VIOParameters->num_points_per_anchor;
    for (i9 = 0; i9 < loop_ub; i9++) {
      r5->data[i9 + 7] = 0.0;
    }

    repmat(r5, b_VIOParameters->num_anchors, r4);
    i9 = xt->size[0];
    xt->size[0] = 13 + r4->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i9, (int)sizeof(double));
    for (i9 = 0; i9 < 3; i9++) {
      xt->data[i9] = ref[i9];
    }

    for (i9 = 0; i9 < 4; i9++) {
      xt->data[i9 + 3] = dv24[i9];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (i9 = 0; i9 < 3; i9++) {
      xt->data[i9 + 10] = 0.0 * IMU_measurements[i9];
    }

    loop_ub = r4->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      xt->data[i9 + 13] = r4->data[i9];
    }

    //  initial real vector
    d5 = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    loop_ub = (int)(b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor));
    i = (int)numStates + (int)d5;
    unnamed_idx_1 = (int)numStates + (int)d5;
    i9 = P->size[0] * P->size[1];
    P->size[0] = i;
    emxEnsureCapacity((emxArray__common *)P, i9, (int)sizeof(double));
    i9 = P->size[0] * P->size[1];
    P->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)P, i9, (int)sizeof(double));
    i *= unnamed_idx_1;
    for (i9 = 0; i9 < i; i9++) {
      P->data[i9] = 0.0;
    }

    if ((int)numStates > 0) {
      i = (int)numStates;
      for (i9 = 0; i9 < i; i9++) {
        unnamed_idx_1 = (int)numStates;
        for (i10 = 0; i10 < unnamed_idx_1; i10++) {
          P->data[i10 + P->size[0] * i9] = 0.0;
        }
      }
    }

    if ((int)d5 > 0) {
      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        i9 = 1;
      } else {
        i9 = (int)numStates + 1;
      }

      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        i10 = 1;
      } else {
        i10 = (int)numStates + 1;
      }

      for (i = 0; i < loop_ub; i++) {
        for (unnamed_idx_1 = 0; unnamed_idx_1 < loop_ub; unnamed_idx_1++) {
          P->data[((i9 + unnamed_idx_1) + P->size[0] * ((i10 + i) - 1)) - 1] =
            0.0;
        }
      }
    }

    //  initial error state covariance
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        P->data[i10 + P->size[0] * i9] = 0.0;
      }
    }

    //  position
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        P->data[(i10 + P->size[0] * (3 + i9)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        P->data[(i10 + P->size[0] * (6 + i9)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        P->data[(i10 + P->size[0] * (9 + i9)) + 9] = y[i10 + 3 * i9];
      }
    }

    //  gyro bias
    for (i = 0; i < 16; i++) {
      updateVect[i] = 0.0;
    }

    SLAM_updIT(P, xt, b_cameraParameters->CameraParameters1.RadialDistortion,
               b_cameraParameters->CameraParameters1.FocalLength,
               b_cameraParameters->CameraParameters1.PrincipalPoint,
               b_cameraParameters->CameraParameters2.RadialDistortion,
               b_cameraParameters->CameraParameters2.FocalLength,
               b_cameraParameters->CameraParameters2.PrincipalPoint,
               b_cameraParameters->r_lr, b_cameraParameters->R_lr,
               b_cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters->image_noise, noiseParameters->orientation_noise,
               noiseParameters->pressure_noise, IMU_measurements, (1.0 -
                rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
               145366.45, *b_VIOParameters, h_u_apo_out, map_out);
    i9 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i9, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      xt_out->data[i9] = xt->data[i9];
    }

    i9 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i9, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (i9 = 0; i9 < loop_ub; i9++) {
      P_apo_out->data[i9] = P->data[i9];
    }

    for (i = 0; i < 4; i++) {
      last_u[i] = 0.0;
    }

    //  the last control outputs (in camera frame)
  } else {
    u_out_x = -(b_ControllerGains->Kp_xy * (xt->data[0] - ref[0]) +
                b_ControllerGains->Kd_xy * xt->data[7]);

    //  control commands in world frame
    u_out_y = -(b_ControllerGains->Kp_xy * (xt->data[1] - ref[1]) +
                b_ControllerGains->Kd_xy * xt->data[8]);

    //  control commands in world frame
    u_out_z = -(b_ControllerGains->Kp_z * (xt->data[2] - ref[2]) +
                b_ControllerGains->Kd_z * xt->data[9]);

    //  control commands in world frame
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_R_bw[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    b_R_bw[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    b_R_bw[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    b_R_bw[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    b_R_bw[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    b_R_bw[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    b_R_bw[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    b_R_bw[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    b_R_bw[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
                 xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (i9 = 0; i9 < 3; i9++) {
      for (i10 = 0; i10 < 3; i10++) {
        R_bw[i9 + 3 * i10] = 0.0;
        for (i = 0; i < 3; i++) {
          R_bw[i9 + 3 * i10] += R_bc[i9 + 3 * i] * b_R_bw[i + 3 * i10];
        }
      }
    }

    u_out_yaw = -b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_bw[3], R_bw[0]) -
      ref[3]);
    b_u_out_x[0] = u_out_x;
    b_u_out_x[1] = u_out_y;
    b_u_out_x[2] = u_out_z;
    for (i9 = 0; i9 < 3; i9++) {
      c_R_bw[i9] = 0.0;
      for (i10 = 0; i10 < 3; i10++) {
        c_R_bw[i9] += R_bw[i9 + 3 * i10] * b_u_out_x[i10];
      }
    }

    for (i9 = 0; i9 < 3; i9++) {
      u_out[i9] = c_R_bw[i9];
    }

    u_out[3] = u_out_yaw;

    //  fprintf('position error (%.3f, %.3f, %.3f, %.3f), control: (%.3f, %.3f, %.3f, %.3f)\n', xt(1) - ref(1), xt(2) - ref(2), xt(3) - ref(3), yaw - ref(4), u_out_x, u_out_y, u_out_z, u_out_yaw); 
    if (b_VIOParameters->use_controller_to_predict) {
    } else {
      for (i = 0; i < 4; i++) {
        last_u[i] = 0.0;
      }
    }

    SLAM_pred(P, xt, dt, noiseParameters->process_noise, IMU_measurements,
              numStates, last_u);
    SLAM_updIT(P, xt, b_cameraParameters->CameraParameters1.RadialDistortion,
               b_cameraParameters->CameraParameters1.FocalLength,
               b_cameraParameters->CameraParameters1.PrincipalPoint,
               b_cameraParameters->CameraParameters2.RadialDistortion,
               b_cameraParameters->CameraParameters2.FocalLength,
               b_cameraParameters->CameraParameters2.PrincipalPoint,
               b_cameraParameters->r_lr, b_cameraParameters->R_lr,
               b_cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters->image_noise, noiseParameters->orientation_noise,
               noiseParameters->pressure_noise, IMU_measurements, 0.0,
               *b_VIOParameters, h_u_apo_out, map_out);
    i9 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i9, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      xt_out->data[i9] = xt->data[i9];
    }

    i9 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i9, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (i9 = 0; i9 < loop_ub; i9++) {
      P_apo_out->data[i9] = P->data[i9];
    }

    last_u[0] = u_out_x;
    last_u[1] = u_out_y;
    last_u[2] = u_out_z;
    last_u[3] = u_out_yaw;

    // % output asserts for coder
  }

  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
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
