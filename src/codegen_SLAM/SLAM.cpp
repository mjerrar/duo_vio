//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 26-Aug-2015 16:12:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "QuatFromRotJ.h"
#include "SLAM_pred.h"
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
  int outsize_idx_0;
  emxArray_real_T *b;
  emxArray_real_T *a;
  int k;
  int loop_ub;
  int outsize_idx_1;
  int ibcol;
  double dv24[9];
  double b_IMU_measurements[9];
  double dv25[4];
  double d5;
  static const double y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  double u_out_x;
  double u_out_y;
  double u_out_z;
  double R_bw[9];
  double u_out_yaw;
  double b_u_out_x[3];
  double b_R_bw[3];

  //  for coder
  for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
    u_out[outsize_idx_0] = 0.0;
  }

  //  for coder
  b_emxInit_real_T(&b, 1);
  b_emxInit_real_T(&a, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    k = a->size[0];
    a->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)a, k, (int)sizeof(double));
    a->data[0] = 0.0;
    a->data[1] = 0.0;
    a->data[2] = 0.0;
    a->data[3] = 0.0;
    a->data[4] = 0.0;
    a->data[5] = 0.0;
    a->data[6] = 1.0;
    loop_ub = (int)b_VIOParameters->num_points_per_anchor;
    for (k = 0; k < loop_ub; k++) {
      a->data[k + 7] = 0.0;
    }

    outsize_idx_0 = a->size[0] * (int)b_VIOParameters->num_anchors;
    k = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      outsize_idx_0 = a->size[0];
      for (outsize_idx_1 = 1; outsize_idx_1 <= (int)b_VIOParameters->num_anchors;
           outsize_idx_1++) {
        ibcol = (outsize_idx_1 - 1) * outsize_idx_0;
        for (k = 0; k + 1 <= outsize_idx_0; k++) {
          b->data[ibcol + k] = a->data[k];
        }
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
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        dv24[k + 3 * ibcol] = 0.0;
        for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
          dv24[k + 3 * ibcol] += R_bc[outsize_idx_0 + 3 * k] *
            b_IMU_measurements[outsize_idx_0 + 3 * ibcol];
        }
      }
    }

    QuatFromRotJ(dv24, dv25);
    k = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, k, (int)sizeof(double));
    for (k = 0; k < 3; k++) {
      xt->data[k] = ref[k];
    }

    for (k = 0; k < 4; k++) {
      xt->data[k + 3] = dv25[k];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (k = 0; k < 3; k++) {
      xt->data[k + 10] = 0.0 * IMU_measurements[k];
    }

    loop_ub = b->size[0];
    for (k = 0; k < loop_ub; k++) {
      xt->data[k + 13] = b->data[k];
    }

    //  initial real vector
    d5 = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    loop_ub = (int)(b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor));
    outsize_idx_0 = (int)numStates + (int)d5;
    outsize_idx_1 = (int)numStates + (int)d5;
    k = P->size[0] * P->size[1];
    P->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)P, k, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    P->size[1] = outsize_idx_1;
    emxEnsureCapacity((emxArray__common *)P, k, (int)sizeof(double));
    outsize_idx_0 *= outsize_idx_1;
    for (k = 0; k < outsize_idx_0; k++) {
      P->data[k] = 0.0;
    }

    if ((int)numStates > 0) {
      outsize_idx_0 = (int)numStates;
      for (k = 0; k < outsize_idx_0; k++) {
        outsize_idx_1 = (int)numStates;
        for (ibcol = 0; ibcol < outsize_idx_1; ibcol++) {
          P->data[ibcol + P->size[0] * k] = 0.0;
        }
      }
    }

    if ((int)d5 > 0) {
      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        k = 1;
      } else {
        k = (int)numStates + 1;
      }

      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        ibcol = 1;
      } else {
        ibcol = (int)numStates + 1;
      }

      for (outsize_idx_0 = 0; outsize_idx_0 < loop_ub; outsize_idx_0++) {
        for (outsize_idx_1 = 0; outsize_idx_1 < loop_ub; outsize_idx_1++) {
          P->data[((k + outsize_idx_1) + P->size[0] * ((ibcol + outsize_idx_0) -
                    1)) - 1] = 0.0;
        }
      }
    }

    //  initial error state covariance
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        P->data[ibcol + P->size[0] * k] = 0.0;
      }
    }

    //  position
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        P->data[(ibcol + P->size[0] * (3 + k)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        P->data[(ibcol + P->size[0] * (6 + k)) + 6] = 0.0;
      }
    }

    //  velocity
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        P->data[(ibcol + P->size[0] * (9 + k)) + 9] = y[ibcol + 3 * k];
      }
    }

    //  gyro bias
    for (outsize_idx_0 = 0; outsize_idx_0 < 16; outsize_idx_0++) {
      updateVect[outsize_idx_0] = 0.0;
    }

    SLAM_updIT(P, xt, b_cameraParameters->CameraParameters1.RadialDistortion,
               b_cameraParameters->CameraParameters1.FocalLength,
               b_cameraParameters->CameraParameters1.PrincipalPoint,
               b_cameraParameters->CameraParameters2.RadialDistortion,
               b_cameraParameters->CameraParameters2.FocalLength,
               b_cameraParameters->CameraParameters2.PrincipalPoint,
               b_cameraParameters->r_lr, b_cameraParameters->R_lr,
               b_cameraParameters->R_rl, updateVect, z_all_l, z_all_r,
               noiseParameters->image_noise, noiseParameters->sigmaInit,
               noiseParameters->orientation_noise,
               noiseParameters->pressure_noise, IMU_measurements, (1.0 -
                rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
               145366.45, *b_VIOParameters, h_u_apo_out, map_out);
    k = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, k, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (k = 0; k < loop_ub; k++) {
      xt_out->data[k] = xt->data[k];
    }

    k = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, k, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (k = 0; k < loop_ub; k++) {
      P_apo_out->data[k] = P->data[k];
    }

    for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
      last_u[outsize_idx_0] = 0.0;
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
    dv24[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv24[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    dv24[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    dv24[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    dv24[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv24[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    dv24[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    dv24[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    dv24[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (k = 0; k < 3; k++) {
      for (ibcol = 0; ibcol < 3; ibcol++) {
        R_bw[k + 3 * ibcol] = 0.0;
        for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
          R_bw[k + 3 * ibcol] += R_bc[k + 3 * outsize_idx_0] *
            dv24[outsize_idx_0 + 3 * ibcol];
        }
      }
    }

    u_out_yaw = -b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_bw[3], R_bw[0]) -
      ref[3]);
    b_u_out_x[0] = u_out_x;
    b_u_out_x[1] = u_out_y;
    b_u_out_x[2] = u_out_z;
    for (k = 0; k < 3; k++) {
      b_R_bw[k] = 0.0;
      for (ibcol = 0; ibcol < 3; ibcol++) {
        b_R_bw[k] += R_bw[k + 3 * ibcol] * b_u_out_x[ibcol];
      }
    }

    for (k = 0; k < 3; k++) {
      u_out[k] = b_R_bw[k];
    }

    u_out[3] = u_out_yaw;

    //  fprintf('position error (%.3f, %.3f, %.3f, %.3f), control: (%.3f, %.3f, %.3f, %.3f)\n', xt(1) - ref(1), xt(2) - ref(2), xt(3) - ref(3), yaw - ref(4), u_out_x, u_out_y, u_out_z, u_out_yaw); 
    if (b_VIOParameters->use_controller_to_predict) {
    } else {
      for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
        last_u[outsize_idx_0] = 0.0;
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
               noiseParameters->image_noise, noiseParameters->sigmaInit,
               noiseParameters->orientation_noise,
               noiseParameters->pressure_noise, IMU_measurements, 0.0,
               *b_VIOParameters, h_u_apo_out, map_out);
    k = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, k, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (k = 0; k < loop_ub; k++) {
      xt_out->data[k] = xt->data[k];
    }

    k = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, k, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (k = 0; k < loop_ub; k++) {
      P_apo_out->data[k] = P->data[k];
    }

    last_u[0] = u_out_x;
    last_u[1] = u_out_y;
    last_u[2] = u_out_z;
    last_u[3] = u_out_yaw;

    // % output asserts for coder
  }

  emxFree_real_T(&a);
  emxFree_real_T(&b);
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
