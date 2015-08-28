//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 15:47:41
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
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double last_u[4];
static boolean_T ext_pose_offset_initialized;
static double ext_pos_offset[3];
static double ext_att_offset[9];

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
//                VIOMeasurements *measurements
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
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, VIOMeasurements *measurements, const ReferenceCommand
          *ref, const VIOParameters *b_VIOParameters, const StereoParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          ControllerGains *b_ControllerGains, boolean_T resetFlag,
          emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
          *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int k;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  double R_bw[9];
  int i6;
  double b_measurements[9];
  double c_measurements[9];
  int i7;
  double dv24[4];
  int loop_ub;
  double d5;
  int unnamed_idx_1;
  static const double y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  double b_R_bw[3];
  double u_out_x;
  double u_out_y;
  double u_out_z;
  double dv25[9];
  double u_out_yaw;
  double b_u_out_x[3];

  //  for coder
  for (k = 0; k < 4; k++) {
    u_out[k] = 0.0;
  }

  //  for coder
  b_emxInit_real_T(&r4, 1);
  b_emxInit_real_T(&r5, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;
    ext_pose_offset_initialized = false;
    for (k = 0; k < 3; k++) {
      ext_pos_offset[k] = 0.0;
    }

    memset(&ext_att_offset[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      ext_att_offset[k + 3 * k] = 1.0;

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      for (i6 = 0; i6 < 3; i6++) {
        R_bw[i6 + 3 * k] = R_bc[k + 3 * i6];
      }
    }

    //      R_iw_init = RotFromQuatJ([0 1 0 0]');
    c_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0] -
                          measurements->att_fmu[1] * measurements->att_fmu[1]) -
                         measurements->att_fmu[2] * measurements->att_fmu[2]) +
      measurements->att_fmu[3] * measurements->att_fmu[3];
    c_measurements[3] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] + measurements->att_fmu[2] * measurements->att_fmu[3]);
    c_measurements[6] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] - measurements->att_fmu[1] * measurements->att_fmu[3]);
    c_measurements[1] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] - measurements->att_fmu[2] * measurements->att_fmu[3]);
    c_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          + measurements->att_fmu[1] * measurements->att_fmu[1])
                         - measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    c_measurements[7] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] + measurements->att_fmu[0] * measurements->att_fmu[3]);
    c_measurements[2] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] + measurements->att_fmu[1] * measurements->att_fmu[3]);
    c_measurements[5] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] - measurements->att_fmu[0] * measurements->att_fmu[3]);
    c_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          - measurements->att_fmu[1] * measurements->att_fmu[1])
                         + measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        b_measurements[i6 + 3 * i7] = 0.0;
        for (k = 0; k < 3; k++) {
          b_measurements[i6 + 3 * i7] += R_bw[i6 + 3 * k] * c_measurements[k + 3
            * i7];
        }
      }
    }

    QuatFromRotJ(b_measurements, dv24);
    i6 = r5->size[0];
    r5->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)r5, i6, (int)sizeof(double));
    r5->data[0] = 0.0;
    r5->data[1] = 0.0;
    r5->data[2] = 0.0;
    r5->data[3] = 0.0;
    r5->data[4] = 0.0;
    r5->data[5] = 0.0;
    r5->data[6] = 1.0;
    loop_ub = (int)b_VIOParameters->num_points_per_anchor;
    for (i6 = 0; i6 < loop_ub; i6++) {
      r5->data[i6 + 7] = 0.0;
    }

    repmat(r5, b_VIOParameters->num_anchors, r4);
    i6 = xt->size[0];
    xt->size[0] = 13 + r4->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i6, (int)sizeof(double));
    for (i6 = 0; i6 < 3; i6++) {
      xt->data[i6] = ref->position[i6];
    }

    for (i6 = 0; i6 < 4; i6++) {
      xt->data[i6 + 3] = dv24[i6];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    loop_ub = r4->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      xt->data[i6 + 13] = r4->data[i6];
    }

    //  initial real vector
    d5 = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    loop_ub = (int)(b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor));
    k = (int)numStates + (int)d5;
    unnamed_idx_1 = (int)numStates + (int)d5;
    i6 = P->size[0] * P->size[1];
    P->size[0] = k;
    emxEnsureCapacity((emxArray__common *)P, i6, (int)sizeof(double));
    i6 = P->size[0] * P->size[1];
    P->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)P, i6, (int)sizeof(double));
    k *= unnamed_idx_1;
    for (i6 = 0; i6 < k; i6++) {
      P->data[i6] = 0.0;
    }

    if ((int)numStates > 0) {
      k = (int)numStates;
      for (i6 = 0; i6 < k; i6++) {
        unnamed_idx_1 = (int)numStates;
        for (i7 = 0; i7 < unnamed_idx_1; i7++) {
          P->data[i7 + P->size[0] * i6] = 0.0;
        }
      }
    }

    if ((int)d5 > 0) {
      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        i6 = 1;
      } else {
        i6 = (int)numStates + 1;
      }

      if ((int)numStates + 1 > (int)numStates + (int)d5) {
        i7 = 1;
      } else {
        i7 = (int)numStates + 1;
      }

      for (k = 0; k < loop_ub; k++) {
        for (unnamed_idx_1 = 0; unnamed_idx_1 < loop_ub; unnamed_idx_1++) {
          P->data[((i6 + unnamed_idx_1) + P->size[0] * ((i7 + k) - 1)) - 1] =
            0.0;
        }
      }
    }

    //  initial error state covariance
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        P->data[i7 + P->size[0] * i6] = 0.0;
      }
    }

    //  position
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        P->data[(i7 + P->size[0] * (3 + i6)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        P->data[(i7 + P->size[0] * (6 + i6)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        P->data[(i7 + P->size[0] * (9 + i6)) + 9] = y[i7 + 3 * i6];
      }
    }

    //  gyro bias
    for (k = 0; k < 16; k++) {
      updateVect[k] = 0.0;
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
               noiseParameters->ext_att_noise, measurements, (1.0 - rt_powd_snf
                (measurements->bar_fmu / 101325.0, 0.190284)) * 145366.45,
               *b_VIOParameters, h_u_apo_out, map_out);
    i6 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i6, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      xt_out->data[i6] = xt->data[i6];
    }

    i6 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i6, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      P_apo_out->data[i6] = P->data[i6];
    }

    for (k = 0; k < 4; k++) {
      last_u[k] = 0.0;
    }

    //  the last control outputs (in camera frame)
  } else {
    if (b_VIOParameters->use_ext_pose) {
      if (!ext_pose_offset_initialized) {
        for (i6 = 0; i6 < 3; i6++) {
          ext_pos_offset[i6] = xt->data[i6] - measurements->pos_ext[i6];
        }

        //  in vio frame
        for (i6 = 0; i6 < 3; i6++) {
          measurements->pos_ext[i6] = xt->data[i6];
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
        c_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0]
                              - measurements->att_ext[1] * measurements->
                              att_ext[1]) - measurements->att_ext[2] *
                             measurements->att_ext[2]) + measurements->att_ext[3]
          * measurements->att_ext[3];
        c_measurements[1] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[1] + measurements->att_ext[2] *
          measurements->att_ext[3]);
        c_measurements[2] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[2] - measurements->att_ext[1] *
          measurements->att_ext[3]);
        c_measurements[3] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[1] - measurements->att_ext[2] *
          measurements->att_ext[3]);
        c_measurements[4] = ((-(measurements->att_ext[0] * measurements->
          att_ext[0]) + measurements->att_ext[1] * measurements->att_ext[1]) -
                             measurements->att_ext[2] * measurements->att_ext[2])
          + measurements->att_ext[3] * measurements->att_ext[3];
        c_measurements[5] = 2.0 * (measurements->att_ext[1] *
          measurements->att_ext[2] + measurements->att_ext[0] *
          measurements->att_ext[3]);
        c_measurements[6] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[2] + measurements->att_ext[1] *
          measurements->att_ext[3]);
        c_measurements[7] = 2.0 * (measurements->att_ext[1] *
          measurements->att_ext[2] - measurements->att_ext[0] *
          measurements->att_ext[3]);
        c_measurements[8] = ((-(measurements->att_ext[0] * measurements->
          att_ext[0]) - measurements->att_ext[1] * measurements->att_ext[1]) +
                             measurements->att_ext[2] * measurements->att_ext[2])
          + measurements->att_ext[3] * measurements->att_ext[3];
        b_measurements[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->
                              data[4]) - xt->data[5] * xt->data[5]) + xt->data[6]
          * xt->data[6];
        b_measurements[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] *
          xt->data[6]);
        b_measurements[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] *
          xt->data[6]);
        b_measurements[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] *
          xt->data[6]);
        b_measurements[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] *
                              xt->data[4]) - xt->data[5] * xt->data[5]) +
          xt->data[6] * xt->data[6];
        b_measurements[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] *
          xt->data[6]);
        b_measurements[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] *
          xt->data[6]);
        b_measurements[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] *
          xt->data[6]);
        b_measurements[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] *
                              xt->data[4]) + xt->data[5] * xt->data[5]) +
          xt->data[6] * xt->data[6];
        for (i6 = 0; i6 < 3; i6++) {
          for (i7 = 0; i7 < 3; i7++) {
            ext_att_offset[i6 + 3 * i7] = 0.0;
            for (k = 0; k < 3; k++) {
              ext_att_offset[i6 + 3 * i7] += c_measurements[i6 + 3 * k] *
                b_measurements[k + 3 * i7];
            }
          }
        }

        for (i6 = 0; i6 < 4; i6++) {
          measurements->att_ext[i6] = xt->data[3 + i6];
        }

        ext_pose_offset_initialized = true;
      } else {
        for (i6 = 0; i6 < 3; i6++) {
          d5 = 0.0;
          for (i7 = 0; i7 < 3; i7++) {
            d5 += ext_att_offset[i7 + 3 * i6] * measurements->pos_ext[i7];
          }

          b_R_bw[i6] = d5 + ext_pos_offset[i6];
        }

        for (i6 = 0; i6 < 3; i6++) {
          measurements->pos_ext[i6] = b_R_bw[i6];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        b_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0]
                              - measurements->att_ext[1] * measurements->
                              att_ext[1]) - measurements->att_ext[2] *
                             measurements->att_ext[2]) + measurements->att_ext[3]
          * measurements->att_ext[3];
        b_measurements[3] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[1] + measurements->att_ext[2] *
          measurements->att_ext[3]);
        b_measurements[6] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[2] - measurements->att_ext[1] *
          measurements->att_ext[3]);
        b_measurements[1] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[1] - measurements->att_ext[2] *
          measurements->att_ext[3]);
        b_measurements[4] = ((-(measurements->att_ext[0] * measurements->
          att_ext[0]) + measurements->att_ext[1] * measurements->att_ext[1]) -
                             measurements->att_ext[2] * measurements->att_ext[2])
          + measurements->att_ext[3] * measurements->att_ext[3];
        b_measurements[7] = 2.0 * (measurements->att_ext[1] *
          measurements->att_ext[2] + measurements->att_ext[0] *
          measurements->att_ext[3]);
        b_measurements[2] = 2.0 * (measurements->att_ext[0] *
          measurements->att_ext[2] + measurements->att_ext[1] *
          measurements->att_ext[3]);
        b_measurements[5] = 2.0 * (measurements->att_ext[1] *
          measurements->att_ext[2] - measurements->att_ext[0] *
          measurements->att_ext[3]);
        b_measurements[8] = ((-(measurements->att_ext[0] * measurements->
          att_ext[0]) - measurements->att_ext[1] * measurements->att_ext[1]) +
                             measurements->att_ext[2] * measurements->att_ext[2])
          + measurements->att_ext[3] * measurements->att_ext[3];
        for (i6 = 0; i6 < 3; i6++) {
          for (i7 = 0; i7 < 3; i7++) {
            c_measurements[i6 + 3 * i7] = 0.0;
            for (k = 0; k < 3; k++) {
              c_measurements[i6 + 3 * i7] += b_measurements[i6 + 3 * k] *
                ext_att_offset[k + 3 * i7];
            }
          }
        }

        QuatFromRotJ(c_measurements, measurements->att_ext);
      }
    }

    u_out_x = -(b_ControllerGains->Kp_xy * (xt->data[0] - ref->position[0]) +
                b_ControllerGains->Kd_xy * (xt->data[7] - ref->velocity[0]));

    //  control commands in world frame
    u_out_y = -(b_ControllerGains->Kp_xy * (xt->data[1] - ref->position[1]) +
                b_ControllerGains->Kd_xy * (xt->data[8] - ref->velocity[1]));

    //  control commands in world frame
    u_out_z = -(b_ControllerGains->Kp_z * (xt->data[2] - ref->position[2]) +
                b_ControllerGains->Kd_z * (xt->data[9] - ref->velocity[2]));

    //  control commands in world frame
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    dv25[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv25[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    dv25[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    dv25[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    dv25[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv25[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    dv25[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    dv25[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    dv25[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        R_bw[i6 + 3 * i7] = 0.0;
        for (k = 0; k < 3; k++) {
          R_bw[i6 + 3 * i7] += R_bc[i6 + 3 * k] * dv25[k + 3 * i7];
        }
      }
    }

    u_out_yaw = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_bw[3], R_bw[0]) -
      ref->position[3]);
    b_u_out_x[0] = u_out_x;
    b_u_out_x[1] = u_out_y;
    b_u_out_x[2] = u_out_z;
    for (i6 = 0; i6 < 3; i6++) {
      b_R_bw[i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        b_R_bw[i6] += R_bw[i6 + 3 * i7] * b_u_out_x[i7];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      u_out[i6] = b_R_bw[i6];
    }

    u_out[3] = u_out_yaw;

    //  fprintf('position error (%.3f, %.3f, %.3f, %.3f), control: (%.3f, %.3f, %.3f, %.3f)\n', xt(1) - ref(1), xt(2) - ref(2), xt(3) - ref(3), yaw - ref(4), u_out_x, u_out_y, u_out_z, u_out_yaw); 
    if (b_VIOParameters->use_controller_to_predict) {
    } else {
      for (k = 0; k < 4; k++) {
        last_u[k] = 0.0;
      }
    }

    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements->gyr_duo,
              measurements->acc_duo, numStates, last_u);
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
               noiseParameters->ext_att_noise, measurements, 0.0,
               *b_VIOParameters, h_u_apo_out, map_out);
    i6 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i6, (int)sizeof(double));
    loop_ub = xt->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      xt_out->data[i6] = xt->data[i6];
    }

    i6 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i6, (int)sizeof(double));
    loop_ub = P->size[0] * P->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      P_apo_out->data[i6] = P->data[i6];
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
