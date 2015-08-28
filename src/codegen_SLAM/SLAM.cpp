//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 19:03:55
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
#include "fprintf.h"
#include "SLAM_pred.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
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
  int i;
  emxArray_real_T *xt_apo;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *r9;
  emxArray_real_T *r10;
  double R_bw[9];
  int k;
  int i11;
  double b_measurements[9];
  double c_measurements[9];
  double u_pred[4];
  double d5;
  static const double y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  double b_R_bw[3];
  double u_out_x;
  double u_out_y;
  double u_out_z;
  double dv24[9];
  double u_out_yaw;
  double b_u_out_x[3];
  double updateVect_apo[16];
  double d_measurements[9];

  //  for coder
  for (i = 0; i < 4; i++) {
    u_out[i] = 0.0;
  }

  //  for coder
  b_emxInit_real_T(&xt_apo, 1);
  b_emxInit_real_T(&r4, 1);
  emxInit_real_T(&r5, 2);
  emxInit_real_T(&r6, 2);
  b_emxInit_real_T(&r7, 1);
  emxInit_real_T(&r8, 2);
  emxInit_real_T(&r9, 2);
  b_emxInit_real_T(&r10, 1);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;
    ext_pose_offset_initialized = false;
    for (i = 0; i < 3; i++) {
      ext_pos_offset[i] = 0.0;
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
      for (i11 = 0; i11 < 3; i11++) {
        R_bw[i11 + 3 * k] = R_bc[k + 3 * i11];
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
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        b_measurements[i11 + 3 * i] = 0.0;
        for (k = 0; k < 3; k++) {
          b_measurements[i11 + 3 * i] += R_bw[i11 + 3 * k] * c_measurements[k +
            3 * i];
        }
      }
    }

    QuatFromRotJ(b_measurements, u_pred);
    i11 = r10->size[0];
    r10->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
    emxEnsureCapacity((emxArray__common *)r10, i11, (int)sizeof(double));
    r10->data[0] = 0.0;
    r10->data[1] = 0.0;
    r10->data[2] = 0.0;
    r10->data[3] = 0.0;
    r10->data[4] = 0.0;
    r10->data[5] = 0.0;
    r10->data[6] = 1.0;
    i = (int)b_VIOParameters->num_points_per_anchor;
    for (i11 = 0; i11 < i; i11++) {
      r10->data[i11 + 7] = 0.0;
    }

    repmat(r10, b_VIOParameters->num_anchors, r4);
    i11 = xt->size[0];
    xt->size[0] = 13 + r4->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i11, (int)sizeof(double));
    for (i11 = 0; i11 < 3; i11++) {
      xt->data[i11] = ref->position[i11];
    }

    for (i11 = 0; i11 < 4; i11++) {
      xt->data[i11 + 3] = u_pred[i11];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    i = r4->size[0];
    for (i11 = 0; i11 < i; i11++) {
      xt->data[i11 + 13] = r4->data[i11];
    }

    //  initial real vector
    d5 = b_VIOParameters->num_anchors * (6.0 +
      b_VIOParameters->num_points_per_anchor);
    i11 = r8->size[0] * r8->size[1];
    r8->size[0] = (int)numStates;
    r8->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r8, i11, (int)sizeof(double));
    i = (int)numStates * (int)numStates;
    for (i11 = 0; i11 < i; i11++) {
      r8->data[i11] = 0.0;
    }

    i11 = r9->size[0] * r9->size[1];
    r9->size[0] = (int)d5;
    r9->size[1] = (int)d5;
    emxEnsureCapacity((emxArray__common *)r9, i11, (int)sizeof(double));
    i = (int)d5 * (int)d5;
    for (i11 = 0; i11 < i; i11++) {
      r9->data[i11] = 0.0;
    }

    blkdiag(r8, r9, P);

    //  initial error state covariance
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        P->data[i + P->size[0] * i11] = 0.0;
      }
    }

    //  position
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (3 + i11)) + 3] = 0.0;
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (6 + i11)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        P->data[(i + P->size[0] * (9 + i11)) + 9] = y[i + 3 * i11];
      }
    }

    //  gyro bias
    for (i = 0; i < 16; i++) {
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
               noiseParameters->ext_att_noise, measurements, (1.0 - rt_powd_snf
                (measurements->bar_fmu / 101325.0, 0.190284)) * 145366.45,
               *b_VIOParameters, h_u_apo_out, map_out);
    i11 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i11, (int)sizeof(double));
    i = xt->size[0];
    for (i11 = 0; i11 < i; i11++) {
      xt_out->data[i11] = xt->data[i11];
    }

    i11 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i11, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i11 = 0; i11 < i; i11++) {
      P_apo_out->data[i11] = P->data[i11];
    }

    for (i = 0; i < 4; i++) {
      last_u[i] = 0.0;
    }

    //  the last control outputs (in camera frame)
  } else {
    if (b_VIOParameters->use_ext_pose) {
      if (!ext_pose_offset_initialized) {
        for (i11 = 0; i11 < 3; i11++) {
          ext_pos_offset[i11] = xt->data[i11] - measurements->pos_ext[i11];
        }

        //  in vio frame
        for (i11 = 0; i11 < 3; i11++) {
          measurements->pos_ext[i11] = xt->data[i11];
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
        for (i11 = 0; i11 < 3; i11++) {
          for (i = 0; i < 3; i++) {
            ext_att_offset[i11 + 3 * i] = 0.0;
            for (k = 0; k < 3; k++) {
              ext_att_offset[i11 + 3 * i] += c_measurements[i11 + 3 * k] *
                b_measurements[k + 3 * i];
            }
          }
        }

        for (i11 = 0; i11 < 4; i11++) {
          measurements->att_ext[i11] = xt->data[3 + i11];
        }

        ext_pose_offset_initialized = true;
      } else {
        for (i11 = 0; i11 < 3; i11++) {
          d5 = 0.0;
          for (i = 0; i < 3; i++) {
            d5 += ext_att_offset[i + 3 * i11] * measurements->pos_ext[i];
          }

          b_R_bw[i11] = d5 + ext_pos_offset[i11];
        }

        for (i11 = 0; i11 < 3; i11++) {
          measurements->pos_ext[i11] = b_R_bw[i11];
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
        for (i11 = 0; i11 < 3; i11++) {
          for (i = 0; i < 3; i++) {
            c_measurements[i11 + 3 * i] = 0.0;
            for (k = 0; k < 3; k++) {
              c_measurements[i11 + 3 * i] += b_measurements[i11 + 3 * k] *
                ext_att_offset[k + 3 * i];
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
    for (i11 = 0; i11 < 3; i11++) {
      for (i = 0; i < 3; i++) {
        R_bw[i11 + 3 * i] = 0.0;
        for (k = 0; k < 3; k++) {
          R_bw[i11 + 3 * i] += R_bc[i11 + 3 * k] * dv24[k + 3 * i];
        }
      }
    }

    u_out_yaw = b_ControllerGains->Kd_yaw * ref->velocity[3] -
      b_ControllerGains->Kp_yaw * (rt_atan2d_snf(R_bw[3], R_bw[0]) -
      ref->position[3]);
    b_u_out_x[0] = u_out_x;
    b_u_out_x[1] = u_out_y;
    b_u_out_x[2] = u_out_z;
    for (i11 = 0; i11 < 3; i11++) {
      b_R_bw[i11] = 0.0;
      for (i = 0; i < 3; i++) {
        b_R_bw[i11] += R_bw[i11 + 3 * i] * b_u_out_x[i];
      }
    }

    for (i11 = 0; i11 < 3; i11++) {
      u_out[i11] = b_R_bw[i11];
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

    last_u[0] = u_out_x;
    last_u[1] = u_out_y;
    last_u[2] = u_out_z;
    last_u[3] = u_out_yaw;
    SLAM_pred(P, xt, dt, noiseParameters->process_noise, measurements->gyr_duo,
              measurements->acc_duo, numStates, u_pred);
    i11 = xt_apo->size[0];
    xt_apo->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_apo, i11, (int)sizeof(double));
    i = xt->size[0];
    for (i11 = 0; i11 < i; i11++) {
      xt_apo->data[i11] = xt->data[i11];
    }

    for (i = 0; i < 16; i++) {
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
               noiseParameters->ext_att_noise, measurements, 0.0,
               *b_VIOParameters, h_u_apo_out, map_out);

    //  if almost all features were lost, do a soft reset
    i = 0;
    for (k = 0; k < 16; k++) {
      if ((updateVect[k] != 0.0) && (updateVect_apo[k] == 1.0)) {
        i++;
      }
    }

    if (i < 3) {
      // #coder
      // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
      //      coder.cinclude('<ros/console.h>')
      //      coder.ceval('ROS_WARN', [str, 0], varargin{:});
      n_fprintf();
      for (i = 0; i < 16; i++) {
        updateVect[i] = 0.0;
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          R_bw[i + 3 * i11] = R_bc[i11 + 3 * i];
        }
      }

      d_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0]
                            - measurements->att_fmu[1] * measurements->att_fmu[1])
                           - measurements->att_fmu[2] * measurements->att_fmu[2])
        + measurements->att_fmu[3] * measurements->att_fmu[3];
      d_measurements[3] = 2.0 * (measurements->att_fmu[0] *
        measurements->att_fmu[1] + measurements->att_fmu[2] *
        measurements->att_fmu[3]);
      d_measurements[6] = 2.0 * (measurements->att_fmu[0] *
        measurements->att_fmu[2] - measurements->att_fmu[1] *
        measurements->att_fmu[3]);
      d_measurements[1] = 2.0 * (measurements->att_fmu[0] *
        measurements->att_fmu[1] - measurements->att_fmu[2] *
        measurements->att_fmu[3]);
      d_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                            + measurements->att_fmu[1] * measurements->att_fmu[1])
                           - measurements->att_fmu[2] * measurements->att_fmu[2])
        + measurements->att_fmu[3] * measurements->att_fmu[3];
      d_measurements[7] = 2.0 * (measurements->att_fmu[1] *
        measurements->att_fmu[2] + measurements->att_fmu[0] *
        measurements->att_fmu[3]);
      d_measurements[2] = 2.0 * (measurements->att_fmu[0] *
        measurements->att_fmu[2] + measurements->att_fmu[1] *
        measurements->att_fmu[3]);
      d_measurements[5] = 2.0 * (measurements->att_fmu[1] *
        measurements->att_fmu[2] - measurements->att_fmu[0] *
        measurements->att_fmu[3]);
      d_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                            - measurements->att_fmu[1] * measurements->att_fmu[1])
                           + measurements->att_fmu[2] * measurements->att_fmu[2])
        + measurements->att_fmu[3] * measurements->att_fmu[3];
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          b_measurements[i11 + 3 * i] = 0.0;
          for (k = 0; k < 3; k++) {
            b_measurements[i11 + 3 * i] += R_bw[i11 + 3 * k] * d_measurements[k
              + 3 * i];
          }
        }
      }

      QuatFromRotJ(b_measurements, u_pred);
      i11 = r7->size[0];
      r7->size[0] = 7 + (int)b_VIOParameters->num_points_per_anchor;
      emxEnsureCapacity((emxArray__common *)r7, i11, (int)sizeof(double));
      r7->data[0] = 0.0;
      r7->data[1] = 0.0;
      r7->data[2] = 0.0;
      r7->data[3] = 0.0;
      r7->data[4] = 0.0;
      r7->data[5] = 0.0;
      r7->data[6] = 1.0;
      i = (int)b_VIOParameters->num_points_per_anchor;
      for (i11 = 0; i11 < i; i11++) {
        r7->data[i11 + 7] = 0.0;
      }

      repmat(r7, b_VIOParameters->num_anchors, r4);
      i11 = xt->size[0];
      xt->size[0] = (xt_apo->size[0] + r4->size[0]) + 10;
      emxEnsureCapacity((emxArray__common *)xt, i11, (int)sizeof(double));
      i = xt_apo->size[0];
      for (i11 = 0; i11 < i; i11++) {
        xt->data[i11] = xt_apo->data[i11];
      }

      for (i11 = 0; i11 < 4; i11++) {
        xt->data[i11 + xt_apo->size[0]] = u_pred[i11];
      }

      xt->data[xt_apo->size[0] + 4] = 0.0;
      xt->data[xt_apo->size[0] + 5] = 0.0;
      xt->data[xt_apo->size[0] + 6] = 0.0;
      xt->data[xt_apo->size[0] + 7] = 0.0;
      xt->data[xt_apo->size[0] + 8] = 0.0;
      xt->data[xt_apo->size[0] + 9] = 0.0;
      i = r4->size[0];
      for (i11 = 0; i11 < i; i11++) {
        xt->data[(i11 + xt_apo->size[0]) + 10] = r4->data[i11];
      }

      //  initial real vector
      d5 = b_VIOParameters->num_anchors * (6.0 +
        b_VIOParameters->num_points_per_anchor);
      i11 = r5->size[0] * r5->size[1];
      r5->size[0] = (int)numStates;
      r5->size[1] = (int)numStates;
      emxEnsureCapacity((emxArray__common *)r5, i11, (int)sizeof(double));
      i = (int)numStates * (int)numStates;
      for (i11 = 0; i11 < i; i11++) {
        r5->data[i11] = 0.0;
      }

      i11 = r6->size[0] * r6->size[1];
      r6->size[0] = (int)d5;
      r6->size[1] = (int)d5;
      emxEnsureCapacity((emxArray__common *)r6, i11, (int)sizeof(double));
      i = (int)d5 * (int)d5;
      for (i11 = 0; i11 < i; i11++) {
        r6->data[i11] = 0.0;
      }

      blkdiag(r5, r6, P);

      //  initial error state covariance
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          P->data[i + P->size[0] * i11] = 0.0;
        }
      }

      //  position
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i11)) + 3] = 0.0;
        }
      }

      //  orientation
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (6 + i11)) + 6] = 0.0;
        }
      }

      //  velocity
      for (i11 = 0; i11 < 3; i11++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (9 + i11)) + 9] = y[i + 3 * i11];
        }
      }

      //  gyro bias
    } else {
      i11 = xt->size[0];
      xt->size[0] = xt_apo->size[0];
      emxEnsureCapacity((emxArray__common *)xt, i11, (int)sizeof(double));
      i = xt_apo->size[0];
      for (i11 = 0; i11 < i; i11++) {
        xt->data[i11] = xt_apo->data[i11];
      }

      memcpy(&updateVect[0], &updateVect_apo[0], sizeof(double) << 4);
    }

    i11 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i11, (int)sizeof(double));
    i = xt->size[0];
    for (i11 = 0; i11 < i; i11++) {
      xt_out->data[i11] = xt->data[i11];
    }

    i11 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i11, (int)sizeof(double));
    i = P->size[0] * P->size[1];
    for (i11 = 0; i11 < i; i11++) {
      P_apo_out->data[i11] = P->data[i11];
    }

    // % output asserts for coder
  }

  emxFree_real_T(&r10);
  emxFree_real_T(&r9);
  emxFree_real_T(&r8);
  emxFree_real_T(&r7);
  emxFree_real_T(&r6);
  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
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
