//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "getAnchorPoses.h"
#include "QuatFromRotJ.h"
#include "getMap.h"
#include "norm.h"
#include "SLAM_pred_euler.h"
#include "SLAM_upd.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static f_struct_T xt;
static emxArray_real_T *P;
static emxArray_real_T *h_u;
static emxArray_real_T *map;
static double delayedStatus[40];

// Function Definitions

//
// NOTE: Comment this out for MEXing
//  in
// Arguments    : int updateVect[40]
//                const double z_all_l[80]
//                const double z_all_r[80]
//                double dt
//                const VIOMeasurements *measurements
//                const DUOParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const VIOParameters *b_VIOParameters
//                boolean_T vision
//                RobotState *xt_out
//                emxArray_real_T *h_u_out
//                emxArray_real_T *map_out
//                emxArray_AnchorPose *anchor_poses_out
//                double delayedStatus_out[40]
// Return Type  : void
//
void SLAM(int updateVect[40], const double z_all_l[80], const double z_all_r[80],
          double dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, RobotState *xt_out,
          emxArray_real_T *h_u_out, emxArray_real_T *map_out,
          emxArray_AnchorPose *anchor_poses_out, double delayedStatus_out[40])
{
  double numStatesPerAnchor;
  e_struct_T anchor_state;
  emxArray_c_struct_T *x;
  boolean_T guard1 = false;
  int i;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  int i30;
  static const d_struct_T r1 = { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0.0,
    0.0, 0.0 };

  double B;
  double R_cw_init[9];
  double t0_pos[3];
  double dv1[9];
  double dv2[9];
  int j;
  double t0_vel[3];
  double t0_att[4];
  double z_b[3];
  double y_n_b[3];
  double x_n_b[3];
  double b_x_n_b[9];
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const double b_y[9] = { 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.001 };

  static const signed char c_y[9] = { 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double b_z_all_l[80];
  double b_z_all_r[80];

  //  TODO
  //  coder.cstructname(ControllerGains, 'ControllerGains', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  //  coder.cstructname(ref, 'ReferenceCommand', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  numStates = 21.0;

  //  also estimate origin orientation, acc bias, Rci
  numStatesxt = 22.0;
  numStatesPerAnchor = 6.0 + numPointsPerAnchor;
  b_emxInitStruct_struct_T(&anchor_state);
  c_emxInit_struct_T(&x, 2);
  guard1 = false;
  if (!initialized_not_empty) {
    for (i = 0; i < 40; i++) {
      updateVect[i] = 0;
    }

    for (i = 0; i < 3; i++) {
      xt.robot_state.IMU.pos[i] = cameraParameters->t_ci[i];
    }

    QuatFromRotJ(cameraParameters->R_ci, xt.robot_state.IMU.att);
    for (i = 0; i < 3; i++) {
      xt.robot_state.IMU.gyro_bias[i] = cameraParameters->gyro_bias[i];
      xt.robot_state.IMU.acc_bias[i] = cameraParameters->acc_bias[i];
      xt.robot_state.pos[i] = 0.0;
    }

    //  position relative to the origin frame
    for (i = 0; i < 4; i++) {
      xt.robot_state.att[i] = iv2[i];
    }

    //  orientation relative to the origin frame
    for (i = 0; i < 3; i++) {
      xt.robot_state.vel[i] = 0.0;
    }

    //  velocity in the origin frame
    xt.fixed_feature = 0.0;
    xt.origin.anchor_idx = 0.0;

    //  idx of the anchor that is at the origin
    for (i = 0; i < 3; i++) {
      xt.origin.pos[i] = 0.0;
    }

    //  position of the origin in the world frame
    for (i = 0; i < 4; i++) {
      xt.origin.att[i] = iv2[i];
    }

    //  orientation of the origin in the world frame
    for (i = 0; i < 3; i++) {
      anchor_state.pos[i] = 0.0;
    }

    for (i = 0; i < 4; i++) {
      anchor_state.att[i] = iv2[i];
    }

    for (i30 = 0; i30 < 6; i30++) {
      anchor_state.P_idx[i30] = 0.0;
    }

    i = (int)numPointsPerAnchor;
    i30 = anchor_state.feature_states->size[0];
    anchor_state.feature_states->size[0] = (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)anchor_state.feature_states, i30, (int)
                      sizeof(d_struct_T));
    for (i30 = 0; i30 < i; i30++) {
      anchor_state.feature_states->data[i30] = r1;
    }

    i = (int)numAnchors;
    i30 = xt.anchor_states->size[0];
    xt.anchor_states->size[0] = (int)numAnchors;
    emxEnsureCapacity_struct_T(xt.anchor_states, i30);
    for (i30 = 0; i30 < i; i30++) {
      emxCopyStruct_struct_T(&xt.anchor_states->data[i30], &anchor_state);
    }

    for (i = 0; i < (int)numAnchors; i++) {
      B = numStates + ((1.0 + (double)i) - 1.0) * numStatesPerAnchor;
      for (i30 = 0; i30 < 6; i30++) {
        xt.anchor_states->data[i].P_idx[i30] = B + (1.0 + (double)i30);
      }
    }

    if (vision) {
      // getWorldState Get the state of the robot in the world frame
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
      R_cw_init[0] = ((xt.origin.att[0] * xt.origin.att[0] - xt.origin.att[1] *
                       xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2])
        + xt.origin.att[3] * xt.origin.att[3];
      R_cw_init[3] = 2.0 * (xt.origin.att[0] * xt.origin.att[1] + xt.origin.att
                            [2] * xt.origin.att[3]);
      R_cw_init[6] = 2.0 * (xt.origin.att[0] * xt.origin.att[2] - xt.origin.att
                            [1] * xt.origin.att[3]);
      R_cw_init[1] = 2.0 * (xt.origin.att[0] * xt.origin.att[1] - xt.origin.att
                            [2] * xt.origin.att[3]);
      R_cw_init[4] = ((-(xt.origin.att[0] * xt.origin.att[0]) + xt.origin.att[1]
                       * xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2])
        + xt.origin.att[3] * xt.origin.att[3];
      R_cw_init[7] = 2.0 * (xt.origin.att[1] * xt.origin.att[2] + xt.origin.att
                            [0] * xt.origin.att[3]);
      R_cw_init[2] = 2.0 * (xt.origin.att[0] * xt.origin.att[2] + xt.origin.att
                            [1] * xt.origin.att[3]);
      R_cw_init[5] = 2.0 * (xt.origin.att[1] * xt.origin.att[2] - xt.origin.att
                            [0] * xt.origin.att[3]);
      R_cw_init[8] = ((-(xt.origin.att[0] * xt.origin.att[0]) - xt.origin.att[1]
                       * xt.origin.att[1]) + xt.origin.att[2] * xt.origin.att[2])
        + xt.origin.att[3] * xt.origin.att[3];
      for (i30 = 0; i30 < 3; i30++) {
        B = 0.0;
        for (i = 0; i < 3; i++) {
          B += R_cw_init[i + 3 * i30] * xt.robot_state.pos[i];
        }

        t0_pos[i30] = xt.origin.pos[i30] + B;
      }

      dv2[0] = ((xt.robot_state.att[0] * xt.robot_state.att[0] -
                 xt.robot_state.att[1] * xt.robot_state.att[1]) -
                xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];
      dv2[3] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[1] +
                      xt.robot_state.att[2] * xt.robot_state.att[3]);
      dv2[6] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[2] -
                      xt.robot_state.att[1] * xt.robot_state.att[3]);
      dv2[1] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[1] -
                      xt.robot_state.att[2] * xt.robot_state.att[3]);
      dv2[4] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) +
                 xt.robot_state.att[1] * xt.robot_state.att[1]) -
                xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];
      dv2[7] = 2.0 * (xt.robot_state.att[1] * xt.robot_state.att[2] +
                      xt.robot_state.att[0] * xt.robot_state.att[3]);
      dv2[2] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[2] +
                      xt.robot_state.att[1] * xt.robot_state.att[3]);
      dv2[5] = 2.0 * (xt.robot_state.att[1] * xt.robot_state.att[2] -
                      xt.robot_state.att[0] * xt.robot_state.att[3]);
      dv2[8] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) -
                 xt.robot_state.att[1] * xt.robot_state.att[1]) +
                xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          dv1[i30 + 3 * i] = 0.0;
          for (j = 0; j < 3; j++) {
            dv1[i30 + 3 * i] += dv2[i30 + 3 * j] * R_cw_init[j + 3 * i];
          }
        }

        t0_vel[i30] = 0.0;
        for (i = 0; i < 3; i++) {
          t0_vel[i30] += R_cw_init[i + 3 * i30] * xt.robot_state.pos[i];
        }
      }

      QuatFromRotJ(dv1, t0_att);
      for (i = 0; i < 3; i++) {
        xt_out->pos[i] = t0_pos[i];
      }

      for (i = 0; i < 4; i++) {
        xt_out->att[i] = t0_att[i];
      }

      for (i = 0; i < 3; i++) {
        xt_out->vel[i] = t0_vel[i];
      }

      for (i = 0; i < 3; i++) {
        xt_out->IMU.gyro_bias[i] = xt.robot_state.IMU.gyro_bias[i];
      }

      for (i = 0; i < 3; i++) {
        xt_out->IMU.acc_bias[i] = xt.robot_state.IMU.acc_bias[i];
      }

      for (i = 0; i < 3; i++) {
        xt_out->IMU.pos[i] = xt.robot_state.IMU.pos[i];
      }

      for (i = 0; i < 4; i++) {
        xt_out->IMU.att[i] = xt.robot_state.IMU.att[i];
      }

      i30 = h_u_out->size[0];
      h_u_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_out, i30, (int)sizeof(double));
      i = (int)(numTrackFeatures * 4.0);
      for (i30 = 0; i30 < i; i30++) {
        h_u_out->data[i30] = rtNaN;
      }

      i30 = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, i30, (int)sizeof(double));
      i = 3 * (int)numTrackFeatures;
      for (i30 = 0; i30 < i; i30++) {
        map_out->data[i30] = rtNaN;
      }

      getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, x);
      i30 = anchor_poses_out->size[0] * anchor_poses_out->size[1];
      anchor_poses_out->size[0] = x->size[0];
      anchor_poses_out->size[1] = x->size[1];
      emxEnsureCapacity((emxArray__common *)anchor_poses_out, i30, (int)sizeof
                        (AnchorPose));
      i = x->size[0] * x->size[1];
      for (j = 0; j < i; j++) {
        for (i30 = 0; i30 < 3; i30++) {
          anchor_poses_out->data[j].pos[i30] = x->data[j].pos[i30];
        }

        for (i30 = 0; i30 < 4; i30++) {
          anchor_poses_out->data[j].att[i30] = x->data[j].att[i30];
        }
      }

      memset(&delayedStatus[0], 0, 40U * sizeof(double));
      memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));
    } else {
      for (i = 0; i < 3; i++) {
        z_b[i] = measurements->acc_duo[i] - xt.robot_state.IMU.acc_bias[i];
      }

      B = norm(z_b);
      for (i30 = 0; i30 < 3; i30++) {
        z_b[i30] /= B;
      }

      y_n_b[0] = z_b[1] - z_b[2] * 0.0;
      y_n_b[1] = z_b[2] * 0.0 - z_b[0];
      y_n_b[2] = z_b[0] * 0.0 - z_b[1] * 0.0;
      B = norm(y_n_b);
      for (i30 = 0; i30 < 3; i30++) {
        y_n_b[i30] /= B;
      }

      x_n_b[0] = y_n_b[1] * z_b[2] - y_n_b[2] * z_b[1];
      x_n_b[1] = y_n_b[2] * z_b[0] - y_n_b[0] * z_b[2];
      x_n_b[2] = y_n_b[0] * z_b[1] - y_n_b[1] * z_b[0];
      B = norm(x_n_b);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      dv1[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv1[3] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv1[6] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv1[1] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv1[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv1[7] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv1[2] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv1[5] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv1[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (i30 = 0; i30 < 3; i30++) {
        b_x_n_b[i30] = x_n_b[i30] / B;
        b_x_n_b[3 + i30] = y_n_b[i30];
        b_x_n_b[6 + i30] = z_b[i30];
      }

      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          R_cw_init[i30 + 3 * i] = 0.0;
          for (j = 0; j < 3; j++) {
            R_cw_init[i30 + 3 * i] += dv1[i30 + 3 * j] * b_x_n_b[j + 3 * i];
          }
        }
      }

      QuatFromRotJ(R_cw_init, xt.origin.att);

      //  orientation of the origin in the world frame
      initialized_not_empty = true;
      B = numStates + numAnchors * (6.0 + numPointsPerAnchor);
      i30 = P->size[0] * P->size[1];
      P->size[0] = (int)B;
      P->size[1] = (int)B;
      emxEnsureCapacity((emxArray__common *)P, i30, (int)sizeof(double));
      i = (int)B * (int)B;
      for (i30 = 0; i30 < i; i30++) {
        P->data[i30] = 0.0;
      }

      //  initial error state covariance
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[i + P->size[0] * i30] = 0.0;
        }
      }

      //  position
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (3 + i30)) + 3] = 0.0;
        }
      }

      //  orientation of camera in origin frame
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (6 + i30)) + 6] = y[i + 3 * i30];
        }
      }

      //  velocity
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (9 + i30)) + 9] = b_y[i + 3 * i30];
        }
      }

      //  gyro bias
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (12 + i30)) + 12] = c_y[i + 3 * i30];
        }
      }

      // 0*0.1*eye(3); % acc bias
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          dv1[i30 + 3 * i] = 0.0;
          for (j = 0; j < 3; j++) {
            dv1[i30 + 3 * i] += 0.1 * R_cw_init[i30 + 3 * j] * (double)b[j + 3 *
              i];
          }
        }
      }

      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i30 + P->size[0] * (15 + i)) + 15] = 0.0;
          for (j = 0; j < 3; j++) {
            P->data[(i30 + P->size[0] * (15 + i)) + 15] += dv1[i30 + 3 * j] *
              R_cw_init[i + 3 * j];
          }
        }
      }

      //  origin orientation
      for (i30 = 0; i30 < 3; i30++) {
        for (i = 0; i < 3; i++) {
          P->data[(i + P->size[0] * (18 + i30)) + 18] = 0.0;
        }
      }

      //  R_ci
      i30 = h_u->size[0];
      h_u->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u, i30, (int)sizeof(double));
      i = (int)(numTrackFeatures * 4.0);
      for (i30 = 0; i30 < i; i30++) {
        h_u->data[i30] = rtNaN;
      }

      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);
      memset(&delayedStatus[0], 0, 40U * sizeof(double));
      guard1 = true;
    }
  } else {
    if (!vision) {
      //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
      SLAM_pred_euler(P, &xt, dt, noiseParameters->process_noise.qv,
                      noiseParameters->process_noise.qw,
                      noiseParameters->process_noise.qao,
                      noiseParameters->process_noise.qwo,
                      noiseParameters->process_noise.qR_ci,
                      measurements->acc_duo, measurements->gyr_duo, numStates);
    } else {
      for (i = 0; i < 80; i++) {
        b_z_all_l[i] = z_all_l[i];
        b_z_all_r[i] = z_all_r[i];
      }

      SLAM_upd(P, &xt, cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
               noiseParameters->image_noise, noiseParameters->sigmaInit,
               b_VIOParameters->max_ekf_iterations,
               b_VIOParameters->fixed_feature,
               b_VIOParameters->delayed_initialization, b_VIOParameters->mono,
               h_u, map, delayedStatus);
    }

    guard1 = true;
  }

  if (guard1) {
    i30 = h_u_out->size[0];
    h_u_out->size[0] = h_u->size[0];
    emxEnsureCapacity((emxArray__common *)h_u_out, i30, (int)sizeof(double));
    i = h_u->size[0];
    for (i30 = 0; i30 < i; i30++) {
      h_u_out->data[i30] = h_u->data[i30];
    }

    i30 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = map->size[1];
    emxEnsureCapacity((emxArray__common *)map_out, i30, (int)sizeof(double));
    i = map->size[0] * map->size[1];
    for (i30 = 0; i30 < i; i30++) {
      map_out->data[i30] = map->data[i30];
    }

    // getWorldState Get the state of the robot in the world frame
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
    R_cw_init[0] = ((xt.origin.att[0] * xt.origin.att[0] - xt.origin.att[1] *
                     xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];
    R_cw_init[3] = 2.0 * (xt.origin.att[0] * xt.origin.att[1] + xt.origin.att[2]
                          * xt.origin.att[3]);
    R_cw_init[6] = 2.0 * (xt.origin.att[0] * xt.origin.att[2] - xt.origin.att[1]
                          * xt.origin.att[3]);
    R_cw_init[1] = 2.0 * (xt.origin.att[0] * xt.origin.att[1] - xt.origin.att[2]
                          * xt.origin.att[3]);
    R_cw_init[4] = ((-(xt.origin.att[0] * xt.origin.att[0]) + xt.origin.att[1] *
                     xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];
    R_cw_init[7] = 2.0 * (xt.origin.att[1] * xt.origin.att[2] + xt.origin.att[0]
                          * xt.origin.att[3]);
    R_cw_init[2] = 2.0 * (xt.origin.att[0] * xt.origin.att[2] + xt.origin.att[1]
                          * xt.origin.att[3]);
    R_cw_init[5] = 2.0 * (xt.origin.att[1] * xt.origin.att[2] - xt.origin.att[0]
                          * xt.origin.att[3]);
    R_cw_init[8] = ((-(xt.origin.att[0] * xt.origin.att[0]) - xt.origin.att[1] *
                     xt.origin.att[1]) + xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];
    for (i30 = 0; i30 < 3; i30++) {
      B = 0.0;
      for (i = 0; i < 3; i++) {
        B += R_cw_init[i + 3 * i30] * xt.robot_state.pos[i];
      }

      t0_pos[i30] = xt.origin.pos[i30] + B;
    }

    dv2[0] = ((xt.robot_state.att[0] * xt.robot_state.att[0] -
               xt.robot_state.att[1] * xt.robot_state.att[1]) -
              xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];
    dv2[3] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[1] +
                    xt.robot_state.att[2] * xt.robot_state.att[3]);
    dv2[6] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[2] -
                    xt.robot_state.att[1] * xt.robot_state.att[3]);
    dv2[1] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[1] -
                    xt.robot_state.att[2] * xt.robot_state.att[3]);
    dv2[4] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) +
               xt.robot_state.att[1] * xt.robot_state.att[1]) -
              xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];
    dv2[7] = 2.0 * (xt.robot_state.att[1] * xt.robot_state.att[2] +
                    xt.robot_state.att[0] * xt.robot_state.att[3]);
    dv2[2] = 2.0 * (xt.robot_state.att[0] * xt.robot_state.att[2] +
                    xt.robot_state.att[1] * xt.robot_state.att[3]);
    dv2[5] = 2.0 * (xt.robot_state.att[1] * xt.robot_state.att[2] -
                    xt.robot_state.att[0] * xt.robot_state.att[3]);
    dv2[8] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) -
               xt.robot_state.att[1] * xt.robot_state.att[1]) +
              xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];
    for (i30 = 0; i30 < 3; i30++) {
      for (i = 0; i < 3; i++) {
        dv1[i30 + 3 * i] = 0.0;
        for (j = 0; j < 3; j++) {
          dv1[i30 + 3 * i] += dv2[i30 + 3 * j] * R_cw_init[j + 3 * i];
        }
      }

      t0_vel[i30] = 0.0;
      for (i = 0; i < 3; i++) {
        t0_vel[i30] += R_cw_init[i + 3 * i30] * xt.robot_state.pos[i];
      }
    }

    QuatFromRotJ(dv1, t0_att);
    for (i = 0; i < 3; i++) {
      xt_out->pos[i] = t0_pos[i];
    }

    for (i = 0; i < 4; i++) {
      xt_out->att[i] = t0_att[i];
    }

    for (i = 0; i < 3; i++) {
      xt_out->vel[i] = t0_vel[i];
    }

    for (i = 0; i < 3; i++) {
      xt_out->IMU.gyro_bias[i] = xt.robot_state.IMU.gyro_bias[i];
    }

    for (i = 0; i < 3; i++) {
      xt_out->IMU.acc_bias[i] = xt.robot_state.IMU.acc_bias[i];
    }

    for (i = 0; i < 3; i++) {
      xt_out->IMU.pos[i] = xt.robot_state.IMU.pos[i];
    }

    for (i = 0; i < 4; i++) {
      xt_out->IMU.att[i] = xt.robot_state.IMU.att[i];
    }

    getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, x);
    i30 = anchor_poses_out->size[0] * anchor_poses_out->size[1];
    anchor_poses_out->size[0] = x->size[0];
    anchor_poses_out->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)anchor_poses_out, i30, (int)sizeof
                      (AnchorPose));
    i = x->size[0] * x->size[1];
    for (j = 0; j < i; j++) {
      for (i30 = 0; i30 < 3; i30++) {
        anchor_poses_out->data[j].pos[i30] = x->data[j].pos[i30];
      }

      for (i30 = 0; i30 < 4; i30++) {
        anchor_poses_out->data[j].att[i30] = x->data[j].att[i30];
      }
    }

    memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));

    //  NOTE: Comment this out for MEXing
    //  out
  }

  c_emxFree_struct_T(&x);
  emxFreeStruct_struct_T(&anchor_state);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_free()
{
  emxFree_real_T(&map);
  emxFree_real_T(&h_u);
  emxFree_real_T(&P);
  b_emxFreeStruct_struct_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_init()
{
  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&h_u, 1);
  emxInit_real_T(&P, 2);
  emxInitStruct_struct_T(&xt);
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
