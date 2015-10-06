//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "getAnchorPoses.h"
#include "QuatFromRotJ.h"
#include "fprintf.h"
#include "getMap.h"
#include "norm.h"
#include "SLAM_pred_euler.h"
#include "SLAM_upd.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static f_struct_T xt;
static emxArray_real_T *P;
static emxArray_real_T *h_u;
static emxArray_real_T *map;
static double delayedStatus[40];

// Function Definitions

//
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
  int j;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  int i26;
  static const d_struct_T r1 = { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0.0,
    0.0, 0.0 };

  double B;
  double R_cw_init[9];
  double t0_pos[3];
  double dv1[9];
  double dv2[9];
  int n;
  double t0_vel[3];
  double t0_att[4];
  double z_b[3];
  double y_n_b[3];
  double x_n_b[3];
  double b_x_n_b[9];
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double d[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  char cv12[18];
  static const char cv13[18] = { 'N', 'o', 'i', 's', 'e', ' ', 'p', 'a', 'r',
    'a', 'm', 'e', 't', 'e', 'r', 's', ':', '\x00' };

  char cv14[15];
  static const char cv15[15] = { 'V', 'I', 'O', ' ', 'p', 'a', 'r', 'a', 'm',
    'e', 't', 'e', 'r', 's', '\x00' };

  int s_size[2];
  static const char cv16[4] = { 'T', 'r', 'u', 'e' };

  char s_data[5];
  static const char cv17[5] = { 'F', 'a', 'l', 's', 'e' };

  double b_z_all_l[80];
  double b_z_all_r[80];

  //  NOTE: Comment this out for MEXing
  //  in
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
    for (j = 0; j < 40; j++) {
      updateVect[j] = 0;
    }

    for (j = 0; j < 3; j++) {
      xt.robot_state.IMU.pos[j] = cameraParameters->t_ci[j];
    }

    QuatFromRotJ(cameraParameters->R_ci, xt.robot_state.IMU.att);
    for (j = 0; j < 3; j++) {
      xt.robot_state.IMU.gyro_bias[j] = cameraParameters->gyro_bias[j];
      xt.robot_state.IMU.acc_bias[j] = cameraParameters->acc_bias[j];
      xt.robot_state.pos[j] = 0.0;
    }

    //  position relative to the origin frame
    for (j = 0; j < 4; j++) {
      xt.robot_state.att[j] = iv2[j];
    }

    //  orientation relative to the origin frame
    for (j = 0; j < 3; j++) {
      xt.robot_state.vel[j] = 0.0;
    }

    //  velocity in the origin frame
    xt.fixed_feature = 0.0;
    xt.origin.anchor_idx = 0.0;

    //  idx of the anchor that is at the origin
    for (j = 0; j < 3; j++) {
      xt.origin.pos[j] = 0.0;
    }

    //  position of the origin in the world frame
    for (j = 0; j < 4; j++) {
      xt.origin.att[j] = iv2[j];
    }

    //  orientation of the origin in the world frame
    for (j = 0; j < 3; j++) {
      anchor_state.pos[j] = 0.0;
    }

    for (j = 0; j < 4; j++) {
      anchor_state.att[j] = iv2[j];
    }

    for (i26 = 0; i26 < 6; i26++) {
      anchor_state.P_idx[i26] = 0.0;
    }

    j = (int)numPointsPerAnchor;
    i26 = anchor_state.feature_states->size[0];
    anchor_state.feature_states->size[0] = (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)anchor_state.feature_states, i26, (int)
                      sizeof(d_struct_T));
    for (i26 = 0; i26 < j; i26++) {
      anchor_state.feature_states->data[i26] = r1;
    }

    j = (int)numAnchors;
    i26 = xt.anchor_states->size[0];
    xt.anchor_states->size[0] = (int)numAnchors;
    emxEnsureCapacity_struct_T(xt.anchor_states, i26);
    for (i26 = 0; i26 < j; i26++) {
      emxCopyStruct_struct_T(&xt.anchor_states->data[i26], &anchor_state);
    }

    for (j = 0; j < (int)numAnchors; j++) {
      B = numStates + ((1.0 + (double)j) - 1.0) * numStatesPerAnchor;
      for (i26 = 0; i26 < 6; i26++) {
        xt.anchor_states->data[j].P_idx[i26] = B + (1.0 + (double)i26);
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
      for (i26 = 0; i26 < 3; i26++) {
        B = 0.0;
        for (j = 0; j < 3; j++) {
          B += R_cw_init[j + 3 * i26] * xt.robot_state.pos[j];
        }

        t0_pos[i26] = xt.origin.pos[i26] + B;
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
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          dv1[i26 + 3 * j] = 0.0;
          for (n = 0; n < 3; n++) {
            dv1[i26 + 3 * j] += dv2[i26 + 3 * n] * R_cw_init[n + 3 * j];
          }
        }

        t0_vel[i26] = 0.0;
        for (j = 0; j < 3; j++) {
          t0_vel[i26] += R_cw_init[j + 3 * i26] * xt.robot_state.pos[j];
        }
      }

      QuatFromRotJ(dv1, t0_att);
      for (j = 0; j < 3; j++) {
        xt_out->pos[j] = t0_pos[j];
      }

      for (j = 0; j < 4; j++) {
        xt_out->att[j] = t0_att[j];
      }

      for (j = 0; j < 3; j++) {
        xt_out->vel[j] = t0_vel[j];
      }

      for (j = 0; j < 3; j++) {
        xt_out->IMU.gyro_bias[j] = xt.robot_state.IMU.gyro_bias[j];
      }

      for (j = 0; j < 3; j++) {
        xt_out->IMU.acc_bias[j] = xt.robot_state.IMU.acc_bias[j];
      }

      for (j = 0; j < 3; j++) {
        xt_out->IMU.pos[j] = xt.robot_state.IMU.pos[j];
      }

      for (j = 0; j < 4; j++) {
        xt_out->IMU.att[j] = xt.robot_state.IMU.att[j];
      }

      i26 = h_u_out->size[0];
      h_u_out->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u_out, i26, (int)sizeof(double));
      j = (int)(numTrackFeatures * 4.0);
      for (i26 = 0; i26 < j; i26++) {
        h_u_out->data[i26] = rtNaN;
      }

      i26 = map_out->size[0] * map_out->size[1];
      map_out->size[0] = 3;
      map_out->size[1] = (int)numTrackFeatures;
      emxEnsureCapacity((emxArray__common *)map_out, i26, (int)sizeof(double));
      j = 3 * (int)numTrackFeatures;
      for (i26 = 0; i26 < j; i26++) {
        map_out->data[i26] = rtNaN;
      }

      getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, x);
      i26 = anchor_poses_out->size[0] * anchor_poses_out->size[1];
      anchor_poses_out->size[0] = x->size[0];
      anchor_poses_out->size[1] = x->size[1];
      emxEnsureCapacity((emxArray__common *)anchor_poses_out, i26, (int)sizeof
                        (AnchorPose));
      n = x->size[0] * x->size[1];
      for (j = 0; j < n; j++) {
        for (i26 = 0; i26 < 3; i26++) {
          anchor_poses_out->data[j].pos[i26] = x->data[j].pos[i26];
        }

        for (i26 = 0; i26 < 4; i26++) {
          anchor_poses_out->data[j].att[i26] = x->data[j].att[i26];
        }
      }

      memset(&delayedStatus[0], 0, 40U * sizeof(double));
      memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));
    } else {
      for (j = 0; j < 3; j++) {
        z_b[j] = measurements->acc_duo[j] - xt.robot_state.IMU.acc_bias[j];
      }

      B = norm(z_b);
      for (i26 = 0; i26 < 3; i26++) {
        z_b[i26] /= B;
      }

      y_n_b[0] = z_b[1] - z_b[2] * 0.0;
      y_n_b[1] = z_b[2] * 0.0 - z_b[0];
      y_n_b[2] = z_b[0] * 0.0 - z_b[1] * 0.0;
      B = norm(y_n_b);
      for (i26 = 0; i26 < 3; i26++) {
        y_n_b[i26] /= B;
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
      for (i26 = 0; i26 < 3; i26++) {
        b_x_n_b[i26] = x_n_b[i26] / B;
        b_x_n_b[3 + i26] = y_n_b[i26];
        b_x_n_b[6 + i26] = z_b[i26];
      }

      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          R_cw_init[i26 + 3 * j] = 0.0;
          for (n = 0; n < 3; n++) {
            R_cw_init[i26 + 3 * j] += dv1[i26 + 3 * n] * b_x_n_b[n + 3 * j];
          }
        }
      }

      QuatFromRotJ(R_cw_init, xt.origin.att);

      //  orientation of the origin in the world frame
      B = numStates + numAnchors * (6.0 + numPointsPerAnchor);
      i26 = P->size[0] * P->size[1];
      P->size[0] = (int)B;
      P->size[1] = (int)B;
      emxEnsureCapacity((emxArray__common *)P, i26, (int)sizeof(double));
      j = (int)B * (int)B;
      for (i26 = 0; i26 < j; i26++) {
        P->data[i26] = 0.0;
      }

      //  initial error state covariance
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[j + P->size[0] * i26] = 0.0;
        }
      }

      //  position
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(j + P->size[0] * (3 + i26)) + 3] = 0.0;
        }
      }

      //  orientation of camera in origin frame
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(j + P->size[0] * (6 + i26)) + 6] = y[j + 3 * i26];
        }
      }

      //  velocity
      //          P(10:12, 10:12) = 0.001*eye(3); % gyro bias
      memset(&d[0], 0, 9U * sizeof(double));
      for (j = 0; j < 3; j++) {
        d[j + 3 * j] = noiseParameters->gyro_bias_initial_unc[j];
      }

      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(j + P->size[0] * (9 + i26)) + 9] = d[j + 3 * i26];
        }
      }

      //  gyro bias
      //          P(13:15, 13:15) = 1*diag([0 1 0]);%0*0.1*eye(3); % acc bias
      memset(&d[0], 0, 9U * sizeof(double));
      for (j = 0; j < 3; j++) {
        d[j + 3 * j] = noiseParameters->acc_bias_initial_unc[j];
      }

      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(j + P->size[0] * (12 + i26)) + 12] = d[j + 3 * i26];
        }
      }

      //  acc bias
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          dv1[i26 + 3 * j] = 0.0;
          for (n = 0; n < 3; n++) {
            dv1[i26 + 3 * j] += 0.1 * R_cw_init[i26 + 3 * n] * (double)b[n + 3 *
              j];
          }
        }
      }

      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(i26 + P->size[0] * (15 + j)) + 15] = 0.0;
          for (n = 0; n < 3; n++) {
            P->data[(i26 + P->size[0] * (15 + j)) + 15] += dv1[i26 + 3 * n] *
              R_cw_init[j + 3 * n];
          }
        }
      }

      //  origin orientation
      for (i26 = 0; i26 < 3; i26++) {
        for (j = 0; j < 3; j++) {
          P->data[(j + P->size[0] * (18 + i26)) + 18] = 0.0;
        }
      }

      //  R_ci
      i26 = h_u->size[0];
      h_u->size[0] = (int)(numTrackFeatures * 4.0);
      emxEnsureCapacity((emxArray__common *)h_u, i26, (int)sizeof(double));
      j = (int)(numTrackFeatures * 4.0);
      for (i26 = 0; i26 < j; i26++) {
        h_u->data[i26] = rtNaN;
      }

      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);
      memset(&delayedStatus[0], 0, 40U * sizeof(double));

      //  print all parameters for debug check
      b_fprintf();

      // #coder
      // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
      //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
      if (debug_level >= 2.0) {
        for (i26 = 0; i26 < 18; i26++) {
          cv12[i26] = cv13[i26];
        }

        ROS_INFO(cv12);
      }

      d_fprintf(noiseParameters->process_noise.qv);
      f_fprintf(noiseParameters->process_noise.qw);
      h_fprintf(noiseParameters->process_noise.qao);
      j_fprintf(noiseParameters->process_noise.qwo);
      l_fprintf(noiseParameters->process_noise.qR_ci);
      n_fprintf(noiseParameters->gyro_bias_initial_unc[0],
                noiseParameters->gyro_bias_initial_unc[1],
                noiseParameters->gyro_bias_initial_unc[2]);
      p_fprintf(noiseParameters->acc_bias_initial_unc[0],
                noiseParameters->acc_bias_initial_unc[1],
                noiseParameters->acc_bias_initial_unc[2]);
      r_fprintf(noiseParameters->image_noise);
      t_fprintf(noiseParameters->inv_depth_initial_unc);
      b_fprintf();

      // #coder
      // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
      //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
      if (debug_level >= 2.0) {
        for (i26 = 0; i26 < 15; i26++) {
          cv14[i26] = cv15[i26];
        }

        ROS_INFO(cv14);
      }

      v_fprintf(b_VIOParameters->num_anchors);
      x_fprintf(b_VIOParameters->num_points_per_anchor);
      ab_fprintf(b_VIOParameters->max_ekf_iterations);
      if (b_VIOParameters->delayed_initialization) {
        s_size[0] = 1;
        s_size[1] = 4;
        for (i26 = 0; i26 < 4; i26++) {
          s_data[i26] = cv16[i26];
        }
      } else {
        s_size[0] = 1;
        s_size[1] = 5;
        for (i26 = 0; i26 < 5; i26++) {
          s_data[i26] = cv17[i26];
        }
      }

      cb_fprintf(s_data, s_size);
      if (b_VIOParameters->fixed_feature) {
        s_size[0] = 1;
        s_size[1] = 4;
        for (i26 = 0; i26 < 4; i26++) {
          s_data[i26] = cv16[i26];
        }
      } else {
        s_size[0] = 1;
        s_size[1] = 5;
        for (i26 = 0; i26 < 5; i26++) {
          s_data[i26] = cv17[i26];
        }
      }

      eb_fprintf(s_data, s_size);
      if (b_VIOParameters->mono) {
        s_size[0] = 1;
        s_size[1] = 4;
        for (i26 = 0; i26 < 4; i26++) {
          s_data[i26] = cv16[i26];
        }
      } else {
        s_size[0] = 1;
        s_size[1] = 5;
        for (i26 = 0; i26 < 5; i26++) {
          s_data[i26] = cv17[i26];
        }
      }

      gb_fprintf(s_data, s_size);
      if (b_VIOParameters->RANSAC) {
        s_size[0] = 1;
        s_size[1] = 4;
        for (i26 = 0; i26 < 4; i26++) {
          s_data[i26] = cv16[i26];
        }
      } else {
        s_size[0] = 1;
        s_size[1] = 5;
        for (i26 = 0; i26 < 5; i26++) {
          s_data[i26] = cv17[i26];
        }
      }

      ib_fprintf(s_data, s_size);
      b_fprintf();
      initialized_not_empty = true;
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
      for (j = 0; j < 80; j++) {
        b_z_all_l[j] = z_all_l[j];
        b_z_all_r[j] = z_all_r[j];
      }

      SLAM_upd(P, &xt, cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
               noiseParameters->image_noise,
               noiseParameters->inv_depth_initial_unc, *b_VIOParameters, h_u,
               map, delayedStatus);
    }

    guard1 = true;
  }

  if (guard1) {
    i26 = h_u_out->size[0];
    h_u_out->size[0] = h_u->size[0];
    emxEnsureCapacity((emxArray__common *)h_u_out, i26, (int)sizeof(double));
    j = h_u->size[0];
    for (i26 = 0; i26 < j; i26++) {
      h_u_out->data[i26] = h_u->data[i26];
    }

    i26 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = map->size[1];
    emxEnsureCapacity((emxArray__common *)map_out, i26, (int)sizeof(double));
    j = map->size[0] * map->size[1];
    for (i26 = 0; i26 < j; i26++) {
      map_out->data[i26] = map->data[i26];
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
    for (i26 = 0; i26 < 3; i26++) {
      B = 0.0;
      for (j = 0; j < 3; j++) {
        B += R_cw_init[j + 3 * i26] * xt.robot_state.pos[j];
      }

      t0_pos[i26] = xt.origin.pos[i26] + B;
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
    for (i26 = 0; i26 < 3; i26++) {
      for (j = 0; j < 3; j++) {
        dv1[i26 + 3 * j] = 0.0;
        for (n = 0; n < 3; n++) {
          dv1[i26 + 3 * j] += dv2[i26 + 3 * n] * R_cw_init[n + 3 * j];
        }
      }

      t0_vel[i26] = 0.0;
      for (j = 0; j < 3; j++) {
        t0_vel[i26] += R_cw_init[j + 3 * i26] * xt.robot_state.pos[j];
      }
    }

    QuatFromRotJ(dv1, t0_att);
    for (j = 0; j < 3; j++) {
      xt_out->pos[j] = t0_pos[j];
    }

    for (j = 0; j < 4; j++) {
      xt_out->att[j] = t0_att[j];
    }

    for (j = 0; j < 3; j++) {
      xt_out->vel[j] = t0_vel[j];
    }

    for (j = 0; j < 3; j++) {
      xt_out->IMU.gyro_bias[j] = xt.robot_state.IMU.gyro_bias[j];
    }

    for (j = 0; j < 3; j++) {
      xt_out->IMU.acc_bias[j] = xt.robot_state.IMU.acc_bias[j];
    }

    for (j = 0; j < 3; j++) {
      xt_out->IMU.pos[j] = xt.robot_state.IMU.pos[j];
    }

    for (j = 0; j < 4; j++) {
      xt_out->IMU.att[j] = xt.robot_state.IMU.att[j];
    }

    getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, x);
    i26 = anchor_poses_out->size[0] * anchor_poses_out->size[1];
    anchor_poses_out->size[0] = x->size[0];
    anchor_poses_out->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)anchor_poses_out, i26, (int)sizeof
                      (AnchorPose));
    n = x->size[0] * x->size[1];
    for (j = 0; j < n; j++) {
      for (i26 = 0; i26 < 3; i26++) {
        anchor_poses_out->data[j].pos[i26] = x->data[j].pos[i26];
      }

      for (i26 = 0; i26 < 4; i26++) {
        anchor_poses_out->data[j].att[i26] = x->data[j].att[i26];
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
