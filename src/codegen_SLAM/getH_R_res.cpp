//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const double xt_robot_state_pos[3]
//                const double xt_robot_state_att[4]
//                const emxArray_b_struct_T *xt_anchor_states
//                const double z_all_l[80]
//                const boolean_T b_status[40]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double noiseParameters_image_noise
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_real_T *R
// Return Type  : void
//
void getH_R_res(const double xt_robot_state_pos[3], const double
                xt_robot_state_att[4], const emxArray_b_struct_T
                *xt_anchor_states, const double z_all_l[80], const boolean_T
                b_status[40], const double cameraparams_FocalLength[2], const
                double cameraparams_PrincipalPoint[2], double
                noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T
                *H, emxArray_real_T *R)
{
  double fx;
  double fy;
  int n;
  int j;
  double R_cw[9];
  int br;
  emxArray_real_T *b_h_u;
  double res_idx;
  int anchorIdx;
  emxArray_real_T *H_robot;
  emxArray_real_T *H_map;
  emxArray_real_T *C;
  emxArray_real_T *b;
  double anchorRot[9];
  int featureIdx;
  double varargin_2;
  double h_c_n_l[2];
  double b_res_idx;
  double h_u_To_h_ci_l[6];
  double h_ci_l_To_R_cw[9];
  int ar;
  int ib;
  double y[3];
  double b_xt_anchor_states[3];
  double h_ci_l_To_rho[3];
  double b_R_cw[9];
  double dv0[9];
  double c_R_cw[9];
  int iv0[2];
  int cr;
  unsigned int unnamed_idx_1;
  int ic;
  int ia;
  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  n = 0;
  for (j = 0; j < 40; j++) {
    if (b_status[j]) {
      n++;
    }
  }

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  br = H->size[0] * H->size[1];
  H->size[0] = (int)((double)n * 2.0);
  H->size[1] = (int)(numStates + numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H, br, (int)sizeof(double));
  j = (int)((double)n * 2.0) * (int)(numStates + numAnchors * (6.0 +
    numPointsPerAnchor));
  for (br = 0; br < j; br++) {
    H->data[br] = 0.0;
  }

  br = r->size[0];
  r->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)r, br, (int)sizeof(double));
  j = (int)((double)n * 2.0);
  for (br = 0; br < j; br++) {
    r->data[br] = 0.0;
  }

  b_emxInit_real_T(&b_h_u, 1);
  br = b_h_u->size[0];
  b_h_u->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)b_h_u, br, (int)sizeof(double));
  j = (int)((double)n * 2.0);
  for (br = 0; br < j; br++) {
    b_h_u->data[br] = 0.0;
  }

  res_idx = 1.0;
  anchorIdx = 0;
  emxInit_real_T(&H_robot, 2);
  emxInit_real_T(&H_map, 2);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&b, 2);
  while (anchorIdx <= (int)numAnchors - 1) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((xt_anchor_states->data[anchorIdx].att[0] *
                     xt_anchor_states->data[anchorIdx].att[0] -
                     xt_anchor_states->data[anchorIdx].att[1] *
                     xt_anchor_states->data[anchorIdx].att[1]) -
                    xt_anchor_states->data[anchorIdx].att[2] *
                    xt_anchor_states->data[anchorIdx].att[2]) +
      xt_anchor_states->data[anchorIdx].att[3] * xt_anchor_states->
      data[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[1] +
                          xt_anchor_states->data[anchorIdx].att[2] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[2] -
                          xt_anchor_states->data[anchorIdx].att[1] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[1] -
                          xt_anchor_states->data[anchorIdx].att[2] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states->data[anchorIdx].att[0] *
                       xt_anchor_states->data[anchorIdx].att[0]) +
                     xt_anchor_states->data[anchorIdx].att[1] *
                     xt_anchor_states->data[anchorIdx].att[1]) -
                    xt_anchor_states->data[anchorIdx].att[2] *
                    xt_anchor_states->data[anchorIdx].att[2]) +
      xt_anchor_states->data[anchorIdx].att[3] * xt_anchor_states->
      data[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (xt_anchor_states->data[anchorIdx].att[1] *
                          xt_anchor_states->data[anchorIdx].att[2] +
                          xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[2] +
                          xt_anchor_states->data[anchorIdx].att[1] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (xt_anchor_states->data[anchorIdx].att[1] *
                          xt_anchor_states->data[anchorIdx].att[2] -
                          xt_anchor_states->data[anchorIdx].att[0] *
                          xt_anchor_states->data[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states->data[anchorIdx].att[0] *
                       xt_anchor_states->data[anchorIdx].att[0]) -
                     xt_anchor_states->data[anchorIdx].att[1] *
                     xt_anchor_states->data[anchorIdx].att[1]) +
                    xt_anchor_states->data[anchorIdx].att[2] *
                    xt_anchor_states->data[anchorIdx].att[2]) +
      xt_anchor_states->data[anchorIdx].att[3] * xt_anchor_states->
      data[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < (int)numPointsPerAnchor; featureIdx++) {
      if ((xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
           status != 0.0) && b_status[(int)xt_anchor_states->data[anchorIdx].
          feature_states->data[featureIdx].status_idx - 1]) {
        // predictMeasurementLeft Predict the measurement of a feature given in the left 
        // camera frame
        //    Get the normalized pixel coordinates where a feature given in the left camera 
        //    frame
        varargin_2 = xt_anchor_states->data[anchorIdx].feature_states->
          data[featureIdx].scaled_map_point[2];
        for (br = 0; br < 2; br++) {
          h_c_n_l[br] = xt_anchor_states->data[anchorIdx].feature_states->
            data[featureIdx].scaled_map_point[br] / varargin_2;
        }

        //  normalized feature in camera frame
        b_res_idx = (res_idx - 1.0) * 2.0;
        b_h_u->data[(int)(b_res_idx + 1.0) - 1] = h_c_n_l[0] *
          cameraparams_FocalLength[0] + cameraparams_PrincipalPoint[0];
        b_h_u->data[(int)(b_res_idx + 2.0) - 1] = h_c_n_l[1] *
          cameraparams_FocalLength[1] + cameraparams_PrincipalPoint[1];
        b_res_idx = (res_idx - 1.0) * 2.0;
        varargin_2 = (xt_anchor_states->data[anchorIdx].feature_states->
                      data[featureIdx].status_idx - 1.0) * 2.0;
        for (br = 0; br < 2; br++) {
          r->data[(int)(b_res_idx + (1.0 + (double)br)) - 1] = z_all_l[(int)
            (varargin_2 + (1.0 + (double)br)) - 1];
        }

        h_u_To_h_ci_l[0] = fx / xt_anchor_states->data[anchorIdx].
          feature_states->data[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[2] = 0.0;
        h_u_To_h_ci_l[4] = -fx * xt_anchor_states->data[anchorIdx].
          feature_states->data[featureIdx].scaled_map_point[0] /
          (xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
           scaled_map_point[2] * xt_anchor_states->data[anchorIdx].
           feature_states->data[featureIdx].scaled_map_point[2]);
        h_u_To_h_ci_l[1] = 0.0;
        h_u_To_h_ci_l[3] = fy / xt_anchor_states->data[anchorIdx].
          feature_states->data[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[5] = -fy * xt_anchor_states->data[anchorIdx].
          feature_states->data[featureIdx].scaled_map_point[1] /
          (xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
           scaled_map_point[2] * xt_anchor_states->data[anchorIdx].
           feature_states->data[featureIdx].scaled_map_point[2]);
        h_ci_l_To_R_cw[0] = 0.0;
        h_ci_l_To_R_cw[3] = -xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[6] = xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[1] = xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[4] = 0.0;
        h_ci_l_To_R_cw[7] = -xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[2] = -xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[5] = xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[8] = 0.0;
        if ((xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
             status == 2.0) || (xt_anchor_states->data[anchorIdx].
                                feature_states->data[featureIdx].inverse_depth <
             0.0)) {
          //  delayed initialization or feature behind anchor
          ar = (int)(numStates - 6.0);
          br = H_robot->size[0] * H_robot->size[1];
          H_robot->size[0] = 3;
          H_robot->size[1] = 6 + (int)(numStates - 6.0);
          emxEnsureCapacity((emxArray__common *)H_robot, br, (int)sizeof(double));
          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * br] = 0.0;
            }
          }

          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * (br + 3)] = 0.0 *
                h_ci_l_To_R_cw[ib + 3 * br];
            }
          }

          for (br = 0; br < ar; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * (br + 6)] = 0.0;
            }
          }
        } else {
          varargin_2 = -xt_anchor_states->data[anchorIdx].feature_states->
            data[featureIdx].inverse_depth;
          ar = (int)(numStates - 6.0);
          br = H_robot->size[0] * H_robot->size[1];
          H_robot->size[0] = 3;
          H_robot->size[1] = 6 + (int)(numStates - 6.0);
          emxEnsureCapacity((emxArray__common *)H_robot, br, (int)sizeof(double));
          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * br] = varargin_2 * R_cw[ib +
                3 * br];
            }
          }

          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * (br + 3)] =
                h_ci_l_To_R_cw[ib + 3 * br];
            }
          }

          for (br = 0; br < ar; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_robot->data[ib + H_robot->size[0] * (br + 6)] = 0.0;
            }
          }
        }

        //             %% anchor state derivatives
        for (br = 0; br < 3; br++) {
          y[br] = 0.0;
          for (ib = 0; ib < 3; ib++) {
            varargin_2 = y[br] + anchorRot[ib + 3 * br] * xt_anchor_states->
              data[anchorIdx].feature_states->data[featureIdx].m[ib];
            y[br] = varargin_2;
          }
        }

        for (br = 0; br < 3; br++) {
          b_xt_anchor_states[br] = xt_anchor_states->data[anchorIdx].pos[br] -
            xt_robot_state_pos[br];
        }

        for (br = 0; br < 3; br++) {
          h_ci_l_To_rho[br] = 0.0;
          for (ib = 0; ib < 3; ib++) {
            h_ci_l_To_rho[br] += R_cw[br + 3 * ib] * b_xt_anchor_states[ib];
          }
        }

        if ((xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
             status == 2.0) || (xt_anchor_states->data[anchorIdx].
                                feature_states->data[featureIdx].inverse_depth <
             0.0)) {
          //  delayed initialization or feature behind anchor
          ar = (int)(numPointsPerAnchor - (1.0 + (double)featureIdx));
          br = H_map->size[0] * H_map->size[1];
          H_map->size[0] = 3;
          H_map->size[1] = ((int)((6.0 + (1.0 + (double)featureIdx)) - 1.0) + ar)
            + 1;
          emxEnsureCapacity((emxArray__common *)H_map, br, (int)sizeof(double));
          j = (int)((6.0 + (1.0 + (double)featureIdx)) - 1.0);
          for (br = 0; br < j; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * br] = 0.0;
            }
          }

          for (br = 0; br < 3; br++) {
            H_map->data[br + H_map->size[0] * (int)((6.0 + (1.0 + (double)
              featureIdx)) - 1.0)] = h_ci_l_To_rho[br];
          }

          for (br = 0; br < ar; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * ((br + (int)((6.0 + (1.0 +
                (double)featureIdx)) - 1.0)) + 1)] = 0.0;
            }
          }
        } else {
          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              b_R_cw[ib + 3 * br] = -R_cw[ib + 3 * br];
            }
          }

          dv0[0] = 0.0;
          dv0[3] = -y[2];
          dv0[6] = y[1];
          dv0[1] = y[2];
          dv0[4] = 0.0;
          dv0[7] = -y[0];
          dv0[2] = -y[1];
          dv0[5] = y[0];
          dv0[8] = 0.0;
          ar = (int)(numPointsPerAnchor - (1.0 + (double)featureIdx));
          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              c_R_cw[br + 3 * ib] = 0.0;
              for (j = 0; j < 3; j++) {
                c_R_cw[br + 3 * ib] += b_R_cw[br + 3 * j] * dv0[j + 3 * ib];
              }
            }
          }

          br = H_map->size[0] * H_map->size[1];
          H_map->size[0] = 3;
          H_map->size[1] = ((int)((1.0 + (double)featureIdx) - 1.0) + ar) + 7;
          emxEnsureCapacity((emxArray__common *)H_map, br, (int)sizeof(double));
          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * br] = xt_anchor_states->
                data[anchorIdx].feature_states->data[featureIdx].inverse_depth *
                R_cw[ib + 3 * br];
            }
          }

          for (br = 0; br < 3; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * (br + 3)] = c_R_cw[ib + 3 * br];
            }
          }

          j = (int)((1.0 + (double)featureIdx) - 1.0);
          for (br = 0; br < j; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * (br + 6)] = 0.0;
            }
          }

          for (br = 0; br < 3; br++) {
            H_map->data[br + H_map->size[0] * (6 + (int)((1.0 + (double)
              featureIdx) - 1.0))] = h_ci_l_To_rho[br];
          }

          for (br = 0; br < ar; br++) {
            for (ib = 0; ib < 3; ib++) {
              H_map->data[ib + H_map->size[0] * ((br + (int)((1.0 + (double)
                featureIdx) - 1.0)) + 7)] = 0.0;
            }
          }
        }

        b_res_idx = (res_idx - 1.0) * 2.0;
        for (br = 0; br < 2; br++) {
          iv0[br] = (int)(b_res_idx + (1.0 + (double)br)) - 1;
        }

        ar = (int)(((1.0 + (double)anchorIdx) - 1.0) * (6.0 + numPointsPerAnchor));
        cr = (int)((numAnchors - (1.0 + (double)anchorIdx)) * (6.0 +
                    numPointsPerAnchor));
        br = b->size[0] * b->size[1];
        b->size[0] = 3;
        b->size[1] = ((H_robot->size[1] + ar) + H_map->size[1]) + cr;
        emxEnsureCapacity((emxArray__common *)b, br, (int)sizeof(double));
        j = H_robot->size[1];
        for (br = 0; br < j; br++) {
          for (ib = 0; ib < 3; ib++) {
            b->data[ib + b->size[0] * br] = H_robot->data[ib + H_robot->size[0] *
              br];
          }
        }

        for (br = 0; br < ar; br++) {
          for (ib = 0; ib < 3; ib++) {
            b->data[ib + b->size[0] * (br + H_robot->size[1])] = 0.0;
          }
        }

        j = H_map->size[1];
        for (br = 0; br < j; br++) {
          for (ib = 0; ib < 3; ib++) {
            b->data[ib + b->size[0] * ((br + H_robot->size[1]) + ar)] =
              H_map->data[ib + H_map->size[0] * br];
          }
        }

        for (br = 0; br < cr; br++) {
          for (ib = 0; ib < 3; ib++) {
            b->data[ib + b->size[0] * (((br + H_robot->size[1]) + ar) +
              H_map->size[1])] = 0.0;
          }
        }

        unnamed_idx_1 = (unsigned int)b->size[1];
        br = C->size[0] * C->size[1];
        C->size[0] = 2;
        emxEnsureCapacity((emxArray__common *)C, br, (int)sizeof(double));
        br = C->size[0] * C->size[1];
        C->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)C, br, (int)sizeof(double));
        j = (int)unnamed_idx_1 << 1;
        for (br = 0; br < j; br++) {
          C->data[br] = 0.0;
        }

        j = (b->size[1] - 1) << 1;
        for (cr = 0; cr <= j; cr += 2) {
          for (ic = cr; ic + 1 <= cr + 2; ic++) {
            C->data[ic] = 0.0;
          }
        }

        br = 0;
        for (cr = 0; cr <= j; cr += 2) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 3; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              for (ic = cr; ic + 1 <= cr + 2; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * h_u_To_h_ci_l[ia - 1];
              }
            }

            ar += 2;
          }

          br += 3;
        }

        j = C->size[1];
        for (br = 0; br < j; br++) {
          for (ib = 0; ib < 2; ib++) {
            H->data[iv0[ib] + H->size[0] * br] = C->data[ib + C->size[0] * br];
          }
        }

        res_idx++;
      }
    }

    anchorIdx++;
  }

  emxFree_real_T(&b);
  emxFree_real_T(&C);
  emxFree_real_T(&H_map);
  emxFree_real_T(&H_robot);
  br = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, br, (int)sizeof(double));
  j = r->size[0];
  for (br = 0; br < j; br++) {
    r->data[br] -= b_h_u->data[br];
  }

  emxFree_real_T(&b_h_u);
  varargin_2 = (double)n * 2.0;
  br = R->size[0] * R->size[1];
  R->size[0] = (int)varargin_2;
  emxEnsureCapacity((emxArray__common *)R, br, (int)sizeof(double));
  br = R->size[0] * R->size[1];
  R->size[1] = (int)varargin_2;
  emxEnsureCapacity((emxArray__common *)R, br, (int)sizeof(double));
  j = (int)varargin_2 * (int)varargin_2;
  for (br = 0; br < j; br++) {
    R->data[br] = 0.0;
  }

  for (j = 0; j + 1 <= (int)varargin_2; j++) {
    R->data[j + R->size[0] * j] = noiseParameters_image_noise;
  }
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
