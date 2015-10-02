//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>

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
//                const double noiseParameters_image_noise[2]
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_real_T *R
// Return Type  : void
//
void getH_R_res(const double xt_robot_state_pos[3], const double
                xt_robot_state_att[4], const emxArray_b_struct_T
                *xt_anchor_states, const double z_all_l[80], const boolean_T
                b_status[40], const double cameraparams_FocalLength[2], const
                double cameraparams_PrincipalPoint[2], const double
                noiseParameters_image_noise[2], emxArray_real_T *r,
                emxArray_real_T *H, emxArray_real_T *R)
{
  double fx;
  double fy;
  int n;
  int kidx;
  double R_cw[9];
  int ar;
  emxArray_real_T *b_h_u;
  double res_idx;
  int anchorIdx;
  emxArray_real_T *H_robot;
  emxArray_real_T *H_map;
  emxArray_real_T *C;
  emxArray_real_T *b;
  double anchorRot[9];
  int featureIdx;
  double b_xt_anchor_states;
  double h_c_n_l[2];
  double b_res_idx;
  double h_u_To_h_ci_l[6];
  double h_ci_l_To_R_cw[9];
  int i2;
  int br;
  double y[3];
  double c_xt_anchor_states[3];
  double h_ci_l_To_rho[3];
  double b_R_cw[9];
  double dv0[9];
  double c_R_cw[9];
  int iv0[2];
  int b_j1;
  double v[2];
  int ic;
  int ia;
  emxArray_real_T *I;
  double d[4];
  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  n = 0;
  for (kidx = 0; kidx < 40; kidx++) {
    if (b_status[kidx]) {
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
  ar = H->size[0] * H->size[1];
  H->size[0] = (int)((double)n * 2.0);
  H->size[1] = (int)(numStates + numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
  kidx = (int)((double)n * 2.0) * (int)(numStates + numAnchors * (6.0 +
    numPointsPerAnchor));
  for (ar = 0; ar < kidx; ar++) {
    H->data[ar] = 0.0;
  }

  ar = r->size[0];
  r->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)r, ar, (int)sizeof(double));
  kidx = (int)((double)n * 2.0);
  for (ar = 0; ar < kidx; ar++) {
    r->data[ar] = 0.0;
  }

  b_emxInit_real_T(&b_h_u, 1);
  ar = b_h_u->size[0];
  b_h_u->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)b_h_u, ar, (int)sizeof(double));
  kidx = (int)((double)n * 2.0);
  for (ar = 0; ar < kidx; ar++) {
    b_h_u->data[ar] = 0.0;
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
        b_xt_anchor_states = xt_anchor_states->data[anchorIdx]
          .feature_states->data[featureIdx].scaled_map_point[2];
        for (ar = 0; ar < 2; ar++) {
          h_c_n_l[ar] = xt_anchor_states->data[anchorIdx].feature_states->
            data[featureIdx].scaled_map_point[ar] / b_xt_anchor_states;
        }

        //  normalized feature in camera frame
        b_res_idx = (res_idx - 1.0) * 2.0;
        b_h_u->data[(int)(b_res_idx + 1.0) - 1] = h_c_n_l[0] *
          cameraparams_FocalLength[0] + cameraparams_PrincipalPoint[0];
        b_h_u->data[(int)(b_res_idx + 2.0) - 1] = h_c_n_l[1] *
          cameraparams_FocalLength[1] + cameraparams_PrincipalPoint[1];
        b_res_idx = (res_idx - 1.0) * 2.0;
        b_xt_anchor_states = (xt_anchor_states->data[anchorIdx]
                              .feature_states->data[featureIdx].status_idx - 1.0)
          * 2.0;
        for (ar = 0; ar < 2; ar++) {
          r->data[(int)(b_res_idx + (1.0 + (double)ar)) - 1] = z_all_l[(int)
            (b_xt_anchor_states + (1.0 + (double)ar)) - 1];
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
          i2 = (int)(numStates - 6.0);
          ar = H_robot->size[0] * H_robot->size[1];
          H_robot->size[0] = 3;
          H_robot->size[1] = 6 + (int)(numStates - 6.0);
          emxEnsureCapacity((emxArray__common *)H_robot, ar, (int)sizeof(double));
          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * ar] = 0.0;
            }
          }

          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * (ar + 3)] = 0.0 *
                h_ci_l_To_R_cw[br + 3 * ar];
            }
          }

          for (ar = 0; ar < i2; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * (ar + 6)] = 0.0;
            }
          }
        } else {
          b_xt_anchor_states = -xt_anchor_states->data[anchorIdx].
            feature_states->data[featureIdx].inverse_depth;
          i2 = (int)(numStates - 6.0);
          ar = H_robot->size[0] * H_robot->size[1];
          H_robot->size[0] = 3;
          H_robot->size[1] = 6 + (int)(numStates - 6.0);
          emxEnsureCapacity((emxArray__common *)H_robot, ar, (int)sizeof(double));
          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * ar] = b_xt_anchor_states *
                R_cw[br + 3 * ar];
            }
          }

          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * (ar + 3)] =
                h_ci_l_To_R_cw[br + 3 * ar];
            }
          }

          for (ar = 0; ar < i2; ar++) {
            for (br = 0; br < 3; br++) {
              H_robot->data[br + H_robot->size[0] * (ar + 6)] = 0.0;
            }
          }
        }

        //             %% anchor state derivatives
        for (ar = 0; ar < 3; ar++) {
          y[ar] = 0.0;
          for (br = 0; br < 3; br++) {
            b_xt_anchor_states = y[ar] + anchorRot[br + 3 * ar] *
              xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx]
              .m[br];
            y[ar] = b_xt_anchor_states;
          }
        }

        for (ar = 0; ar < 3; ar++) {
          c_xt_anchor_states[ar] = xt_anchor_states->data[anchorIdx].pos[ar] -
            xt_robot_state_pos[ar];
        }

        for (ar = 0; ar < 3; ar++) {
          h_ci_l_To_rho[ar] = 0.0;
          for (br = 0; br < 3; br++) {
            h_ci_l_To_rho[ar] += R_cw[ar + 3 * br] * c_xt_anchor_states[br];
          }
        }

        if ((xt_anchor_states->data[anchorIdx].feature_states->data[featureIdx].
             status == 2.0) || (xt_anchor_states->data[anchorIdx].
                                feature_states->data[featureIdx].inverse_depth <
             0.0)) {
          //  delayed initialization or feature behind anchor
          i2 = (int)(numPointsPerAnchor - (1.0 + (double)featureIdx));
          ar = H_map->size[0] * H_map->size[1];
          H_map->size[0] = 3;
          H_map->size[1] = ((int)((6.0 + (1.0 + (double)featureIdx)) - 1.0) + i2)
            + 1;
          emxEnsureCapacity((emxArray__common *)H_map, ar, (int)sizeof(double));
          kidx = (int)((6.0 + (1.0 + (double)featureIdx)) - 1.0);
          for (ar = 0; ar < kidx; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * ar] = 0.0;
            }
          }

          for (ar = 0; ar < 3; ar++) {
            H_map->data[ar + H_map->size[0] * (int)((6.0 + (1.0 + (double)
              featureIdx)) - 1.0)] = h_ci_l_To_rho[ar];
          }

          for (ar = 0; ar < i2; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * ((ar + (int)((6.0 + (1.0 +
                (double)featureIdx)) - 1.0)) + 1)] = 0.0;
            }
          }
        } else {
          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              b_R_cw[br + 3 * ar] = -R_cw[br + 3 * ar];
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
          i2 = (int)(numPointsPerAnchor - (1.0 + (double)featureIdx));
          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              c_R_cw[ar + 3 * br] = 0.0;
              for (kidx = 0; kidx < 3; kidx++) {
                c_R_cw[ar + 3 * br] += b_R_cw[ar + 3 * kidx] * dv0[kidx + 3 * br];
              }
            }
          }

          ar = H_map->size[0] * H_map->size[1];
          H_map->size[0] = 3;
          H_map->size[1] = ((int)((1.0 + (double)featureIdx) - 1.0) + i2) + 7;
          emxEnsureCapacity((emxArray__common *)H_map, ar, (int)sizeof(double));
          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * ar] = xt_anchor_states->
                data[anchorIdx].feature_states->data[featureIdx].inverse_depth *
                R_cw[br + 3 * ar];
            }
          }

          for (ar = 0; ar < 3; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * (ar + 3)] = c_R_cw[br + 3 * ar];
            }
          }

          kidx = (int)((1.0 + (double)featureIdx) - 1.0);
          for (ar = 0; ar < kidx; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * (ar + 6)] = 0.0;
            }
          }

          for (ar = 0; ar < 3; ar++) {
            H_map->data[ar + H_map->size[0] * (6 + (int)((1.0 + (double)
              featureIdx) - 1.0))] = h_ci_l_To_rho[ar];
          }

          for (ar = 0; ar < i2; ar++) {
            for (br = 0; br < 3; br++) {
              H_map->data[br + H_map->size[0] * ((ar + (int)((1.0 + (double)
                featureIdx) - 1.0)) + 7)] = 0.0;
            }
          }
        }

        b_res_idx = (res_idx - 1.0) * 2.0;
        for (ar = 0; ar < 2; ar++) {
          iv0[ar] = (int)(b_res_idx + (1.0 + (double)ar)) - 1;
        }

        i2 = (int)(((1.0 + (double)anchorIdx) - 1.0) * (6.0 + numPointsPerAnchor));
        b_j1 = (int)((numAnchors - (1.0 + (double)anchorIdx)) * (6.0 +
          numPointsPerAnchor));
        ar = b->size[0] * b->size[1];
        b->size[0] = 3;
        b->size[1] = ((H_robot->size[1] + i2) + H_map->size[1]) + b_j1;
        emxEnsureCapacity((emxArray__common *)b, ar, (int)sizeof(double));
        kidx = H_robot->size[1];
        for (ar = 0; ar < kidx; ar++) {
          for (br = 0; br < 3; br++) {
            b->data[br + b->size[0] * ar] = H_robot->data[br + H_robot->size[0] *
              ar];
          }
        }

        for (ar = 0; ar < i2; ar++) {
          for (br = 0; br < 3; br++) {
            b->data[br + b->size[0] * (ar + H_robot->size[1])] = 0.0;
          }
        }

        kidx = H_map->size[1];
        for (ar = 0; ar < kidx; ar++) {
          for (br = 0; br < 3; br++) {
            b->data[br + b->size[0] * ((ar + H_robot->size[1]) + i2)] =
              H_map->data[br + H_map->size[0] * ar];
          }
        }

        for (ar = 0; ar < b_j1; ar++) {
          for (br = 0; br < 3; br++) {
            b->data[br + b->size[0] * (((ar + H_robot->size[1]) + i2) +
              H_map->size[1])] = 0.0;
          }
        }

        v[1] = b->size[1];
        ar = C->size[0] * C->size[1];
        C->size[0] = 2;
        emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
        ar = C->size[0] * C->size[1];
        C->size[1] = (int)v[1];
        emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
        kidx = (int)v[1] << 1;
        for (ar = 0; ar < kidx; ar++) {
          C->data[ar] = 0.0;
        }

        kidx = (b->size[1] - 1) << 1;
        for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
          for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
            C->data[ic] = 0.0;
          }
        }

        br = 0;
        for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
          ar = 0;
          for (i2 = br; i2 + 1 <= br + 3; i2++) {
            if (b->data[i2] != 0.0) {
              ia = ar;
              for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
                ia++;
                C->data[ic] += b->data[i2] * h_u_To_h_ci_l[ia - 1];
              }
            }

            ar += 2;
          }

          br += 3;
        }

        kidx = C->size[1];
        for (ar = 0; ar < kidx; ar++) {
          for (br = 0; br < 2; br++) {
            H->data[iv0[br] + H->size[0] * ar] = C->data[br + C->size[0] * ar];
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
  ar = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, ar, (int)sizeof(double));
  kidx = r->size[0];
  for (ar = 0; ar < kidx; ar++) {
    r->data[ar] -= b_h_u->data[ar];
  }

  emxFree_real_T(&b_h_u);
  emxInit_real_T(&I, 2);
  ar = I->size[0] * I->size[1];
  I->size[0] = n;
  I->size[1] = n;
  emxEnsureCapacity((emxArray__common *)I, ar, (int)sizeof(double));
  kidx = n * n;
  for (ar = 0; ar < kidx; ar++) {
    I->data[ar] = 0.0;
  }

  if (n > 0) {
    for (kidx = 0; kidx + 1 <= n; kidx++) {
      I->data[kidx + I->size[0] * kidx] = 1.0;
    }
  }

  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (kidx = 0; kidx < 2; kidx++) {
    d[kidx + (kidx << 1)] = v[kidx];
  }

  kidx = I->size[0] << 1;
  i2 = I->size[1] << 1;
  ar = R->size[0] * R->size[1];
  R->size[0] = kidx;
  R->size[1] = i2;
  emxEnsureCapacity((emxArray__common *)R, ar, (int)sizeof(double));
  kidx = -1;
  for (b_j1 = 1; b_j1 <= I->size[1]; b_j1++) {
    for (br = 0; br < 2; br++) {
      for (ar = 1; ar <= I->size[0]; ar++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          R->data[kidx] = I->data[(ar + I->size[0] * (b_j1 - 1)) - 1] * d[i2 +
            (br << 1)];
        }
      }
    }
  }

  emxFree_real_T(&I);

  //  figure(3); plot(r, '.-');
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
