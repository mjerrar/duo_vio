//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 20:07:15
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "QuatFromRotJ.h"
#include "predictMeasurement_stereo.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_left.h"
#include "ros_error.h"
#include "any.h"
#include "eye.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//    Does not take into account the derivative with respect to anchor states
//    (static map)
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
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const emxArray_real_T *map
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u_data[]
//                int h_u_size[1]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void b_getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                  stateSize, const double z_all_l[32], const double
                  indMeas_data[], const int indMeas_size[1], const
                  emxArray_real_T *map, const double
                  c_cameraparams_CameraParameters[3], const double
                  d_cameraparams_CameraParameters[2], const double
                  e_cameraparams_CameraParameters[2], const emxArray_real_T
                  *anchorIdx, const emxArray_real_T *featureAnchorIdx, const
                  emxArray_real_T *b_m_vect, const double
                  noiseParameters_image_noise[2], double
                  c_noiseParameters_orientation_n, double
                  noiseParameters_pressure_noise, double
                  noiseParameters_ext_pos_noise, double
                  noiseParameters_ext_att_noise, const VIOMeasurements
                  *measurements, double b_height_offset_pressure, const
                  VIOParameters b_VIOParameters, double r_data[], int r_size[1],
                  emxArray_real_T *H, double h_u_data[], int h_u_size[1], double
                  R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double numPointsPerAnchor;
  double numStatesPerAnchor;
  double numErrorStatesPerAnchor;
  double R_cw[9];
  double fx_l;
  double fy_l;
  double k1_l;
  double k2_l;
  double k3_l;
  int ar;
  int b_j1;
  emxArray_real_T *H_xc;
  double z_data[32];
  int k;
  emxArray_real_T *H_orientation;
  emxArray_real_T *H_pressure;
  emxArray_int32_T *r2;
  emxArray_real_T *C;
  boolean_T bv1[3];
  double b_map[3];
  double r_orientation[3];
  int i2;
  double h_cin_l[3];
  int kidx;
  double dv3[2];
  double indMeas;
  signed char b_k;
  signed char iv2[2];
  double d[4];
  double b_fx_l[4];
  double c_fx_l[4];
  double dv4[6];
  double d_fx_l[6];
  double b_R_cw[36];
  double e_fx_l[24];
  double b_stateSize;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double anchorRot[9];
  double c_xt;
  double d2;
  double y[3];
  double c;
  double dv5[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int br;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double b_anchorIdx;
  double f_fx_l[4];
  double dv6[6];
  double b_y[6];
  double v[2];
  int ic;
  int ia;
  double r_pressure;
  double r_ext_pose[6];
  emxArray_real_T *H_ext_pose;
  emxArray_real_T *r3;
  double A_data[256];
  double R_vision_data[1024];
  int R_vision_size_idx_0;
  int residual_size;
  int orientation_idx;
  int pressure_idx;
  int ext_pose_idx;
  double b_measurements[9];
  signed char I[9];
  double c_measurements[9];
  int tmp_data[32];
  signed char iv3[3];
  signed char iv4[6];
  emxInit_real_T(&H_xm, 2);
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  numStatesPerAnchor = 7.0 + b_VIOParameters.num_points_per_anchor;
  numErrorStatesPerAnchor = 6.0 + b_VIOParameters.num_points_per_anchor;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  fx_l = d_cameraparams_CameraParameters[0];
  fy_l = d_cameraparams_CameraParameters[1];

  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  k1_l = c_cameraparams_CameraParameters[0];
  k2_l = c_cameraparams_CameraParameters[1];
  k3_l = c_cameraparams_CameraParameters[2];
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  b_j1 = (indMeas_size[0] << 1) * (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_xm->data[ar] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  b_j1 = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ar = 0; ar < b_j1; ar++) {
    H_xc->data[ar] = 0.0;
  }

  b_j1 = indMeas_size[0] << 1;
  for (ar = 0; ar < b_j1; ar++) {
    z_data[ar] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  b_j1 = indMeas_size[0] << 1;
  for (ar = 0; ar < b_j1; ar++) {
    h_u_data[ar] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_orientation, 2);
  emxInit_real_T(&H_pressure, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  while (k <= indMeas_size[0] - 1) {
    for (ar = 0; ar < 3; ar++) {
      bv1[ar] = rtIsNaN(map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)]);
    }

    if (c_any(bv1)) {
      b_ros_error();
    }

    for (ar = 0; ar < 3; ar++) {
      b_map[ar] = map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      r_orientation[ar] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        r_orientation[ar] += R_cw[ar + 3 * i2] * b_map[i2];
      }
    }

    for (kidx = 0; kidx < 3; kidx++) {
      h_cin_l[kidx] = r_orientation[kidx] / r_orientation[2];
    }

    predictMeasurement_left(r_orientation, c_cameraparams_CameraParameters,
      d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, dv3);
    b_j1 = k << 1;
    for (ar = 0; ar < 2; ar++) {
      h_u_data[ar + b_j1] = dv3[ar];
    }

    b_j1 = k << 1;
    indMeas = (indMeas_data[k] - 1.0) * 2.0;
    for (ar = 0; ar < 2; ar++) {
      z_data[ar + b_j1] = z_all_l[(int)(indMeas + (1.0 + (double)ar)) - 1];
    }

    //     %% computation of H(x)
    b_k = (signed char)((signed char)k << 1);
    for (ar = 0; ar < 2; ar++) {
      iv2[ar] = (signed char)(ar + b_k);
    }

    b_j1 = H_xc->size[1];
    ar = r2->size[0];
    r2->size[0] = b_j1;
    emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
    for (ar = 0; ar < b_j1; ar++) {
      r2->data[ar] = ar;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    b_fx_l[0] = fx_l;
    b_fx_l[2] = 0.0;
    b_fx_l[1] = 0.0;
    b_fx_l[3] = fy_l;
    dv4[0] = 1.0 / r_orientation[2];
    dv4[2] = 0.0;
    dv4[4] = -r_orientation[0] / (r_orientation[2] * r_orientation[2]);
    dv4[1] = 0.0;
    dv4[3] = 1.0 / r_orientation[2];
    dv4[5] = -r_orientation[1] / (r_orientation[2] * r_orientation[2]);
    for (ar = 0; ar < 2; ar++) {
      for (i2 = 0; i2 < 2; i2++) {
        c_fx_l[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          c_fx_l[ar + (i2 << 1)] += b_fx_l[ar + (kidx << 1)] * d[kidx + (i2 << 1)];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        d_fx_l[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          d_fx_l[ar + (i2 << 1)] += c_fx_l[ar + (kidx << 1)] * dv4[kidx + (i2 <<
            1)];
        }
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_R_cw[i2 + 3 * ar] = -R_cw[i2 + 3 * ar];
      }
    }

    b_R_cw[9] = 0.0;
    b_R_cw[12] = -r_orientation[2];
    b_R_cw[15] = r_orientation[1];
    b_R_cw[10] = r_orientation[2];
    b_R_cw[13] = 0.0;
    b_R_cw[16] = -r_orientation[0];
    b_R_cw[11] = -r_orientation[1];
    b_R_cw[14] = r_orientation[0];
    b_R_cw[17] = 0.0;
    for (ar = 0; ar < 6; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_R_cw[i2 + 3 * (ar + 6)] = 0.0;
      }
    }

    for (ar = 0; ar < 2; ar++) {
      for (i2 = 0; i2 < 12; i2++) {
        e_fx_l[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          e_fx_l[ar + (i2 << 1)] += d_fx_l[ar + (kidx << 1)] * b_R_cw[kidx + 3 *
            i2];
        }
      }
    }

    kidx = r2->size[0];
    for (ar = 0; ar < kidx; ar++) {
      for (i2 = 0; i2 < 2; i2++) {
        H_xc->data[iv2[i2] + H_xc->size[0] * r2->data[ar]] = e_fx_l[i2 + (ar <<
          1)];
      }
    }

    //     %% anchor state derivatives
    //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      numStatesPerAnchor;
    anchorRot[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
    anchorRot[3] = 2.0 * (b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(c_stateSize + 5.0) - 1] + b_xt->data[(int)
                          (d_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (e_stateSize + 7.0) - 1]);
    anchorRot[6] = 2.0 * (b_xt->data[(int)(f_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(g_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (h_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (i_stateSize + 7.0) - 1]);
    anchorRot[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(k_stateSize + 5.0) - 1] - b_xt->data[(int)
                          (l_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (m_stateSize + 7.0) - 1]);
    anchorRot[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
    anchorRot[7] = 2.0 * (b_xt->data[(int)(n_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(o_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (p_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (q_stateSize + 7.0) - 1]);
    anchorRot[2] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(s_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (t_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (u_stateSize + 7.0) - 1]);
    anchorRot[5] = 2.0 * (b_xt->data[(int)(v_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(w_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (x_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (y_stateSize + 7.0) - 1]);
    anchorRot[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;
    c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
      indMeas_data[k] - 1]) - 1];
    for (ar = 0; ar < 3; ar++) {
      d2 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        d2 += anchorRot[i2 + 3 * ar] * b_m_vect->data[i2 + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      y[ar] = d2 / c_xt;
    }

    c = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1]
      - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
                         indMeas_data[k] - 1]) - 1] * b_xt->data[(int)
      (((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
         numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)indMeas_data[k]
       - 1]) - 1];
    dv5[0] = 0.0;
    dv5[3] = -y[2];
    dv5[6] = y[1];
    dv5[1] = y[2];
    dv5[4] = 0.0;
    dv5[7] = -y[0];
    dv5[2] = -y[1];
    dv5[5] = y[0];
    dv5[8] = 0.0;
    kidx = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_anchorRot[i2 + 3 * ar] = -anchorRot[ar + 3 * i2];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      d2 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        d2 += b_anchorRot[ar + 3 * i2] * b_m_vect->data[i2 + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_map[ar] = d2 / c;
    }

    b_j1 = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k]
                 - 1]);
    ar = H_orientation->size[0] * H_orientation->size[1];
    H_orientation->size[0] = 3;
    H_orientation->size[1] = (kidx + b_j1) + 7;
    emxEnsureCapacity((emxArray__common *)H_orientation, ar, (int)sizeof(double));
    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_orientation->data[i2 + H_orientation->size[0] * ar] = b[i2 + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_orientation->data[i2 + H_orientation->size[0] * (ar + 3)] = -dv5[i2 +
          3 * ar];
      }
    }

    for (ar = 0; ar < kidx; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_orientation->data[i2 + H_orientation->size[0] * (ar + 6)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      H_orientation->data[ar + H_orientation->size[0] * (6 + kidx)] = b_map[ar];
    }

    for (ar = 0; ar < b_j1; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_orientation->data[i2 + H_orientation->size[0] * ((ar + kidx) + 7)] =
          0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (ar = 0; ar < 2; ar++) {
      iv2[ar] = (signed char)(ar + b_k);
    }

    if (rtIsNaN(numErrorStatesPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (6.0 + numPointsPerAnchor < 1.0) {
      br = -1;
      anew = 1.0;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (rtIsInf(numErrorStatesPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else {
      anew = 1.0;
      ndbl = floor((numErrorStatesPerAnchor - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
      absb = fabs(6.0 + numPointsPerAnchor);
      if ((1.0 >= absb) || rtIsNaN(absb)) {
        absb = 1.0;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = 6.0 + numPointsPerAnchor;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        br = (int)ndbl - 1;
      } else {
        br = -1;
      }
    }

    ar = H_pressure->size[0] * H_pressure->size[1];
    H_pressure->size[0] = 1;
    H_pressure->size[1] = br + 1;
    emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
    if (br + 1 > 0) {
      H_pressure->data[0] = anew;
      if (br + 1 > 1) {
        H_pressure->data[br] = apnd;
        kidx = (br + (br < 0)) >> 1;
        for (b_j1 = 1; b_j1 < kidx; b_j1++) {
          H_pressure->data[b_j1] = anew + (double)b_j1;
          H_pressure->data[br - b_j1] = apnd - (double)b_j1;
        }

        if (kidx << 1 == br) {
          H_pressure->data[kidx] = (anew + apnd) / 2.0;
        } else {
          H_pressure->data[kidx] = anew + (double)kidx;
          H_pressure->data[kidx + 1] = apnd - (double)kidx;
        }
      }
    }

    b_anchorIdx = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    ar = r2->size[0];
    r2->size[0] = H_pressure->size[1];
    emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
    b_j1 = H_pressure->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      r2->data[ar] = (int)(b_anchorIdx + H_pressure->data[H_pressure->size[0] *
                           ar]) - 1;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    f_fx_l[0] = fx_l;
    f_fx_l[2] = 0.0;
    f_fx_l[1] = 0.0;
    f_fx_l[3] = fy_l;
    dv6[0] = 1.0 / r_orientation[2];
    dv6[2] = 0.0;
    dv6[4] = -r_orientation[0] / (r_orientation[2] * r_orientation[2]);
    dv6[1] = 0.0;
    dv6[3] = 1.0 / r_orientation[2];
    dv6[5] = -r_orientation[1] / (r_orientation[2] * r_orientation[2]);
    for (ar = 0; ar < 2; ar++) {
      for (i2 = 0; i2 < 2; i2++) {
        c_fx_l[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          c_fx_l[ar + (i2 << 1)] += f_fx_l[ar + (kidx << 1)] * d[kidx + (i2 << 1)];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        d_fx_l[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          d_fx_l[ar + (i2 << 1)] += c_fx_l[ar + (kidx << 1)] * dv6[kidx + (i2 <<
            1)];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        b_y[ar + (i2 << 1)] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          b_y[ar + (i2 << 1)] += d_fx_l[ar + (kidx << 1)] * R_cw[kidx + 3 * i2];
        }
      }
    }

    v[1] = H_orientation->size[1];
    ar = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
    ar = C->size[0] * C->size[1];
    C->size[1] = (int)v[1];
    emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
    b_j1 = (int)v[1] << 1;
    for (ar = 0; ar < b_j1; ar++) {
      C->data[ar] = 0.0;
    }

    kidx = (H_orientation->size[1] - 1) << 1;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      ar = 0;
      for (i2 = br; i2 + 1 <= br + 3; i2++) {
        if (H_orientation->data[i2] != 0.0) {
          ia = ar;
          for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
            ia++;
            C->data[ic] += H_orientation->data[i2] * b_y[ia - 1];
          }
        }

        ar += 2;
      }

      br += 3;
    }

    b_j1 = C->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (i2 = 0; i2 < 2; i2++) {
        H_xm->data[iv2[i2] + H_xm->size[0] * r2->data[ar]] = C->data[i2 +
          C->size[0] * ar];
      }
    }

    k++;
  }

  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  for (kidx = 0; kidx < 3; kidx++) {
    r_orientation[kidx] = 0.0;
  }

  r_pressure = 0.0;
  for (kidx = 0; kidx < 6; kidx++) {
    r_ext_pose[kidx] = 0.0;
  }

  ar = H_orientation->size[0] * H_orientation->size[1];
  H_orientation->size[0] = 3;
  H_orientation->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_orientation, ar, (int)sizeof(double));
  b_j1 = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_orientation->data[ar] = 0.0;
  }

  ar = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
  b_j1 = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_pressure->data[ar] = 0.0;
  }

  emxInit_real_T(&H_ext_pose, 2);
  ar = H_ext_pose->size[0] * H_ext_pose->size[1];
  H_ext_pose->size[0] = 6;
  H_ext_pose->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_ext_pose, ar, (int)sizeof(double));
  b_j1 = 6 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_ext_pose->data[ar] = 0.0;
  }

  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (kidx = 0; kidx < 2; kidx++) {
    d[kidx + (kidx << 1)] = v[kidx];
  }

  emxInit_real_T(&r3, 2);
  eye((double)indMeas_size[0], r3);
  ia = r3->size[0];
  ic = r3->size[1];
  b_j1 = r3->size[0] * r3->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    A_data[ar] = r3->data[ar];
  }

  emxFree_real_T(&r3);
  R_vision_size_idx_0 = (signed char)(ia << 1);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= ic; b_j1++) {
    for (br = 0; br < 2; br++) {
      for (ar = 1; ar <= ia; ar++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          R_vision_data[kidx] = A_data[(ar + ia * (b_j1 - 1)) - 1] * d[i2 + (br <<
            1)];
        }
      }
    }
  }

  residual_size = indMeas_size[0] << 1;
  orientation_idx = 0;
  pressure_idx = 0;
  ext_pose_idx = 0;
  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0] -
                          measurements->att_fmu[1] * measurements->att_fmu[1]) -
                         measurements->att_fmu[2] * measurements->att_fmu[2]) +
      measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[3] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] + measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[6] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] - measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[1] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] - measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          + measurements->att_fmu[1] * measurements->att_fmu[1])
                         - measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[7] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] + measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[2] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] + measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[5] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] - measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          - measurements->att_fmu[1] * measurements->att_fmu[1])
                         + measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_anchorRot[ar + 3 * i2] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          b_anchorRot[ar + 3 * i2] += R_bc[kidx + 3 * ar] * b_measurements[kidx
            + 3 * i2];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        dv5[ar + 3 * i2] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          dv5[ar + 3 * i2] += b_anchorRot[ar + 3 * kidx] * R_cw[i2 + 3 * kidx];
        }
      }
    }

    QuatFromRotJ(dv5, d);
    for (kidx = 0; kidx < 3; kidx++) {
      r_orientation[kidx] = d[kidx];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_orientation->data[i2 + H_orientation->size[0] * (3 + ar)] = I[i2 + 3 *
          ar];
      }
    }

    orientation_idx = residual_size;
    residual_size += 3;
  }

  if (b_VIOParameters.use_pressure) {
    r_pressure = (1.0 - rt_powd_snf(measurements->bar_fmu / 101325.0, 0.190284))
      * 145366.45 - b_height_offset_pressure;
    H_pressure->data[2] = 1.0;
    pressure_idx = residual_size;
    residual_size++;
  }

  if (b_VIOParameters.use_ext_pose) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0] -
                          measurements->att_ext[1] * measurements->att_ext[1]) -
                         measurements->att_ext[2] * measurements->att_ext[2]) +
      measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[3] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] + measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[6] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] - measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[1] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] - measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[4] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          + measurements->att_ext[1] * measurements->att_ext[1])
                         - measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[7] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] + measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[2] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] + measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[5] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] - measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[8] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          - measurements->att_ext[1] * measurements->att_ext[1])
                         + measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_measurements[ar + 3 * i2] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          b_measurements[ar + 3 * i2] += c_measurements[ar + 3 * kidx] * R_cw[i2
            + 3 * kidx];
        }
      }
    }

    QuatFromRotJ(b_measurements, d);
    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar] = measurements->pos_ext[ar] - b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar + 3] = d[ar];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    memset(&anchorRot[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      anchorRot[k + 3 * k] = 1.0;
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_ext_pose->data[i2 + H_ext_pose->size[0] * ar] = I[i2 + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_ext_pose->data[i2 + H_ext_pose->size[0] * (ar + 3)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_ext_pose->data[(i2 + H_ext_pose->size[0] * ar) + 3] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H_ext_pose->data[(i2 + H_ext_pose->size[0] * (ar + 3)) + 3] =
          anchorRot[i2 + 3 * ar];
      }
    }

    ext_pose_idx = residual_size;
    residual_size += 6;
  }

  r_size[0] = residual_size;
  for (ar = 0; ar < residual_size; ar++) {
    r_data[ar] = 0.0;
  }

  ar = H->size[0] * H->size[1];
  H->size[0] = residual_size;
  H->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
  b_j1 = residual_size * (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H->data[ar] = 0.0;
  }

  R_size[0] = residual_size;
  R_size[1] = residual_size;
  b_j1 = residual_size * residual_size;
  for (ar = 0; ar < b_j1; ar++) {
    R_data[ar] = 0.0;
  }

  ar = indMeas_size[0] << 1;
  if (1 > ar) {
    b_j1 = 0;
  } else {
    b_j1 = ar;
  }

  for (ar = 0; ar < b_j1; ar++) {
    tmp_data[ar] = ar;
  }

  for (ar = 0; ar < b_j1; ar++) {
    r_data[tmp_data[ar]] = z_data[ar] - h_u_data[ar];
  }

  b_j1 = H_xc->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    kidx = H_xc->size[0];
    for (i2 = 0; i2 < kidx; i2++) {
      H->data[i2 + H->size[0] * ar] = H_xc->data[i2 + H_xc->size[0] * ar];
    }
  }

  b_j1 = H_xm->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    kidx = H_xm->size[0];
    for (i2 = 0; i2 < kidx; i2++) {
      H->data[i2 + H->size[0] * (ar + H_xc->size[1])] = H_xm->data[i2 +
        H_xm->size[0] * ar];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  b_j1 = (signed char)(ic << 1);
  for (ar = 0; ar < b_j1; ar++) {
    for (i2 = 0; i2 < R_vision_size_idx_0; i2++) {
      R_data[i2 + R_size[0] * ar] = R_vision_data[i2 + R_vision_size_idx_0 * ar];
    }
  }

  if (b_VIOParameters.use_orientation) {
    for (ar = 0; ar < 3; ar++) {
      r_data[ar + orientation_idx] = r_orientation[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      iv3[ar] = (signed char)(ar + (signed char)orientation_idx);
    }

    b_j1 = H_orientation->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        H->data[iv3[i2] + H->size[0] * ar] = H_orientation->data[i2 +
          H_orientation->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        R_data[(i2 + orientation_idx) + R_size[0] * (ar + orientation_idx)] =
          c_noiseParameters_orientation_n * (double)b[i2 + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_orientation);
  if (b_VIOParameters.use_pressure) {
    r_data[pressure_idx] = r_pressure;
    b_j1 = H_pressure->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      H->data[pressure_idx + H->size[0] * ar] = H_pressure->data
        [H_pressure->size[0] * ar];
    }

    R_data[pressure_idx + R_size[0] * pressure_idx] =
      noiseParameters_pressure_noise;
  }

  emxFree_real_T(&H_pressure);
  if (b_VIOParameters.use_ext_pose) {
    for (ar = 0; ar < 6; ar++) {
      r_data[ar + ext_pose_idx] = r_ext_pose[ar];
    }

    for (ar = 0; ar < 6; ar++) {
      iv4[ar] = (signed char)(ar + (signed char)ext_pose_idx);
    }

    b_j1 = H_ext_pose->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (i2 = 0; i2 < 6; i2++) {
        H->data[iv4[i2] + H->size[0] * ar] = H_ext_pose->data[i2 +
          H_ext_pose->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        R_data[(i2 + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)] =
          noiseParameters_ext_pos_noise * (double)b[i2 + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        R_data[(i2 + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        R_data[((i2 + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)) + 3] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (i2 = 0; i2 < 3; i2++) {
        R_data[((i2 + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)) + 3]
          = noiseParameters_ext_att_noise * (double)b[i2 + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_ext_pose);
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//    Does not take into account the derivative with respect to anchor states
//    (static map)
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
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                double indMeas
//                const emxArray_real_T *map
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u[2]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[32], double indMeas, const
                emxArray_real_T *map, const double
                c_cameraparams_CameraParameters[3], const double
                d_cameraparams_CameraParameters[2], const double
                e_cameraparams_CameraParameters[2], const emxArray_real_T
                *anchorIdx, const emxArray_real_T *featureAnchorIdx, const
                emxArray_real_T *b_m_vect, const double
                noiseParameters_image_noise[2], double
                c_noiseParameters_orientation_n, double
                noiseParameters_pressure_noise, double
                noiseParameters_ext_pos_noise, double
                noiseParameters_ext_att_noise, const VIOMeasurements
                *measurements, double b_height_offset_pressure, const
                VIOParameters b_VIOParameters, double r_data[], int r_size[1],
                emxArray_real_T *H, double h_u[2], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ar;
  int nm1d2;
  emxArray_real_T *H_xc;
  boolean_T bv0[3];
  double b_map[3];
  double h_ci_l[3];
  int br;
  double h_cin_l[3];
  double anew;
  double z[2];
  emxArray_int32_T *r0;
  double d[4];
  double f_cameraparams_CameraParameters[4];
  double g_cameraparams_CameraParameters[4];
  double dv0[6];
  double h_cameraparams_CameraParameters[6];
  double b_R_cw[36];
  double i_cameraparams_CameraParameters[24];
  double b_stateSize;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double anchorRot[9];
  double d1;
  double y[3];
  double dv1[9];
  double b_anchorRot[9];
  emxArray_real_T *H_iy;
  int n;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *H_pressure;
  double j_cameraparams_CameraParameters[4];
  double dv2[6];
  double b_y[6];
  emxArray_real_T *C;
  double v[2];
  int ic;
  int ib;
  int ia;
  double r_pressure;
  double r_ext_pose[6];
  emxArray_real_T *H_ext_pose;
  double R_vision[4];
  int residual_size;
  int orientation_idx;
  int pressure_idx;
  int ext_pose_idx;
  double b_measurements[9];
  signed char I[9];
  double c_measurements[9];
  signed char iv0[3];
  signed char iv1[6];
  emxInit_real_T(&H_xm, 2);

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = 2;
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  nm1d2 = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor)) << 1;
  for (ar = 0; ar < nm1d2; ar++) {
    H_xm->data[ar] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  nm1d2 = (int)errorStateSize << 1;
  for (ar = 0; ar < nm1d2; ar++) {
    H_xc->data[ar] = 0.0;
  }

  for (ar = 0; ar < 3; ar++) {
    bv0[ar] = rtIsNaN(map->data[ar + map->size[0] * ((int)indMeas - 1)]);
  }

  if (c_any(bv0)) {
    b_ros_error();
  }

  for (ar = 0; ar < 3; ar++) {
    b_map[ar] = map->data[ar + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ar];
  }

  for (ar = 0; ar < 3; ar++) {
    h_ci_l[ar] = 0.0;
    for (br = 0; br < 3; br++) {
      h_ci_l[ar] += R_cw[ar + 3 * br] * b_map[br];
    }
  }

  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  predictMeasurement_left(h_ci_l, c_cameraparams_CameraParameters,
    d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, h_u);
  anew = (indMeas - 1.0) * 2.0;
  for (ar = 0; ar < 2; ar++) {
    z[ar] = z_all_l[(int)(anew + (1.0 + (double)ar)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  nm1d2 = (int)errorStateSize;
  ar = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  for (ar = 0; ar < nm1d2; ar++) {
    r0->data[ar] = ar;
  }

  Ch_dn_To_h_un(c_cameraparams_CameraParameters[0],
                c_cameraparams_CameraParameters[1],
                c_cameraparams_CameraParameters[2], h_cin_l[0], h_cin_l[1], d);
  f_cameraparams_CameraParameters[0] = d_cameraparams_CameraParameters[0];
  f_cameraparams_CameraParameters[2] = 0.0;
  f_cameraparams_CameraParameters[1] = 0.0;
  f_cameraparams_CameraParameters[3] = d_cameraparams_CameraParameters[1];
  dv0[0] = 1.0 / h_ci_l[2];
  dv0[2] = 0.0;
  dv0[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv0[1] = 0.0;
  dv0[3] = 1.0 / h_ci_l[2];
  dv0[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 2; br++) {
      g_cameraparams_CameraParameters[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        g_cameraparams_CameraParameters[ar + (br << 1)] +=
          f_cameraparams_CameraParameters[ar + (nm1d2 << 1)] * d[nm1d2 + (br <<
          1)];
      }
    }

    for (br = 0; br < 3; br++) {
      h_cameraparams_CameraParameters[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_cameraparams_CameraParameters[ar + (br << 1)] +=
          g_cameraparams_CameraParameters[ar + (nm1d2 << 1)] * dv0[nm1d2 + (br <<
          1)];
      }
    }
  }

  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * ar] = -R_cw[br + 3 * ar];
    }
  }

  b_R_cw[9] = 0.0;
  b_R_cw[12] = -h_ci_l[2];
  b_R_cw[15] = h_ci_l[1];
  b_R_cw[10] = h_ci_l[2];
  b_R_cw[13] = 0.0;
  b_R_cw[16] = -h_ci_l[0];
  b_R_cw[11] = -h_ci_l[1];
  b_R_cw[14] = h_ci_l[0];
  b_R_cw[17] = 0.0;
  for (ar = 0; ar < 6; ar++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * (ar + 6)] = 0.0;
    }
  }

  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 12; br++) {
      i_cameraparams_CameraParameters[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        i_cameraparams_CameraParameters[ar + (br << 1)] +=
          h_cameraparams_CameraParameters[ar + (nm1d2 << 1)] * b_R_cw[nm1d2 + 3 *
          br];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H_xc->data[br + H_xc->size[0] * r0->data[ar]] =
        i_cameraparams_CameraParameters[br + (ar << 1)];
    }
  }

  //     %% anchor state derivatives
  //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  c_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  d_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  e_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  f_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  g_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  h_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  i_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  j_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  k_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  l_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  m_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  n_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  o_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  p_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  q_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  r_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  s_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  t_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  u_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  v_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  w_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  x_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  y_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  anchorRot[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  anchorRot[3] = 2.0 * (b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(c_stateSize + 5.0) - 1] + b_xt->data[(int)
                        (d_stateSize + 6.0) - 1] * b_xt->data[(int)(e_stateSize
    + 7.0) - 1]);
  anchorRot[6] = 2.0 * (b_xt->data[(int)(f_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(g_stateSize + 6.0) - 1] - b_xt->data[(int)
                        (h_stateSize + 5.0) - 1] * b_xt->data[(int)(i_stateSize
    + 7.0) - 1]);
  anchorRot[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(k_stateSize + 5.0) - 1] - b_xt->data[(int)
                        (l_stateSize + 6.0) - 1] * b_xt->data[(int)(m_stateSize
    + 7.0) - 1]);
  anchorRot[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  anchorRot[7] = 2.0 * (b_xt->data[(int)(n_stateSize + 5.0) - 1] * b_xt->data
                        [(int)(o_stateSize + 6.0) - 1] + b_xt->data[(int)
                        (p_stateSize + 4.0) - 1] * b_xt->data[(int)(q_stateSize
    + 7.0) - 1]);
  anchorRot[2] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(s_stateSize + 6.0) - 1] + b_xt->data[(int)
                        (t_stateSize + 5.0) - 1] * b_xt->data[(int)(u_stateSize
    + 7.0) - 1]);
  anchorRot[5] = 2.0 * (b_xt->data[(int)(v_stateSize + 5.0) - 1] * b_xt->data
                        [(int)(w_stateSize + 6.0) - 1] - b_xt->data[(int)
                        (x_stateSize + 4.0) - 1] * b_xt->data[(int)(y_stateSize
    + 7.0) - 1]);
  anchorRot[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;
  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1];
  for (ar = 0; ar < 3; ar++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += anchorRot[br + 3 * ar] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    y[ar] = d1 / anew;
  }

  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1] * b_xt->data[(int)
    (((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
        b_VIOParameters.num_points_per_anchor)) + 7.0) + featureAnchorIdx->data
     [(int)indMeas - 1]) - 1];
  dv1[0] = 0.0;
  dv1[3] = -y[2];
  dv1[6] = y[1];
  dv1[1] = y[2];
  dv1[4] = 0.0;
  dv1[7] = -y[0];
  dv1[2] = -y[1];
  dv1[5] = y[0];
  dv1[8] = 0.0;
  nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas - 1] - 1.0);
  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      b_anchorRot[br + 3 * ar] = -anchorRot[ar + 3 * br];
    }
  }

  for (ar = 0; ar < 3; ar++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += b_anchorRot[ar + 3 * br] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    b_map[ar] = d1 / anew;
  }

  emxInit_real_T(&H_iy, 2);
  n = (int)(b_VIOParameters.num_points_per_anchor - featureAnchorIdx->data[(int)
            indMeas - 1]);
  ar = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + n) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ar, (int)sizeof(double));
  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ar] = b[br + 3 * ar];
    }
  }

  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * (ar + 3)] = -dv1[br + 3 * ar];
    }
  }

  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * (ar + 6)] = 0.0;
    }
  }

  for (ar = 0; ar < 3; ar++) {
    H_iy->data[ar + H_iy->size[0] * (6 + nm1d2)] = b_map[ar];
  }

  for (ar = 0; ar < n; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ((ar + nm1d2) + 7)] = 0.0;
    }
  }

  if (rtIsNaN(6.0 + b_VIOParameters.num_points_per_anchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else if (6.0 + b_VIOParameters.num_points_per_anchor < 1.0) {
    n = -1;
    anew = 1.0;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else if (rtIsInf(6.0 + b_VIOParameters.num_points_per_anchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else {
    anew = 1.0;
    ndbl = floor(((6.0 + b_VIOParameters.num_points_per_anchor) - 1.0) + 0.5);
    apnd = 1.0 + ndbl;
    cdiff = (1.0 + ndbl) - (6.0 + b_VIOParameters.num_points_per_anchor);
    absb = fabs(6.0 + b_VIOParameters.num_points_per_anchor);
    if ((1.0 >= absb) || rtIsNaN(absb)) {
      absb = 1.0;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
    } else if (cdiff > 0.0) {
      apnd = 1.0 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl - 1;
    } else {
      n = -1;
    }
  }

  emxInit_real_T(&H_pressure, 2);
  ar = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
  if (n + 1 > 0) {
    H_pressure->data[0] = anew;
    if (n + 1 > 1) {
      H_pressure->data[n] = apnd;
      nm1d2 = (n + (n < 0)) >> 1;
      for (br = 1; br < nm1d2; br++) {
        H_pressure->data[br] = anew + (double)br;
        H_pressure->data[n - br] = apnd - (double)br;
      }

      if (nm1d2 << 1 == n) {
        H_pressure->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        H_pressure->data[nm1d2] = anew + (double)nm1d2;
        H_pressure->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  anew = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 +
    b_VIOParameters.num_points_per_anchor);
  ar = r0->size[0];
  r0->size[0] = H_pressure->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  nm1d2 = H_pressure->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    r0->data[ar] = (int)(anew + H_pressure->data[H_pressure->size[0] * ar]) - 1;
  }

  Ch_dn_To_h_un(c_cameraparams_CameraParameters[0],
                c_cameraparams_CameraParameters[1],
                c_cameraparams_CameraParameters[2], h_cin_l[0], h_cin_l[1], d);
  j_cameraparams_CameraParameters[0] = d_cameraparams_CameraParameters[0];
  j_cameraparams_CameraParameters[2] = 0.0;
  j_cameraparams_CameraParameters[1] = 0.0;
  j_cameraparams_CameraParameters[3] = d_cameraparams_CameraParameters[1];
  dv2[0] = 1.0 / h_ci_l[2];
  dv2[2] = 0.0;
  dv2[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv2[1] = 0.0;
  dv2[3] = 1.0 / h_ci_l[2];
  dv2[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 2; br++) {
      f_cameraparams_CameraParameters[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        f_cameraparams_CameraParameters[ar + (br << 1)] +=
          j_cameraparams_CameraParameters[ar + (nm1d2 << 1)] * d[nm1d2 + (br <<
          1)];
      }
    }

    for (br = 0; br < 3; br++) {
      h_cameraparams_CameraParameters[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_cameraparams_CameraParameters[ar + (br << 1)] +=
          f_cameraparams_CameraParameters[ar + (nm1d2 << 1)] * dv2[nm1d2 + (br <<
          1)];
      }
    }

    for (br = 0; br < 3; br++) {
      b_y[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        b_y[ar + (br << 1)] += h_cameraparams_CameraParameters[ar + (nm1d2 << 1)]
          * R_cw[nm1d2 + 3 * br];
      }
    }
  }

  emxInit_real_T(&C, 2);
  v[1] = H_iy->size[1];
  ar = C->size[0] * C->size[1];
  C->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
  ar = C->size[0] * C->size[1];
  C->size[1] = (int)v[1];
  emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
  nm1d2 = (int)v[1] << 1;
  for (ar = 0; ar < nm1d2; ar++) {
    C->data[ar] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (n = 0; n <= nm1d2; n += 2) {
    for (ic = n; ic + 1 <= n + 2; ic++) {
      C->data[ic] = 0.0;
    }
  }

  br = 0;
  for (n = 0; n <= nm1d2; n += 2) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (H_iy->data[ib] != 0.0) {
        ia = ar;
        for (ic = n; ic + 1 <= n + 2; ic++) {
          ia++;
          C->data[ic] += H_iy->data[ib] * b_y[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  nm1d2 = C->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H_xm->data[br + H_xm->size[0] * r0->data[ar]] = C->data[br + C->size[0] *
        ar];
    }
  }

  emxFree_real_T(&C);
  emxFree_int32_T(&r0);
  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_ci_l[nm1d2] = 0.0;
  }

  r_pressure = 0.0;
  for (nm1d2 = 0; nm1d2 < 6; nm1d2++) {
    r_ext_pose[nm1d2] = 0.0;
  }

  ar = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_iy, ar, (int)sizeof(double));
  nm1d2 = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < nm1d2; ar++) {
    H_iy->data[ar] = 0.0;
  }

  ar = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
  nm1d2 = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < nm1d2; ar++) {
    H_pressure->data[ar] = 0.0;
  }

  emxInit_real_T(&H_ext_pose, 2);
  ar = H_ext_pose->size[0] * H_ext_pose->size[1];
  H_ext_pose->size[0] = 6;
  H_ext_pose->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_ext_pose, ar, (int)sizeof(double));
  nm1d2 = 6 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < nm1d2; ar++) {
    H_ext_pose->data[ar] = 0.0;
  }

  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  nm1d2 = -1;
  for (n = 0; n < 2; n++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_vision[nm1d2] = d[br + (n << 1)];
    }
  }

  residual_size = 2;
  orientation_idx = 0;
  pressure_idx = 0;
  ext_pose_idx = 0;
  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0] -
                          measurements->att_fmu[1] * measurements->att_fmu[1]) -
                         measurements->att_fmu[2] * measurements->att_fmu[2]) +
      measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[3] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] + measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[6] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] - measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[1] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] - measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          + measurements->att_fmu[1] * measurements->att_fmu[1])
                         - measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[7] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] + measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[2] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] + measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[5] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] - measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          - measurements->att_fmu[1] * measurements->att_fmu[1])
                         + measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        b_anchorRot[ar + 3 * br] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          b_anchorRot[ar + 3 * br] += R_bc[nm1d2 + 3 * ar] *
            b_measurements[nm1d2 + 3 * br];
        }
      }

      for (br = 0; br < 3; br++) {
        dv1[ar + 3 * br] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[ar + 3 * br] += b_anchorRot[ar + 3 * nm1d2] * R_cw[br + 3 * nm1d2];
        }
      }
    }

    QuatFromRotJ(dv1, f_cameraparams_CameraParameters);
    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      h_ci_l[nm1d2] = f_cameraparams_CameraParameters[nm1d2];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (br = 0; br < 3; br++) {
      I[br + 3 * br] = 1;
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_iy->data[br + H_iy->size[0] * (3 + ar)] = I[br + 3 * ar];
      }
    }

    orientation_idx = 2;
    residual_size = 5;
  }

  if (b_VIOParameters.use_pressure) {
    r_pressure = (1.0 - rt_powd_snf(measurements->bar_fmu / 101325.0, 0.190284))
      * 145366.45 - b_height_offset_pressure;
    H_pressure->data[2] = 1.0;
    pressure_idx = residual_size;
    residual_size++;
  }

  if (b_VIOParameters.use_ext_pose) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0] -
                          measurements->att_ext[1] * measurements->att_ext[1]) -
                         measurements->att_ext[2] * measurements->att_ext[2]) +
      measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[3] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] + measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[6] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] - measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[1] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] - measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[4] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          + measurements->att_ext[1] * measurements->att_ext[1])
                         - measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[7] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] + measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[2] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] + measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[5] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] - measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[8] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          - measurements->att_ext[1] * measurements->att_ext[1])
                         + measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        b_measurements[ar + 3 * br] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          b_measurements[ar + 3 * br] += c_measurements[ar + 3 * nm1d2] *
            R_cw[br + 3 * nm1d2];
        }
      }
    }

    QuatFromRotJ(b_measurements, f_cameraparams_CameraParameters);
    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar] = measurements->pos_ext[ar] - b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar + 3] = f_cameraparams_CameraParameters[ar];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (br = 0; br < 3; br++) {
      I[br + 3 * br] = 1;
    }

    memset(&anchorRot[0], 0, 9U * sizeof(double));
    for (br = 0; br < 3; br++) {
      anchorRot[br + 3 * br] = 1.0;
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[br + H_ext_pose->size[0] * ar] = I[br + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[br + H_ext_pose->size[0] * (ar + 3)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[(br + H_ext_pose->size[0] * ar) + 3] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[(br + H_ext_pose->size[0] * (ar + 3)) + 3] =
          anchorRot[br + 3 * ar];
      }
    }

    ext_pose_idx = residual_size;
    residual_size += 6;
  }

  r_size[0] = residual_size;
  for (ar = 0; ar < residual_size; ar++) {
    r_data[ar] = 0.0;
  }

  ar = H->size[0] * H->size[1];
  H->size[0] = residual_size;
  H->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
  nm1d2 = residual_size * (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < nm1d2; ar++) {
    H->data[ar] = 0.0;
  }

  R_size[0] = residual_size;
  R_size[1] = residual_size;
  nm1d2 = residual_size * residual_size;
  for (ar = 0; ar < nm1d2; ar++) {
    R_data[ar] = 0.0;
  }

  for (ar = 0; ar < 2; ar++) {
    r_data[ar] = z[ar] - h_u[ar];
  }

  nm1d2 = H_xc->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H->data[br + H->size[0] * ar] = H_xc->data[br + H_xc->size[0] * ar];
    }
  }

  nm1d2 = H_xm->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H->data[br + H->size[0] * (ar + H_xc->size[1])] = H_xm->data[br +
        H_xm->size[0] * ar];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 2; br++) {
      R_data[br + R_size[0] * ar] = R_vision[br + (ar << 1)];
    }
  }

  if (b_VIOParameters.use_orientation) {
    for (ar = 0; ar < 3; ar++) {
      r_data[ar + orientation_idx] = h_ci_l[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      iv0[ar] = (signed char)(ar + orientation_idx);
    }

    nm1d2 = H_iy->size[1];
    for (ar = 0; ar < nm1d2; ar++) {
      for (br = 0; br < 3; br++) {
        H->data[iv0[br] + H->size[0] * ar] = H_iy->data[br + H_iy->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + orientation_idx) + R_size[0] * (ar + orientation_idx)] =
          c_noiseParameters_orientation_n * (double)b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_iy);
  if (b_VIOParameters.use_pressure) {
    r_data[pressure_idx] = r_pressure;
    nm1d2 = H_pressure->size[1];
    for (ar = 0; ar < nm1d2; ar++) {
      H->data[pressure_idx + H->size[0] * ar] = H_pressure->data
        [H_pressure->size[0] * ar];
    }

    R_data[pressure_idx + R_size[0] * pressure_idx] =
      noiseParameters_pressure_noise;
  }

  emxFree_real_T(&H_pressure);
  if (b_VIOParameters.use_ext_pose) {
    for (ar = 0; ar < 6; ar++) {
      r_data[ar + ext_pose_idx] = r_ext_pose[ar];
    }

    for (ar = 0; ar < 6; ar++) {
      iv1[ar] = (signed char)(ar + ext_pose_idx);
    }

    nm1d2 = H_ext_pose->size[1];
    for (ar = 0; ar < nm1d2; ar++) {
      for (br = 0; br < 6; br++) {
        H->data[iv1[br] + H->size[0] * ar] = H_ext_pose->data[br +
          H_ext_pose->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)] =
          noiseParameters_ext_pos_noise * (double)b[br + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[((br + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)) + 3] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[((br + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)) + 3]
          = noiseParameters_ext_att_noise * (double)b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_ext_pose);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
