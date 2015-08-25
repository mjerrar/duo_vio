//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 16:09:08
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "predictMeasurement_stereo.h"
#include "QuatFromRotJ.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_left.h"
#include "kron.h"
#include "eye.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
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
//                const double IMU_measurements[23]
//                double height_offset_pressure
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
                  noiseParameters_pressure_noise, const double IMU_measurements
                  [23], double height_offset_pressure, const VIOParameters
                  b_VIOParameters, double r_data[], int r_size[1],
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
  int i5;
  int br;
  emxArray_real_T *H_xc;
  int z_size_idx_0;
  double z_data[32];
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r2;
  emxArray_real_T *C;
  emxArray_real_T *y;
  double b_map[3];
  double h_ci_l[3];
  int i6;
  double h_cin_l[3];
  int nm1d2;
  double dv3[2];
  int cr;
  double indMeas;
  signed char b_k;
  signed char iv0[2];
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
  double b_y[3];
  double c;
  double dv5[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double b_anchorIdx;
  double f_fx_l[4];
  double dv6[6];
  double c_y[6];
  double v[2];
  int ic;
  int ar;
  int ib;
  int ia;
  double b_z_data;
  emxArray_real_T *H_v;
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_p_size_idx_1;
  emxArray_real_T *r3;
  int R_v_size[2];
  double R_v_data[1024];
  double b_IMU_measurements[9];
  double r_g_data[4];
  double R_g_data[9];
  double R_p_data[1];
  double r_p_data[1];
  int tmp_data[3];
  int b_tmp_data[3];
  int c_tmp_data[1];
  int d_tmp_data[1];
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
  i5 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, i5, (int)sizeof(double));
  i5 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, i5, (int)sizeof(double));
  br = (indMeas_size[0] << 1) * (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (i5 = 0; i5 < br; i5++) {
    H_xm->data[i5] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  i5 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, i5, (int)sizeof(double));
  i5 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, i5, (int)sizeof(double));
  br = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (i5 = 0; i5 < br; i5++) {
    H_xc->data[i5] = 0.0;
  }

  z_size_idx_0 = indMeas_size[0] << 1;
  br = indMeas_size[0] << 1;
  for (i5 = 0; i5 < br; i5++) {
    z_data[i5] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  br = indMeas_size[0] << 1;
  for (i5 = 0; i5 < br; i5++) {
    h_u_data[i5] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (i5 = 0; i5 < 3; i5++) {
      b_map[i5] = map->data[i5 + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[i5];
    }

    for (i5 = 0; i5 < 3; i5++) {
      h_ci_l[i5] = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        h_ci_l[i5] += R_cw[i5 + 3 * i6] * b_map[i6];
      }
    }

    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
    }

    predictMeasurement_left(h_ci_l, c_cameraparams_CameraParameters,
      d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, dv3);
    cr = k << 1;
    for (i5 = 0; i5 < 2; i5++) {
      h_u_data[i5 + cr] = dv3[i5];
    }

    cr = k << 1;
    indMeas = (indMeas_data[k] - 1.0) * 2.0;
    for (i5 = 0; i5 < 2; i5++) {
      z_data[i5 + cr] = z_all_l[(int)(indMeas + (1.0 + (double)i5)) - 1];
    }

    //     %% computation of H(x)
    b_k = (signed char)((signed char)k << 1);
    for (i5 = 0; i5 < 2; i5++) {
      iv0[i5] = (signed char)(i5 + b_k);
    }

    br = H_xc->size[1];
    i5 = r2->size[0];
    r2->size[0] = br;
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(int));
    for (i5 = 0; i5 < br; i5++) {
      r2->data[i5] = i5;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    b_fx_l[0] = fx_l;
    b_fx_l[2] = 0.0;
    b_fx_l[1] = 0.0;
    b_fx_l[3] = fy_l;
    dv4[0] = 1.0 / h_ci_l[2];
    dv4[2] = 0.0;
    dv4[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    dv4[1] = 0.0;
    dv4[3] = 1.0 / h_ci_l[2];
    dv4[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    for (i5 = 0; i5 < 2; i5++) {
      for (i6 = 0; i6 < 2; i6++) {
        c_fx_l[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
          c_fx_l[i5 + (i6 << 1)] += b_fx_l[i5 + (nm1d2 << 1)] * d[nm1d2 + (i6 <<
            1)];
        }
      }

      for (i6 = 0; i6 < 3; i6++) {
        d_fx_l[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
          d_fx_l[i5 + (i6 << 1)] += c_fx_l[i5 + (nm1d2 << 1)] * dv4[nm1d2 + (i6 <<
            1)];
        }
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_R_cw[i6 + 3 * i5] = -R_cw[i6 + 3 * i5];
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
    for (i5 = 0; i5 < 6; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_R_cw[i6 + 3 * (i5 + 6)] = 0.0;
      }
    }

    for (i5 = 0; i5 < 2; i5++) {
      for (i6 = 0; i6 < 12; i6++) {
        e_fx_l[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          e_fx_l[i5 + (i6 << 1)] += d_fx_l[i5 + (nm1d2 << 1)] * b_R_cw[nm1d2 + 3
            * i6];
        }
      }
    }

    nm1d2 = r2->size[0];
    for (i5 = 0; i5 < nm1d2; i5++) {
      for (i6 = 0; i6 < 2; i6++) {
        H_xc->data[iv0[i6] + H_xc->size[0] * r2->data[i5]] = e_fx_l[i6 + (i5 <<
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
    for (i5 = 0; i5 < 3; i5++) {
      d2 = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        d2 += anchorRot[i6 + 3 * i5] * b_m_vect->data[i6 + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_y[i5] = d2 / c_xt;
    }

    c = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1]
      - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
                         indMeas_data[k] - 1]) - 1] * b_xt->data[(int)
      (((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
         numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)indMeas_data[k]
       - 1]) - 1];
    dv5[0] = 0.0;
    dv5[3] = -b_y[2];
    dv5[6] = b_y[1];
    dv5[1] = b_y[2];
    dv5[4] = 0.0;
    dv5[7] = -b_y[0];
    dv5[2] = -b_y[1];
    dv5[5] = b_y[0];
    dv5[8] = 0.0;
    nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_anchorRot[i6 + 3 * i5] = -anchorRot[i5 + 3 * i6];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      d2 = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        d2 += b_anchorRot[i5 + 3 * i6] * b_m_vect->data[i6 + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_map[i5] = d2 / c;
    }

    cr = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k]
               - 1]);
    i5 = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (nm1d2 + cr) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, i5, (int)sizeof(double));
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        H_iy->data[i6 + H_iy->size[0] * i5] = b[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        H_iy->data[i6 + H_iy->size[0] * (i5 + 3)] = -dv5[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < nm1d2; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        H_iy->data[i6 + H_iy->size[0] * (i5 + 6)] = 0.0;
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      H_iy->data[i5 + H_iy->size[0] * (6 + nm1d2)] = b_map[i5];
    }

    for (i5 = 0; i5 < cr; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        H_iy->data[i6 + H_iy->size[0] * ((i5 + nm1d2) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (i5 = 0; i5 < 2; i5++) {
      iv0[i5] = (signed char)(i5 + b_k);
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

    i5 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = br + 1;
    emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
    if (br + 1 > 0) {
      y->data[0] = anew;
      if (br + 1 > 1) {
        y->data[br] = apnd;
        nm1d2 = (br + (br < 0)) >> 1;
        for (cr = 1; cr < nm1d2; cr++) {
          y->data[cr] = anew + (double)cr;
          y->data[br - cr] = apnd - (double)cr;
        }

        if (nm1d2 << 1 == br) {
          y->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          y->data[nm1d2] = anew + (double)nm1d2;
          y->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    b_anchorIdx = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    i5 = r2->size[0];
    r2->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(int));
    br = y->size[1];
    for (i5 = 0; i5 < br; i5++) {
      r2->data[i5] = (int)(b_anchorIdx + y->data[y->size[0] * i5]) - 1;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    f_fx_l[0] = fx_l;
    f_fx_l[2] = 0.0;
    f_fx_l[1] = 0.0;
    f_fx_l[3] = fy_l;
    dv6[0] = 1.0 / h_ci_l[2];
    dv6[2] = 0.0;
    dv6[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    dv6[1] = 0.0;
    dv6[3] = 1.0 / h_ci_l[2];
    dv6[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    for (i5 = 0; i5 < 2; i5++) {
      for (i6 = 0; i6 < 2; i6++) {
        c_fx_l[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
          c_fx_l[i5 + (i6 << 1)] += f_fx_l[i5 + (nm1d2 << 1)] * d[nm1d2 + (i6 <<
            1)];
        }
      }

      for (i6 = 0; i6 < 3; i6++) {
        d_fx_l[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
          d_fx_l[i5 + (i6 << 1)] += c_fx_l[i5 + (nm1d2 << 1)] * dv6[nm1d2 + (i6 <<
            1)];
        }
      }

      for (i6 = 0; i6 < 3; i6++) {
        c_y[i5 + (i6 << 1)] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          c_y[i5 + (i6 << 1)] += d_fx_l[i5 + (nm1d2 << 1)] * R_cw[nm1d2 + 3 * i6];
        }
      }
    }

    v[1] = H_iy->size[1];
    i5 = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, i5, (int)sizeof(double));
    i5 = C->size[0] * C->size[1];
    C->size[1] = (int)v[1];
    emxEnsureCapacity((emxArray__common *)C, i5, (int)sizeof(double));
    br = (int)v[1] << 1;
    for (i5 = 0; i5 < br; i5++) {
      C->data[i5] = 0.0;
    }

    nm1d2 = (H_iy->size[1] - 1) << 1;
    for (cr = 0; cr <= nm1d2; cr += 2) {
      for (ic = cr; ic + 1 <= cr + 2; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= nm1d2; cr += 2) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (H_iy->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[ib] * c_y[ia - 1];
          }
        }

        ar += 2;
      }

      br += 3;
    }

    br = C->size[1];
    for (i5 = 0; i5 < br; i5++) {
      for (i6 = 0; i6 < 2; i6++) {
        H_xm->data[iv0[i6] + H_xm->size[0] * r2->data[i5]] = C->data[i6 +
          C->size[0] * i5];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  emxFree_real_T(&H_iy);
  for (i5 = 0; i5 < z_size_idx_0; i5++) {
    b_z_data = z_data[i5] - h_u_data[i5];
    z_data[i5] = b_z_data;
  }

  emxInit_real_T(&H_v, 2);

  //  residual with respect to camera measurements
  ar = 0;

  //  gravity residual
  ia = 0;

  //  pressure residual
  i5 = H_v->size[0] * H_v->size[1];
  H_v->size[0] = H_xc->size[0];
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, i5, (int)sizeof(double));
  br = H_xc->size[1];
  for (i5 = 0; i5 < br; i5++) {
    nm1d2 = H_xc->size[0];
    for (i6 = 0; i6 < nm1d2; i6++) {
      H_v->data[i6 + H_v->size[0] * i5] = H_xc->data[i6 + H_xc->size[0] * i5];
    }
  }

  br = H_xm->size[1];
  for (i5 = 0; i5 < br; i5++) {
    nm1d2 = H_xm->size[0];
    for (i6 = 0; i6 < nm1d2; i6++) {
      H_v->data[i6 + H_v->size[0] * (i5 + H_xc->size[1])] = H_xm->data[i6 +
        H_xm->size[0] * i5];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  i5 = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, i5, (int)sizeof(double));

  //  jacobian for gravity residual
  i5 = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, i5, (int)sizeof(double));

  //  jacobian for pressure residual
  ib = 0;
  cr = 0;
  ic = 0;
  R_p_size_idx_1 = 0;
  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (i5 = 0; i5 < 4; i5++) {
    d[i5] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  emxInit_real_T(&r3, 2);
  eye((double)indMeas_size[0], r3);
  kron(r3->data, r3->size, d, R_v_data, R_v_size);
  emxFree_real_T(&r3);
  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
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
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_anchorRot[i5 + 3 * i6] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          b_anchorRot[i5 + 3 * i6] += R_bc[nm1d2 + 3 * i5] *
            b_IMU_measurements[nm1d2 + 3 * i6];
        }
      }

      for (i6 = 0; i6 < 3; i6++) {
        dv5[i5 + 3 * i6] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv5[i5 + 3 * i6] += b_anchorRot[i5 + 3 * nm1d2] * R_cw[i6 + 3 * nm1d2];
        }
      }
    }

    QuatFromRotJ(dv5, d);
    ar = 3;
    for (i5 = 0; i5 < 3; i5++) {
      r_g_data[i5] = d[i5];
    }

    i5 = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    emxEnsureCapacity((emxArray__common *)H_g, i5, (int)sizeof(double));
    br = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    for (i5 = 0; i5 < br; i5++) {
      H_g->data[i5] = 0.0;
    }

    memset(&R_cw[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      R_cw[k + 3 * k] = 1.0;
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        H_g->data[i6 + H_g->size[0] * (3 + i5)] = R_cw[i6 + 3 * i5];
      }
    }

    ib = 3;
    cr = 3;
    for (i5 = 0; i5 < 9; i5++) {
      R_g_data[i5] = c_noiseParameters_orientation_n * (double)b[i5];
    }
  }

  if (b_VIOParameters.use_pressure) {
    i5 = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    emxEnsureCapacity((emxArray__common *)H_p, i5, (int)sizeof(double));
    br = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
                b_VIOParameters.num_points_per_anchor));
    for (i5 = 0; i5 < br; i5++) {
      H_p->data[i5] = 0.0;
    }

    H_p->data[2] = 1.0;
    ic = 1;
    R_p_size_idx_1 = 1;
    R_p_data[0] = noiseParameters_pressure_noise;
    ia = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (b_VIOParameters.use_orientation) {
    if (b_VIOParameters.use_pressure) {
      r_size[0] = (z_size_idx_0 + ar) + ia;
      for (i5 = 0; i5 < z_size_idx_0; i5++) {
        r_data[i5] = z_data[i5];
      }

      for (i5 = 0; i5 < ar; i5++) {
        r_data[i5 + z_size_idx_0] = r_g_data[i5];
      }

      i5 = 0;
      while (i5 <= ia - 1) {
        r_data[z_size_idx_0 + ar] = r_p_data[0];
        i5 = 1;
      }

      i5 = H->size[0] * H->size[1];
      H->size[0] = (H_v->size[0] + H_g->size[0]) + H_p->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i5, (int)sizeof(double));
      br = H_v->size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = H_v->size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          H->data[i6 + H->size[0] * i5] = H_v->data[i6 + H_v->size[0] * i5];
        }
      }

      br = H_g->size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = H_g->size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          H->data[(i6 + H_v->size[0]) + H->size[0] * i5] = H_g->data[i6 +
            H_g->size[0] * i5];
        }
      }

      br = H_p->size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = H_p->size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          H->data[((i6 + H_v->size[0]) + H_g->size[0]) + H->size[0] * i5] =
            H_p->data[i6 + H_p->size[0] * i5];
        }
      }

      i5 = (R_v_size[0] + ib) + ib;
      R_size[0] = i5;
      R_size[1] = i5;
      br = i5 * i5;
      for (i5 = 0; i5 < br; i5++) {
        R_data[i5] = 0.0;
      }

      //  faster than blkdiag
      br = R_v_size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = R_v_size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          R_data[i6 + R_size[0] * i5] = R_v_data[i6 + R_v_size[0] * i5];
        }
      }

      br = ib - 1;
      for (i5 = 0; i5 <= br; i5++) {
        tmp_data[i5] = (int)((double)R_v_size[0] + (1.0 + (double)i5)) - 1;
      }

      br = ib - 1;
      for (i5 = 0; i5 <= br; i5++) {
        b_tmp_data[i5] = (int)((double)R_v_size[0] + (1.0 + (double)i5)) - 1;
      }

      for (i5 = 0; i5 < cr; i5++) {
        for (i6 = 0; i6 < ib; i6++) {
          R_data[tmp_data[i6] + R_size[0] * b_tmp_data[i5]] = R_g_data[i6 + ib *
            i5];
        }
      }

      i5 = (R_v_size[0] + ic) + 1;
      if (i5 > (R_v_size[0] + ib) + ic) {
        i5 = 0;
      } else {
        i5--;
      }

      nm1d2 = (signed char)((signed char)R_v_size[0] + ib);
      br = ib - 1;
      for (i6 = 0; i6 <= br; i6++) {
        tmp_data[i6] = nm1d2 + i6;
      }

      i6 = 0;
      while (i6 <= R_p_size_idx_1 - 1) {
        i6 = 0;
        while (i6 <= ic - 1) {
          R_data[tmp_data[0] + R_size[0] * i5] = R_p_data[0];
          i6 = 1;
        }

        i6 = 1;
      }
    } else {
      r_size[0] = z_size_idx_0 + ar;
      for (i5 = 0; i5 < z_size_idx_0; i5++) {
        r_data[i5] = z_data[i5];
      }

      for (i5 = 0; i5 < ar; i5++) {
        r_data[i5 + z_size_idx_0] = r_g_data[i5];
      }

      i5 = H->size[0] * H->size[1];
      H->size[0] = H_v->size[0] + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i5, (int)sizeof(double));
      br = H_v->size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = H_v->size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          H->data[i6 + H->size[0] * i5] = H_v->data[i6 + H_v->size[0] * i5];
        }
      }

      br = H_g->size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = H_g->size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          H->data[(i6 + H_v->size[0]) + H->size[0] * i5] = H_g->data[i6 +
            H_g->size[0] * i5];
        }
      }

      i5 = R_v_size[0] + ib;
      R_size[0] = i5;
      R_size[1] = i5;
      br = i5 * i5;
      for (i5 = 0; i5 < br; i5++) {
        R_data[i5] = 0.0;
      }

      //  faster than blkdiag
      br = R_v_size[1];
      for (i5 = 0; i5 < br; i5++) {
        nm1d2 = R_v_size[0];
        for (i6 = 0; i6 < nm1d2; i6++) {
          R_data[i6 + R_size[0] * i5] = R_v_data[i6 + R_v_size[0] * i5];
        }
      }

      br = ib - 1;
      for (i5 = 0; i5 <= br; i5++) {
        tmp_data[i5] = (int)((double)R_v_size[0] + (1.0 + (double)i5)) - 1;
      }

      br = ib - 1;
      for (i5 = 0; i5 <= br; i5++) {
        b_tmp_data[i5] = (int)((double)R_v_size[0] + (1.0 + (double)i5)) - 1;
      }

      for (i5 = 0; i5 < cr; i5++) {
        for (i6 = 0; i6 < ib; i6++) {
          R_data[tmp_data[i6] + R_size[0] * b_tmp_data[i5]] = R_g_data[i6 + ib *
            i5];
        }
      }
    }
  } else if (b_VIOParameters.use_pressure) {
    r_size[0] = z_size_idx_0 + ia;
    for (i5 = 0; i5 < z_size_idx_0; i5++) {
      r_data[i5] = z_data[i5];
    }

    i5 = 0;
    while (i5 <= ia - 1) {
      r_data[z_size_idx_0] = r_p_data[0];
      i5 = 1;
    }

    i5 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0] + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i5, (int)sizeof(double));
    br = H_v->size[1];
    for (i5 = 0; i5 < br; i5++) {
      nm1d2 = H_v->size[0];
      for (i6 = 0; i6 < nm1d2; i6++) {
        H->data[i6 + H->size[0] * i5] = H_v->data[i6 + H_v->size[0] * i5];
      }
    }

    br = H_p->size[1];
    for (i5 = 0; i5 < br; i5++) {
      nm1d2 = H_p->size[0];
      for (i6 = 0; i6 < nm1d2; i6++) {
        H->data[(i6 + H_v->size[0]) + H->size[0] * i5] = H_p->data[i6 +
          H_p->size[0] * i5];
      }
    }

    i5 = R_v_size[0] + ic;
    R_size[0] = i5;
    R_size[1] = i5;
    br = i5 * i5;
    for (i5 = 0; i5 < br; i5++) {
      R_data[i5] = 0.0;
    }

    //  faster than blkdiag
    br = R_v_size[1];
    for (i5 = 0; i5 < br; i5++) {
      nm1d2 = R_v_size[0];
      for (i6 = 0; i6 < nm1d2; i6++) {
        R_data[i6 + R_size[0] * i5] = R_v_data[i6 + R_v_size[0] * i5];
      }
    }

    br = ic - 1;
    i5 = 0;
    while (i5 <= br) {
      c_tmp_data[0] = (R_v_size[0] + 1) - 1;
      i5 = 1;
    }

    br = ic - 1;
    i5 = 0;
    while (i5 <= br) {
      d_tmp_data[0] = (R_v_size[0] + 1) - 1;
      i5 = 1;
    }

    i5 = 0;
    while (i5 <= R_p_size_idx_1 - 1) {
      i5 = 0;
      while (i5 <= ic - 1) {
        R_data[c_tmp_data[0] + R_size[0] * d_tmp_data[0]] = R_p_data[0];
        i5 = 1;
      }

      i5 = 1;
    }
  } else {
    r_size[0] = z_size_idx_0;
    for (i5 = 0; i5 < z_size_idx_0; i5++) {
      r_data[i5] = z_data[i5];
    }

    i5 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i5, (int)sizeof(double));
    br = H_v->size[0] * H_v->size[1];
    for (i5 = 0; i5 < br; i5++) {
      H->data[i5] = H_v->data[i5];
    }

    R_size[0] = R_v_size[0];
    R_size[1] = R_v_size[1];
    br = R_v_size[0] * R_v_size[1];
    for (i5 = 0; i5 < br; i5++) {
      R_data[i5] = R_v_data[i5];
    }
  }

  emxFree_real_T(&H_p);
  emxFree_real_T(&H_g);
  emxFree_real_T(&H_v);
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
//                const double IMU_measurements[23]
//                double height_offset_pressure
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
                noiseParameters_pressure_noise, const double IMU_measurements[23],
                double height_offset_pressure, const VIOParameters
                b_VIOParameters, double r_data[], int r_size[1], emxArray_real_T
                *H, double h_u[2], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ib;
  int n;
  emxArray_real_T *H_xc;
  double b_map[3];
  double h_ci_l[3];
  int br;
  double h_cin_l[3];
  int nm1d2;
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
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *b_y;
  double j_cameraparams_CameraParameters[4];
  double dv2[6];
  double c_y[6];
  emxArray_real_T *H_v;
  double v[2];
  int ic;
  int ar;
  int ia;
  double b_z;
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size_idx_1;
  int R_p_size_idx_0;
  int R_p_size_idx_1;
  double R_v[4];
  double b_IMU_measurements[9];
  double r_g_data[4];
  double R_g_data[9];
  double R_p_data[1];
  double r_p_data[1];
  signed char tmp_data[3];
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
  ib = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = 2;
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  n = (int)(b_VIOParameters.num_anchors * (6.0 +
             b_VIOParameters.num_points_per_anchor)) << 1;
  for (ib = 0; ib < n; ib++) {
    H_xm->data[ib] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  n = (int)errorStateSize << 1;
  for (ib = 0; ib < n; ib++) {
    H_xc->data[ib] = 0.0;
  }

  for (ib = 0; ib < 3; ib++) {
    b_map[ib] = map->data[ib + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ib];
  }

  for (ib = 0; ib < 3; ib++) {
    h_ci_l[ib] = 0.0;
    for (br = 0; br < 3; br++) {
      h_ci_l[ib] += R_cw[ib + 3 * br] * b_map[br];
    }
  }

  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  predictMeasurement_left(h_ci_l, c_cameraparams_CameraParameters,
    d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, h_u);
  anew = (indMeas - 1.0) * 2.0;
  for (ib = 0; ib < 2; ib++) {
    z[ib] = z_all_l[(int)(anew + (1.0 + (double)ib)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  n = (int)errorStateSize;
  ib = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  for (ib = 0; ib < n; ib++) {
    r0->data[ib] = ib;
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
  for (ib = 0; ib < 2; ib++) {
    for (br = 0; br < 2; br++) {
      g_cameraparams_CameraParameters[ib + (br << 1)] = 0.0;
      for (n = 0; n < 2; n++) {
        g_cameraparams_CameraParameters[ib + (br << 1)] +=
          f_cameraparams_CameraParameters[ib + (n << 1)] * d[n + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      h_cameraparams_CameraParameters[ib + (br << 1)] = 0.0;
      for (n = 0; n < 2; n++) {
        h_cameraparams_CameraParameters[ib + (br << 1)] +=
          g_cameraparams_CameraParameters[ib + (n << 1)] * dv0[n + (br << 1)];
      }
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * ib] = -R_cw[br + 3 * ib];
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
  for (ib = 0; ib < 6; ib++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 2; ib++) {
    for (br = 0; br < 12; br++) {
      i_cameraparams_CameraParameters[ib + (br << 1)] = 0.0;
      for (n = 0; n < 3; n++) {
        i_cameraparams_CameraParameters[ib + (br << 1)] +=
          h_cameraparams_CameraParameters[ib + (n << 1)] * b_R_cw[n + 3 * br];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ib = 0; ib < nm1d2; ib++) {
    for (br = 0; br < 2; br++) {
      H_xc->data[br + H_xc->size[0] * r0->data[ib]] =
        i_cameraparams_CameraParameters[br + (ib << 1)];
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
  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += anchorRot[br + 3 * ib] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    y[ib] = d1 / anew;
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
  for (ib = 0; ib < 3; ib++) {
    for (br = 0; br < 3; br++) {
      b_anchorRot[br + 3 * ib] = -anchorRot[ib + 3 * br];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += b_anchorRot[ib + 3 * br] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    b_map[ib] = d1 / anew;
  }

  emxInit_real_T(&H_iy, 2);
  n = (int)(b_VIOParameters.num_points_per_anchor - featureAnchorIdx->data[(int)
            indMeas - 1]);
  ib = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + n) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
  for (ib = 0; ib < 3; ib++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ib] = b[br + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * (ib + 3)] = -dv1[br + 3 * ib];
    }
  }

  for (ib = 0; ib < nm1d2; ib++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    H_iy->data[ib + H_iy->size[0] * (6 + nm1d2)] = b_map[ib];
  }

  for (ib = 0; ib < n; ib++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ((ib + nm1d2) + 7)] = 0.0;
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

  emxInit_real_T(&b_y, 2);
  ib = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)b_y, ib, (int)sizeof(double));
  if (n + 1 > 0) {
    b_y->data[0] = anew;
    if (n + 1 > 1) {
      b_y->data[n] = apnd;
      nm1d2 = (n + (n < 0)) >> 1;
      for (br = 1; br < nm1d2; br++) {
        b_y->data[br] = anew + (double)br;
        b_y->data[n - br] = apnd - (double)br;
      }

      if (nm1d2 << 1 == n) {
        b_y->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = anew + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  anew = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 +
    b_VIOParameters.num_points_per_anchor);
  ib = r0->size[0];
  r0->size[0] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  n = b_y->size[1];
  for (ib = 0; ib < n; ib++) {
    r0->data[ib] = (int)(anew + b_y->data[b_y->size[0] * ib]) - 1;
  }

  emxFree_real_T(&b_y);
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
  for (ib = 0; ib < 2; ib++) {
    for (br = 0; br < 2; br++) {
      f_cameraparams_CameraParameters[ib + (br << 1)] = 0.0;
      for (n = 0; n < 2; n++) {
        f_cameraparams_CameraParameters[ib + (br << 1)] +=
          j_cameraparams_CameraParameters[ib + (n << 1)] * d[n + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      h_cameraparams_CameraParameters[ib + (br << 1)] = 0.0;
      for (n = 0; n < 2; n++) {
        h_cameraparams_CameraParameters[ib + (br << 1)] +=
          f_cameraparams_CameraParameters[ib + (n << 1)] * dv2[n + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      c_y[ib + (br << 1)] = 0.0;
      for (n = 0; n < 3; n++) {
        c_y[ib + (br << 1)] += h_cameraparams_CameraParameters[ib + (n << 1)] *
          R_cw[n + 3 * br];
      }
    }
  }

  emxInit_real_T(&H_v, 2);
  v[1] = H_iy->size[1];
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[1] = (int)v[1];
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  n = (int)v[1] << 1;
  for (ib = 0; ib < n; ib++) {
    H_v->data[ib] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (n = 0; n <= nm1d2; n += 2) {
    for (ic = n; ic + 1 <= n + 2; ic++) {
      H_v->data[ic] = 0.0;
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
          H_v->data[ic] += H_iy->data[ib] * c_y[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  emxFree_real_T(&H_iy);
  n = H_v->size[1];
  for (ib = 0; ib < n; ib++) {
    for (br = 0; br < 2; br++) {
      H_xm->data[br + H_xm->size[0] * r0->data[ib]] = H_v->data[br + H_v->size[0]
        * ib];
    }
  }

  emxFree_int32_T(&r0);
  for (ib = 0; ib < 2; ib++) {
    b_z = z[ib] - h_u[ib];
    z[ib] = b_z;
  }

  //  residual with respect to camera measurements
  ar = 0;

  //  gravity residual
  ia = 0;

  //  pressure residual
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  n = H_xc->size[1];
  for (ib = 0; ib < n; ib++) {
    for (br = 0; br < 2; br++) {
      H_v->data[br + H_v->size[0] * ib] = H_xc->data[br + H_xc->size[0] * ib];
    }
  }

  n = H_xm->size[1];
  for (ib = 0; ib < n; ib++) {
    for (br = 0; br < 2; br++) {
      H_v->data[br + H_v->size[0] * (ib + H_xc->size[1])] = H_xm->data[br +
        H_xm->size[0] * ib];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  ib = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, ib, (int)sizeof(double));

  //  jacobian for gravity residual
  ib = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, ib, (int)sizeof(double));

  //  jacobian for pressure residual
  ic = 0;
  R_g_size_idx_1 = 0;
  R_p_size_idx_0 = 0;
  R_p_size_idx_1 = 0;
  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ib = 0; ib < 4; ib++) {
    d[ib] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  nm1d2 = -1;
  for (n = 0; n < 2; n++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_v[nm1d2] = d[br + (n << 1)];
    }
  }

  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
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
    for (ib = 0; ib < 3; ib++) {
      for (br = 0; br < 3; br++) {
        b_anchorRot[ib + 3 * br] = 0.0;
        for (n = 0; n < 3; n++) {
          b_anchorRot[ib + 3 * br] += R_bc[n + 3 * ib] * b_IMU_measurements[n +
            3 * br];
        }
      }

      for (br = 0; br < 3; br++) {
        dv1[ib + 3 * br] = 0.0;
        for (n = 0; n < 3; n++) {
          dv1[ib + 3 * br] += b_anchorRot[ib + 3 * n] * R_cw[br + 3 * n];
        }
      }
    }

    QuatFromRotJ(dv1, f_cameraparams_CameraParameters);
    ar = 3;
    for (ib = 0; ib < 3; ib++) {
      r_g_data[ib] = f_cameraparams_CameraParameters[ib];
    }

    ib = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    emxEnsureCapacity((emxArray__common *)H_g, ib, (int)sizeof(double));
    n = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    for (ib = 0; ib < n; ib++) {
      H_g->data[ib] = 0.0;
    }

    memset(&R_cw[0], 0, 9U * sizeof(double));
    for (br = 0; br < 3; br++) {
      R_cw[br + 3 * br] = 1.0;
    }

    for (ib = 0; ib < 3; ib++) {
      for (br = 0; br < 3; br++) {
        H_g->data[br + H_g->size[0] * (3 + ib)] = R_cw[br + 3 * ib];
      }
    }

    ic = 3;
    R_g_size_idx_1 = 3;
    for (ib = 0; ib < 9; ib++) {
      R_g_data[ib] = c_noiseParameters_orientation_n * (double)b[ib];
    }
  }

  if (b_VIOParameters.use_pressure) {
    ib = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
      b_VIOParameters.num_points_per_anchor));
    emxEnsureCapacity((emxArray__common *)H_p, ib, (int)sizeof(double));
    n = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
               b_VIOParameters.num_points_per_anchor));
    for (ib = 0; ib < n; ib++) {
      H_p->data[ib] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size_idx_0 = 1;
    R_p_size_idx_1 = 1;
    R_p_data[0] = noiseParameters_pressure_noise;
    ia = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (b_VIOParameters.use_orientation) {
    if (b_VIOParameters.use_pressure) {
      r_size[0] = (ar + ia) + 2;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < ar; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = 0;
      while (ib <= ia - 1) {
        r_data[ar + 2] = r_p_data[0];
        ib = 1;
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = (H_g->size[0] + H_p->size[0]) + 2;
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      n = H_v->size[1];
      for (ib = 0; ib < n; ib++) {
        for (br = 0; br < 2; br++) {
          H->data[br + H->size[0] * ib] = H_v->data[br + H_v->size[0] * ib];
        }
      }

      n = H_g->size[1];
      for (ib = 0; ib < n; ib++) {
        nm1d2 = H_g->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[(br + H->size[0] * ib) + 2] = H_g->data[br + H_g->size[0] * ib];
        }
      }

      n = H_p->size[1];
      for (ib = 0; ib < n; ib++) {
        nm1d2 = H_p->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[((br + H_g->size[0]) + H->size[0] * ib) + 2] = H_p->data[br +
            H_p->size[0] * ib];
        }
      }

      ib = (ic + ic) + 2;
      R_size[0] = ib;
      R_size[1] = ib;
      n = ib * ib;
      for (ib = 0; ib < n; ib++) {
        R_data[ib] = 0.0;
      }

      //  faster than blkdiag
      for (ib = 0; ib < 2; ib++) {
        for (br = 0; br < 2; br++) {
          R_data[br + R_size[0] * ib] = R_v[br + (ib << 1)];
        }
      }

      if (3 > 2 + ic) {
        ib = 0;
      } else {
        ib = 2;
      }

      if (3 > 2 + ic) {
        br = 0;
      } else {
        br = 2;
      }

      for (n = 0; n < R_g_size_idx_1; n++) {
        for (nm1d2 = 0; nm1d2 < ic; nm1d2++) {
          R_data[(ib + nm1d2) + R_size[0] * (br + n)] = R_g_data[nm1d2 + ic * n];
        }
      }

      if (R_p_size_idx_0 + 3 > (ic + R_p_size_idx_0) + 2) {
        ib = 1;
      } else {
        ib = R_p_size_idx_0 + 3;
      }

      n = ic - 1;
      for (br = 0; br <= n; br++) {
        tmp_data[br] = (signed char)((ic + br) + 2);
      }

      br = 0;
      while (br <= R_p_size_idx_1 - 1) {
        br = 0;
        while (br <= R_p_size_idx_0 - 1) {
          R_data[tmp_data[0] + R_size[0] * (ib - 1)] = R_p_data[0];
          br = 1;
        }

        br = 1;
      }
    } else {
      r_size[0] = 2 + ar;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < ar; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = 2 + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      n = H_v->size[1];
      for (ib = 0; ib < n; ib++) {
        for (br = 0; br < 2; br++) {
          H->data[br + H->size[0] * ib] = H_v->data[br + H_v->size[0] * ib];
        }
      }

      n = H_g->size[1];
      for (ib = 0; ib < n; ib++) {
        nm1d2 = H_g->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[(br + H->size[0] * ib) + 2] = H_g->data[br + H_g->size[0] * ib];
        }
      }

      R_size[0] = 2 + ic;
      R_size[1] = 2 + ic;
      n = (2 + ic) * (2 + ic);
      for (ib = 0; ib < n; ib++) {
        R_data[ib] = 0.0;
      }

      //  faster than blkdiag
      for (ib = 0; ib < 2; ib++) {
        for (br = 0; br < 2; br++) {
          R_data[br + R_size[0] * ib] = R_v[br + (ib << 1)];
        }
      }

      if (3 > 2 + ic) {
        ib = 0;
      } else {
        ib = 2;
      }

      if (3 > 2 + ic) {
        br = 0;
      } else {
        br = 2;
      }

      for (n = 0; n < R_g_size_idx_1; n++) {
        for (nm1d2 = 0; nm1d2 < ic; nm1d2++) {
          R_data[(ib + nm1d2) + R_size[0] * (br + n)] = R_g_data[nm1d2 + ic * n];
        }
      }
    }
  } else if (b_VIOParameters.use_pressure) {
    r_size[0] = 2 + ia;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib] = z[ib];
    }

    ib = 0;
    while (ib <= ia - 1) {
      r_data[2] = r_p_data[0];
      ib = 1;
    }

    ib = H->size[0] * H->size[1];
    H->size[0] = 2 + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
    n = H_v->size[1];
    for (ib = 0; ib < n; ib++) {
      for (br = 0; br < 2; br++) {
        H->data[br + H->size[0] * ib] = H_v->data[br + H_v->size[0] * ib];
      }
    }

    n = H_p->size[1];
    for (ib = 0; ib < n; ib++) {
      nm1d2 = H_p->size[0];
      for (br = 0; br < nm1d2; br++) {
        H->data[(br + H->size[0] * ib) + 2] = H_p->data[br + H_p->size[0] * ib];
      }
    }

    R_size[0] = 2 + R_p_size_idx_0;
    R_size[1] = 2 + R_p_size_idx_0;
    n = (2 + R_p_size_idx_0) * (2 + R_p_size_idx_0);
    for (ib = 0; ib < n; ib++) {
      R_data[ib] = 0.0;
    }

    //  faster than blkdiag
    for (ib = 0; ib < 2; ib++) {
      for (br = 0; br < 2; br++) {
        R_data[br + R_size[0] * ib] = R_v[br + (ib << 1)];
      }
    }

    if (3 > 2 + R_p_size_idx_0) {
      ib = 0;
    } else {
      ib = 2;
    }

    if (3 > 2 + R_p_size_idx_0) {
      br = 0;
    } else {
      br = 2;
    }

    n = 0;
    while (n <= R_p_size_idx_1 - 1) {
      n = 0;
      while (n <= R_p_size_idx_0 - 1) {
        R_data[ib + R_size[0] * br] = R_p_data[0];
        n = 1;
      }

      n = 1;
    }
  } else {
    r_size[0] = 2;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib] = z[ib];
    }

    ib = H->size[0] * H->size[1];
    H->size[0] = 2;
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
    n = H_v->size[0] * H_v->size[1];
    for (ib = 0; ib < n; ib++) {
      H->data[ib] = H_v->data[ib];
    }

    R_size[0] = 2;
    R_size[1] = 2;
    for (ib = 0; ib < 4; ib++) {
      R_data[ib] = R_v[ib];
    }
  }

  emxFree_real_T(&H_p);
  emxFree_real_T(&H_g);
  emxFree_real_T(&H_v);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
