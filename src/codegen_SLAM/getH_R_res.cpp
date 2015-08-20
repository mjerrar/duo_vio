//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 11:33:45
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "blkdiag.h"
#include "predictMeasurement_stereo.h"
#include "norm.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_left.h"
#include "diag.h"
#include "power.h"
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
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
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
                  e_cameraparams_CameraParameters[2], double numAnchors, double
                  numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                  emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                  *b_m_vect, const double imNoise[2], const double
                  IMU_measurements[23], double height_offset_pressure, double
                  r_data[], int r_size[1], emxArray_real_T *H, double h_u_data[],
                  int h_u_size[1], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  double fx_l;
  double fy_l;
  double k1_l;
  double k2_l;
  double k3_l;
  int i1;
  int ia;
  emxArray_real_T *H_xc;
  int z_size_idx_0;
  double z_data[32];
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r2;
  emxArray_real_T *C;
  emxArray_real_T *y;
  double b_map[3];
  double h_rz[3];
  int ic;
  double h_cin_l[3];
  int kidx;
  double dv4[2];
  int r;
  double anew;
  signed char b_k;
  signed char iv0[2];
  double B[4];
  double b_fx_l[4];
  double c_fx_l[4];
  double dv5[6];
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
  double d2;
  double b_y[3];
  double dv6[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int ar;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double f_fx_l[4];
  double dv7[6];
  double c_y[6];
  int c;
  double b_z_data;
  emxArray_real_T *H_v;
  int r_p_size_idx_0;
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  emxArray_real_T *r3;
  int R_g_size[2];
  int R_p_size[2];
  double A_data[256];
  double dv8[2];
  double R_v_data[1024];
  int R_v_size[2];
  boolean_T x[3];
  boolean_T d_y;
  boolean_T exitg1;
  double r_g_data[3];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv9[9];
  double R_g_data[9];
  double R_p_data[1];
  double r_p_data[1];
  emxArray_real_T b_R_v_data;
  emxArray_real_T b_R_p_data;
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
  fx_l = d_cameraparams_CameraParameters[0];
  fy_l = d_cameraparams_CameraParameters[1];

  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  k1_l = c_cameraparams_CameraParameters[0];
  k2_l = c_cameraparams_CameraParameters[1];
  k3_l = c_cameraparams_CameraParameters[2];
  i1 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, i1, (int)sizeof(double));
  i1 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, i1, (int)sizeof(double));
  ia = (indMeas_size[0] << 1) * (int)(numAnchors * (6.0 + numPointsPerAnchor));
  for (i1 = 0; i1 < ia; i1++) {
    H_xm->data[i1] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  i1 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, i1, (int)sizeof(double));
  i1 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, i1, (int)sizeof(double));
  ia = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (i1 = 0; i1 < ia; i1++) {
    H_xc->data[i1] = 0.0;
  }

  z_size_idx_0 = indMeas_size[0] << 1;
  ia = indMeas_size[0] << 1;
  for (i1 = 0; i1 < ia; i1++) {
    z_data[i1] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  ia = indMeas_size[0] << 1;
  for (i1 = 0; i1 < ia; i1++) {
    h_u_data[i1] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (i1 = 0; i1 < 3; i1++) {
      b_map[i1] = map->data[i1 + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      h_rz[i1] = 0.0;
      for (ic = 0; ic < 3; ic++) {
        h_rz[i1] += R_cw[i1 + 3 * ic] * b_map[ic];
      }
    }

    for (kidx = 0; kidx < 3; kidx++) {
      h_cin_l[kidx] = h_rz[kidx] / h_rz[2];
    }

    predictMeasurement_left(h_rz, c_cameraparams_CameraParameters,
      d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, dv4);
    r = k << 1;
    for (i1 = 0; i1 < 2; i1++) {
      h_u_data[i1 + r] = dv4[i1];
    }

    r = k << 1;
    anew = (indMeas_data[k] - 1.0) * 2.0;
    for (i1 = 0; i1 < 2; i1++) {
      z_data[i1 + r] = z_all_l[(int)(anew + (1.0 + (double)i1)) - 1];
    }

    //     %% computation of H(x)
    b_k = (signed char)((signed char)k << 1);
    for (i1 = 0; i1 < 2; i1++) {
      iv0[i1] = (signed char)(i1 + b_k);
    }

    ia = H_xc->size[1];
    i1 = r2->size[0];
    r2->size[0] = ia;
    emxEnsureCapacity((emxArray__common *)r2, i1, (int)sizeof(int));
    for (i1 = 0; i1 < ia; i1++) {
      r2->data[i1] = i1;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], B);
    b_fx_l[0] = fx_l;
    b_fx_l[2] = 0.0;
    b_fx_l[1] = 0.0;
    b_fx_l[3] = fy_l;
    dv5[0] = 1.0 / h_rz[2];
    dv5[2] = 0.0;
    dv5[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv5[1] = 0.0;
    dv5[3] = 1.0 / h_rz[2];
    dv5[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (i1 = 0; i1 < 2; i1++) {
      for (ic = 0; ic < 2; ic++) {
        c_fx_l[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          c_fx_l[i1 + (ic << 1)] += b_fx_l[i1 + (r << 1)] * B[r + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        d_fx_l[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          d_fx_l[i1 + (ic << 1)] += c_fx_l[i1 + (r << 1)] * dv5[r + (ic << 1)];
        }
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (ic = 0; ic < 3; ic++) {
        b_R_cw[ic + 3 * i1] = -R_cw[ic + 3 * i1];
      }
    }

    b_R_cw[9] = 0.0;
    b_R_cw[12] = -h_rz[2];
    b_R_cw[15] = h_rz[1];
    b_R_cw[10] = h_rz[2];
    b_R_cw[13] = 0.0;
    b_R_cw[16] = -h_rz[0];
    b_R_cw[11] = -h_rz[1];
    b_R_cw[14] = h_rz[0];
    b_R_cw[17] = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      for (ic = 0; ic < 3; ic++) {
        b_R_cw[ic + 3 * (i1 + 6)] = 0.0;
      }
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (ic = 0; ic < 12; ic++) {
        e_fx_l[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          e_fx_l[i1 + (ic << 1)] += d_fx_l[i1 + (r << 1)] * b_R_cw[r + 3 * ic];
        }
      }
    }

    kidx = r2->size[0];
    for (i1 = 0; i1 < kidx; i1++) {
      for (ic = 0; ic < 2; ic++) {
        H_xc->data[iv0[ic] + H_xc->size[0] * r2->data[i1]] = e_fx_l[ic + (i1 <<
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
      (7.0 + numPointsPerAnchor);
    a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
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
    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1];
    for (i1 = 0; i1 < 3; i1++) {
      d2 = 0.0;
      for (ic = 0; ic < 3; ic++) {
        d2 += anchorRot[ic + 3 * i1] * b_m_vect->data[ic + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_y[i1] = d2 / anew;
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1] * b_xt->data[(int)(((stateSize +
      (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0 +
      numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas_data[k]
      - 1]) - 1];
    dv6[0] = 0.0;
    dv6[3] = -b_y[2];
    dv6[6] = b_y[1];
    dv6[1] = b_y[2];
    dv6[4] = 0.0;
    dv6[7] = -b_y[0];
    dv6[2] = -b_y[1];
    dv6[5] = b_y[0];
    dv6[8] = 0.0;
    kidx = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (i1 = 0; i1 < 3; i1++) {
      for (ic = 0; ic < 3; ic++) {
        b_anchorRot[ic + 3 * i1] = -anchorRot[i1 + 3 * ic];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      d2 = 0.0;
      for (ic = 0; ic < 3; ic++) {
        d2 += b_anchorRot[i1 + 3 * ic] * b_m_vect->data[ic + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_map[i1] = d2 / anew;
    }

    r = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k] -
              1]);
    i1 = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (kidx + r) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, i1, (int)sizeof(double));
    for (i1 = 0; i1 < 3; i1++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * i1] = b[ic + 3 * i1];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * (i1 + 3)] = -dv6[ic + 3 * i1];
      }
    }

    for (i1 = 0; i1 < kidx; i1++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * (i1 + 6)] = 0.0;
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      H_iy->data[i1 + H_iy->size[0] * (6 + kidx)] = b_map[i1];
    }

    for (i1 = 0; i1 < r; i1++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * ((i1 + kidx) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (i1 = 0; i1 < 2; i1++) {
      iv0[i1] = (signed char)(i1 + b_k);
    }

    if (rtIsNaN(6.0 + numPointsPerAnchor)) {
      ar = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (6.0 + numPointsPerAnchor < 1.0) {
      ar = -1;
      anew = 1.0;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
      ar = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else {
      anew = 1.0;
      ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
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
        ar = (int)ndbl - 1;
      } else {
        ar = -1;
      }
    }

    i1 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = ar + 1;
    emxEnsureCapacity((emxArray__common *)y, i1, (int)sizeof(double));
    if (ar + 1 > 0) {
      y->data[0] = anew;
      if (ar + 1 > 1) {
        y->data[ar] = apnd;
        kidx = (ar + (ar < 0)) >> 1;
        for (r = 1; r < kidx; r++) {
          y->data[r] = anew + (double)r;
          y->data[ar - r] = apnd - (double)r;
        }

        if (kidx << 1 == ar) {
          y->data[kidx] = (anew + apnd) / 2.0;
        } else {
          y->data[kidx] = anew + (double)kidx;
          y->data[kidx + 1] = apnd - (double)kidx;
        }
      }
    }

    anew = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    i1 = r2->size[0];
    r2->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r2, i1, (int)sizeof(int));
    ia = y->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      r2->data[i1] = (int)(anew + y->data[y->size[0] * i1]) - 1;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], B);
    f_fx_l[0] = fx_l;
    f_fx_l[2] = 0.0;
    f_fx_l[1] = 0.0;
    f_fx_l[3] = fy_l;
    dv7[0] = 1.0 / h_rz[2];
    dv7[2] = 0.0;
    dv7[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv7[1] = 0.0;
    dv7[3] = 1.0 / h_rz[2];
    dv7[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (i1 = 0; i1 < 2; i1++) {
      for (ic = 0; ic < 2; ic++) {
        c_fx_l[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          c_fx_l[i1 + (ic << 1)] += f_fx_l[i1 + (r << 1)] * B[r + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        d_fx_l[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          d_fx_l[i1 + (ic << 1)] += c_fx_l[i1 + (r << 1)] * dv7[r + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        c_y[i1 + (ic << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          c_y[i1 + (ic << 1)] += d_fx_l[i1 + (r << 1)] * R_cw[r + 3 * ic];
        }
      }
    }

    kidx = H_iy->size[1];
    i1 = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, i1, (int)sizeof(double));
    i1 = C->size[0] * C->size[1];
    C->size[1] = kidx;
    emxEnsureCapacity((emxArray__common *)C, i1, (int)sizeof(double));
    ia = kidx << 1;
    for (i1 = 0; i1 < ia; i1++) {
      C->data[i1] = 0.0;
    }

    c = (H_iy->size[1] - 1) << 1;
    for (kidx = 0; kidx <= c; kidx += 2) {
      for (ic = kidx + 1; ic <= kidx + 2; ic++) {
        C->data[ic - 1] = 0.0;
      }
    }

    r = 0;
    for (kidx = 0; kidx <= c; kidx += 2) {
      ar = 0;
      for (i1 = r; i1 + 1 <= r + 3; i1++) {
        if (H_iy->data[i1] != 0.0) {
          ia = ar;
          for (ic = kidx; ic + 1 <= kidx + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[i1] * c_y[ia - 1];
          }
        }

        ar += 2;
      }

      r += 3;
    }

    ia = C->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      for (ic = 0; ic < 2; ic++) {
        H_xm->data[iv0[ic] + H_xm->size[0] * r2->data[i1]] = C->data[ic +
          C->size[0] * i1];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  emxFree_real_T(&H_iy);
  for (i1 = 0; i1 < z_size_idx_0; i1++) {
    b_z_data = z_data[i1] - h_u_data[i1];
    z_data[i1] = b_z_data;
  }

  emxInit_real_T(&H_v, 2);

  //  residual with respect to camera measurements
  k = 0;

  //  gravity residual
  r_p_size_idx_0 = 0;

  //  pressure residual
  i1 = H_v->size[0] * H_v->size[1];
  H_v->size[0] = H_xc->size[0];
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, i1, (int)sizeof(double));
  ia = H_xc->size[1];
  for (i1 = 0; i1 < ia; i1++) {
    ar = H_xc->size[0];
    for (ic = 0; ic < ar; ic++) {
      H_v->data[ic + H_v->size[0] * i1] = H_xc->data[ic + H_xc->size[0] * i1];
    }
  }

  ia = H_xm->size[1];
  for (i1 = 0; i1 < ia; i1++) {
    ar = H_xm->size[0];
    for (ic = 0; ic < ar; ic++) {
      H_v->data[ic + H_v->size[0] * (i1 + H_xc->size[1])] = H_xm->data[ic +
        H_xm->size[0] * i1];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  emxInit_real_T(&r3, 2);
  i1 = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, i1, (int)sizeof(double));

  //  jacobian for gravity residual
  i1 = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, i1, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  eye((double)indMeas_size[0], r3);
  ic = r3->size[0];
  c = r3->size[1];
  ia = r3->size[0] * r3->size[1];
  for (i1 = 0; i1 < ia; i1++) {
    A_data[i1] = r3->data[i1];
  }

  power(imNoise, dv8);
  diag(dv8, B);
  R_v_size[0] = ic << 1;
  R_v_size[1] = c << 1;
  kidx = -1;
  for (r = 1; r <= c; r++) {
    for (ar = 0; ar < 2; ar++) {
      for (i1 = 1; i1 <= ic; i1++) {
        for (ia = 0; ia < 2; ia++) {
          kidx++;
          R_v_data[kidx] = A_data[(i1 + ic * (r - 1)) - 1] * B[ia + (ar << 1)];
        }
      }
    }
  }

  if (gravityUpdate) {
    for (kidx = 0; kidx < 3; kidx++) {
      x[kidx] = (IMU_measurements[kidx + 3] == 0.0);
    }

    d_y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 3)) {
      if (!x[k]) {
        d_y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (!d_y) {
      anew = norm(*(double (*)[3])&IMU_measurements[3]);

      //  normalize the acceleration measurement
      //  the earth-z axis transformed into the body frame
      k = 3;
      for (i1 = 0; i1 < 3; i1++) {
        h_rz[i1] = 0.0;
        for (ic = 0; ic < 3; ic++) {
          h_rz[i1] += R_cw[i1 + 3 * ic] * (double)b_b[ic];
        }

        r_g_data[i1] = IMU_measurements[3 + i1] / anew - h_rz[i1];
      }
    } else {
      k = 3;
      for (kidx = 0; kidx < 3; kidx++) {
        h_rz[kidx] = 0.0;
        r_g_data[kidx] = 0.0;
      }
    }

    i1 = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, i1, (int)sizeof(double));
    ia = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (i1 = 0; i1 < ia; i1++) {
      H_g->data[i1] = 0.0;
    }

    dv9[0] = 0.0;
    dv9[3] = -h_rz[2];
    dv9[6] = h_rz[1];
    dv9[1] = h_rz[2];
    dv9[4] = 0.0;
    dv9[7] = -h_rz[0];
    dv9[2] = -h_rz[1];
    dv9[5] = h_rz[0];
    dv9[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      for (ic = 0; ic < 3; ic++) {
        H_g->data[ic + H_g->size[0] * (3 + i1)] = dv9[ic + 3 * i1];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (i1 = 0; i1 < 9; i1++) {
      R_g_data[i1] = gravAlignNoise * (double)b[i1];
    }
  }

  if (useAirPressure) {
    i1 = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, i1, (int)sizeof(double));
    ia = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (i1 = 0; i1 < ia; i1++) {
      H_p->data[i1] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    r_p_size_idx_0 = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (z_size_idx_0 + k) + r_p_size_idx_0;
      for (i1 = 0; i1 < z_size_idx_0; i1++) {
        r_data[i1] = z_data[i1];
      }

      for (i1 = 0; i1 < k; i1++) {
        r_data[i1 + z_size_idx_0] = r_g_data[i1];
      }

      i1 = 0;
      while (i1 <= r_p_size_idx_0 - 1) {
        r_data[z_size_idx_0 + k] = r_p_data[0];
        i1 = 1;
      }

      i1 = H->size[0] * H->size[1];
      H->size[0] = (H_v->size[0] + H_g->size[0]) + H_p->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i1, (int)sizeof(double));
      ia = H_v->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        ar = H_v->size[0];
        for (ic = 0; ic < ar; ic++) {
          H->data[ic + H->size[0] * i1] = H_v->data[ic + H_v->size[0] * i1];
        }
      }

      ia = H_g->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        ar = H_g->size[0];
        for (ic = 0; ic < ar; ic++) {
          H->data[(ic + H_v->size[0]) + H->size[0] * i1] = H_g->data[ic +
            H_g->size[0] * i1];
        }
      }

      ia = H_p->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        ar = H_p->size[0];
        for (ic = 0; ic < ar; ic++) {
          H->data[((ic + H_v->size[0]) + H_g->size[0]) + H->size[0] * i1] =
            H_p->data[ic + H_p->size[0] * i1];
        }
      }

      output_size(R_v_size, R_g_size, R_p_size, &kidx, &r);
      R_size[0] = kidx;
      R_size[1] = r;
      ia = kidx * r;
      for (i1 = 0; i1 < ia; i1++) {
        R_data[i1] = 0.0;
      }

      if ((R_v_size[0] > 0) && (R_v_size[1] > 0)) {
        ia = R_v_size[1];
        for (i1 = 0; i1 < ia; i1++) {
          ar = R_v_size[0];
          for (ic = 0; ic < ar; ic++) {
            R_data[ic + R_size[0] * i1] = R_v_data[ic + R_v_size[0] * i1];
          }
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        i1 = R_v_size[0] + R_g_size[0];
        if (R_v_size[0] + 1 > i1) {
          i1 = 1;
        } else {
          i1 = R_v_size[0] + 1;
        }

        ic = R_v_size[1] + R_g_size[1];
        if (R_v_size[1] + 1 > ic) {
          ic = 1;
        } else {
          ic = R_v_size[1] + 1;
        }

        ia = R_g_size[1];
        for (r = 0; r < ia; r++) {
          ar = R_g_size[0];
          for (kidx = 0; kidx < ar; kidx++) {
            R_data[((i1 + kidx) + R_size[0] * ((ic + r) - 1)) - 1] =
              R_g_data[kidx + R_g_size[0] * r];
          }
        }
      }

      r = R_v_size[0] + R_g_size[0];
      c = R_v_size[1] + R_g_size[1];
      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ia = R_p_size[1];
        for (i1 = 0; i1 < ia; i1++) {
          ar = R_p_size[0];
          for (ic = 0; ic < ar; ic++) {
            R_data[(r + ic) + R_size[0] * (c + i1)] = R_p_data[ic + R_p_size[0] *
              i1];
          }
        }
      }
    } else {
      r_size[0] = z_size_idx_0 + k;
      for (i1 = 0; i1 < z_size_idx_0; i1++) {
        r_data[i1] = z_data[i1];
      }

      for (i1 = 0; i1 < k; i1++) {
        r_data[i1 + z_size_idx_0] = r_g_data[i1];
      }

      i1 = H->size[0] * H->size[1];
      H->size[0] = H_v->size[0] + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i1, (int)sizeof(double));
      ia = H_v->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        ar = H_v->size[0];
        for (ic = 0; ic < ar; ic++) {
          H->data[ic + H->size[0] * i1] = H_v->data[ic + H_v->size[0] * i1];
        }
      }

      ia = H_g->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        ar = H_g->size[0];
        for (ic = 0; ic < ar; ic++) {
          H->data[(ic + H_v->size[0]) + H->size[0] * i1] = H_g->data[ic +
            H_g->size[0] * i1];
        }
      }

      b_R_v_data.data = (double *)&R_v_data;
      b_R_v_data.size = (int *)&R_v_size;
      b_R_v_data.allocatedSize = 1024;
      b_R_v_data.numDimensions = 2;
      b_R_v_data.canFreeData = false;
      b_R_p_data.data = (double *)&R_g_data;
      b_R_p_data.size = (int *)&R_g_size;
      b_R_p_data.allocatedSize = 9;
      b_R_p_data.numDimensions = 2;
      b_R_p_data.canFreeData = false;
      blkdiag(&b_R_v_data, &b_R_p_data, r3);
      R_size[0] = r3->size[0];
      R_size[1] = r3->size[1];
      ia = r3->size[0] * r3->size[1];
      for (i1 = 0; i1 < ia; i1++) {
        R_data[i1] = r3->data[i1];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = z_size_idx_0 + r_p_size_idx_0;
    for (i1 = 0; i1 < z_size_idx_0; i1++) {
      r_data[i1] = z_data[i1];
    }

    i1 = 0;
    while (i1 <= r_p_size_idx_0 - 1) {
      r_data[z_size_idx_0] = r_p_data[0];
      i1 = 1;
    }

    i1 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0] + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i1, (int)sizeof(double));
    ia = H_v->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      ar = H_v->size[0];
      for (ic = 0; ic < ar; ic++) {
        H->data[ic + H->size[0] * i1] = H_v->data[ic + H_v->size[0] * i1];
      }
    }

    ia = H_p->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      ar = H_p->size[0];
      for (ic = 0; ic < ar; ic++) {
        H->data[(ic + H_v->size[0]) + H->size[0] * i1] = H_p->data[ic +
          H_p->size[0] * i1];
      }
    }

    b_R_v_data.data = (double *)&R_v_data;
    b_R_v_data.size = (int *)&R_v_size;
    b_R_v_data.allocatedSize = 1024;
    b_R_v_data.numDimensions = 2;
    b_R_v_data.canFreeData = false;
    b_R_p_data.data = (double *)&R_p_data;
    b_R_p_data.size = (int *)&R_p_size;
    b_R_p_data.allocatedSize = 1;
    b_R_p_data.numDimensions = 2;
    b_R_p_data.canFreeData = false;
    blkdiag(&b_R_v_data, &b_R_p_data, r3);
    R_size[0] = r3->size[0];
    R_size[1] = r3->size[1];
    ia = r3->size[0] * r3->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      R_data[i1] = r3->data[i1];
    }
  } else {
    r_size[0] = z_size_idx_0;
    for (i1 = 0; i1 < z_size_idx_0; i1++) {
      r_data[i1] = z_data[i1];
    }

    i1 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i1, (int)sizeof(double));
    ia = H_v->size[0] * H_v->size[1];
    for (i1 = 0; i1 < ia; i1++) {
      H->data[i1] = H_v->data[i1];
    }

    R_size[0] = R_v_size[0];
    R_size[1] = R_v_size[1];
    ia = R_v_size[0] * R_v_size[1];
    for (i1 = 0; i1 < ia; i1++) {
      R_data[i1] = R_v_data[i1];
    }
  }

  emxFree_real_T(&r3);
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
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
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
                e_cameraparams_CameraParameters[2], double numAnchors, double
                numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                *b_m_vect, const double imNoise[2], const double
                IMU_measurements[23], double height_offset_pressure, double
                r_data[], int r_size[1], emxArray_real_T *H, double h_u[2],
                double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ar;
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
  double d_y[2];
  int ic;
  int ib;
  int ia;
  double b_z;
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  double R_v[4];
  boolean_T x[3];
  boolean_T e_y;
  boolean_T exitg1;
  double r_g_data[3];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv3[9];
  double R_g_data[9];
  double R_p_data[1];
  double r_p_data[1];
  signed char unnamed_idx_0;
  signed char unnamed_idx_1;
  int tmp_size[2];
  double tmp_data[25];
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
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  n = (int)(numAnchors * (6.0 + numPointsPerAnchor)) << 1;
  for (ar = 0; ar < n; ar++) {
    H_xm->data[ar] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  n = (int)errorStateSize << 1;
  for (ar = 0; ar < n; ar++) {
    H_xc->data[ar] = 0.0;
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
  n = (int)errorStateSize;
  ar = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  for (ar = 0; ar < n; ar++) {
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
    numPointsPerAnchor);
  a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  c_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  d_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  e_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  f_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  g_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  h_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  i_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  j_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  k_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  l_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  m_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  n_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  o_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  p_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  q_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  r_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  s_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  t_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  u_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  v_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  w_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  x_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  y_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
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
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1];
  for (ar = 0; ar < 3; ar++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += anchorRot[br + 3 * ar] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    y[ar] = d1 / anew;
  }

  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1] * b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1]
    - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
    indMeas - 1]) - 1];
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
  n = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas - 1]);
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

  if (rtIsNaN(6.0 + numPointsPerAnchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (6.0 + numPointsPerAnchor < 1.0) {
    n = -1;
    anew = 1.0;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + numPointsPerAnchor;
  } else {
    anew = 1.0;
    ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
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
      n = (int)ndbl - 1;
    } else {
      n = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  ar = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)b_y, ar, (int)sizeof(double));
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

  anew = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 + numPointsPerAnchor);
  ar = r0->size[0];
  r0->size[0] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  n = b_y->size[1];
  for (ar = 0; ar < n; ar++) {
    r0->data[ar] = (int)(anew + b_y->data[b_y->size[0] * ar]) - 1;
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
      c_y[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        c_y[ar + (br << 1)] += h_cameraparams_CameraParameters[ar + (nm1d2 << 1)]
          * R_cw[nm1d2 + 3 * br];
      }
    }
  }

  emxInit_real_T(&H_v, 2);
  d_y[1] = H_iy->size[1];
  ar = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)H_v, ar, (int)sizeof(double));
  ar = H_v->size[0] * H_v->size[1];
  H_v->size[1] = (int)d_y[1];
  emxEnsureCapacity((emxArray__common *)H_v, ar, (int)sizeof(double));
  n = (int)d_y[1] << 1;
  for (ar = 0; ar < n; ar++) {
    H_v->data[ar] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (n = 0; n <= nm1d2; n += 2) {
    for (ic = n + 1; ic <= n + 2; ic++) {
      H_v->data[ic - 1] = 0.0;
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
  for (ar = 0; ar < n; ar++) {
    for (br = 0; br < 2; br++) {
      H_xm->data[br + H_xm->size[0] * r0->data[ar]] = H_v->data[br + H_v->size[0]
        * ar];
    }
  }

  emxFree_int32_T(&r0);
  for (ar = 0; ar < 2; ar++) {
    b_z = z[ar] - h_u[ar];
    z[ar] = b_z;
  }

  //  residual with respect to camera measurements
  ib = 0;

  //  gravity residual
  ia = 0;

  //  pressure residual
  ar = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ar, (int)sizeof(double));
  n = H_xc->size[1];
  for (ar = 0; ar < n; ar++) {
    for (br = 0; br < 2; br++) {
      H_v->data[br + H_v->size[0] * ar] = H_xc->data[br + H_xc->size[0] * ar];
    }
  }

  n = H_xm->size[1];
  for (ar = 0; ar < n; ar++) {
    for (br = 0; br < 2; br++) {
      H_v->data[br + H_v->size[0] * (ar + H_xc->size[1])] = H_xm->data[br +
        H_xm->size[0] * ar];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  ar = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, ar, (int)sizeof(double));

  //  jacobian for gravity residual
  ar = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, ar, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  for (br = 0; br < 2; br++) {
    d_y[br] = imNoise[br] * imNoise[br];
  }

  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = d_y[nm1d2];
  }

  nm1d2 = -1;
  for (n = 0; n < 2; n++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_v[nm1d2] = d[br + (n << 1)];
    }
  }

  if (gravityUpdate) {
    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      x[nm1d2] = (IMU_measurements[nm1d2 + 3] == 0.0);
    }

    e_y = true;
    br = 0;
    exitg1 = false;
    while ((!exitg1) && (br < 3)) {
      if (!x[br]) {
        e_y = false;
        exitg1 = true;
      } else {
        br++;
      }
    }

    if (!e_y) {
      anew = norm(*(double (*)[3])&IMU_measurements[3]);

      //  normalize the acceleration measurement
      //  the earth-z axis transformed into the body frame
      ib = 3;
      for (ar = 0; ar < 3; ar++) {
        h_ci_l[ar] = 0.0;
        for (br = 0; br < 3; br++) {
          h_ci_l[ar] += R_cw[ar + 3 * br] * (double)b_b[br];
        }

        r_g_data[ar] = IMU_measurements[3 + ar] / anew - h_ci_l[ar];
      }
    } else {
      ib = 3;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        h_ci_l[nm1d2] = 0.0;
        r_g_data[nm1d2] = 0.0;
      }
    }

    ar = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, ar, (int)sizeof(double));
    n = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ar = 0; ar < n; ar++) {
      H_g->data[ar] = 0.0;
    }

    dv3[0] = 0.0;
    dv3[3] = -h_ci_l[2];
    dv3[6] = h_ci_l[1];
    dv3[1] = h_ci_l[2];
    dv3[4] = 0.0;
    dv3[7] = -h_ci_l[0];
    dv3[2] = -h_ci_l[1];
    dv3[5] = h_ci_l[0];
    dv3[8] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_g->data[br + H_g->size[0] * (3 + ar)] = dv3[br + 3 * ar];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (ar = 0; ar < 9; ar++) {
      R_g_data[ar] = gravAlignNoise * (double)b[ar];
    }
  }

  if (useAirPressure) {
    ar = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, ar, (int)sizeof(double));
    n = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ar = 0; ar < n; ar++) {
      H_p->data[ar] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    ia = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (ib + ia) + 2;
      for (ar = 0; ar < 2; ar++) {
        r_data[ar] = z[ar];
      }

      for (ar = 0; ar < ib; ar++) {
        r_data[ar + 2] = r_g_data[ar];
      }

      ar = 0;
      while (ar <= ia - 1) {
        r_data[ib + 2] = r_p_data[0];
        ar = 1;
      }

      ar = H->size[0] * H->size[1];
      H->size[0] = (H_g->size[0] + H_p->size[0]) + 2;
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
      n = H_v->size[1];
      for (ar = 0; ar < n; ar++) {
        for (br = 0; br < 2; br++) {
          H->data[br + H->size[0] * ar] = H_v->data[br + H_v->size[0] * ar];
        }
      }

      n = H_g->size[1];
      for (ar = 0; ar < n; ar++) {
        nm1d2 = H_g->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[(br + H->size[0] * ar) + 2] = H_g->data[br + H_g->size[0] * ar];
        }
      }

      n = H_p->size[1];
      for (ar = 0; ar < n; ar++) {
        nm1d2 = H_p->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[((br + H_g->size[0]) + H->size[0] * ar) + 2] = H_p->data[br +
            H_p->size[0] * ar];
        }
      }

      unnamed_idx_0 = (signed char)((R_g_size[0] + R_p_size[0]) + 2);
      unnamed_idx_1 = (signed char)((R_g_size[1] + R_p_size[1]) + 2);
      R_size[0] = unnamed_idx_0;
      R_size[1] = unnamed_idx_1;
      n = unnamed_idx_0 * unnamed_idx_1;
      for (ar = 0; ar < n; ar++) {
        R_data[ar] = 0.0;
      }

      for (ar = 0; ar < 2; ar++) {
        for (br = 0; br < 2; br++) {
          R_data[br + R_size[0] * ar] = R_v[br + (ar << 1)];
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        n = R_g_size[1];
        for (ar = 0; ar < n; ar++) {
          nm1d2 = R_g_size[0];
          for (br = 0; br < nm1d2; br++) {
            R_data[(br + R_size[0] * (2 + ar)) + 2] = R_g_data[br + R_g_size[0] *
              ar];
          }
        }
      }

      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ar = 0;
        while (ar <= 0) {
          ar = 0;
          while (ar <= 0) {
            R_data[(R_g_size[0] + R_size[0] * (R_g_size[1] + 2)) + 2] = 2.0;
            ar = 1;
          }

          ar = 1;
        }
      }
    } else {
      r_size[0] = 2 + ib;
      for (ar = 0; ar < 2; ar++) {
        r_data[ar] = z[ar];
      }

      for (ar = 0; ar < ib; ar++) {
        r_data[ar + 2] = r_g_data[ar];
      }

      ar = H->size[0] * H->size[1];
      H->size[0] = 2 + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
      n = H_v->size[1];
      for (ar = 0; ar < n; ar++) {
        for (br = 0; br < 2; br++) {
          H->data[br + H->size[0] * ar] = H_v->data[br + H_v->size[0] * ar];
        }
      }

      n = H_g->size[1];
      for (ar = 0; ar < n; ar++) {
        nm1d2 = H_g->size[0];
        for (br = 0; br < nm1d2; br++) {
          H->data[(br + H->size[0] * ar) + 2] = H_g->data[br + H_g->size[0] * ar];
        }
      }

      b_blkdiag(R_v, R_g_data, R_g_size, tmp_data, tmp_size);
      R_size[0] = tmp_size[0];
      R_size[1] = tmp_size[1];
      n = tmp_size[0] * tmp_size[1];
      for (ar = 0; ar < n; ar++) {
        R_data[ar] = tmp_data[ar];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = 2 + ia;
    for (ar = 0; ar < 2; ar++) {
      r_data[ar] = z[ar];
    }

    ar = 0;
    while (ar <= ia - 1) {
      r_data[2] = r_p_data[0];
      ar = 1;
    }

    ar = H->size[0] * H->size[1];
    H->size[0] = 2 + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
    n = H_v->size[1];
    for (ar = 0; ar < n; ar++) {
      for (br = 0; br < 2; br++) {
        H->data[br + H->size[0] * ar] = H_v->data[br + H_v->size[0] * ar];
      }
    }

    n = H_p->size[1];
    for (ar = 0; ar < n; ar++) {
      nm1d2 = H_p->size[0];
      for (br = 0; br < nm1d2; br++) {
        H->data[(br + H->size[0] * ar) + 2] = H_p->data[br + H_p->size[0] * ar];
      }
    }

    b_blkdiag(R_v, R_p_data, R_p_size, tmp_data, tmp_size);
    R_size[0] = tmp_size[0];
    R_size[1] = tmp_size[1];
    n = tmp_size[0] * tmp_size[1];
    for (ar = 0; ar < n; ar++) {
      R_data[ar] = tmp_data[ar];
    }
  } else {
    r_size[0] = 2;
    for (ar = 0; ar < 2; ar++) {
      r_data[ar] = z[ar];
    }

    ar = H->size[0] * H->size[1];
    H->size[0] = 2;
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
    n = H_v->size[0] * H_v->size[1];
    for (ar = 0; ar < n; ar++) {
      H->data[ar] = H_v->data[ar];
    }

    R_size[0] = 2;
    R_size[1] = 2;
    for (ar = 0; ar < 4; ar++) {
      R_data[ar] = R_v[ar];
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
