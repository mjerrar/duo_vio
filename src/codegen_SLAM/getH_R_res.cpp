//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_left.h"
#include "blkdiag.h"
#include "norm.h"
#include "kron.h"
#include "eye.h"
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
//                const emxArray_real_T *indMeas
//                const emxArray_real_T *map
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                double b_numAnchors
//                double b_numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double b_imNoise[2]
//                const double IMU_measurements[23]
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_real_T *h_u
//                emxArray_real_T *R
// Return Type  : void
//
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[32], const emxArray_real_T
                *indMeas, const emxArray_real_T *map, const double
                c_cameraparams_CameraParameters[3], const double
                d_cameraparams_CameraParameters[2], const double
                e_cameraparams_CameraParameters[2], double b_numAnchors, double
                b_numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                *b_m_vect, const double b_imNoise[2], const double
                IMU_measurements[23], emxArray_real_T *r, emxArray_real_T *H,
                emxArray_real_T *h_u, emxArray_real_T *R)
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  double fx_l;
  double fy_l;
  double k1_l;
  double k2_l;
  double k3_l;
  int b_r;
  int i7;
  int ia;
  emxArray_real_T *H_xc;
  emxArray_real_T *z;
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r0;
  emxArray_real_T *C;
  emxArray_real_T *y;
  double anew;
  double z_curr_l[2];
  double b_map[3];
  double h_rz[3];
  int ic;
  double h_cin_l[3];
  double dv0[2];
  unsigned int b_k;
  int iv0[2];
  double d[4];
  double b_fx_l[4];
  double c_fx_l[4];
  double dv1[6];
  double d_fx_l[6];
  double b_R_cw[36];
  int br;
  double e_fx_l[24];
  int c_k;
  double ndbl;
  double apnd;
  double cdiff;
  double absb;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double b_stateSize;
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
  double anchorRot[9];
  double b_y[3];
  double dv2[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double f_fx_l[4];
  double dv3[6];
  double c_y[6];
  double d_y[2];
  int c;
  int ib;
  emxArray_real_T *H_v;
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  boolean_T x[3];
  boolean_T e_y;
  boolean_T exitg1;
  double r_g_data[3];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv4[9];
  double R_g_data[9];
  double R_p_data[1];
  emxArray_real_T b_R_p_data;
  b_emxInit_real_T(&H_xm, 2);

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
  b_r = (int)((double)indMeas->size[0] * 2.0);
  i7 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = b_r;
  emxEnsureCapacity((emxArray__common *)H_xm, i7, (int)sizeof(double));
  i7 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(b_numAnchors * (6.0 + b_numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, i7, (int)sizeof(double));
  ia = (int)((double)indMeas->size[0] * 2.0) * (int)(b_numAnchors * (6.0 +
    b_numPointsPerAnchor));
  for (i7 = 0; i7 < ia; i7++) {
    H_xm->data[i7] = 0.0;
  }

  b_emxInit_real_T(&H_xc, 2);
  b_r = (int)((double)indMeas->size[0] * 2.0);
  i7 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = b_r;
  emxEnsureCapacity((emxArray__common *)H_xc, i7, (int)sizeof(double));
  i7 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, i7, (int)sizeof(double));
  ia = (int)((double)indMeas->size[0] * 2.0) * (int)errorStateSize;
  for (i7 = 0; i7 < ia; i7++) {
    H_xc->data[i7] = 0.0;
  }

  emxInit_real_T(&z, 1);
  i7 = z->size[0];
  z->size[0] = (int)((double)indMeas->size[0] * 2.0);
  emxEnsureCapacity((emxArray__common *)z, i7, (int)sizeof(double));
  ia = (int)((double)indMeas->size[0] * 2.0);
  for (i7 = 0; i7 < ia; i7++) {
    z->data[i7] = 0.0;
  }

  i7 = h_u->size[0];
  h_u->size[0] = (int)((double)indMeas->size[0] * 2.0);
  emxEnsureCapacity((emxArray__common *)h_u, i7, (int)sizeof(double));
  ia = (int)((double)indMeas->size[0] * 2.0);
  for (i7 = 0; i7 < ia; i7++) {
    h_u->data[i7] = 0.0;
  }

  k = 0;
  b_emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r0, 1);
  b_emxInit_real_T(&C, 2);
  b_emxInit_real_T(&y, 2);
  while (k <= indMeas->size[0] - 1) {
    anew = (indMeas->data[k] - 1.0) * 2.0;
    for (i7 = 0; i7 < 2; i7++) {
      z_curr_l[i7] = z_all_l[(int)(anew + (1.0 + (double)i7)) - 1];
    }

    b_r = (int)indMeas->data[k];
    for (i7 = 0; i7 < 3; i7++) {
      b_map[i7] = map->data[i7 + map->size[0] * (b_r - 1)] - b_xt->data[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      h_rz[i7] = 0.0;
      for (ic = 0; ic < 3; ic++) {
        h_rz[i7] += R_cw[i7 + 3 * ic] * b_map[ic];
      }
    }

    for (b_r = 0; b_r < 3; b_r++) {
      h_cin_l[b_r] = h_rz[b_r] / h_rz[2];
    }

    predictMeasurement_left(h_rz, c_cameraparams_CameraParameters,
      d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, dv0);
    b_k = ((unsigned int)k << 1) + 1U;
    for (i7 = 0; i7 < 2; i7++) {
      h_u->data[(int)(i7 + b_k) - 1] = dv0[i7];
    }

    b_k = ((unsigned int)k << 1) + 1U;
    for (i7 = 0; i7 < 2; i7++) {
      z->data[(int)(i7 + b_k) - 1] = z_curr_l[i7];
    }

    //     %% computation of H(x)
    b_k = ((unsigned int)k << 1) + 1U;
    for (i7 = 0; i7 < 2; i7++) {
      iv0[i7] = (int)(i7 + b_k) - 1;
    }

    ia = H_xc->size[1];
    i7 = r0->size[0];
    r0->size[0] = ia;
    emxEnsureCapacity((emxArray__common *)r0, i7, (int)sizeof(int));
    for (i7 = 0; i7 < ia; i7++) {
      r0->data[i7] = i7;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    b_fx_l[0] = fx_l;
    b_fx_l[2] = 0.0;
    b_fx_l[1] = 0.0;
    b_fx_l[3] = fy_l;
    dv1[0] = 1.0 / h_rz[2];
    dv1[2] = 0.0;
    dv1[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv1[1] = 0.0;
    dv1[3] = 1.0 / h_rz[2];
    dv1[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (i7 = 0; i7 < 2; i7++) {
      for (ic = 0; ic < 2; ic++) {
        c_fx_l[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          c_fx_l[i7 + (ic << 1)] += b_fx_l[i7 + (br << 1)] * d[br + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        d_fx_l[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          d_fx_l[i7 + (ic << 1)] += c_fx_l[i7 + (br << 1)] * dv1[br + (ic << 1)];
        }
      }
    }

    for (i7 = 0; i7 < 3; i7++) {
      for (ic = 0; ic < 3; ic++) {
        b_R_cw[ic + 3 * i7] = -R_cw[ic + 3 * i7];
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
    for (i7 = 0; i7 < 6; i7++) {
      for (ic = 0; ic < 3; ic++) {
        b_R_cw[ic + 3 * (i7 + 6)] = 0.0;
      }
    }

    for (i7 = 0; i7 < 2; i7++) {
      for (ic = 0; ic < 12; ic++) {
        e_fx_l[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 3; br++) {
          e_fx_l[i7 + (ic << 1)] += d_fx_l[i7 + (br << 1)] * b_R_cw[br + 3 * ic];
        }
      }
    }

    c_k = r0->size[0];
    for (i7 = 0; i7 < c_k; i7++) {
      for (ic = 0; ic < 2; ic++) {
        H_xc->data[iv0[ic] + H_xc->size[0] * r0->data[i7]] = e_fx_l[ic + (i7 <<
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
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    ndbl = b_xt->data[(int)(anew + 4.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    apnd = b_xt->data[(int)(anew + 5.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    cdiff = b_xt->data[(int)(anew + 6.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    absb = b_xt->data[(int)(anew + 7.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    a = b_xt->data[(int)(anew + 4.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    b_a = b_xt->data[(int)(anew + 5.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    c_a = b_xt->data[(int)(anew + 6.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    d_a = b_xt->data[(int)(anew + 7.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    e_a = b_xt->data[(int)(anew + 4.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    f_a = b_xt->data[(int)(anew + 5.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    g_a = b_xt->data[(int)(anew + 6.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    h_a = b_xt->data[(int)(anew + 7.0) - 1];
    anew = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0
      + b_numPointsPerAnchor);
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0)
      * (7.0 + b_numPointsPerAnchor);
    anchorRot[0] = ((ndbl * ndbl - apnd * apnd) - cdiff * cdiff) + absb * absb;
    anchorRot[3] = 2.0 * (b_xt->data[(int)(anew + 4.0) - 1] * b_xt->data[(int)
                          (b_stateSize + 5.0) - 1] + b_xt->data[(int)
                          (c_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (d_stateSize + 7.0) - 1]);
    anchorRot[6] = 2.0 * (b_xt->data[(int)(e_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(f_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (g_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (h_stateSize + 7.0) - 1]);
    anchorRot[1] = 2.0 * (b_xt->data[(int)(i_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(j_stateSize + 5.0) - 1] - b_xt->data[(int)
                          (k_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (l_stateSize + 7.0) - 1]);
    anchorRot[4] = ((-(a * a) + b_a * b_a) - c_a * c_a) + d_a * d_a;
    anchorRot[7] = 2.0 * (b_xt->data[(int)(m_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(n_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (o_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (p_stateSize + 7.0) - 1]);
    anchorRot[2] = 2.0 * (b_xt->data[(int)(q_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(r_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (s_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (t_stateSize + 7.0) - 1]);
    anchorRot[5] = 2.0 * (b_xt->data[(int)(u_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(v_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (w_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (x_stateSize + 7.0) - 1]);
    anchorRot[8] = ((-(e_a * e_a) - f_a * f_a) + g_a * g_a) + h_a * h_a;
    b_r = (int)indMeas->data[k];
    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas->data[k]
      - 1] - 1.0) * (7.0 + b_numPointsPerAnchor)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas->data[k] - 1]) - 1];
    for (i7 = 0; i7 < 3; i7++) {
      ndbl = 0.0;
      for (ic = 0; ic < 3; ic++) {
        ndbl += anchorRot[ic + 3 * i7] * b_m_vect->data[ic + b_m_vect->size[0] *
          (b_r - 1)];
      }

      b_y[i7] = ndbl / anew;
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas->data[k]
      - 1] - 1.0) * (7.0 + b_numPointsPerAnchor)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas->data[k] - 1]) - 1] * b_xt->data[(int)
      (((stateSize + (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (7.0 +
          b_numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
       indMeas->data[k] - 1]) - 1];
    dv2[0] = 0.0;
    dv2[3] = -b_y[2];
    dv2[6] = b_y[1];
    dv2[1] = b_y[2];
    dv2[4] = 0.0;
    dv2[7] = -b_y[0];
    dv2[2] = -b_y[1];
    dv2[5] = b_y[0];
    dv2[8] = 0.0;
    c_k = (int)(featureAnchorIdx->data[(int)indMeas->data[k] - 1] - 1.0);
    b_r = (int)indMeas->data[k];
    for (i7 = 0; i7 < 3; i7++) {
      for (ic = 0; ic < 3; ic++) {
        b_anchorRot[ic + 3 * i7] = -anchorRot[i7 + 3 * ic];
      }
    }

    for (i7 = 0; i7 < 3; i7++) {
      ndbl = 0.0;
      for (ic = 0; ic < 3; ic++) {
        ndbl += b_anchorRot[i7 + 3 * ic] * b_m_vect->data[ic + b_m_vect->size[0]
          * (b_r - 1)];
      }

      b_map[i7] = ndbl / anew;
    }

    b_r = (int)(b_numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas->
                data[k] - 1]);
    i7 = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (c_k + b_r) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, i7, (int)sizeof(double));
    for (i7 = 0; i7 < 3; i7++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * i7] = b[ic + 3 * i7];
      }
    }

    for (i7 = 0; i7 < 3; i7++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * (i7 + 3)] = -dv2[ic + 3 * i7];
      }
    }

    for (i7 = 0; i7 < c_k; i7++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * (i7 + 6)] = 0.0;
      }
    }

    for (i7 = 0; i7 < 3; i7++) {
      H_iy->data[i7 + H_iy->size[0] * (6 + c_k)] = b_map[i7];
    }

    for (i7 = 0; i7 < b_r; i7++) {
      for (ic = 0; ic < 3; ic++) {
        H_iy->data[ic + H_iy->size[0] * ((i7 + c_k) + 7)] = 0.0;
      }
    }

    b_k = ((unsigned int)k << 1) + 1U;
    for (i7 = 0; i7 < 2; i7++) {
      iv0[i7] = (int)(i7 + b_k) - 1;
    }

    if (rtIsNaN(6.0 + b_numPointsPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + b_numPointsPerAnchor;
    } else if (rtIsInf(6.0 + b_numPointsPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + b_numPointsPerAnchor;
    } else {
      anew = 1.0;
      ndbl = floor(((6.0 + b_numPointsPerAnchor) - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - (6.0 + b_numPointsPerAnchor);
      absb = fabs(6.0 + b_numPointsPerAnchor);
      if ((1.0 >= absb) || rtIsNaN(absb)) {
        absb = 1.0;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = 6.0 + b_numPointsPerAnchor;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      br = (int)ndbl - 1;
    }

    i7 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = br + 1;
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
    y->data[0] = anew;
    if (br + 1 > 1) {
      y->data[br] = apnd;
      b_r = (br + (br < 0)) >> 1;
      for (c_k = 1; c_k < b_r; c_k++) {
        y->data[c_k] = anew + (double)c_k;
        y->data[br - c_k] = apnd - (double)c_k;
      }

      if (b_r << 1 == br) {
        y->data[b_r] = (anew + apnd) / 2.0;
      } else {
        y->data[b_r] = anew + (double)b_r;
        y->data[b_r + 1] = apnd - (double)b_r;
      }
    }

    anew = (anchorIdx->data[(int)indMeas->data[k] - 1] - 1.0) * (6.0 +
      b_numPointsPerAnchor);
    i7 = r0->size[0];
    r0->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r0, i7, (int)sizeof(int));
    ia = y->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      r0->data[i7] = (int)(anew + y->data[y->size[0] * i7]) - 1;
    }

    Ch_dn_To_h_un(k1_l, k2_l, k3_l, h_cin_l[0], h_cin_l[1], d);
    f_fx_l[0] = fx_l;
    f_fx_l[2] = 0.0;
    f_fx_l[1] = 0.0;
    f_fx_l[3] = fy_l;
    dv3[0] = 1.0 / h_rz[2];
    dv3[2] = 0.0;
    dv3[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv3[1] = 0.0;
    dv3[3] = 1.0 / h_rz[2];
    dv3[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (i7 = 0; i7 < 2; i7++) {
      for (ic = 0; ic < 2; ic++) {
        c_fx_l[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          c_fx_l[i7 + (ic << 1)] += f_fx_l[i7 + (br << 1)] * d[br + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        d_fx_l[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 2; br++) {
          d_fx_l[i7 + (ic << 1)] += c_fx_l[i7 + (br << 1)] * dv3[br + (ic << 1)];
        }
      }

      for (ic = 0; ic < 3; ic++) {
        c_y[i7 + (ic << 1)] = 0.0;
        for (br = 0; br < 3; br++) {
          c_y[i7 + (ic << 1)] += d_fx_l[i7 + (br << 1)] * R_cw[br + 3 * ic];
        }
      }
    }

    d_y[1] = H_iy->size[1];
    i7 = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, i7, (int)sizeof(double));
    i7 = C->size[0] * C->size[1];
    C->size[1] = (int)d_y[1];
    emxEnsureCapacity((emxArray__common *)C, i7, (int)sizeof(double));
    ia = (int)d_y[1] << 1;
    for (i7 = 0; i7 < ia; i7++) {
      C->data[i7] = 0.0;
    }

    c = (H_iy->size[1] - 1) << 1;
    for (c_k = 0; c_k <= c; c_k += 2) {
      for (ic = c_k + 1; ic <= c_k + 2; ic++) {
        C->data[ic - 1] = 0.0;
      }
    }

    br = 0;
    for (c_k = 0; c_k <= c; c_k += 2) {
      b_r = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (H_iy->data[ib] != 0.0) {
          ia = b_r;
          for (ic = c_k; ic + 1 <= c_k + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[ib] * c_y[ia - 1];
          }
        }

        b_r += 2;
      }

      br += 3;
    }

    ia = C->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      for (ic = 0; ic < 2; ic++) {
        H_xm->data[iv0[ic] + H_xm->size[0] * r0->data[i7]] = C->data[ic +
          C->size[0] * i7];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r0);
  emxFree_real_T(&H_iy);
  i7 = z->size[0];
  emxEnsureCapacity((emxArray__common *)z, i7, (int)sizeof(double));
  ia = z->size[0];
  for (i7 = 0; i7 < ia; i7++) {
    z->data[i7] -= h_u->data[i7];
  }

  b_emxInit_real_T(&H_v, 2);

  //  residual with respect to camera measurements
  c_k = 0;

  //  gravity residual
  //  pressure residual
  i7 = H_v->size[0] * H_v->size[1];
  H_v->size[0] = H_xc->size[0];
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, i7, (int)sizeof(double));
  ia = H_xc->size[1];
  for (i7 = 0; i7 < ia; i7++) {
    ib = H_xc->size[0];
    for (ic = 0; ic < ib; ic++) {
      H_v->data[ic + H_v->size[0] * i7] = H_xc->data[ic + H_xc->size[0] * i7];
    }
  }

  ia = H_xm->size[1];
  for (i7 = 0; i7 < ia; i7++) {
    ib = H_xm->size[0];
    for (ic = 0; ic < ib; ic++) {
      H_v->data[ic + H_v->size[0] * (i7 + H_xc->size[1])] = H_xm->data[ic +
        H_xm->size[0] * i7];
    }
  }

  b_emxInit_real_T(&H_g, 2);
  b_emxInit_real_T(&H_p, 2);
  i7 = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, i7, (int)sizeof(double));

  //  jacobian for gravity residual
  i7 = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, i7, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  for (k = 0; k < 2; k++) {
    d_y[k] = b_imNoise[k] * b_imNoise[k];
  }

  for (i7 = 0; i7 < 4; i7++) {
    d[i7] = 0.0;
  }

  for (b_r = 0; b_r < 2; b_r++) {
    d[b_r + (b_r << 1)] = d_y[b_r];
  }

  eye((double)indMeas->size[0], H_xm);
  kron(H_xm, d, H_xc);
  emxFree_real_T(&H_xm);
  if (gravityUpdate) {
    for (b_r = 0; b_r < 3; b_r++) {
      x[b_r] = (IMU_measurements[b_r + 3] == 0.0);
    }

    e_y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 3)) {
      if (!x[k]) {
        e_y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (!e_y) {
      anew = norm(*(double (*)[3])&IMU_measurements[3]);

      //  normalize the acceleration measurement
      //  the earth-z axis transformed into the body frame
      c_k = 3;
      for (i7 = 0; i7 < 3; i7++) {
        h_rz[i7] = 0.0;
        for (ic = 0; ic < 3; ic++) {
          h_rz[i7] += R_cw[i7 + 3 * ic] * (double)b_b[ic];
        }

        r_g_data[i7] = IMU_measurements[3 + i7] / anew - h_rz[i7];
      }
    } else {
      c_k = 3;
      for (b_r = 0; b_r < 3; b_r++) {
        h_rz[b_r] = 0.0;
        r_g_data[b_r] = 0.0;
      }
    }

    i7 = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + b_numAnchors * (6.0 +
      b_numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, i7, (int)sizeof(double));
    ia = 3 * (int)(errorStateSize + b_numAnchors * (6.0 + b_numPointsPerAnchor));
    for (i7 = 0; i7 < ia; i7++) {
      H_g->data[i7] = 0.0;
    }

    dv4[0] = 0.0;
    dv4[3] = -h_rz[2];
    dv4[6] = h_rz[1];
    dv4[1] = h_rz[2];
    dv4[4] = 0.0;
    dv4[7] = -h_rz[0];
    dv4[2] = -h_rz[1];
    dv4[5] = h_rz[0];
    dv4[8] = 0.0;
    for (i7 = 0; i7 < 3; i7++) {
      for (ic = 0; ic < 3; ic++) {
        H_g->data[ic + H_g->size[0] * (3 + i7)] = dv4[ic + 3 * i7];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (i7 = 0; i7 < 9; i7++) {
      R_g_data[i7] = gravAlignNoise * (double)b[i7];
    }
  }

  if (useAirPressure) {
    i7 = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + b_numAnchors * (6.0 +
      b_numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, i7, (int)sizeof(double));
    ia = (int)(errorStateSize + b_numAnchors * (6.0 + b_numPointsPerAnchor));
    for (i7 = 0; i7 < ia; i7++) {
      H_p->data[i7] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      i7 = r->size[0];
      r->size[0] = z->size[0] + c_k;
      emxEnsureCapacity((emxArray__common *)r, i7, (int)sizeof(double));
      ia = z->size[0];
      for (i7 = 0; i7 < ia; i7++) {
        r->data[i7] = z->data[i7];
      }

      for (i7 = 0; i7 < c_k; i7++) {
        r->data[i7 + z->size[0]] = r_g_data[i7];
      }

      i7 = H->size[0] * H->size[1];
      H->size[0] = (H_v->size[0] + H_g->size[0]) + H_p->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i7, (int)sizeof(double));
      ia = H_v->size[1];
      for (i7 = 0; i7 < ia; i7++) {
        ib = H_v->size[0];
        for (ic = 0; ic < ib; ic++) {
          H->data[ic + H->size[0] * i7] = H_v->data[ic + H_v->size[0] * i7];
        }
      }

      ia = H_g->size[1];
      for (i7 = 0; i7 < ia; i7++) {
        ib = H_g->size[0];
        for (ic = 0; ic < ib; ic++) {
          H->data[(ic + H_v->size[0]) + H->size[0] * i7] = H_g->data[ic +
            H_g->size[0] * i7];
        }
      }

      ia = H_p->size[1];
      for (i7 = 0; i7 < ia; i7++) {
        ib = H_p->size[0];
        for (ic = 0; ic < ib; ic++) {
          H->data[((ic + H_v->size[0]) + H_g->size[0]) + H->size[0] * i7] =
            H_p->data[ic + H_p->size[0] * i7];
        }
      }

      b_r = (H_xc->size[0] + R_g_size[0]) + R_p_size[0];
      c_k = (H_xc->size[1] + R_g_size[1]) + R_p_size[1];
      i7 = R->size[0] * R->size[1];
      R->size[0] = b_r;
      emxEnsureCapacity((emxArray__common *)R, i7, (int)sizeof(double));
      i7 = R->size[0] * R->size[1];
      R->size[1] = c_k;
      emxEnsureCapacity((emxArray__common *)R, i7, (int)sizeof(double));
      ia = b_r * c_k;
      for (i7 = 0; i7 < ia; i7++) {
        R->data[i7] = 0.0;
      }

      if ((H_xc->size[0] > 0) && (H_xc->size[1] > 0)) {
        ia = H_xc->size[1];
        for (i7 = 0; i7 < ia; i7++) {
          ib = H_xc->size[0];
          for (ic = 0; ic < ib; ic++) {
            R->data[ic + R->size[0] * i7] = H_xc->data[ic + H_xc->size[0] * i7];
          }
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        i7 = H_xc->size[0] + R_g_size[0];
        if (H_xc->size[0] + 1 > i7) {
          i7 = 1;
        } else {
          i7 = H_xc->size[0] + 1;
        }

        ic = H_xc->size[1] + R_g_size[1];
        if (H_xc->size[1] + 1 > ic) {
          ic = 1;
        } else {
          ic = H_xc->size[1] + 1;
        }

        ia = R_g_size[1];
        for (br = 0; br < ia; br++) {
          ib = R_g_size[0];
          for (c_k = 0; c_k < ib; c_k++) {
            R->data[((i7 + c_k) + R->size[0] * ((ic + br) - 1)) - 1] =
              R_g_data[c_k + R_g_size[0] * br];
          }
        }
      }

      b_r = H_xc->size[0] + R_g_size[0];
      c = H_xc->size[1] + R_g_size[1];
      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ia = R_p_size[1];
        for (i7 = 0; i7 < ia; i7++) {
          ib = R_p_size[0];
          for (ic = 0; ic < ib; ic++) {
            R->data[(b_r + ic) + R->size[0] * (c + i7)] = R_p_data[ic +
              R_p_size[0] * i7];
          }
        }
      }
    } else {
      i7 = r->size[0];
      r->size[0] = z->size[0] + c_k;
      emxEnsureCapacity((emxArray__common *)r, i7, (int)sizeof(double));
      ia = z->size[0];
      for (i7 = 0; i7 < ia; i7++) {
        r->data[i7] = z->data[i7];
      }

      for (i7 = 0; i7 < c_k; i7++) {
        r->data[i7 + z->size[0]] = r_g_data[i7];
      }

      i7 = H->size[0] * H->size[1];
      H->size[0] = H_v->size[0] + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, i7, (int)sizeof(double));
      ia = H_v->size[1];
      for (i7 = 0; i7 < ia; i7++) {
        ib = H_v->size[0];
        for (ic = 0; ic < ib; ic++) {
          H->data[ic + H->size[0] * i7] = H_v->data[ic + H_v->size[0] * i7];
        }
      }

      ia = H_g->size[1];
      for (i7 = 0; i7 < ia; i7++) {
        ib = H_g->size[0];
        for (ic = 0; ic < ib; ic++) {
          H->data[(ic + H_v->size[0]) + H->size[0] * i7] = H_g->data[ic +
            H_g->size[0] * i7];
        }
      }

      b_R_p_data.data = (double *)&R_g_data;
      b_R_p_data.size = (int *)&R_g_size;
      b_R_p_data.allocatedSize = 9;
      b_R_p_data.numDimensions = 2;
      b_R_p_data.canFreeData = false;
      blkdiag(H_xc, &b_R_p_data, R);
    }
  } else if (useAirPressure) {
    i7 = r->size[0];
    r->size[0] = z->size[0];
    emxEnsureCapacity((emxArray__common *)r, i7, (int)sizeof(double));
    ia = z->size[0];
    for (i7 = 0; i7 < ia; i7++) {
      r->data[i7] = z->data[i7];
    }

    i7 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0] + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i7, (int)sizeof(double));
    ia = H_v->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      ib = H_v->size[0];
      for (ic = 0; ic < ib; ic++) {
        H->data[ic + H->size[0] * i7] = H_v->data[ic + H_v->size[0] * i7];
      }
    }

    ia = H_p->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      ib = H_p->size[0];
      for (ic = 0; ic < ib; ic++) {
        H->data[(ic + H_v->size[0]) + H->size[0] * i7] = H_p->data[ic +
          H_p->size[0] * i7];
      }
    }

    b_R_p_data.data = (double *)&R_p_data;
    b_R_p_data.size = (int *)&R_p_size;
    b_R_p_data.allocatedSize = 1;
    b_R_p_data.numDimensions = 2;
    b_R_p_data.canFreeData = false;
    blkdiag(H_xc, &b_R_p_data, R);
  } else {
    i7 = r->size[0];
    r->size[0] = z->size[0];
    emxEnsureCapacity((emxArray__common *)r, i7, (int)sizeof(double));
    ia = z->size[0];
    for (i7 = 0; i7 < ia; i7++) {
      r->data[i7] = z->data[i7];
    }

    i7 = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, i7, (int)sizeof(double));
    ia = H_v->size[0] * H_v->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      H->data[i7] = H_v->data[i7];
    }

    i7 = R->size[0] * R->size[1];
    R->size[0] = H_xc->size[0];
    R->size[1] = H_xc->size[1];
    emxEnsureCapacity((emxArray__common *)R, i7, (int)sizeof(double));
    ia = H_xc->size[0] * H_xc->size[1];
    for (i7 = 0; i7 < ia; i7++) {
      R->data[i7] = H_xc->data[i7];
    }
  }

  emxFree_real_T(&H_p);
  emxFree_real_T(&H_g);
  emxFree_real_T(&H_v);
  emxFree_real_T(&z);
  emxFree_real_T(&H_xc);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
