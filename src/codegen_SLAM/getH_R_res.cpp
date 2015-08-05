//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 15:44:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "blkdiag.h"
#include "predictMeasurement_stereo.h"
#include "norm.h"
#include "power.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_left.h"
#include "kron.h"
#include "diag.h"
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
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[13]
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
                  emxArray_real_T *map, double numAnchors, double
                  numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                  emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                  *b_m_vect, const double imNoise[2], const double
                  IMU_measurements[13], double height_offset_pressure, double
                  r_data[], int r_size[1], emxArray_real_T *H, double h_u_data[],
                  int h_u_size[1], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ic;
  int ib;
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
  int ia;
  double h_cin_l[3];
  int nm1d2;
  double dv300[2];
  int r;
  double anew;
  signed char b_k;
  signed char iv3[2];
  double dv301[4];
  double h_un_To_h_d_l[4];
  double dv302[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 537.083588825387, 0.0, 0.0,
    539.036743091681 };

  double d_h_un_To_h_d_l[24];
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
  double dv303[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int ar;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double dv304[6];
  double c_y[6];
  int c;
  double b_z_data;
  emxArray_real_T *H_v;
  emxArray_real_T *r3;
  double dv305[2];
  double dv306[4];
  int R_v_size[2];
  double R_v_data[1024];
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv307[9];
  double R_g_data[9];
  double r_g_data[3];
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
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ic = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ic, (int)sizeof(double));
  ic = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ic, (int)sizeof(double));
  ib = (indMeas_size[0] << 1) * (int)(numAnchors * (6.0 + numPointsPerAnchor));
  for (ic = 0; ic < ib; ic++) {
    H_xm->data[ic] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ic = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ic, (int)sizeof(double));
  ic = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ic, (int)sizeof(double));
  ib = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ic = 0; ic < ib; ic++) {
    H_xc->data[ic] = 0.0;
  }

  z_size_idx_0 = indMeas_size[0] << 1;
  ib = indMeas_size[0] << 1;
  for (ic = 0; ic < ib; ic++) {
    z_data[ic] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  ib = indMeas_size[0] << 1;
  for (ic = 0; ic < ib; ic++) {
    h_u_data[ic] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (ic = 0; ic < 3; ic++) {
      b_map[ic] = map->data[ic + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[ic];
    }

    for (ic = 0; ic < 3; ic++) {
      h_rz[ic] = 0.0;
      for (ia = 0; ia < 3; ia++) {
        h_rz[ic] += R_cw[ic + 3 * ia] * b_map[ia];
      }
    }

    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      h_cin_l[nm1d2] = h_rz[nm1d2] / h_rz[2];
    }

    predictMeasurement_left(h_rz, dv300);
    r = k << 1;
    for (ic = 0; ic < 2; ic++) {
      h_u_data[ic + r] = dv300[ic];
    }

    r = k << 1;
    anew = (indMeas_data[k] - 1.0) * 2.0;
    for (ic = 0; ic < 2; ic++) {
      z_data[ic + r] = z_all_l[(int)(anew + (1.0 + (double)ic)) - 1];
    }

    //     %% computation of H(x)
    b_k = (signed char)((signed char)k << 1);
    for (ic = 0; ic < 2; ic++) {
      iv3[ic] = (signed char)(ic + b_k);
    }

    ib = H_xc->size[1];
    ic = r2->size[0];
    r2->size[0] = ib;
    emxEnsureCapacity((emxArray__common *)r2, ic, (int)sizeof(int));
    for (ic = 0; ic < ib; ic++) {
      r2->data[ic] = ic;
    }

    Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], dv301);
    dv302[0] = 1.0 / h_rz[2];
    dv302[2] = 0.0;
    dv302[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv302[1] = 0.0;
    dv302[3] = 1.0 / h_rz[2];
    dv302[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          h_un_To_h_d_l[ic + (ia << 1)] += c_h_un_To_h_d_l[ic + (r << 1)] *
            dv301[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        b_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          b_h_un_To_h_d_l[ic + (ia << 1)] += h_un_To_h_d_l[ic + (r << 1)] *
            dv302[r + (ia << 1)];
        }
      }
    }

    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_R_cw[ia + 3 * ic] = -R_cw[ia + 3 * ic];
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
    for (ic = 0; ic < 6; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_R_cw[ia + 3 * (ic + 6)] = 0.0;
      }
    }

    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 12; ia++) {
        d_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          d_h_un_To_h_d_l[ic + (ia << 1)] += b_h_un_To_h_d_l[ic + (r << 1)] *
            b_R_cw[r + 3 * ia];
        }
      }
    }

    nm1d2 = r2->size[0];
    for (ic = 0; ic < nm1d2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        H_xc->data[iv3[ia] + H_xc->size[0] * r2->data[ic]] = d_h_un_To_h_d_l[ia
          + (ic << 1)];
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
    for (ic = 0; ic < 3; ic++) {
      d2 = 0.0;
      for (ia = 0; ia < 3; ia++) {
        d2 += anchorRot[ia + 3 * ic] * b_m_vect->data[ia + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_y[ic] = d2 / anew;
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1] * b_xt->data[(int)(((stateSize +
      (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0 +
      numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas_data[k]
      - 1]) - 1];
    dv303[0] = 0.0;
    dv303[3] = -b_y[2];
    dv303[6] = b_y[1];
    dv303[1] = b_y[2];
    dv303[4] = 0.0;
    dv303[7] = -b_y[0];
    dv303[2] = -b_y[1];
    dv303[5] = b_y[0];
    dv303[8] = 0.0;
    nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_anchorRot[ia + 3 * ic] = -anchorRot[ic + 3 * ia];
      }
    }

    for (ic = 0; ic < 3; ic++) {
      d2 = 0.0;
      for (ia = 0; ia < 3; ia++) {
        d2 += b_anchorRot[ic + 3 * ia] * b_m_vect->data[ia + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_map[ic] = d2 / anew;
    }

    r = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k] -
              1]);
    ic = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (nm1d2 + r) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, ic, (int)sizeof(double));
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * ic] = b[ia + 3 * ic];
      }
    }

    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * (ic + 3)] = -dv303[ia + 3 * ic];
      }
    }

    for (ic = 0; ic < nm1d2; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * (ic + 6)] = 0.0;
      }
    }

    for (ic = 0; ic < 3; ic++) {
      H_iy->data[ic + H_iy->size[0] * (6 + nm1d2)] = b_map[ic];
    }

    for (ic = 0; ic < r; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * ((ic + nm1d2) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (ic = 0; ic < 2; ic++) {
      iv3[ic] = (signed char)(ic + b_k);
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

    ic = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = ar + 1;
    emxEnsureCapacity((emxArray__common *)y, ic, (int)sizeof(double));
    if (ar + 1 > 0) {
      y->data[0] = anew;
      if (ar + 1 > 1) {
        y->data[ar] = apnd;
        nm1d2 = (ar + (ar < 0)) >> 1;
        for (r = 1; r < nm1d2; r++) {
          y->data[r] = anew + (double)r;
          y->data[ar - r] = apnd - (double)r;
        }

        if (nm1d2 << 1 == ar) {
          y->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          y->data[nm1d2] = anew + (double)nm1d2;
          y->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    anew = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    ic = r2->size[0];
    r2->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r2, ic, (int)sizeof(int));
    ib = y->size[1];
    for (ic = 0; ic < ib; ic++) {
      r2->data[ic] = (int)(anew + y->data[y->size[0] * ic]) - 1;
    }

    Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], dv301);
    dv304[0] = 1.0 / h_rz[2];
    dv304[2] = 0.0;
    dv304[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv304[1] = 0.0;
    dv304[3] = 1.0 / h_rz[2];
    dv304[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          h_un_To_h_d_l[ic + (ia << 1)] += c_h_un_To_h_d_l[ic + (r << 1)] *
            dv301[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        b_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          b_h_un_To_h_d_l[ic + (ia << 1)] += h_un_To_h_d_l[ic + (r << 1)] *
            dv304[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        c_y[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          c_y[ic + (ia << 1)] += b_h_un_To_h_d_l[ic + (r << 1)] * R_cw[r + 3 *
            ia];
        }
      }
    }

    nm1d2 = H_iy->size[1];
    ic = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, ic, (int)sizeof(double));
    ic = C->size[0] * C->size[1];
    C->size[1] = nm1d2;
    emxEnsureCapacity((emxArray__common *)C, ic, (int)sizeof(double));
    ib = nm1d2 << 1;
    for (ic = 0; ic < ib; ic++) {
      C->data[ic] = 0.0;
    }

    c = (H_iy->size[1] - 1) << 1;
    for (nm1d2 = 0; nm1d2 <= c; nm1d2 += 2) {
      for (ic = nm1d2 + 1; ic <= nm1d2 + 2; ic++) {
        C->data[ic - 1] = 0.0;
      }
    }

    r = 0;
    for (nm1d2 = 0; nm1d2 <= c; nm1d2 += 2) {
      ar = 0;
      for (ib = r; ib + 1 <= r + 3; ib++) {
        if (H_iy->data[ib] != 0.0) {
          ia = ar;
          for (ic = nm1d2; ic + 1 <= nm1d2 + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[ib] * c_y[ia - 1];
          }
        }

        ar += 2;
      }

      r += 3;
    }

    ib = C->size[1];
    for (ic = 0; ic < ib; ic++) {
      for (ia = 0; ia < 2; ia++) {
        H_xm->data[iv3[ia] + H_xm->size[0] * r2->data[ic]] = C->data[ia +
          C->size[0] * ic];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  emxFree_real_T(&H_iy);
  for (ic = 0; ic < z_size_idx_0; ic++) {
    b_z_data = z_data[ic] - h_u_data[ic];
    z_data[ic] = b_z_data;
  }

  emxInit_real_T(&H_v, 2);
  emxInit_real_T(&r3, 2);

  //  residual with respect to camera measurements
  b_eye((double)indMeas_size[0], r3);
  power(imNoise, dv305);
  diag(dv305, dv306);
  kron(r3->data, r3->size, dv306, R_v_data, R_v_size);
  nm1d2 = 0;

  //  gravity residual
  r = 0;

  //  pressure residual
  ic = H_v->size[0] * H_v->size[1];
  H_v->size[0] = H_xc->size[0];
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ic, (int)sizeof(double));
  ib = H_xc->size[1];
  for (ic = 0; ic < ib; ic++) {
    ar = H_xc->size[0];
    for (ia = 0; ia < ar; ia++) {
      H_v->data[ia + H_v->size[0] * ic] = H_xc->data[ia + H_xc->size[0] * ic];
    }
  }

  ib = H_xm->size[1];
  for (ic = 0; ic < ib; ic++) {
    ar = H_xm->size[0];
    for (ia = 0; ia < ar; ia++) {
      H_v->data[ia + H_v->size[0] * (ic + H_xc->size[1])] = H_xm->data[ia +
        H_xm->size[0] * ic];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  ic = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, ic, (int)sizeof(double));

  //  jacobian for gravity residual
  ic = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, ic, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  if (gravityUpdate) {
    anew = norm(*(double (*)[3])&IMU_measurements[3]);

    //  normalize the acceleration measurement
    //  normalize the magnetometer measurement
    for (ic = 0; ic < 3; ic++) {
      h_rz[ic] = 0.0;
      for (ia = 0; ia < 3; ia++) {
        h_rz[ic] += R_cw[ic + 3 * ia] * (double)b_b[ia];
      }
    }

    //  the earth-z axis transformed into the body frame
    ic = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, ic, (int)sizeof(double));
    ib = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ic = 0; ic < ib; ic++) {
      H_g->data[ic] = 0.0;
    }

    dv307[0] = 0.0;
    dv307[3] = -h_rz[2];
    dv307[6] = h_rz[1];
    dv307[1] = h_rz[2];
    dv307[4] = 0.0;
    dv307[7] = -h_rz[0];
    dv307[2] = -h_rz[1];
    dv307[5] = h_rz[0];
    dv307[8] = 0.0;
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_g->data[ia + H_g->size[0] * (3 + ic)] = dv307[ia + 3 * ic];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (ic = 0; ic < 9; ic++) {
      R_g_data[ic] = gravAlignNoise * (double)b[ic];
    }

    nm1d2 = 3;
    for (ic = 0; ic < 3; ic++) {
      r_g_data[ic] = IMU_measurements[3 + ic] / anew - h_rz[ic];
    }
  }

  if (useAirPressure) {
    ic = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, ic, (int)sizeof(double));
    ib = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ic = 0; ic < ib; ic++) {
      H_p->data[ic] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    r = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (z_size_idx_0 + nm1d2) + r;
      for (ic = 0; ic < z_size_idx_0; ic++) {
        r_data[ic] = z_data[ic];
      }

      for (ic = 0; ic < nm1d2; ic++) {
        r_data[ic + z_size_idx_0] = r_g_data[ic];
      }

      ic = 0;
      while (ic <= r - 1) {
        r_data[z_size_idx_0 + nm1d2] = r_p_data[0];
        ic = 1;
      }

      ic = H->size[0] * H->size[1];
      H->size[0] = (H_v->size[0] + H_g->size[0]) + H_p->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
      ib = H_v->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_v->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
        }
      }

      ib = H_g->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_g->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_g->data[ia +
            H_g->size[0] * ic];
        }
      }

      ib = H_p->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_p->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[((ia + H_v->size[0]) + H_g->size[0]) + H->size[0] * ic] =
            H_p->data[ia + H_p->size[0] * ic];
        }
      }

      output_size(R_v_size, R_g_size, R_p_size, &nm1d2, &r);
      R_size[0] = nm1d2;
      R_size[1] = r;
      ib = nm1d2 * r;
      for (ic = 0; ic < ib; ic++) {
        R_data[ic] = 0.0;
      }

      if ((R_v_size[0] > 0) && (R_v_size[1] > 0)) {
        ib = R_v_size[1];
        for (ic = 0; ic < ib; ic++) {
          ar = R_v_size[0];
          for (ia = 0; ia < ar; ia++) {
            R_data[ia + R_size[0] * ic] = R_v_data[ia + R_v_size[0] * ic];
          }
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        ic = R_v_size[0] + R_g_size[0];
        if (R_v_size[0] + 1 > ic) {
          ic = 1;
        } else {
          ic = R_v_size[0] + 1;
        }

        ia = R_v_size[1] + R_g_size[1];
        if (R_v_size[1] + 1 > ia) {
          ia = 1;
        } else {
          ia = R_v_size[1] + 1;
        }

        ib = R_g_size[1];
        for (r = 0; r < ib; r++) {
          ar = R_g_size[0];
          for (nm1d2 = 0; nm1d2 < ar; nm1d2++) {
            R_data[((ic + nm1d2) + R_size[0] * ((ia + r) - 1)) - 1] =
              R_g_data[nm1d2 + R_g_size[0] * r];
          }
        }
      }

      r = R_v_size[0] + R_g_size[0];
      c = R_v_size[1] + R_g_size[1];
      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ib = R_p_size[1];
        for (ic = 0; ic < ib; ic++) {
          ar = R_p_size[0];
          for (ia = 0; ia < ar; ia++) {
            R_data[(r + ia) + R_size[0] * (c + ic)] = R_p_data[ia + R_p_size[0] *
              ic];
          }
        }
      }
    } else {
      r_size[0] = z_size_idx_0 + nm1d2;
      for (ic = 0; ic < z_size_idx_0; ic++) {
        r_data[ic] = z_data[ic];
      }

      for (ic = 0; ic < nm1d2; ic++) {
        r_data[ic + z_size_idx_0] = r_g_data[ic];
      }

      ic = H->size[0] * H->size[1];
      H->size[0] = H_v->size[0] + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
      ib = H_v->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_v->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
        }
      }

      ib = H_g->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_g->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_g->data[ia +
            H_g->size[0] * ic];
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
      ib = r3->size[0] * r3->size[1];
      for (ic = 0; ic < ib; ic++) {
        R_data[ic] = r3->data[ic];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = z_size_idx_0 + r;
    for (ic = 0; ic < z_size_idx_0; ic++) {
      r_data[ic] = z_data[ic];
    }

    ic = 0;
    while (ic <= r - 1) {
      r_data[z_size_idx_0] = r_p_data[0];
      ic = 1;
    }

    ic = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0] + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
    ib = H_v->size[1];
    for (ic = 0; ic < ib; ic++) {
      ar = H_v->size[0];
      for (ia = 0; ia < ar; ia++) {
        H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
      }
    }

    ib = H_p->size[1];
    for (ic = 0; ic < ib; ic++) {
      ar = H_p->size[0];
      for (ia = 0; ia < ar; ia++) {
        H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_p->data[ia +
          H_p->size[0] * ic];
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
    ib = r3->size[0] * r3->size[1];
    for (ic = 0; ic < ib; ic++) {
      R_data[ic] = r3->data[ic];
    }
  } else {
    r_size[0] = z_size_idx_0;
    for (ic = 0; ic < z_size_idx_0; ic++) {
      r_data[ic] = z_data[ic];
    }

    ic = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
    ib = H_v->size[0] * H_v->size[1];
    for (ic = 0; ic < ib; ic++) {
      H->data[ic] = H_v->data[ic];
    }

    R_size[0] = R_v_size[0];
    R_size[1] = R_v_size[1];
    ib = R_v_size[0] * R_v_size[1];
    for (ic = 0; ic < ib; ic++) {
      R_data[ic] = R_v_data[ic];
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
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[13]
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
                emxArray_real_T *map, double numAnchors, double
                numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                *b_m_vect, const double imNoise[2], const double
                IMU_measurements[13], double height_offset_pressure, double
                r_data[], int r_size[1], emxArray_real_T *H, double h_u[2],
                double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ib;
  int br;
  emxArray_real_T *H_xc;
  double b_map[3];
  double h_ci_l[3];
  int ar;
  double h_cin_l[3];
  int nm1d2;
  double anew;
  double z[2];
  emxArray_int32_T *r0;
  double d[4];
  double h_un_To_h_d_l[4];
  double dv296[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 537.083588825387, 0.0, 0.0,
    539.036743091681 };

  double d_h_un_To_h_d_l[24];
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
  double dv297[9];
  double b_anchorRot[9];
  emxArray_real_T *H_iy;
  int k;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *b_y;
  double dv298[6];
  double c_y[6];
  emxArray_real_T *H_v;
  double v[2];
  int ic;
  int ia;
  double b_z;
  double R_v[4];
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv299[9];
  double R_g_data[9];
  double r_g_data[3];
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
  ib = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = 2;
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  br = (int)(numAnchors * (6.0 + numPointsPerAnchor)) << 1;
  for (ib = 0; ib < br; ib++) {
    H_xm->data[ib] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  br = (int)errorStateSize << 1;
  for (ib = 0; ib < br; ib++) {
    H_xc->data[ib] = 0.0;
  }

  for (ib = 0; ib < 3; ib++) {
    b_map[ib] = map->data[ib + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ib];
  }

  for (ib = 0; ib < 3; ib++) {
    h_ci_l[ib] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      h_ci_l[ib] += R_cw[ib + 3 * ar] * b_map[ar];
    }
  }

  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  predictMeasurement_left(h_ci_l, h_u);
  anew = (indMeas - 1.0) * 2.0;
  for (ib = 0; ib < 2; ib++) {
    z[ib] = z_all_l[(int)(anew + (1.0 + (double)ib)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  br = (int)errorStateSize;
  ib = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  for (ib = 0; ib < br; ib++) {
    r0->data[ib] = ib;
  }

  Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], d);
  dv296[0] = 1.0 / h_ci_l[2];
  dv296[2] = 0.0;
  dv296[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv296[1] = 0.0;
  dv296[3] = 1.0 / h_ci_l[2];
  dv296[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ib + (ar << 1)] += c_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          d[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      b_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ib + (ar << 1)] += h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          dv296[nm1d2 + (ar << 1)];
      }
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      b_R_cw[ar + 3 * ib] = -R_cw[ar + 3 * ib];
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
    for (ar = 0; ar < 3; ar++) {
      b_R_cw[ar + 3 * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 12; ar++) {
      d_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        d_h_un_To_h_d_l[ib + (ar << 1)] += b_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          b_R_cw[nm1d2 + 3 * ar];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xc->data[ar + H_xc->size[0] * r0->data[ib]] = d_h_un_To_h_d_l[ar + (ib <<
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
  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (ar = 0; ar < 3; ar++) {
      d1 += anchorRot[ar + 3 * ib] * b_m_vect->data[ar + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    y[ib] = d1 / anew;
  }

  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1] * b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1]
    - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
    indMeas - 1]) - 1];
  dv297[0] = 0.0;
  dv297[3] = -y[2];
  dv297[6] = y[1];
  dv297[1] = y[2];
  dv297[4] = 0.0;
  dv297[7] = -y[0];
  dv297[2] = -y[1];
  dv297[5] = y[0];
  dv297[8] = 0.0;
  nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas - 1] - 1.0);
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      b_anchorRot[ar + 3 * ib] = -anchorRot[ib + 3 * ar];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (ar = 0; ar < 3; ar++) {
      d1 += b_anchorRot[ib + 3 * ar] * b_m_vect->data[ar + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    b_map[ib] = d1 / anew;
  }

  emxInit_real_T(&H_iy, 2);
  k = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas - 1]);
  ib = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + k) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ib] = b[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 3)] = -dv297[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    H_iy->data[ib + H_iy->size[0] * (6 + nm1d2)] = b_map[ib];
  }

  for (ib = 0; ib < k; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ((ib + nm1d2) + 7)] = 0.0;
    }
  }

  if (rtIsNaN(6.0 + numPointsPerAnchor)) {
    br = 0;
    anew = rtNaN;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (6.0 + numPointsPerAnchor < 1.0) {
    br = -1;
    anew = 1.0;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
    br = 0;
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
      br = (int)ndbl - 1;
    } else {
      br = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  ib = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = br + 1;
  emxEnsureCapacity((emxArray__common *)b_y, ib, (int)sizeof(double));
  if (br + 1 > 0) {
    b_y->data[0] = anew;
    if (br + 1 > 1) {
      b_y->data[br] = apnd;
      nm1d2 = (br + (br < 0)) >> 1;
      for (k = 1; k < nm1d2; k++) {
        b_y->data[k] = anew + (double)k;
        b_y->data[br - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == br) {
        b_y->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = anew + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  anew = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 + numPointsPerAnchor);
  ib = r0->size[0];
  r0->size[0] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  br = b_y->size[1];
  for (ib = 0; ib < br; ib++) {
    r0->data[ib] = (int)(anew + b_y->data[b_y->size[0] * ib]) - 1;
  }

  emxFree_real_T(&b_y);
  Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], d);
  dv298[0] = 1.0 / h_ci_l[2];
  dv298[2] = 0.0;
  dv298[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv298[1] = 0.0;
  dv298[3] = 1.0 / h_ci_l[2];
  dv298[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ib + (ar << 1)] += c_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          d[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      b_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ib + (ar << 1)] += h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          dv298[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      c_y[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        c_y[ib + (ar << 1)] += b_h_un_To_h_d_l[ib + (nm1d2 << 1)] * R_cw[nm1d2 +
          3 * ar];
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
  br = (int)v[1] << 1;
  for (ib = 0; ib < br; ib++) {
    H_v->data[ib] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (k = 0; k <= nm1d2; k += 2) {
    for (ic = k + 1; ic <= k + 2; ic++) {
      H_v->data[ic - 1] = 0.0;
    }
  }

  br = 0;
  for (k = 0; k <= nm1d2; k += 2) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (H_iy->data[ib] != 0.0) {
        ia = ar;
        for (ic = k; ic + 1 <= k + 2; ic++) {
          ia++;
          H_v->data[ic] += H_iy->data[ib] * c_y[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  emxFree_real_T(&H_iy);
  br = H_v->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xm->data[ar + H_xm->size[0] * r0->data[ib]] = H_v->data[ar + H_v->size[0]
        * ib];
    }
  }

  emxFree_int32_T(&r0);
  for (ib = 0; ib < 2; ib++) {
    b_z = z[ib] - h_u[ib];
    z[ib] = b_z;
  }

  //  residual with respect to camera measurements
  power(imNoise, v);
  for (ib = 0; ib < 4; ib++) {
    d[ib] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  nm1d2 = -1;
  for (k = 0; k < 2; k++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_v[nm1d2] = d[br + (k << 1)];
    }
  }

  nm1d2 = 0;

  //  gravity residual
  k = 0;

  //  pressure residual
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  br = H_xc->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_v->data[ar + H_v->size[0] * ib] = H_xc->data[ar + H_xc->size[0] * ib];
    }
  }

  br = H_xm->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_v->data[ar + H_v->size[0] * (ib + H_xc->size[1])] = H_xm->data[ar +
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
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  if (gravityUpdate) {
    anew = norm(*(double (*)[3])&IMU_measurements[3]);

    //  normalize the acceleration measurement
    //  normalize the magnetometer measurement
    for (ib = 0; ib < 3; ib++) {
      h_ci_l[ib] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        h_ci_l[ib] += R_cw[ib + 3 * ar] * (double)b_b[ar];
      }
    }

    //  the earth-z axis transformed into the body frame
    ib = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, ib, (int)sizeof(double));
    br = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ib = 0; ib < br; ib++) {
      H_g->data[ib] = 0.0;
    }

    dv299[0] = 0.0;
    dv299[3] = -h_ci_l[2];
    dv299[6] = h_ci_l[1];
    dv299[1] = h_ci_l[2];
    dv299[4] = 0.0;
    dv299[7] = -h_ci_l[0];
    dv299[2] = -h_ci_l[1];
    dv299[5] = h_ci_l[0];
    dv299[8] = 0.0;
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_g->data[ar + H_g->size[0] * (3 + ib)] = dv299[ar + 3 * ib];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (ib = 0; ib < 9; ib++) {
      R_g_data[ib] = gravAlignNoise * (double)b[ib];
    }

    nm1d2 = 3;
    for (ib = 0; ib < 3; ib++) {
      r_g_data[ib] = IMU_measurements[3 + ib] / anew - h_ci_l[ib];
    }
  }

  if (useAirPressure) {
    ib = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, ib, (int)sizeof(double));
    br = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ib = 0; ib < br; ib++) {
      H_p->data[ib] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    k = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (nm1d2 + k) + 2;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < nm1d2; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = 0;
      while (ib <= k - 1) {
        r_data[nm1d2 + 2] = r_p_data[0];
        ib = 1;
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = (H_g->size[0] + H_p->size[0]) + 2;
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      br = H_v->size[1];
      for (ib = 0; ib < br; ib++) {
        for (ar = 0; ar < 2; ar++) {
          H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
        }
      }

      br = H_g->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_g->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[(ar + H->size[0] * ib) + 2] = H_g->data[ar + H_g->size[0] * ib];
        }
      }

      br = H_p->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_p->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[((ar + H_g->size[0]) + H->size[0] * ib) + 2] = H_p->data[ar +
            H_p->size[0] * ib];
        }
      }

      unnamed_idx_0 = (signed char)((R_g_size[0] + R_p_size[0]) + 2);
      unnamed_idx_1 = (signed char)((R_g_size[1] + R_p_size[1]) + 2);
      R_size[0] = unnamed_idx_0;
      R_size[1] = unnamed_idx_1;
      br = unnamed_idx_0 * unnamed_idx_1;
      for (ib = 0; ib < br; ib++) {
        R_data[ib] = 0.0;
      }

      for (ib = 0; ib < 2; ib++) {
        for (ar = 0; ar < 2; ar++) {
          R_data[ar + R_size[0] * ib] = R_v[ar + (ib << 1)];
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        br = R_g_size[1];
        for (ib = 0; ib < br; ib++) {
          nm1d2 = R_g_size[0];
          for (ar = 0; ar < nm1d2; ar++) {
            R_data[(ar + R_size[0] * (2 + ib)) + 2] = R_g_data[ar + R_g_size[0] *
              ib];
          }
        }
      }

      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ib = 0;
        while (ib <= 0) {
          ib = 0;
          while (ib <= 0) {
            R_data[(R_g_size[0] + R_size[0] * (R_g_size[1] + 2)) + 2] = 2.0;
            ib = 1;
          }

          ib = 1;
        }
      }
    } else {
      r_size[0] = 2 + nm1d2;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < nm1d2; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = 2 + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      br = H_v->size[1];
      for (ib = 0; ib < br; ib++) {
        for (ar = 0; ar < 2; ar++) {
          H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
        }
      }

      br = H_g->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_g->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[(ar + H->size[0] * ib) + 2] = H_g->data[ar + H_g->size[0] * ib];
        }
      }

      b_blkdiag(R_v, R_g_data, R_g_size, tmp_data, tmp_size);
      R_size[0] = tmp_size[0];
      R_size[1] = tmp_size[1];
      br = tmp_size[0] * tmp_size[1];
      for (ib = 0; ib < br; ib++) {
        R_data[ib] = tmp_data[ib];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = 2 + k;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib] = z[ib];
    }

    ib = 0;
    while (ib <= k - 1) {
      r_data[2] = r_p_data[0];
      ib = 1;
    }

    ib = H->size[0] * H->size[1];
    H->size[0] = 2 + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
    br = H_v->size[1];
    for (ib = 0; ib < br; ib++) {
      for (ar = 0; ar < 2; ar++) {
        H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
      }
    }

    br = H_p->size[1];
    for (ib = 0; ib < br; ib++) {
      nm1d2 = H_p->size[0];
      for (ar = 0; ar < nm1d2; ar++) {
        H->data[(ar + H->size[0] * ib) + 2] = H_p->data[ar + H_p->size[0] * ib];
      }
    }

    b_blkdiag(R_v, R_p_data, R_p_size, tmp_data, tmp_size);
    R_size[0] = tmp_size[0];
    R_size[1] = tmp_size[1];
    br = tmp_size[0] * tmp_size[1];
    for (ib = 0; ib < br; ib++) {
      R_data[ib] = tmp_data[ib];
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
    br = H_v->size[0] * H_v->size[1];
    for (ib = 0; ib < br; ib++) {
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
