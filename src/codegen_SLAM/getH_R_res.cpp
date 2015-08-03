//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Aug-2015 13:58:52
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "predictMeasurement_stereo.h"
#include "predictMeasurement_left.h"
#include "eye.h"
#include "SLAM_rtwutil.h"
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
//                double c_numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
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
                  emxArray_real_T *map, double c_numAnchors, double
                  numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                  emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                  *b_m_vect, const double imNoise[2], double r_data[], int
                  r_size[1], emxArray_real_T *H, double h_u_data[], int
                  h_u_size[1], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int i2;
  int br;
  emxArray_real_T *H_xc;
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r1;
  emxArray_real_T *C;
  emxArray_real_T *y;
  double b_map[3];
  double h_ci_l[3];
  int b_j1;
  double h_uin_l[2];
  int kidx;
  double dv299[2];
  double anew;
  double x2;
  double y2;
  double a;
  double h_dn_l_To_h_un_l[4];
  signed char b_k;
  signed char iv4[2];
  double h_un_To_h_d_l[4];
  double dv300[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 537.083588825387, 0.0, 0.0,
    539.036743091681 };

  double d_h_un_To_h_d_l[24];
  double b_stateSize;
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
  double b_y[3];
  double dv301[9];
  double b_anchorRot[9];
  static const signed char iv5[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double dv302[6];
  double c_y[6];
  double d_y[2];
  int ic;
  int ar;
  int ia;
  emxArray_real_T *r2;
  double A_data[256];
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
  i2 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, i2, (int)sizeof(double));
  i2 = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(c_numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, i2, (int)sizeof(double));
  br = (indMeas_size[0] << 1) * (int)(c_numAnchors * (6.0 + numPointsPerAnchor));
  for (i2 = 0; i2 < br; i2++) {
    H_xm->data[i2] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  i2 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, i2, (int)sizeof(double));
  i2 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, i2, (int)sizeof(double));
  br = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (i2 = 0; i2 < br; i2++) {
    H_xc->data[i2] = 0.0;
  }

  r_size[0] = indMeas_size[0] << 1;
  br = indMeas_size[0] << 1;
  for (i2 = 0; i2 < br; i2++) {
    r_data[i2] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  br = indMeas_size[0] << 1;
  for (i2 = 0; i2 < br; i2++) {
    h_u_data[i2] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (i2 = 0; i2 < 3; i2++) {
      b_map[i2] = map->data[i2 + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[i2];
    }

    for (i2 = 0; i2 < 3; i2++) {
      h_ci_l[i2] = 0.0;
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        h_ci_l[i2] += R_cw[i2 + 3 * b_j1] * b_map[b_j1];
      }
    }

    for (kidx = 0; kidx < 2; kidx++) {
      h_uin_l[kidx] = h_ci_l[kidx] / h_ci_l[2];
    }

    predictMeasurement_left(h_ci_l, dv299);
    br = k << 1;
    for (i2 = 0; i2 < 2; i2++) {
      h_u_data[i2 + br] = dv299[i2];
    }

    br = k << 1;
    anew = (indMeas_data[k] - 1.0) * 2.0;
    for (i2 = 0; i2 < 2; i2++) {
      r_data[i2 + br] = z_all_l[(int)(anew + (1.0 + (double)i2)) - 1];
    }

    //     %% computation of H(x)
    // UNTITLED Summary of this function goes here
    //   computs the derivatives with respect to the normalised undistorted point 
    x2 = h_uin_l[0] * h_uin_l[0];
    y2 = h_uin_l[1] * h_uin_l[1];
    a = x2 + y2;
    h_dn_l_To_h_un_l[0] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
      0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
      4.0)) + h_uin_l[0] * ((-0.82084366875022 * h_uin_l[0] + 0.885061321211732 *
      h_uin_l[0] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[0] * rt_powd_snf
      (x2 + y2, 3.0));
    h_dn_l_To_h_un_l[2] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
      0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 +
      y2, 3.0));
    h_dn_l_To_h_un_l[1] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
      0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 +
      y2, 3.0));
    a = x2 + y2;
    h_dn_l_To_h_un_l[3] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
      0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
      4.0)) + h_uin_l[1] * ((-0.82084366875022 * h_uin_l[1] + 0.885061321211732 *
      h_uin_l[1] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[1] * rt_powd_snf
      (x2 + y2, 3.0));

    //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
    //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
    //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
    //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
    b_k = (signed char)((signed char)k << 1);
    for (i2 = 0; i2 < 2; i2++) {
      iv4[i2] = (signed char)(i2 + b_k);
    }

    br = H_xc->size[1];
    i2 = r1->size[0];
    r1->size[0] = br;
    emxEnsureCapacity((emxArray__common *)r1, i2, (int)sizeof(int));
    for (i2 = 0; i2 < br; i2++) {
      r1->data[i2] = i2;
    }

    dv300[0] = 1.0 / h_ci_l[2];
    dv300[2] = 0.0;
    dv300[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    dv300[1] = 0.0;
    dv300[3] = 1.0 / h_ci_l[2];
    dv300[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    for (i2 = 0; i2 < 2; i2++) {
      for (b_j1 = 0; b_j1 < 2; b_j1++) {
        h_un_To_h_d_l[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          h_un_To_h_d_l[i2 + (b_j1 << 1)] += c_h_un_To_h_d_l[i2 + (kidx << 1)] *
            h_dn_l_To_h_un_l[kidx + (b_j1 << 1)];
        }
      }

      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        b_h_un_To_h_d_l[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          b_h_un_To_h_d_l[i2 + (b_j1 << 1)] += h_un_To_h_d_l[i2 + (kidx << 1)] *
            dv300[kidx + (b_j1 << 1)];
        }
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        b_R_cw[b_j1 + 3 * i2] = -R_cw[b_j1 + 3 * i2];
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
    for (i2 = 0; i2 < 6; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        b_R_cw[b_j1 + 3 * (i2 + 6)] = 0.0;
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      for (b_j1 = 0; b_j1 < 12; b_j1++) {
        d_h_un_To_h_d_l[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          d_h_un_To_h_d_l[i2 + (b_j1 << 1)] += b_h_un_To_h_d_l[i2 + (kidx << 1)]
            * b_R_cw[kidx + 3 * b_j1];
        }
      }
    }

    kidx = r1->size[0];
    for (i2 = 0; i2 < kidx; i2++) {
      for (b_j1 = 0; b_j1 < 2; b_j1++) {
        H_xc->data[iv4[b_j1] + H_xc->size[0] * r1->data[i2]] =
          d_h_un_To_h_d_l[b_j1 + (i2 << 1)];
      }
    }

    //     %% anchor state derivatives
    for (kidx = 0; kidx < 2; kidx++) {
      h_uin_l[kidx] = h_ci_l[kidx] / h_ci_l[2];
    }

    //     %% computation of H(x)
    // UNTITLED Summary of this function goes here
    //   computs the derivatives with respect to the normalised undistorted point 
    x2 = h_uin_l[0] * h_uin_l[0];
    y2 = h_uin_l[1] * h_uin_l[1];
    a = x2 + y2;
    h_dn_l_To_h_un_l[0] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
      0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
      4.0)) + h_uin_l[0] * ((-0.82084366875022 * h_uin_l[0] + 0.885061321211732 *
      h_uin_l[0] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[0] * rt_powd_snf
      (x2 + y2, 3.0));
    h_dn_l_To_h_un_l[2] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
      0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 +
      y2, 3.0));
    h_dn_l_To_h_un_l[1] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
      0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 +
      y2, 3.0));
    a = x2 + y2;
    h_dn_l_To_h_un_l[3] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
      0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
      4.0)) + h_uin_l[1] * ((-0.82084366875022 * h_uin_l[1] + 0.885061321211732 *
      h_uin_l[1] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[1] * rt_powd_snf
      (x2 + y2, 3.0));

    //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
    //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
    //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
    //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
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
    for (i2 = 0; i2 < 3; i2++) {
      a = 0.0;
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        a += anchorRot[b_j1 + 3 * i2] * b_m_vect->data[b_j1 + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_y[i2] = a / anew;
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1] * b_xt->data[(int)(((stateSize +
      (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0 +
      numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas_data[k]
      - 1]) - 1];
    dv301[0] = 0.0;
    dv301[3] = -b_y[2];
    dv301[6] = b_y[1];
    dv301[1] = b_y[2];
    dv301[4] = 0.0;
    dv301[7] = -b_y[0];
    dv301[2] = -b_y[1];
    dv301[5] = b_y[0];
    dv301[8] = 0.0;
    kidx = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (i2 = 0; i2 < 3; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        b_anchorRot[b_j1 + 3 * i2] = -anchorRot[i2 + 3 * b_j1];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      a = 0.0;
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        a += b_anchorRot[i2 + 3 * b_j1] * b_m_vect->data[b_j1 + b_m_vect->size[0]
          * ((int)indMeas_data[k] - 1)];
      }

      b_map[i2] = a / anew;
    }

    br = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k]
               - 1]);
    i2 = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (kidx + br) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, i2, (int)sizeof(double));
    for (i2 = 0; i2 < 3; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        H_iy->data[b_j1 + H_iy->size[0] * i2] = iv5[b_j1 + 3 * i2];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        H_iy->data[b_j1 + H_iy->size[0] * (i2 + 3)] = -dv301[b_j1 + 3 * i2];
      }
    }

    for (i2 = 0; i2 < kidx; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        H_iy->data[b_j1 + H_iy->size[0] * (i2 + 6)] = 0.0;
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      H_iy->data[i2 + H_iy->size[0] * (6 + kidx)] = b_map[i2];
    }

    for (i2 = 0; i2 < br; i2++) {
      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        H_iy->data[b_j1 + H_iy->size[0] * ((i2 + kidx) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (i2 = 0; i2 < 2; i2++) {
      iv4[i2] = (signed char)(i2 + b_k);
    }

    if (rtIsNaN(6.0 + numPointsPerAnchor)) {
      b_j1 = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (6.0 + numPointsPerAnchor < 1.0) {
      b_j1 = -1;
      anew = 1.0;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
      b_j1 = 0;
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
        b_j1 = (int)ndbl - 1;
      } else {
        b_j1 = -1;
      }
    }

    i2 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = b_j1 + 1;
    emxEnsureCapacity((emxArray__common *)y, i2, (int)sizeof(double));
    if (b_j1 + 1 > 0) {
      y->data[0] = anew;
      if (b_j1 + 1 > 1) {
        y->data[b_j1] = apnd;
        kidx = (b_j1 + (b_j1 < 0)) >> 1;
        for (br = 1; br < kidx; br++) {
          y->data[br] = anew + (double)br;
          y->data[b_j1 - br] = apnd - (double)br;
        }

        if (kidx << 1 == b_j1) {
          y->data[kidx] = (anew + apnd) / 2.0;
        } else {
          y->data[kidx] = anew + (double)kidx;
          y->data[kidx + 1] = apnd - (double)kidx;
        }
      }
    }

    anew = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    i2 = r1->size[0];
    r1->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r1, i2, (int)sizeof(int));
    br = y->size[1];
    for (i2 = 0; i2 < br; i2++) {
      r1->data[i2] = (int)(anew + y->data[y->size[0] * i2]) - 1;
    }

    dv302[0] = 1.0 / h_ci_l[2];
    dv302[2] = 0.0;
    dv302[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    dv302[1] = 0.0;
    dv302[3] = 1.0 / h_ci_l[2];
    dv302[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    for (i2 = 0; i2 < 2; i2++) {
      for (b_j1 = 0; b_j1 < 2; b_j1++) {
        h_un_To_h_d_l[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          h_un_To_h_d_l[i2 + (b_j1 << 1)] += c_h_un_To_h_d_l[i2 + (kidx << 1)] *
            h_dn_l_To_h_un_l[kidx + (b_j1 << 1)];
        }
      }

      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        b_h_un_To_h_d_l[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 2; kidx++) {
          b_h_un_To_h_d_l[i2 + (b_j1 << 1)] += h_un_To_h_d_l[i2 + (kidx << 1)] *
            dv302[kidx + (b_j1 << 1)];
        }
      }

      for (b_j1 = 0; b_j1 < 3; b_j1++) {
        c_y[i2 + (b_j1 << 1)] = 0.0;
        for (kidx = 0; kidx < 3; kidx++) {
          c_y[i2 + (b_j1 << 1)] += b_h_un_To_h_d_l[i2 + (kidx << 1)] * R_cw[kidx
            + 3 * b_j1];
        }
      }
    }

    d_y[1] = H_iy->size[1];
    i2 = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, i2, (int)sizeof(double));
    i2 = C->size[0] * C->size[1];
    C->size[1] = (int)d_y[1];
    emxEnsureCapacity((emxArray__common *)C, i2, (int)sizeof(double));
    br = (int)d_y[1] << 1;
    for (i2 = 0; i2 < br; i2++) {
      C->data[i2] = 0.0;
    }

    kidx = (H_iy->size[1] - 1) << 1;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      ar = 0;
      for (i2 = br; i2 + 1 <= br + 3; i2++) {
        if (H_iy->data[i2] != 0.0) {
          ia = ar;
          for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[i2] * c_y[ia - 1];
          }
        }

        ar += 2;
      }

      br += 3;
    }

    br = C->size[1];
    for (i2 = 0; i2 < br; i2++) {
      for (b_j1 = 0; b_j1 < 2; b_j1++) {
        H_xm->data[iv4[b_j1] + H_xm->size[0] * r1->data[i2]] = C->data[b_j1 +
          C->size[0] * i2];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r1);
  emxFree_real_T(&H_iy);
  br = r_size[0];
  for (i2 = 0; i2 < br; i2++) {
    r_data[i2] -= h_u_data[i2];
  }

  //  residual with respect to camera measurements
  for (k = 0; k < 2; k++) {
    d_y[k] = imNoise[k] * imNoise[k];
  }

  for (i2 = 0; i2 < 4; i2++) {
    h_dn_l_To_h_un_l[i2] = 0.0;
  }

  for (kidx = 0; kidx < 2; kidx++) {
    h_dn_l_To_h_un_l[kidx + (kidx << 1)] = d_y[kidx];
  }

  emxInit_real_T(&r2, 2);
  b_eye((double)r_size[0] / 2.0, r2);
  ia = r2->size[0];
  ic = r2->size[1];
  br = r2->size[0] * r2->size[1];
  for (i2 = 0; i2 < br; i2++) {
    A_data[i2] = r2->data[i2];
  }

  emxFree_real_T(&r2);
  R_size[0] = (signed char)(ia << 1);
  R_size[1] = (signed char)(ic << 1);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= ic; b_j1++) {
    for (br = 0; br < 2; br++) {
      for (ar = 1; ar <= ia; ar++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          R_data[kidx] = A_data[(ar + ia * (b_j1 - 1)) - 1] *
            h_dn_l_To_h_un_l[i2 + (br << 1)];
        }
      }
    }
  }

  i2 = H->size[0] * H->size[1];
  H->size[0] = H_xc->size[0];
  H->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H, i2, (int)sizeof(double));
  br = H_xc->size[1];
  for (i2 = 0; i2 < br; i2++) {
    kidx = H_xc->size[0];
    for (b_j1 = 0; b_j1 < kidx; b_j1++) {
      H->data[b_j1 + H->size[0] * i2] = H_xc->data[b_j1 + H_xc->size[0] * i2];
    }
  }

  br = H_xm->size[1];
  for (i2 = 0; i2 < br; i2++) {
    kidx = H_xm->size[0];
    for (b_j1 = 0; b_j1 < kidx; b_j1++) {
      H->data[b_j1 + H->size[0] * (i2 + H_xc->size[1])] = H_xm->data[b_j1 +
        H_xm->size[0] * i2];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);

  //      if gravityUpdate
  //          R_cw = RotFromQuatJ(x(4:7));
  //          h_rz = R_cw*[0;0;1];
  //          H_g = zeros(3,numStates+numAnchors*numStatesPerAnchor);
  //          H_g(:,4:6) = skew(h_rz);
  //          H = [H; H_g];
  //          R = blkdiag(R,r_g);
  //          r_g = [r; za/norm(za) - h_rz];
  //      end
  //
  //      if MagUpdate
  //          R_cw = RotFromQuatJ(xt(4:7));
  //          h_rz = 1.0e+04*R_cw*[2.1586;0.0708;4.2611];
  //        %  H_g = zeros(3,numStates+numAnchors*numStatesPerAnchor);
  //         % H_g(:,4:6) = skew(h_rz);
  //  %         H = [H; H_g];
  //  %         R = blkdiag(R,r_g);
  //  %         r_g = [r; za/norm(za) - h_rz];
  //      end
  //
  //      if useMagAcc
  //          za=IMU_measurements(4:6);
  //          zm=IMU_measurements(11:13);
  //          z_n_b       = za/norm(za);
  //          m_n_b       = zm/norm(zm);
  //          y_n_b       = cross(z_n_b,m_n_b);
  //          y_n_b       = y_n_b./norm(y_n_b);
  //          x_n_b       = (cross(y_n_b,z_n_b));
  //          x_n_b       = x_n_b./norm(x_n_b);
  //
  //          R_cw_z = [x_n_b,y_n_b,z_n_b];
  //
  //          w_skew=1/2*(R_cw_z'*R_cw-R_cw'*R_cw_z);
  //          err_w=SOtoso(w_skew);
  //          % at the moment nod used
  //          H_ma = zeros(1,size(H,2));
  //  %         alpha=0.9;
  //  %         r(4:6)=alpha*r(4:6)+(1-alpha)*err_w;
  //      end
  //      if useAirPressure
  //          H_p = zeros(1,size(H,2));
  //          H_p(3) = 1;
  //          H = [H; H_p];
  //          R =  blkdiag(R,2);
  //          r = [r;(1-(IMU_measurements(10)/101325)^(0.190284))*145366.45-height_offset_pressure]; 
  //      end
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
//                double c_numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                double r[2]
//                emxArray_real_T *H
//                double h_u[2]
//                double R[4]
// Return Type  : void
//
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[32], double indMeas, const
                emxArray_real_T *map, double c_numAnchors, double
                numPointsPerAnchor, const emxArray_real_T *anchorIdx, const
                emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                *b_m_vect, const double imNoise[2], double r[2], emxArray_real_T
                *H, double h_u[2], double R[4])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ar;
  int nm1d2;
  emxArray_real_T *H_xc;
  double b_map[3];
  double h_ci_l[3];
  int br;
  double h_uin_l[2];
  double h_cin_l[3];
  double radsq_l;
  double b;
  double b_indMeas;
  emxArray_int32_T *r0;
  double x2;
  double y2;
  double a;
  double h_dn_l_To_h_un_l[4];
  double h_un_To_h_d_l[4];
  double dv296[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 537.083588825387, 0.0, 0.0,
    539.036743091681 };

  double d_h_un_To_h_d_l[24];
  double b_stateSize;
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
  double d1;
  double c;
  double dv297[9];
  double b_anchorRot[9];
  emxArray_real_T *H_iy;
  int k;
  static const signed char iv3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *y;
  double b_anchorIdx;
  double dv298[6];
  double b_y[6];
  emxArray_real_T *C;
  double c_y[2];
  int ic;
  int ib;
  int ia;
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
  H_xm->size[1] = (int)(c_numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  nm1d2 = (int)(c_numAnchors * (6.0 + numPointsPerAnchor)) << 1;
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
    b_map[ar] = map->data[ar + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ar];
  }

  for (ar = 0; ar < 3; ar++) {
    h_ci_l[ar] = 0.0;
    for (br = 0; br < 3; br++) {
      h_ci_l[ar] += R_cw[ar + 3 * br] * b_map[br];
    }
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    h_uin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  // PREDICTMEASUREMENT Predict the measurement of a feature given in the left
  // camera frame
  //    Get the pixel coordinates where a feature given in the left camera
  //    frame would be visible in both cameras
  //  camera parameters for the left and right camera
  //  r_lr = cameraparams.r_lr;
  //  R_lr = cameraparams.R_lr;
  //  R_rl=R_lr';
  //  if fp_l(3) == 0
  //      ROS_ERROR('h_ci_l(3)==0')
  //  end
  //  if fp_r(3) == 0
  //      ROS_ERROR('h_ci_r(3)==0')
  //  end
  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  //  if any(isnan(h_cin_l))
  //      ROS_ERROR('h_cin_l')
  //      ROS_ERROR('h_ci_l(3) = %f', h_cin_l(3))
  //  end
  //  if any(isnan(h_cin_r))
  //      ROS_ERROR('h_cin_r')
  //      ROS_ERROR('h_ci_r(3) = %f', h_cin_r(3))
  //  end
  //  h_uin_l = h_cin_l(1:2);
  //  h_uin_r = h_cin_r(1:2);
  //  if any(isnan(h_uin_l))
  //      ROS_ERROR('h_uin_l')
  //  end
  //  if any(isnan(h_uin_r))
  //      ROS_ERROR('h_uin_r')
  //  end
  //  rad_l=sqrt(h_uin_l(1)^2+h_uin_l(2)^2);
  //  rad_r=sqrt(h_uin_r(1)^2+h_uin_r(2)^2);
  radsq_l = h_cin_l[0] * h_cin_l[0] + h_cin_l[1] * h_cin_l[1];
  b = ((1.0 + -0.41042183437511 * radsq_l) + 0.221265330302933 * (radsq_l *
        radsq_l)) + -0.0619354263198911 * rt_powd_snf(radsq_l, 4.0);
  for (ar = 0; ar < 3; ar++) {
    h_cin_l[ar] *= b;
  }

  //  if any(isnan(rad_l))
  //      ROS_ERROR('rad_l')
  //  end
  //  if any(isnan(rad_r))
  //      ROS_ERROR('rad_r')
  //  end
  //  if any(isnan(h_din_l))
  //      ROS_ERROR('h_din_l')
  //  end
  //  if any(isnan(h_din_r))
  //      ROS_ERROR('h_din_r')
  //  end
  //  if any(isnan(h_u_l))
  //      ROS_ERROR('h_di_l')
  //  end
  //  if any(isnan(h_u_r))
  //      ROS_ERROR('h_di_r')
  //  end
  h_u[0] = 370.467750713497 + 537.083588825387 * h_cin_l[0];
  h_u[1] = 226.640025353723 + 539.036743091681 * h_cin_l[1];
  b_indMeas = (indMeas - 1.0) * 2.0;
  for (ar = 0; ar < 2; ar++) {
    r[ar] = z_all_l[(int)(b_indMeas + (1.0 + (double)ar)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  // UNTITLED Summary of this function goes here
  //   computs the derivatives with respect to the normalised undistorted point
  x2 = h_uin_l[0] * h_uin_l[0];
  y2 = h_uin_l[1] * h_uin_l[1];
  a = x2 + y2;
  h_dn_l_To_h_un_l[0] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
    0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
    4.0)) + h_uin_l[0] * ((-0.82084366875022 * h_uin_l[0] + 0.885061321211732 *
    h_uin_l[0] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[0] * rt_powd_snf(x2
    + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
    0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 + y2,
    3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
    0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 + y2,
    3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
    0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
    4.0)) + h_uin_l[1] * ((-0.82084366875022 * h_uin_l[1] + 0.885061321211732 *
    h_uin_l[1] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[1] * rt_powd_snf(x2
    + y2, 3.0));

  //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
  //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
  //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
  //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
  nm1d2 = (int)errorStateSize;
  ar = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  for (ar = 0; ar < nm1d2; ar++) {
    r0->data[ar] = ar;
  }

  dv296[0] = 1.0 / h_ci_l[2];
  dv296[2] = 0.0;
  dv296[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv296[1] = 0.0;
  dv296[3] = 1.0 / h_ci_l[2];
  dv296[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 2; br++) {
      h_un_To_h_d_l[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ar + (br << 1)] += c_h_un_To_h_d_l[ar + (nm1d2 << 1)] *
          h_dn_l_To_h_un_l[nm1d2 + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      b_h_un_To_h_d_l[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ar + (br << 1)] += h_un_To_h_d_l[ar + (nm1d2 << 1)] *
          dv296[nm1d2 + (br << 1)];
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
      d_h_un_To_h_d_l[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        d_h_un_To_h_d_l[ar + (br << 1)] += b_h_un_To_h_d_l[ar + (nm1d2 << 1)] *
          b_R_cw[nm1d2 + 3 * br];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H_xc->data[br + H_xc->size[0] * r0->data[ar]] = d_h_un_To_h_d_l[br + (ar <<
        1)];
    }
  }

  //     %% anchor state derivatives
  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    h_uin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  //     %% computation of H(x)
  // UNTITLED Summary of this function goes here
  //   computs the derivatives with respect to the normalised undistorted point
  x2 = h_uin_l[0] * h_uin_l[0];
  y2 = h_uin_l[1] * h_uin_l[1];
  a = x2 + y2;
  h_dn_l_To_h_un_l[0] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
    0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
    4.0)) + h_uin_l[0] * ((-0.82084366875022 * h_uin_l[0] + 0.885061321211732 *
    h_uin_l[0] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[0] * rt_powd_snf(x2
    + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
    0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 + y2,
    3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * h_uin_l[0] * h_uin_l[1] * ((-0.41042183437511 +
    0.442530660605866 * (x2 + y2)) + -0.24774170527956441 * rt_powd_snf(x2 + y2,
    3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + -0.41042183437511 * (x2 + y2)) +
    0.221265330302933 * (a * a)) + -0.0619354263198911 * rt_powd_snf(x2 + y2,
    4.0)) + h_uin_l[1] * ((-0.82084366875022 * h_uin_l[1] + 0.885061321211732 *
    h_uin_l[1] * (x2 + y2)) + -0.49548341055912881 * h_uin_l[1] * rt_powd_snf(x2
    + y2, 3.0));

  //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
  //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
  //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
  //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
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
  c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1];
  for (ar = 0; ar < 3; ar++) {
    d1 = 0.0;
    for (br = 0; br < 3; br++) {
      d1 += anchorRot[br + 3 * ar] * b_m_vect->data[br + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    h_cin_l[ar] = d1 / c_xt;
  }

  c = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) *
    (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas - 1])
    - 1] * b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] -
    1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
    indMeas - 1]) - 1];
  dv297[0] = 0.0;
  dv297[3] = -h_cin_l[2];
  dv297[6] = h_cin_l[1];
  dv297[1] = h_cin_l[2];
  dv297[4] = 0.0;
  dv297[7] = -h_cin_l[0];
  dv297[2] = -h_cin_l[1];
  dv297[5] = h_cin_l[0];
  dv297[8] = 0.0;
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

    b_map[ar] = d1 / c;
  }

  emxInit_real_T(&H_iy, 2);
  k = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas - 1]);
  ar = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + k) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ar, (int)sizeof(double));
  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ar] = iv3[br + 3 * ar];
    }
  }

  for (ar = 0; ar < 3; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * (ar + 3)] = -dv297[br + 3 * ar];
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

  for (ar = 0; ar < k; ar++) {
    for (br = 0; br < 3; br++) {
      H_iy->data[br + H_iy->size[0] * ((ar + nm1d2) + 7)] = 0.0;
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

  emxInit_real_T(&y, 2);
  ar = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = br + 1;
  emxEnsureCapacity((emxArray__common *)y, ar, (int)sizeof(double));
  if (br + 1 > 0) {
    y->data[0] = anew;
    if (br + 1 > 1) {
      y->data[br] = apnd;
      nm1d2 = (br + (br < 0)) >> 1;
      for (k = 1; k < nm1d2; k++) {
        y->data[k] = anew + (double)k;
        y->data[br - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == br) {
        y->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        y->data[nm1d2] = anew + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  b_anchorIdx = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 +
    numPointsPerAnchor);
  ar = r0->size[0];
  r0->size[0] = y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ar, (int)sizeof(int));
  nm1d2 = y->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    r0->data[ar] = (int)(b_anchorIdx + y->data[y->size[0] * ar]) - 1;
  }

  emxFree_real_T(&y);
  dv298[0] = 1.0 / h_ci_l[2];
  dv298[2] = 0.0;
  dv298[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv298[1] = 0.0;
  dv298[3] = 1.0 / h_ci_l[2];
  dv298[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ar = 0; ar < 2; ar++) {
    for (br = 0; br < 2; br++) {
      h_un_To_h_d_l[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ar + (br << 1)] += c_h_un_To_h_d_l[ar + (nm1d2 << 1)] *
          h_dn_l_To_h_un_l[nm1d2 + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      b_h_un_To_h_d_l[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ar + (br << 1)] += h_un_To_h_d_l[ar + (nm1d2 << 1)] *
          dv298[nm1d2 + (br << 1)];
      }
    }

    for (br = 0; br < 3; br++) {
      b_y[ar + (br << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        b_y[ar + (br << 1)] += b_h_un_To_h_d_l[ar + (nm1d2 << 1)] * R_cw[nm1d2 +
          3 * br];
      }
    }
  }

  emxInit_real_T(&C, 2);
  c_y[1] = H_iy->size[1];
  ar = C->size[0] * C->size[1];
  C->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
  ar = C->size[0] * C->size[1];
  C->size[1] = (int)c_y[1];
  emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
  nm1d2 = (int)c_y[1] << 1;
  for (ar = 0; ar < nm1d2; ar++) {
    C->data[ar] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (k = 0; k <= nm1d2; k += 2) {
    for (ic = k; ic + 1 <= k + 2; ic++) {
      C->data[ic] = 0.0;
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
          C->data[ic] += H_iy->data[ib] * b_y[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  emxFree_real_T(&H_iy);
  nm1d2 = C->size[1];
  for (ar = 0; ar < nm1d2; ar++) {
    for (br = 0; br < 2; br++) {
      H_xm->data[br + H_xm->size[0] * r0->data[ar]] = C->data[br + C->size[0] *
        ar];
    }
  }

  emxFree_real_T(&C);
  emxFree_int32_T(&r0);
  for (ar = 0; ar < 2; ar++) {
    r[ar] -= h_u[ar];
  }

  //  residual with respect to camera measurements
  for (k = 0; k < 2; k++) {
    c_y[k] = imNoise[k] * imNoise[k];
  }

  for (ar = 0; ar < 4; ar++) {
    h_dn_l_To_h_un_l[ar] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    h_dn_l_To_h_un_l[nm1d2 + (nm1d2 << 1)] = c_y[nm1d2];
  }

  nm1d2 = -1;
  for (k = 0; k < 2; k++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R[nm1d2] = h_dn_l_To_h_un_l[br + (k << 1)];
    }
  }

  ar = H->size[0] * H->size[1];
  H->size[0] = 2;
  H->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
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

  //      if gravityUpdate
  //          R_cw = RotFromQuatJ(x(4:7));
  //          h_rz = R_cw*[0;0;1];
  //          H_g = zeros(3,numStates+numAnchors*numStatesPerAnchor);
  //          H_g(:,4:6) = skew(h_rz);
  //          H = [H; H_g];
  //          R = blkdiag(R,r_g);
  //          r_g = [r; za/norm(za) - h_rz];
  //      end
  //
  //      if MagUpdate
  //          R_cw = RotFromQuatJ(xt(4:7));
  //          h_rz = 1.0e+04*R_cw*[2.1586;0.0708;4.2611];
  //        %  H_g = zeros(3,numStates+numAnchors*numStatesPerAnchor);
  //         % H_g(:,4:6) = skew(h_rz);
  //  %         H = [H; H_g];
  //  %         R = blkdiag(R,r_g);
  //  %         r_g = [r; za/norm(za) - h_rz];
  //      end
  //
  //      if useMagAcc
  //          za=IMU_measurements(4:6);
  //          zm=IMU_measurements(11:13);
  //          z_n_b       = za/norm(za);
  //          m_n_b       = zm/norm(zm);
  //          y_n_b       = cross(z_n_b,m_n_b);
  //          y_n_b       = y_n_b./norm(y_n_b);
  //          x_n_b       = (cross(y_n_b,z_n_b));
  //          x_n_b       = x_n_b./norm(x_n_b);
  //
  //          R_cw_z = [x_n_b,y_n_b,z_n_b];
  //
  //          w_skew=1/2*(R_cw_z'*R_cw-R_cw'*R_cw_z);
  //          err_w=SOtoso(w_skew);
  //          % at the moment nod used
  //          H_ma = zeros(1,size(H,2));
  //  %         alpha=0.9;
  //  %         r(4:6)=alpha*r(4:6)+(1-alpha)*err_w;
  //      end
  //      if useAirPressure
  //          H_p = zeros(1,size(H,2));
  //          H_p(3) = 1;
  //          H = [H; H_p];
  //          R =  blkdiag(R,2);
  //          r = [r;(1-(IMU_measurements(10)/101325)^(0.190284))*145366.45-height_offset_pressure]; 
  //      end
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
