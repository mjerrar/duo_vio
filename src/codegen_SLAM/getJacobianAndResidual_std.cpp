//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getJacobianAndResidual_std.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getJacobianAndResidual_std.h"
#include "SLAM_emxutil.h"
#include "fprintf.h"
#include "QuatFromRotJ.h"
#include "initializePoint.h"
#include "any.h"
#include "RotFromQuatJ.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// GETJACOBIANANDRESIDUAL Get measurement Jacobian H and residual r
//    Uses the ocam model and builds the residual in the ray space
//
//  INPUT ARGUMENTS:
//  - xt:              The current state
//  - errorStateSize:  The size of the error state
//  - z_all_l:         The feature points in the left camera (2N x 1)
//  - z_all_r:         The feature points in the right camera (2N x 1)
//  - map:             Map of the estimated feature points (3 x N)
//  - indMeas:         The indices of the valid feature points
//  - cameraparams:    A struct with the fields
//     - TODO          ???
//     - r_lr:         Translation of right camera in left camera frame
//     - R_lr:         Orientation of right camera in left camera frame
//
//  where N = length(indMeas) is number of valid points in the image
//
//  OUTPUT ARGUMENTS:
//  - r: The residual (6N x 1)
//  - H: The Jacobian of the measurement function (6N x length(xt))
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                const double z_all_l[64]
//                const double z_all_r[64]
//                const double b_map[96]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H_xc
//                double h_u_data[]
//                int h_u_size[1]
// Return Type  : void
//
void getJacobianAndResidual_std(const emxArray_real_T *b_xt, double
  errorStateSize, const double z_all_l[64], const double z_all_r[64], const
  double b_map[96], const double indMeas_data[], const int indMeas_size[1],
  const double cameraparams_r_lr[3], const double cameraparams_R_lr[9], double
  r_data[], int r_size[1], emxArray_real_T *H_xc, double h_u_data[], int
  h_u_size[1])
{
  double R_cw[9];
  boolean_T bv0[9];
  int i2;
  boolean_T bv1[3];
  boolean_T b[3];
  int i;
  boolean_T y;
  int k;
  boolean_T exitg9;
  emxArray_int32_T *r0;
  double c_map[3];
  double h_ci_l[3];
  int i3;
  double b_R_cw[3];
  double h_ci_r[3];
  double d0;
  double h_cin_l[3];
  double h_cin_r[3];
  double b_h_cin_l;
  boolean_T exitg8;
  boolean_T exitg7;
  boolean_T b_b[2];
  boolean_T exitg6;
  boolean_T exitg5;
  double rad_l;
  double rad_r;
  double c_b;
  double d_b;
  double h_din_l[2];
  double h_din_r[2];
  double b_h_din_l;
  boolean_T exitg4;
  boolean_T exitg3;
  double h_di_l[2];
  boolean_T exitg2;
  boolean_T exitg1;
  double x2;
  double y2;
  double h_dn_l_To_h_un_l[4];
  double h_dn_r_To_h_un_r[4];
  double indMeas;
  double b_indMeas;
  unsigned char b_k;
  unsigned char uv0[4];
  double h_un_To_h_d_l[4];
  double dv614[6];
  double b_h_un_To_h_d_l[6];
  double c_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 538.062556767505, 0.0, 0.0,
    540.300263920769 };

  static const double h_un_To_h_d_r[4] = { 538.360393533763, 0.0, 0.0,
    540.849047073219 };

  double dv615[6];
  double b_cameraparams_R_lr[9];
  double dv616[9];
  double c_cameraparams_R_lr[9];
  double d_cameraparams_R_lr[9];
  double b_h_un_To_h_d_r[6];
  double e_cameraparams_R_lr[36];
  double d_h_un_To_h_d_l[24];
  double c_h_un_To_h_d_r[24];
  double e_h_un_To_h_d_l[48];
  RotFromQuatJ(*(double (*)[4])&b_xt->data[3], R_cw);
  for (i2 = 0; i2 < 9; i2++) {
    bv0[i2] = rtIsNaN(R_cw[i2]);
  }

  any(bv0, bv1);
  if (b_any(bv1)) {
    b_fprintf();
  }

  for (i2 = 0; i2 < 3; i2++) {
    b[i2] = rtIsNaN(b_xt->data[i2]);
  }

  if (!!c_any(b)) {
    d_fprintf();
  }

  for (i = 0; i < 3; i++) {
    b[i] = rtIsNaN(cameraparams_r_lr[i]);
  }

  if (!!c_any(b)) {
    f_fprintf();
  }

  for (i2 = 0; i2 < 9; i2++) {
    bv0[i2] = rtIsNaN(cameraparams_R_lr[i2]);
  }

  any(bv0, b);
  y = false;
  k = 0;
  exitg9 = false;
  while ((!exitg9) && (k < 3)) {
    if (!!b[k]) {
      y = true;
      exitg9 = true;
    } else {
      k++;
    }
  }

  if (y) {
    h_fprintf();
  }

  //  camera parameters for the left and right camera
  i2 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 2;
  emxEnsureCapacity((emxArray__common *)H_xc, i2, (int)sizeof(double));
  i2 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, i2, (int)sizeof(double));
  i = (indMeas_size[0] << 2) * (int)errorStateSize;
  for (i2 = 0; i2 < i; i2++) {
    H_xc->data[i2] = 0.0;
  }

  r_size[0] = indMeas_size[0] << 2;
  i = indMeas_size[0] << 2;
  for (i2 = 0; i2 < i; i2++) {
    r_data[i2] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 2;
  i = indMeas_size[0] << 2;
  for (i2 = 0; i2 < i; i2++) {
    h_u_data[i2] = 0.0;
  }

  k = 0;
  emxInit_int32_T(&r0, 1);
  while (k <= indMeas_size[0] - 1) {
    for (i2 = 0; i2 < 3; i2++) {
      c_map[i2] = b_map[i2 + 3 * ((int)indMeas_data[k] - 1)] - b_xt->data[i2];
    }

    for (i2 = 0; i2 < 3; i2++) {
      h_ci_l[i2] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        h_ci_l[i2] += R_cw[i2 + 3 * i3] * c_map[i3];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      c_map[i2] = b_map[i2 + 3 * ((int)indMeas_data[k] - 1)] - b_xt->data[i2];
    }

    for (i2 = 0; i2 < 3; i2++) {
      b_R_cw[i2] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        b_R_cw[i2] += R_cw[i2 + 3 * i3] * c_map[i3];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      d0 = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        d0 += cameraparams_R_lr[i3 + 3 * i2] * b_R_cw[i3];
      }

      h_ci_r[i2] = d0 - cameraparams_r_lr[i2];
    }

    if (h_ci_l[2] == 0.0) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      j_fprintf(i2);
    }

    if (h_ci_r[2] == 0.0) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      l_fprintf(i2);
    }

    for (i = 0; i < 3; i++) {
      b_h_cin_l = h_ci_l[i] / h_ci_l[2];
      h_cin_r[i] = h_ci_r[i] / h_ci_r[2];
      b[i] = rtIsNaN(b_h_cin_l);
      h_cin_l[i] = b_h_cin_l;
    }

    y = false;
    i = 0;
    exitg8 = false;
    while ((!exitg8) && (i < 3)) {
      if (!!b[i]) {
        y = true;
        exitg8 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      n_fprintf(i2);
      p_fprintf(h_ci_l[2]);
    }

    for (i = 0; i < 3; i++) {
      b[i] = rtIsNaN(h_cin_r[i]);
    }

    y = false;
    i = 0;
    exitg7 = false;
    while ((!exitg7) && (i < 3)) {
      if (!!b[i]) {
        y = true;
        exitg7 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      r_fprintf(i2);
      t_fprintf(h_ci_r[2]);
    }

    for (i = 0; i < 2; i++) {
      b_b[i] = rtIsNaN(h_cin_l[i]);
    }

    y = false;
    i = 0;
    exitg6 = false;
    while ((!exitg6) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg6 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      v_fprintf(i2);
    }

    for (i = 0; i < 2; i++) {
      b_b[i] = rtIsNaN(h_cin_r[i]);
    }

    y = false;
    i = 0;
    exitg5 = false;
    while ((!exitg5) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg5 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      x_fprintf(i2);
    }

    rad_l = sqrt(h_cin_l[0] * h_cin_l[0] + h_cin_l[1] * h_cin_l[1]);
    rad_r = sqrt(h_cin_r[0] * h_cin_r[0] + h_cin_r[1] * h_cin_r[1]);
    if (!!rtIsNaN(rad_l)) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      ab_fprintf(i2);
    }

    if (!!rtIsNaN(rad_r)) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      cb_fprintf(i2);
    }

    c_b = (1.0 + -0.403158360273884 * (rad_l * rad_l)) + 0.203403755605372 *
      rt_powd_snf(rad_l, 4.0);
    d_b = (1.0 + -0.397408316532857 * (rad_r * rad_r)) + 0.190419257967814 *
      rt_powd_snf(rad_r, 4.0);
    for (i = 0; i < 2; i++) {
      b_h_din_l = h_cin_l[i] * c_b;
      h_din_r[i] = h_cin_r[i] * d_b;
      b_b[i] = rtIsNaN(b_h_din_l);
      h_din_l[i] = b_h_din_l;
    }

    y = false;
    i = 0;
    exitg4 = false;
    while ((!exitg4) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg4 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      eb_fprintf(i2);
    }

    for (i = 0; i < 2; i++) {
      b_b[i] = rtIsNaN(h_din_r[i]);
    }

    y = false;
    i = 0;
    exitg3 = false;
    while ((!exitg3) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg3 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      gb_fprintf(i2);
    }

    h_di_l[0] = 370.875239827556 + 538.062556767505 * h_din_l[0];
    h_di_l[1] = 233.561696038582 + 540.300263920769 * h_din_l[1];
    h_din_l[0] = 390.89484296851 + 538.360393533763 * h_din_r[0];
    h_din_l[1] = 222.869843438459 + 540.849047073219 * h_din_r[1];
    for (i = 0; i < 2; i++) {
      b_b[i] = rtIsNaN(h_di_l[i]);
    }

    y = false;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg2 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      ib_fprintf(i2);
    }

    for (i = 0; i < 2; i++) {
      b_b[i] = rtIsNaN(h_din_l[i]);
    }

    y = false;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 2)) {
      if (!!b_b[i]) {
        y = true;
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (y) {
      d0 = rt_roundd_snf(indMeas_data[k]);
      if (d0 < 2.147483648E+9) {
        if (d0 >= -2.147483648E+9) {
          i2 = (int)d0;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d0 >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      kb_fprintf(i2);
    }

    // UNTITLED Summary of this function goes here
    //   computs the derivatives with respect to the normalised undistorted point 
    //  h_dn_l_To_h_un_l(1,1)=1 + k1*(x2 + y2) + k2*(x2 + y2)^2 + k3*(x2 + y2)^4 + x*(2*k1*x + 4*k2*x*(x2 + y2) + 8 *k3* x*(x2 + y2)^3); 
    //  h_dn_l_To_h_un_l(1,2)=2*x*y*(k1 + 2*k2*(x2 + y2) + 4*k3*(x2 + y2)^3);
    //  h_dn_l_To_h_un_l(2,1)=2*x*y*(k1 + 2*k2*(x2 + y2) + 4*k3*(x2 + y2)^3);
    //  h_dn_l_To_h_un_l(2,2)=1 + k1*(x2 + y2) + k2*(x2 + y2)^2 + k3*(x2 + y2)^4 + y*(2*k1*y + 4*k2*y*(x2 + y2) + 8*k3*y*(x2 + y2)^3); 
    x2 = h_cin_l[0] * h_cin_l[0];
    y2 = h_cin_l[1] * h_cin_l[1];
    h_dn_l_To_h_un_l[0] = (1.0 + -0.403158360273884 * (3.0 * x2 + y2)) +
      0.203403755605372 * ((5.0 * (x2 * x2) + 6.0 * x2 * y2) + y2 * y2);
    h_dn_l_To_h_un_l[2] = 2.0 * h_cin_l[0] * h_cin_l[1] * (-0.403158360273884 +
      0.406807511210744 * (x2 + y2));
    h_dn_l_To_h_un_l[1] = 2.0 * h_cin_l[0] * h_cin_l[1] * (-0.403158360273884 +
      0.406807511210744 * (x2 + y2));
    h_dn_l_To_h_un_l[3] = (1.0 + -0.403158360273884 * (x2 + 3.0 * y2)) +
      0.203403755605372 * ((x2 * x2 + 6.0 * x2 * y2) + 5.0 * (y2 * y2));

    // UNTITLED Summary of this function goes here
    //   computs the derivatives with respect to the normalised undistorted point 
    //  h_dn_l_To_h_un_l(1,1)=1 + k1*(x2 + y2) + k2*(x2 + y2)^2 + k3*(x2 + y2)^4 + x*(2*k1*x + 4*k2*x*(x2 + y2) + 8 *k3* x*(x2 + y2)^3); 
    //  h_dn_l_To_h_un_l(1,2)=2*x*y*(k1 + 2*k2*(x2 + y2) + 4*k3*(x2 + y2)^3);
    //  h_dn_l_To_h_un_l(2,1)=2*x*y*(k1 + 2*k2*(x2 + y2) + 4*k3*(x2 + y2)^3);
    //  h_dn_l_To_h_un_l(2,2)=1 + k1*(x2 + y2) + k2*(x2 + y2)^2 + k3*(x2 + y2)^4 + y*(2*k1*y + 4*k2*y*(x2 + y2) + 8*k3*y*(x2 + y2)^3); 
    x2 = h_cin_r[0] * h_cin_r[0];
    y2 = h_cin_r[1] * h_cin_r[1];
    h_dn_r_To_h_un_r[0] = (1.0 + -0.397408316532857 * (3.0 * x2 + y2)) +
      0.190419257967814 * ((5.0 * (x2 * x2) + 6.0 * x2 * y2) + y2 * y2);
    h_dn_r_To_h_un_r[2] = 2.0 * h_cin_r[0] * h_cin_r[1] * (-0.397408316532857 +
      0.380838515935628 * (x2 + y2));
    h_dn_r_To_h_un_r[1] = 2.0 * h_cin_r[0] * h_cin_r[1] * (-0.397408316532857 +
      0.380838515935628 * (x2 + y2));
    h_dn_r_To_h_un_r[3] = (1.0 + -0.397408316532857 * (x2 + 3.0 * y2)) +
      0.190419257967814 * ((x2 * x2 + 6.0 * x2 * y2) + 5.0 * (y2 * y2));
    i = k << 2;
    indMeas = (indMeas_data[k] - 1.0) * 2.0;
    b_indMeas = (indMeas_data[k] - 1.0) * 2.0;
    for (i2 = 0; i2 < 2; i2++) {
      r_data[i2 + i] = z_all_l[(int)(indMeas + (1.0 + (double)i2)) - 1];
    }

    for (i2 = 0; i2 < 2; i2++) {
      r_data[(i2 + i) + 2] = z_all_r[(int)(b_indMeas + (1.0 + (double)i2)) - 1];
    }

    // %%%%%%%%%%%%%%%%%%%%%%%
    //     %% computation of H(x)%%
    // %%%%%%%%%%%%%%%%%%%%%%%
    //  nonlinear predicted measurement
    i = k << 2;
    for (i2 = 0; i2 < 2; i2++) {
      h_u_data[i2 + i] = h_di_l[i2];
    }

    for (i2 = 0; i2 < 2; i2++) {
      h_u_data[(i2 + i) + 2] = h_din_l[i2];
    }

    //  stack the matrices for all feature measurements
    b_k = (unsigned char)((unsigned char)k << 2);
    for (i2 = 0; i2 < 4; i2++) {
      uv0[i2] = (unsigned char)((unsigned int)i2 + b_k);
    }

    i = H_xc->size[1];
    i2 = r0->size[0];
    r0->size[0] = i;
    emxEnsureCapacity((emxArray__common *)r0, i2, (int)sizeof(int));
    for (i2 = 0; i2 < i; i2++) {
      r0->data[i2] = i2;
    }

    dv614[0] = 1.0 / h_ci_l[2];
    dv614[2] = 0.0;
    dv614[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    dv614[1] = 0.0;
    dv614[3] = 1.0 / h_ci_l[2];
    dv614[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    for (i2 = 0; i2 < 2; i2++) {
      for (i3 = 0; i3 < 2; i3++) {
        h_un_To_h_d_l[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 2; i++) {
          h_un_To_h_d_l[i2 + (i3 << 1)] += c_h_un_To_h_d_l[i2 + (i << 1)] *
            h_dn_l_To_h_un_l[i + (i3 << 1)];
        }
      }

      for (i3 = 0; i3 < 3; i3++) {
        b_h_un_To_h_d_l[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 2; i++) {
          b_h_un_To_h_d_l[i2 + (i3 << 1)] += h_un_To_h_d_l[i2 + (i << 1)] *
            dv614[i + (i3 << 1)];
        }
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        c_R_cw[i3 + 3 * i2] = -R_cw[i3 + 3 * i2];
      }
    }

    c_R_cw[9] = 0.0;
    c_R_cw[12] = -h_ci_l[2];
    c_R_cw[15] = h_ci_l[1];
    c_R_cw[10] = h_ci_l[2];
    c_R_cw[13] = 0.0;
    c_R_cw[16] = -h_ci_l[0];
    c_R_cw[11] = -h_ci_l[1];
    c_R_cw[14] = h_ci_l[0];
    c_R_cw[17] = 0.0;
    for (i2 = 0; i2 < 6; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        c_R_cw[i3 + 3 * (i2 + 6)] = 0.0;
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      for (i3 = 0; i3 < 2; i3++) {
        h_un_To_h_d_l[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 2; i++) {
          h_un_To_h_d_l[i2 + (i3 << 1)] += h_un_To_h_d_r[i2 + (i << 1)] *
            h_dn_r_To_h_un_r[i + (i3 << 1)];
        }
      }
    }

    dv615[0] = 1.0 / h_ci_r[2];
    dv615[2] = 0.0;
    dv615[4] = -h_ci_r[0] / (h_ci_r[2] * h_ci_r[2]);
    dv615[1] = 0.0;
    dv615[3] = 1.0 / h_ci_r[2];
    dv615[5] = -h_ci_r[1] / (h_ci_r[2] * h_ci_r[2]);
    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        b_cameraparams_R_lr[i3 + 3 * i2] = -cameraparams_R_lr[i2 + 3 * i3];
      }
    }

    dv616[0] = 0.0;
    dv616[3] = -h_ci_r[2];
    dv616[6] = h_ci_r[1];
    dv616[1] = h_ci_r[2];
    dv616[4] = 0.0;
    dv616[7] = -h_ci_r[0];
    dv616[2] = -h_ci_r[1];
    dv616[5] = h_ci_r[0];
    dv616[8] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        c_cameraparams_R_lr[i2 + 3 * i3] = 0.0;
        for (i = 0; i < 3; i++) {
          c_cameraparams_R_lr[i2 + 3 * i3] += b_cameraparams_R_lr[i2 + 3 * i] *
            R_cw[i + 3 * i3];
        }

        d_cameraparams_R_lr[i2 + 3 * i3] = 0.0;
        for (i = 0; i < 3; i++) {
          d_cameraparams_R_lr[i2 + 3 * i3] += cameraparams_R_lr[i + 3 * i2] *
            dv616[i + 3 * i3];
        }
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        b_h_un_To_h_d_r[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 2; i++) {
          b_h_un_To_h_d_r[i2 + (i3 << 1)] += h_un_To_h_d_l[i2 + (i << 1)] *
            dv615[i + (i3 << 1)];
        }
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        e_cameraparams_R_lr[i3 + 3 * i2] = c_cameraparams_R_lr[i3 + 3 * i2];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        e_cameraparams_R_lr[i3 + 3 * (i2 + 3)] = d_cameraparams_R_lr[i3 + 3 * i2];
      }
    }

    for (i2 = 0; i2 < 6; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        e_cameraparams_R_lr[i3 + 3 * (i2 + 6)] = 0.0;
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      for (i3 = 0; i3 < 12; i3++) {
        d_h_un_To_h_d_l[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 3; i++) {
          d_h_un_To_h_d_l[i2 + (i3 << 1)] += b_h_un_To_h_d_l[i2 + (i << 1)] *
            c_R_cw[i + 3 * i3];
        }

        c_h_un_To_h_d_r[i2 + (i3 << 1)] = 0.0;
        for (i = 0; i < 3; i++) {
          c_h_un_To_h_d_r[i2 + (i3 << 1)] += b_h_un_To_h_d_r[i2 + (i << 1)] *
            e_cameraparams_R_lr[i + 3 * i3];
        }
      }
    }

    for (i2 = 0; i2 < 12; i2++) {
      for (i3 = 0; i3 < 2; i3++) {
        e_h_un_To_h_d_l[i3 + (i2 << 2)] = d_h_un_To_h_d_l[i3 + (i2 << 1)];
      }

      for (i3 = 0; i3 < 2; i3++) {
        e_h_un_To_h_d_l[(i3 + (i2 << 2)) + 2] = c_h_un_To_h_d_r[i3 + (i2 << 1)];
      }
    }

    i = r0->size[0];
    for (i2 = 0; i2 < i; i2++) {
      for (i3 = 0; i3 < 4; i3++) {
        H_xc->data[uv0[i3] + H_xc->size[0] * r0->data[i2]] = e_h_un_To_h_d_l[i3
          + (i2 << 2)];
      }
    }

    k++;
  }

  emxFree_int32_T(&r0);
  i = r_size[0];
  for (i2 = 0; i2 < i; i2++) {
    r_data[i2] -= h_u_data[i2];
  }
}

//
// File trailer for getJacobianAndResidual_std.cpp
//
// [EOF]
//
