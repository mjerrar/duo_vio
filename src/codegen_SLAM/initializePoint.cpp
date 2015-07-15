//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: initializePoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "initializePoint.h"
#include "svd.h"
#include "norm.h"
#include "QuatFromRotJ.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Declarations
static void b_eml_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double f_eml_xnrm2(int n, const double x[30], int ix0);

// Function Definitions

//
// Arguments    : double A[30]
//                double tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_eml_xgeqp3(double A[30], double tau[5], int jpvt[5])
{
  double work[5];
  int i20;
  double vn1[5];
  double vn2[5];
  int k;
  int iy;
  double smax;
  double temp2;
  int itemp;
  double absxk;
  double t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int32_T exitg1;
  for (i20 = 0; i20 < 5; i20++) {
    jpvt[i20] = 1 + i20;
    work[i20] = 0.0;
  }

  k = 1;
  for (iy = 0; iy < 5; iy++) {
    smax = 0.0;
    temp2 = 2.2250738585072014E-308;
    for (itemp = k; itemp <= k + 5; itemp++) {
      absxk = fabs(A[itemp - 1]);
      if (absxk > temp2) {
        t = temp2 / absxk;
        smax = 1.0 + smax * t * t;
        temp2 = absxk;
      } else {
        t = absxk / temp2;
        smax += t * t;
      }
    }

    smax = temp2 * sqrt(smax);
    vn1[iy] = smax;
    vn2[iy] = vn1[iy];
    k += 6;
  }

  for (i = 0; i < 5; i++) {
    i_i = i + i * 6;
    itemp = 0;
    if (5 - i > 1) {
      ix = i;
      smax = fabs(vn1[i]);
      for (k = 1; k + 1 <= 5 - i; k++) {
        ix++;
        temp2 = fabs(vn1[ix]);
        if (temp2 > smax) {
          itemp = k;
          smax = temp2;
        }
      }
    }

    pvt = i + itemp;
    if (pvt + 1 != i + 1) {
      ix = 6 * pvt;
      iy = 6 * i;
      for (k = 0; k < 6; k++) {
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix++;
        iy++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = itemp;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    absxk = A[i_i];
    temp2 = 0.0;
    smax = f_eml_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0) {
      smax = rt_hypotd_snf(A[i_i], smax);
      if (A[i_i] >= 0.0) {
        smax = -smax;
      }

      if (fabs(smax) < 1.0020841800044864E-292) {
        itemp = 0;
        do {
          itemp++;
          i20 = i_i - i;
          for (k = i_i + 1; k + 1 <= i20 + 6; k++) {
            A[k] *= 9.9792015476736E+291;
          }

          smax *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(fabs(smax) >= 1.0020841800044864E-292));

        smax = rt_hypotd_snf(absxk, f_eml_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0 / (absxk - smax);
        i20 = i_i - i;
        for (k = i_i + 1; k + 1 <= i20 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i20 = i_i - i;
        for (k = i_i + 1; k + 1 <= i20 + 6; k++) {
          A[k] *= absxk;
        }

        absxk = smax;
      }
    }

    tau[i] = temp2;
    A[i_i] = absxk;
    if (i + 1 < 5) {
      absxk = A[i_i];
      A[i_i] = 1.0;
      i_ip1 = (i + (i + 1) * 6) + 1;
      if (tau[i] != 0.0) {
        lastv = 6 - i;
        itemp = i_i - i;
        while ((lastv > 0) && (A[itemp + 5] == 0.0)) {
          lastv--;
          itemp--;
        }

        lastc = 4 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          itemp = i_ip1 + (lastc - 1) * 6;
          k = itemp;
          do {
            exitg1 = 0;
            if (k <= (itemp + lastv) - 1) {
              if (A[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc == 0) {
        } else {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0;
          }

          iy = 0;
          i20 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i20; itemp += 6) {
            ix = i_i;
            smax = 0.0;
            pvt = (itemp + lastv) - 1;
            for (k = itemp; k <= pvt; k++) {
              smax += A[k - 1] * A[ix];
              ix++;
            }

            work[iy] += smax;
            iy++;
          }
        }

        if (-tau[i] == 0.0) {
        } else {
          itemp = i_ip1 - 1;
          pvt = 0;
          for (iy = 1; iy <= lastc; iy++) {
            if (work[pvt] != 0.0) {
              smax = work[pvt] * -tau[i];
              ix = i_i;
              i20 = lastv + itemp;
              for (k = itemp; k + 1 <= i20; k++) {
                A[k] += A[ix] * smax;
                ix++;
              }
            }

            pvt++;
            itemp += 6;
          }
        }
      }

      A[i_i] = absxk;
    }

    for (iy = i + 1; iy + 1 < 6; iy++) {
      if (vn1[iy] != 0.0) {
        smax = fabs(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          vn1[iy] = d_eml_xnrm2(5 - i, A, (i + 6 * iy) + 2);
          vn2[iy] = vn1[iy];
        } else {
          vn1[iy] *= sqrt(smax);
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
static double f_eml_xnrm2(int n, const double x[30], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
double d_eml_xnrm2(int n, const double x[30], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// INITIALIZEPOINT Initialize a feature point from current measurements
//
//  INPUT ARGUMENTS:
//  - xt: The current camera state
//  - cameraparams: The camera parameter struct
//  - z_l: The measurement of the feature in the left camera
//  - z_r: The measurement of the feature in the right camera
//
//  OUTPUT ARGUMENTS:
//  - fp: The feature point in the world frame
// Arguments    : const emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                const double z_l[2]
//                const double z_r[2]
//                double fp[3]
// Return Type  : void
//
void initializePoint(const emxArray_real_T *b_xt, const double
                     cameraparams_r_lr[3], const double cameraparams_R_lr[9],
                     const double z_l[2], const double z_r[2], double fp[3])
{
  double R_cw[9];
  double c_xt[3];
  int i8;
  double tol;
  int i9;
  double pos[6];
  double dv617[4];
  double b_cameraparams_R_lr[9];
  int i;
  double dv618[4];
  double rot[8];
  double zn_d_l[2];
  double zn_d_r[2];
  double rad_d_l;
  double rad_d_r;
  double r_u_l;
  double r_u_r;
  double b_r_u_l[2];
  boolean_T b[2];
  boolean_T y;
  boolean_T exitg2;
  boolean_T guard1 = false;
  double c_r_u_l[2];
  boolean_T exitg1;
  double B;
  double ml[3];
  double mr[3];
  double m[6];
  double A[30];
  double b_b[6];
  int rankR;
  int j;
  double b_rot[9];
  double unusedExpr[5];
  int jpvt[5];
  double tau[5];
  double x[5];

  //  camera parameters for the left and right camera
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
  for (i8 = 0; i8 < 3; i8++) {
    tol = 0.0;
    for (i9 = 0; i9 < 3; i9++) {
      tol += R_cw[i8 + 3 * i9] * cameraparams_r_lr[i9];
    }

    c_xt[i8] = b_xt->data[i8] + tol;
  }

  for (i8 = 0; i8 < 3; i8++) {
    pos[i8] = b_xt->data[i8];
  }

  for (i8 = 0; i8 < 3; i8++) {
    pos[3 + i8] = c_xt[i8];
  }

  QuatFromRotJ(R_cw, dv617);
  for (i8 = 0; i8 < 3; i8++) {
    for (i9 = 0; i9 < 3; i9++) {
      b_cameraparams_R_lr[i8 + 3 * i9] = 0.0;
      for (i = 0; i < 3; i++) {
        b_cameraparams_R_lr[i8 + 3 * i9] += cameraparams_R_lr[i + 3 * i8] *
          R_cw[i + 3 * i9];
      }
    }
  }

  QuatFromRotJ(b_cameraparams_R_lr, dv618);
  for (i8 = 0; i8 < 4; i8++) {
    rot[i8] = dv617[i8];
    rot[4 + i8] = dv618[i8];
  }

  zn_d_l[0] = (z_l[0] - 370.875239827556) / 538.062556767505;
  zn_d_l[1] = (z_l[1] - 233.561696038582) / 540.300263920769;
  zn_d_r[0] = (z_r[0] - 390.89484296851) / 538.360393533763;
  zn_d_r[1] = (z_r[1] - 222.869843438459) / 540.849047073219;
  rad_d_l = sqrt(zn_d_l[0] * zn_d_l[0] + zn_d_l[1] * zn_d_l[1]);

  //  the radius for the undistortion
  rad_d_r = sqrt(zn_d_r[0] * zn_d_r[0] + zn_d_r[1] * zn_d_r[1]);

  // R_U Summary of this function goes here
  //    Detailed explanation goes here
  r_u_l = 1.0;

  // R_U Summary of this function goes here
  //    Detailed explanation goes here
  r_u_r = 1.0;
  for (i = 0; i < 10; i++) {
    r_u_l -= (((r_u_l + -0.403158360273884 * rt_powd_snf(r_u_l, 3.0)) +
               0.203403755605372 * rt_powd_snf(r_u_l, 5.0)) - rad_d_l) / ((1.0 +
      -1.2094750808216521 * (r_u_l * r_u_l)) + 0.813615022421488 * rt_powd_snf
      (r_u_l, 3.0));
    r_u_r -= (((r_u_r + -0.397408316532857 * rt_powd_snf(r_u_r, 3.0)) +
               0.190419257967814 * rt_powd_snf(r_u_r, 5.0)) - rad_d_r) / ((1.0 +
      -1.1922249495985711 * (r_u_r * r_u_r)) + 0.761677031871256 * rt_powd_snf
      (r_u_r, 3.0));
  }

  b_r_u_l[0] = r_u_l;
  b_r_u_l[1] = r_u_r;
  for (i8 = 0; i8 < 2; i8++) {
    b[i8] = rtIsNaN(b_r_u_l[i8]);
  }

  y = false;
  i = 0;
  exitg2 = false;
  while ((!exitg2) && (i < 2)) {
    if (!!b[i]) {
      y = true;
      exitg2 = true;
    } else {
      i++;
    }
  }

  guard1 = false;
  if (!y) {
    c_r_u_l[0] = r_u_l;
    c_r_u_l[1] = r_u_r;
    for (i8 = 0; i8 < 2; i8++) {
      b[i8] = rtIsInf(c_r_u_l[i8]);
    }

    y = false;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 2)) {
      if (!!b[i]) {
        y = true;
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (!y) {
      B = (1.0 + -0.403158360273884 * (r_u_l * r_u_l)) + 0.203403755605372 *
        rt_powd_snf(r_u_l, 4.0);

      // undistort points
      tol = (1.0 + -0.397408316532857 * (r_u_r * r_u_r)) + 0.190419257967814 *
        rt_powd_snf(r_u_r, 4.0);
      for (i8 = 0; i8 < 2; i8++) {
        zn_d_l[i8] /= B;
        zn_d_r[i8] /= tol;
      }

      ml[0] = zn_d_l[0];
      ml[1] = zn_d_l[1];
      ml[2] = 1.0;
      mr[0] = zn_d_r[0];
      mr[1] = zn_d_r[1];
      mr[2] = 1.0;
      B = norm(ml);
      tol = norm(mr);
      for (i8 = 0; i8 < 3; i8++) {
        m[i8] = ml[i8] / B;
        m[3 + i8] = mr[i8] / tol;
      }

      // TRIANGULATIONN Triangulate a point from several measurements
      //    Calculate the world position of a point given measurements of it from 
      //    different positions. The solution minimizes the least squares error
      //
      //  INPUTS:
      //  pos: a 3 x n matrix containing the positions of the cameras
      //  rot: a 4 x n matrix containing the JPL quaternions of the cameras
      //  m: a 3 x n matrix containing the unit vectors pointing towards the
      //      feature in the camera frame
      //  where n is the number of poses
      //
      //  OUTPUTS:
      //  pf: the feature position in world coordinates
      //  condition: the condition number of the least squares problem. High
      //      numbers mean a badly conditiond problem and a bad result
      memset(&A[0], 0, 30U * sizeof(double));
      for (rankR = 0; rankR < 2; rankR++) {
        j = rankR * 3;
        b_rot[0] = ((rot[rankR << 2] * rot[rankR << 2] - rot[1 + (rankR << 2)] *
                     rot[1 + (rankR << 2)]) - rot[2 + (rankR << 2)] * rot[2 +
                    (rankR << 2)]) + rot[3 + (rankR << 2)] * rot[3 + (rankR << 2)];
        b_rot[1] = 2.0 * (rot[rankR << 2] * rot[1 + (rankR << 2)] + rot[2 +
                          (rankR << 2)] * rot[3 + (rankR << 2)]);
        b_rot[2] = 2.0 * (rot[rankR << 2] * rot[2 + (rankR << 2)] - rot[1 +
                          (rankR << 2)] * rot[3 + (rankR << 2)]);
        b_rot[3] = 2.0 * (rot[rankR << 2] * rot[1 + (rankR << 2)] - rot[2 +
                          (rankR << 2)] * rot[3 + (rankR << 2)]);
        b_rot[4] = ((-(rot[rankR << 2] * rot[rankR << 2]) + rot[1 + (rankR << 2)]
                     * rot[1 + (rankR << 2)]) - rot[2 + (rankR << 2)] * rot[2 +
                    (rankR << 2)]) + rot[3 + (rankR << 2)] * rot[3 + (rankR << 2)];
        b_rot[5] = 2.0 * (rot[1 + (rankR << 2)] * rot[2 + (rankR << 2)] +
                          rot[rankR << 2] * rot[3 + (rankR << 2)]);
        b_rot[6] = 2.0 * (rot[rankR << 2] * rot[2 + (rankR << 2)] + rot[1 +
                          (rankR << 2)] * rot[3 + (rankR << 2)]);
        b_rot[7] = 2.0 * (rot[1 + (rankR << 2)] * rot[2 + (rankR << 2)] -
                          rot[rankR << 2] * rot[3 + (rankR << 2)]);
        b_rot[8] = ((-(rot[rankR << 2] * rot[rankR << 2]) - rot[1 + (rankR << 2)]
                     * rot[1 + (rankR << 2)]) + rot[2 + (rankR << 2)] * rot[2 +
                    (rankR << 2)]) + rot[3 + (rankR << 2)] * rot[3 + (rankR << 2)];
        for (i8 = 0; i8 < 3; i8++) {
          A[(i8 + j) + 6 * rankR] = 0.0;
          for (i9 = 0; i9 < 3; i9++) {
            A[(i8 + j) + 6 * rankR] += b_rot[i8 + 3 * i9] * m[i9 + 3 * rankR];
          }
        }

        memset(&R_cw[0], 0, 9U * sizeof(double));
        j = rankR * 3;
        for (i = 0; i < 3; i++) {
          R_cw[i + 3 * i] = 1.0;
          for (i8 = 0; i8 < 3; i8++) {
            A[(i8 + j) + 6 * (2 + i)] = -R_cw[i8 + 3 * i];
          }
        }

        j = rankR * 3;
        for (i8 = 0; i8 < 3; i8++) {
          b_b[i8 + j] = -pos[i8 + 3 * rankR];
        }
      }

      svd(A, unusedExpr);
      b_eml_xgeqp3(A, tau, jpvt);
      rankR = 0;
      tol = 6.0 * fabs(A[0]) * 2.2204460492503131E-16;
      while ((rankR < 5) && (fabs(A[rankR + 6 * rankR]) >= tol)) {
        rankR++;
      }

      for (i = 0; i < 5; i++) {
        x[i] = 0.0;
      }

      for (j = 0; j < 5; j++) {
        if (tau[j] != 0.0) {
          tol = b_b[j];
          for (i = j + 1; i + 1 < 7; i++) {
            tol += A[i + 6 * j] * b_b[i];
          }

          tol *= tau[j];
          if (tol != 0.0) {
            b_b[j] -= tol;
            for (i = j + 1; i + 1 < 7; i++) {
              b_b[i] -= A[i + 6 * j] * tol;
            }
          }
        }
      }

      for (i = 0; i + 1 <= rankR; i++) {
        x[jpvt[i] - 1] = b_b[i];
      }

      for (j = rankR - 1; j + 1 > 0; j--) {
        x[jpvt[j] - 1] /= A[j + 6 * j];
        for (i = 0; i + 1 <= j; i++) {
          x[jpvt[i] - 1] -= x[jpvt[j] - 1] * A[i + 6 * j];
        }
      }

      for (i = 0; i < 3; i++) {
        fp[i] = x[i + 2];
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    for (i = 0; i < 3; i++) {
      fp[i] = rtNaN;
    }
  }
}

//
// File trailer for initializePoint.cpp
//
// [EOF]
//
