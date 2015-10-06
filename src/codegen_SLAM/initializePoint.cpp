//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: initializePoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "initializePoint.h"
#include "norm.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static void b_eml_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double c_eml_xnrm2(int n, const double x[30], int ix0);

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
  int i39;
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
  for (i39 = 0; i39 < 5; i39++) {
    jpvt[i39] = 1 + i39;
    work[i39] = 0.0;
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
      smax = vn1[i];
      for (k = 1; k + 1 <= 5 - i; k++) {
        ix++;
        if (vn1[ix] > smax) {
          itemp = k;
          smax = vn1[ix];
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
    smax = c_eml_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0) {
      smax = rt_hypotd_snf(A[i_i], smax);
      if (A[i_i] >= 0.0) {
        smax = -smax;
      }

      if (fabs(smax) < 1.0020841800044864E-292) {
        itemp = 0;
        do {
          itemp++;
          i39 = i_i - i;
          for (k = i_i + 1; k + 1 <= i39 + 6; k++) {
            A[k] *= 9.9792015476736E+291;
          }

          smax *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(fabs(smax) >= 1.0020841800044864E-292));

        smax = rt_hypotd_snf(absxk, c_eml_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0 / (absxk - smax);
        i39 = i_i - i;
        for (k = i_i + 1; k + 1 <= i39 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i39 = i_i - i;
        for (k = i_i + 1; k + 1 <= i39 + 6; k++) {
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
          i39 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i39; itemp += 6) {
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
              i39 = lastv + itemp;
              for (k = itemp; k + 1 <= i39; k++) {
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
      itemp = (i + 6 * iy) + 1;
      if (vn1[iy] != 0.0) {
        smax = fabs(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          smax = 0.0;
          if (5 - i == 1) {
            smax = fabs(A[itemp]);
          } else {
            temp2 = 2.2250738585072014E-308;
            pvt = (itemp - i) + 5;
            while (itemp + 1 <= pvt) {
              absxk = fabs(A[itemp]);
              if (absxk > temp2) {
                t = temp2 / absxk;
                smax = 1.0 + smax * t * t;
                temp2 = absxk;
              } else {
                t = absxk / temp2;
                smax += t * t;
              }

              itemp++;
            }

            smax = temp2 * sqrt(smax);
          }

          vn1[iy] = smax;
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
static double c_eml_xnrm2(int n, const double x[30], int ix0)
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
// INITIALIZEPOINT Initialize a feature point from undistorted measurements
//
//  INPUT ARGUMENTS:
//  - cameraparams: The camera parameter struct
//  - z_l: The undistorted measurement of the feature in the left camera
//  - z_r: The undistorted measurement of the feature in the right camera
//
//  OUTPUT ARGUMENTS:
//  - fp: The feature point in the camera frame
//  - m:  The ray in the left and right camera frame to the feature
// Arguments    : const double z_u_l[2]
//                const double z_u_r[2]
//                const double c_cameraparams_CameraParameters[2]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const double f_cameraparams_CameraParameters[2]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                double fp[3]
//                double b_m[6]
//                boolean_T *success
// Return Type  : void
//
void initializePoint(const double z_u_l[2], const double z_u_r[2], const double
                     c_cameraparams_CameraParameters[2], const double
                     d_cameraparams_CameraParameters[2], const double
                     e_cameraparams_CameraParameters[2], const double
                     f_cameraparams_CameraParameters[2], const double
                     cameraparams_r_lr[3], const double cameraparams_R_lr[9],
                     double fp[3], double b_m[6], boolean_T *success)
{
  double b_pos[6];
  int j;
  int i;
  static const signed char iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double rot[18];
  double ml[3];
  double mr[3];
  double tol;
  double B;
  double A[30];
  double b[6];
  int anchorIdx;
  int b_anchorIdx;
  int rankR;
  double b_rot[9];
  signed char I[9];
  int jpvt[5];
  double tau[5];
  double x[5];
  for (j = 0; j < 3; j++) {
    b_pos[j] = 0.0;
    b_pos[3 + j] = cameraparams_r_lr[j];
    for (i = 0; i < 3; i++) {
      rot[i + 3 * j] = iv1[i + 3 * j];
    }
  }

  ml[0] = (z_u_l[0] - d_cameraparams_CameraParameters[0]) /
    c_cameraparams_CameraParameters[0];
  ml[1] = (z_u_l[1] - d_cameraparams_CameraParameters[1]) /
    c_cameraparams_CameraParameters[1];
  ml[2] = 1.0;
  mr[0] = (z_u_r[0] - f_cameraparams_CameraParameters[0]) /
    e_cameraparams_CameraParameters[0];
  mr[1] = (z_u_r[1] - f_cameraparams_CameraParameters[1]) /
    e_cameraparams_CameraParameters[1];
  mr[2] = 1.0;
  tol = norm(ml);
  B = norm(mr);
  for (j = 0; j < 3; j++) {
    for (i = 0; i < 3; i++) {
      rot[i + 3 * (j + 3)] = cameraparams_R_lr[j + 3 * i];
    }

    b_m[j] = ml[j] / tol;
    b_m[3 + j] = mr[j] / B;
  }

  //  normalized rays in left frame
  for (j = 0; j < 3; j++) {
    ml[j] = 0.0;
    for (i = 0; i < 3; i++) {
      ml[j] += cameraparams_R_lr[j + 3 * i] * b_m[3 + i];
    }
  }

  if (b_m[2] * ml[0] - b_m[0] * ml[2] > 0.0) {
    *success = false;
    for (j = 0; j < 3; j++) {
      fp[j] = b_m[j];
    }
  } else {
    *success = true;

    // triangulatePoint Triangulate a point from several measurements
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
    for (anchorIdx = 0; anchorIdx < 2; anchorIdx++) {
      b_anchorIdx = anchorIdx * 3;
      rankR = anchorIdx * 3;
      for (j = 0; j < 3; j++) {
        for (i = 0; i < 3; i++) {
          b_rot[i + 3 * j] = rot[j + 3 * (i + rankR)];
        }
      }

      for (j = 0; j < 3; j++) {
        A[(j + b_anchorIdx) + 6 * anchorIdx] = 0.0;
        for (i = 0; i < 3; i++) {
          A[(j + b_anchorIdx) + 6 * anchorIdx] += b_rot[j + 3 * i] * b_m[i + 3 *
            anchorIdx];
        }
      }

      for (j = 0; j < 9; j++) {
        I[j] = 0;
      }

      b_anchorIdx = anchorIdx * 3;
      for (rankR = 0; rankR < 3; rankR++) {
        I[rankR + 3 * rankR] = 1;
        for (j = 0; j < 3; j++) {
          A[(j + b_anchorIdx) + 6 * (2 + rankR)] = -(double)I[j + 3 * rankR];
        }
      }

      b_anchorIdx = anchorIdx * 3;
      for (j = 0; j < 3; j++) {
        b[j + b_anchorIdx] = -b_pos[j + 3 * anchorIdx];
      }
    }

    //  condition = cond(A);
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
        tol = b[j];
        for (i = j + 1; i + 1 < 7; i++) {
          tol += A[i + 6 * j] * b[i];
        }

        tol *= tau[j];
        if (tol != 0.0) {
          b[j] -= tol;
          for (i = j + 1; i + 1 < 7; i++) {
            b[i] -= A[i + 6 * j] * tol;
          }
        }
      }
    }

    for (i = 0; i + 1 <= rankR; i++) {
      x[jpvt[i] - 1] = b[i];
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

    //  is this better?
    //  A = [cameraparams.R_rl * ml, -mr];
    //  b = cameraparams.r_lr;
    //  x = A\b;
    //  fp = ml*x(1);
  }
}

//
// File trailer for initializePoint.cpp
//
// [EOF]
//
