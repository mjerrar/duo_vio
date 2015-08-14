//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: initializePoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 14-Aug-2015 15:27:50
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "initializePoint.h"
#include "predictMeasurement_stereo.h"
#include "norm.h"
#include "svd.h"
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
  int i26;
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
  for (i26 = 0; i26 < 5; i26++) {
    jpvt[i26] = 1 + i26;
    work[i26] = 0.0;
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
          i26 = i_i - i;
          for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
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
        i26 = i_i - i;
        for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i26 = i_i - i;
        for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
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
          i26 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i26; itemp += 6) {
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
              i26 = lastv + itemp;
              for (k = itemp; k + 1 <= i26; k++) {
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
//  - m:  The ray in the left camera frame to the feature
// Arguments    : const emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                const double z_l[2]
//                const double z_r[2]
//                double fp[3]
//                double m_out[3]
// Return Type  : void
//
void initializePoint(const emxArray_real_T *b_xt, const double
                     cameraparams_r_lr[3], const double cameraparams_R_lr[9],
                     const double z_l[2], const double z_r[2], double fp[3],
                     double m_out[3])
{
  double pos[6];
  int j;
  double dv11[4];
  double b_cameraparams_R_lr[9];
  int rankR;
  double dv12[4];
  double rot[8];
  double zn_d_l[2];
  double zn_d_r[2];
  double rad_d_l;
  double rad_d_r;
  double r_u_l;
  double r_u_r;
  int i;
  double b_r_u_l[2];
  boolean_T b[2];
  boolean_T y;
  boolean_T exitg3;
  boolean_T guard1 = false;
  double c_r_u_l[2];
  boolean_T exitg2;
  double B;
  double absxk;
  double mr[3];
  double m[6];
  double A[30];
  double b_b[6];
  int anchorIdx;
  double b_rot[9];
  signed char I[9];
  double s[5];
  int exponent;
  double unusedExpr[5];
  int jpvt[5];
  double x[5];
  boolean_T exitg1;
  double c_xt[9];

  //  camera parameters for the left and right camera
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  //  R_lw = R_cw;
  //  R_rw = R_lr'*R_cw;
  //  pos  = [r_cw,r_cw + R_cw*r_lr];
  //  rot  = [QuatFromRotJ(R_lw),QuatFromRotJ(R_rw)];
  for (j = 0; j < 3; j++) {
    pos[j] = 0.0;
    pos[3 + j] = cameraparams_r_lr[j];
  }

  b_QuatFromRotJ(dv11);
  for (j = 0; j < 3; j++) {
    for (rankR = 0; rankR < 3; rankR++) {
      b_cameraparams_R_lr[rankR + 3 * j] = cameraparams_R_lr[j + 3 * rankR];
    }
  }

  QuatFromRotJ(b_cameraparams_R_lr, dv12);
  for (j = 0; j < 4; j++) {
    rot[j] = dv11[j];
    rot[4 + j] = dv12[j];
  }

  zn_d_l[0] = (z_l[0] - 155.972717007495) / 268.155648020127;
  zn_d_l[1] = (z_l[1] - 113.206085625994) / 268.867732741683;
  zn_d_r[0] = (z_r[0] - 167.100031218981) / 268.839577384212;
  zn_d_r[1] = (z_r[1] - 107.901779803044) / 269.510643351885;
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
    // ru=ru-(ru+k1*ru^3+k2*ru^5-rd)/(1+3*k1*ru^2+5*k2*ru^4);
    r_u_l -= ((((r_u_l + -0.414085141240295 * rt_powd_snf(r_u_l, 3.0)) +
                0.236451305145822 * rt_powd_snf(r_u_l, 5.0)) +
               -0.0871296995623235 * rt_powd_snf(r_u_l, 7.0)) - rad_d_l) /
      (((1.0 + -1.2422554237208849 * (r_u_l * r_u_l)) + 1.18225652572911 *
        rt_powd_snf(r_u_l, 4.0)) + -0.60990789693626446 * rt_powd_snf(r_u_l, 6.0));

    // ru=ru-(ru+k1*ru^3+k2*ru^5-rd)/(1+3*k1*ru^2+5*k2*ru^4);
    r_u_r -= ((((r_u_r + -0.410786366925601 * rt_powd_snf(r_u_r, 3.0)) +
                0.222940449996276 * rt_powd_snf(r_u_r, 5.0)) +
               -0.0755554113677893 * rt_powd_snf(r_u_r, 7.0)) - rad_d_r) /
      (((1.0 + -1.2323591007768031 * (r_u_r * r_u_r)) + 1.11470224998138 *
        rt_powd_snf(r_u_r, 4.0)) + -0.5288878795745251 * rt_powd_snf(r_u_r, 6.0));
  }

  b_r_u_l[0] = r_u_l;
  b_r_u_l[1] = r_u_r;
  for (j = 0; j < 2; j++) {
    b[j] = rtIsNaN(b_r_u_l[j]);
  }

  y = false;
  rankR = 0;
  exitg3 = false;
  while ((!exitg3) && (rankR < 2)) {
    if (!!b[rankR]) {
      y = true;
      exitg3 = true;
    } else {
      rankR++;
    }
  }

  guard1 = false;
  if (!y) {
    c_r_u_l[0] = r_u_l;
    c_r_u_l[1] = r_u_r;
    for (j = 0; j < 2; j++) {
      b[j] = rtIsInf(c_r_u_l[j]);
    }

    y = false;
    rankR = 0;
    exitg2 = false;
    while ((!exitg2) && (rankR < 2)) {
      if (!!b[rankR]) {
        y = true;
        exitg2 = true;
      } else {
        rankR++;
      }
    }

    if (!y) {
      B = ((1.0 + -0.414085141240295 * (r_u_l * r_u_l)) + 0.236451305145822 *
           rt_powd_snf(r_u_l, 4.0)) + -0.0871296995623235 * rt_powd_snf(r_u_l,
        6.0);

      // undistort points
      absxk = ((1.0 + -0.410786366925601 * (r_u_r * r_u_r)) + 0.222940449996276 *
               rt_powd_snf(r_u_r, 4.0)) + -0.0755554113677893 * rt_powd_snf
        (r_u_r, 6.0);
      for (j = 0; j < 2; j++) {
        zn_d_l[j] /= B;
        zn_d_r[j] /= absxk;
      }

      m_out[0] = zn_d_l[0];
      m_out[1] = zn_d_l[1];
      m_out[2] = 1.0;
      mr[0] = zn_d_r[0];
      mr[1] = zn_d_r[1];
      mr[2] = 1.0;
      B = norm(m_out);
      absxk = norm(mr);
      for (j = 0; j < 3; j++) {
        m[j] = m_out[j] / B;
        m[3 + j] = mr[j] / absxk;
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
      for (i = 0; i < 2; i++) {
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        anchorIdx = i * 3;
        b_rot[0] = ((rot[i << 2] * rot[i << 2] - rot[1 + (i << 2)] * rot[1 + (i <<
          2)]) - rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3 + (i << 2)] *
          rot[3 + (i << 2)];
        b_rot[1] = 2.0 * (rot[i << 2] * rot[1 + (i << 2)] + rot[2 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[2] = 2.0 * (rot[i << 2] * rot[2 + (i << 2)] - rot[1 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[3] = 2.0 * (rot[i << 2] * rot[1 + (i << 2)] - rot[2 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[4] = ((-(rot[i << 2] * rot[i << 2]) + rot[1 + (i << 2)] * rot[1 +
                     (i << 2)]) - rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3
          + (i << 2)] * rot[3 + (i << 2)];
        b_rot[5] = 2.0 * (rot[1 + (i << 2)] * rot[2 + (i << 2)] + rot[i << 2] *
                          rot[3 + (i << 2)]);
        b_rot[6] = 2.0 * (rot[i << 2] * rot[2 + (i << 2)] + rot[1 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[7] = 2.0 * (rot[1 + (i << 2)] * rot[2 + (i << 2)] - rot[i << 2] *
                          rot[3 + (i << 2)]);
        b_rot[8] = ((-(rot[i << 2] * rot[i << 2]) - rot[1 + (i << 2)] * rot[1 +
                     (i << 2)]) + rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3
          + (i << 2)] * rot[3 + (i << 2)];
        for (j = 0; j < 3; j++) {
          A[(j + anchorIdx) + 6 * i] = 0.0;
          for (rankR = 0; rankR < 3; rankR++) {
            A[(j + anchorIdx) + 6 * i] += b_rot[j + 3 * rankR] * m[rankR + 3 * i];
          }
        }

        for (j = 0; j < 9; j++) {
          I[j] = 0;
        }

        anchorIdx = i * 3;
        for (rankR = 0; rankR < 3; rankR++) {
          I[rankR + 3 * rankR] = 1;
          for (j = 0; j < 3; j++) {
            A[(j + anchorIdx) + 6 * (2 + rankR)] = -(double)I[j + 3 * rankR];
          }
        }

        anchorIdx = i * 3;
        for (j = 0; j < 3; j++) {
          b_b[j + anchorIdx] = -pos[j + 3 * i];
        }
      }

      B = 0.0;
      svd(A, s);
      absxk = fabs(s[0]);
      if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
        if (absxk <= 2.2250738585072014E-308) {
          absxk = 4.94065645841247E-324;
        } else {
          frexp(absxk, &exponent);
          absxk = ldexp(1.0, exponent - 53);
        }
      } else {
        absxk = rtNaN;
      }

      absxk *= 6.0;
      rankR = 0;
      while ((rankR < 5) && (s[rankR] > absxk)) {
        B++;
        rankR++;
      }

      if (B < 4.0) {
        for (i = 0; i < 3; i++) {
          mr[i] = rtNaN;
        }
      } else {
        svd(A, unusedExpr);
        b_eml_xgeqp3(A, s, jpvt);
        rankR = 0;
        absxk = 6.0 * fabs(A[0]) * 2.2204460492503131E-16;
        while ((rankR < 5) && (fabs(A[rankR + 6 * rankR]) >= absxk)) {
          rankR++;
        }

        for (i = 0; i < 5; i++) {
          x[i] = 0.0;
        }

        for (j = 0; j < 5; j++) {
          if (s[j] != 0.0) {
            absxk = b_b[j];
            for (i = j + 1; i + 1 < 7; i++) {
              absxk += A[i + 6 * j] * b_b[i];
            }

            absxk *= s[j];
            if (absxk != 0.0) {
              b_b[j] -= absxk;
              for (i = j + 1; i + 1 < 7; i++) {
                b_b[i] -= A[i + 6 * j] * absxk;
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

        for (i = 0; i < 2; i++) {
          b[i] = (x[i] < 0.0);
        }

        y = false;
        rankR = 0;
        exitg1 = false;
        while ((!exitg1) && (rankR < 2)) {
          if (!!b[rankR]) {
            y = true;
            exitg1 = true;
          } else {
            rankR++;
          }
        }

        if (y) {
          for (i = 0; i < 3; i++) {
            mr[i] = rtNaN;
          }
        } else {
          for (i = 0; i < 3; i++) {
            mr[i] = x[i + 2];
          }
        }
      }

      // transform to world coordinates
      c_xt[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4])
                 - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
      c_xt[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] *
                       b_xt->data[6]);
      c_xt[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] *
                       b_xt->data[6]);
      c_xt[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] *
                       b_xt->data[6]);
      c_xt[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data
                  [4]) - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      c_xt[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] *
                       b_xt->data[6]);
      c_xt[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] *
                       b_xt->data[6]);
      c_xt[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] *
                       b_xt->data[6]);
      c_xt[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data
                  [4]) + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      for (j = 0; j < 3; j++) {
        absxk = 0.0;
        for (rankR = 0; rankR < 3; rankR++) {
          absxk += c_xt[j + 3 * rankR] * mr[rankR];
        }

        fp[j] = absxk + b_xt->data[j];
      }

      B = norm(m_out);
      for (j = 0; j < 3; j++) {
        m_out[j] /= B;
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
      m_out[i] = rtNaN;
    }
  }
}

//
// File trailer for initializePoint.cpp
//
// [EOF]
//
