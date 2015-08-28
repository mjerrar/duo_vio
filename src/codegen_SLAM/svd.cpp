//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 21:19:22
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "svd.h"
#include "initializePoint.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static void b_eml_xaxpy(int n, double a, const double x[30], int ix0, double y[6],
  int iy0);
static void c_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[30],
  int iy0);
static double e_eml_xnrm2(int n, const double x[5], int ix0);
static void eml_xaxpy(int n, double a, int ix0, double y[30], int iy0);
static double eml_xdotc(int n, const double x[30], int ix0, const double y[30],
  int iy0);
static void eml_xrotg(double *a, double *b, double *c, double *s);

// Function Definitions

//
// Arguments    : int n
//                double a
//                const double x[30]
//                int ix0
//                double y[6]
//                int iy0
// Return Type  : void
//
static void b_eml_xaxpy(int n, double a, const double x[30], int ix0, double y[6],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[6]
//                int ix0
//                double y[30]
//                int iy0
// Return Type  : void
//
static void c_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[30],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                const double x[5]
//                int ix0
// Return Type  : double
//
static double e_eml_xnrm2(int n, const double x[5], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
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

  return scale * sqrt(y);
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[30]
//                int iy0
// Return Type  : void
//
static void eml_xaxpy(int n, double a, int ix0, double y[30], int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
//                const double y[30]
//                int iy0
// Return Type  : double
//
static double eml_xdotc(int n, const double x[30], int ix0, const double y[30],
  int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

//
// Arguments    : double *a
//                double *b
//                double *c
//                double *s
// Return Type  : void
//
static void eml_xrotg(double *a, double *b, double *c, double *s)
{
  double roe;
  double absa;
  double absb;
  double scale;
  double ads;
  double bds;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    scale = 0.0;
    *b = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }
  }

  *a = scale;
}

//
// Arguments    : const double A[30]
//                double U[5]
// Return Type  : void
//
void svd(const double A[30], double U[5])
{
  double b_A[30];
  double s[5];
  double e[5];
  int kase;
  double work[6];
  int q;
  int qs;
  boolean_T apply_transform;
  double ztest0;
  int ixstart;
  int m;
  double ztest;
  double rt;
  int iter;
  double snorm;
  int32_T exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 30U * sizeof(double));
  for (kase = 0; kase < 5; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
  }

  for (kase = 0; kase < 6; kase++) {
    work[kase] = 0.0;
  }

  for (q = 0; q < 5; q++) {
    qs = q + 6 * q;
    apply_transform = false;
    ztest0 = d_eml_xnrm2(6 - q, b_A, qs + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[qs] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (qs - q) + 6;
        for (ixstart = qs; ixstart + 1 <= kase; ixstart++) {
          b_A[ixstart] *= ztest0;
        }
      } else {
        kase = (qs - q) + 6;
        for (ixstart = qs; ixstart + 1 <= kase; ixstart++) {
          b_A[ixstart] /= s[q];
        }
      }

      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (kase = q + 1; kase + 1 < 6; kase++) {
      ixstart = q + 6 * kase;
      if (apply_transform) {
        eml_xaxpy(6 - q, -(eml_xdotc(6 - q, b_A, qs + 1, b_A, ixstart + 1) /
                           b_A[q + 6 * q]), qs + 1, b_A, ixstart + 1);
      }

      e[kase] = b_A[ixstart];
    }

    if (q + 1 <= 3) {
      ztest0 = e_eml_xnrm2(4 - q, e, q + 2);
      if (ztest0 == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          ztest0 = 1.0 / e[q];
          for (ixstart = q + 1; ixstart + 1 < 6; ixstart++) {
            e[ixstart] *= ztest0;
          }
        } else {
          for (ixstart = q + 1; ixstart + 1 < 6; ixstart++) {
            e[ixstart] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (kase = q + 1; kase + 1 < 7; kase++) {
          work[kase] = 0.0;
        }

        for (kase = q + 1; kase + 1 < 6; kase++) {
          b_eml_xaxpy(5 - q, e[kase], b_A, (q + 6 * kase) + 2, work, q + 2);
        }

        for (kase = q + 1; kase + 1 < 6; kase++) {
          c_eml_xaxpy(5 - q, -e[kase] / e[q + 1], work, q + 2, b_A, (q + 6 *
            kase) + 2);
        }
      }
    }
  }

  m = 3;
  e[3] = b_A[27];
  e[4] = 0.0;
  for (q = 0; q < 5; q++) {
    ztest = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest0 = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 5) {
        ztest = e[q] / ztest0;
      }
    }

    if ((q + 1 < 5) && (ztest != 0.0)) {
      rt = fabs(ztest);
      ztest0 = ztest;
      ztest = rt;
      s[q + 1] *= rt / ztest0;
    }

    e[q] = ztest;
  }

  iter = 0;
  snorm = 0.0;
  for (kase = 0; kase < 5; kase++) {
    ztest0 = fabs(s[kase]);
    ztest = fabs(e[kase]);
    if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaN(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    kase = m;
    do {
      exitg3 = 0;
      q = kase + 1;
      if (kase + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[kase]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[kase]) + fabs(s[kase + 1])))
            || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) && (ztest0 <=
              2.2204460492503131E-16 * snorm))) {
          e[kase] = 0.0;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if (kase + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      ixstart = m + 2;
      exitg2 = false;
      while ((!exitg2) && (ixstart >= kase + 1)) {
        qs = ixstart;
        if (ixstart == kase + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (ixstart < m + 2) {
            ztest0 = fabs(e[ixstart - 1]);
          }

          if (ixstart > kase + 2) {
            ztest0 += fabs(e[ixstart - 2]);
          }

          ztest = fabs(s[ixstart - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[ixstart - 1] = 0.0;
            exitg2 = true;
          } else {
            ixstart--;
          }
        }
      }

      if (qs == kase + 1) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (ixstart = m; ixstart + 1 >= q + 1; ixstart--) {
        ztest0 = s[ixstart];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[ixstart] = ztest0;
        if (ixstart + 1 > q + 1) {
          f = -rt * e[ixstart - 1];
          e[ixstart - 1] *= ztest;
        }
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      while (q + 1 <= m + 2) {
        eml_xrotg(&s[q], &f, &ztest, &rt);
        f = -rt * e[q];
        e[q] *= ztest;
        q++;
      }
      break;

     case 3:
      varargin_1[0] = fabs(s[m + 1]);
      varargin_1[1] = fabs(s[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s[q]);
      varargin_1[4] = fabs(e[q]);
      ixstart = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        kase = 2;
        exitg1 = false;
        while ((!exitg1) && (kase < 6)) {
          ixstart = kase;
          if (!rtIsNaN(varargin_1[kase - 1])) {
            mtmp = varargin_1[kase - 1];
            exitg1 = true;
          } else {
            kase++;
          }
        }
      }

      if (ixstart < 5) {
        while (ixstart + 1 < 6) {
          if (varargin_1[ixstart] > mtmp) {
            mtmp = varargin_1[ixstart];
          }

          ixstart++;
        }
      }

      f = s[m + 1] / mtmp;
      ztest0 = s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0) || (ztest0 != 0.0)) {
        ztest = sqrt(rt * rt + ztest0);
        if (rt < 0.0) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      ztest0 = sqds * (e[q] / mtmp);
      for (ixstart = q + 1; ixstart <= m + 1; ixstart++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (ixstart > q + 1) {
          e[ixstart - 2] = f;
        }

        f = ztest * s[ixstart - 1] + rt * e[ixstart - 1];
        e[ixstart - 1] = ztest * e[ixstart - 1] - rt * s[ixstart - 1];
        ztest0 = rt * s[ixstart];
        s[ixstart] *= ztest;
        s[ixstart - 1] = f;
        eml_xrotg(&s[ixstart - 1], &ztest0, &ztest, &rt);
        f = ztest * e[ixstart - 1] + rt * s[ixstart];
        s[ixstart] = -rt * e[ixstart - 1] + ztest * s[ixstart];
        ztest0 = rt * e[ixstart];
        e[ixstart] *= ztest;
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
      }

      kase = q + 1;
      while ((q + 1 < 5) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ixstart = 0; ixstart < 5; ixstart++) {
    U[ixstart] = s[ixstart];
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
