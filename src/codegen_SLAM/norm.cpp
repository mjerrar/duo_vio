//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "norm.h"
#include "SLAM_emxutil.h"
#include "mrdivide.h"
#include "svd.h"
#include <stdio.h>

// Function Declarations
static void b_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0);
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void c_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0);
static void eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0);
static double eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0);
static void eml_xgesvd(const emxArray_real_T *A, emxArray_real_T *S);

// Function Definitions

//
// Arguments    : int n
//                double a
//                const emxArray_real_T *x
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void b_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * x->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
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
//                double a
//                const emxArray_real_T *x
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void c_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * x->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * y->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
//                const emxArray_real_T *y
//                int iy0
// Return Type  : double
//
static double eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x->data[ix - 1] * y->data[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *S
// Return Type  : void
//
static void eml_xgesvd(const emxArray_real_T *A, emxArray_real_T *S)
{
  emxArray_real_T *b_A;
  int m;
  int ns;
  int n;
  int p;
  int minnp;
  emxArray_real_T *s;
  emxArray_real_T *e;
  emxArray_real_T *work;
  int nrt;
  int nct;
  int q;
  int iter;
  int mm;
  boolean_T apply_transform;
  double ztest0;
  int qs;
  double ztest;
  double snorm;
  boolean_T exitg3;
  boolean_T exitg2;
  double f;
  double b;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  b_emxInit_real_T(&b_A, 2);
  m = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, m, (int)sizeof(double));
  ns = A->size[0] * A->size[1];
  for (m = 0; m < ns; m++) {
    b_A->data[m] = A->data[m];
  }

  n = A->size[0];
  p = A->size[1];
  if (A->size[0] + 1 <= A->size[1]) {
    ns = A->size[0] + 1;
  } else {
    ns = A->size[1];
  }

  if (A->size[0] <= A->size[1]) {
    minnp = A->size[0];
  } else {
    minnp = A->size[1];
  }

  emxInit_real_T(&s, 1);
  m = s->size[0];
  s->size[0] = ns;
  emxEnsureCapacity((emxArray__common *)s, m, (int)sizeof(double));
  for (m = 0; m < ns; m++) {
    s->data[m] = 0.0;
  }

  emxInit_real_T(&e, 1);
  ns = A->size[1];
  m = e->size[0];
  e->size[0] = ns;
  emxEnsureCapacity((emxArray__common *)e, m, (int)sizeof(double));
  for (m = 0; m < ns; m++) {
    e->data[m] = 0.0;
  }

  emxInit_real_T(&work, 1);
  ns = A->size[0];
  m = work->size[0];
  work->size[0] = ns;
  emxEnsureCapacity((emxArray__common *)work, m, (int)sizeof(double));
  for (m = 0; m < ns; m++) {
    work->data[m] = 0.0;
  }

  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    if (A->size[1] < 2) {
      ns = 0;
    } else {
      ns = A->size[1] - 2;
    }

    if (ns <= A->size[0]) {
      nrt = ns;
    } else {
      nrt = A->size[0];
    }

    ns = A->size[0] - 1;
    if (ns <= A->size[1]) {
      nct = ns;
    } else {
      nct = A->size[1];
    }

    if (nct >= nrt) {
      m = nct;
    } else {
      m = nrt;
    }

    for (q = 1; q <= m; q++) {
      iter = q + n * (q - 1);
      mm = n - q;
      apply_transform = false;
      if (q <= nct) {
        ztest0 = eml_xnrm2(mm + 1, b_A, iter);
        if (ztest0 > 0.0) {
          apply_transform = true;
          if (b_A->data[iter - 1] < 0.0) {
            ztest0 = -ztest0;
          }

          s->data[q - 1] = ztest0;
          if (fabs(s->data[q - 1]) >= 1.0020841800044864E-292) {
            ztest0 = 1.0 / s->data[q - 1];
            ns = iter + mm;
            for (qs = iter; qs <= ns; qs++) {
              b_A->data[qs - 1] *= ztest0;
            }
          } else {
            ns = iter + mm;
            for (qs = iter; qs <= ns; qs++) {
              b_A->data[qs - 1] /= s->data[q - 1];
            }
          }

          b_A->data[iter - 1]++;
          s->data[q - 1] = -s->data[q - 1];
        } else {
          s->data[q - 1] = 0.0;
        }
      }

      for (ns = q; ns + 1 <= p; ns++) {
        qs = q + n * ns;
        if (apply_transform) {
          ztest0 = eml_xdotc(mm + 1, b_A, iter, b_A, qs);
          ztest0 = -(ztest0 / b_A->data[(q + b_A->size[0] * (q - 1)) - 1]);
          eml_xaxpy(mm + 1, ztest0, iter, b_A, qs);
        }

        e->data[ns] = b_A->data[qs - 1];
      }

      if (q <= nrt) {
        ns = p - q;
        ztest0 = b_eml_xnrm2(ns, e, q + 1);
        if (ztest0 == 0.0) {
          e->data[q - 1] = 0.0;
        } else {
          if (e->data[q] < 0.0) {
            ztest0 = -ztest0;
          }

          e->data[q - 1] = ztest0;
          ztest0 = e->data[q - 1];
          if (fabs(ztest0) >= 1.0020841800044864E-292) {
            ztest0 = 1.0 / ztest0;
            ns += q;
            for (qs = q; qs + 1 <= ns; qs++) {
              e->data[qs] *= ztest0;
            }
          } else {
            ns += q;
            for (qs = q; qs + 1 <= ns; qs++) {
              e->data[qs] /= ztest0;
            }
          }

          e->data[q]++;
          e->data[q - 1] = -e->data[q - 1];
          if (q + 1 <= n) {
            for (ns = q; ns + 1 <= n; ns++) {
              work->data[ns] = 0.0;
            }

            for (ns = q; ns + 1 <= p; ns++) {
              b_eml_xaxpy(mm, e->data[ns], b_A, (q + n * ns) + 1, work, q + 1);
            }

            for (ns = q; ns + 1 <= p; ns++) {
              c_eml_xaxpy(mm, -e->data[ns] / e->data[q], work, q + 1, b_A, (q +
                n * ns) + 1);
            }
          }
        }
      }
    }

    if (A->size[1] <= A->size[0] + 1) {
      m = A->size[1];
    } else {
      m = A->size[0] + 1;
    }

    if (nct < A->size[1]) {
      s->data[nct] = b_A->data[nct + b_A->size[0] * nct];
    }

    if (A->size[0] < m) {
      s->data[m - 1] = 0.0;
    }

    if (nrt + 1 < m) {
      e->data[nrt] = b_A->data[nrt + b_A->size[0] * (m - 1)];
    }

    e->data[m - 1] = 0.0;
    for (q = 0; q + 1 <= m; q++) {
      if (s->data[q] != 0.0) {
        ztest = fabs(s->data[q]);
        ztest0 = s->data[q] / ztest;
        s->data[q] = ztest;
        if (q + 1 < m) {
          e->data[q] /= ztest0;
        }
      }

      if ((q + 1 < m) && (e->data[q] != 0.0)) {
        ztest = fabs(e->data[q]);
        ztest0 = e->data[q];
        e->data[q] = ztest;
        s->data[q + 1] *= ztest / ztest0;
      }
    }

    mm = m;
    iter = 0;
    snorm = 0.0;
    for (ns = 0; ns + 1 <= m; ns++) {
      ztest0 = fabs(s->data[ns]);
      ztest = fabs(e->data[ns]);
      if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
      } else {
        ztest0 = ztest;
      }

      if ((snorm >= ztest0) || rtIsNaN(ztest0)) {
      } else {
        snorm = ztest0;
      }
    }

    while ((m > 0) && (!(iter >= 75))) {
      q = m - 1;
      exitg3 = false;
      while (!(exitg3 || (q == 0))) {
        ztest0 = fabs(e->data[q - 1]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s->data[q - 1]) + fabs
              (s->data[q]))) || (ztest0 <= 1.0020841800044864E-292) || ((iter >
              20) && (ztest0 <= 2.2204460492503131E-16 * snorm))) {
          e->data[q - 1] = 0.0;
          exitg3 = true;
        } else {
          q--;
        }
      }

      if (q == m - 1) {
        ns = 4;
      } else {
        qs = m;
        ns = m;
        exitg2 = false;
        while ((!exitg2) && (ns >= q)) {
          qs = ns;
          if (ns == q) {
            exitg2 = true;
          } else {
            ztest0 = 0.0;
            if (ns < m) {
              ztest0 = fabs(e->data[ns - 1]);
            }

            if (ns > q + 1) {
              ztest0 += fabs(e->data[ns - 2]);
            }

            ztest = fabs(s->data[ns - 1]);
            if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
                 1.0020841800044864E-292)) {
              s->data[ns - 1] = 0.0;
              exitg2 = true;
            } else {
              ns--;
            }
          }
        }

        if (qs == q) {
          ns = 3;
        } else if (qs == m) {
          ns = 1;
        } else {
          ns = 2;
          q = qs;
        }
      }

      switch (ns) {
       case 1:
        f = e->data[m - 2];
        e->data[m - 2] = 0.0;
        for (qs = m - 3; qs + 2 >= q + 1; qs--) {
          ztest0 = s->data[qs + 1];
          eml_xrotg(&ztest0, &f, &ztest, &b);
          s->data[qs + 1] = ztest0;
          if (qs + 2 > q + 1) {
            f = -b * e->data[qs];
            e->data[qs] *= ztest;
          }
        }
        break;

       case 2:
        f = e->data[q - 1];
        e->data[q - 1] = 0.0;
        while (q + 1 <= m) {
          eml_xrotg(&s->data[q], &f, &ztest, &b);
          f = -b * e->data[q];
          e->data[q] *= ztest;
          q++;
        }
        break;

       case 3:
        varargin_1[0] = fabs(s->data[m - 1]);
        varargin_1[1] = fabs(s->data[m - 2]);
        varargin_1[2] = fabs(e->data[m - 2]);
        varargin_1[3] = fabs(s->data[q]);
        varargin_1[4] = fabs(e->data[q]);
        ns = 1;
        mtmp = varargin_1[0];
        if (rtIsNaN(varargin_1[0])) {
          qs = 2;
          exitg1 = false;
          while ((!exitg1) && (qs < 6)) {
            ns = qs;
            if (!rtIsNaN(varargin_1[qs - 1])) {
              mtmp = varargin_1[qs - 1];
              exitg1 = true;
            } else {
              qs++;
            }
          }
        }

        if (ns < 5) {
          while (ns + 1 < 6) {
            if (varargin_1[ns] > mtmp) {
              mtmp = varargin_1[ns];
            }

            ns++;
          }
        }

        f = s->data[m - 1] / mtmp;
        ztest0 = s->data[m - 2] / mtmp;
        ztest = e->data[m - 2] / mtmp;
        sqds = s->data[q] / mtmp;
        b = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
        ztest0 = f * ztest;
        ztest0 *= ztest0;
        if ((b != 0.0) || (ztest0 != 0.0)) {
          ztest = sqrt(b * b + ztest0);
          if (b < 0.0) {
            ztest = -ztest;
          }

          ztest = ztest0 / (b + ztest);
        } else {
          ztest = 0.0;
        }

        f = (sqds + f) * (sqds - f) + ztest;
        ztest0 = sqds * (e->data[q] / mtmp);
        for (qs = q + 1; qs < m; qs++) {
          eml_xrotg(&f, &ztest0, &ztest, &b);
          if (qs > q + 1) {
            e->data[qs - 2] = f;
          }

          f = ztest * s->data[qs - 1] + b * e->data[qs - 1];
          e->data[qs - 1] = ztest * e->data[qs - 1] - b * s->data[qs - 1];
          ztest0 = b * s->data[qs];
          s->data[qs] *= ztest;
          s->data[qs - 1] = f;
          eml_xrotg(&s->data[qs - 1], &ztest0, &ztest, &b);
          f = ztest * e->data[qs - 1] + b * s->data[qs];
          s->data[qs] = -b * e->data[qs - 1] + ztest * s->data[qs];
          ztest0 = b * e->data[qs];
          e->data[qs] *= ztest;
        }

        e->data[m - 2] = f;
        iter++;
        break;

       default:
        if (s->data[q] < 0.0) {
          s->data[q] = -s->data[q];
        }

        ns = q + 1;
        while ((q + 1 < mm) && (s->data[q] < s->data[ns])) {
          ztest = s->data[q];
          s->data[q] = s->data[ns];
          s->data[ns] = ztest;
          q = ns;
          ns++;
        }

        iter = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&work);
  emxFree_real_T(&e);
  emxFree_real_T(&b_A);
  m = S->size[0];
  S->size[0] = minnp;
  emxEnsureCapacity((emxArray__common *)S, m, (int)sizeof(double));
  for (qs = 0; qs + 1 <= minnp; qs++) {
    S->data[qs] = s->data[qs];
  }

  emxFree_real_T(&s);
}

//
// Arguments    : const double x[4]
// Return Type  : double
//
double b_norm(const double x[4])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = fabs(x[k]);
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
// Arguments    : const emxArray_real_T *x
// Return Type  : double
//
double c_norm(const emxArray_real_T *x)
{
  double y;
  int n;
  double scale;
  int k;
  double absxk;
  double t;
  emxArray_real_T *U;
  if ((x->size[0] == 1) || (x->size[1] == 1)) {
    n = x->size[0] * x->size[1];
    y = 0.0;
    if (n < 1) {
    } else if (n == 1) {
      y = fabs(x->data[0]);
    } else {
      scale = 2.2250738585072014E-308;
      for (k = 1; k <= n; k++) {
        absxk = fabs(x->data[k - 1]);
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
  } else {
    emxInit_real_T(&U, 1);
    eml_xgesvd(x, U);
    y = U->data[0];
    emxFree_real_T(&U);
  }

  return y;
}

//
// Arguments    : const double x[2]
// Return Type  : double
//
double d_norm(const double x[2])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 2; k++) {
    absxk = fabs(x[k]);
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
// Arguments    : const double x[3]
// Return Type  : double
//
double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(x[k]);
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
// File trailer for norm.cpp
//
// [EOF]
//
