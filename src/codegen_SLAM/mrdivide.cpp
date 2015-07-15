//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "mrdivide.h"
#include "SLAM_emxutil.h"
#include "det.h"
#include "initializePoint.h"
#include "colon.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Declarations
static double b_eml_matlab_zlarfg();
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb);
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static int eml_ixamax(int n, const double x_data[], int ix0);
static void eml_lusolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *X);
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, emxArray_real_T *
  C, int ic0, int ldc, double work_data[]);
static double eml_matlab_zlarfg(int n, double *alpha1, emxArray_real_T *x, int
  ix0);
static void eml_qrsolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *Y);
static void eml_xgeqp3(emxArray_real_T *A, double tau_data[], int tau_size[1],
  int jpvt_data[], int jpvt_size[2]);
static double eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
                      emxArray_real_T *B, int ldb);
static int rankFromQR(const emxArray_real_T *A);

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
static double b_eml_matlab_zlarfg()
{
  return 0.0;
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
//                emxArray_real_T *x
//                int ix0
//                int iy0
// Return Type  : void
//
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x->data[ix];
    x->data[ix] = x->data[iy];
    x->data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  if (B->size[0] == 0) {
  } else {
    for (j = n; j > 0; j--) {
      jBcol = ldb * (j - 1);
      jAcol = lda * (j - 1);
      for (k = j; k + 1 <= n; k++) {
        kBcol = ldb * k;
        if (A->data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A->data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
//                const double x_data[]
//                int ix0
// Return Type  : int
//
static int eml_ixamax(int n, const double x_data[], int ix0)
{
  int idxmax;
  int ix;
  double smax;
  int k;
  double s;
  if (n < 1) {
    idxmax = 0;
  } else {
    idxmax = 1;
    if (n > 1) {
      ix = ix0 - 1;
      smax = fabs(x_data[ix0 - 1]);
      for (k = 2; k <= n; k++) {
        ix++;
        s = fabs(x_data[ix]);
        if (s > smax) {
          idxmax = k;
          smax = s;
        }
      }
    }
  }

  return idxmax;
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *X
// Return Type  : void
//
static void eml_lusolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *X)
{
  emxArray_real_T *b_A;
  int info;
  int jp;
  int ipiv_size[2];
  int ipiv_data[128];
  int xi;
  double temp;
  emxInit_real_T(&b_A, 2);
  info = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, info, (int)sizeof(double));
  jp = A->size[0] * A->size[1];
  for (info = 0; info < jp; info++) {
    b_A->data[info] = A->data[info];
  }

  eml_lapack_xgetrf(A->size[1], A->size[1], b_A, A->size[1], ipiv_data,
                    ipiv_size, &info);
  info = X->size[0] * X->size[1];
  X->size[0] = B->size[0];
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, info, (int)sizeof(double));
  jp = B->size[0] * B->size[1];
  for (info = 0; info < jp; info++) {
    X->data[info] = B->data[info];
  }

  eml_xtrsm(B->size[0], A->size[1], b_A, A->size[1], X, B->size[0]);
  b_eml_xtrsm(B->size[0], A->size[1], b_A, A->size[1], X, B->size[0]);
  info = A->size[1] - 2;
  emxFree_real_T(&b_A);
  while (info + 1 > 0) {
    if (ipiv_data[info] != info + 1) {
      jp = ipiv_data[info] - 1;
      for (xi = 0; xi + 1 <= B->size[0]; xi++) {
        temp = X->data[xi + X->size[0] * info];
        X->data[xi + X->size[0] * info] = X->data[xi + X->size[0] * jp];
        X->data[xi + X->size[0] * jp] = temp;
      }
    }

    info--;
  }
}

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                emxArray_real_T *C
//                int ic0
//                int ldc
//                double work_data[]
// Return Type  : void
//
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, emxArray_real_T *
  C, int ic0, int ldc, double work_data[])
{
  int lastv;
  int i;
  int lastc;
  boolean_T exitg2;
  int ia;
  int32_T exitg1;
  int i19;
  int jy;
  int ix;
  double c;
  int j;
  if (tau != 0.0) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C->data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      i = ic0 + (lastc - 1) * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C->data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
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
      for (i = 1; i <= lastc; i++) {
        work_data[i - 1] = 0.0;
      }

      i = 0;
      i19 = ic0 + ldc * (lastc - 1);
      for (jy = ic0; jy <= i19; jy += ldc) {
        ix = iv0;
        c = 0.0;
        j = (jy + lastv) - 1;
        for (ia = jy; ia <= j; ia++) {
          c += C->data[ia - 1] * C->data[ix - 1];
          ix++;
        }

        work_data[i] += c;
        i++;
      }
    }

    if (-tau == 0.0) {
    } else {
      i = ic0 - 1;
      jy = 0;
      for (j = 1; j <= lastc; j++) {
        if (work_data[jy] != 0.0) {
          c = work_data[jy] * -tau;
          ix = iv0;
          i19 = lastv + i;
          for (ia = i; ia + 1 <= i19; ia++) {
            C->data[ia] += C->data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        i += ldc;
      }
    }
  }
}

//
// Arguments    : int n
//                double *alpha1
//                emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double eml_matlab_zlarfg(int n, double *alpha1, emxArray_real_T *x, int
  ix0)
{
  double tau;
  double xnorm;
  int knt;
  int i18;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i18 = (ix0 + n) - 2;
          for (k = ix0; k <= i18; k++) {
            x->data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotd_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i18 = (ix0 + n) - 2;
        for (k = ix0; k <= i18; k++) {
          x->data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i18 = (ix0 + n) - 2;
        for (k = ix0; k <= i18; k++) {
          x->data[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *Y
// Return Type  : void
//
static void eml_qrsolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *Y)
{
  emxArray_real_T *b_A;
  int i;
  int j;
  emxArray_real_T *b_B;
  int jpvt_size[2];
  int jpvt_data[128];
  int tau_size[1];
  double tau_data[128];
  int rankR;
  int m;
  int nb;
  int mn;
  int k;
  double wj;
  emxInit_real_T(&b_A, 2);
  i = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i, (int)sizeof(double));
  j = A->size[0] * A->size[1];
  for (i = 0; i < j; i++) {
    b_A->data[i] = A->data[i];
  }

  emxInit_real_T(&b_B, 2);
  eml_xgeqp3(b_A, tau_data, tau_size, jpvt_data, jpvt_size);
  rankR = rankFromQR(b_A);
  i = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)b_B, i, (int)sizeof(double));
  j = B->size[0] * B->size[1];
  for (i = 0; i < j; i++) {
    b_B->data[i] = B->data[i];
  }

  m = b_A->size[0];
  nb = B->size[1];
  j = b_A->size[0];
  mn = b_A->size[1];
  if (j <= mn) {
    mn = j;
  }

  j = b_A->size[1];
  k = B->size[1];
  i = Y->size[0] * Y->size[1];
  Y->size[0] = j;
  emxEnsureCapacity((emxArray__common *)Y, i, (int)sizeof(double));
  i = Y->size[0] * Y->size[1];
  Y->size[1] = k;
  emxEnsureCapacity((emxArray__common *)Y, i, (int)sizeof(double));
  j *= k;
  for (i = 0; i < j; i++) {
    Y->data[i] = 0.0;
  }

  for (j = 0; j + 1 <= mn; j++) {
    if (tau_data[j] != 0.0) {
      for (k = 0; k + 1 <= nb; k++) {
        wj = b_B->data[j + b_B->size[0] * k];
        for (i = j + 1; i + 1 <= m; i++) {
          wj += b_A->data[i + b_A->size[0] * j] * b_B->data[i + b_B->size[0] * k];
        }

        wj *= tau_data[j];
        if (wj != 0.0) {
          b_B->data[j + b_B->size[0] * k] -= wj;
          for (i = j + 1; i + 1 <= m; i++) {
            b_B->data[i + b_B->size[0] * k] -= b_A->data[i + b_A->size[0] * j] *
              wj;
          }
        }
      }
    }
  }

  for (k = 0; k + 1 <= nb; k++) {
    for (i = 0; i + 1 <= rankR; i++) {
      Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] = b_B->data[i + b_B->size[0] *
        k];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      Y->data[(jpvt_data[j] + Y->size[0] * k) - 1] /= b_A->data[j + b_A->size[0]
        * j];
      for (i = 0; i + 1 <= j; i++) {
        Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt_data[j] +
          Y->size[0] * k) - 1] * b_A->data[i + b_A->size[0] * j];
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_real_T(&b_A);
}

//
// Arguments    : emxArray_real_T *A
//                double tau_data[]
//                int tau_size[1]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : void
//
static void eml_xgeqp3(emxArray_real_T *A, double tau_data[], int tau_size[1],
  int jpvt_data[], int jpvt_size[2])
{
  int m;
  int n;
  int mn;
  unsigned char unnamed_idx_0;
  int k;
  int itemp;
  double work_data[128];
  double vn1_data[128];
  double vn2_data[128];
  int i;
  int i_i;
  int nmi;
  int mmi;
  double atmp;
  double temp2;
  m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  tau_size[0] = (unsigned char)mn;
  eml_signed_integer_colon(A->size[1], jpvt_data, jpvt_size);
  unnamed_idx_0 = (unsigned char)A->size[1];
  k = unnamed_idx_0;
  for (itemp = 0; itemp < k; itemp++) {
    work_data[itemp] = 0.0;
  }

  k = 1;
  for (itemp = 0; itemp + 1 <= n; itemp++) {
    vn1_data[itemp] = eml_xnrm2(m, A, k);
    vn2_data[itemp] = vn1_data[itemp];
    k += m;
  }

  for (i = 0; i + 1 <= mn; i++) {
    i_i = i + i * m;
    nmi = (n - i) - 1;
    mmi = m - i;
    k = eml_ixamax(1 + nmi, vn1_data, i + 1);
    k = (i + k) - 1;
    if (k + 1 != i + 1) {
      b_eml_xswap(m, A, 1 + m * k, 1 + m * i);
      itemp = jpvt_data[k];
      jpvt_data[k] = jpvt_data[i];
      jpvt_data[i] = itemp;
      vn1_data[k] = vn1_data[i];
      vn2_data[k] = vn2_data[i];
    }

    if (i + 1 < m) {
      atmp = A->data[i_i];
      tau_data[i] = eml_matlab_zlarfg(mmi, &atmp, A, i_i + 2);
    } else {
      atmp = A->data[i_i];
      tau_data[i] = b_eml_matlab_zlarfg();
    }

    A->data[i_i] = atmp;
    if (i + 1 < n) {
      atmp = A->data[i_i];
      A->data[i_i] = 1.0;
      eml_matlab_zlarf(mmi, nmi, i_i + 1, tau_data[i], A, (i + (i + 1) * m) + 1,
                       m, work_data);
      A->data[i_i] = atmp;
    }

    for (itemp = i + 1; itemp + 1 <= n; itemp++) {
      if (vn1_data[itemp] != 0.0) {
        atmp = fabs(A->data[i + A->size[0] * itemp]) / vn1_data[itemp];
        atmp = 1.0 - atmp * atmp;
        if (atmp < 0.0) {
          atmp = 0.0;
        }

        temp2 = vn1_data[itemp] / vn2_data[itemp];
        temp2 = atmp * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          if (i + 1 < m) {
            vn1_data[itemp] = c_eml_xnrm2(mmi - 1, A, (i + m * itemp) + 2);
            vn2_data[itemp] = vn1_data[itemp];
          } else {
            vn1_data[itemp] = 0.0;
            vn2_data[itemp] = 0.0;
          }
        } else {
          vn1_data[itemp] *= sqrt(atmp);
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
                      emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  double temp;
  if (B->size[0] == 0) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      jBcol = ldb * j;
      jAcol = lda * j;
      for (k = 0; k + 1 <= j; k++) {
        kBcol = ldb * k;
        if (A->data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A->data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }

      temp = 1.0 / A->data[j + jAcol];
      for (i = 0; i + 1 <= m; i++) {
        B->data[i + jBcol] *= temp;
      }
    }
  }
}

//
// Arguments    : const emxArray_real_T *A
// Return Type  : int
//
static int rankFromQR(const emxArray_real_T *A)
{
  int r;
  int minmn;
  int maxmn;
  double tol;
  r = 0;
  if (A->size[0] < A->size[1]) {
    minmn = A->size[0];
    maxmn = A->size[1];
  } else {
    minmn = A->size[1];
    maxmn = A->size[0];
  }

  tol = (double)maxmn * fabs(A->data[0]) * 2.2204460492503131E-16;
  while ((r < minmn) && (fabs(A->data[r + A->size[0] * r]) >= tol)) {
    r++;
  }

  return r;
}

//
// Arguments    : double A[4]
//                const double B[16]
// Return Type  : void
//
void b_mrdivide(double A[4], const double B[16])
{
  double b_A[16];
  signed char ipiv[4];
  int i15;
  int j;
  int c;
  int jAcol;
  int ix;
  double temp;
  int jy;
  double s;
  int b_j;
  int ijA;
  memcpy(&b_A[0], &B[0], sizeof(double) << 4);
  for (i15 = 0; i15 < 4; i15++) {
    ipiv[i15] = (signed char)(1 + i15);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    jAcol = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (jy = 2; jy <= 4 - j; jy++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        jAcol = jy - 1;
        temp = s;
      }
    }

    if (b_A[c + jAcol] != 0.0) {
      if (jAcol != 0) {
        ipiv[j] = (signed char)((j + jAcol) + 1);
        ix = j;
        jAcol += j;
        for (jy = 0; jy < 4; jy++) {
          temp = b_A[ix];
          b_A[ix] = b_A[jAcol];
          b_A[jAcol] = temp;
          ix += 4;
          jAcol += 4;
        }
      }

      i15 = (c - j) + 4;
      for (jAcol = c + 1; jAcol + 1 <= i15; jAcol++) {
        b_A[jAcol] /= b_A[c];
      }
    }

    jAcol = c;
    jy = c + 4;
    for (b_j = 1; b_j <= 3 - j; b_j++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i15 = (jAcol - j) + 8;
        for (ijA = 5 + jAcol; ijA + 1 <= i15; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 4;
      jAcol += 4;
    }
  }

  for (j = 0; j < 4; j++) {
    jAcol = j << 2;
    for (jy = 1; jy <= j; jy++) {
      if (b_A[(jy + jAcol) - 1] != 0.0) {
        A[j] -= b_A[(jy + jAcol) - 1] * A[jy - 1];
      }
    }

    A[j] *= 1.0 / b_A[j + jAcol];
  }

  for (j = 3; j > -1; j += -1) {
    jAcol = j << 2;
    for (jy = j + 1; jy + 1 < 5; jy++) {
      if (b_A[jy + jAcol] != 0.0) {
        A[j] -= b_A[jy + jAcol] * A[jy];
      }
    }
  }

  for (jAcol = 2; jAcol > -1; jAcol += -1) {
    if (ipiv[jAcol] != jAcol + 1) {
      temp = A[jAcol];
      A[jAcol] = A[ipiv[jAcol] - 1];
      A[ipiv[jAcol] - 1] = temp;
    }
  }
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *y
// Return Type  : void
//
void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
              emxArray_real_T *y)
{
  unsigned char unnamed_idx_1;
  int i4;
  emxArray_real_T *b_B;
  int loop_ub;
  int A_idx_1;
  int i5;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  emxArray_real_T *r1;
  if (A->size[0] == 0) {
    unnamed_idx_1 = (unsigned char)B->size[0];
    i4 = y->size[0] * y->size[1];
    y->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
    i4 = y->size[0] * y->size[1];
    y->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
  } else if (B->size[0] == B->size[1]) {
    eml_lusolve(B, A, y);
  } else {
    emxInit_real_T(&b_B, 2);
    i4 = b_B->size[0] * b_B->size[1];
    b_B->size[0] = B->size[1];
    b_B->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_B, i4, (int)sizeof(double));
    loop_ub = B->size[0];
    for (i4 = 0; i4 < loop_ub; i4++) {
      A_idx_1 = B->size[1];
      for (i5 = 0; i5 < A_idx_1; i5++) {
        b_B->data[i5 + b_B->size[0] * i4] = B->data[i4 + B->size[0] * i5];
      }
    }

    emxInit_real_T(&b_A, 2);
    emxInit_real_T(&c_A, 2);
    i4 = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[1];
    c_A->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)c_A, i4, (int)sizeof(double));
    loop_ub = A->size[0];
    for (i4 = 0; i4 < loop_ub; i4++) {
      A_idx_1 = A->size[1];
      for (i5 = 0; i5 < A_idx_1; i5++) {
        c_A->data[i5 + c_A->size[0] * i4] = A->data[i4 + A->size[0] * i5];
      }
    }

    loop_ub = A->size[1];
    A_idx_1 = A->size[0];
    i4 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = loop_ub;
    b_A->size[1] = A_idx_1;
    emxEnsureCapacity((emxArray__common *)b_A, i4, (int)sizeof(double));
    for (i4 = 0; i4 < A_idx_1; i4++) {
      for (i5 = 0; i5 < loop_ub; i5++) {
        b_A->data[i5 + b_A->size[0] * i4] = c_A->data[i5 + loop_ub * i4];
      }
    }

    emxFree_real_T(&c_A);
    emxInit_real_T(&r1, 2);
    eml_qrsolve(b_B, b_A, r1);
    i4 = y->size[0] * y->size[1];
    y->size[0] = r1->size[1];
    y->size[1] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)y, i4, (int)sizeof(double));
    loop_ub = r1->size[0];
    emxFree_real_T(&b_A);
    emxFree_real_T(&b_B);
    for (i4 = 0; i4 < loop_ub; i4++) {
      A_idx_1 = r1->size[1];
      for (i5 = 0; i5 < A_idx_1; i5++) {
        y->data[i5 + y->size[0] * i4] = r1->data[i4 + r1->size[0] * i5];
      }
    }

    emxFree_real_T(&r1);
  }
}

//
// File trailer for mrdivide.cpp
//
// [EOF]
//
