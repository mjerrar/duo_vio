//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 19:17:37
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "mrdivide.h"
#include "SLAM_emxutil.h"
#include "colon.h"
#include "initializePoint.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Declarations
static double b_eml_matlab_zlarfg();
static double b_eml_xnrm2(int n, const double x_data[], int ix0);
static void b_eml_xswap(int n, double x_data[], int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const double A_data[], int lda,
  emxArray_real_T *B, int ldb);
static double c_eml_xnrm2(int n, const double x_data[], int ix0);
static int eml_ixamax(int n, const double x_data[], int ix0);
static void eml_lusolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *X);
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, double C_data[],
  int ic0, int ldc, double work_data[]);
static double eml_matlab_zlarfg(int n, double *alpha1, double x_data[], int ix0);
static void eml_qrsolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *Y);
static void eml_xgeqp3(double A_data[], int A_size[2], double tau_data[], int
  tau_size[1], int jpvt_data[], int jpvt_size[2]);
static void eml_xgetrf(int m, int n, double A_data[], int A_size[2], int lda,
  int ipiv_data[], int ipiv_size[2], int *info);
static double eml_xnrm2(int n, const double x_data[], int ix0);
static void eml_xswap(int n, double x_data[], int ix0, int incx, int iy0, int
                      incy);
static void eml_xtrsm(int m, int n, const double A_data[], int lda,
                      emxArray_real_T *B, int ldb);
static int rankFromQR(const double A_data[], const int A_size[2]);

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
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
//                double x_data[]
//                int ix0
//                int iy0
// Return Type  : void
//
static void b_eml_xswap(int n, double x_data[], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x_data[ix];
    x_data[ix] = x_data[iy];
    x_data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void b_eml_xtrsm(int m, int n, const double A_data[], int lda,
  emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = n; j > 0; j--) {
      jBcol = ldb * (j - 1) - 1;
      jAcol = lda * (j - 1) - 1;
      for (k = j + 1; k <= n; k++) {
        kBcol = ldb * (k - 1);
        if (A_data[k + jAcol] != 0.0) {
          for (i = 1; i <= m; i++) {
            B->data[i + jBcol] -= A_data[k + jAcol] * B->data[(i + kBcol) - 1];
          }
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double c_eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const emxArray_real_T *B
//                emxArray_real_T *X
// Return Type  : void
//
static void eml_lusolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *X)
{
  int b_A_size[2];
  int info;
  int jp;
  double b_A_data[1296];
  int ipiv_size[2];
  int ipiv_data[36];
  int xi;
  double temp;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  info = A_size[0] * A_size[1];
  for (jp = 0; jp < info; jp++) {
    b_A_data[jp] = A_data[jp];
  }

  eml_xgetrf(A_size[1], A_size[1], b_A_data, b_A_size, A_size[1], ipiv_data,
             ipiv_size, &info);
  jp = X->size[0] * X->size[1];
  X->size[0] = B->size[0];
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, jp, (int)sizeof(double));
  info = B->size[0] * B->size[1];
  for (jp = 0; jp < info; jp++) {
    X->data[jp] = B->data[jp];
  }

  eml_xtrsm(B->size[0], A_size[1], b_A_data, A_size[1], X, B->size[0]);
  b_eml_xtrsm(B->size[0], A_size[1], b_A_data, A_size[1], X, B->size[0]);
  for (info = A_size[1] - 2; info + 1 > 0; info--) {
    if (ipiv_data[info] != info + 1) {
      jp = ipiv_data[info] - 1;
      for (xi = 0; xi + 1 <= B->size[0]; xi++) {
        temp = X->data[xi + X->size[0] * info];
        X->data[xi + X->size[0] * info] = X->data[xi + X->size[0] * jp];
        X->data[xi + X->size[0] * jp] = temp;
      }
    }
  }
}

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                double C_data[]
//                int ic0
//                int ldc
//                double work_data[]
// Return Type  : void
//
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, double C_data[],
  int ic0, int ldc, double work_data[])
{
  int lastv;
  int i;
  int lastc;
  boolean_T exitg2;
  int ia;
  int32_T exitg1;
  int i22;
  int jy;
  int ix;
  double c;
  int j;
  if (tau != 0.0) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C_data[i - 2] == 0.0)) {
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
          if (C_data[ia - 1] != 0.0) {
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
      i22 = ic0 + ldc * (lastc - 1);
      jy = ic0;
      while ((ldc > 0) && (jy <= i22)) {
        ix = iv0;
        c = 0.0;
        j = (jy + lastv) - 1;
        for (ia = jy; ia <= j; ia++) {
          c += C_data[ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[i] += c;
        i++;
        jy += ldc;
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
          i22 = lastv + i;
          for (ia = i; ia + 1 <= i22; ia++) {
            C_data[ia] += C_data[ix - 1] * c;
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
//                double x_data[]
//                int ix0
// Return Type  : double
//
static double eml_matlab_zlarfg(int n, double *alpha1, double x_data[], int ix0)
{
  double tau;
  double xnorm;
  int knt;
  int i21;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i21 = (ix0 + n) - 2;
          for (k = ix0; k <= i21; k++) {
            x_data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
        xnorm = rt_hypotd_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i21 = (ix0 + n) - 2;
        for (k = ix0; k <= i21; k++) {
          x_data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i21 = (ix0 + n) - 2;
        for (k = ix0; k <= i21; k++) {
          x_data[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const emxArray_real_T *B
//                emxArray_real_T *Y
// Return Type  : void
//
static void eml_qrsolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *Y)
{
  int b_A_size[2];
  int j;
  int k;
  double b_A_data[1296];
  emxArray_real_T *b_B;
  int jpvt_size[2];
  int jpvt_data[36];
  int tau_size[1];
  double tau_data[36];
  int rankR;
  int m;
  int nb;
  int mn;
  double wj;
  int i;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  j = A_size[0] * A_size[1];
  for (k = 0; k < j; k++) {
    b_A_data[k] = A_data[k];
  }

  emxInit_real_T(&b_B, 2);
  eml_xgeqp3(b_A_data, b_A_size, tau_data, tau_size, jpvt_data, jpvt_size);
  rankR = rankFromQR(b_A_data, b_A_size);
  k = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)b_B, k, (int)sizeof(double));
  j = B->size[0] * B->size[1];
  for (k = 0; k < j; k++) {
    b_B->data[k] = B->data[k];
  }

  m = b_A_size[0];
  nb = B->size[1];
  j = b_A_size[0];
  mn = b_A_size[1];
  if (j <= mn) {
    mn = j;
  }

  j = B->size[1];
  k = Y->size[0] * Y->size[1];
  Y->size[0] = b_A_size[1];
  emxEnsureCapacity((emxArray__common *)Y, k, (int)sizeof(double));
  k = Y->size[0] * Y->size[1];
  Y->size[1] = j;
  emxEnsureCapacity((emxArray__common *)Y, k, (int)sizeof(double));
  j *= b_A_size[1];
  for (k = 0; k < j; k++) {
    Y->data[k] = 0.0;
  }

  for (j = 0; j + 1 <= mn; j++) {
    if (tau_data[j] != 0.0) {
      for (k = 0; k + 1 <= nb; k++) {
        wj = b_B->data[j + b_B->size[0] * k];
        for (i = j + 1; i + 1 <= m; i++) {
          wj += b_A_data[i + b_A_size[0] * j] * b_B->data[i + b_B->size[0] * k];
        }

        wj *= tau_data[j];
        if (wj != 0.0) {
          b_B->data[j + b_B->size[0] * k] -= wj;
          for (i = j + 1; i + 1 <= m; i++) {
            b_B->data[i + b_B->size[0] * k] -= b_A_data[i + b_A_size[0] * j] *
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
      Y->data[(jpvt_data[j] + Y->size[0] * k) - 1] /= b_A_data[j + b_A_size[0] *
        j];
      for (i = 0; i + 1 <= j; i++) {
        Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt_data[j] +
          Y->size[0] * k) - 1] * b_A_data[i + b_A_size[0] * j];
      }
    }
  }

  emxFree_real_T(&b_B);
}

//
// Arguments    : double A_data[]
//                int A_size[2]
//                double tau_data[]
//                int tau_size[1]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : void
//
static void eml_xgeqp3(double A_data[], int A_size[2], double tau_data[], int
  tau_size[1], int jpvt_data[], int jpvt_size[2])
{
  int m;
  int n;
  int mn;
  int k;
  int itemp;
  double work_data[36];
  double vn1_data[36];
  double vn2_data[36];
  int i;
  int i_i;
  int nmi;
  int mmi;
  double atmp;
  double temp2;
  m = A_size[0];
  n = A_size[1];
  if (A_size[0] <= A_size[1]) {
    mn = A_size[0];
  } else {
    mn = A_size[1];
  }

  tau_size[0] = (signed char)mn;
  eml_signed_integer_colon(A_size[1], jpvt_data, jpvt_size);
  if ((A_size[0] == 0) || (A_size[1] == 0)) {
  } else {
    k = (signed char)A_size[1];
    for (itemp = 0; itemp < k; itemp++) {
      work_data[itemp] = 0.0;
    }

    k = 1;
    for (itemp = 0; itemp + 1 <= n; itemp++) {
      vn1_data[itemp] = eml_xnrm2(m, A_data, k);
      vn2_data[itemp] = vn1_data[itemp];
      k += m;
    }

    for (i = 1; i <= mn; i++) {
      i_i = (i + (i - 1) * m) - 1;
      nmi = n - i;
      mmi = m - i;
      k = eml_ixamax(1 + nmi, vn1_data, i);
      k = (i + k) - 2;
      if (k + 1 != i) {
        b_eml_xswap(m, A_data, 1 + m * k, 1 + m * (i - 1));
        itemp = jpvt_data[k];
        jpvt_data[k] = jpvt_data[i - 1];
        jpvt_data[i - 1] = itemp;
        vn1_data[k] = vn1_data[i - 1];
        vn2_data[k] = vn2_data[i - 1];
      }

      if (i < m) {
        atmp = A_data[i_i];
        tau_data[i - 1] = eml_matlab_zlarfg(mmi + 1, &atmp, A_data, i_i + 2);
      } else {
        atmp = A_data[i_i];
        tau_data[i - 1] = b_eml_matlab_zlarfg();
      }

      A_data[i_i] = atmp;
      if (i < n) {
        atmp = A_data[i_i];
        A_data[i_i] = 1.0;
        eml_matlab_zlarf(mmi + 1, nmi, i_i + 1, tau_data[i - 1], A_data, i + i *
                         m, m, work_data);
        A_data[i_i] = atmp;
      }

      for (itemp = i; itemp + 1 <= n; itemp++) {
        if (vn1_data[itemp] != 0.0) {
          atmp = fabs(A_data[(i + A_size[0] * itemp) - 1]) / vn1_data[itemp];
          atmp = 1.0 - atmp * atmp;
          if (atmp < 0.0) {
            atmp = 0.0;
          }

          temp2 = vn1_data[itemp] / vn2_data[itemp];
          temp2 = atmp * (temp2 * temp2);
          if (temp2 <= 1.4901161193847656E-8) {
            if (i < m) {
              vn1_data[itemp] = c_eml_xnrm2(mmi, A_data, (i + m * itemp) + 1);
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
}

//
// Arguments    : int m
//                int n
//                double A_data[]
//                int A_size[2]
//                int lda
//                int ipiv_data[]
//                int ipiv_size[2]
//                int *info
// Return Type  : void
//
static void eml_xgetrf(int m, int n, double A_data[], int A_size[2], int lda,
  int ipiv_data[], int ipiv_size[2], int *info)
{
  int b_m;
  int i19;
  int j;
  int mmj;
  int c;
  int i;
  int ix;
  double smax;
  int jA;
  double s;
  int i20;
  int jy;
  int b_j;
  int ijA;
  if (m <= n) {
    b_m = m;
  } else {
    b_m = n;
  }

  eml_signed_integer_colon(b_m, ipiv_data, ipiv_size);
  *info = 0;
  if ((m < 1) || (n < 1)) {
  } else {
    if (m - 1 <= n) {
      i19 = m - 1;
    } else {
      i19 = n;
    }

    for (j = 1; j <= i19; j++) {
      mmj = (m - j) + 1;
      c = (j - 1) * (lda + 1);
      if (mmj < 1) {
        i = -1;
      } else {
        i = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(A_data[c]);
          for (jA = 1; jA + 1 <= mmj; jA++) {
            ix++;
            s = fabs(A_data[ix]);
            if (s > smax) {
              i = jA;
              smax = s;
            }
          }
        }
      }

      if (A_data[c + i] != 0.0) {
        if (i != 0) {
          ipiv_data[j - 1] = j + i;
          eml_xswap(n, A_data, j, lda, j + i, lda);
        }

        i20 = c + mmj;
        for (i = c + 1; i + 1 <= i20; i++) {
          A_data[i] /= A_data[c];
        }
      } else {
        *info = j;
      }

      i = n - j;
      jA = c + lda;
      jy = c + lda;
      for (b_j = 1; b_j <= i; b_j++) {
        smax = A_data[jy];
        if (A_data[jy] != 0.0) {
          ix = c + 1;
          i20 = mmj + jA;
          for (ijA = 1 + jA; ijA + 1 <= i20; ijA++) {
            A_data[ijA] += A_data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jA += lda;
      }
    }

    if ((*info == 0) && (m <= n) && (!(A_data[(m + A_size[0] * (m - 1)) - 1] !=
          0.0))) {
      *info = m;
    }
  }
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
//                double x_data[]
//                int ix0
//                int incx
//                int iy0
//                int incy
// Return Type  : void
//
static void eml_xswap(int n, double x_data[], int ix0, int incx, int iy0, int
                      incy)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x_data[ix];
    x_data[ix] = x_data[iy];
    x_data[iy] = temp;
    ix += incx;
    iy += incy;
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void eml_xtrsm(int m, int n, const double A_data[], int lda,
                      emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  double temp;
  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      jBcol = ldb * j;
      jAcol = lda * j;
      for (k = 0; k + 1 <= j; k++) {
        kBcol = ldb * k;
        if (A_data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A_data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }

      temp = 1.0 / A_data[j + jAcol];
      for (i = 0; i + 1 <= m; i++) {
        B->data[i + jBcol] *= temp;
      }
    }
  }
}

//
// Arguments    : const double A_data[]
//                const int A_size[2]
// Return Type  : int
//
static int rankFromQR(const double A_data[], const int A_size[2])
{
  int r;
  int minmn;
  int maxmn;
  double tol;
  r = 0;
  if (A_size[0] < A_size[1]) {
    minmn = A_size[0];
    maxmn = A_size[1];
  } else {
    minmn = A_size[1];
    maxmn = A_size[0];
  }

  if (minmn > 0) {
    tol = (double)maxmn * fabs(A_data[0]) * 2.2204460492503131E-16;
    while ((r < minmn) && (fabs(A_data[r + A_size[0] * r]) >= tol)) {
      r++;
    }
  }

  return r;
}

//
// Arguments    : const double A[2]
//                const double B[4]
//                double y[2]
// Return Type  : void
//
void b_mrdivide(const double A[2], const double B[4], double y[2])
{
  int r1;
  int r2;
  double a21;
  if (fabs(B[1]) > fabs(B[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = B[r2] / B[r1];
  y[r1] = A[0] / B[r1];
  y[r2] = (A[1] - y[r1] * B[2 + r1]) / (B[2 + r2] - a21 * B[2 + r1]);
  y[r1] -= y[r2] * a21;
}

//
// Arguments    : const emxArray_real_T *A
//                const double B_data[]
//                const int B_size[2]
//                emxArray_real_T *y
// Return Type  : void
//
void mrdivide(const emxArray_real_T *A, const double B_data[], const int B_size
              [2], emxArray_real_T *y)
{
  emxArray_real_T *r1;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  unsigned int unnamed_idx_0;
  int i6;
  int loop_ub;
  double b_B_data[1296];
  int b_B_size[2];
  int A_idx_1;
  int i7;
  emxInit_real_T(&r1, 2);
  emxInit_real_T(&b_A, 2);
  emxInit_real_T(&c_A, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B_size[0] == 0) || (B_size[1] ==
        0))) {
    unnamed_idx_0 = (unsigned int)A->size[0];
    i6 = y->size[0] * y->size[1];
    y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)y, i6, (int)sizeof(double));
    i6 = y->size[0] * y->size[1];
    y->size[1] = B_size[0];
    emxEnsureCapacity((emxArray__common *)y, i6, (int)sizeof(double));
    loop_ub = (int)unnamed_idx_0 * B_size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      y->data[i6] = 0.0;
    }
  } else if (B_size[0] == B_size[1]) {
    eml_lusolve(B_data, B_size, A, y);
  } else {
    b_B_size[0] = B_size[1];
    b_B_size[1] = B_size[0];
    loop_ub = B_size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      A_idx_1 = B_size[1];
      for (i7 = 0; i7 < A_idx_1; i7++) {
        b_B_data[i7 + b_B_size[0] * i6] = B_data[i6 + B_size[0] * i7];
      }
    }

    i6 = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[1];
    c_A->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)c_A, i6, (int)sizeof(double));
    loop_ub = A->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      A_idx_1 = A->size[1];
      for (i7 = 0; i7 < A_idx_1; i7++) {
        c_A->data[i7 + c_A->size[0] * i6] = A->data[i6 + A->size[0] * i7];
      }
    }

    loop_ub = A->size[1];
    A_idx_1 = A->size[0];
    i6 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = loop_ub;
    b_A->size[1] = A_idx_1;
    emxEnsureCapacity((emxArray__common *)b_A, i6, (int)sizeof(double));
    for (i6 = 0; i6 < A_idx_1; i6++) {
      for (i7 = 0; i7 < loop_ub; i7++) {
        b_A->data[i7 + b_A->size[0] * i6] = c_A->data[i7 + loop_ub * i6];
      }
    }

    eml_qrsolve(b_B_data, b_B_size, b_A, r1);
    i6 = y->size[0] * y->size[1];
    y->size[0] = r1->size[1];
    y->size[1] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)y, i6, (int)sizeof(double));
    loop_ub = r1->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      A_idx_1 = r1->size[1];
      for (i7 = 0; i7 < A_idx_1; i7++) {
        y->data[i7 + y->size[0] * i6] = r1->data[i6 + r1->size[0] * i7];
      }
    }
  }

  emxFree_real_T(&c_A);
  emxFree_real_T(&b_A);
  emxFree_real_T(&r1);
}

//
// File trailer for mrdivide.cpp
//
// [EOF]
//
