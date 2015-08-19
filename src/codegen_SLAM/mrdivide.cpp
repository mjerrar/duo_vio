//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
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
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static double d_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static int eml_ixamax(int n, const emxArray_real_T *x, int ix0);
static void eml_xgeqp3(emxArray_real_T *A, emxArray_real_T *tau,
  emxArray_int32_T *jpvt);
static void eml_xgetrf(int m, int n, emxArray_real_T *A, int lda,
  emxArray_int32_T *ipiv, int *info);
static void eml_xscal(int n, double a, emxArray_real_T *x, int ix0);
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static int rankFromQR(const emxArray_real_T *A);

// Function Definitions

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
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double d_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
//                const emxArray_real_T *x
//                int ix0
// Return Type  : int
//
static int eml_ixamax(int n, const emxArray_real_T *x, int ix0)
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
      smax = fabs(x->data[ix0 - 1]);
      for (k = 2; k <= n; k++) {
        ix++;
        s = fabs(x->data[ix]);
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
// Arguments    : emxArray_real_T *A
//                emxArray_real_T *tau
//                emxArray_int32_T *jpvt
// Return Type  : void
//
static void eml_xgeqp3(emxArray_real_T *A, emxArray_real_T *tau,
  emxArray_int32_T *jpvt)
{
  int m;
  int n;
  int mn;
  int i26;
  emxArray_real_T *work;
  int pvt;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int k;
  int j;
  int i;
  int i_i;
  int nmi;
  int mmi;
  double atmp;
  double d4;
  double xnorm;
  double beta1;
  int i_ip1;
  int lastv;
  boolean_T exitg2;
  int ia;
  int32_T exitg1;
  int ix;
  m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i26 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i26, (int)sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real_T(&work, 1);
    pvt = A->size[1];
    i26 = work->size[0];
    work->size[0] = pvt;
    emxEnsureCapacity((emxArray__common *)work, i26, (int)sizeof(double));
    for (i26 = 0; i26 < pvt; i26++) {
      work->data[i26] = 0.0;
    }

    emxInit_real_T(&vn1, 1);
    emxInit_real_T(&vn2, 1);
    pvt = A->size[1];
    i26 = vn1->size[0];
    vn1->size[0] = pvt;
    emxEnsureCapacity((emxArray__common *)vn1, i26, (int)sizeof(double));
    i26 = vn2->size[0];
    vn2->size[0] = pvt;
    emxEnsureCapacity((emxArray__common *)vn2, i26, (int)sizeof(double));
    k = 1;
    for (j = 0; j + 1 <= n; j++) {
      vn1->data[j] = c_eml_xnrm2(m, A, k);
      vn2->data[j] = vn1->data[j];
      k += m;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i + i * m;
      nmi = (n - i) - 1;
      mmi = (m - i) - 1;
      pvt = eml_ixamax(1 + nmi, vn1, i + 1);
      pvt = (i + pvt) - 1;
      if (pvt + 1 != i + 1) {
        eml_xswap(m, A, 1 + m * pvt, 1 + m * i);
        k = jpvt->data[pvt];
        jpvt->data[pvt] = jpvt->data[i];
        jpvt->data[i] = k;
        vn1->data[pvt] = vn1->data[i];
        vn2->data[pvt] = vn2->data[i];
      }

      if (i + 1 < m) {
        atmp = A->data[i_i];
        d4 = 0.0;
        if (1 + mmi <= 0) {
        } else {
          xnorm = d_eml_xnrm2(mmi, A, i_i + 2);
          if (xnorm != 0.0) {
            beta1 = rt_hypotd_snf(A->data[i_i], xnorm);
            if (A->data[i_i] >= 0.0) {
              beta1 = -beta1;
            }

            if (fabs(beta1) < 1.0020841800044864E-292) {
              pvt = 0;
              do {
                pvt++;
                eml_xscal(mmi, 9.9792015476736E+291, A, i_i + 2);
                beta1 *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while (!(fabs(beta1) >= 1.0020841800044864E-292));

              xnorm = d_eml_xnrm2(mmi, A, i_i + 2);
              beta1 = rt_hypotd_snf(atmp, xnorm);
              if (atmp >= 0.0) {
                beta1 = -beta1;
              }

              d4 = (beta1 - atmp) / beta1;
              eml_xscal(mmi, 1.0 / (atmp - beta1), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                beta1 *= 1.0020841800044864E-292;
              }

              atmp = beta1;
            } else {
              d4 = (beta1 - A->data[i_i]) / beta1;
              eml_xscal(mmi, 1.0 / (A->data[i_i] - beta1), A, i_i + 2);
              atmp = beta1;
            }
          }
        }

        tau->data[i] = d4;
      } else {
        xnorm = A->data[i_i];
        atmp = A->data[i_i];
        A->data[i_i] = xnorm;
        tau->data[i] = 0.0;
      }

      A->data[i_i] = atmp;
      if (i + 1 < n) {
        atmp = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = (i + (i + 1) * m) + 1;
        if (tau->data[i] != 0.0) {
          lastv = mmi;
          pvt = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[pvt] == 0.0)) {
            lastv--;
            pvt--;
          }

          exitg2 = false;
          while ((!exitg2) && (nmi > 0)) {
            pvt = i_ip1 + (nmi - 1) * m;
            ia = pvt;
            do {
              exitg1 = 0;
              if (ia <= pvt + lastv) {
                if (A->data[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          nmi = 0;
        }

        if (lastv + 1 > 0) {
          if (nmi == 0) {
          } else {
            for (pvt = 1; pvt <= nmi; pvt++) {
              work->data[pvt - 1] = 0.0;
            }

            pvt = 0;
            i26 = i_ip1 + m * (nmi - 1);
            k = i_ip1;
            while ((m > 0) && (k <= i26)) {
              ix = i_i;
              xnorm = 0.0;
              j = k + lastv;
              for (ia = k; ia <= j; ia++) {
                xnorm += A->data[ia - 1] * A->data[ix];
                ix++;
              }

              work->data[pvt] += xnorm;
              pvt++;
              k += m;
            }
          }

          if (-tau->data[i] == 0.0) {
          } else {
            pvt = 0;
            for (j = 1; j <= nmi; j++) {
              if (work->data[pvt] != 0.0) {
                xnorm = work->data[pvt] * -tau->data[i];
                ix = i_i;
                i26 = lastv + i_ip1;
                for (k = i_ip1; k <= i26; k++) {
                  A->data[k - 1] += A->data[ix] * xnorm;
                  ix++;
                }
              }

              pvt++;
              i_ip1 += m;
            }
          }
        }

        A->data[i_i] = atmp;
      }

      for (j = i + 1; j + 1 <= n; j++) {
        if (vn1->data[j] != 0.0) {
          xnorm = fabs(A->data[i + A->size[0] * j]) / vn1->data[j];
          xnorm = 1.0 - xnorm * xnorm;
          if (xnorm < 0.0) {
            xnorm = 0.0;
          }

          beta1 = vn1->data[j] / vn2->data[j];
          beta1 = xnorm * (beta1 * beta1);
          if (beta1 <= 1.4901161193847656E-8) {
            if (i + 1 < m) {
              vn1->data[j] = eml_xnrm2(mmi, A, (i + m * j) + 2);
              vn2->data[j] = vn1->data[j];
            } else {
              vn1->data[j] = 0.0;
              vn2->data[j] = 0.0;
            }
          } else {
            vn1->data[j] *= sqrt(xnorm);
          }
        }
      }
    }

    emxFree_real_T(&vn2);
    emxFree_real_T(&vn1);
    emxFree_real_T(&work);
  }
}

//
// Arguments    : int m
//                int n
//                emxArray_real_T *A
//                int lda
//                emxArray_int32_T *ipiv
//                int *info
// Return Type  : void
//
static void eml_xgetrf(int m, int n, emxArray_real_T *A, int lda,
  emxArray_int32_T *ipiv, int *info)
{
  int b_m;
  int b_info;
  int i24;
  int j;
  int mmj;
  int c;
  int iy;
  int ix;
  double smax;
  int jA;
  double s;
  int i25;
  int jy;
  int b_j;
  int ijA;
  if (m <= n) {
    b_m = m;
  } else {
    b_m = n;
  }

  eml_signed_integer_colon(b_m, ipiv);
  b_info = 0;
  if ((m < 1) || (n < 1)) {
  } else {
    if (m - 1 <= n) {
      i24 = m - 1;
    } else {
      i24 = n;
    }

    for (j = 0; j + 1 <= i24; j++) {
      mmj = m - j;
      c = j * (lda + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(A->data[c]);
          for (jA = 1; jA + 1 <= mmj; jA++) {
            ix++;
            s = fabs(A->data[ix]);
            if (s > smax) {
              iy = jA;
              smax = s;
            }
          }
        }
      }

      if (A->data[c + iy] != 0.0) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (jA = 1; jA <= n; jA++) {
            smax = A->data[ix];
            A->data[ix] = A->data[iy];
            A->data[iy] = smax;
            ix += lda;
            iy += lda;
          }
        }

        i25 = c + mmj;
        for (iy = c + 1; iy + 1 <= i25; iy++) {
          A->data[iy] /= A->data[c];
        }
      } else {
        b_info = j + 1;
      }

      iy = (n - j) - 1;
      jA = c + lda;
      jy = c + lda;
      for (b_j = 1; b_j <= iy; b_j++) {
        smax = A->data[jy];
        if (A->data[jy] != 0.0) {
          ix = c + 1;
          i25 = mmj + jA;
          for (ijA = 1 + jA; ijA + 1 <= i25; ijA++) {
            A->data[ijA] += A->data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jA += lda;
      }
    }

    if ((b_info == 0) && (m <= n) && (!(A->data[(m + A->size[0] * (m - 1)) - 1]
          != 0.0))) {
      b_info = m;
    }
  }

  *info = b_info;
}

//
// Arguments    : int n
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
static void eml_xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i27;
  int k;
  i27 = (ix0 + n) - 1;
  for (k = ix0; k <= i27; k++) {
    x->data[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int iy0
// Return Type  : void
//
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0)
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

  if (minmn > 0) {
    tol = (double)maxmn * fabs(A->data[0]) * 2.2204460492503131E-16;
    while ((r < minmn) && (fabs(A->data[r + A->size[0] * r]) >= tol)) {
      r++;
    }
  }

  return r;
}

//
// Arguments    : emxArray_real_T *A
//                const emxArray_real_T *B
// Return Type  : void
//
void b_mrdivide(emxArray_real_T *A, const emxArray_real_T *B)
{
  emxArray_real_T *Y;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_B;
  unsigned int unnamed_idx_1;
  int j;
  int mn;
  int rankR;
  int jAcol;
  int m;
  double wj;
  emxInit_real_T(&Y, 1);
  b_emxInit_real_T(&b_A, 2);
  emxInit_real_T(&tau, 1);
  b_emxInit_int32_T(&jpvt, 2);
  emxInit_real_T(&b_B, 1);
  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
    unnamed_idx_1 = (unsigned int)B->size[0];
    j = A->size[0] * A->size[1];
    A->size[0] = 1;
    A->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)A, j, (int)sizeof(double));
    mn = (int)unnamed_idx_1;
    for (j = 0; j < mn; j++) {
      A->data[A->size[0] * j] = 0.0;
    }
  } else if (B->size[0] == B->size[1]) {
    rankR = B->size[1];
    j = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[0];
    b_A->size[1] = B->size[1];
    emxEnsureCapacity((emxArray__common *)b_A, j, (int)sizeof(double));
    mn = B->size[0] * B->size[1];
    for (j = 0; j < mn; j++) {
      b_A->data[j] = B->data[j];
    }

    eml_xgetrf(B->size[1], B->size[1], b_A, B->size[1], jpvt, &jAcol);
    if (A->size[1] == 0) {
    } else {
      for (j = 0; j + 1 <= rankR; j++) {
        jAcol = rankR * j;
        for (m = 0; m + 1 <= j; m++) {
          if (b_A->data[m + jAcol] != 0.0) {
            A->data[j] -= b_A->data[m + jAcol] * A->data[m];
          }
        }

        wj = b_A->data[j + jAcol];
        A->data[j] *= 1.0 / wj;
      }
    }

    if (A->size[1] == 0) {
    } else {
      for (j = B->size[1]; j > 0; j--) {
        jAcol = rankR * (j - 1);
        for (m = j; m + 1 <= rankR; m++) {
          if (b_A->data[m + jAcol] != 0.0) {
            A->data[j - 1] -= b_A->data[m + jAcol] * A->data[m];
          }
        }
      }
    }

    for (jAcol = B->size[1] - 2; jAcol + 1 > 0; jAcol--) {
      if (jpvt->data[jAcol] != jAcol + 1) {
        wj = A->data[A->size[0] * jAcol];
        A->data[A->size[0] * jAcol] = A->data[A->size[0] * (jpvt->data[jAcol] -
          1)];
        A->data[A->size[0] * (jpvt->data[jAcol] - 1)] = wj;
      }
    }
  } else {
    j = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, j, (int)sizeof(double));
    mn = B->size[0];
    for (j = 0; j < mn; j++) {
      jAcol = B->size[1];
      for (m = 0; m < jAcol; m++) {
        b_A->data[m + b_A->size[0] * j] = B->data[j + B->size[0] * m];
      }
    }

    eml_xgeqp3(b_A, tau, jpvt);
    rankR = rankFromQR(b_A);
    j = b_B->size[0];
    b_B->size[0] = A->size[1];
    emxEnsureCapacity((emxArray__common *)b_B, j, (int)sizeof(double));
    mn = A->size[1];
    for (j = 0; j < mn; j++) {
      b_B->data[j] = A->data[A->size[0] * j];
    }

    m = b_A->size[0];
    jAcol = b_A->size[0];
    mn = b_A->size[1];
    if (jAcol <= mn) {
      mn = jAcol;
    }

    jAcol = b_A->size[1];
    j = Y->size[0];
    Y->size[0] = jAcol;
    emxEnsureCapacity((emxArray__common *)Y, j, (int)sizeof(double));
    for (j = 0; j < jAcol; j++) {
      Y->data[j] = 0.0;
    }

    for (j = 0; j + 1 <= mn; j++) {
      if (tau->data[j] != 0.0) {
        wj = b_B->data[j];
        for (jAcol = j + 1; jAcol + 1 <= m; jAcol++) {
          wj += b_A->data[jAcol + b_A->size[0] * j] * b_B->data[jAcol];
        }

        wj *= tau->data[j];
        if (wj != 0.0) {
          b_B->data[j] -= wj;
          for (jAcol = j + 1; jAcol + 1 <= m; jAcol++) {
            b_B->data[jAcol] -= b_A->data[jAcol + b_A->size[0] * j] * wj;
          }
        }
      }
    }

    for (jAcol = 0; jAcol + 1 <= rankR; jAcol++) {
      Y->data[jpvt->data[jAcol] - 1] = b_B->data[jAcol];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      Y->data[jpvt->data[j] - 1] /= b_A->data[j + b_A->size[0] * j];
      for (jAcol = 0; jAcol + 1 <= j; jAcol++) {
        Y->data[jpvt->data[jAcol] - 1] -= Y->data[jpvt->data[j] - 1] * b_A->
          data[jAcol + b_A->size[0] * j];
      }
    }

    j = A->size[0] * A->size[1];
    A->size[0] = 1;
    A->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)A, j, (int)sizeof(double));
    mn = Y->size[0];
    for (j = 0; j < mn; j++) {
      A->data[A->size[0] * j] = Y->data[j];
    }
  }

  emxFree_real_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
  emxFree_real_T(&Y);
}

//
// Arguments    : emxArray_real_T *A
//                const emxArray_real_T *B
// Return Type  : void
//
void c_mrdivide(emxArray_real_T *A, const emxArray_real_T *B)
{
  emxArray_real_T *Y;
  emxArray_real_T *b_B;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  int jBcol;
  int kBcol;
  int j;
  int rankR;
  int m;
  int nb;
  int k;
  int i;
  double wj;
  int mn;
  b_emxInit_real_T(&Y, 2);
  b_emxInit_real_T(&b_B, 2);
  b_emxInit_real_T(&b_A, 2);
  emxInit_real_T(&tau, 1);
  b_emxInit_int32_T(&jpvt, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    jBcol = A->size[0];
    kBcol = B->size[0];
    j = A->size[0] * A->size[1];
    A->size[0] = jBcol;
    A->size[1] = kBcol;
    emxEnsureCapacity((emxArray__common *)A, j, (int)sizeof(double));
    for (j = 0; j < kBcol; j++) {
      for (rankR = 0; rankR < jBcol; rankR++) {
        A->data[rankR + A->size[0] * j] = 0.0;
      }
    }
  } else if (B->size[0] == B->size[1]) {
    m = B->size[1];
    j = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[0];
    b_A->size[1] = B->size[1];
    emxEnsureCapacity((emxArray__common *)b_A, j, (int)sizeof(double));
    kBcol = B->size[0] * B->size[1];
    for (j = 0; j < kBcol; j++) {
      b_A->data[j] = B->data[j];
    }

    eml_xgetrf(B->size[1], B->size[1], b_A, B->size[1], jpvt, &jBcol);
    nb = A->size[0];
    if ((A->size[0] == 0) || (A->size[1] == 0)) {
    } else {
      for (j = 0; j + 1 <= m; j++) {
        jBcol = nb * j - 1;
        rankR = m * j;
        for (k = 1; k <= j; k++) {
          kBcol = nb * (k - 1);
          if (b_A->data[(k + rankR) - 1] != 0.0) {
            for (i = 1; i <= nb; i++) {
              A->data[i + jBcol] -= b_A->data[(k + rankR) - 1] * A->data[(i +
                kBcol) - 1];
            }
          }
        }

        wj = 1.0 / b_A->data[j + rankR];
        for (i = 1; i <= nb; i++) {
          A->data[i + jBcol] *= wj;
        }
      }
    }

    if ((A->size[0] == 0) || (A->size[1] == 0)) {
    } else {
      for (j = B->size[1]; j > 0; j--) {
        jBcol = nb * (j - 1) - 1;
        rankR = m * (j - 1) - 1;
        for (k = j + 1; k <= m; k++) {
          kBcol = nb * (k - 1);
          if (b_A->data[k + rankR] != 0.0) {
            for (i = 1; i <= nb; i++) {
              A->data[i + jBcol] -= b_A->data[k + rankR] * A->data[(i + kBcol) -
                1];
            }
          }
        }
      }
    }

    for (jBcol = B->size[1] - 2; jBcol + 1 > 0; jBcol--) {
      if (jpvt->data[jBcol] != jBcol + 1) {
        rankR = jpvt->data[jBcol] - 1;
        for (kBcol = 0; kBcol + 1 <= nb; kBcol++) {
          wj = A->data[kBcol + A->size[0] * jBcol];
          A->data[kBcol + A->size[0] * jBcol] = A->data[kBcol + A->size[0] *
            rankR];
          A->data[kBcol + A->size[0] * rankR] = wj;
        }
      }
    }
  } else {
    j = b_B->size[0] * b_B->size[1];
    b_B->size[0] = A->size[1];
    b_B->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)b_B, j, (int)sizeof(double));
    kBcol = A->size[0];
    for (j = 0; j < kBcol; j++) {
      jBcol = A->size[1];
      for (rankR = 0; rankR < jBcol; rankR++) {
        b_B->data[rankR + b_B->size[0] * j] = A->data[j + A->size[0] * rankR];
      }
    }

    j = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, j, (int)sizeof(double));
    kBcol = B->size[0];
    for (j = 0; j < kBcol; j++) {
      jBcol = B->size[1];
      for (rankR = 0; rankR < jBcol; rankR++) {
        b_A->data[rankR + b_A->size[0] * j] = B->data[j + B->size[0] * rankR];
      }
    }

    eml_xgeqp3(b_A, tau, jpvt);
    rankR = rankFromQR(b_A);
    m = b_A->size[0];
    nb = b_B->size[1];
    jBcol = b_A->size[0];
    mn = b_A->size[1];
    if (jBcol <= mn) {
      mn = jBcol;
    }

    jBcol = b_A->size[1];
    kBcol = b_B->size[1];
    j = Y->size[0] * Y->size[1];
    Y->size[0] = jBcol;
    emxEnsureCapacity((emxArray__common *)Y, j, (int)sizeof(double));
    j = Y->size[0] * Y->size[1];
    Y->size[1] = kBcol;
    emxEnsureCapacity((emxArray__common *)Y, j, (int)sizeof(double));
    kBcol *= jBcol;
    for (j = 0; j < kBcol; j++) {
      Y->data[j] = 0.0;
    }

    for (j = 0; j + 1 <= mn; j++) {
      if (tau->data[j] != 0.0) {
        for (k = 0; k + 1 <= nb; k++) {
          wj = b_B->data[j + b_B->size[0] * k];
          for (i = j + 1; i + 1 <= m; i++) {
            wj += b_A->data[i + b_A->size[0] * j] * b_B->data[i + b_B->size[0] *
              k];
          }

          wj *= tau->data[j];
          if (wj != 0.0) {
            b_B->data[j + b_B->size[0] * k] -= wj;
            for (i = j + 1; i + 1 <= m; i++) {
              b_B->data[i + b_B->size[0] * k] -= b_A->data[i + b_A->size[0] * j]
                * wj;
            }
          }
        }
      }
    }

    for (k = 0; k + 1 <= nb; k++) {
      for (i = 0; i + 1 <= rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] = b_B->data[i + b_B->size
          [0] * k];
      }

      for (j = rankR - 1; j + 1 > 0; j--) {
        Y->data[(jpvt->data[j] + Y->size[0] * k) - 1] /= b_A->data[j + b_A->
          size[0] * j];
        for (i = 0; i + 1 <= j; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt->data[j]
            + Y->size[0] * k) - 1] * b_A->data[i + b_A->size[0] * j];
        }
      }
    }

    j = A->size[0] * A->size[1];
    A->size[0] = Y->size[1];
    A->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)A, j, (int)sizeof(double));
    kBcol = Y->size[0];
    for (j = 0; j < kBcol; j++) {
      jBcol = Y->size[1];
      for (rankR = 0; rankR < jBcol; rankR++) {
        A->data[rankR + A->size[0] * j] = Y->data[j + Y->size[0] * rankR];
      }
    }
  }

  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
  emxFree_real_T(&b_B);
  emxFree_real_T(&Y);
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
double eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
// Arguments    : const double A[2]
//                const double B[4]
//                double y[2]
// Return Type  : void
//
void mrdivide(const double A[2], const double B[4], double y[2])
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
// File trailer for mrdivide.cpp
//
// [EOF]
//
