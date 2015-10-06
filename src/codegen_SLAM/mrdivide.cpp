//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "mrdivide.h"
#include "SLAM_emxutil.h"
#include "colon.h"
#include "initializePoint.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void eml_lusolve(const emxArray_real_T *A, emxArray_real_T *B);
static double eml_matlab_zlarfg(int n, double *alpha1, emxArray_real_T *x, int
  ix0);
static void eml_xgeqp3(emxArray_real_T *A, emxArray_real_T *tau,
  emxArray_int32_T *jpvt);
static double eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void eml_xscal(int n, double a, emxArray_real_T *x, int ix0);

// Function Definitions

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
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *B
// Return Type  : void
//
static void eml_lusolve(const emxArray_real_T *A, emxArray_real_T *B)
{
  emxArray_real_T *b_A;
  int n;
  int i36;
  int iy;
  emxArray_int32_T *ipiv;
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  int jA;
  int jy;
  int nb;
  emxInit_real_T(&b_A, 2);
  n = A->size[1];
  i36 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i36, (int)sizeof(double));
  iy = A->size[0] * A->size[1];
  for (i36 = 0; i36 < iy; i36++) {
    b_A->data[i36] = A->data[i36];
  }

  emxInit_int32_T(&ipiv, 2);
  iy = A->size[1];
  eml_signed_integer_colon(iy, ipiv);
  if (A->size[1] < 1) {
  } else {
    if (A->size[1] - 1 <= A->size[1]) {
      i36 = A->size[1] - 1;
    } else {
      i36 = A->size[1];
    }

    for (j = 0; j + 1 <= i36; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(b_A->data[c]);
          for (k = 1; k + 1 <= mmj; k++) {
            ix++;
            s = fabs(b_A->data[ix]);
            if (s > smax) {
              iy = k;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[c + iy] != 0.0) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (k = 1; k <= n; k++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        k = c + mmj;
        for (i = c + 1; i + 1 <= k; i++) {
          b_A->data[i] /= b_A->data[c];
        }
      }

      iy = (n - j) - 1;
      jA = c + n;
      jy = c + n;
      for (nb = 1; nb <= iy; nb++) {
        smax = b_A->data[jy];
        if (b_A->data[jy] != 0.0) {
          ix = c + 1;
          k = mmj + jA;
          for (i = 1 + jA; i + 1 <= k; i++) {
            b_A->data[i] += b_A->data[ix] * -smax;
            ix++;
          }
        }

        jy += n;
        jA += n;
      }
    }
  }

  nb = B->size[0];
  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      jA = nb * j;
      jy = n * j;
      for (k = 0; k + 1 <= j; k++) {
        iy = nb * k;
        if (b_A->data[k + jy] != 0.0) {
          for (i = 0; i + 1 <= nb; i++) {
            B->data[i + jA] -= b_A->data[k + jy] * B->data[i + iy];
          }
        }
      }

      smax = 1.0 / b_A->data[j + jy];
      for (i = 0; i + 1 <= nb; i++) {
        B->data[i + jA] *= smax;
      }
    }
  }

  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = A->size[1]; j > 0; j--) {
      jA = nb * (j - 1);
      jy = n * (j - 1);
      for (k = j; k + 1 <= n; k++) {
        iy = nb * k;
        if (b_A->data[k + jy] != 0.0) {
          for (i = 0; i + 1 <= nb; i++) {
            B->data[i + jA] -= b_A->data[k + jy] * B->data[i + iy];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_A);
  for (iy = A->size[1] - 2; iy + 1 > 0; iy--) {
    if (ipiv->data[iy] != iy + 1) {
      jA = ipiv->data[iy] - 1;
      for (jy = 0; jy + 1 <= nb; jy++) {
        smax = B->data[jy + B->size[0] * iy];
        B->data[jy + B->size[0] * iy] = B->data[jy + B->size[0] * jA];
        B->data[jy + B->size[0] * jA] = smax;
      }
    }
  }

  emxFree_int32_T(&ipiv);
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
          eml_xscal(n - 1, 9.9792015476736E+291, x, ix0);
          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotd_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        eml_xscal(n - 1, 1.0 / (*alpha1 - xnorm), x, ix0);
        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        eml_xscal(n - 1, 1.0 / (*alpha1 - xnorm), x, ix0);
        *alpha1 = xnorm;
      }
    }
  }

  return tau;
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
  int b_m;
  int n;
  int mn;
  int i37;
  emxArray_real_T *work;
  int k;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int pvt;
  int ix;
  double smax;
  double s;
  int iy;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int32_T exitg1;
  double absxk;
  double t;
  b_m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i37 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i37, (int)sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    b_emxInit_real_T(&work, 1);
    k = A->size[1];
    i37 = work->size[0];
    work->size[0] = k;
    emxEnsureCapacity((emxArray__common *)work, i37, (int)sizeof(double));
    for (i37 = 0; i37 < k; i37++) {
      work->data[i37] = 0.0;
    }

    b_emxInit_real_T(&vn1, 1);
    b_emxInit_real_T(&vn2, 1);
    k = A->size[1];
    i37 = vn1->size[0];
    vn1->size[0] = k;
    emxEnsureCapacity((emxArray__common *)vn1, i37, (int)sizeof(double));
    i37 = vn2->size[0];
    vn2->size[0] = k;
    emxEnsureCapacity((emxArray__common *)vn2, i37, (int)sizeof(double));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = eml_xnrm2(b_m, A, k);
      vn2->data[nmi] = vn1->data[nmi];
      k += b_m;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i + i * b_m;
      nmi = n - i;
      mmi = (b_m - i) - 1;
      if (nmi < 1) {
        pvt = -1;
      } else {
        pvt = 0;
        if (nmi > 1) {
          ix = i;
          smax = fabs(vn1->data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            ix++;
            s = fabs(vn1->data[ix]);
            if (s > smax) {
              pvt = k + 1;
              smax = s;
            }
          }
        }
      }

      pvt += i;
      if (pvt + 1 != i + 1) {
        ix = b_m * pvt;
        iy = b_m * i;
        for (k = 1; k <= b_m; k++) {
          smax = A->data[ix];
          A->data[ix] = A->data[iy];
          A->data[iy] = smax;
          ix++;
          iy++;
        }

        k = jpvt->data[pvt];
        jpvt->data[pvt] = jpvt->data[i];
        jpvt->data[i] = k;
        vn1->data[pvt] = vn1->data[i];
        vn2->data[pvt] = vn2->data[i];
      }

      if (i + 1 < b_m) {
        s = A->data[i_i];
        tau->data[i] = eml_matlab_zlarfg(1 + mmi, &s, A, i_i + 2);
      } else {
        smax = A->data[i_i];
        s = A->data[i_i];
        A->data[i_i] = smax;
        tau->data[i] = 0.0;
      }

      A->data[i_i] = s;
      if (i + 1 < n) {
        s = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = (i + (i + 1) * b_m) + 1;
        if (tau->data[i] != 0.0) {
          lastv = mmi;
          pvt = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[pvt] == 0.0)) {
            lastv--;
            pvt--;
          }

          lastc = nmi - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            k = i_ip1 + (lastc - 1) * b_m;
            nmi = k;
            do {
              exitg1 = 0;
              if (nmi <= k + lastv) {
                if (A->data[nmi - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  nmi++;
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
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc == 0) {
          } else {
            for (iy = 1; iy <= lastc; iy++) {
              work->data[iy - 1] = 0.0;
            }

            iy = 0;
            i37 = i_ip1 + b_m * (lastc - 1);
            pvt = i_ip1;
            while ((b_m > 0) && (pvt <= i37)) {
              ix = i_i;
              smax = 0.0;
              k = pvt + lastv;
              for (nmi = pvt; nmi <= k; nmi++) {
                smax += A->data[nmi - 1] * A->data[ix];
                ix++;
              }

              work->data[iy] += smax;
              iy++;
              pvt += b_m;
            }
          }

          if (-tau->data[i] == 0.0) {
          } else {
            pvt = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[pvt] != 0.0) {
                smax = work->data[pvt] * -tau->data[i];
                ix = i_i;
                i37 = lastv + i_ip1;
                for (k = i_ip1; k <= i37; k++) {
                  A->data[k - 1] += A->data[ix] * smax;
                  ix++;
                }
              }

              pvt++;
              i_ip1 += b_m;
            }
          }
        }

        A->data[i_i] = s;
      }

      for (nmi = i + 1; nmi + 1 <= n; nmi++) {
        k = (i + b_m * nmi) + 1;
        if (vn1->data[nmi] != 0.0) {
          smax = fabs(A->data[i + A->size[0] * nmi]) / vn1->data[nmi];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 1.4901161193847656E-8) {
            if (i + 1 < b_m) {
              smax = 0.0;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = fabs(A->data[k]);
              } else {
                s = 2.2250738585072014E-308;
                pvt = k + mmi;
                while (k + 1 <= pvt) {
                  absxk = fabs(A->data[k]);
                  if (absxk > s) {
                    t = s / absxk;
                    smax = 1.0 + smax * t * t;
                    s = absxk;
                  } else {
                    t = absxk / s;
                    smax += t * t;
                  }

                  k++;
                }

                smax = s * sqrt(smax);
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0;
              vn2->data[nmi] = 0.0;
            }
          } else {
            vn1->data[nmi] *= sqrt(smax);
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
// Arguments    : int n
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
static void eml_xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i38;
  int k;
  i38 = (ix0 + n) - 1;
  for (k = ix0; k <= i38; k++) {
    x->data[k - 1] *= a;
  }
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
//                const emxArray_real_T *B
//                emxArray_real_T *y
// Return Type  : void
//
void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
              emxArray_real_T *y)
{
  emxArray_real_T *Y;
  emxArray_real_T *b_B;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int i16;
  int i;
  int minmn;
  int maxmn;
  int rankR;
  double tol;
  int b_m;
  int nb;
  int mn;
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_B, 2);
  emxInit_real_T(&b_A, 2);
  b_emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    unnamed_idx_0 = (unsigned int)A->size[0];
    unnamed_idx_1 = (unsigned int)B->size[0];
    i16 = y->size[0] * y->size[1];
    y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
    i16 = y->size[0] * y->size[1];
    y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
    i = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i16 = 0; i16 < i; i16++) {
      y->data[i16] = 0.0;
    }
  } else if (B->size[0] == B->size[1]) {
    i16 = y->size[0] * y->size[1];
    y->size[0] = A->size[0];
    y->size[1] = A->size[1];
    emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
    i = A->size[0] * A->size[1];
    for (i16 = 0; i16 < i; i16++) {
      y->data[i16] = A->data[i16];
    }

    eml_lusolve(B, y);
  } else {
    i16 = b_B->size[0] * b_B->size[1];
    b_B->size[0] = A->size[1];
    b_B->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)b_B, i16, (int)sizeof(double));
    i = A->size[0];
    for (i16 = 0; i16 < i; i16++) {
      minmn = A->size[1];
      for (maxmn = 0; maxmn < minmn; maxmn++) {
        b_B->data[maxmn + b_B->size[0] * i16] = A->data[i16 + A->size[0] * maxmn];
      }
    }

    i16 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, i16, (int)sizeof(double));
    i = B->size[0];
    for (i16 = 0; i16 < i; i16++) {
      minmn = B->size[1];
      for (maxmn = 0; maxmn < minmn; maxmn++) {
        b_A->data[maxmn + b_A->size[0] * i16] = B->data[i16 + B->size[0] * maxmn];
      }
    }

    eml_xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      tol = (double)maxmn * fabs(b_A->data[0]) * 2.2204460492503131E-16;
      while ((rankR < minmn) && (fabs(b_A->data[rankR + b_A->size[0] * rankR]) >=
              tol)) {
        rankR++;
      }
    }

    b_m = b_A->size[0];
    nb = b_B->size[1];
    minmn = b_A->size[0];
    mn = b_A->size[1];
    if (minmn <= mn) {
      mn = minmn;
    }

    minmn = b_A->size[1];
    maxmn = b_B->size[1];
    i16 = Y->size[0] * Y->size[1];
    Y->size[0] = minmn;
    emxEnsureCapacity((emxArray__common *)Y, i16, (int)sizeof(double));
    i16 = Y->size[0] * Y->size[1];
    Y->size[1] = maxmn;
    emxEnsureCapacity((emxArray__common *)Y, i16, (int)sizeof(double));
    i = minmn * maxmn;
    for (i16 = 0; i16 < i; i16++) {
      Y->data[i16] = 0.0;
    }

    for (minmn = 0; minmn + 1 <= mn; minmn++) {
      if (tau->data[minmn] != 0.0) {
        for (maxmn = 0; maxmn + 1 <= nb; maxmn++) {
          tol = b_B->data[minmn + b_B->size[0] * maxmn];
          for (i = minmn + 1; i + 1 <= b_m; i++) {
            tol += b_A->data[i + b_A->size[0] * minmn] * b_B->data[i + b_B->
              size[0] * maxmn];
          }

          tol *= tau->data[minmn];
          if (tol != 0.0) {
            b_B->data[minmn + b_B->size[0] * maxmn] -= tol;
            for (i = minmn + 1; i + 1 <= b_m; i++) {
              b_B->data[i + b_B->size[0] * maxmn] -= b_A->data[i + b_A->size[0] *
                minmn] * tol;
            }
          }
        }
      }
    }

    for (maxmn = 0; maxmn + 1 <= nb; maxmn++) {
      for (i = 0; i + 1 <= rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] = b_B->data[i +
          b_B->size[0] * maxmn];
      }

      for (minmn = rankR - 1; minmn + 1 > 0; minmn--) {
        Y->data[(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] /= b_A->data[minmn
          + b_A->size[0] * minmn];
        for (i = 0; i + 1 <= minmn; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] -= Y->data
            [(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] * b_A->data[i +
            b_A->size[0] * minmn];
        }
      }
    }

    i16 = y->size[0] * y->size[1];
    y->size[0] = Y->size[1];
    y->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
    i = Y->size[0];
    for (i16 = 0; i16 < i; i16++) {
      minmn = Y->size[1];
      for (maxmn = 0; maxmn < minmn; maxmn++) {
        y->data[maxmn + y->size[0] * i16] = Y->data[i16 + Y->size[0] * maxmn];
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
// File trailer for mrdivide.cpp
//
// [EOF]
//
