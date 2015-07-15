//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "det.h"
#include "SLAM_emxutil.h"
#include "colon.h"
#include <stdio.h>

// Function Declarations
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int incx, int iy0, int
                      incy);

// Function Definitions

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int incx
//                int iy0
//                int incy
// Return Type  : void
//
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int incx, int iy0, int
                      incy)
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
    ix += incx;
    iy += incy;
  }
}

//
// Arguments    : const emxArray_real_T *x
// Return Type  : double
//
double det(const emxArray_real_T *x)
{
  double y;
  emxArray_real_T *b_x;
  int info;
  int loop_ub;
  int ipiv_size[2];
  int ipiv_data[128];
  boolean_T isodd;
  emxInit_real_T(&b_x, 2);
  info = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, info, (int)sizeof(double));
  loop_ub = x->size[0] * x->size[1];
  for (info = 0; info < loop_ub; info++) {
    b_x->data[info] = x->data[info];
  }

  eml_lapack_xgetrf(x->size[0], x->size[1], b_x, x->size[0], ipiv_data,
                    ipiv_size, &info);
  y = b_x->data[0];
  for (info = 1; info - 1 <= b_x->size[0] - 2; info++) {
    y *= b_x->data[info + b_x->size[0] * info];
  }

  emxFree_real_T(&b_x);
  isodd = false;
  for (info = 0; info <= ipiv_size[1] - 2; info++) {
    if (ipiv_data[info] > 1 + info) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

//
// Arguments    : int m
//                int n
//                emxArray_real_T *A
//                int lda
//                int ipiv_data[]
//                int ipiv_size[2]
//                int *info
// Return Type  : void
//
void eml_lapack_xgetrf(int m, int n, emxArray_real_T *A, int lda, int ipiv_data[],
  int ipiv_size[2], int *info)
{
  int b_m;
  int i16;
  int j;
  int mmj;
  int c;
  int i;
  int ix;
  double smax;
  int jA;
  double s;
  int i17;
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
  if (m - 1 <= n) {
    i16 = m - 1;
  } else {
    i16 = n;
  }

  for (j = 1; j <= i16; j++) {
    mmj = (m - j) + 1;
    c = (j - 1) * (lda + 1);
    if (mmj < 1) {
      i = -1;
    } else {
      i = 0;
      if (mmj > 1) {
        ix = c;
        smax = fabs(A->data[c]);
        for (jA = 1; jA + 1 <= mmj; jA++) {
          ix++;
          s = fabs(A->data[ix]);
          if (s > smax) {
            i = jA;
            smax = s;
          }
        }
      }
    }

    if (A->data[c + i] != 0.0) {
      if (i != 0) {
        ipiv_data[j - 1] = j + i;
        eml_xswap(n, A, j, lda, j + i, lda);
      }

      i17 = c + mmj;
      for (i = c + 1; i + 1 <= i17; i++) {
        A->data[i] /= A->data[c];
      }
    } else {
      *info = j;
    }

    i = n - j;
    jA = c + lda;
    jy = c + lda;
    for (b_j = 1; b_j <= i; b_j++) {
      smax = A->data[jy];
      if (A->data[jy] != 0.0) {
        ix = c + 1;
        i17 = mmj + jA;
        for (ijA = 1 + jA; ijA + 1 <= i17; ijA++) {
          A->data[ijA] += A->data[ix] * -smax;
          ix++;
        }
      }

      jy += lda;
      jA += lda;
    }
  }

  if ((*info == 0) && (m <= n) && (!(A->data[(m + A->size[0] * (m - 1)) - 1] !=
        0.0))) {
    *info = m;
  }
}

//
// File trailer for det.cpp
//
// [EOF]
//
