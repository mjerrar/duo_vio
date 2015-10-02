//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_sort.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "eml_sort.h"
#include "SLAM_emxutil.h"
#include <ros/console.h>

// Function Declarations
static void b_eml_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx);
static void eml_sort_idx(emxArray_real_T *x, emxArray_int32_T *idx);
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel);

// Function Definitions

//
// Arguments    : emxArray_real_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void b_eml_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx)
{
  int i35;
  emxArray_real_T *vwork;
  int vstride;
  int unnamed_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i35 = x->size[0];
  } else {
    i35 = 1;
  }

  b_emxInit_real_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i35;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(double));
  unnamed_idx_0 = x->size[0];
  vstride = idx->size[0];
  idx->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, vstride, (int)sizeof(int));
  vstride = 1;
  unnamed_idx_0 = 1;
  while (unnamed_idx_0 <= dim - 1) {
    vstride *= x->size[0];
    unnamed_idx_0 = 2;
  }

  j = 0;
  b_emxInit_int32_T(&iidx, 1);
  while (j + 1 <= vstride) {
    for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= i35; unnamed_idx_0++) {
      vwork->data[unnamed_idx_0] = x->data[j + unnamed_idx_0 * vstride];
    }

    eml_sort_idx(vwork, iidx);
    for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= i35; unnamed_idx_0++) {
      x->data[j + unnamed_idx_0 * vstride] = vwork->data[unnamed_idx_0];
      idx->data[j + unnamed_idx_0 * vstride] = iidx->data[unnamed_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void eml_sort_idx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_real_T *b_x;
  int ib;
  int p;
  int b_m;
  int n;
  double x4[4];
  int idx4[4];
  emxArray_real_T *xwork;
  int nNaNs;
  int k;
  int wOffset;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int iwork[256];
  double b_xwork[256];
  int bLen2;
  int nPairs;
  int32_T exitg1;
  b_emxInit_real_T(&b_x, 1);
  ib = x->size[0];
  p = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, p, (int)sizeof(double));
  b_m = x->size[0];
  for (p = 0; p < b_m; p++) {
    b_x->data[p] = x->data[p];
  }

  p = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)idx, p, (int)sizeof(int));
  for (p = 0; p < ib; p++) {
    idx->data[p] = 0;
  }

  n = x->size[0];
  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  b_emxInit_real_T(&xwork, 1);
  ib = x->size[0];
  p = xwork->size[0];
  xwork->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)xwork, p, (int)sizeof(double));
  nNaNs = 1;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaN(b_x->data[k])) {
      idx->data[n - nNaNs] = k + 1;
      xwork->data[n - nNaNs] = b_x->data[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = k + 1;
      x4[ib - 1] = b_x->data[k];
      if (ib == 4) {
        ib = k - nNaNs;
        if (x4[0] <= x4[1]) {
          b_m = 1;
          p = 2;
        } else {
          b_m = 2;
          p = 1;
        }

        if (x4[2] <= x4[3]) {
          wOffset = 3;
          i4 = 4;
        } else {
          wOffset = 4;
          i4 = 3;
        }

        if (x4[b_m - 1] <= x4[wOffset - 1]) {
          if (x4[p - 1] <= x4[wOffset - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)p;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else if (x4[p - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else if (x4[b_m - 1] <= x4[i4 - 1]) {
          if (x4[p - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else {
          perm[0] = (signed char)wOffset;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)b_m;
          perm[3] = (signed char)p;
        }

        idx->data[ib - 2] = idx4[perm[0] - 1];
        idx->data[ib - 1] = idx4[perm[1] - 1];
        idx->data[ib] = idx4[perm[2] - 1];
        idx->data[ib + 1] = idx4[perm[3] - 1];
        b_x->data[ib - 2] = x4[perm[0] - 1];
        b_x->data[ib - 1] = x4[perm[1] - 1];
        b_x->data[ib] = x4[perm[2] - 1];
        b_x->data[ib + 1] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  wOffset = x->size[0] - nNaNs;
  if (ib > 0) {
    for (b_m = 0; b_m < 4; b_m++) {
      perm[b_m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      b_x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  b_m = (nNaNs - 1) >> 1;
  for (k = 1; k <= b_m; k++) {
    p = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[n - k];
    idx->data[n - k] = p;
    b_x->data[wOffset + k] = xwork->data[n - k];
    b_x->data[n - k] = xwork->data[wOffset + k];
  }

  if (((nNaNs - 1) & 1) != 0) {
    b_x->data[(wOffset + b_m) + 1] = xwork->data[(wOffset + b_m) + 1];
  }

  emxFree_real_T(&xwork);
  nNonNaN = (x->size[0] - nNaNs) + 1;
  b_m = 2;
  if (nNonNaN > 1) {
    if (x->size[0] >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        for (wOffset = 1; wOffset <= nBlocks; wOffset++) {
          i4 = ((wOffset - 1) << 8) - 1;
          for (nNaNs = 0; nNaNs < 6; nNaNs++) {
            n = 1 << (nNaNs + 2);
            bLen2 = n << 1;
            nPairs = 256 >> (nNaNs + 3);
            for (k = 1; k <= nPairs; k++) {
              b_m = i4 + (k - 1) * bLen2;
              for (ib = 1; ib <= bLen2; ib++) {
                iwork[ib - 1] = idx->data[b_m + ib];
                b_xwork[ib - 1] = b_x->data[b_m + ib];
              }

              p = 0;
              ib = n;
              do {
                exitg1 = 0;
                b_m++;
                if (b_xwork[p] <= b_xwork[ib]) {
                  idx->data[b_m] = iwork[p];
                  b_x->data[b_m] = b_xwork[p];
                  if (p + 1 < n) {
                    p++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[b_m] = iwork[ib];
                  b_x->data[b_m] = b_xwork[ib];
                  if (ib + 1 < bLen2) {
                    ib++;
                  } else {
                    ib = b_m - p;
                    while (p + 1 <= n) {
                      idx->data[(ib + p) + 1] = iwork[p];
                      b_x->data[(ib + p) + 1] = b_xwork[p];
                      p++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        b_m = nBlocks << 8;
        ib = nNonNaN - b_m;
        if (ib > 0) {
          merge_block(idx, b_x, b_m, ib, 2);
        }

        b_m = 8;
      }
    }

    merge_block(idx, b_x, 0, nNonNaN, b_m);
  }

  p = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, p, (int)sizeof(double));
  b_m = b_x->size[0];
  for (p = 0; p < b_m; p++) {
    x->data[p] = b_x->data[p];
  }

  emxFree_real_T(&b_x);
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int np
//                int nq
// Return Type  : void
//
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq)
{
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  int n;
  int qend;
  int p;
  int iout;
  int32_T exitg1;
  b_emxInit_int32_T(&iwork, 1);
  b_emxInit_real_T(&xwork, 1);
  n = iwork->size[0];
  iwork->size[0] = idx->size[0];
  emxEnsureCapacity((emxArray__common *)iwork, n, (int)sizeof(int));
  qend = x->size[0];
  n = xwork->size[0];
  xwork->size[0] = qend;
  emxEnsureCapacity((emxArray__common *)xwork, n, (int)sizeof(double));
  if (nq == 0) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork->data[qend] = idx->data[offset + qend];
      xwork->data[qend] = x->data[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[n]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[n];
        x->data[iout] = xwork->data[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          while (p + 1 <= np) {
            idx->data[n + p] = iwork->data[p];
            x->data[n + p] = xwork->data[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }

  emxFree_real_T(&xwork);
  emxFree_int32_T(&iwork);
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int n
//                int preSortLevel
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel)
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      merge(idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen);
  }
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
void eml_sort(emxArray_real_T *x, emxArray_int32_T *idx)
{
  int dim;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  b_eml_sort(x, dim, idx);
}

//
// File trailer for eml_sort.cpp
//
// [EOF]
//
