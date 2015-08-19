//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kron.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "kron.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                const double B[4]
//                emxArray_real_T *K
// Return Type  : void
//
void kron(const emxArray_real_T *A, const double B[4], emxArray_real_T *K)
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = A->size[0] << 1;
  b_j1 = A->size[1] << 1;
  j2 = K->size[0] * K->size[1];
  K->size[0] = kidx;
  K->size[1] = b_j1;
  emxEnsureCapacity((emxArray__common *)K, j2, (int)sizeof(double));
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A->size[1]; b_j1++) {
    for (j2 = 0; j2 < 2; j2++) {
      for (i1 = 1; i1 <= A->size[0]; i1++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          K->data[kidx] = A->data[(i1 + A->size[0] * (b_j1 - 1)) - 1] * B[i2 +
            (j2 << 1)];
        }
      }
    }
  }
}

//
// File trailer for kron.cpp
//
// [EOF]
//
