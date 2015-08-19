//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: blkdiag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "blkdiag.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                emxArray_real_T *y
// Return Type  : void
//
void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
             *varargin_2, emxArray_real_T *y)
{
  int unnamed_idx_0;
  int unnamed_idx_1;
  int i1;
  int loop_ub;
  int i2;
  int i3;
  unnamed_idx_0 = varargin_1->size[0] + varargin_2->size[0];
  unnamed_idx_1 = varargin_1->size[1] + varargin_2->size[1];
  i1 = y->size[0] * y->size[1];
  y->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, i1, (int)sizeof(double));
  i1 = y->size[0] * y->size[1];
  y->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)y, i1, (int)sizeof(double));
  unnamed_idx_0 *= unnamed_idx_1;
  for (i1 = 0; i1 < unnamed_idx_0; i1++) {
    y->data[i1] = 0.0;
  }

  if ((varargin_1->size[0] > 0) && (varargin_1->size[1] > 0)) {
    unnamed_idx_0 = varargin_1->size[1];
    for (i1 = 0; i1 < unnamed_idx_0; i1++) {
      loop_ub = varargin_1->size[0];
      for (unnamed_idx_1 = 0; unnamed_idx_1 < loop_ub; unnamed_idx_1++) {
        y->data[unnamed_idx_1 + y->size[0] * i1] = varargin_1->
          data[unnamed_idx_1 + varargin_1->size[0] * i1];
      }
    }
  }

  if ((varargin_2->size[0] > 0) && (varargin_2->size[1] > 0)) {
    i1 = varargin_1->size[0] + varargin_2->size[0];
    if (varargin_1->size[0] + 1 > i1) {
      i1 = 1;
    } else {
      i1 = varargin_1->size[0] + 1;
    }

    unnamed_idx_1 = varargin_1->size[1] + varargin_2->size[1];
    if (varargin_1->size[1] + 1 > unnamed_idx_1) {
      unnamed_idx_1 = 1;
    } else {
      unnamed_idx_1 = varargin_1->size[1] + 1;
    }

    unnamed_idx_0 = varargin_2->size[1];
    for (i2 = 0; i2 < unnamed_idx_0; i2++) {
      loop_ub = varargin_2->size[0];
      for (i3 = 0; i3 < loop_ub; i3++) {
        y->data[((i1 + i3) + y->size[0] * ((unnamed_idx_1 + i2) - 1)) - 1] =
          varargin_2->data[i3 + varargin_2->size[0] * i2];
      }
    }
  }
}

//
// File trailer for blkdiag.cpp
//
// [EOF]
//
