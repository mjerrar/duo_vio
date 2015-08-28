//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: repmat.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 14:45:19
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "repmat.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_real_T *a
//                double varargin_1
//                emxArray_real_T *b
// Return Type  : void
//
void repmat(const emxArray_real_T *a, double varargin_1, emxArray_real_T *b)
{
  int outsize_idx_0;
  int itilerow;
  int ibcol;
  int k;
  outsize_idx_0 = a->size[0] * (int)varargin_1;
  itilerow = b->size[0];
  b->size[0] = outsize_idx_0;
  emxEnsureCapacity((emxArray__common *)b, itilerow, (int)sizeof(double));
  if (!(outsize_idx_0 == 0)) {
    outsize_idx_0 = a->size[0];
    for (itilerow = 1; itilerow <= (int)varargin_1; itilerow++) {
      ibcol = (itilerow - 1) * outsize_idx_0;
      for (k = 0; k + 1 <= outsize_idx_0; k++) {
        b->data[ibcol + k] = a->data[k];
      }
    }
  }
}

//
// File trailer for repmat.cpp
//
// [EOF]
//
