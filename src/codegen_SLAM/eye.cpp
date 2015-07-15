//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "eye.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : double I[144]
// Return Type  : void
//
void b_eye(double I[144])
{
  int k;
  memset(&I[0], 0, 144U * sizeof(double));
  for (k = 0; k < 12; k++) {
    I[k + 12 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
//                emxArray_real_T *I
// Return Type  : void
//
void eye(double varargin_1, emxArray_real_T *I)
{
  int k;
  int loop_ub;
  k = I->size[0] * I->size[1];
  I->size[0] = (int)varargin_1;
  I->size[1] = (int)varargin_1;
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  loop_ub = (int)varargin_1 * (int)varargin_1;
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)varargin_1 > 0) {
    for (k = 0; k + 1 <= (int)varargin_1; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
