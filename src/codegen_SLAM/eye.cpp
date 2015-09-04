//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 16:33:03
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "eye.h"
#include "SLAM_emxutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : double varargin_1
//                emxArray_real_T *I
// Return Type  : void
//
void b_eye(double varargin_1, emxArray_real_T *I)
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
// Arguments    : const double varargin_1[2]
//                emxArray_real_T *I
// Return Type  : void
//
void c_eye(const double varargin_1[2], emxArray_real_T *I)
{
  double minval;
  int k;
  int loop_ub;
  if ((varargin_1[0] <= varargin_1[1]) || rtIsNaN(varargin_1[1])) {
    minval = varargin_1[0];
  } else {
    minval = varargin_1[1];
  }

  k = I->size[0] * I->size[1];
  I->size[0] = (int)varargin_1[0];
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  k = I->size[0] * I->size[1];
  I->size[1] = (int)varargin_1[1];
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  loop_ub = (int)varargin_1[0] * (int)varargin_1[1];
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)minval > 0) {
    for (k = 0; k + 1 <= (int)minval; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

//
// Arguments    : double I[225]
// Return Type  : void
//
void d_eye(double I[225])
{
  int k;
  memset(&I[0], 0, 225U * sizeof(double));
  for (k = 0; k < 15; k++) {
    I[k + 15 * k] = 1.0;
  }
}

//
// Arguments    : double I[9]
// Return Type  : void
//
void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
