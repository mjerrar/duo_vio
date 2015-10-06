//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "any.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const boolean_T x[40]
// Return Type  : boolean_T
//
boolean_T any(const boolean_T x[40])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 40)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T b_any(const boolean_T x[3])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const emxArray_real_T *x
// Return Type  : boolean_T
//
boolean_T c_any(const emxArray_real_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if ((x->data[ix] == 0.0) || rtIsNaN(x->data[ix])) {
      b1 = true;
    } else {
      b1 = false;
    }

    if (!b1) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// File trailer for any.cpp
//
// [EOF]
//
