//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "any.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
boolean_T any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    b0 = !x->data[ix - 1];
    if (!b0) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// Arguments    : const emxArray_boolean_T *x
//                emxArray_boolean_T *y
// Return Type  : void
//
void b_any(const emxArray_boolean_T *x, emxArray_boolean_T *y)
{
  unsigned int outsize[2];
  int vstride;
  int iy;
  int i1;
  int i2;
  int j;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  for (vstride = 0; vstride < 2; vstride++) {
    outsize[vstride] = (unsigned int)x->size[vstride];
  }

  vstride = y->size[0];
  y->size[0] = (int)outsize[0];
  emxEnsureCapacity((emxArray__common *)y, vstride, (int)sizeof(boolean_T));
  iy = (int)outsize[0];
  for (vstride = 0; vstride < iy; vstride++) {
    y->data[vstride] = false;
  }

  vstride = x->size[0];
  iy = -1;
  i1 = 0;
  i2 = (x->size[1] - 1) * x->size[0];
  for (j = 1; j <= vstride; j++) {
    i1++;
    i2++;
    iy++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      b1 = !x->data[ix - 1];
      if (!b1) {
        y->data[iy] = true;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T c_any(const boolean_T x[3])
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
// Arguments    : const boolean_T x[16]
// Return Type  : boolean_T
//
boolean_T d_any(const boolean_T x[16])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
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
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
boolean_T e_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b2;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[1])) {
    b2 = !x->data[ix - 1];
    if (!b2) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T f_any(const boolean_T x[3])
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
// File trailer for any.cpp
//
// [EOF]
//
