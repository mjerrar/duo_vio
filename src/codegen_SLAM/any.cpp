//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "any.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const boolean_T x[9]
//                boolean_T y[3]
// Return Type  : void
//
void any(const boolean_T x[9], boolean_T y[3])
{
  int i2;
  int iy;
  int i;
  int i1;
  boolean_T exitg1;
  for (i2 = 0; i2 < 3; i2++) {
    y[i2] = false;
  }

  i2 = 0;
  iy = -1;
  for (i = 0; i < 3; i++) {
    i1 = i2 + 1;
    i2 += 3;
    iy++;
    exitg1 = false;
    while ((!exitg1) && (i1 <= i2)) {
      if (!!x[i1 - 1]) {
        y[iy] = true;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
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
// Arguments    : boolean_T x
// Return Type  : boolean_T
//
boolean_T d_any(boolean_T x)
{
  return !!x;
}

//
// Arguments    : const emxArray_boolean_T *x
//                emxArray_boolean_T *y
// Return Type  : void
//
void e_any(const emxArray_boolean_T *x, emxArray_boolean_T *y)
{
  unsigned int outsize[2];
  int i2;
  int iy;
  int i;
  int i1;
  boolean_T exitg1;
  boolean_T b0;
  for (i2 = 0; i2 < 2; i2++) {
    outsize[i2] = (unsigned int)x->size[i2];
  }

  i2 = y->size[0] * y->size[1];
  y->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)y, i2, (int)sizeof(boolean_T));
  i2 = y->size[0] * y->size[1];
  y->size[1] = (int)outsize[1];
  emxEnsureCapacity((emxArray__common *)y, i2, (int)sizeof(boolean_T));
  iy = (int)outsize[1];
  for (i2 = 0; i2 < iy; i2++) {
    y->data[i2] = false;
  }

  i2 = 0;
  iy = -1;
  for (i = 1; i <= x->size[1]; i++) {
    i1 = i2 + 1;
    i2 += x->size[0];
    iy++;
    exitg1 = false;
    while ((!exitg1) && (i1 <= i2)) {
      b0 = !x->data[i1 - 1];
      if (!b0) {
        y->data[iy] = true;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
}

//
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
boolean_T f_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[1])) {
    b1 = !x->data[ix - 1];
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
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
boolean_T g_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b2;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
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
// File trailer for any.cpp
//
// [EOF]
//
