//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sum.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 22:44:17
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "sum.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_boolean_T *x
//                double y[24]
// Return Type  : void
//
void sum(const emxArray_boolean_T *x, double y[24])
{
  int iy;
  int ixstart;
  int j;
  int ix;
  double s;
  int k;
  if (x->size[1] == 0) {
    memset(&y[0], 0, 24U * sizeof(double));
  } else {
    iy = -1;
    ixstart = -1;
    for (j = 0; j < 24; j++) {
      ixstart++;
      ix = ixstart;
      s = x->data[ixstart];
      for (k = 2; k <= x->size[1]; k++) {
        ix += 24;
        s += (double)x->data[ix];
      }

      iy++;
      y[iy] = s;
    }
  }
}

//
// File trailer for sum.cpp
//
// [EOF]
//
