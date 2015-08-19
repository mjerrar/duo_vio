//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sum.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "sum.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double x[3]
// Return Type  : double
//
double sum(const double x[3])
{
  double y;
  int k;
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  return y;
}

//
// File trailer for sum.cpp
//
// [EOF]
//
