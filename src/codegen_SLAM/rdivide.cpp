//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rdivide.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Aug-2015 16:40:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "rdivide.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double x[3]
//                double y
//                double z[3]
// Return Type  : void
//
void rdivide(const double x[3], double y, double z[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

//
// File trailer for rdivide.cpp
//
// [EOF]
//
