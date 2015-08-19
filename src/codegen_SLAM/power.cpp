//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: power.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 18:46:47
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "power.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double a[2]
//                double y[2]
// Return Type  : void
//
void power(const double a[2], double y[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    y[k] = a[k] * a[k];
  }
}

//
// File trailer for power.cpp
//
// [EOF]
//
