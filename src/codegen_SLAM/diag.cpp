//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "diag.h"
#include <ros/console.h>

// Function Definitions

//
// Arguments    : const double v[15]
//                double d[225]
// Return Type  : void
//
void diag(const double v[15], double d[225])
{
  int j;
  memset(&d[0], 0, 225U * sizeof(double));
  for (j = 0; j < 15; j++) {
    d[j + 15 * j] = v[j];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
