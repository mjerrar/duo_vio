//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 16:03:26
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "diag.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double v[9]
//                double d[81]
// Return Type  : void
//
void b_diag(const double v[9], double d[81])
{
  int j;
  memset(&d[0], 0, 81U * sizeof(double));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
  }
}

//
// Arguments    : const double v[2]
//                double d[4]
// Return Type  : void
//
void diag(const double v[2], double d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = 0.0;
  }

  for (j = 0; j < 2; j++) {
    d[j + (j << 1)] = v[j];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
